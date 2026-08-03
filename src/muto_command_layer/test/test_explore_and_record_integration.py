"""Integration tests for the synthetic exploration and recording command."""

import math
import os
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import TransformStamped
from muto_command_layer.action import ExploreAndRecord
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sam2_object_registry.srv import GetStoredObjects
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def wait_future(future, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), 'ROS future did not complete before timeout'
    return future.result()


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert predicate(), 'condition did not become true before timeout'


class FakeProgramBackends(Node):
    def __init__(self):
        super().__init__('fake_explore_and_record_backends')
        self.control_actions = []
        self.navigate_poses = []
        self.spin_angles = []
        self.save_calls = 0
        self.map_save_prefixes = []
        self.map_save_directory = ''
        self.create_service(
            ControlExploration,
            '/control_exploration',
            self.control_callback,
        )
        self.create_service(
            GetStoredObjects,
            '/sam2/get_stored_objects',
            self.registry_callback,
        )
        self.create_service(
            Trigger,
            '/sam2/save_stored_objects',
            self.save_callback,
        )
        self.create_service(
            SaveMap,
            '/test/slam_toolbox/save_map',
            self.map_save_callback,
        )
        self.create_service(
            GetCostmap,
            '/global_costmap/get_costmap',
            self.costmap_callback,
        )
        self.spin_server = ActionServer(
            self,
            Spin,
            '/spin',
            execute_callback=self.execute_spin,
        )
        self.navigate_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self.execute_navigation,
        )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', map_qos)
        self.completion_publisher = self.create_publisher(
            Empty, '/explore/exploration_complete', 1)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_test_environment()

    def publish_test_environment(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_frame'
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 1.0
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

        map_message = OccupancyGrid()
        map_message.header.stamp = self.get_clock().now().to_msg()
        map_message.header.frame_id = 'map'
        map_message.info.resolution = 0.1
        map_message.info.width = 20
        map_message.info.height = 20
        map_message.info.origin.orientation.w = 1.0
        map_message.data = [0] * 400
        for y in range(20):
            for x in range(20):
                if x in (0, 19) or y in (0, 19):
                    map_message.data[y * 20 + x] = 100
        self.map_publisher.publish(map_message)

    def publish_exploration_complete(self):
        self.completion_publisher.publish(Empty())

    def control_callback(self, request, response):
        self.control_actions.append(request.action)
        response.accepted = True
        response.scheduled = False
        if request.action == ControlExploration.Request.ACTION_START:
            response.state = ControlExploration.Request.STATE_RUNNING
            response.message = 'fake exploration started'
        else:
            response.state = ControlExploration.Request.STATE_IDLE
            response.message = 'fake exploration stopped'
        return response

    @staticmethod
    def registry_callback(_request, response):
        response.result.header.frame_id = 'map'
        return response

    def save_callback(self, _request, response):
        self.save_calls += 1
        response.success = True
        response.message = 'fake registry checkpointed'
        return response

    def map_save_callback(self, request, response):
        self.map_save_prefixes.append(request.name.data)
        response.result = SaveMap.Response.RESULT_SUCCESS
        return response

    @staticmethod
    def costmap_callback(_request, response):
        response.map.header.frame_id = 'map'
        response.map.metadata.resolution = 0.1
        response.map.metadata.size_x = 20
        response.map.metadata.size_y = 20
        response.map.metadata.origin.orientation.w = 1.0
        response.map.data = [0] * 400
        return response

    def execute_spin(self, goal_handle):
        self.spin_angles.append(goal_handle.request.target_yaw)
        goal_handle.succeed()
        return Spin.Result()

    def execute_navigation(self, goal_handle):
        self.navigate_poses.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture
def running_command_layer(tmp_path):
    rclpy.init()
    backend = FakeProgramBackends()
    backend.map_save_directory = str(tmp_path)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_command_layer', 'command_layer_node',
            '--ros-args',
            '-p', 'exploration_cycle_duration:=0.1',
            '-p', 'observation_duration:=0.05',
            '-p', 'navigation_settle_time:=0.0',
            '-p', 'program_endpoint_timeout:=1.0',
            '-p', 'spin_time_allowance:=1.0',
            '-p', 'tf_timeout:=1.0',
            '-p', 'visibility_map_timeout:=1.0',
            '-p', 'visibility_robot_clearance:=0.1',
            '-p', 'visibility_candidate_spacing:=5.0',
            '-p', 'visibility_range:=5.0',
            '-p', 'visibility_minimum_new_cells:=1',
            '-p', 'visibility_completion_ratio:=1.0',
            '-p', 'save_map_service:=/test/save_map',
            '-p',
            'slam_toolbox_save_map_service:=/test/slam_toolbox/save_map',
            '-p', f'map_save_directory:={tmp_path}',
            '-p', 'default_map_name:=test_default',
            '-p', 'save_map_timeout:=1.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        action_client = ActionClient(
            backend, ExploreAndRecord, '/explore_and_record')
        assert action_client.wait_for_server(timeout_sec=5.0)
        yield backend, action_client
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5.0)
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        backend.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, output


def test_one_cycle_spins_checkpoints_and_reports_counts(running_command_layer):
    backend, action_client = running_command_layer
    goal = ExploreAndRecord.Goal()
    goal.max_cycles = 1

    goal_handle = wait_future(action_client.send_goal_async(goal))
    assert goal_handle.accepted
    result_response = wait_future(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert result_response.result.success
    assert result_response.result.completed_cycles == 1
    assert result_response.result.objects_before == 0
    assert result_response.result.objects_after == 0
    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
        ControlExploration.Request.ACTION_STOP,
    ]
    assert backend.save_calls == 1
    assert backend.spin_angles == pytest.approx([math.pi / 4.0] * 8)
    assert sum(backend.spin_angles) == pytest.approx(2.0 * math.pi)


def test_save_map_wrapper_confines_names_and_applies_default(
        running_command_layer):
    backend, _ = running_command_layer
    client = backend.create_client(SaveMap, '/test/save_map')
    assert client.wait_for_service(timeout_sec=2.0)

    named_request = SaveMap.Request()
    named_request.name.data = 'warehouse.v2'
    named_response = wait_future(client.call_async(named_request))
    default_response = wait_future(client.call_async(SaveMap.Request()))
    invalid_request = SaveMap.Request()
    invalid_request.name.data = '../escape'
    invalid_response = wait_future(client.call_async(invalid_request))

    assert named_response.result == SaveMap.Response.RESULT_SUCCESS
    assert default_response.result == SaveMap.Response.RESULT_SUCCESS
    assert invalid_response.result == \
        SaveMap.Response.RESULT_UNDEFINED_FAILURE
    assert backend.map_save_prefixes == [
        os.path.join(backend.map_save_directory, 'warehouse.v2'),
        os.path.join(backend.map_save_directory, 'test_default'),
    ]


def test_cancel_stops_exploration_and_rejects_manual_control(
        running_command_layer):
    backend, action_client = running_command_layer
    backend.control_actions.clear()
    backend.spin_angles.clear()
    backend.save_calls = 0
    goal = ExploreAndRecord.Goal()
    goal.exploration_duration = 30.0

    goal_handle = wait_future(action_client.send_goal_async(goal))
    assert goal_handle.accepted
    wait_until(lambda: backend.control_actions == [
        ControlExploration.Request.ACTION_START,
    ])

    explore_client = backend.create_client(SetBool, '/explore')
    assert explore_client.wait_for_service(timeout_sec=2.0)
    request = SetBool.Request()
    request.data = False
    response = wait_future(explore_client.call_async(request))
    assert not response.success
    assert 'cancel that action first' in response.message

    wait_future(goal_handle.cancel_goal_async())
    result_response = wait_future(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_CANCELED
    assert not result_response.result.success
    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
        ControlExploration.Request.ACTION_STOP,
    ]
    assert backend.spin_angles == []
    assert backend.save_calls == 1


def test_frontier_completion_runs_visibility_navigation_and_scan(
        running_command_layer):
    backend, action_client = running_command_layer
    goal = ExploreAndRecord.Goal()
    goal.exploration_duration = 30.0

    goal_handle = wait_future(action_client.send_goal_async(goal))
    assert goal_handle.accepted
    wait_until(lambda: backend.control_actions == [
        ControlExploration.Request.ACTION_START,
    ])
    backend.publish_exploration_complete()
    result_response = wait_future(goal_handle.get_result_async(), timeout=10.0)

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert result_response.result.success
    assert 'visibility coverage completed' in result_response.result.message
    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
        ControlExploration.Request.ACTION_STOP,
    ]
    assert len(backend.navigate_poses) == 1
    target = backend.navigate_poses[0]
    assert target.header.frame_id == 'map'
    assert 0.0 < target.pose.position.x < 2.0
    assert 0.0 < target.pose.position.y < 2.0
    assert backend.spin_angles == pytest.approx([math.pi / 4.0] * 8)
    assert backend.save_calls == 2
