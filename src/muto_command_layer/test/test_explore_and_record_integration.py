"""Integration tests for the synthetic exploration and recording command."""

import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
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
from rclpy.qos import qos_profile_action_status_default
from sam2_object_registry.msg import DetectedObjectArray
from sam2_object_registry.srv import GetStoredObjects
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Empty, Header, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


_TEST_DOMAIN_IDS = itertools.count(200)


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


def wait_for_single_finalized_bag(output_directory, timeout=5.0):
    output_directory = Path(output_directory)

    def finalized_metadata_paths():
        paths = list(output_directory.glob('muto_explore_*/metadata.yaml'))
        return [
            path for path in paths
            if 'muto_schema: explore_and_record_v1' in
            path.read_text(encoding='utf-8')
        ]

    wait_until(
        lambda: len(finalized_metadata_paths()) == 1,
        timeout=timeout,
    )
    return finalized_metadata_paths()[0]


class FakeProgramBackends(Node):
    def __init__(self):
        super().__init__('fake_explore_and_record_backends')
        self.control_actions = []
        self.navigate_poses = []
        self.spin_angles = []
        self.publish_detections_after_spin = True
        self.stop_responses_before_idle = 0
        self.save_calls = 0
        self.map_save_prefixes = []
        self.map_save_directory = ''
        self.bag_output_directory = ''
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
        self.detection_heartbeat_publisher = self.create_publisher(
            Header, '/sam2/detection_heartbeat', 10)
        self.detections_publisher = self.create_publisher(
            DetectedObjectArray, '/sam2/detections', 10)
        self.navigation_status_publisher = self.create_publisher(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            qos_profile_action_status_default,
        )
        self.operator_event_publisher = self.create_publisher(
            String, '/explore_and_record/operator_event', 10)
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

    def publish_navigation_active(self, active):
        message = GoalStatusArray()
        if active:
            status = GoalStatus()
            status.status = GoalStatus.STATUS_EXECUTING
            message.status_list.append(status)
        self.navigation_status_publisher.publish(message)

    def publish_detection_burst(self):
        for _ in range(3):
            self.detections_publisher.publish(DetectedObjectArray())
            self.detection_heartbeat_publisher.publish(Header())
            time.sleep(0.01)

    def control_callback(self, request, response):
        self.control_actions.append(request.action)
        response.accepted = True
        response.scheduled = False
        if request.action == ControlExploration.Request.ACTION_START:
            response.state = ControlExploration.Request.STATE_RUNNING
            response.message = 'fake exploration started'
        elif self.stop_responses_before_idle > 0:
            self.publish_navigation_active(False)
            self.stop_responses_before_idle -= 1
            response.state = ControlExploration.Request.STATE_STOPPING
            response.message = 'fake exploration is still stopping'
        else:
            self.publish_navigation_active(False)
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
        if self.publish_detections_after_spin:
            threading.Thread(
                target=self._publish_delayed_detection_burst,
                daemon=True,
            ).start()
        return Spin.Result()

    def _publish_delayed_detection_burst(self):
        wait_until(
            lambda: self.detection_heartbeat_publisher
            .get_subscription_count() >= 2,
            timeout=0.5,
        )
        self.publish_detection_burst()

    def execute_navigation(self, goal_handle):
        self.navigate_poses.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture
def running_command_layer(tmp_path):
    domain_id = next(_TEST_DOMAIN_IDS)
    process_environment = os.environ.copy()
    process_environment['ROS_DOMAIN_ID'] = str(domain_id)
    rclpy.init(domain_id=domain_id)
    backend = FakeProgramBackends()
    backend.map_save_directory = str(tmp_path)
    backend.bag_output_directory = str(tmp_path / 'bags')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    recorder_process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_exploration_bag',
            'exploration_bag_recorder',
            '--ros-args',
            '-p', f'output_directory:={backend.bag_output_directory}',
            '-p', 'post_terminal_delay:=0.1',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=process_environment,
    )
    wait_until(
        lambda: backend.count_subscribers(
            '/explore_and_record/recording_event') >= 1,
        timeout=5.0,
    )
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_command_layer', 'command_layer_node',
            '--ros-args',
            '-p', 'exploration_cycle_duration:=0.1',
            '-p', 'observation_duration:=1.0',
            '-p', 'observation_min_detection_frames:=3',
            '-p', 'scan_step_count:=6',
            '-p', 'navigation_settle_time:=0.01',
            '-p', 'program_endpoint_timeout:=1.0',
            '-p', 'frontier_navigation_start_timeout:=0.5',
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
            '-p', 'exploration_bag_enabled:=true',
            '-p', 'exploration_bag_required:=true',
            '-p', 'exploration_bag_start_timeout:=2.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=process_environment,
    )
    try:
        action_client = ActionClient(
            backend, ExploreAndRecord, '/explore_and_record')
        assert action_client.wait_for_server(timeout_sec=5.0)
        yield backend, action_client
    finally:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            bag_directories = list(
                Path(backend.bag_output_directory).glob('muto_explore_*'))
            if (not bag_directories or
                    all((path / 'metadata.yaml').exists()
                        for path in bag_directories)):
                break
            time.sleep(0.02)
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5.0)
        os.killpg(recorder_process.pid, signal.SIGINT)
        try:
            recorder_output, _ = recorder_process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(recorder_process.pid, signal.SIGKILL)
            recorder_output, _ = recorder_process.communicate(timeout=5.0)
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        backend.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, output
        assert recorder_process.returncode == 0, recorder_output


def test_one_cycle_spins_checkpoints_and_reports_counts(running_command_layer):
    backend, action_client = running_command_layer
    goal = ExploreAndRecord.Goal()
    goal.max_cycles = 1

    goal_handle = wait_future(action_client.send_goal_async(goal))
    assert goal_handle.accepted
    wait_until(lambda: backend.control_actions == [
        ControlExploration.Request.ACTION_START,
    ])
    operator_event = String()
    operator_event.data = 'observation: test chair visible by the doorway'
    backend.operator_event_publisher.publish(operator_event)
    started = time.monotonic()
    result_response = wait_future(goal_handle.get_result_async())
    elapsed = time.monotonic() - started

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
    assert elapsed < 5.0
    assert backend.spin_angles == pytest.approx([math.pi / 3.0] * 6)
    assert sum(backend.spin_angles) == pytest.approx(2.0 * math.pi)

    metadata_path = wait_for_single_finalized_bag(
        backend.bag_output_directory)
    metadata = metadata_path.read_text(encoding='utf-8')
    assert 'muto_schema: explore_and_record_v1' in metadata
    assert 'git_revision:' in metadata
    assert 'git_dirty:' in metadata
    assert '/sam2/detections' in metadata
    assert '/tf_static' in metadata
    assert '/navigate_to_pose/_action/status' in metadata
    assert '/explore_and_record/recording_event' in metadata
    assert '/explore_and_record/operator_event' in metadata
    metadata_document = yaml.safe_load(metadata)
    topic_counts = {
        entry['topic_metadata']['name']: entry['message_count']
        for entry in metadata_document[
            'rosbag2_bagfile_information']['topics_with_message_count']
    }
    assert topic_counts['/explore_and_record/operator_event'] == 1
    assert topic_counts['/explore_and_record/recording_event'] >= 2
    assert list(metadata_path.parent.glob('*.mcap'))
    manifest_path = metadata_path.parent / 'muto_recording_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    assert manifest['muto_schema'] == 'explore_and_record_v1'
    assert manifest['goal_id']
    assert manifest['git_revision']
    assert manifest['git_dirty'] in ('true', 'false')
    assert manifest['bag_path'] == str(metadata_path.parent)
    assert manifest['manifest_file'] == manifest_path.name
    assert manifest['topic_scope'] == 'all_topics_excluding_regex'
    assert '/camera/' in manifest['exclude_regex']
    assert '/sam2/' in manifest['exclude_regex']
    assert json.loads(manifest['start_event'])['event'] == 'mission_started'


def test_frontier_primitive_travels_without_scanning_or_checkpointing(
        running_command_layer):
    backend, _ = running_command_layer
    backend.publish_navigation_active(True)
    client = ActionClient(
        backend,
        ExploreAndRecord,
        '/command_primitives/explore_frontier',
    )
    assert client.wait_for_server(timeout_sec=2.0)
    goal = ExploreAndRecord.Goal()
    goal.exploration_duration = 0.1

    goal_handle = wait_future(client.send_goal_async(goal))
    result_response = wait_future(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert result_response.result.success
    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
        ControlExploration.Request.ACTION_STOP,
    ]
    assert backend.spin_angles == []
    assert backend.save_calls == 0


def test_frontier_primitive_waits_for_idle_when_stop_is_still_pending(
        running_command_layer):
    backend, _ = running_command_layer
    backend.publish_navigation_active(True)
    backend.stop_responses_before_idle = 1
    client = ActionClient(
        backend,
        ExploreAndRecord,
        '/command_primitives/explore_frontier',
    )
    assert client.wait_for_server(timeout_sec=2.0)
    goal = ExploreAndRecord.Goal()
    goal.exploration_duration = 0.1

    goal_handle = wait_future(client.send_goal_async(goal))
    result_response = wait_future(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
        ControlExploration.Request.ACTION_STOP,
        ControlExploration.Request.ACTION_STOP,
    ]


def test_minimum_interval_does_not_interrupt_active_frontier_travel(
        running_command_layer):
    backend, action_client = running_command_layer
    backend.publish_navigation_active(True)
    goal = ExploreAndRecord.Goal()
    goal.max_cycles = 1

    goal_handle = wait_future(action_client.send_goal_async(goal))
    assert goal_handle.accepted
    wait_until(lambda: backend.control_actions == [
        ControlExploration.Request.ACTION_START,
    ])
    time.sleep(0.25)

    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
    ]
    assert backend.spin_angles == []

    backend.publish_navigation_active(False)
    result_response = wait_future(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert backend.control_actions == [
        ControlExploration.Request.ACTION_START,
        ControlExploration.Request.ACTION_STOP,
    ]
    assert backend.spin_angles == pytest.approx([math.pi / 3.0] * 6)


def test_detector_timeout_preserves_scan_when_perception_is_unavailable(
        running_command_layer):
    backend, action_client = running_command_layer
    backend.publish_detections_after_spin = False
    goal = ExploreAndRecord.Goal()
    goal.observation_duration = 0.05
    goal.max_cycles = 1

    goal_handle = wait_future(action_client.send_goal_async(goal))
    assert goal_handle.accepted
    result_response = wait_future(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED
    assert backend.spin_angles == pytest.approx([math.pi / 3.0] * 6)


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

    metadata_path = wait_for_single_finalized_bag(
        backend.bag_output_directory)
    metadata = metadata_path.read_text(encoding='utf-8')
    assert 'muto_schema: explore_and_record_v1' in metadata
    assert '/explore_and_record/recording_event' in metadata
    assert list(metadata_path.parent.glob('*.mcap'))


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
    assert backend.spin_angles == pytest.approx([math.pi / 3.0] * 6)
    assert backend.save_calls == 2
