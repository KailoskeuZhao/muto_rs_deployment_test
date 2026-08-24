"""ROS integration test for the frontier-only NavigateToPose adapter."""

import math
import os
import signal
import subprocess
import sys
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _wait_future(future, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), 'ROS future did not complete before timeout'
    return future.result()


def _wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert predicate(), 'condition did not become true before timeout'


class FakeNav2(Node):
    def __init__(self, test_id):
        super().__init__(f'fake_frontier_adapter_nav2_{test_id}')
        prefix = f'/test/{test_id}'
        self.input_action = f'{prefix}/frontier_navigate_to_pose'
        self.nav2_action = f'{prefix}/navigate_to_pose'
        self.map_topic = f'{prefix}/map'
        self.original_topic = f'{prefix}/original_goal'
        self.projected_topic = f'{prefix}/projected_goal'
        self.status_topic = f'{prefix}/status'
        self.base_frame = f'{test_id}_base_frame'
        self.received_goals = []
        self.status_messages = []
        self.projected_messages = []
        self.cmd_vel_messages = []
        self.hold_navigation = False
        self.navigation_started = threading.Event()
        self.navigation_cancellations = 0
        callback_group = ReentrantCallbackGroup()

        self.server = ActionServer(
            self,
            NavigateToPose,
            self.nav2_action,
            execute_callback=self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, self.map_topic, map_qos)
        self.create_subscription(
            String, self.status_topic,
            lambda message: self.status_messages.append(message.data), 10)
        self.create_subscription(
            type(self._pose_message()), self.projected_topic,
            lambda message: self.projected_messages.append(message), 10)
        self.create_subscription(
            Twist, '/cmd_vel',
            lambda message: self.cmd_vel_messages.append(message), 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = (50 + 0.5) * 0.04
        transform.transform.translation.y = (40 + 0.5) * 0.04
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _pose_message():
        from geometry_msgs.msg import PoseStamped
        return PoseStamped()

    def _execute(self, goal_handle):
        self.received_goals.append(goal_handle.request)
        self.navigation_started.set()
        result = NavigateToPose.Result()
        if hasattr(result, 'error_code'):
            result.error_code = 0
        while self.hold_navigation and not goal_handle.is_cancel_requested:
            time.sleep(0.01)
        if goal_handle.is_cancel_requested:
            self.navigation_cancellations += 1
            goal_handle.canceled()
            return result
        goal_handle.succeed()
        return result

    def publish_open_patch(self):
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = 100
        message.info.height = 80
        message.info.resolution = 0.04
        message.info.origin.orientation.w = 1.0
        message.data = [-1] * (100 * 80)
        for cell_y in range(20, 60):
            for cell_x in range(25, 75):
                message.data[cell_y * 100 + cell_x] = 0
        self.map_publisher.publish(message)

    def publish_shallow_impasse(self):
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = 100
        message.info.height = 80
        message.info.resolution = 0.04
        message.info.origin.orientation.w = 1.0
        message.data = [-1] * (100 * 80)
        # The robot is at (50, 40). After applying its 0.27 m footprint, this
        # patch offers only a few centimetres of forward staging room.
        for cell_y in range(33, 49):
            for cell_x in range(43, 58):
                message.data[cell_y * 100 + cell_x] = 0
        self.map_publisher.publish(message)

    def publish_lateral_staging_patch(self):
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.width = 100
        message.info.height = 80
        message.info.resolution = 0.04
        message.info.origin.orientation.w = 1.0
        message.data = [-1] * (100 * 80)
        for cell_y in range(33, 50):
            for cell_x in range(20, 81):
                message.data[cell_y * 100 + cell_x] = 0
        self.map_publisher.publish(message)


@pytest.fixture
def running_adapter():
    rclpy.init()
    backend = FakeNav2(f't{uuid.uuid4().hex[:8]}')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    source_executable = os.environ.get(
        'FRONTIER_GOAL_ADAPTER_EXECUTABLE')
    if source_executable:
        command = [sys.executable, source_executable]
    else:
        command = [
            'ros2', 'run', 'muto_slam_mapping', 'frontier_goal_adapter']
    process = subprocess.Popen(
        [
            *command,
            '--ros-args',
            '-p', f'input_action:={backend.input_action}',
            '-p', f'nav2_action:={backend.nav2_action}',
            '-p', f'map_topic:={backend.map_topic}',
            '-p', f'original_goal_topic:={backend.original_topic}',
            '-p', f'projected_goal_topic:={backend.projected_topic}',
            '-p', f'status_topic:={backend.status_topic}',
            '-p', f'robot_base_frame:={backend.base_frame}',
            '-p', 'effective_robot_radius:=0.27',
            '-p', 'maximum_projection_distance:=0.0',
            '-p', 'map_wait_timeout:=1.0',
            '-p', 'nav2_server_timeout:=1.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        client = ActionClient(
            backend, NavigateToPose, backend.input_action)
        assert client.wait_for_server(timeout_sec=5.0)
        backend.publish_open_patch()
        yield backend, client
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5.0)
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        backend.server.destroy()
        backend.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, output


def test_unknown_frontier_endpoint_is_projected_before_nav2(running_adapter):
    backend, client = running_adapter
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = (50 + 0.5) * 0.04
    goal.pose.pose.position.y = (60 + 0.5) * 0.04
    goal.pose.pose.orientation.w = 1.0

    goal_handle = _wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    wrapped = _wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert len(backend.received_goals) == 1
    forwarded = backend.received_goals[0].pose
    assert math.isclose(
        forwarded.pose.position.x,
        goal.pose.pose.position.x,
        abs_tol=1.0e-6,
    )
    assert forwarded.pose.position.y < goal.pose.pose.position.y
    _wait_until(lambda: len(backend.status_messages) >= 2)
    assert len(backend.projected_messages) == 1
    assert any('"outcome":"projected"' in item
               for item in backend.status_messages)
    assert any('"outcome":"succeeded"' in item
               for item in backend.status_messages)
    assert backend.cmd_vel_messages == []


def test_distant_unsafe_frontier_is_forwarded_as_staged_advance(
        running_adapter):
    backend, client = running_adapter
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = (50 + 0.5) * 0.04
    goal.pose.pose.position.y = (78 + 0.5) * 0.04
    goal.pose.pose.orientation.w = 1.0

    goal_handle = _wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    wrapped = _wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert len(backend.received_goals) == 1
    forwarded = backend.received_goals[0].pose
    displacement = math.hypot(
        forwarded.pose.position.x - goal.pose.pose.position.x,
        forwarded.pose.position.y - goal.pose.pose.position.y,
    )
    assert displacement > 0.80
    assert forwarded.pose.position.y < goal.pose.pose.position.y
    assert backend.cmd_vel_messages == []


def test_local_impasse_is_rejected_instead_of_reporting_false_progress(
        running_adapter):
    backend, client = running_adapter
    backend.publish_shallow_impasse()
    time.sleep(0.15)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = (50 + 0.5) * 0.04
    goal.pose.pose.position.y = (60 + 0.5) * 0.04
    goal.pose.pose.orientation.w = 1.0

    goal_handle = _wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    wrapped = _wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert backend.received_goals == []
    _wait_until(lambda: any(
        '"outcome":"rejected_no_progress"' in item
        for item in backend.status_messages))
    assert backend.cmd_vel_messages == []


def test_near_projection_uses_reachable_lateral_staging_goal(running_adapter):
    backend, client = running_adapter
    backend.publish_lateral_staging_patch()
    time.sleep(0.15)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = (50 + 0.5) * 0.04
    goal.pose.pose.position.y = (60 + 0.5) * 0.04
    goal.pose.pose.orientation.w = 1.0

    goal_handle = _wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    wrapped = _wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert len(backend.received_goals) == 1
    forwarded = backend.received_goals[0].pose.pose.position
    robot_x = (50 + 0.5) * 0.04
    robot_y = (40 + 0.5) * 0.04
    assert math.hypot(forwarded.x - robot_x, forwarded.y - robot_y) >= 0.20
    assert not any(
        '"outcome":"rejected_no_progress"' in item
        for item in backend.status_messages)


def test_parent_cancel_propagates_to_the_owned_nav2_goal(running_adapter):
    backend, client = running_adapter
    backend.hold_navigation = True
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = (50 + 0.5) * 0.04
    goal.pose.pose.position.y = (60 + 0.5) * 0.04
    goal.pose.pose.orientation.w = 1.0

    goal_handle = _wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    assert backend.navigation_started.wait(timeout=5.0)
    cancel_response = _wait_future(goal_handle.cancel_goal_async())
    wrapped = _wait_future(goal_handle.get_result_async())

    assert cancel_response.goals_canceling
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert backend.navigation_cancellations == 1
    assert backend.cmd_vel_messages == []
