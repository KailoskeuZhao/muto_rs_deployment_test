"""Integration tests for the composed active object-search action."""

import os
import signal
import subprocess
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from muto_command_layer.action import ExploreAndRecord, FindObject, FindSomething
from muto_command_layer.msg import ObjectMatch
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sam2_object_registry.msg import StoredObject, StoredObjectArray


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


class FakeActiveSearchBackends(Node):
    def __init__(self, test_id):
        super().__init__(f'fake_active_object_search_backends_{test_id}')
        prefix = f'/test/{test_id}'
        self.find_action = f'{prefix}/find_object'
        self.program_action = f'{prefix}/explore_and_record'
        self.registry_topic = f'{prefix}/stored_objects'
        self.active_search_action = f'{prefix}/find_something'
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.match_available = False
        self.release_program = threading.Event()
        self.find_prompts = []
        self.program_goals = 0
        self.program_cancellations = 0

        self.find_server = ActionServer(
            self,
            FindObject,
            self.find_action,
            execute_callback=self.execute_find,
            callback_group=self._callback_group,
        )
        self.program_server = ActionServer(
            self,
            ExploreAndRecord,
            self.program_action,
            execute_callback=self.execute_program,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.registry_publisher = self.create_publisher(
            StoredObjectArray, self.registry_topic, qos)
        self.publish_registry()

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def publish_registry(self):
        message = StoredObjectArray()
        message.header.frame_id = 'map'
        with self._lock:
            match_available = self.match_available
        if match_available:
            stored = StoredObject()
            stored.name = 'red_mug_2'
            stored.label = 'cup'
            stored.image_path = '/tmp/red_mug_2.jpg'
            message.objects = [stored]
        self.registry_publisher.publish(message)

    def execute_find(self, goal_handle):
        self.find_prompts.append(goal_handle.request.prompt)
        result = FindObject.Result()
        result.success = True
        with self._lock:
            match_available = self.match_available
        if match_available:
            match = ObjectMatch()
            match.object_id = 'red_mug_2'
            match.label = 'cup'
            match.description = 'fake matching red mug'
            result.matches = [match]
            result.message = 'found one matching static object'
        else:
            result.message = 'no registered object matched'
        goal_handle.succeed()
        return result

    def execute_program(self, goal_handle):
        self.program_goals += 1
        while not goal_handle.is_cancel_requested and \
                not self.release_program.is_set():
            time.sleep(0.01)
        result = ExploreAndRecord.Result()
        if goal_handle.is_cancel_requested:
            self.program_cancellations += 1
            result.success = False
            result.message = 'fake program canceled'
            goal_handle.canceled()
            return result
        result.success = True
        result.message = 'fake predictive search completed'
        goal_handle.succeed()
        return result

    def make_match_available(self):
        with self._lock:
            self.match_available = True
        self.publish_registry()


@pytest.fixture
def running_active_search():
    rclpy.init()
    backend = FakeActiveSearchBackends(f't{uuid.uuid4().hex[:8]}')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_command_layer',
            'active_object_search_node',
            '--ros-args',
            '-p', f'action_name:={backend.active_search_action}',
            '-p', f'find_object_action:={backend.find_action}',
            '-p', f'explore_and_record_action:={backend.program_action}',
            '-p', f'registry_topic:={backend.registry_topic}',
            '-p', 'endpoint_timeout:=1.0',
            '-p', 'find_result_timeout:=2.0',
            '-p', 'child_stop_timeout:=2.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        client = ActionClient(
            backend, FindSomething, backend.active_search_action)
        assert client.wait_for_server(timeout_sec=5.0)
        yield backend, client
    finally:
        backend.release_program.set()
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


def send_search(client, prompt='the red mug'):
    goal = FindSomething.Goal()
    goal.prompt = prompt
    goal_handle = wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    return goal_handle


def test_existing_match_returns_without_starting_exploration(
        running_active_search):
    backend, client = running_active_search
    backend.make_match_available()

    wrapped = wait_future(send_search(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert [match.object_id for match in wrapped.result.matches] == [
        'red_mug_2']
    assert backend.find_prompts == ['the red mug']
    assert backend.program_goals == 0


def test_new_registry_object_stops_exploration_and_returns_match(
        running_active_search):
    backend, client = running_active_search
    goal_handle = send_search(client)
    wait_until(lambda: backend.program_goals == 1)

    backend.make_match_available()
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert [match.object_id for match in wrapped.result.matches] == [
        'red_mug_2']
    assert backend.find_prompts == ['the red mug', 'the red mug']
    assert backend.program_cancellations == 1


def test_completed_predictive_search_makes_final_no_match_check(
        running_active_search):
    backend, client = running_active_search
    goal_handle = send_search(client)
    wait_until(lambda: backend.program_goals == 1)

    backend.release_program.set()
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert wrapped.result.matches == []
    assert 'no matching static object' in wrapped.result.message
    assert backend.find_prompts == ['the red mug', 'the red mug']


def test_parent_cancel_stops_composed_exploration(running_active_search):
    backend, client = running_active_search
    goal_handle = send_search(client)
    wait_until(lambda: backend.program_goals == 1)

    cancel_response = wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert cancel_response.goals_canceling
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.success
    assert backend.program_cancellations == 1
