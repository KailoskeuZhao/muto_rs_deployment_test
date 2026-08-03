"""Integration tests for validated natural-language command dispatch."""

from collections import deque
import json
import os
import signal
import subprocess
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from muto_command_layer.action import (
    ExploreAndRecord,
    FindObject,
    FindSomething,
    GoToObject,
    NaturalLanguageCommand,
)
from muto_command_layer.msg import ObjectMatch
from muto_vlm_socket.action import GenerateVlm
import pytest
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_srvs.srv import SetBool


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


def intent_response(command, object_query='', **overrides):
    response = {
        'command': command,
        'object_query': object_query,
        'map_name': '',
        'exploration_duration': 0.0,
        'observation_duration': 0.0,
        'scan_step_count': 0,
        'max_cycles': 0,
    }
    response.update(overrides)
    return json.dumps(response)


class FakeCommandBackends(Node):
    def __init__(self, test_id):
        super().__init__(f'fake_natural_language_command_backends_{test_id}')
        prefix = f'/test/{test_id}'
        self.vlm_action = f'{prefix}/vlm'
        self.find_action = f'{prefix}/find_object'
        self.find_something_action = f'{prefix}/find_something'
        self.go_action = f'{prefix}/go_to_object'
        self.program_action = f'{prefix}/explore_and_record'
        self.explore_service_name = f'{prefix}/explore'
        self.save_map_service_name = f'{prefix}/save_map'
        self.router_action = f'{prefix}/natural_language_command'
        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._vlm_responses = deque()
        self.release_motion = threading.Event()
        self.schemas = []
        self.find_prompts = []
        self.go_object_ids = []
        self.program_goals = []
        self.explore_requests = []
        self.save_map_requests = []
        self.motion_cancellations = []

        self.vlm_server = ActionServer(
            self,
            GenerateVlm,
            self.vlm_action,
            execute_callback=self.execute_vlm,
            callback_group=self._callback_group,
        )
        self.find_server = ActionServer(
            self,
            FindObject,
            self.find_action,
            execute_callback=self.execute_find,
            callback_group=self._callback_group,
        )
        self.go_server = ActionServer(
            self,
            GoToObject,
            self.go_action,
            execute_callback=self.execute_go,
            cancel_callback=self.accept_cancel,
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
        self.active_search_server = ActionServer(
            self,
            FindSomething,
            self.find_something_action,
            execute_callback=self.execute_active_search,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
        self.explore_service = self.create_service(
            SetBool,
            self.explore_service_name,
            self.handle_explore,
            callback_group=self._callback_group,
        )
        self.save_map_service = self.create_service(
            SaveMap,
            self.save_map_service_name,
            self.handle_save_map,
            callback_group=self._callback_group,
        )

    def queue_intent(self, command, object_query='', **overrides):
        with self._lock:
            self._vlm_responses.append(intent_response(
                command, object_query, **overrides))

    def execute_vlm(self, goal_handle):
        with self._lock:
            assert self._vlm_responses, 'test did not queue a VLM intent'
            response_text = self._vlm_responses.popleft()
            self.schemas.append(json.loads(
                goal_handle.request.response_json_schema))
        result = GenerateVlm.Result()
        result.success = True
        result.response_text = response_text
        goal_handle.succeed()
        return result

    def execute_find(self, goal_handle):
        self.find_prompts.append(goal_handle.request.prompt)
        match = ObjectMatch()
        match.object_id = 'chair_2'
        match.label = 'chair'
        match.description = 'fake exact match'
        result = FindObject.Result()
        result.success = True
        result.message = 'found one fake object'
        result.matches = [match]
        goal_handle.succeed()
        return result

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _wait_for_motion(self, goal_handle, command):
        while not goal_handle.is_cancel_requested and \
                not self.release_motion.is_set():
            time.sleep(0.01)
        if goal_handle.is_cancel_requested:
            self.motion_cancellations.append(command)
            goal_handle.canceled()
            return False
        goal_handle.succeed()
        return True

    def execute_go(self, goal_handle):
        self.go_object_ids.append(goal_handle.request.object_id)
        succeeded = self._wait_for_motion(goal_handle, 'go_to_object')
        result = GoToObject.Result()
        result.success = succeeded
        result.message = 'fake navigation completed or canceled'
        return result

    def execute_program(self, goal_handle):
        request = goal_handle.request
        self.program_goals.append((
            request.exploration_duration,
            request.observation_duration,
            request.scan_step_count,
            request.max_cycles,
        ))
        succeeded = self._wait_for_motion(goal_handle, 'explore_and_record')
        result = ExploreAndRecord.Result()
        result.success = succeeded
        result.message = 'fake mission completed or canceled'
        return result

    def execute_active_search(self, goal_handle):
        self.find_prompts.append(goal_handle.request.prompt)
        succeeded = self._wait_for_motion(goal_handle, 'find_something')
        result = FindSomething.Result()
        result.success = succeeded
        result.message = 'fake active search completed or canceled'
        return result

    def handle_explore(self, request, response):
        self.explore_requests.append(request.data)
        response.success = True
        response.message = (
            'fake exploration started' if request.data
            else 'fake exploration stopped'
        )
        return response

    def handle_save_map(self, request, response):
        self.save_map_requests.append(request.name.data)
        response.result = SaveMap.Response.RESULT_SUCCESS
        return response


@pytest.fixture
def running_router():
    rclpy.init()
    backend = FakeCommandBackends(f't{uuid.uuid4().hex[:8]}')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_command_layer',
            'natural_language_command_node',
            '--ros-args',
            '-p', f'action_name:={backend.router_action}',
            '-p', f'vlm_action:={backend.vlm_action}',
            '-p', f'find_object_action:={backend.find_action}',
            '-p', f'find_something_action:={backend.find_something_action}',
            '-p', f'go_to_object_action:={backend.go_action}',
            '-p', f'explore_service:={backend.explore_service_name}',
            '-p', f'save_map_service:={backend.save_map_service_name}',
            '-p', f'explore_and_record_action:={backend.program_action}',
            '-p', 'endpoint_timeout:=1.0',
            '-p', 'cancel_timeout:=1.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        client = ActionClient(
            backend,
            NaturalLanguageCommand,
            backend.router_action,
        )
        assert client.wait_for_server(timeout_sec=5.0)
        yield backend, client
    finally:
        backend.release_motion.set()
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


def send_command(client, query):
    goal = NaturalLanguageCommand.Goal()
    goal.query = query
    goal_handle = wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    return wait_future(goal_handle.get_result_async())


def test_service_and_object_search_commands_are_typed(running_router):
    backend, client = running_router

    backend.queue_intent('start_exploration')
    started = send_command(client, 'please start exploring')
    backend.queue_intent('stop_exploration')
    stopped = send_command(client, 'stop exploration')
    backend.queue_intent('find_object', 'the red chair')
    found = send_command(client, 'which red chair did you record?')

    assert started.status == GoalStatus.STATUS_SUCCEEDED
    assert stopped.status == GoalStatus.STATUS_SUCCEEDED
    assert found.status == GoalStatus.STATUS_SUCCEEDED
    assert backend.explore_requests == [True, False]
    assert backend.find_prompts == ['the red chair']
    assert found.result.object_ids == ['chair_2']
    assert all(schema['type'] == 'object' for schema in backend.schemas)
    assert all('schema' not in schema for schema in backend.schemas)


def test_save_map_dispatches_only_validated_basename(running_router):
    backend, client = running_router

    backend.queue_intent('save_map', map_name='warehouse.v2')
    saved = send_command(client, 'save the map as warehouse version two')

    assert saved.status == GoalStatus.STATUS_SUCCEEDED
    assert saved.result.command == 'save_map'
    assert backend.save_map_requests == ['warehouse.v2']


def test_go_to_object_dispatches_exact_id_then_natural_cancel(
        running_router):
    backend, client = running_router

    backend.queue_intent('go_to_object', 'the red chair')
    dispatched = send_command(client, 'go to the red chair')
    wait_until(lambda: backend.go_object_ids == ['chair_2'])
    backend.queue_intent('cancel_active_command')
    canceled = send_command(client, 'cancel the current command')
    wait_until(lambda: backend.motion_cancellations == ['go_to_object'])

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert dispatched.result.object_ids == ['chair_2']
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED
    assert 'cancellation accepted' in canceled.result.message


def test_explore_and_record_forwards_only_bounded_arguments(running_router):
    backend, client = running_router

    backend.queue_intent(
        'explore_and_record',
        exploration_duration=12.0,
        observation_duration=2.5,
        scan_step_count=8,
        max_cycles=3,
    )
    dispatched = send_command(
        client, 'explore, scan in eight steps, and record objects')
    wait_until(lambda: len(backend.program_goals) == 1)
    backend.queue_intent('cancel_active_command')
    canceled = send_command(client, 'stop the autonomous mission')
    wait_until(
        lambda: backend.motion_cancellations == ['explore_and_record'])

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert backend.program_goals[0] == pytest.approx((12.0, 2.5, 8, 3))
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED


def test_find_something_dispatches_active_search_and_supports_cancel(
        running_router):
    backend, client = running_router

    backend.queue_intent('find_something', 'the red mug')
    dispatched = send_command(client, 'go search for the red mug')
    wait_until(lambda: backend.find_prompts == ['the red mug'])
    backend.queue_intent('cancel_active_command')
    canceled = send_command(client, 'cancel the active search')
    wait_until(lambda: backend.motion_cancellations == ['find_something'])

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert dispatched.result.command == 'find_something'
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED
