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
    FindObject,
    GoToObject,
    LookForObject,
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from std_srvs.srv import SetBool


def wait_future(future, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), 'ROS future did not complete before timeout'
    return future.result()


def wait_until(predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert predicate(), 'condition did not become true before timeout'


def intent_response(mission_type, target_description='', **overrides):
    desired_end_states = {
        'locate_object': 'report_object',
        'locate_and_approach_object': 'approach_object',
        'approach_known_object': 'approach_object',
        'query_object_registry': 'report_object',
        'start_manual_exploration': 'manual_exploration_started',
        'stop_manual_exploration': 'manual_exploration_stopped',
        'save_current_map': 'map_saved',
        'cancel_active_mission': 'active_mission_canceled',
        'unsupported': 'none',
    }
    response = {
        'mission_type': mission_type,
        'desired_end_state': desired_end_states.get(mission_type, 'none'),
        'target_description': target_description,
        'map_name': '',
    }
    response.update(overrides)
    return json.dumps(response)


class FakeCommandBackends(Node):
    def __init__(self, test_id):
        super().__init__(f'fake_natural_language_command_backends_{test_id}')
        prefix = f'/test/{test_id}'
        self.vlm_action = f'{prefix}/vlm'
        self.find_action = f'{prefix}/find_object'
        self.look_for_object_action = f'{prefix}/look_for_object'
        self.go_action = f'{prefix}/go_to_object'
        self.explore_service_name = f'{prefix}/explore'
        self.save_map_service_name = f'{prefix}/save_map'
        self.router_action = f'{prefix}/natural_language_command'
        self.decision_topic = f'{prefix}/natural_language_decisions'
        self._lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._vlm_responses = deque()
        self.release_motion = threading.Event()
        self.schemas = []
        self.find_prompts = []
        self.model_search_prompts = []
        self.model_search_completion_modes = []
        self.go_object_ids = []
        self.explore_requests = []
        self.save_map_requests = []
        self.motion_cancellations = []
        self.decision_events = []

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
        self.model_search_server = ActionServer(
            self,
            LookForObject,
            self.look_for_object_action,
            execute_callback=self.execute_model_search,
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
        trace_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.decision_subscription = self.create_subscription(
            String,
            self.decision_topic,
            lambda message: self.decision_events.append(
                json.loads(message.data)),
            trace_qos,
        )

    def queue_intent(self, mission_type, target_description='', **overrides):
        with self._lock:
            self._vlm_responses.append(intent_response(
                mission_type, target_description, **overrides))

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

    def execute_model_search(self, goal_handle):
        self.model_search_prompts.append(goal_handle.request.prompt)
        self.model_search_completion_modes.append(
            int(goal_handle.request.completion_mode))
        succeeded = self._wait_for_motion(goal_handle, 'look_for_object')
        result = LookForObject.Result()
        result.success = succeeded
        result.found = False
        result.outcome = (
            LookForObject.Result.OUTCOME_NOT_FOUND
            if succeeded else LookForObject.Result.OUTCOME_CANCELED
        )
        result.message = 'fake model search completed or canceled'
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
    test_id = f't{uuid.uuid4().hex[:8]}'
    domain_id = 100 + (int(test_id[1:], 16) % 100)
    rclpy.init(domain_id=domain_id)
    backend = FakeCommandBackends(test_id)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    process_env = os.environ.copy()
    process_env['ROS_DOMAIN_ID'] = str(domain_id)
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_command_layer',
            'natural_language_command_node',
            '--ros-args',
            '-p', f'action_name:={backend.router_action}',
            '-p', f'vlm_action:={backend.vlm_action}',
            '-p', f'find_object_action:={backend.find_action}',
            '-p', f'look_for_object_action:={backend.look_for_object_action}',
            '-p', f'go_to_object_action:={backend.go_action}',
            '-p', f'explore_service:={backend.explore_service_name}',
            '-p', f'save_map_service:={backend.save_map_service_name}',
            '-p', f'decision_event_topic:={backend.decision_topic}',
            '-p', 'endpoint_timeout:=1.0',
            '-p', 'cancel_timeout:=1.0',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=process_env,
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

    backend.queue_intent('start_manual_exploration')
    started = send_command(client, 'please start exploring')
    stopped = send_command(client, 'stop exploration')
    backend.queue_intent('query_object_registry', 'the red chair')
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

    backend.queue_intent('save_current_map', map_name='warehouse.v2')
    saved = send_command(client, 'save the map as warehouse version two')

    assert saved.status == GoalStatus.STATUS_SUCCEEDED
    assert saved.result.command == 'save_map'
    assert backend.save_map_requests == ['warehouse.v2']


def test_go_to_object_dispatches_exact_id_then_natural_cancel(
        running_router):
    backend, client = running_router

    dispatched = send_command(client, 'go to the red chair')
    wait_until(lambda: backend.go_object_ids == ['chair_2'])
    canceled = send_command(client, 'cancel the current command')
    wait_until(lambda: backend.motion_cancellations == ['go_to_object'])

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert dispatched.result.object_ids == ['chair_2']
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED
    assert 'cancellation accepted' in canceled.result.message
    assert len(backend.schemas) == 0


def test_plain_find_dispatches_model_commander_and_supports_cancel(
        running_router):
    backend, client = running_router

    dispatched = send_command(client, 'find the red mug')
    wait_until(lambda: backend.model_search_prompts == ['the red mug'])
    canceled = send_command(client, 'cancel the active command')
    wait_until(
        lambda: backend.motion_cancellations == ['look_for_object'])

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert dispatched.result.command == 'look_for_object'
    assert backend.model_search_completion_modes == [
        LookForObject.Goal.COMPLETION_REPORT_OBJECT]
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED
    assert len(backend.schemas) == 0
    wait_until(lambda: any(
        event['event'] == 'validated_intent' and
        event['query'] == 'find the red mug' and
        event['command'] == 'look_for_object'
        for event in backend.decision_events
    ))
    assert any(
        event['event'] == 'validated_intent' and
        event['query'] == 'find the red mug' and
        event['command'] == 'look_for_object'
        for event in backend.decision_events
    )


def test_find_then_approach_is_dispatched_as_one_supervised_mission(
        running_router):
    backend, client = running_router

    dispatched = send_command(
        client, 'go find a green chair, and thehen go near the chair')
    wait_until(lambda: backend.model_search_prompts == ['a green chair'])
    canceled = send_command(client, 'cancel the active command')

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert dispatched.result.command == 'look_for_object'
    assert 'find-and-approach' in dispatched.result.message
    assert backend.model_search_completion_modes == [
        LookForObject.Goal.COMPLETION_APPROACH_OBJECT]
    assert backend.go_object_ids == []
    assert backend.schemas == []
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED


def test_explicit_registry_query_remains_stationary(
        running_router):
    backend, client = running_router

    found = send_command(client, 'check registry for the red chair')

    assert found.status == GoalStatus.STATUS_SUCCEEDED
    assert found.result.command == 'find_object'
    assert backend.find_prompts == ['the red chair']
    assert backend.motion_cancellations == []
    assert len(backend.schemas) == 0


def test_look_for_object_dispatches_persistent_model_mission_and_cancel(
        running_router):
    backend, client = running_router

    dispatched = send_command(client, 'look for the red mug by the kettle')
    wait_until(
        lambda: backend.model_search_prompts == [
            'the red mug by the kettle'])
    canceled = send_command(client, 'cancel the active command')
    wait_until(
        lambda: backend.motion_cancellations == ['look_for_object'])

    assert dispatched.status == GoalStatus.STATUS_SUCCEEDED
    assert dispatched.result.command == 'look_for_object'
    assert canceled.status == GoalStatus.STATUS_SUCCEEDED
    assert len(backend.schemas) == 0
