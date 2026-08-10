"""Integration tests for the persistent model-supervised object search."""

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
    LookForObject,
)
from muto_command_layer.msg import ObjectMatch
from muto_vlm_socket.action import GenerateVlm
from muto_vlm_socket.msg import VlmContent
import pytest
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sam2_object_registry.msg import StoredObject, StoredObjectArray
from sensor_msgs.msg import Image
from std_msgs.msg import String


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


def decision_response(
        decision, reason='bounded test decision',
        wait_seconds=0.0, exploration_cycles=0,
        visual_observation='synthetic camera view is clear',
        target_evidence='not_visible'):
    return json.dumps({
        'decision': decision,
        'reason': reason,
        'wait_seconds': wait_seconds,
        'exploration_cycles': exploration_cycles,
        'visual_observation': visual_observation,
        'target_evidence': target_evidence,
    })


def active_inspection_response(
        directive='continue_current_command',
        reason='the bounded search remains useful',
        visual_observation='the visible route remains clear',
        target_evidence='not_visible'):
    return json.dumps({
        'directive': directive,
        'reason': reason,
        'visual_observation': visual_observation,
        'target_evidence': target_evidence,
    })


class FakeCommanderBackends(Node):
    def __init__(self, test_id):
        super().__init__(f'fake_model_commander_backends_{test_id}')
        prefix = f'/test/{test_id}'
        self.vlm_action = f'{prefix}/vlm'
        self.find_action = f'{prefix}/find_object'
        self.program_action = f'{prefix}/explore_and_record'
        self.registry_topic = f'{prefix}/stored_objects'
        self.image_topic = f'{prefix}/camera/color/image_raw'
        self.commander_action = f'{prefix}/look_for_object'
        self.status_topic = f'{prefix}/model_commander_status'
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._decisions = deque()
        self.match_available = False
        self.unrelated_object_available = False
        self.find_failures_remaining = 0
        self.find_started = threading.Event()
        self.find_release_event = None
        self.release_program = threading.Event()
        self.vlm_requests = 0
        self.vlm_camera_jpegs = []
        self.vlm_camera_labels = []
        self.vlm_cancellations = 0
        self.vlm_rejections_remaining = 0
        self.ignore_vlm_cancel = False
        self.find_prompts = []
        self.program_cycles = []
        self.program_cancellations = 0
        self.statuses = []
        self.camera_sequence = 0

        self.vlm_server = ActionServer(
            self,
            GenerateVlm,
            self.vlm_action,
            execute_callback=self.execute_vlm,
            goal_callback=self.accept_or_reject_vlm,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
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
        camera_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.camera_publisher = self.create_publisher(
            Image, self.image_topic, camera_qos)
        self.camera_timer = self.create_timer(0.03, self.publish_camera)
        self.status_subscription = self.create_subscription(
            String, self.status_topic, self.status_callback, qos)
        self.publish_registry()

    @staticmethod
    def accept_cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def accept_or_reject_vlm(self, _goal_request):
        with self._lock:
            if self.vlm_rejections_remaining > 0:
                self.vlm_rejections_remaining -= 1
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def queue_decision(
            self, decision, reason='bounded test decision',
            wait_seconds=0.0, exploration_cycles=0, release_event=None,
            visual_observation='synthetic camera view is clear',
            target_evidence='not_visible'):
        with self._lock:
            self._decisions.append((
                decision_response(
                    decision,
                    reason=reason,
                    wait_seconds=wait_seconds,
                    exploration_cycles=exploration_cycles,
                    visual_observation=visual_observation,
                    target_evidence=target_evidence,
                ),
                release_event,
                '',
            ))

    def queue_model_failure(self, message='fake model transport failure'):
        with self._lock:
            self._decisions.append(('', None, message))

    def queue_active_inspection(
            self, directive='continue_current_command',
            reason='the bounded search remains useful', release_event=None,
            visual_observation='the visible route remains clear',
            target_evidence='not_visible'):
        with self._lock:
            self._decisions.append((
                active_inspection_response(
                    directive=directive,
                    reason=reason,
                    visual_observation=visual_observation,
                    target_evidence=target_evidence,
                ),
                release_event,
                '',
            ))

    def execute_vlm(self, goal_handle):
        with self._lock:
            assert self._decisions, 'test did not queue a model decision'
            response_text, release_event, failure = self._decisions.popleft()
            self.vlm_requests += 1
        assert json.loads(
            goal_handle.request.response_json_schema)['additionalProperties'] \
            is False
        parts = list(goal_handle.request.content)
        assert [part.type for part in parts] == [
            VlmContent.TYPE_TEXT,
            VlmContent.TYPE_TEXT,
            VlmContent.TYPE_JPEG,
        ]
        assert 'LIVE_CAMERA_VIEW_METADATA_JSON=' in parts[1].text
        jpeg_data = bytes(parts[2].jpeg_data)
        assert jpeg_data.startswith(b'\xff\xd8')
        assert jpeg_data.endswith(b'\xff\xd9')
        self.vlm_camera_labels.append(parts[1].text)
        self.vlm_camera_jpegs.append(jpeg_data)
        while release_event is not None and not release_event.is_set() and \
                (not goal_handle.is_cancel_requested or
                 self.ignore_vlm_cancel):
            time.sleep(0.01)
        result = GenerateVlm.Result()
        if goal_handle.is_cancel_requested:
            self.vlm_cancellations += 1
            result.success = False
            result.error_message = 'fake planning canceled'
            goal_handle.canceled()
            return result
        if failure:
            result.success = False
            result.error_message = failure
            goal_handle.abort()
            return result
        result.success = True
        result.response_text = response_text
        goal_handle.succeed()
        return result

    def execute_find(self, goal_handle):
        self.find_prompts.append(goal_handle.request.prompt)
        with self._lock:
            match_available = self.match_available
            fail = self.find_failures_remaining > 0
            release_event = self.find_release_event
            if fail:
                self.find_failures_remaining -= 1
        self.find_started.set()
        while release_event is not None and not release_event.is_set() and \
                not goal_handle.is_cancel_requested:
            time.sleep(0.01)
        result = FindObject.Result()
        if fail:
            result.success = False
            result.message = 'fake transient registry search failure'
            goal_handle.abort()
            return result
        result.success = True
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
        self.program_cycles.append(goal_handle.request.max_cycles)
        feedback = ExploreAndRecord.Feedback()
        feedback.phase = ExploreAndRecord.Feedback.PHASE_EXPLORING
        feedback.status = 'fake bounded exploration is active'
        goal_handle.publish_feedback(feedback)
        while not goal_handle.is_cancel_requested and \
                not self.release_program.is_set():
            time.sleep(0.01)
        result = ExploreAndRecord.Result()
        if goal_handle.is_cancel_requested:
            self.program_cancellations += 1
            result.success = False
            result.message = 'fake exploration canceled'
            goal_handle.canceled()
            return result
        result.success = True
        result.message = 'fake bounded exploration completed'
        result.completed_cycles = goal_handle.request.max_cycles
        goal_handle.succeed()
        return result

    def publish_registry(self):
        message = StoredObjectArray()
        message.header.frame_id = 'map'
        with self._lock:
            match_available = self.match_available
            unrelated_available = self.unrelated_object_available
        objects = []
        if unrelated_available:
            unrelated = StoredObject()
            unrelated.name = 'blue_box'
            unrelated.label = 'box'
            unrelated.class_id = 1
            unrelated.image_path = '/tmp/blue_box.jpg'
            objects.append(unrelated)
        if match_available:
            stored = StoredObject()
            stored.name = 'red_mug_2'
            stored.label = 'cup'
            stored.class_id = 2
            stored.image_path = '/tmp/red_mug_2.jpg'
            objects.append(stored)
        message.objects = objects
        self.registry_publisher.publish(message)

    def publish_camera(self):
        self.camera_sequence += 1
        value = self.camera_sequence % 256
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'camera_color_optical_frame'
        message.height = 8
        message.width = 8
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = bytes([value, 40, 200]) * (
            message.height * message.width)
        self.camera_publisher.publish(message)

    def make_match_available(self):
        with self._lock:
            self.match_available = True
        self.publish_registry()

    def clear_match(self):
        with self._lock:
            self.match_available = False
        self.publish_registry()

    def make_unrelated_object_available(self):
        with self._lock:
            self.unrelated_object_available = True
        self.publish_registry()

    def status_callback(self, message):
        self.statuses.append(json.loads(message.data))


@pytest.fixture
def running_commander():
    test_id = f't{uuid.uuid4().hex[:8]}'
    domain_id = 100 + (int(test_id[1:], 16) % 100)
    rclpy.init(domain_id=domain_id)
    backend = FakeCommanderBackends(test_id)
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(backend)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    process_env = os.environ.copy()
    process_env['ROS_DOMAIN_ID'] = str(domain_id)
    process = subprocess.Popen(
        [
            'ros2', 'run', 'muto_command_layer', 'model_commander_node',
            '--ros-args',
            '-p', f'action_name:={backend.commander_action}',
            '-p', f'vlm_action:={backend.vlm_action}',
            '-p', f'find_object_action:={backend.find_action}',
            '-p', f'explore_and_record_action:={backend.program_action}',
            '-p', f'registry_topic:={backend.registry_topic}',
            '-p', f'visual_observation_topic:={backend.image_topic}',
            '-p', f'status_topic:={backend.status_topic}',
            '-p', 'endpoint_timeout:=1.0',
            '-p', 'vlm_result_timeout:=3.0',
            '-p', 'find_result_timeout:=3.0',
            '-p', 'child_stop_timeout:=2.0',
            '-p', 'default_max_duration:=20.0',
            '-p', 'default_max_planning_steps:=12',
            '-p', 'planner_retry_initial_delay:=0.05',
            '-p', 'planner_retry_max_delay:=0.1',
            '-p', 'command_retry_initial_delay:=0.05',
            '-p', 'command_retry_max_delay:=0.1',
            '-p', 'visual_observation_timeout:=1.0',
            '-p', 'visual_observation_max_age:=0.5',
            '-p', 'active_inspection_period:=0.5',
            '-p', 'active_inspection_timeout:=2.0',
            '-p', 'active_inspection_max_decision_age:=3.0',
            '-p', 'monitor_period:=0.01',
            '-p', 'status_publish_period:=0.05',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=process_env,
    )
    try:
        client = ActionClient(
            backend, LookForObject, backend.commander_action)
        assert client.wait_for_server(timeout_sec=8.0)
        yield backend, client
    finally:
        backend.release_program.set()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5.0)
        executor.shutdown(timeout_sec=5.0)
        spin_thread.join(timeout=5.0)
        backend.destroy_node()
        rclpy.shutdown()
        assert process.returncode == 0, output


def send_mission(client, prompt='the red mug'):
    goal = LookForObject.Goal()
    goal.prompt = prompt
    goal_handle = wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted
    return goal_handle


def test_existing_match_finishes_without_planning_or_motion(
        running_commander):
    backend, client = running_commander
    backend.make_match_available()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert wrapped.result.found
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_FOUND
    assert [item.object_id for item in wrapped.result.matches] == ['red_mug_2']
    assert backend.vlm_requests == 0
    assert backend.program_cycles == []


def test_transient_registry_search_failure_retries_without_motion(
        running_commander):
    backend, client = running_commander
    with backend._lock:
        backend.find_failures_remaining = 1
    backend.make_match_available()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.found
    assert backend.find_prompts == ['the red mug', 'the red mug']
    assert backend.vlm_requests == 0
    assert backend.program_cycles == []


def test_stale_find_match_is_discarded_when_object_set_changes(
        running_commander):
    backend, client = running_commander
    release_find = threading.Event()
    backend.make_match_available()
    backend.find_release_event = release_find
    backend.queue_decision(
        'wait', reason='wait after verifying the new inventory',
        wait_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(backend.find_started.is_set)
    backend.clear_match()
    release_find.set()

    wait_until(lambda: len(backend.find_prompts) >= 2)
    wait_until(lambda: backend.vlm_requests >= 1)
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.found
    assert backend.find_prompts[:2] == ['the red mug', 'the red mug']


def test_registry_change_cancels_bounded_step_and_returns_match(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    goal_handle = send_mission(client)
    wait_until(lambda: backend.program_cycles == [1])

    backend.make_match_available()
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.found
    assert backend.program_cancellations == 1
    assert backend.find_prompts == ['the red mug', 'the red mug']


def test_unresponsive_inspector_cancellation_still_stops_moving_child(
        running_commander):
    backend, client = running_commander
    hold_inspector = threading.Event()
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    backend.queue_active_inspection(
        'continue_current_command', release_event=hold_inspector)
    backend.ignore_vlm_cancel = True

    goal_handle = send_mission(client)
    wait_until(lambda: backend.vlm_requests >= 2)
    backend.make_unrelated_object_available()

    wrapped = wait_future(goal_handle.get_result_async(), timeout=8.0)
    hold_inspector.set()

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert backend.program_cycles == [1]
    assert backend.program_cancellations == 1


def test_deferred_decision_wakes_early_for_registry_change(
        running_commander):
    backend, client = running_commander
    backend.queue_decision(
        'wait', reason='wait for new evidence', wait_seconds=10.0)
    goal_handle = send_mission(client)
    wait_until(lambda: any(
        status['phase'] == 'deferred' for status in backend.statuses))

    backend.make_match_available()
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.result.found
    assert backend.program_cycles == []
    assert backend.vlm_requests == 1


def test_vlm_actively_inspects_and_can_end_a_model_directed_wait(
        running_commander):
    backend, client = running_commander
    backend.queue_decision(
        'wait', reason='wait for visual context', wait_seconds=10.0)
    backend.queue_active_inspection(
        'interrupt_and_replan',
        reason='target-like evidence should be checked now',
        visual_observation='a red mug-like object may be visible',
        target_evidence='possible',
    )
    backend.queue_decision(
        'wait', reason='continue after registry verification',
        wait_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(lambda: backend.vlm_requests >= 3, timeout=4.0)

    assert backend.program_cycles == []
    assert backend.find_prompts == ['the red mug', 'the red mug']
    assert any(
        status.get('active_visual_inspection_count', 0) >= 1 and
        status.get('visual_interrupt_count', 0) == 1
        for status in backend.statuses
    )

    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_held_active_inspection_cannot_overrun_model_wait_deadline(
        running_commander):
    backend, client = running_commander
    hold_inspector = threading.Event()
    backend.queue_decision(
        'wait', reason='one-second bounded wait', wait_seconds=1.0)
    backend.queue_active_inspection(
        'continue_current_command', release_event=hold_inspector)
    backend.queue_decision(
        'wait', reason='second wait after the first ends', wait_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(lambda: any(
        status.get('phase') == 'deferred' and
        status.get('status') == 'one-second bounded wait'
        for status in backend.statuses
    ))
    wait_started = time.monotonic()
    wait_until(lambda: backend.vlm_cancellations >= 1, timeout=2.0)
    elapsed = time.monotonic() - wait_started

    assert elapsed < 1.8
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    hold_inspector.set()
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_premature_finish_is_rejected_until_search_evidence_is_current(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('finish_not_found')
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    backend.queue_decision(
        'finish_not_found',
        reason='possible target remains in the current view',
        visual_observation='a target-like object may be visible',
        target_evidence='possible',
    )
    backend.queue_decision(
        'finish_not_found', reason='bounded search completed without a match')
    backend.release_program.set()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert not wrapped.result.found
    assert wrapped.result.outcome == \
        LookForObject.Result.OUTCOME_NOT_FOUND
    assert backend.program_cycles == [1]
    assert backend.vlm_requests == 4


def test_registry_revision_discards_inflight_model_decision(
        running_commander):
    backend, client = running_commander
    release_model = threading.Event()
    backend.queue_decision(
        'explore_and_record',
        exploration_cycles=1,
        release_event=release_model,
    )
    goal_handle = send_mission(client)
    wait_until(lambda: backend.vlm_requests == 1)

    backend.make_match_available()
    wrapped = wait_future(goal_handle.get_result_async())
    release_model.set()

    assert wrapped.result.found
    assert backend.vlm_cancellations == 1
    assert backend.program_cycles == []


def test_registry_change_during_camera_capture_forces_check_before_planning(
        running_commander):
    backend, client = running_commander
    backend.camera_timer.cancel()
    goal_handle = send_mission(client)
    wait_until(lambda: backend.count_subscribers(backend.image_topic) == 1)

    backend.make_match_available()
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.found
    assert backend.vlm_requests == 0
    assert backend.program_cycles == []


def test_vlm_inspects_jpeg_and_camera_churn_does_not_cancel_inference(
        running_commander):
    backend, client = running_commander
    release_model = threading.Event()
    backend.queue_decision(
        'explore_and_record',
        exploration_cycles=1,
        release_event=release_model,
        visual_observation='a red-colored area is unclear near the doorway',
        target_evidence='unclear',
    )
    goal_handle = send_mission(client)
    wait_until(lambda: backend.vlm_requests == 1)
    wait_until(lambda: backend.count_subscribers(backend.image_topic) == 0)
    sequence_during_inference = backend.camera_sequence

    time.sleep(0.15)
    assert backend.camera_sequence > sequence_during_inference
    assert backend.vlm_cancellations == 0

    release_model.set()
    wait_until(lambda: backend.program_cycles == [1])
    wait_until(lambda: any(
        status.get('target_evidence') == 'unclear'
        for status in backend.statuses
    ))
    assert len(backend.vlm_camera_jpegs) == 1
    assert 'camera_color_optical_frame' in backend.vlm_camera_labels[0]

    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_possible_planner_evidence_forces_registry_check_before_motion(
        running_commander):
    backend, client = running_commander
    backend.queue_decision(
        'explore_and_record',
        exploration_cycles=1,
        visual_observation='a red mug-like object may be visible',
        target_evidence='possible',
    )
    backend.queue_decision(
        'wait', reason='wait after the required check', wait_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(lambda: len(backend.find_prompts) >= 2)
    wait_until(lambda: backend.vlm_requests >= 2)

    assert backend.program_cycles == []
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_vlm_actively_monitors_and_can_interrupt_running_exploration(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    backend.queue_active_inspection('continue_current_command')
    backend.queue_active_inspection(
        'interrupt_and_replan',
        reason='a target-like red mug is visible; stop and verify',
        visual_observation='a red mug-like object is visible ahead',
        target_evidence='likely',
    )
    backend.queue_decision(
        'wait', reason='allow registry processing', wait_seconds=10.0)

    goal_handle = send_mission(client)
    result_future = goal_handle.get_result_async()
    deadline = time.monotonic() + 4.0
    while backend.vlm_requests < 2 and not result_future.done() and \
            time.monotonic() < deadline:
        time.sleep(0.01)
    assert backend.vlm_requests >= 2 or result_future.done(), (
        backend.program_cycles,
        [(
            item.get('phase'),
            item.get('status'),
            item.get('visual_subscription_active'),
            item.get('visual_inspection_count'),
        ) for item in backend.statuses[-8:]],
    )
    assert not result_future.done(), result_future.result().result.message

    assert backend.program_cycles == [1]
    assert backend.program_cancellations == 0

    wait_until(lambda: backend.program_cancellations == 1, timeout=4.0)
    wait_until(lambda: backend.vlm_requests >= 4, timeout=4.0)
    wait_until(lambda: any(
        status.get('active_visual_inspection_count', 0) >= 2 and
        status.get('visual_interrupt_count', 0) == 1
        for status in backend.statuses
    ))

    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(result_future)
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.found


def test_repeated_active_monitor_failure_stops_motion_and_aborts(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    backend.queue_model_failure('active monitor failure one')
    backend.queue_model_failure('active monitor failure two')
    backend.queue_model_failure('active monitor failure three')

    wrapped = wait_future(
        send_mission(client).get_result_async(), timeout=8.0)

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_FAILED
    assert 'active visual monitoring repeatedly failed' in \
        wrapped.result.message
    assert backend.program_cycles == [1]
    assert backend.program_cancellations == 1


def test_stale_camera_aborts_without_vlm_or_motion(running_commander):
    backend, client = running_commander
    backend.camera_timer.cancel()
    time.sleep(0.6)

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_FAILED
    assert backend.vlm_requests == 0
    assert backend.program_cycles == []


def test_transient_model_failure_defers_then_replans_without_blind_motion(
        running_commander):
    backend, client = running_commander
    backend.queue_model_failure()
    backend.queue_decision('wait', wait_seconds=0.05)
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    backend.queue_decision('finish_not_found')
    backend.release_program.set()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert not wrapped.result.found
    assert backend.vlm_requests == 4
    assert backend.program_cycles == [1]


def test_rejected_model_goal_defers_then_retries(running_commander):
    backend, client = running_commander
    with backend._lock:
        backend.vlm_rejections_remaining = 1
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    backend.queue_decision('finish_not_found')
    backend.release_program.set()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_NOT_FOUND
    assert backend.vlm_requests == 2
    assert backend.program_cycles == [1]


def test_parent_cancel_stops_owned_exploration(running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    goal_handle = send_mission(client)
    wait_until(lambda: backend.program_cycles == [1])

    cancel_response = wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert cancel_response.goals_canceling
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_CANCELED
    assert backend.program_cancellations == 1


def test_concurrent_goal_is_rejected_and_slot_reopens_after_cancel(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_and_record', exploration_cycles=1)
    first = send_mission(client)
    wait_until(lambda: backend.program_cycles == [1])

    competing = LookForObject.Goal()
    competing.prompt = 'a blue box'
    competing_handle = wait_future(client.send_goal_async(competing))
    assert not competing_handle.accepted

    wait_future(first.cancel_goal_async())
    canceled = wait_future(first.get_result_async())
    assert canceled.status == GoalStatus.STATUS_CANCELED

    backend.make_match_available()
    replacement = wait_future(send_mission(client).get_result_async())
    assert replacement.result.found


def test_planning_budget_returns_explicit_budget_outcome(running_commander):
    backend, client = running_commander
    backend.queue_decision('wait', wait_seconds=0.05)
    goal = LookForObject.Goal()
    goal.prompt = 'the red mug'
    goal.max_planning_steps = 1
    goal_handle = wait_future(client.send_goal_async(goal))
    assert goal_handle.accepted

    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert not wrapped.result.found
    assert wrapped.result.outcome == \
        LookForObject.Result.OUTCOME_BUDGET_EXHAUSTED
    assert wrapped.result.planning_steps == 1
    assert backend.program_cycles == []
