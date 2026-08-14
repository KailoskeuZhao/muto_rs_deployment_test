"""Integration tests for the persistent model-supervised object search."""

from collections import deque
import json
import math
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
    GoToObject,
    LookForObject,
)
from muto_command_layer.msg import ObjectMatch, VisibilityPointOfInterest
from muto_command_layer.srv import GetVisibilityCoverage
from muto_vlm_socket.action import GenerateVlm
from muto_vlm_socket.msg import VlmContent
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
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
from sam2_object_registry.msg import (
    DetectedObjectArray,
    StoredObject,
    StoredObjectArray,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger


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
        wait_seconds=0.0, exploration_seconds=0.0,
        rotation_radians=0.0, observation_seconds=0.0,
        visual_observation='synthetic camera view is clear',
        target_evidence='not_visible'):
    return json.dumps({
        'decision': decision,
        'reason': reason,
        'wait_seconds': wait_seconds,
        'exploration_seconds': exploration_seconds,
        'rotation_radians': rotation_radians,
        'observation_seconds': observation_seconds,
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
        self.explore_action = f'{prefix}/explore_frontier'
        self.go_action = f'{prefix}/go_to_object'
        self.navigate_action = f'{prefix}/navigate_to_pose'
        self.visibility_service = f'{prefix}/visibility_coverage'
        self.spin_action = f'{prefix}/spin'
        self.odom_topic = f'{prefix}/odometry/filtered'
        self.checkpoint_service = f'{prefix}/save_stored_objects'
        self.detection_heartbeat_topic = f'{prefix}/detection_heartbeat'
        self.detections_topic = f'{prefix}/detections'
        self.registry_topic = f'{prefix}/stored_objects'
        self.image_topic = f'{prefix}/camera/color/image_raw'
        self.commander_action = f'{prefix}/look_for_object'
        self.status_topic = f'{prefix}/model_commander_status'
        self.decision_topic = f'{prefix}/model_commander_decisions'
        self.inspected_image_topic = f'{prefix}/inspected_image'
        self.bag_event_topic = f'{prefix}/command_bag_lifecycle'
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._decisions = deque()
        self.match_available = False
        self.unrelated_object_available = False
        self.ambiguous_matches_available = False
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
        self.find_candidate_id_sets = []
        self.program_cycles = []
        self.primitive_names = []
        self.program_cancellations = 0
        self.approach_object_ids = []
        self.poi_navigation_goals = []
        self.visibility_queries = 0
        self.visibility_observation_counts = []
        self.checkpoint_requests = 0
        self.statuses = []
        self.trace_events = []
        self.inspected_images = []
        self.lifecycle_events = []
        self.camera_sequence = 0
        self._yaw = 0.0
        self._x = 0.0
        self.simulate_explore_motion = True
        self.simulate_navigate_motion = True

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
        self.explore_server = ActionServer(
            self,
            ExploreAndRecord,
            self.explore_action,
            execute_callback=self.execute_explore,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
        self.go_server = ActionServer(
            self,
            GoToObject,
            self.go_action,
            execute_callback=self.execute_go_to_object,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
        self.navigate_server = ActionServer(
            self,
            NavigateToPose,
            self.navigate_action,
            execute_callback=self.execute_navigate,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
        self.spin_server = ActionServer(
            self,
            Spin,
            self.spin_action,
            execute_callback=self.execute_spin,
            cancel_callback=self.accept_cancel,
            callback_group=self._callback_group,
        )
        self.visibility_server = self.create_service(
            GetVisibilityCoverage,
            self.visibility_service,
            self.handle_visibility_coverage,
            callback_group=self._callback_group,
        )
        self.checkpoint_server = self.create_service(
            Trigger,
            self.checkpoint_service,
            self.handle_checkpoint,
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
        self.odom_publisher = self.create_publisher(
            Odometry, self.odom_topic, 10)
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)
        self.detection_heartbeat_publisher = self.create_publisher(
            Header, self.detection_heartbeat_topic, 10)
        self.detections_publisher = self.create_publisher(
            DetectedObjectArray, self.detections_topic, 10)
        self.detection_heartbeat_timer = self.create_timer(
            0.02,
            lambda: self.detection_heartbeat_publisher.publish(Header()),
        )
        self.status_subscription = self.create_subscription(
            String, self.status_topic, self.status_callback, qos)
        self.lifecycle_subscription = self.create_subscription(
            String,
            self.bag_event_topic,
            lambda message: self.lifecycle_events.append(
                json.loads(message.data)),
            qos,
        )
        trace_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.trace_subscription = self.create_subscription(
            String, self.decision_topic, self.trace_callback, trace_qos)
        self.inspected_image_subscription = self.create_subscription(
            CompressedImage,
            self.inspected_image_topic,
            self.inspected_image_callback,
            trace_qos,
        )
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
            wait_seconds=0.0, exploration_seconds=0.0,
            rotation_radians=0.0, observation_seconds=0.0,
            release_event=None,
            visual_observation='synthetic camera view is clear',
            target_evidence='not_visible'):
        with self._lock:
            self._decisions.append((
                decision_response(
                    decision,
                    reason=reason,
                    wait_seconds=wait_seconds,
                    exploration_seconds=exploration_seconds,
                    rotation_radians=rotation_radians,
                    observation_seconds=observation_seconds,
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
        candidate_ids = list(goal_handle.request.candidate_ids)
        self.find_candidate_id_sets.append(candidate_ids)
        with self._lock:
            match_available = self.match_available
            ambiguous_available = self.ambiguous_matches_available
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
        elif ambiguous_available and not candidate_ids:
            first = ObjectMatch()
            first.object_id = 'red_mug_2'
            first.label = 'cup'
            first.description = 'fake matching red mug'
            second = ObjectMatch()
            second.object_id = 'red_mug_3'
            second.label = 'cup'
            second.description = 'fake second red mug candidate'
            result.matches = [first, second]
            result.message = 'found two matching static objects'
        elif ambiguous_available and candidate_ids == ['red_mug_2',
                                                       'red_mug_3']:
            match = ObjectMatch()
            match.object_id = 'red_mug_3'
            match.label = 'cup'
            match.description = 'stored JPEG best matches the requested mug'
            result.matches = [match]
            result.message = 'refined to one matching static object'
        else:
            result.message = 'no registered object matched'
        goal_handle.succeed()
        return result

    def execute_go_to_object(self, goal_handle):
        self.approach_object_ids.append(goal_handle.request.object_id)
        result = GoToObject.Result()
        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = 'fake object approach canceled'
            goal_handle.canceled()
            return result
        result.success = True
        result.message = 'fake object approach completed'
        goal_handle.succeed()
        return result

    def execute_navigate(self, goal_handle):
        pose = goal_handle.request.pose.pose
        self.poi_navigation_goals.append((
            round(float(pose.position.x), 3),
            round(float(pose.position.y), 3),
        ))
        if self.simulate_navigate_motion:
            while not goal_handle.is_cancel_requested and \
                    not self.release_program.is_set():
                time.sleep(0.01)
        else:
            while not goal_handle.is_cancel_requested:
                time.sleep(0.01)
        result = NavigateToPose.Result()
        if goal_handle.is_cancel_requested:
            self.program_cancellations += 1
            goal_handle.canceled()
            return result
        with self._lock:
            self._x = float(pose.position.x)
        self.publish_odometry()
        goal_handle.succeed()
        return result

    def execute_spin(self, goal_handle):
        self.primitive_names.append('rotate')
        self.program_cycles.append(1)
        target = float(goal_handle.request.target_yaw)
        feedback = Spin.Feedback()
        while not goal_handle.is_cancel_requested and \
                not self.release_program.is_set():
            with self._lock:
                self._yaw += math.copysign(0.03, target)
                traveled = abs(self._yaw)
            feedback.angular_distance_traveled = min(abs(target), traveled)
            goal_handle.publish_feedback(feedback)
            self.publish_odometry()
            if traveled >= abs(target):
                break
            time.sleep(0.01)
        result = Spin.Result()
        if goal_handle.is_cancel_requested:
            self.program_cancellations += 1
            goal_handle.canceled()
            return result
        with self._lock:
            self._yaw = target
        self.publish_odometry()
        goal_handle.succeed()
        return result

    def handle_visibility_coverage(self, request, response):
        self.visibility_queries += 1
        self.visibility_observation_counts.append(len(request.observations))
        response.success = True
        response.message = 'fake visibility coverage calculated'
        response.state.header.frame_id = 'map'
        response.state.complete = False
        response.state.applied_observations = len(request.observations)
        response.state.rejected_observations = 0
        response.state.candidate_count = 3
        response.state.target_free_cells = 100
        response.state.coverable_free_cells = 80
        response.state.covered_free_cells = 24
        response.state.target_boundary_cells = 40
        response.state.coverable_boundary_cells = 35
        response.state.covered_boundary_cells = 10
        response.state.map_coverage_ratio = 0.24
        response.state.observable_coverage_ratio = 0.30
        response.state.boundary_coverage_ratio = 0.286
        response.state.combined_coverage_ratio = 0.286
        response.state.robot_pose.header.frame_id = 'map'
        poi = VisibilityPointOfInterest()
        poi.candidate_index = 7
        poi.pose.header.frame_id = 'map'
        poi.pose.pose.position.x = 1.2
        poi.pose.pose.position.y = 0.4
        poi.pose.pose.orientation.w = 1.0
        poi.cell_x = 12
        poi.cell_y = 4
        poi.new_free_cells = 20
        poi.new_boundary_cells = 8
        poi.path_length_m = 1.2
        poi.weighted_gain = 36.0
        poi.score = 1.44
        response.state.points_of_interest = [poi]
        _ = request
        return response

    def execute_explore(self, goal_handle):
        self.primitive_names.append('explore_frontier')
        return self.execute_program(goal_handle)

    def publish_odometry(self):
        with self._lock:
            yaw = self._yaw
            x = self._x
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.odom_publisher.publish(odom)

    def handle_checkpoint(self, _request, response):
        self.primitive_names.append('checkpoint_registry')
        self.checkpoint_requests += 1
        response.success = True
        response.message = 'fake registry checkpoint completed'
        return response

    def execute_program(self, goal_handle):
        self.program_cycles.append(1)
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
        if self.simulate_explore_motion:
            with self._lock:
                self._x += 0.6
            self.publish_odometry()
            time.sleep(0.05)
        result.success = True
        result.message = 'fake bounded exploration completed'
        result.completed_cycles = 1
        goal_handle.succeed()
        return result

    def publish_registry(self):
        message = StoredObjectArray()
        message.header.frame_id = 'map'
        with self._lock:
            match_available = self.match_available
            unrelated_available = self.unrelated_object_available
            ambiguous_available = self.ambiguous_matches_available
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
        if ambiguous_available:
            for name in ('red_mug_2', 'red_mug_3'):
                stored = StoredObject()
                stored.name = name
                stored.label = 'cup'
                stored.class_id = 2
                stored.image_path = f'/tmp/{name}.jpg'
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

    def make_ambiguous_matches_available(self):
        with self._lock:
            self.ambiguous_matches_available = True
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

    def trace_callback(self, message):
        self.trace_events.append(json.loads(message.data))

    def inspected_image_callback(self, message):
        self.inspected_images.append(bytes(message.data))


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
            '-p', f'go_to_object_action:={backend.go_action}',
            '-p', f'explore_frontier_action:={backend.explore_action}',
            '-p', f'navigate_to_pose_action:={backend.navigate_action}',
            '-p', ('visibility_coverage_service:='
                   f'{backend.visibility_service}'),
            '-p', f'spin_action:={backend.spin_action}',
            '-p', f'registry_save_service:={backend.checkpoint_service}',
            '-p', ('detection_heartbeat_topic:='
                   f'{backend.detection_heartbeat_topic}'),
            '-p', f'registry_topic:={backend.registry_topic}',
            '-p', f'robot_pose_topic:={backend.odom_topic}',
            '-p', f'visual_observation_topic:={backend.image_topic}',
            '-p', f'status_topic:={backend.status_topic}',
            '-p', f'decision_event_topic:={backend.decision_topic}',
            '-p', f'inspected_image_topic:={backend.inspected_image_topic}',
            '-p', f'command_bag_event_topic:={backend.bag_event_topic}',
            '-p', 'command_bag_enabled:=false',
            '-p', 'endpoint_timeout:=1.0',
            '-p', 'vlm_result_timeout:=3.0',
            '-p', 'find_result_timeout:=3.0',
            '-p', 'child_stop_timeout:=2.0',
            '-p', 'default_max_duration:=20.0',
            '-p', 'default_max_planning_steps:=12',
            '-p', 'rotate_executable_yaw_velocity:=0.19',
            '-p', 'rotate_goal_tolerance:=0.04',
            '-p', 'planner_retry_initial_delay:=0.05',
            '-p', 'planner_retry_max_delay:=0.1',
            '-p', 'command_retry_initial_delay:=0.05',
            '-p', 'command_retry_max_delay:=0.1',
            '-p', 'visual_observation_timeout:=1.0',
            '-p', 'visual_observation_max_age:=0.5',
            '-p', 'active_inspection_period:=0.5',
            '-p', 'active_inspection_timeout:=2.0',
            '-p', 'active_inspection_max_decision_age:=3.0',
            '-p', 'minimum_no_match_observations:=1',
            '-p', 'minimum_no_match_rotation_radians:=1.0',
            '-p', 'minimum_explore_progress_distance_m:=0.1',
            '-p', 'navigation_progress_distance_m:=0.05',
            '-p', 'navigation_no_progress_timeout:=0.3',
            '-p', 'max_consecutive_navigation_no_progress:=2',
            '-p', 'minimum_no_match_travel_distance_m:=0.5',
            '-p', 'observation_min_detection_frames:=1',
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


def send_mission(
        client, prompt='the red mug',
        completion_mode=LookForObject.Goal.COMPLETION_REPORT_OBJECT):
    goal = LookForObject.Goal()
    goal.prompt = prompt
    goal.completion_mode = completion_mode
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
    assert not wrapped.result.approached
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_FOUND
    assert [item.object_id for item in wrapped.result.matches] == ['red_mug_2']
    assert backend.vlm_requests == 0
    assert backend.program_cycles == []
    wait_until(lambda: any(
        event['event'] == 'succeeded'
        for event in backend.lifecycle_events
    ))
    assert [event['event'] for event in backend.lifecycle_events] == [
        'mission_started', 'succeeded']
    assert backend.lifecycle_events[-1]['matched_object_ids'] == ['red_mug_2']
    wait_until(lambda: any(
        event.get('event') == 'mission_result'
        for event in backend.trace_events
    ))
    mission_result = next(
        event for event in backend.trace_events
        if event.get('event') == 'mission_result'
    )
    assert mission_result['outcome'] == 'found'
    assert mission_result['matched_object_ids'] == ['red_mug_2']


def test_approach_completion_is_planned_after_exact_registry_confirmation(
        running_commander):
    backend, client = running_commander
    backend.make_match_available()
    backend.queue_decision(
        'approach_object',
        reason='the exact confirmed target can now be approached',
    )

    goal_handle = send_mission(
        client,
        completion_mode=LookForObject.Goal.COMPLETION_APPROACH_OBJECT,
    )
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert wrapped.result.found
    assert wrapped.result.approached
    assert backend.approach_object_ids == ['red_mug_2']
    assert backend.vlm_requests == 1
    assert backend.program_cycles == []
    approach_event = next(
        event for event in backend.trace_events
        if event.get('event') == 'command_result' and
        event.get('command') == 'approach_object'
    )
    assert approach_event['object_id'] == 'red_mug_2'
    assert approach_event['outcome'] == 'completed'


def test_approach_can_refine_multiple_confirmed_registry_candidates(
        running_commander):
    backend, client = running_commander
    backend.make_ambiguous_matches_available()
    backend.queue_decision(
        'refine_registry_selection',
        reason='use stored registry crop images to choose one mug',
    )
    backend.queue_decision(
        'approach_object',
        reason='the registry refinement selected one exact target',
    )

    goal_handle = send_mission(
        client,
        completion_mode=LookForObject.Goal.COMPLETION_APPROACH_OBJECT,
    )
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert wrapped.result.found
    assert wrapped.result.approached
    assert [item.object_id for item in wrapped.result.matches] == [
        'red_mug_3']
    assert backend.approach_object_ids == ['red_mug_3']
    assert backend.find_candidate_id_sets == [
        [],
        ['red_mug_2', 'red_mug_3'],
    ]
    refine_event = next(
        event for event in backend.trace_events
        if event.get('event') == 'command_result' and
        event.get('command') == 'refine_registry_selection'
    )
    assert refine_event['candidate_object_ids'] == ['red_mug_2', 'red_mug_3']
    assert refine_event['matched_object_ids'] == ['red_mug_3']


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


def test_registry_churn_is_coalesced_and_search_motion_still_starts(
        running_commander):
    backend, client = running_commander
    release_find = threading.Event()
    backend.make_match_available()
    backend.find_release_event = release_find
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(backend.find_started.is_set)
    backend.clear_match()
    release_find.set()

    wait_until(lambda: backend.program_cycles == [1])
    wait_until(lambda: len(backend.find_prompts) >= 2)
    wait_until(lambda: any(
        event.get('event') == 'background_registry_request'
        for event in backend.trace_events
    ))
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.found
    assert backend.find_prompts[:2] == ['the red mug', 'the red mug']


def test_registry_change_cancels_bounded_step_and_returns_match(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
    goal_handle = send_mission(client)
    wait_until(lambda: backend.program_cycles == [1])

    backend.make_match_available()
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.found
    assert backend.program_cancellations == 1
    assert len(backend.find_prompts) >= 2
    assert set(backend.find_prompts) == {'the red mug'}


def test_commander_schedules_rotate_observe_and_checkpoint_independently(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('rotate', rotation_radians=1.57)
    backend.queue_decision('observe', observation_seconds=0.3)
    backend.queue_decision('checkpoint_registry')
    backend.queue_decision(
        'wait', reason='await more evidence', wait_seconds=10.0)
    backend.release_program.set()

    assert backend.detection_heartbeat_publisher.get_subscription_count() == 0
    assert backend.detections_publisher.get_subscription_count() == 0
    goal_handle = send_mission(client)
    wait_until(
        lambda: backend.detection_heartbeat_publisher
        .get_subscription_count() == 1)
    assert backend.detections_publisher.get_subscription_count() == 0
    wait_until(lambda: backend.primitive_names == [
        'rotate', 'checkpoint_registry'])
    wait_until(lambda: backend.vlm_requests == 4)
    wait_until(
        lambda: backend.detection_heartbeat_publisher
        .get_subscription_count() == 0)
    assert backend.detections_publisher.get_subscription_count() == 0
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert not wrapped.result.found
    assert backend.primitive_names == ['rotate', 'checkpoint_registry']
    assert any(
        event.get('command') == 'observe' and
        event.get('outcome') == 'completed'
        for event in backend.trace_events
    )
    event_names = [event['event'] for event in backend.trace_events]
    assert 'planning_request' in event_names
    assert 'planning_decision' in event_names
    assert 'command_result' in event_names
    assert backend.inspected_images
    assert all(image.startswith(b'\xff\xd8') for image in
               backend.inspected_images)


def test_commander_uses_visibility_poi_helper_and_records_context(
        running_commander):
    backend, client = running_commander
    backend.queue_decision(
        'explore_frontier',
        reason='first establish actual frontier-search progress',
        exploration_seconds=10.0,
    )
    backend.queue_decision(
        'navigate_to_observation_poi',
        reason='coverage helper reports a useful observation point',
    )
    backend.queue_decision(
        'wait', reason='hold after reaching observation point',
        wait_seconds=10.0)
    backend.release_program.set()

    goal_handle = send_mission(client)
    wait_until(lambda: backend.poi_navigation_goals == [(1.2, 0.4)])
    wait_until(lambda: backend.vlm_requests >= 2)
    wait_until(lambda: any(
        event.get('event') == 'command_result' and
        event.get('command') == 'navigate_to_observation_poi'
        for event in backend.trace_events))

    planning_requests = [
        event for event in backend.trace_events
        if event.get('event') == 'planning_request'
    ]
    assert not planning_requests[0]['state']['visibility_coverage'][
        'available']
    assert 'before frontier exploration' in planning_requests[0][
        'state']['visibility_coverage']['message']
    planning_request = next(
        event for event in planning_requests[1:]
        if event['state']['visibility_coverage'].get('available')
    )
    coverage = planning_request['state']['visibility_coverage']
    assert coverage['available']
    assert coverage['points_of_interest'][0]['candidate_index'] == 7
    assert planning_request['state']['perception_readiness'][
        'camera_snapshot_seen']
    assert 'navigation_health' in planning_request['state']
    result_event = next(
        event for event in backend.trace_events
        if event.get('event') == 'command_result' and
        event.get('command') == 'navigate_to_observation_poi'
    )
    assert result_event['outcome'] == 'completed'
    assert result_event['selected_poi']['candidate_index'] == 7
    assert result_event['visibility_coverage']['available']
    assert result_event['delta_pose']['distance_xy'] > 0.0
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert backend.visibility_queries >= 2


def test_repeated_stationary_poi_navigation_stops_the_mission(
        running_commander):
    backend, client = running_commander
    backend.simulate_navigate_motion = False
    backend.queue_decision(
        'explore_frontier', exploration_seconds=10.0,
        reason='establish mapped-space search progress')
    backend.queue_decision(
        'navigate_to_observation_poi', reason='inspect first mapped POI')
    backend.queue_decision(
        'navigate_to_observation_poi', reason='inspect second mapped POI')
    backend.release_program.set()

    wrapped = wait_future(send_mission(client).get_result_async(), timeout=8.0)

    assert wrapped.status == GoalStatus.STATUS_ABORTED
    assert not wrapped.result.success
    assert 'repeatedly made no physical progress' in wrapped.result.message
    stalled = [
        event for event in backend.trace_events
        if event.get('command') == 'navigate_to_observation_poi' and
        event.get('outcome') == 'no_spatial_progress'
    ]
    assert len(stalled) == 2
    assert len(backend.poi_navigation_goals) == 2
    assert backend.program_cancellations >= 2

def test_initial_planning_does_not_block_on_visibility_coverage(
        running_commander):
    backend, client = running_commander
    backend.queue_decision(
        'wait', reason='initial image and registry context are sufficient',
        wait_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(lambda: backend.vlm_requests == 1)

    first_request = next(
        event for event in backend.trace_events
        if event.get('event') == 'planning_request'
    )
    assert backend.visibility_queries == 0
    assert not first_request['state']['visibility_coverage']['available']
    assert 'before frontier exploration' in first_request[
        'state']['visibility_coverage']['message']

    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_completed_observation_is_replayed_into_later_coverage(
        running_commander):
    backend, client = running_commander
    backend.queue_decision(
        'explore_frontier', exploration_seconds=10.0,
        reason='expand known space before inspection planning')
    backend.queue_decision(
        'observe', observation_seconds=0.15,
        reason='collect detector-backed evidence at the current pose')
    backend.queue_decision(
        'navigate_to_observation_poi',
        reason='move to remaining uninspected mapped space')
    backend.queue_decision('wait', wait_seconds=10.0)
    backend.release_program.set()

    goal_handle = send_mission(client)
    wait_until(lambda: backend.poi_navigation_goals == [(1.2, 0.4)])
    wait_until(lambda: any(count >= 1 for count in
                           backend.visibility_observation_counts))

    planning_request = next(
        event for event in backend.trace_events
        if event.get('event') == 'planning_request' and
        event['state']['visibility_coverage'].get(
            'applied_observations', 0) >= 1
    )
    assert planning_request['state']['frontier_search'][
        'utility'] == 'productive'
    assert planning_request['state']['visibility_coverage'][
        'applied_observations'] == 1
    assert backend.visibility_observation_counts[-1] == 1

    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_repeated_transient_input_cycles_keep_commander_alive(
        running_commander):
    backend, client = running_commander
    for _ in range(6):
        backend.queue_decision('checkpoint_registry')
    backend.queue_decision(
        'wait', reason='hold after repeated snapshots', wait_seconds=10.0)

    first_goal = send_mission(client)
    wait_until(lambda: backend.checkpoint_requests == 6, timeout=12.0)
    wait_until(lambda: backend.vlm_requests >= 7, timeout=12.0)
    wait_future(first_goal.cancel_goal_async())
    first_result = wait_future(first_goal.get_result_async())
    assert first_result.status == GoalStatus.STATUS_CANCELED
    wait_until(
        lambda: backend.camera_publisher.get_subscription_count() == 0,
        timeout=3.0,
    )

    # A second accepted mission demonstrates that rapid create/spin/destroy
    # cycles did not crash the Humble executor or leave admission wedged.
    backend.queue_decision(
        'wait', reason='second mission proves liveness', wait_seconds=10.0)
    requests_before = backend.vlm_requests
    second_goal = send_mission(client)
    wait_until(lambda: backend.vlm_requests > requests_before)
    wait_future(second_goal.cancel_goal_async())
    second_result = wait_future(second_goal.get_result_async())
    assert second_result.status == GoalStatus.STATUS_CANCELED


def test_unresponsive_inspector_cancellation_still_stops_moving_child(
        running_commander):
    backend, client = running_commander
    hold_inspector = threading.Event()
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
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
        'wait', reason='two-second bounded wait', wait_seconds=2.0)
    backend.queue_active_inspection(
        'continue_current_command', release_event=hold_inspector)
    backend.queue_decision(
        'wait', reason='second wait after the first ends', wait_seconds=10.0)

    goal_handle = send_mission(client)
    wait_until(lambda: any(
        status.get('phase') == 'deferred' and
        status.get('status') == 'two-second bounded wait'
        for status in backend.statuses
    ))
    wait_started = time.monotonic()
    wait_until(lambda: backend.vlm_cancellations >= 1, timeout=3.0)
    elapsed = time.monotonic() - wait_started

    assert elapsed < 2.8
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())
    hold_inspector.set()
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_premature_finish_is_rejected_until_search_evidence_is_current(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('finish_not_found')
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
    backend.queue_decision('rotate', rotation_radians=1.57)
    backend.queue_decision('observe', observation_seconds=0.3)
    backend.queue_decision('checkpoint_registry')
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
    assert backend.program_cycles == [1, 1]
    assert backend.vlm_requests == 7


def test_stationary_frontier_timeout_does_not_count_as_search_travel(
        running_commander):
    backend, client = running_commander
    backend.simulate_explore_motion = False
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
    backend.queue_decision(
        'finish_not_found', reason='incorrectly treating elapsed time as travel')
    backend.queue_decision('wait', wait_seconds=10.0)
    backend.release_program.set()

    goal_handle = send_mission(client)
    result_future = goal_handle.get_result_async()
    wait_until(lambda: any(
        event.get('command') == 'explore_frontier' and
        event.get('outcome') == 'no_spatial_progress'
        for event in backend.trace_events
    ))
    wait_until(lambda: any(
        event.get('command') == 'finish_not_found' and
        event.get('outcome') == 'premature_finish_rejected'
        for event in backend.trace_events
    ))

    assert not result_future.done()
    wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(result_future)
    assert wrapped.status == GoalStatus.STATUS_CANCELED


def test_registry_revision_discards_inflight_model_decision(
        running_commander):
    backend, client = running_commander
    release_model = threading.Event()
    backend.queue_decision(
        'explore_frontier',
        exploration_seconds=10.0,
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
        'explore_frontier',
        exploration_seconds=10.0,
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
        'explore_frontier',
        exploration_seconds=10.0,
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
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
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
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
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
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
    backend.queue_decision('rotate', rotation_radians=1.57)
    backend.queue_decision('observe', observation_seconds=0.3)
    backend.queue_decision('checkpoint_registry')
    backend.queue_decision('finish_not_found')
    backend.release_program.set()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success
    assert not wrapped.result.found
    assert backend.vlm_requests == 7
    assert backend.program_cycles == [1, 1]


def test_rejected_model_goal_defers_then_retries(running_commander):
    backend, client = running_commander
    with backend._lock:
        backend.vlm_rejections_remaining = 1
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
    backend.queue_decision('rotate', rotation_radians=1.57)
    backend.queue_decision('observe', observation_seconds=0.3)
    backend.queue_decision('checkpoint_registry')
    backend.queue_decision('finish_not_found')
    backend.release_program.set()

    wrapped = wait_future(send_mission(client).get_result_async())

    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_NOT_FOUND
    assert backend.vlm_requests == 5
    assert backend.program_cycles == [1, 1]


def test_parent_cancel_stops_owned_exploration(running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
    goal_handle = send_mission(client)
    wait_until(lambda: backend.program_cycles == [1])

    cancel_response = wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert cancel_response.goals_canceling
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_CANCELED
    assert backend.program_cancellations == 1


def test_parent_cancel_stops_owned_rotation(running_commander):
    backend, client = running_commander
    backend.queue_decision('rotate', rotation_radians=1.57)
    goal_handle = send_mission(client)
    wait_until(lambda: backend.primitive_names == ['rotate'])

    cancel_response = wait_future(goal_handle.cancel_goal_async())
    wrapped = wait_future(goal_handle.get_result_async())

    assert cancel_response.goals_canceling
    assert wrapped.status == GoalStatus.STATUS_CANCELED
    assert wrapped.result.outcome == LookForObject.Result.OUTCOME_CANCELED
    assert backend.program_cancellations == 1


def test_concurrent_goal_is_rejected_and_slot_reopens_after_cancel(
        running_commander):
    backend, client = running_commander
    backend.queue_decision('explore_frontier', exploration_seconds=10.0)
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
