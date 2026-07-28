#!/usr/bin/env python3
"""Two-stage registry and VLM object-search action server."""

import math
from pathlib import Path
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from muto_command_layer.action import FindObject
from muto_command_layer.msg import ObjectMatch
from muto_vlm_socket.action import GenerateVlm
from muto_vlm_socket.msg import VlmContent
from object_search_protocol import (
    build_selection_schema,
    build_shortlist_prompt,
    build_visual_refinement_prompt,
    candidate_image_tag,
    format_judgement_log,
    parse_selection,
    SearchProtocolError,
)
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sam2_object_registry.srv import GetStoredObjects


class SearchCanceled(RuntimeError):
    """Raised internally when the parent FindObject goal is canceled."""


class SearchFailure(RuntimeError):
    """Raised for bounded dependency or registry failures."""


class ObjectSearchNode(Node):
    """Find registered objects using metadata first and JPEGs only if needed."""

    def __init__(self):
        super().__init__('object_search')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._callback_group = ReentrantCallbackGroup()
        self._registry_client = self.create_client(
            GetStoredObjects,
            self.registry_service,
            callback_group=self._callback_group,
        )
        self._vlm_client = ActionClient(
            self,
            GenerateVlm,
            self.vlm_action,
            callback_group=self._callback_group,
        )
        match_qos = QoSProfile(
            depth=32,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._match_publisher = self.create_publisher(
            ObjectMatch, self.match_topic, match_qos)

        self._state_lock = threading.Lock()
        self._busy = False
        self._active_vlm_goal = None
        self._action_server = ActionServer(
            self,
            FindObject,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'Object search ready: action={self.action_name} '
            f'registry={self.registry_service} vlm={self.vlm_action} '
            f'matches={self.match_topic}')

    def _declare_parameters(self):
        """Declare search endpoints, timeouts, and resource boundaries."""
        self.declare_parameter('action_name', '/find_object')
        self.declare_parameter(
            'registry_service', '/sam2/get_stored_objects')
        self.declare_parameter('vlm_action', '/vlm/generate')
        self.declare_parameter('match_topic', '/object_search/matches')
        self.declare_parameter('vlm_model', '')
        self.declare_parameter('registry_timeout', 3.0)
        self.declare_parameter('vlm_server_timeout', 5.0)
        self.declare_parameter('vlm_result_timeout', 180.0)
        self.declare_parameter('cancel_timeout', 2.0)
        self.declare_parameter('max_prompt_characters', 8192)
        self.declare_parameter('max_registry_objects', 256)
        self.declare_parameter('max_shortlist_size', 8)
        self.declare_parameter('max_description_characters', 4096)
        self.declare_parameter('max_candidate_jpeg_bytes', 8388608)
        self.declare_parameter('log_vlm_judgements', True)
        self.declare_parameter('max_log_description_characters', 240)
        self.declare_parameter('max_log_filtered_ids', 32)
        self.declare_parameter('require_all_candidate_images', True)

    def _read_parameters(self):
        """Read immutable search configuration once at startup."""
        self.action_name = self.get_parameter('action_name').value
        self.registry_service = self.get_parameter('registry_service').value
        self.vlm_action = self.get_parameter('vlm_action').value
        self.match_topic = self.get_parameter('match_topic').value
        self.vlm_model = self.get_parameter('vlm_model').value
        self.registry_timeout = self.get_parameter('registry_timeout').value
        self.vlm_server_timeout = self.get_parameter(
            'vlm_server_timeout').value
        self.vlm_result_timeout = self.get_parameter(
            'vlm_result_timeout').value
        self.cancel_timeout = self.get_parameter('cancel_timeout').value
        self.max_prompt_characters = self.get_parameter(
            'max_prompt_characters').value
        self.max_registry_objects = self.get_parameter(
            'max_registry_objects').value
        self.max_shortlist_size = self.get_parameter(
            'max_shortlist_size').value
        self.max_description_characters = self.get_parameter(
            'max_description_characters').value
        self.max_candidate_jpeg_bytes = self.get_parameter(
            'max_candidate_jpeg_bytes').value
        self.log_vlm_judgements = self.get_parameter(
            'log_vlm_judgements').value
        self.max_log_description_characters = self.get_parameter(
            'max_log_description_characters').value
        self.max_log_filtered_ids = self.get_parameter(
            'max_log_filtered_ids').value
        self.require_all_candidate_images = self.get_parameter(
            'require_all_candidate_images').value

    def _validate_parameters(self):
        """Reject invalid limits and endpoint names during startup."""
        for parameter_name in (
                'action_name', 'registry_service', 'vlm_action',
                'match_topic'):
            if not getattr(self, parameter_name):
                raise ValueError(f'{parameter_name} must not be empty')
        for parameter_name in (
                'registry_timeout', 'vlm_server_timeout',
                'vlm_result_timeout', 'cancel_timeout'):
            value = getattr(self, parameter_name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f'{parameter_name} must be finite and positive')
        for parameter_name in (
                'max_prompt_characters', 'max_registry_objects',
                'max_shortlist_size', 'max_description_characters',
                'max_candidate_jpeg_bytes',
                'max_log_description_characters', 'max_log_filtered_ids'):
            value = getattr(self, parameter_name)
            if not isinstance(value, int) or value <= 0:
                raise ValueError(
                    f'{parameter_name} must be a positive integer')
        if self.max_shortlist_size > self.max_registry_objects:
            raise ValueError(
                'max_shortlist_size must not exceed max_registry_objects')

    def _log_vlm_judgement(
            self, stage, selections, considered_ids):
        """Log one validated VLM decision without raw response content."""
        if not self.log_vlm_judgements:
            return
        payload = format_judgement_log(
            selections,
            considered_ids,
            self.max_log_description_characters,
            self.max_log_filtered_ids,
        )
        self.get_logger().info(f'VLM {stage} judgement: {payload}')

    def _goal_callback(self, goal_request):
        """Reject empty, oversized, or overlapping searches."""
        prompt = goal_request.prompt.strip()
        if not prompt or len(prompt) > self.max_prompt_characters:
            self.get_logger().warning(
                'Rejected FindObject goal with an empty or oversized prompt')
            return GoalResponse.REJECT
        with self._state_lock:
            if self._busy:
                self.get_logger().warning(
                    'Rejected FindObject goal because another search is active')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        """Accept parent cancellation and forward it to the child VLM goal."""
        self._cancel_active_vlm_goal()
        return CancelResponse.ACCEPT

    def _cancel_active_vlm_goal(self):
        """Best-effort cancellation of the currently active VLM request."""
        with self._state_lock:
            vlm_goal = self._active_vlm_goal
        if vlm_goal is not None:
            try:
                vlm_goal.cancel_goal_async()
            except Exception as error:  # noqa: B902
                self.get_logger().error(
                    'Failed to forward VLM cancellation: '
                    f'{type(error).__name__}')

    @staticmethod
    def _publish_feedback(goal_handle, phase, status, candidate_count):
        """Publish non-sensitive search progress."""
        feedback = FindObject.Feedback()
        feedback.phase = phase
        feedback.status = status
        feedback.candidate_count = candidate_count
        goal_handle.publish_feedback(feedback)

    def _wait_for_endpoint(
            self, ready_function, goal_handle, timeout, endpoint_name):
        """Poll a ROS endpoint without blocking executor callback threads."""
        deadline = time.monotonic() + timeout
        while not ready_function():
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise SearchFailure(f'{endpoint_name} is unavailable')
            time.sleep(0.05)

    def _wait_for_future(
            self, future, goal_handle, timeout, operation_name):
        """Wait for a ROS future while preserving cancellation and timeout."""
        deadline = time.monotonic() + timeout
        while not future.done():
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise SearchFailure(f'{operation_name} timed out')
            time.sleep(0.05)
        return future.result()

    @staticmethod
    def _check_parent_state(goal_handle):
        """Convert action cancellation and shutdown into internal exceptions."""
        if goal_handle.is_cancel_requested:
            raise SearchCanceled()
        if not rclpy.ok():
            raise SearchFailure('ROS context is shutting down')

    def _query_registry(self, goal_handle):
        """Fetch a deterministic snapshot of every registered object."""
        self._wait_for_endpoint(
            self._registry_client.service_is_ready,
            goal_handle,
            self.registry_timeout,
            'object registry service',
        )
        request = GetStoredObjects.Request()
        request.name = ''
        request.label = ''
        response = self._wait_for_future(
            self._registry_client.call_async(request),
            goal_handle,
            self.registry_timeout,
            'object registry query',
        )
        registry = response.result
        if len(registry.objects) > self.max_registry_objects:
            raise SearchFailure(
                'object registry exceeds max_registry_objects')
        objects = sorted(registry.objects, key=lambda item: item.name)
        names = [item.name for item in objects]
        if any(not name for name in names) or len(names) != len(set(names)):
            raise SearchFailure(
                'object registry contains empty or duplicate IDs')
        return registry.header, objects

    def _call_vlm(self, content, response_json_schema, goal_handle):
        """Send one child action goal and return its successful result."""
        self._wait_for_endpoint(
            self._vlm_client.server_is_ready,
            goal_handle,
            self.vlm_server_timeout,
            'VLM action server',
        )
        child_goal = GenerateVlm.Goal()
        child_goal.content = content
        child_goal.model = self.vlm_model
        child_goal.response_json_schema = response_json_schema
        send_future = self._vlm_client.send_goal_async(child_goal)
        try:
            child_handle = self._wait_for_future(
                send_future,
                goal_handle,
                self.vlm_server_timeout,
                'VLM goal dispatch',
            )
        except SearchCanceled:
            self._cancel_vlm_when_available(send_future)
            raise
        if not child_handle.accepted:
            raise SearchFailure('VLM action server rejected the request')
        with self._state_lock:
            self._active_vlm_goal = child_handle
        try:
            wrapped_result = self._wait_for_future(
                child_handle.get_result_async(),
                goal_handle,
                self.vlm_result_timeout,
                'VLM inference',
            )
        except (SearchCanceled, SearchFailure):
            self._cancel_active_vlm_goal()
            raise
        finally:
            with self._state_lock:
                self._active_vlm_goal = None
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise SearchFailure('VLM child action did not succeed')
        if not wrapped_result.result.success:
            message = wrapped_result.result.error_message.strip()
            raise SearchFailure(message or 'VLM child action failed')
        return wrapped_result.result

    def _cancel_vlm_when_available(self, send_future):
        """Avoid an orphan VLM goal when cancellation races goal acceptance."""
        deadline = time.monotonic() + self.cancel_timeout
        while not send_future.done() and rclpy.ok() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if send_future.done():
            try:
                child_handle = send_future.result()
                if child_handle.accepted:
                    child_handle.cancel_goal_async()
            except Exception:
                pass

    @staticmethod
    def _text_content(text):
        """Create one VLM text content part."""
        part = VlmContent()
        part.type = VlmContent.TYPE_TEXT
        part.text = text
        return part

    @staticmethod
    def _jpeg_content(jpeg_data):
        """Create one VLM JPEG content part."""
        part = VlmContent()
        part.type = VlmContent.TYPE_JPEG
        part.jpeg_data = jpeg_data
        return part

    def _read_candidate_jpeg(self, stored_object):
        """Read one registry JPEG with a strict upper bound and magic check."""
        if not stored_object.image_path:
            raise SearchFailure(
                f'candidate {stored_object.name} has no stored image')
        path = Path(stored_object.image_path).expanduser()
        try:
            with path.open('rb') as image_file:
                jpeg_data = image_file.read(
                    self.max_candidate_jpeg_bytes + 1)
        except OSError as error:
            raise SearchFailure(
                f'candidate {stored_object.name} image is unavailable') from error
        if len(jpeg_data) > self.max_candidate_jpeg_bytes:
            raise SearchFailure(
                f'candidate {stored_object.name} image exceeds size limit')
        if len(jpeg_data) < 4 or not jpeg_data.startswith(b'\xff\xd8') or \
                not jpeg_data.endswith(b'\xff\xd9'):
            raise SearchFailure(
                f'candidate {stored_object.name} image is not a complete JPEG')
        return jpeg_data

    def _build_visual_content(self, prompt, shortlist, objects_by_id):
        """Pack visual instructions followed by exact ID-tag/JPEG pairs."""
        included = []
        images = []
        for selection in shortlist:
            stored_object = objects_by_id[selection.object_id]
            try:
                jpeg_data = self._read_candidate_jpeg(stored_object)
            except SearchFailure as error:
                if self.require_all_candidate_images:
                    raise
                self.get_logger().warning(str(error))
                continue
            included.append(selection.object_id)
            images.append(jpeg_data)
        if not included:
            raise SearchFailure(
                'no shortlisted candidate has a usable stored image')

        content = [self._text_content(
            build_visual_refinement_prompt(prompt, included))]
        for object_id, jpeg_data in zip(included, images):
            content.append(self._text_content(candidate_image_tag(object_id)))
            content.append(self._jpeg_content(jpeg_data))
        return content, included

    def _publish_matches(
            self, prompt, selections, objects_by_id, registry_header):
        """Publish one correlated ObjectMatch message per final selection."""
        query_id = uuid.uuid4().hex
        total = len(selections)
        stamp = self.get_clock().now().to_msg()
        messages = []
        for index, selection in enumerate(selections):
            stored_object = objects_by_id[selection.object_id]
            message = ObjectMatch()
            message.header.stamp = stamp
            message.header.frame_id = registry_header.frame_id
            message.query_id = query_id
            message.rank = index + 1
            message.total = total
            message.object_id = selection.object_id
            message.label = stored_object.label
            message.description = selection.description
            self._match_publisher.publish(message)
            messages.append(message)
        self.get_logger().info(
            f'Object search completed: query_id={query_id} '
            f'matches={total} prompt_characters={len(prompt)}')
        return messages

    def _execute_callback(self, goal_handle):
        """Run metadata shortlisting and conditional visual refinement."""
        result = FindObject.Result()
        prompt = goal_handle.request.prompt.strip()
        try:
            self._publish_feedback(
                goal_handle,
                FindObject.Feedback.PHASE_REGISTRY_LOOKUP,
                'reading object registry',
                0,
            )
            registry_header, objects = self._query_registry(goal_handle)
            if not objects:
                result.success = True
                result.message = 'object registry is empty'
                goal_handle.succeed()
                return result
            objects_by_id = {item.name: item for item in objects}
            inventory = [{
                'id': item.name,
                'label': item.label,
                'class_id': item.class_id,
            } for item in objects]

            self._publish_feedback(
                goal_handle,
                FindObject.Feedback.PHASE_TEXT_SHORTLIST,
                'asking VLM for metadata shortlist',
                len(objects),
            )
            shortlist_prompt = build_shortlist_prompt(
                prompt, inventory, self.max_shortlist_size)
            shortlist_schema = build_selection_schema(
                'candidates',
                list(objects_by_id),
                self.max_shortlist_size,
                self.max_description_characters,
            )
            shortlist_result = self._call_vlm(
                [self._text_content(shortlist_prompt)],
                shortlist_schema,
                goal_handle,
            )
            shortlist = parse_selection(
                shortlist_result.response_text,
                'candidates',
                list(objects_by_id),
                self.max_shortlist_size,
                self.max_description_characters,
            )
            self._log_vlm_judgement(
                'metadata shortlist', shortlist, list(objects_by_id))

            final_selections = shortlist
            if len(shortlist) > 1:
                self._publish_feedback(
                    goal_handle,
                    FindObject.Feedback.PHASE_VISUAL_REFINEMENT,
                    'comparing shortlisted registry JPEGs',
                    len(shortlist),
                )
                visual_content, visual_ids = self._build_visual_content(
                    prompt, shortlist, objects_by_id)
                visual_schema = build_selection_schema(
                    'matches',
                    visual_ids,
                    len(visual_ids),
                    self.max_description_characters,
                )
                visual_result = self._call_vlm(
                    visual_content, visual_schema, goal_handle)
                final_selections = parse_selection(
                    visual_result.response_text,
                    'matches',
                    visual_ids,
                    len(visual_ids),
                    self.max_description_characters,
                )
                self._log_vlm_judgement(
                    'visual target filtering',
                    final_selections,
                    visual_ids,
                )
            elif self.log_vlm_judgements:
                reason = (
                    'no metadata candidates'
                    if not shortlist else 'one metadata candidate'
                )
                self.get_logger().info(
                    f'VLM visual target filtering skipped: {reason}')

            self._publish_feedback(
                goal_handle,
                FindObject.Feedback.PHASE_PUBLISHING,
                'publishing final object matches',
                len(final_selections),
            )
            result.matches = self._publish_matches(
                prompt,
                final_selections,
                objects_by_id,
                registry_header,
            )
            result.success = True
            if result.matches:
                result.message = f'found {len(result.matches)} object match(es)'
            else:
                result.message = 'no registered object matched the prompt'
            goal_handle.succeed()
            return result
        except SearchCanceled:
            result.success = False
            result.message = 'object search canceled'
            goal_handle.canceled()
            return result
        except (SearchFailure, SearchProtocolError) as error:
            result.success = False
            result.message = str(error)
            goal_handle.abort()
            self.get_logger().warning(
                f'Object search aborted: {error}')
            return result
        except Exception as error:  # noqa: B902
            result.success = False
            result.message = 'internal object-search error'
            goal_handle.abort()
            self.get_logger().error(
                f'Internal object-search error: {type(error).__name__}')
            return result
        finally:
            self._cancel_active_vlm_goal()
            with self._state_lock:
                self._active_vlm_goal = None
                self._busy = False


def main(args=None):
    """Run object search with enough executor threads for nested actions."""
    rclpy.init(args=args)
    node = ObjectSearchNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_active_vlm_goal()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
