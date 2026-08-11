#!/usr/bin/env python3
"""Validated natural-language router for the typed Muto command layer."""

import json
import math
import threading
import time

from action_msgs.msg import GoalStatus
from muto_command_layer.action import (
    FindObject,
    GoToObject,
    LookForObject,
    NaturalLanguageCommand,
)
from muto_vlm_socket.action import GenerateVlm
from muto_vlm_socket.msg import VlmContent
from natural_language_command_protocol import (
    build_command_prompt,
    build_command_schema,
    CommandProtocolError,
    parse_command_intent,
    parse_explicit_local_command,
)
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CommandCanceled(RuntimeError):
    """Raised internally when the natural-language action is canceled."""


class CommandFailure(RuntimeError):
    """Raised for validated commands that cannot be dispatched safely."""


class NaturalLanguageCommandNode(Node):
    """Classify natural language and dispatch only whitelisted ROS commands."""

    def __init__(self):
        super().__init__('natural_language_command_router')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._callback_group = ReentrantCallbackGroup()
        self._vlm_client = ActionClient(
            self,
            GenerateVlm,
            self.vlm_action,
            callback_group=self._callback_group,
        )
        self._find_client = ActionClient(
            self,
            FindObject,
            self.find_object_action,
            callback_group=self._callback_group,
        )
        self._go_client = ActionClient(
            self,
            GoToObject,
            self.go_to_object_action,
            callback_group=self._callback_group,
        )
        self._model_commander_client = ActionClient(
            self,
            LookForObject,
            self.look_for_object_action,
            callback_group=self._callback_group,
        )
        self._explore_client = self.create_client(
            SetBool,
            self.explore_service,
            callback_group=self._callback_group,
        )
        self._save_map_client = self.create_client(
            SaveMap,
            self.save_map_service,
            callback_group=self._callback_group,
        )

        self._state_lock = threading.Lock()
        self._busy = False
        self._active_vlm_goal = None
        self._active_child_goal = None
        self._active_motion_goal = None
        self._active_motion_command = ''
        self._manual_exploration_active = False
        trace_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._decision_event_publisher = self.create_publisher(
            String, self.decision_event_topic, trace_qos)

        self._action_server = ActionServer(
            self,
            NaturalLanguageCommand,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'Natural-language command router ready: action={self.action_name} '
            f'vlm={self.vlm_action}')

    def _declare_parameters(self):
        self.declare_parameter('action_name', '/natural_language_command')
        self.declare_parameter('vlm_action', '/vlm/generate')
        self.declare_parameter('find_object_action', '/find_object')
        self.declare_parameter('look_for_object_action', '/look_for_object')
        self.declare_parameter('go_to_object_action', '/go_to_object')
        self.declare_parameter('explore_service', '/explore')
        self.declare_parameter('save_map_service', '/save_map')
        self.declare_parameter(
            'decision_event_topic',
            '/natural_language_command/decision_event')
        self.declare_parameter('vlm_model', 'gpt-5.3-codex-spark')
        self.declare_parameter('endpoint_timeout', 5.0)
        self.declare_parameter('vlm_result_timeout', 45.0)
        self.declare_parameter('find_result_timeout', 400.0)
        self.declare_parameter('save_map_result_timeout', 15.0)
        self.declare_parameter('cancel_timeout', 2.0)
        self.declare_parameter('max_query_characters', 4096)
        self.declare_parameter('max_object_query_characters', 1024)
        self.declare_parameter('max_map_name_characters', 128)

    def _read_parameters(self):
        for name in (
                'action_name', 'vlm_action', 'find_object_action',
                'go_to_object_action', 'look_for_object_action',
                'explore_service',
                'save_map_service',
                'decision_event_topic',
                'vlm_model',
                'endpoint_timeout', 'vlm_result_timeout',
                'find_result_timeout', 'save_map_result_timeout',
                'cancel_timeout',
                'max_query_characters', 'max_object_query_characters',
                'max_map_name_characters'):
            setattr(self, name, self.get_parameter(name).value)

    def _validate_parameters(self):
        for name in (
                'action_name', 'vlm_action', 'find_object_action',
                'go_to_object_action', 'look_for_object_action',
                'explore_service',
                'save_map_service', 'decision_event_topic'):
            if not getattr(self, name):
                raise ValueError(f'{name} must not be empty')
        for name in (
                'endpoint_timeout', 'vlm_result_timeout',
                'find_result_timeout', 'save_map_result_timeout',
                'cancel_timeout'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or \
                    not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        for name in (
                'max_query_characters', 'max_object_query_characters',
                'max_map_name_characters'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f'{name} must be a positive integer')

    def _goal_callback(self, goal_request):
        query = goal_request.query.strip()
        if not query or len(query) > self.max_query_characters:
            self.get_logger().warning(
                'Rejected natural-language command with empty/oversized query')
            return GoalResponse.REJECT
        with self._state_lock:
            if self._busy:
                self.get_logger().warning(
                    'Rejected natural-language command while router is busy')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _publish_decision_event(
            self, event, goal_handle, query, source, result, intent=None):
        payload = {
            'schema': 'muto_natural_language_decision_v1',
            'event': event,
            'goal_id': bytes(goal_handle.goal_id.uuid).hex(),
            'query': query,
            'source': source,
        }
        if event == 'dispatch_result':
            payload['success'] = bool(result.success)
            payload['message'] = result.message
        if intent is not None:
            payload.update({
                'command': intent.command,
                'arguments': json.loads(intent.arguments_json()),
                'model': self.vlm_model if source == 'model' else '',
            })
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=True, separators=(',', ':'), sort_keys=True)
        self._decision_event_publisher.publish(message)

    def _cancel_callback(self, _goal_handle):
        """Forward parent cancellation to in-flight inference or child work."""
        with self._state_lock:
            handles = (self._active_vlm_goal, self._active_child_goal)
        for handle in handles:
            self._cancel_goal_best_effort(handle)
        return CancelResponse.ACCEPT

    def cancel_outstanding_work(self):
        """Best-effort shutdown cancellation for transient and motion goals."""
        with self._state_lock:
            handles = (
                self._active_vlm_goal,
                self._active_child_goal,
                self._active_motion_goal,
            )
        for handle in handles:
            self._cancel_goal_best_effort(handle)

    def _cancel_goal_best_effort(self, handle):
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as error:  # noqa: B902
            self.get_logger().error(
                f'Failed to forward cancellation: {type(error).__name__}')

    @staticmethod
    def _publish_feedback(goal_handle, phase, status, command=''):
        feedback = NaturalLanguageCommand.Feedback()
        feedback.phase = phase
        feedback.status = status
        feedback.command = command
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _check_parent_state(goal_handle):
        if goal_handle.is_cancel_requested:
            raise CommandCanceled()
        if not rclpy.ok():
            raise CommandFailure('ROS context is shutting down')

    def _wait_for_endpoint(
            self, ready_function, goal_handle, endpoint_name,
            timeout=None):
        deadline = time.monotonic() + (timeout or self.endpoint_timeout)
        while not ready_function():
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise CommandFailure(f'{endpoint_name} is unavailable')
            time.sleep(0.05)

    def _wait_for_future(
            self, future, goal_handle, timeout, operation_name):
        deadline = time.monotonic() + timeout
        while not future.done():
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise CommandFailure(f'{operation_name} timed out')
            time.sleep(0.05)
        self._check_parent_state(goal_handle)
        try:
            return future.result()
        except Exception as error:
            raise CommandFailure(f'{operation_name} failed') from error

    def _cancel_when_available(self, send_future):
        def cancel_dispatched_goal(completed_future):
            try:
                handle = completed_future.result()
                if handle.accepted:
                    self._cancel_goal_best_effort(handle)
            except Exception:
                pass

        send_future.add_done_callback(cancel_dispatched_goal)

    @staticmethod
    def _text_content(text):
        content = VlmContent()
        content.type = VlmContent.TYPE_TEXT
        content.text = text
        return content

    def _interpret(self, query, goal_handle):
        self._wait_for_endpoint(
            self._vlm_client.server_is_ready,
            goal_handle,
            'VLM action server',
        )
        vlm_goal = GenerateVlm.Goal()
        vlm_goal.content = [self._text_content(build_command_prompt(query))]
        vlm_goal.model = self.vlm_model
        vlm_goal.response_json_schema = build_command_schema(
            self.max_object_query_characters,
            self.max_map_name_characters,
        )
        send_future = self._vlm_client.send_goal_async(vlm_goal)
        try:
            child_handle = self._wait_for_future(
                send_future,
                goal_handle,
                self.endpoint_timeout,
                'VLM goal dispatch',
            )
        except (CommandCanceled, CommandFailure):
            self._cancel_when_available(send_future)
            raise
        if not child_handle.accepted:
            raise CommandFailure('VLM action server rejected the request')
        with self._state_lock:
            self._active_vlm_goal = child_handle
        try:
            wrapped = self._wait_for_future(
                child_handle.get_result_async(),
                goal_handle,
                self.vlm_result_timeout,
                'VLM command interpretation',
            )
        except (CommandCanceled, CommandFailure):
            self._cancel_goal_best_effort(child_handle)
            raise
        finally:
            with self._state_lock:
                if self._active_vlm_goal is child_handle:
                    self._active_vlm_goal = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise CommandFailure('VLM command interpretation did not succeed')
        if not wrapped.result.success:
            message = wrapped.result.error_message.strip()
            raise CommandFailure(message or 'VLM command interpretation failed')
        return parse_command_intent(
            wrapped.result.response_text,
            self.max_object_query_characters,
            self.max_map_name_characters,
        )

    def _send_child_goal(
            self, client, child_goal, goal_handle, endpoint_name):
        self._wait_for_endpoint(
            client.server_is_ready, goal_handle, endpoint_name)
        send_future = client.send_goal_async(child_goal)
        try:
            child_handle = self._wait_for_future(
                send_future,
                goal_handle,
                self.endpoint_timeout,
                f'{endpoint_name} goal dispatch',
            )
        except (CommandCanceled, CommandFailure):
            self._cancel_when_available(send_future)
            raise
        if not child_handle.accepted:
            raise CommandFailure(f'{endpoint_name} rejected the command')
        with self._state_lock:
            self._active_child_goal = child_handle
        return child_handle

    def _clear_transient_child(self, child_handle):
        with self._state_lock:
            if self._active_child_goal is child_handle:
                self._active_child_goal = None

    def _find_objects(self, object_query, goal_handle):
        child_goal = FindObject.Goal()
        child_goal.prompt = object_query
        child_handle = self._send_child_goal(
            self._find_client,
            child_goal,
            goal_handle,
            'FindObject action server',
        )
        try:
            wrapped = self._wait_for_future(
                child_handle.get_result_async(),
                goal_handle,
                self.find_result_timeout,
                'object search',
            )
        except (CommandCanceled, CommandFailure):
            self._cancel_goal_best_effort(child_handle)
            raise
        finally:
            self._clear_transient_child(child_handle)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise CommandFailure('object search did not succeed')
        if not wrapped.result.success:
            raise CommandFailure(
                wrapped.result.message or 'object search failed')
        return list(wrapped.result.matches), wrapped.result.message

    def _require_motion_idle(self):
        with self._state_lock:
            active_command = self._active_motion_command
            manual_active = self._manual_exploration_active
        if active_command:
            raise CommandFailure(
                f'{active_command} is active; cancel it before another motion '
                'command')
        if manual_active:
            raise CommandFailure(
                'manual exploration is active; stop it before a motion command')

    def _dispatch_motion(
            self, client, child_goal, goal_handle, command, endpoint_name):
        self._require_motion_idle()
        child_handle = self._send_child_goal(
            client, child_goal, goal_handle, endpoint_name)
        with self._state_lock:
            self._active_motion_goal = child_handle
            self._active_motion_command = command
        try:
            self._check_parent_state(goal_handle)
            result_future = child_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self._motion_finished(
                    child_handle, command, future))
        except Exception:
            self._cancel_goal_best_effort(child_handle)
            with self._state_lock:
                if self._active_motion_goal is child_handle:
                    self._active_motion_goal = None
                    self._active_motion_command = ''
            raise
        finally:
            self._clear_transient_child(child_handle)

    def _motion_finished(self, child_handle, command, future):
        status = None
        message = ''
        try:
            wrapped = future.result()
            status = wrapped.status
            message = getattr(wrapped.result, 'message', '')
        except Exception as error:
            message = type(error).__name__
        with self._state_lock:
            if self._active_motion_goal is child_handle:
                self._active_motion_goal = None
                self._active_motion_command = ''
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Dispatched {command} completed: {message}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Dispatched {command} was canceled')
        else:
            self.get_logger().warning(
                f'Dispatched {command} ended without success: {message}')

    def _set_manual_exploration(self, enabled, goal_handle):
        self._wait_for_endpoint(
            self._explore_client.service_is_ready,
            goal_handle,
            'exploration service',
        )
        request = SetBool.Request()
        request.data = enabled
        response = self._wait_for_future(
            self._explore_client.call_async(request),
            goal_handle,
            self.endpoint_timeout,
            'exploration service call',
        )
        if not response.success:
            raise CommandFailure(
                response.message or 'exploration service rejected the command')
        with self._state_lock:
            self._manual_exploration_active = enabled
        return response.message

    def _start_exploration(self, goal_handle):
        with self._state_lock:
            active_command = self._active_motion_command
        if active_command:
            raise CommandFailure(
                f'{active_command} is active; cancel it before exploration')
        return self._set_manual_exploration(True, goal_handle)

    def _save_map(self, map_name, goal_handle):
        self._wait_for_endpoint(
            self._save_map_client.service_is_ready,
            goal_handle,
            'save-map service',
        )
        request = SaveMap.Request()
        request.name.data = map_name
        response = self._wait_for_future(
            self._save_map_client.call_async(request),
            goal_handle,
            self.save_map_result_timeout,
            'save-map service call',
        )
        if response.result == SaveMap.Response.RESULT_NO_MAP_RECEIEVD:
            raise CommandFailure('SLAM Toolbox has no map to save yet')
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            raise CommandFailure(
                f'map save failed with result code {response.result}')
        if map_name:
            return f"map saved as '{map_name}'"
        return 'map saved using the configured default name'

    def _cancel_active_command(self, goal_handle):
        with self._state_lock:
            motion_goal = self._active_motion_goal
            motion_command = self._active_motion_command
            manual_active = self._manual_exploration_active
        messages = []
        if motion_goal is not None:
            cancel_response = self._wait_for_future(
                motion_goal.cancel_goal_async(),
                goal_handle,
                self.cancel_timeout,
                f'{motion_command} cancellation',
            )
            if not cancel_response.goals_canceling:
                raise CommandFailure(
                    f'{motion_command} was no longer cancelable')
            messages.append(f'cancellation accepted for {motion_command}')
        if manual_active or motion_goal is None:
            try:
                messages.append(self._set_manual_exploration(False, goal_handle))
            except CommandFailure:
                if motion_goal is None:
                    raise
        if not messages:
            raise CommandFailure('there is no router-dispatched command to cancel')
        return '; '.join(filter(None, messages))

    def _dispatch_intent(self, intent, goal_handle, result):
        command = intent.command
        if command == 'unsupported':
            raise CommandFailure(
                'the request does not map to one supported robot command')

        if command in ('find_object', 'go_to_object'):
            self._publish_feedback(
                goal_handle,
                NaturalLanguageCommand.Feedback.PHASE_RESOLVING_OBJECT,
                'resolving static registered objects',
                command,
            )
            matches, search_message = self._find_objects(
                intent.object_query, goal_handle)
            object_ids = [match.object_id for match in matches]
            result.object_ids = object_ids
            if command == 'find_object':
                return search_message
            if not object_ids:
                raise CommandFailure('no registered object matched the request')
            if len(object_ids) != 1:
                raise CommandFailure(
                    'go_to_object is ambiguous; matching IDs: {}'.format(
                        ', '.join(object_ids)))
            self._publish_feedback(
                goal_handle,
                NaturalLanguageCommand.Feedback.PHASE_DISPATCHING,
                'dispatching exact object ID to navigation',
                command,
            )
            child_goal = GoToObject.Goal()
            child_goal.object_id = object_ids[0]
            self._dispatch_motion(
                self._go_client,
                child_goal,
                goal_handle,
                command,
                'GoToObject action server',
            )
            return f'navigation dispatched for {object_ids[0]}'

        if command == 'look_for_object':
            child_goal = LookForObject.Goal()
            child_goal.prompt = intent.object_query
            child_goal.max_duration = 0.0
            child_goal.max_planning_steps = 0
            child_goal.completion_mode = (
                LookForObject.Goal.COMPLETION_APPROACH_OBJECT
                if intent.completion_mode == 'approach_object'
                else LookForObject.Goal.COMPLETION_REPORT_OBJECT
            )
            self._dispatch_motion(
                self._model_commander_client,
                child_goal,
                goal_handle,
                command,
                'LookForObject action server',
            )
            if intent.completion_mode == 'approach_object':
                return (
                    'model-supervised find-and-approach mission dispatched'
                )
            return 'model-supervised object-search mission dispatched'

        if command == 'start_exploration':
            return self._start_exploration(goal_handle)
        if command == 'stop_exploration':
            return self._set_manual_exploration(False, goal_handle)
        if command == 'save_map':
            return self._save_map(intent.map_name, goal_handle)
        if command == 'cancel_active_command':
            self._publish_feedback(
                goal_handle,
                NaturalLanguageCommand.Feedback.PHASE_CANCELING,
                'canceling active command',
                command,
            )
            return self._cancel_active_command(goal_handle)
        raise CommandFailure('validated command has no dispatcher')

    def _execute_callback(self, goal_handle):
        result = NaturalLanguageCommand.Result()
        query = goal_handle.request.query.strip()
        source = 'local'
        intent = None
        try:
            intent = parse_explicit_local_command(query)
            if intent is None:
                source = 'model'
                self._publish_feedback(
                    goal_handle,
                    NaturalLanguageCommand.Feedback.PHASE_INTERPRETING,
                    'interpreting natural-language command',
                )
                intent = self._interpret(query, goal_handle)
            else:
                self._publish_feedback(
                    goal_handle,
                    NaturalLanguageCommand.Feedback.PHASE_INTERPRETING,
                    'recognized explicit command locally',
                    intent.command,
                )
            result.command = intent.command
            result.arguments_json = intent.arguments_json()
            self._publish_decision_event(
                'validated_intent', goal_handle, query, source, result,
                intent=intent)
            self.get_logger().info(
                f'Validated natural-language command: {intent.command} '
                f'query_characters={len(query)}')
            self._publish_feedback(
                goal_handle,
                NaturalLanguageCommand.Feedback.PHASE_DISPATCHING,
                'dispatching validated command',
                intent.command,
            )
            result.message = self._dispatch_intent(
                intent, goal_handle, result)
            result.success = True
            self._publish_decision_event(
                'dispatch_result', goal_handle, query, source, result,
                intent=intent)
            goal_handle.succeed()
            return result
        except CommandCanceled:
            result.success = False
            result.message = 'natural-language command canceled'
            self._publish_decision_event(
                'dispatch_result', goal_handle, query, source, result,
                intent=intent)
            goal_handle.canceled()
            return result
        except (CommandFailure, CommandProtocolError) as error:
            result.success = False
            result.message = str(error)
            self._publish_decision_event(
                'dispatch_result', goal_handle, query, source, result,
                intent=intent)
            goal_handle.abort()
            self.get_logger().warning(f'Command dispatch aborted: {error}')
            return result
        except Exception as error:  # noqa: B902
            result.success = False
            result.message = 'internal natural-language command error'
            self._publish_decision_event(
                'dispatch_result', goal_handle, query, source, result,
                intent=intent)
            goal_handle.abort()
            self.get_logger().error(
                f'Internal command-router error: {type(error).__name__}')
            return result
        finally:
            with self._state_lock:
                self._active_vlm_goal = None
                self._active_child_goal = None
                self._busy = False


def main(args=None):
    """Run with two threads: one command worker and one ROS progress worker."""
    rclpy.init(args=args)
    node = NaturalLanguageCommandNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_outstanding_work()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
