#!/usr/bin/env python3
"""ROS 2 action server for ordered text and JPEG VLM requests."""

import math
import os
import threading

from muto_vlm_socket.action import GenerateVlm
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vlm_socket_protocol import (
    build_request_body,
    decode_response_json_schema,
    encode_content,
    extract_response,
    parse_endpoint,
    post_vlm_request,
    ProtocolError,
    ProtocolLimits,
    TransportError,
    validate_limits,
    validate_wire_api,
)


class VlmSocketNode(Node):
    """Expose an OpenAI-compatible multimodal endpoint as a ROS action."""

    def __init__(self):
        super().__init__('vlm_socket')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._state_lock = threading.Lock()
        self._request_cancel_event = threading.Event()
        self._busy = False
        self._active_connection = None
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            GenerateVlm,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        auth_description = (
            f'environment variable {self.api_key_env}'
            if self.require_api_key else 'optional authentication'
        )
        self.get_logger().info(
            f'VLM socket ready: action={self.action_name} '
            f'endpoint={self.base_url} wire_api={self.wire_api} '
            f'model={self.default_model} '
            f'credentials={auth_description}')

    def _declare_parameters(self):
        """Declare the complete configuration surface."""
        self.declare_parameter('action_name', '/vlm/generate')
        self.declare_parameter('base_url', 'http://127.0.0.1:8000/v1')
        self.declare_parameter('wire_api', 'responses')
        self.declare_parameter('api_key_env', 'DASHSCOPE_API_KEY')
        self.declare_parameter('require_api_key', True)
        self.declare_parameter('default_model', 'gpt-5.5')
        self.declare_parameter('system_prompt', '')
        self.declare_parameter('request_timeout', 120.0)
        self.declare_parameter('max_content_parts', 16)
        self.declare_parameter('max_text_characters', 65536)
        self.declare_parameter('max_jpeg_bytes', 8388608)
        self.declare_parameter('max_total_jpeg_bytes', 25165824)
        self.declare_parameter('max_request_bytes', 41943040)
        self.declare_parameter('max_response_bytes', 4194304)
        self.declare_parameter('max_tokens', 0)
        self.declare_parameter('temperature', -1.0)
        self.declare_parameter('store_response', False)

    def _read_parameters(self):
        """Read parameters once because transport identity is immutable."""
        self.action_name = self.get_parameter('action_name').value
        self.base_url = self.get_parameter('base_url').value
        self.wire_api = self.get_parameter('wire_api').value
        self.api_key_env = self.get_parameter('api_key_env').value
        self.require_api_key = self.get_parameter('require_api_key').value
        self.default_model = self.get_parameter('default_model').value
        self.system_prompt = self.get_parameter('system_prompt').value
        self.request_timeout = self.get_parameter('request_timeout').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value
        self.store_response = self.get_parameter('store_response').value
        self.limits = ProtocolLimits(
            self.get_parameter('max_content_parts').value,
            self.get_parameter('max_text_characters').value,
            self.get_parameter('max_jpeg_bytes').value,
            self.get_parameter('max_total_jpeg_bytes').value,
            self.get_parameter('max_request_bytes').value,
            self.get_parameter('max_response_bytes').value,
        )

    def _validate_parameters(self):
        """Fail at startup instead of failing every action request."""
        if not self.action_name:
            raise ValueError('action_name must not be empty')
        validate_wire_api(self.wire_api)
        self.endpoint = parse_endpoint(self.base_url, self.wire_api)
        validate_limits(self.limits)
        if not self.default_model.strip():
            raise ValueError('default_model must not be empty')
        if self.require_api_key and not self.api_key_env.strip():
            raise ValueError(
                'api_key_env must not be empty when require_api_key is true')
        if not math.isfinite(self.request_timeout) or \
                self.request_timeout <= 0.0:
            raise ValueError('request_timeout must be finite and positive')
        if self.max_tokens < 0:
            raise ValueError('max_tokens must be nonnegative')
        if not math.isfinite(self.temperature) or (
                self.temperature < 0.0 and self.temperature != -1.0):
            raise ValueError('temperature must be -1 or nonnegative')

    def _goal_callback(self, goal_request):
        """Reject malformed or overlapping requests before execution."""
        if not goal_request.content:
            self.get_logger().warning(
                'Rejected VLM goal without content parts')
            return GoalResponse.REJECT
        with self._state_lock:
            if self._busy:
                self.get_logger().warning(
                    'Rejected VLM goal because another request is active')
                return GoalResponse.REJECT
            self._request_cancel_event.clear()
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        """Accept cancellation and interrupt the active HTTP connection."""
        self._request_cancel_event.set()
        self._close_active_connection()
        return CancelResponse.ACCEPT

    def _set_active_connection(self, connection):
        """Publish the current connection to the cancellation callback."""
        close_immediately = False
        with self._state_lock:
            if connection is not None and self._request_cancel_event.is_set():
                close_immediately = True
                self._active_connection = None
            else:
                self._active_connection = connection
        if close_immediately:
            try:
                connection.close()
            except OSError:
                pass

    def _close_active_connection(self):
        """Best-effort interruption for a blocked HTTP request."""
        with self._state_lock:
            connection = self._active_connection
            self._active_connection = None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    @staticmethod
    def _publish_feedback(goal_handle, phase, status):
        """Publish non-sensitive action progress."""
        feedback = GenerateVlm.Feedback()
        feedback.phase = phase
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _canceled_result(goal_handle, message):
        """Finish a requested cancellation with no response payload."""
        result = GenerateVlm.Result()
        result.success = False
        result.error_message = message
        goal_handle.canceled()
        return result

    def _execute_callback(self, goal_handle):
        """Validate, send, and decode one VLM action goal."""
        result = GenerateVlm.Result()
        encoded = None
        try:
            self._publish_feedback(
                goal_handle, 1, 'validating multimodal content')
            selected_model = goal_handle.request.model.strip()
            if not selected_model:
                selected_model = self.default_model
            api_key = os.getenv(self.api_key_env) if self.api_key_env else None
            if api_key:
                api_key = api_key.strip()
            if self.require_api_key and not api_key:
                raise ProtocolError(
                    f'required credential environment variable '
                    f'{self.api_key_env} is not set')

            encoded = encode_content(goal_handle.request.content, self.limits)
            response_json_schema = decode_response_json_schema(
                goal_handle.request.response_json_schema,
                self.limits.max_text_characters,
            )
            request_body = build_request_body(
                encoded,
                selected_model,
                self.system_prompt,
                self.max_tokens,
                self.temperature,
                self.limits.max_request_bytes,
                self.wire_api,
                self.store_response,
                response_json_schema,
            )
            if goal_handle.is_cancel_requested:
                return self._canceled_result(
                    goal_handle, 'VLM request canceled before dispatch')

            self._publish_feedback(
                goal_handle, 2, 'connecting to VLM endpoint')
            self._publish_feedback(
                goal_handle, 3, 'waiting for VLM response')
            response = post_vlm_request(
                self.endpoint,
                request_body,
                api_key,
                self.request_timeout,
                self.limits.max_response_bytes,
                self._set_active_connection,
                self.wire_api,
            )
            if goal_handle.is_cancel_requested:
                return self._canceled_result(
                    goal_handle, 'VLM request canceled')

            self._publish_feedback(
                goal_handle, 4, 'decoding VLM response')
            response_text, prompt_tokens, completion_tokens = \
                extract_response(response, self.wire_api)
            result.success = True
            result.response_text = response_text
            result.prompt_tokens = prompt_tokens
            result.completion_tokens = completion_tokens
            goal_handle.succeed()
            self.get_logger().info(
                f'VLM request completed: parts={len(encoded.parts)} '
                f'text_characters={encoded.text_characters} '
                f'jpeg_bytes={encoded.jpeg_bytes} '
                f'prompt_tokens={prompt_tokens} '
                f'completion_tokens={completion_tokens}')
            return result
        except (ProtocolError, TransportError) as error:
            if goal_handle.is_cancel_requested:
                return self._canceled_result(
                    goal_handle, 'VLM request canceled')
            result.success = False
            result.error_message = str(error)
            goal_handle.abort()
            self.get_logger().warning(
                f'VLM request aborted: {error}')
            return result
        except Exception as error:  # noqa: B902
            if goal_handle.is_cancel_requested:
                return self._canceled_result(
                    goal_handle, 'VLM request canceled')
            result.success = False
            result.error_message = 'internal VLM socket error'
            goal_handle.abort()
            self.get_logger().error(
                f'Internal VLM socket error: {type(error).__name__}')
            return result
        finally:
            self._close_active_connection()
            with self._state_lock:
                self._busy = False


def main(args=None):
    """Run the VLM socket in a multithreaded executor for cancellation."""
    rclpy.init(args=args)
    node = VlmSocketNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._close_active_connection()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
