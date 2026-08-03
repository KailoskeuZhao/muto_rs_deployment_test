#!/usr/bin/env python3
"""Active static-object search composed from existing command actions."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from muto_command_layer.action import (
    ExploreAndRecord,
    FindObject,
    FindSomething,
)
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sam2_object_registry.msg import StoredObjectArray


class SearchCanceled(RuntimeError):
    """Raised internally when the active-search action is canceled."""


class SearchFailure(RuntimeError):
    """Raised when a composed child command cannot complete safely."""


class ActiveObjectSearchNode(Node):
    """Check the registry, explore, and recheck as objects are confirmed."""

    def __init__(self):
        super().__init__('active_object_search')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._callback_group = ReentrantCallbackGroup()
        self._find_client = ActionClient(
            self,
            FindObject,
            self.find_object_action,
            callback_group=self._callback_group,
        )
        self._program_client = ActionClient(
            self,
            ExploreAndRecord,
            self.explore_and_record_action,
            callback_group=self._callback_group,
        )

        self._state_lock = threading.Lock()
        self._busy = False
        self._active_find_goal = None
        self._active_program_goal = None
        self._registry_signature = None
        self._registry_revision = 0
        self._confirmed_object_count = 0

        registry_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._registry_subscription = self.create_subscription(
            StoredObjectArray,
            self.registry_topic,
            self._registry_callback,
            registry_qos,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            FindSomething,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'Active object search ready: action={self.action_name} '
            f'find={self.find_object_action} '
            f'program={self.explore_and_record_action}')

    def _declare_parameters(self):
        self.declare_parameter('action_name', '/find_something')
        self.declare_parameter('find_object_action', '/find_object')
        self.declare_parameter(
            'explore_and_record_action', '/explore_and_record')
        self.declare_parameter('registry_topic', '/sam2/stored_objects')
        self.declare_parameter('endpoint_timeout', 5.0)
        self.declare_parameter('find_result_timeout', 400.0)
        self.declare_parameter('child_stop_timeout', 10.0)
        self.declare_parameter('max_prompt_characters', 8192)

    def _read_parameters(self):
        for name in (
                'action_name', 'find_object_action',
                'explore_and_record_action', 'registry_topic',
                'endpoint_timeout', 'find_result_timeout',
                'child_stop_timeout', 'max_prompt_characters'):
            setattr(self, name, self.get_parameter(name).value)

    def _validate_parameters(self):
        for name in (
                'action_name', 'find_object_action',
                'explore_and_record_action', 'registry_topic'):
            if not getattr(self, name):
                raise ValueError(f'{name} must not be empty')
        for name in (
                'endpoint_timeout', 'find_result_timeout',
                'child_stop_timeout'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or \
                    not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if isinstance(self.max_prompt_characters, bool) or \
                not isinstance(self.max_prompt_characters, int) or \
                self.max_prompt_characters <= 0:
            raise ValueError('max_prompt_characters must be a positive integer')

    def _registry_callback(self, message):
        signature = tuple(sorted(
            (item.name, item.label, item.image_path)
            for item in message.objects
        ))
        with self._state_lock:
            self._confirmed_object_count = len(message.objects)
            if signature != self._registry_signature:
                self._registry_signature = signature
                self._registry_revision += 1

    def _goal_callback(self, goal_request):
        prompt = goal_request.prompt.strip()
        if not prompt or len(prompt) > self.max_prompt_characters:
            self.get_logger().warning(
                'Rejected active search with empty/oversized prompt')
            return GoalResponse.REJECT
        with self._state_lock:
            if self._busy:
                self.get_logger().warning(
                    'Rejected active search while another search is running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.cancel_outstanding_work()
        return CancelResponse.ACCEPT

    def cancel_outstanding_work(self):
        """Best-effort cancellation for currently composed child actions."""
        with self._state_lock:
            handles = (self._active_find_goal, self._active_program_goal)
        for handle in handles:
            self._cancel_goal_best_effort(handle)

    def _cancel_goal_best_effort(self, handle):
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as error:  # noqa: B902
            self.get_logger().error(
                f'Failed to forward child cancellation: '
                f'{type(error).__name__}')

    def _cancel_goal_and_wait_best_effort(
            self, handle, timeout, operation_name):
        if handle is None:
            return
        try:
            cancel_future = handle.cancel_goal_async()
            result_future = handle.get_result_async()
            deadline = time.monotonic() + timeout
            while not cancel_future.done() and rclpy.ok() and \
                    time.monotonic() < deadline:
                time.sleep(0.05)
            while not result_future.done() and rclpy.ok() and \
                    time.monotonic() < deadline:
                time.sleep(0.05)
            if not result_future.done():
                self.get_logger().error(
                    f'{operation_name} did not stop before cleanup timeout')
        except Exception as error:  # noqa: B902
            self.get_logger().error(
                f'Failed to stop {operation_name} during cleanup: '
                f'{type(error).__name__}')

    @staticmethod
    def _check_parent_state(goal_handle):
        if goal_handle.is_cancel_requested:
            raise SearchCanceled()
        if not rclpy.ok():
            raise SearchFailure('ROS context is shutting down')

    @staticmethod
    def _publish_feedback(goal_handle, phase, status, object_count):
        feedback = FindSomething.Feedback()
        feedback.phase = phase
        feedback.status = status
        feedback.confirmed_object_count = object_count
        goal_handle.publish_feedback(feedback)

    def _state_snapshot(self):
        with self._state_lock:
            return self._registry_revision, self._confirmed_object_count

    def _wait_for_registry_snapshot(self, goal_handle):
        deadline = time.monotonic() + self.endpoint_timeout
        while True:
            with self._state_lock:
                ready = self._registry_signature is not None
            if ready:
                return self._state_snapshot()
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise SearchFailure(
                    'confirmed-object registry snapshot is unavailable')
            time.sleep(0.05)

    def _wait_for_endpoint(self, ready_function, goal_handle, endpoint_name):
        deadline = time.monotonic() + self.endpoint_timeout
        while not ready_function():
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise SearchFailure(f'{endpoint_name} is unavailable')
            time.sleep(0.05)

    def _wait_for_future(
            self, future, goal_handle, timeout, operation_name):
        deadline = time.monotonic() + timeout
        while not future.done():
            self._check_parent_state(goal_handle)
            if time.monotonic() >= deadline:
                raise SearchFailure(f'{operation_name} timed out')
            time.sleep(0.05)
        self._check_parent_state(goal_handle)
        try:
            return future.result()
        except Exception as error:
            raise SearchFailure(f'{operation_name} failed') from error

    def _cancel_when_available(self, send_future):
        deadline = time.monotonic() + self.endpoint_timeout
        while not send_future.done() and rclpy.ok() and \
                time.monotonic() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            return
        try:
            handle = send_future.result()
            if handle.accepted:
                self._cancel_goal_and_wait_best_effort(
                    handle,
                    self.child_stop_timeout,
                    'newly dispatched child action',
                )
        except Exception:
            pass

    def _send_goal(self, client, child_goal, goal_handle, endpoint_name):
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
        except (SearchCanceled, SearchFailure):
            self._cancel_when_available(send_future)
            raise
        if not child_handle.accepted:
            raise SearchFailure(f'{endpoint_name} rejected the command')
        return child_handle

    def _run_find(self, prompt, goal_handle):
        find_goal = FindObject.Goal()
        find_goal.prompt = prompt
        child_handle = self._send_goal(
            self._find_client,
            find_goal,
            goal_handle,
            'FindObject action server',
        )
        with self._state_lock:
            self._active_find_goal = child_handle
        try:
            wrapped = self._wait_for_future(
                child_handle.get_result_async(),
                goal_handle,
                self.find_result_timeout,
                'registry object search',
            )
        except (SearchCanceled, SearchFailure):
            self._cancel_goal_and_wait_best_effort(
                child_handle,
                self.child_stop_timeout,
                'FindObject action',
            )
            raise
        finally:
            with self._state_lock:
                if self._active_find_goal is child_handle:
                    self._active_find_goal = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise SearchFailure('registry object search did not succeed')
        if not wrapped.result.success:
            raise SearchFailure(
                wrapped.result.message or 'registry object search failed')
        return list(wrapped.result.matches), wrapped.result.message

    def _start_program(self, goal_handle):
        child_handle = self._send_goal(
            self._program_client,
            ExploreAndRecord.Goal(),
            goal_handle,
            'ExploreAndRecord action server',
        )
        with self._state_lock:
            self._active_program_goal = child_handle
        return child_handle, child_handle.get_result_async()

    def _stop_program(self, child_handle, result_future, goal_handle):
        cancel_response = self._wait_for_future(
            child_handle.cancel_goal_async(),
            goal_handle,
            self.child_stop_timeout,
            'explore-and-record cancellation',
        )
        if not cancel_response.goals_canceling and not result_future.done():
            raise SearchFailure(
                'explore-and-record did not accept cancellation')
        self._wait_for_future(
            result_future,
            goal_handle,
            self.child_stop_timeout,
            'explore-and-record stop',
        )

    def _stop_program_best_effort(self, child_handle):
        if child_handle is None:
            return
        self._cancel_goal_and_wait_best_effort(
            child_handle,
            self.child_stop_timeout,
            'ExploreAndRecord action',
        )

    @staticmethod
    def _success_result(message, matches):
        result = FindSomething.Result()
        result.success = True
        result.message = message
        result.matches = matches
        return result

    def _execute_callback(self, goal_handle):
        prompt = goal_handle.request.prompt.strip()
        result = FindSomething.Result()
        program_handle = None
        program_future = None
        try:
            self._publish_feedback(
                goal_handle,
                FindSomething.Feedback.PHASE_CHECKING_REGISTRY,
                'waiting for the confirmed static-object registry',
                0,
            )
            handled_revision, object_count = \
                self._wait_for_registry_snapshot(goal_handle)
            self._publish_feedback(
                goal_handle,
                FindSomething.Feedback.PHASE_CHECKING_REGISTRY,
                'checking already registered static objects',
                object_count,
            )
            matches, message = self._run_find(prompt, goal_handle)
            if matches:
                result = self._success_result(message, matches)
                goal_handle.succeed()
                return result

            self._publish_feedback(
                goal_handle,
                FindSomething.Feedback.PHASE_EXPLORING,
                'starting active exploration and object recording',
                object_count,
            )
            program_handle, program_future = self._start_program(goal_handle)

            while rclpy.ok():
                self._check_parent_state(goal_handle)
                revision, object_count = self._state_snapshot()
                if revision != handled_revision:
                    handled_revision = revision
                    self._publish_feedback(
                        goal_handle,
                        FindSomething.Feedback.PHASE_RECHECKING_REGISTRY,
                        'new confirmed object set; checking for a match',
                        object_count,
                    )
                    matches, message = self._run_find(prompt, goal_handle)
                    if matches:
                        self._publish_feedback(
                            goal_handle,
                            FindSomething.Feedback.PHASE_STOPPING_EXPLORATION,
                            'match found; stopping active exploration',
                            object_count,
                        )
                        self._stop_program(
                            program_handle, program_future, goal_handle)
                        result = self._success_result(message, matches)
                        goal_handle.succeed()
                        return result

                if program_future.done():
                    try:
                        wrapped = program_future.result()
                    except Exception as error:
                        raise SearchFailure(
                            'explore-and-record result failed') from error
                    if wrapped.status != GoalStatus.STATUS_SUCCEEDED or \
                            not wrapped.result.success:
                        raise SearchFailure(
                            wrapped.result.message or
                            'explore-and-record did not succeed')
                    self._publish_feedback(
                        goal_handle,
                        FindSomething.Feedback.PHASE_RECHECKING_REGISTRY,
                        'search mission complete; making final registry check',
                        object_count,
                    )
                    matches, message = self._run_find(prompt, goal_handle)
                    if not matches:
                        message = (
                            'active search completed; no matching static '
                            'object was found')
                    result = self._success_result(message, matches)
                    goal_handle.succeed()
                    return result
                time.sleep(0.05)
            raise SearchFailure('ROS context is shutting down')
        except SearchCanceled:
            self.cancel_outstanding_work()
            self._stop_program_best_effort(program_handle)
            result.success = False
            result.message = 'active object search canceled'
            goal_handle.canceled()
            return result
        except SearchFailure as error:
            self.cancel_outstanding_work()
            self._stop_program_best_effort(program_handle)
            result.success = False
            result.message = str(error)
            goal_handle.abort()
            self.get_logger().warning(f'Active object search aborted: {error}')
            return result
        except Exception as error:  # noqa: B902
            self.cancel_outstanding_work()
            self._stop_program_best_effort(program_handle)
            result.success = False
            result.message = 'internal active object search error'
            goal_handle.abort()
            self.get_logger().error(
                f'Internal active-search error: {type(error).__name__}')
            return result
        finally:
            with self._state_lock:
                self._active_find_goal = None
                self._active_program_goal = None
                self._busy = False


def main(args=None):
    """Run one search worker and one ROS progress worker."""
    rclpy.init(args=args)
    node = ActiveObjectSearchNode()
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
