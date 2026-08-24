#!/usr/bin/env python3
"""ROS 2 action/status transport for the independent v2 executive.

The node is deliberately usable with injected commander and tool backends in
tests or composition.  The standalone entry point fails a mission closed
with ``commander_unavailable`` until those independent authorities are wired
by the deployment launch; it never falls back to the old command layer.
"""

from threading import Lock
from typing import Callable, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from muto_command_layer_v2.action import Mission
from muto_command_layer_v2.msg import MissionBoard as MissionBoardMsg
from muto_command_layer_v2.msg import MissionEvent as MissionEventMsg
from muto_command_layer_v2.msg import RecorderStatus
from muto_command_layer_v2.msg import MissionRejection

from muto_command_layer_v2.commander import CommanderAgent
from muto_command_layer_v2.contracts import (
    CompletionPolicy,
    ContractError,
    LifecycleState,
    MissionAction,
    SCHEMA_VERSION,
)
from muto_command_layer_v2.executive import MissionExecutive
from muto_command_layer_v2.natural_language import (
    ActionRejection,
    CancellationRequest,
    NaturalLanguageAdapter,
)
from muto_command_layer_v2.ros_projection import board_to_msg, event_to_msg
from muto_command_layer_v2.runtime import CommanderRuntime
from muto_command_layer_v2.tools import ToolDispatcher


class MissionExecutiveNode(Node):
    """Expose one v2 mission action and the canonical board/event projections."""

    def __init__(
        self,
        *,
        commander: Optional[CommanderAgent] = None,
        dispatcher: Optional[ToolDispatcher] = None,
        action_name: str = "muto/mission",
        pose_supplier: Optional[Callable] = None,
        scenario_completion_policy: Optional[str] = None,
        scenario_id: str = "",
    ) -> None:
        super().__init__("muto_command_layer_v2_executive")
        self._commander = commander
        self._dispatcher = dispatcher
        self._pose_supplier = pose_supplier
        self._executive = MissionExecutive(
            scenario_completion_policy=(
                CompletionPolicy(scenario_completion_policy)
                if scenario_completion_policy else None
            ),
            scenario_id=scenario_id,
        )
        self._mission_lock = Lock()
        self._goal_handle = None
        self._active_runtime = None
        self._last_event_sequence = 0

        board_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._board_pub = self.create_publisher(
            MissionBoardMsg, "muto/mission_board", board_qos
        )
        self._event_pub = self.create_publisher(
            MissionEventMsg, "muto/mission_event", event_qos
        )
        self._recorder_status_sub = self.create_subscription(
            RecorderStatus,
            "muto/mission_recorder_status",
            self._recorder_status_callback,
            board_qos,
        )
        self._rejection_pub = self.create_publisher(
            MissionRejection, "muto/mission_rejection", board_qos
        )
        self._action_server = ActionServer(
            self,
            Mission,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    @property
    def executive(self) -> MissionExecutive:
        return self._executive

    def configure_dependencies(
        self,
        commander: CommanderAgent,
        dispatcher: ToolDispatcher,
    ) -> None:
        """Inject independent planner/backends before the first mission."""

        if commander is None or dispatcher is None:
            raise ValueError("commander and dispatcher are required")
        with self._mission_lock:
            if self._goal_handle is not None or self._executive.board.lifecycle_state not in {
                LifecycleState.IDLE,
                LifecycleState.SUCCEEDED,
                LifecycleState.CANCELED,
                LifecycleState.FAILED,
            }:
                raise RuntimeError("cannot reconfigure an active executive")
            self._commander = commander
            self._dispatcher = dispatcher

    def configure_pose_supplier(self, pose_supplier: Optional[Callable]) -> None:
        """Set the high-level pose source used for board projections."""

        if pose_supplier is not None and not callable(pose_supplier):
            raise ValueError("pose_supplier must be callable or None")
        with self._mission_lock:
            if self._goal_handle is not None:
                raise RuntimeError("cannot reconfigure pose supplier during a mission")
            self._pose_supplier = pose_supplier

    def _goal_callback(self, goal_request: Mission.Goal) -> GoalResponse:
        try:
            _to_action(goal_request)
        except (ContractError, TypeError, ValueError) as exc:
            # A natural-language stop request is a control request, not a new
            # mission.  Propagate it to the active runtime immediately.
            normalized = NaturalLanguageAdapter().normalize(
                getattr(goal_request, "objective", ""),
                request_id=getattr(goal_request, "request_id", ""),
            )
            if isinstance(normalized, CancellationRequest):
                runtime = self._active_runtime
                if runtime is not None:
                    runtime.request_cancel()
                self._publish_rejection(
                    goal_request,
                    "cancel_requested",
                    "The active mission was asked to cancel.",
                    active_mission_id=self._executive.board.mission_id,
                )
                return GoalResponse.REJECT
            self.get_logger().warning(
                "rejecting invalid v2 mission: {}".format(exc)
            )
            self._publish_rejection(
                goal_request,
                str(exc).split(":", 1)[0] or "invalid_request",
                str(exc),
            )
            return GoalResponse.REJECT
        with self._mission_lock:
            if self._goal_handle is not None or (
                self._executive.board.lifecycle_state not in {
                    LifecycleState.IDLE,
                    LifecycleState.SUCCEEDED,
                    LifecycleState.CANCELED,
                    LifecycleState.FAILED,
                }
            ):
                self._publish_rejection(
                    goal_request,
                    "mission_already_active",
                    "A mission is already active.",
                    active_mission_id=self._executive.board.mission_id,
                )
                return GoalResponse.REJECT
            # Reserve the single mission slot before execute_callback starts.
            self._goal_handle = False
        return GoalResponse.ACCEPT

    def _publish_rejection(
        self,
        goal_request,
        reason_code: str,
        message: str,
        *,
        active_mission_id: str = "",
        clarification: str = "",
    ) -> None:
        rejection = MissionRejection()
        rejection.header.stamp = self.get_clock().now().to_msg()
        rejection.schema_version = SCHEMA_VERSION
        rejection.request_id = str(getattr(goal_request, "request_id", ""))
        rejection.reason_code = reason_code
        rejection.message = message
        rejection.clarification = clarification
        rejection.active_mission_id = active_mission_id
        self._rejection_pub.publish(rejection)

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        runtime = self._active_runtime
        if runtime is not None:
            runtime.request_cancel()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        action = _to_action(goal_handle.request)
        with self._mission_lock:
            self._goal_handle = goal_handle
        try:
            if self._commander is None or self._dispatcher is None:
                self._executive.accept(action)
                self._publish_update()
                self._executive.start()
                self._publish_update()
                self._executive.fail("commander_unavailable")
                self._publish_update()
            else:
                runtime = CommanderRuntime(
                    self._executive,
                    self._commander,
                    self._dispatcher,
                    on_update=lambda _board, _events: self._publish_update(goal_handle),
                )
                self._active_runtime = runtime
                try:
                    runtime.run(
                        action,
                        cancel_check=lambda: _is_cancel_requested(goal_handle),
                    )
                finally:
                    self._active_runtime = None
            if _is_cancel_requested(goal_handle):
                # The runtime normally commits this transition at its next
                # boundary; this covers cancellation just after a terminal
                # tool result.
                if not self._executive.board.lifecycle_state.terminal:
                    self._executive.cancel()
                    self._publish_update(goal_handle)
                goal_handle.canceled()
            elif self._executive.board.lifecycle_state is LifecycleState.SUCCEEDED:
                goal_handle.succeed()
            else:
                # A terminal executive failure is an action abort, not a
                # transport-level success with a failed result payload.
                goal_handle.abort()
            return _result_from_board(self._executive.board)
        except Exception as exc:
            self.get_logger().error(
                "v2 executive execution failed: {}".format(exc)
            )
            if not self._executive.board.lifecycle_state.terminal:
                self._executive.fail("executive_exception")
                self._publish_update(goal_handle)
            goal_handle.abort()
            return _result_from_board(self._executive.board, fallback_reason=str(exc))
        finally:
            with self._mission_lock:
                self._goal_handle = None

    def _publish_update(self, goal_handle=None) -> None:
        self._refresh_board_observation()
        stamp = self.get_clock().now().to_msg()
        board = self._executive.board
        self._board_pub.publish(board_to_msg(board, stamp=stamp))
        for event in self._executive.events:
            if event.sequence <= self._last_event_sequence:
                continue
            self._event_pub.publish(event_to_msg(event, stamp=stamp))
            self._last_event_sequence = event.sequence
        active_goal = goal_handle or (
            self._goal_handle if self._goal_handle is not False else None
        )
        if active_goal is not None and hasattr(active_goal, "publish_feedback"):
            feedback = Mission.Feedback()
            feedback.state = _feedback_state(board.lifecycle_state)
            feedback.mission_id = board.mission_id
            feedback.active_skill = board.active_skill.value if board.active_skill else ""
            feedback.active_tool = board.active_tool.value if board.active_tool else ""
            feedback.board_revision = board.board_revision
            feedback.last_event_type = board.last_event_type
            feedback.last_reason_code = board.last_reason_code
            feedback.active_command_status = board.active_command_status
            feedback.confirmed_target_id = board.confirmed_target_id
            feedback.search_progress = board.search_progress
            feedback.approach_progress = board.approach_progress
            feedback.recorder_uri = board.recorder_uri
            active_goal.publish_feedback(feedback)

    def _recorder_status_callback(self, message: RecorderStatus) -> None:
        if not message.mission_id or message.mission_id != self._executive.board.mission_id:
            return
        self._executive.record_recorder_status(
            available=message.available,
            uri=message.uri,
            reason_code=message.reason_code,
        )
        self._publish_update()

    def _refresh_board_observation(self) -> None:
        if self._pose_supplier is None:
            return
        try:
            pose = self._pose_supplier(self._executive.board)
            self._executive.record_board_observation(robot_pose=pose)
        except Exception as exc:
            self.get_logger().debug(
                "v2 pose observation unavailable: {}".format(exc)
            )


def _to_action(goal: Mission.Goal) -> MissionAction:
    if goal.schema_version != SCHEMA_VERSION:
        raise ContractError("unsupported schema_version")
    # The v2 action doubles as the small natural-language frontend when the
    # caller leaves ``object_request`` empty. Structured callers may provide
    # both fields and bypass normalization without changing the executive
    # contract.
    if not str(goal.object_request).strip():
        normalized = NaturalLanguageAdapter().normalize(
            goal.objective,
            request_id=goal.request_id,
            completion_policy=goal.completion_policy or None,
        )
        if isinstance(normalized, ActionRejection):
            raise ContractError(
                "{}: {}".format(normalized.reason_code, normalized.message)
            )
        if isinstance(normalized, CancellationRequest):
            raise ContractError(
                "cancellation must use the ROS action cancel protocol"
            )
        return normalized
    return MissionAction(
        request_id=goal.request_id,
        objective=goal.objective,
        object_request=goal.object_request,
        completion_policy=goal.completion_policy,
        schema_version=goal.schema_version,
    )


def _feedback_state(state: LifecycleState) -> int:
    return {
        LifecycleState.IDLE: Mission.Feedback.STATE_IDLE,
        LifecycleState.ACCEPTED: Mission.Feedback.STATE_ACCEPTED,
        LifecycleState.RUNNING: Mission.Feedback.STATE_RUNNING,
        LifecycleState.SUCCEEDED: Mission.Feedback.STATE_SUCCEEDED,
        LifecycleState.CANCELED: Mission.Feedback.STATE_CANCELED,
        LifecycleState.FAILED: Mission.Feedback.STATE_FAILED,
    }[state]


def _is_cancel_requested(goal_handle) -> bool:
    """Normalize Humble's boolean property and newer callable variants."""

    value = getattr(goal_handle, "is_cancel_requested", False)
    return bool(value() if callable(value) else value)


def _result_from_board(board, *, fallback_reason: str = "") -> Mission.Result:
    result = Mission.Result()
    if board.lifecycle_state is LifecycleState.SUCCEEDED:
        result.outcome = Mission.Result.OUTCOME_SUCCEEDED
    elif board.lifecycle_state is LifecycleState.CANCELED:
        result.outcome = Mission.Result.OUTCOME_CANCELED
    else:
        result.outcome = Mission.Result.OUTCOME_FAILED
    result.mission_id = board.mission_id
    result.reason_code = fallback_reason or board.last_reason_code
    result.summary = board.last_outcome or board.last_event_type
    result.confirmed_target_id = board.confirmed_target_id
    result.bag_uri = board.recorder_uri
    return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutiveNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
