"""ROS Humble transport projection tests (no running graph required)."""

import sys
import threading
import time

import pytest

pytest.importorskip("rclpy")

# launch_testing may prepend the source workspace while collecting this file.
# Remove only those path entries before importing the installed ROSIDL package;
# never reset ``sys.modules`` because that invalidates live type-support
# capsules for tests collected later in the same process.
for _path in list(sys.path):
    if _path.rstrip("/").endswith("/src") or "/src/muto_command_layer_v2" in _path:
        sys.path.remove(_path)
for _name, _module in list(sys.modules.items()):
    _module_file = getattr(_module, "__file__", "") or ""
    if (
        (_name == "muto_command_layer_v2" or _name.startswith("muto_command_layer_v2."))
        and "/src/muto_command_layer_v2" in _module_file
    ):
        del sys.modules[_name]

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from muto_command_layer_v2.action import Mission
from muto_command_layer_v2.contracts import (
    CandidateEvidence,
    CompletionPolicy,
    LifecycleState,
    MissionAction,
    MissionBoard,
)
from muto_command_layer_v2.mission_executive_node import (
    _feedback_state,
    _result_from_board,
    _to_action,
)
from muto_command_layer_v2.commander import CommanderAgent
from muto_command_layer_v2.ros_projection import board_to_msg
from muto_command_layer_v2.tools import ToolDispatcher, ToolResult


def test_goal_and_board_projection_use_v2_schema():
    goal = Mission.Goal()
    goal.request_id = "request-1"
    goal.objective = "find the chair"
    goal.object_request = "chair"
    goal.completion_policy = CompletionPolicy.REPORT_CONFIRMED.value
    goal.schema_version = "muto_command_layer_v2"
    action = _to_action(goal)
    assert action.object_request == "chair"

    natural_language_goal = Mission.Goal()
    natural_language_goal.request_id = "request-nl"
    natural_language_goal.objective = "please find the purple chair"
    natural_language_goal.object_request = ""
    natural_language_goal.completion_policy = ""
    natural_language_goal.schema_version = "muto_command_layer_v2"
    normalized = _to_action(natural_language_goal)
    assert normalized.object_request == "purple chair"
    assert normalized.completion_policy is CompletionPolicy.REPORT_CONFIRMED

    board = MissionBoard(
        lifecycle_state=LifecycleState.RUNNING,
        mission_id="mission-0001",
        request_id="request-1",
        objective=action.objective,
        object_request=action.object_request,
        completion_policy=action.completion_policy,
        board_revision=3,
        robot_pose=(1.0, 2.0, 1.57),
    )
    msg = board_to_msg(board)
    assert msg.schema_version == "muto_command_layer_v2"
    assert msg.lifecycle_state == msg.RUNNING
    assert msg.board_revision == 3
    assert msg.robot_pose.header.frame_id == "map"
    assert msg.robot_pose.pose.position.x == 1.0
    assert _feedback_state(LifecycleState.RUNNING) == Mission.Feedback.STATE_RUNNING


def test_board_projection_preserves_candidate_evidence_provenance():
    board = MissionBoard(
        lifecycle_state=LifecycleState.RUNNING,
        mission_id="mission-0002",
        request_id="request-2",
        board_revision=5,
        registry_revision="registry-7",
        candidate_evidence=(
            CandidateEvidence(
                "chair-1",
                "registry-7",
                evidence_id="jpeg-1",
                source="vlm_candidate_inspection",
                confidence=0.91,
                observed_at_s=123.5,
                reason_code="exact_match",
            ),
        ),
    )
    msg = board_to_msg(board)
    assert list(msg.candidate_evidence_candidate_ids) == ["chair-1"]
    assert list(msg.candidate_evidence_ids) == ["jpeg-1"]
    assert list(msg.candidate_evidence_sources) == ["vlm_candidate_inspection"]
    assert list(msg.candidate_evidence_confidences) == pytest.approx([0.91], abs=1e-5)
    assert list(msg.candidate_evidence_reason_codes) == ["exact_match"]
    assert list(msg.candidate_evidence_timestamps_s) == pytest.approx([123.5])


def test_terminal_result_is_projected_without_bag_dependency():
    board = MissionBoard(
        lifecycle_state=LifecycleState.FAILED,
        mission_id="mission-0001",
        request_id="request-1",
        board_revision=4,
        last_reason_code="commander_unavailable",
        last_outcome="failed",
    )
    result = _result_from_board(board)
    assert result.outcome == Mission.Result.OUTCOME_FAILED
    assert result.reason_code == "commander_unavailable"
    assert result.bag_uri == ""


class _ActionBackend:
    def query_registry(self, _call, _board):
        return ToolResult(True, candidate_ids=("chair-1",), registry_revision="r1")

    def inspect_candidates(self, _call, _board):
        return ToolResult(
            True,
            candidate_ids=("chair-1",),
            confirmed_target_id="chair-1",
            registry_revision="r1",
            evidence=(CandidateEvidence("chair-1", "r1", evidence_id="img-chair-1", source="test", confidence=1.0),),
        )

    def observe(self, _call, _board):
        return ToolResult(True, progress_delta=1.0)

    def rotate_to_heading(self, _call, _board):
        return ToolResult(True, progress_delta=1.0)

    def go_to_point(self, _call, _board):
        return ToolResult(True, progress_delta=1.0)


class _CancelableBackend(_ActionBackend):
    def observe(self, _call, _board):
        # Hold the tool boundary long enough for the ROS cancel request to be
        # serviced by the executive's reentrant action callback group.
        time.sleep(0.25)
        return ToolResult(True)


def _wait_future(future, timeout=5.0):
    done = threading.Event()
    holder = []

    def _complete(completed):
        holder.append(completed)
        done.set()

    future.add_done_callback(_complete)
    assert done.wait(timeout), "ROS future timed out"
    return holder[0].result()


def test_real_v2_mission_action_runs_natural_language_to_terminal_result():
    """Exercise the ROS action boundary with a deterministic backend."""

    def planner(board):
        if not board.shortlisted_candidate_ids:
            return {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {"name": "query_registry"},
            }
        if not board.confirmed_target_id:
            return {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {
                    "name": "inspect_candidates",
                    "candidate_ids": list(board.shortlisted_candidate_ids),
                    "registry_revision": board.registry_revision,
                },
            }
        return {
            "schema_version": "muto_command_layer_v2",
            "skill": "search_for_object",
            "tool": None,
            "completion_proposal": "report_confirmed",
            "rationale": "exact stored candidate confirmed",
        }

    from muto_command_layer_v2.mission_executive_node import MissionExecutiveNode

    rclpy.init()
    executive_node = MissionExecutiveNode(
        commander=CommanderAgent(planner),
        dispatcher=ToolDispatcher(_ActionBackend()),
        action_name="/v2/test_mission",
    )
    client_node = Node("v2_mission_action_client")
    client = ActionClient(client_node, Mission, "/v2/test_mission")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(executive_node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=5.0)
        goal = Mission.Goal()
        goal.request_id = "ros-action-request"
        goal.objective = "find the purple chair"
        goal.object_request = ""
        goal.completion_policy = ""
        goal.schema_version = "muto_command_layer_v2"
        goal_handle = _wait_future(client.send_goal_async(goal))
        assert goal_handle.accepted
        wrapped = _wait_future(goal_handle.get_result_async())
        assert wrapped.result.outcome == Mission.Result.OUTCOME_SUCCEEDED
        assert wrapped.result.confirmed_target_id == "chair-1"
    finally:
        executor.shutdown(timeout_sec=2.0)
        client_node.destroy_node()
        executive_node.destroy_node()
        thread.join(timeout=2.0)
        rclpy.shutdown()


def test_real_v2_mission_action_cancel_stops_at_tool_boundary():
    """Cancellation is cooperative and produces a canceled terminal result."""

    def planner(_board):
        return {
            "schema_version": "muto_command_layer_v2",
            "skill": "search_for_object",
            "tool": {"name": "observe"},
            "completion_proposal": None,
            "rationale": "hold at a cancellable observation boundary",
        }

    from muto_command_layer_v2.mission_executive_node import MissionExecutiveNode

    rclpy.init()
    executive_node = MissionExecutiveNode(
        commander=CommanderAgent(planner),
        dispatcher=ToolDispatcher(_CancelableBackend()),
        action_name="/v2/test_cancel_mission",
    )
    client_node = Node("v2_cancel_action_client")
    client = ActionClient(client_node, Mission, "/v2/test_cancel_mission")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(executive_node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=5.0)
        goal = Mission.Goal()
        goal.request_id = "ros-cancel-request"
        goal.objective = "find the purple chair"
        goal.object_request = ""
        goal.completion_policy = ""
        goal.schema_version = "muto_command_layer_v2"
        goal_handle = _wait_future(client.send_goal_async(goal))
        assert goal_handle.accepted
        time.sleep(0.05)
        cancel_response = _wait_future(goal_handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        wrapped = _wait_future(goal_handle.get_result_async())
        assert wrapped.result.outcome == Mission.Result.OUTCOME_CANCELED
        assert executive_node.executive.board.lifecycle_state is LifecycleState.CANCELED
    finally:
        executor.shutdown(timeout_sec=2.0)
        client_node.destroy_node()
        executive_node.destroy_node()
        thread.join(timeout=2.0)
        rclpy.shutdown()
