"""A full in-process decision -> tool -> board -> terminal trace."""

from muto_command_layer_v2.commander import CommanderAgent
from muto_command_layer_v2.contracts import CandidateEvidence, CompletionPolicy, MissionAction, SkillName, ToolName
from muto_command_layer_v2.executive import MissionExecutive
from muto_command_layer_v2.runtime import CommanderRuntime
from muto_command_layer_v2.tools import ToolCall, ToolDispatcher, ToolResult


class SearchBackend:
    def query_registry(self, call, board):
        return ToolResult(True, candidate_ids=("chair-1",), registry_revision="r1")

    def inspect_candidates(self, call, board):
        return ToolResult(
            True,
            candidate_ids=("chair-1",),
            confirmed_target_id="chair-1",
            registry_revision="r1",
            evidence=(CandidateEvidence("chair-1", "r1", evidence_id="img-chair-1", source="test", confidence=1.0),),
        )

    def observe(self, call, board):
        return ToolResult(True)

    def rotate_to_heading(self, call, board):
        return ToolResult(True)

    def go_to_point(self, call, board):
        return ToolResult(True)


def test_runtime_confirms_only_after_registry_then_inspection():
    def planner(board):
        if not board.shortlisted_candidate_ids:
            return {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {"name": "query_registry", "object_request": "purple chair"},
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
            "completion_proposal": "report_confirmed",
        }

    executive = MissionExecutive()
    runtime = CommanderRuntime(
        executive,
        CommanderAgent(planner),
        ToolDispatcher(SearchBackend()),
    )
    result = runtime.run(
        MissionAction(
            request_id="req-1",
            objective="find purple chair",
            object_request="purple chair",
            completion_policy=CompletionPolicy.REPORT_CONFIRMED,
        )
    )
    assert result.board.lifecycle_state.value == "succeeded"
    assert result.board.confirmed_target_id == "chair-1"
    assert [event.event_type.value for event in executive.events] == [
        "mission_accepted",
        "board_updated",
        "mission_started",
        "skill_selected",
        "tool_requested",
        "tool_result",
        "tool_requested",
        "tool_result",
        "candidate_confirmed",
        "mission_succeeded",
    ]


def test_runtime_stops_repeated_invalid_planner_output_without_mission_budget():
    executive = MissionExecutive()
    runtime = CommanderRuntime(
        executive,
        CommanderAgent(lambda _board: {"not": "a v2 decision"}),
        ToolDispatcher(SearchBackend()),
        consecutive_failure_limit=3,
    )
    result = runtime.run(
        MissionAction(
            request_id="req-invalid",
            objective="find purple chair",
            object_request="purple chair",
            completion_policy=CompletionPolicy.REPORT_CONFIRMED,
        )
    )
    assert result.board.lifecycle_state.value == "failed"
    assert result.board.last_reason_code == "consecutive_failures"
    assert result.decisions == 3


def test_runtime_rejects_completion_that_redefines_scenario_policy():
    def planner(_board):
        return {
            "schema_version": "muto_command_layer_v2",
            "skill": "search_for_object",
            "tool": None,
            "completion_proposal": "search_until_exhausted",
        }

    result = CommanderRuntime(
        MissionExecutive(),
        CommanderAgent(planner),
        ToolDispatcher(SearchBackend()),
        consecutive_failure_limit=1,
    ).run(
        MissionAction(
            request_id="req-policy-guard",
            objective="find a chair",
            object_request="chair",
            completion_policy=CompletionPolicy.REPORT_CONFIRMED,
        )
    )
    assert result.board.lifecycle_state.value == "failed"
    assert result.board.last_reason_code == "consecutive_failures"


def test_runtime_accepts_valid_not_found_exhaustion_as_success():
    def planner(board):
        if board.search_progress == 0.0:
            return {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {"name": "observe"},
            }
        return {
            "schema_version": "muto_command_layer_v2",
            "skill": "search_for_object",
            "tool": None,
            "completion_proposal": "search_until_exhausted",
            "rationale": "the scenario search space is exhausted",
        }

    class ExhaustedBackend(SearchBackend):
        def observe(self, call, board):
            return ToolResult(True, progress_delta=1.0, reason_code="frontier_exhausted")

    result = CommanderRuntime(
        MissionExecutive(),
        CommanderAgent(planner),
        ToolDispatcher(ExhaustedBackend()),
    ).run(
        MissionAction(
            request_id="req-not-found",
            objective="find the absent chair",
            object_request="absent chair",
            completion_policy=CompletionPolicy.SEARCH_UNTIL_EXHAUSTED,
        )
    )
    assert result.board.lifecycle_state.value == "succeeded"
    assert result.board.confirmed_target_id == ""
    assert result.board.last_reason_code == "search_exhausted"


def test_runtime_replans_after_one_backend_failure():
    calls = []

    class FlakyBackend(SearchBackend):
        def observe(self, call, board):
            calls.append(len(calls))
            if len(calls) == 1:
                return ToolResult(False, reason_code="nav2_navigation_aborted")
            return ToolResult(
                True,
                progress_delta=1.0,
                reason_code="frontier_exhausted",
            )

    def planner(board):
        if board.search_progress == 0.0:
            return {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {"name": "observe"},
            }
        return {
            "schema_version": "muto_command_layer_v2",
            "skill": "search_for_object",
            "tool": None,
            "completion_proposal": "search_until_exhausted",
        }

    result = CommanderRuntime(
        MissionExecutive(),
        CommanderAgent(planner),
        ToolDispatcher(FlakyBackend()),
    ).run(
        MissionAction(
            request_id="req-replan",
            objective="search the room",
            object_request="room",
            completion_policy=CompletionPolicy.SEARCH_UNTIL_EXHAUSTED,
        )
    )
    assert result.board.lifecycle_state.value == "succeeded"
    assert calls == [0, 1]
