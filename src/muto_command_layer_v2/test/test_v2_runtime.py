"""Focused tests for the v2 boundaries beyond the executive state machine."""

import json

import pytest

from muto_command_layer_v2.contracts import (
    CandidateEvidence,
    CompletionPolicy,
    ContractError,
    LifecycleState,
    MissionAction,
    ReachabilityState,
    SkillName,
    ToolName,
)
from muto_command_layer_v2.commander import PlannerFailure, parse_decision
from muto_command_layer_v2.executive import MissionExecutive
from muto_command_layer_v2.natural_language import NaturalLanguageAdapter
from muto_command_layer_v2.reachability import (
    OccupancyGrid,
    ReachabilityConfig,
    ReachabilityPlanner,
)
from muto_command_layer_v2.recorder import HighLevelRecorder
from muto_command_layer_v2.tools import ToolCall, ToolDispatcher, ToolResult


class Backend:
    def query_registry(self, call, board):
        return ToolResult(True, candidate_ids=("c1",), registry_revision="r1")

    def inspect_candidates(self, call, board):
        return ToolResult(True, confirmed_target_id="c1", candidate_ids=("c1",))

    def observe(self, call, board):
        return ToolResult(True, progress_delta=0.1)

    def rotate_to_heading(self, call, board):
        return ToolResult(True, progress_delta=0.1)

    def go_to_point(self, call, board):
        return ToolResult(True, progress_delta=1.0)


def _running(skill=SkillName.SEARCH_FOR_OBJECT):
    ex = MissionExecutive()
    ex.accept(MissionAction("r1", "find chair", CompletionPolicy.REPORT_CONFIRMED, "chair"))
    ex.start()
    ex.select_skill(SkillName.SEARCH_FOR_OBJECT)
    if skill is SkillName.APPROACH_CONFIRMED_OBJECT:
        ex.record_tool_result(
            ToolName.QUERY_REGISTRY,
            success=True,
            candidate_ids=("c1",),
            registry_revision="r1",
        )
        ex.record_tool_result(
            ToolName.INSPECT_CANDIDATES,
            success=True,
            candidate_ids=("c1",),
            confirmed_target_id="c1",
            registry_revision="r1",
            evidence=(CandidateEvidence("c1", "r1", evidence_id="img-c1", source="test", confidence=1.0),),
        )
        ex.select_skill(skill)
    return ex


def _grid(data, width=3, height=3, revision=1, freshness="fresh"):
    return OccupancyGrid(width, height, 1.0, 0.0, 0.0, tuple(data), revision, freshness)


def test_natural_language_rejects_unsupported_and_normalizes_cancel():
    adapter = NaturalLanguageAdapter()
    action = adapter.normalize("please find the purple chair", request_id="r1")
    assert action.object_request == "purple chair"
    assert action.completion_policy is CompletionPolicy.REPORT_CONFIRMED
    approach = adapter.normalize("please go to the chair", request_id="r1a")
    assert approach.completion_policy is CompletionPolicy.APPROACH_CONFIRMED
    cancel = adapter.normalize("stop", request_id="r2")
    assert cancel.reason_code == "cancel_requested"
    rejected = adapter.normalize("dance around", request_id="r3")
    assert rejected.reason_code == "unsupported_request"


def test_commander_json_is_strict_and_skill_scoped():
    board = _running().board
    decision = parse_decision(
        {
            "schema_version": "muto_command_layer_v2",
            "skill": "search_for_object",
            "tool": {"name": "observe"},
        },
        board,
    )
    assert decision.tool.tool is ToolName.OBSERVE
    with pytest.raises(PlannerFailure):
        parse_decision(
            {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "unexpected": True,
            },
            board,
        )
    approach_board = _running(SkillName.APPROACH_CONFIRMED_OBJECT).board
    with pytest.raises(PlannerFailure):
        parse_decision(
            {
                "schema_version": "muto_command_layer_v2",
                "skill": "approach_confirmed_object",
                "tool": {"name": "query_registry"},
            },
            approach_board,
        )
    with pytest.raises(PlannerFailure):
        parse_decision(
            {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {"name": "observe"},
                "completion_proposal": "search_until_exhausted",
            },
            board,
        )


def test_static_dispatcher_requires_confirmation_for_approach():
    ex = _running()
    ex._board = ex.board.evolve(
        active_skill=SkillName.APPROACH_CONFIRMED_OBJECT,
        confirmed_target_id="",
        confirmed_registry_revision="",
    )
    dispatcher = ToolDispatcher(Backend())
    with pytest.raises(ContractError):
        dispatcher.dispatch(ToolCall(ToolName.GO_TO_POINT, point=(1.0, 1.0)), ex.board)

    ex._board = ex.board.evolve(
        registry_revision="r1",
        confirmed_target_id="c1",
        confirmed_registry_revision="r1",
    )  # test fixture only
    result = dispatcher.dispatch(
        ToolCall(ToolName.GO_TO_POINT, point=(1.0, 1.0), candidate_id="c1"),
        ex.board,
    )
    assert result.success

    with pytest.raises(ContractError):
        dispatcher.dispatch(
            ToolCall(ToolName.GO_TO_POINT, point=(1.0, 1.0)),
            ex.board,
        )

    # A confirmed candidate ID is sufficient for the backend to resolve the
    # authoritative registry position; the model need not invent coordinates.
    result = dispatcher.dispatch(
        ToolCall(ToolName.GO_TO_POINT, candidate_id="c1"),
        ex.board,
    )
    assert result.success


def test_commander_allows_only_confirmed_search_to_approach_handoff():
    ex = _running()
    ex.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("c1",),
        registry_revision="r1",
    )
    ex.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_ids=("c1",),
        confirmed_target_id="c1",
        registry_revision="r1",
        evidence=(CandidateEvidence("c1", "r1", evidence_id="img-c1", source="test", confidence=1.0),),
    )
    with pytest.raises(PlannerFailure):
        parse_decision(
            {
                "schema_version": "muto_command_layer_v2",
                "skill": "approach_confirmed_object",
                "tool": {"name": "observe"},
            },
            ex.board,
        )
    decision = parse_decision(
        {
            "schema_version": "muto_command_layer_v2",
            "skill": "approach_confirmed_object",
            "tool": {"name": "go_to_point", "candidate_id": "c1"},
        },
        ex.board,
    )
    assert decision.skill is SkillName.APPROACH_CONFIRMED_OBJECT
    ex_unconfirmed = _running()
    with pytest.raises(PlannerFailure):
        parse_decision(
            {
                "schema_version": "muto_command_layer_v2",
                "skill": "approach_confirmed_object",
                "tool": {"name": "observe"},
            },
            ex_unconfirmed.board,
        )


def test_search_cannot_bypass_poi_grid_with_raw_navigation():
    board = _running().board
    with pytest.raises(PlannerFailure):
        parse_decision(
            {
                "schema_version": "muto_command_layer_v2",
                "skill": "search_for_object",
                "tool": {"name": "go_to_point", "point": [1.0, 0.0]},
            },
            board,
        )


def test_failed_invalid_approach_result_is_nonfatal_board_evidence():
    ex = _running(SkillName.APPROACH_CONFIRMED_OBJECT)
    ex._board = ex.board.evolve(
        registry_revision="r1",
        confirmed_target_id="c1",
        confirmed_registry_revision="r1",
    )
    ex.record_tool_result(
        ToolName.GO_TO_POINT,
        success=False,
        reason_code="confirmed_target_mismatch",
        candidate_id="wrong-candidate",
    )
    assert ex.board.lifecycle_state.value == "running"
    assert ex.board.last_reason_code == "confirmed_target_mismatch"


def test_registry_revision_replaces_shortlist_and_old_confirmation():
    ex = _running()
    ex.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("old",),
        registry_revision="r1",
    )
    ex.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_ids=("old",),
        confirmed_target_id="old",
        registry_revision="r1",
        evidence=(CandidateEvidence("old", "r1", evidence_id="img-old", source="test", confidence=1.0),),
    )
    assert ex.board.confirmed_target_id == "old"
    ex.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("new",),
        registry_revision="r2",
    )
    assert ex.board.shortlisted_candidate_ids == ("new",)
    assert ex.board.confirmed_target_id == ""


def test_candidate_rejection_is_an_append_only_event():
    ex = _running()
    ex.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("c1", "c2"),
        registry_revision="r1",
    )
    ex.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=False,
        reason_code="candidate_rejected",
        candidate_ids=("c1", "c2"),
        rejected_candidate_ids=("c1", "c2"),
        registry_revision="r1",
    )
    rejected = [
        event for event in ex.events
        if event.event_type.value == "candidate_rejected"
    ]
    assert [event.candidate_id for event in rejected] == ["c1", "c2"]
    assert ex.board.rejected_candidate_ids == ("c1", "c2")


def test_reachability_blocks_diagonal_corner_and_unknown_goal():
    planner = ReachabilityPlanner()
    # Start at (0,0), goal at (2,2), with both cardinal exits blocked.
    corner = _grid([0, 100, 0, 100, 0, 0, 0, 0, 0])
    report = planner.evaluate(corner, (0.5, 0.5), (2.5, 2.5))
    assert report.state is ReachabilityState.UNREACHABLE
    assert report.reason_code == "preflight_disconnected"

    unknown = _grid([0, 0, 0, 0, -1, 0, 0, 0, 0])
    report = planner.evaluate(unknown, (0.5, 0.5), (1.5, 1.5))
    assert report.reason_code == "preflight_unknown_space"


def test_reachability_projection_and_stale_costmap():
    planner = ReachabilityPlanner()
    blocked_goal = _grid([0, 0, 0, 0, 100, 0, 0, 0, 0])
    report = planner.evaluate(
        blocked_goal, (0.5, 0.5), (1.5, 1.5), projection_policy="allow"
    )
    assert report.state is ReachabilityState.REACHABLE
    assert report.projected is True
    assert report.reason_code == "preflight_goal_projected"
    stale = _grid([0] * 9, freshness="stale")
    assert planner.evaluate(stale, (0.5, 0.5), (1.5, 1.5)).state is ReachabilityState.UNKNOWN


def test_reachability_rejects_gap_narrower_than_robot_footprint():
    # 0.1 m cells with a 0.42 m opening between two obstacles.  The plant and
    # Nav2 use a 0.26 m radius (0.52 m diameter), so a 0.26 m preflight must
    # not claim that the opening is traversable.
    width, height = 40, 80
    values = [0] * (width * height)
    for y in range(height):
        for x in range(width):
            world_x = -2.0 + (x + 0.5) * 0.1
            world_y = -4.0 + (y + 0.5) * 0.1
            if 0.4 <= world_x <= 0.8 and (world_y <= -0.16 or world_y >= 0.26):
                values[y * width + x] = 100
    planner = ReachabilityPlanner(
        config=ReachabilityConfig(footprint_radius_m=0.26)
    )
    grid = OccupancyGrid(width, height, 0.1, -2.0, -4.0, tuple(values), 1, "fresh")
    report = planner.evaluate(grid, (0.0, 0.0), (1.5, 0.0))
    assert report.state is ReachabilityState.UNREACHABLE


def test_recorder_is_passive_and_serializes_only_typed_records(tmp_path):
    ex = _running()
    recorder = HighLevelRecorder(str(tmp_path / "mission.jsonl"))
    recorder.start(ex.board)
    recorder.record_event(ex.events[0])
    recorder.close()
    lines = (tmp_path / "mission.jsonl").read_text(encoding="utf-8").splitlines()
    assert [json.loads(line)["kind"] for line in lines] == ["manifest", "board", "event"]
    assert recorder.available
