"""Contract and state-transition tests for the independent v2 slice."""

import pytest

from muto_command_layer_v2.contracts import (
    CompletionPolicy,
    CandidateEvidence,
    ContractError,
    EventType,
    LifecycleState,
    MissionAction,
    ReachabilityReport,
    ReachabilityState,
    SkillName,
    ToolName,
)
from muto_command_layer_v2.executive import (
    DuplicateMissionError,
    MissionExecutive,
    TerminalMissionError,
)


def make_action(policy=CompletionPolicy.REPORT_CONFIRMED):
    return MissionAction(
        request_id="request-1",
        objective="find the purple chair",
        object_request="purple chair",
        completion_policy=policy,
    )


def started_executive(policy=CompletionPolicy.REPORT_CONFIRMED):
    executive = MissionExecutive()
    executive.accept(make_action(policy))
    executive.start()
    return executive


def test_mission_action_is_strict_and_has_no_scenario_field():
    action = make_action()
    assert action.schema_version == "muto_command_layer_v2"
    assert not hasattr(action, "scenario_id")

    with pytest.raises(ContractError):
        MissionAction(
            request_id="request-1",
            objective="find it",
            completion_policy="unsupported",
        )


def test_reachability_report_is_one_typed_result():
    report = ReachabilityReport(
        state=ReachabilityState.REACHABLE,
        reason_code="preflight_reachable",
        path_length_m=1.25,
        estimated_time_s=4.0,
        costmap_revision=7,
        freshness="fresh",
        selected_pose=(1.0, 2.0, 3.14),
    )
    assert report.state is ReachabilityState.REACHABLE
    assert report.projected is False

    with pytest.raises(ContractError):
        ReachabilityReport(path_length_m=-1.0)


def test_executive_accepts_one_active_mission_and_emits_ordered_events():
    executive = MissionExecutive()
    board = executive.accept(make_action())
    assert board.lifecycle_state is LifecycleState.ACCEPTED
    executive.start()
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=False,
        reason_code="registry_unavailable",
    )

    assert executive.board.lifecycle_state is LifecycleState.RUNNING
    assert executive.board.consecutive_failures == 1
    assert [event.sequence for event in executive.events] == [1, 2, 3, 4]
    assert executive.events[-1].event_type is EventType.TOOL_RESULT

    with pytest.raises(DuplicateMissionError):
        executive.accept(MissionAction(
            request_id="request-2",
            objective="find another object",
            completion_policy=CompletionPolicy.REPORT_CONFIRMED,
        ))

    executive.record_tool_result(ToolName.OBSERVE, success=True, progress_delta=0.5)
    assert executive.board.consecutive_failures == 0
    assert executive.board.no_progress_count == 0


def test_board_observation_keeps_robot_pose_on_the_canonical_board():
    executive = started_executive()
    executive.record_board_observation(robot_pose=(1.0, -2.0, 3.14))
    assert executive.board.robot_pose == (1.0, -2.0, 3.14)
    assert executive.events[-1].event_type is EventType.BOARD_UPDATED


def test_tool_scope_is_enforced_without_a_second_command_layer():
    executive = started_executive()
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-1",),
        registry_revision="r1",
    )
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_ids=("candidate-1",),
        confirmed_target_id="candidate-1",
        registry_revision="r1",
        evidence=(CandidateEvidence("candidate-1", "r1", evidence_id="img-1", source="test", confidence=1.0),),
    )
    executive.select_skill(SkillName.APPROACH_CONFIRMED_OBJECT)
    with pytest.raises(ContractError):
        executive.record_tool_result(ToolName.QUERY_REGISTRY, success=True)


def test_child_failure_is_nonfatal_and_confirmation_is_explicit():
    executive = started_executive()
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-2",),
        registry_revision="r7",
    )
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=False,
        reason_code="candidate_rejected",
        candidate_ids=("candidate-2",),
        rejected_candidate_ids=("candidate-2",),
        registry_revision="r7",
    )
    assert executive.board.lifecycle_state is LifecycleState.RUNNING
    assert executive.board.confirmed_target_id == ""

    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-2",),
        registry_revision="r8",
    )
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_id="candidate-2",
        confirmed_target_id="candidate-2",
        candidate_ids=("candidate-2",),
        registry_revision="r8",
        evidence=(CandidateEvidence("candidate-2", "r8", evidence_id="img-2", source="test", confidence=1.0),),
    )
    assert executive.board.confirmed_target_id == "candidate-2"


def test_completion_policy_rejects_premature_finish_and_accepts_confirmation():
    executive = started_executive(CompletionPolicy.REPORT_CONFIRMED)
    with pytest.raises(ContractError):
        executive.complete()
    with pytest.raises(ContractError):
        executive.complete(confirmed_target_id="unconfirmed")

    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-1",),
        registry_revision="r3",
    )
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_id="candidate-1",
        confirmed_target_id="candidate-1",
        candidate_ids=("candidate-1",),
        registry_revision="r3",
        evidence=(CandidateEvidence("candidate-1", "r3", evidence_id="img-1", source="test", confidence=1.0),),
    )
    board = executive.complete()
    assert board.lifecycle_state is LifecycleState.SUCCEEDED
    assert executive.events[-1].event_type is EventType.MISSION_SUCCEEDED

    with pytest.raises(TerminalMissionError):
        executive.select_skill(SkillName.SEARCH_FOR_OBJECT)


def test_search_exhaustion_completion_requires_authority_evidence():
    executive = started_executive(CompletionPolicy.SEARCH_UNTIL_EXHAUSTED)
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(
        ToolName.OBSERVE,
        success=True,
        progress_delta=1.0,
        reason_code="poi_goal_succeeded",
    )
    with pytest.raises(ContractError, match="explicit exhaustion evidence"):
        executive.complete(search_exhausted=True)

    executive.record_tool_result(
        ToolName.OBSERVE,
        success=True,
        progress_delta=1.0,
        reason_code="poi_exhausted",
    )
    assert executive.complete(search_exhausted=True).lifecycle_state is LifecycleState.SUCCEEDED


def test_cancel_is_direct_and_terminal():
    executive = started_executive()
    board = executive.cancel()
    assert board.lifecycle_state is LifecycleState.CANCELED
    assert executive.events[-1].event_type is EventType.MISSION_CANCELED
    assert executive.cancel().lifecycle_state is LifecycleState.CANCELED
