"""Trace-oriented contract scenarios required by the v2 constitution."""

import threading
import time

from muto_command_layer_v2.commander import CommanderAgent
from muto_command_layer_v2.contracts import (
    CandidateEvidence,
    CompletionPolicy,
    MissionAction,
    SkillName,
    ToolName,
)
from muto_command_layer_v2.executive import MissionExecutive
from muto_command_layer_v2.ros_authorities import _shortest_angle
from muto_command_layer_v2.runtime import CommanderRuntime
from muto_command_layer_v2.tools import ToolCall, ToolDispatcher, ToolResult


class RevisionBackend:
    def query_registry(self, _call, _board):
        return ToolResult(
            True,
            candidate_ids=("candidate-a", "candidate-b"),
            registry_revision="r1",
        )

    def inspect_candidates(self, call, _board):
        if call.candidate_ids == ("candidate-a", "candidate-b"):
            return ToolResult(
                False,
                reason_code="candidate_rejected",
                candidate_ids=call.candidate_ids,
                rejected_candidate_ids=("candidate-a",),
                registry_revision="r1",
                evidence=(
                    CandidateEvidence("candidate-a", "r1", "img-a", "fixture", 0.1),
                    CandidateEvidence("candidate-b", "r1", "img-b", "fixture", 0.2),
                ),
            )
        return ToolResult(
            True,
            reason_code="candidate_confirmed",
            candidate_ids=call.candidate_ids,
            confirmed_target_id="candidate-b",
            registry_revision="r1",
            evidence=(CandidateEvidence(
                "candidate-b", "r1", "img-b", "fixture", 0.95,
                matched_attributes=("chair",),
            ),),
        )

    def observe(self, _call, _board):
        return ToolResult(True, progress_delta=1.0, detail="coverage advanced")

    def rotate_to_heading(self, _call, _board):
        return ToolResult(True, progress_delta=1.0)

    def go_to_point(self, _call, _board):
        return ToolResult(True, progress_delta=1.0)


def test_multiple_candidate_rejection_then_confirmation_is_traceable():
    def planner(board):
        if not board.shortlisted_candidate_ids:
            return {"schema_version": "muto_command_layer_v2", "skill": "search_for_object",
                    "tool": {"name": "query_registry"}}
        if not board.confirmed_target_id:
            return {"schema_version": "muto_command_layer_v2", "skill": "search_for_object",
                    "tool": {"name": "inspect_candidates",
                             "candidate_ids": list(board.shortlisted_candidate_ids),
                             "registry_revision": board.registry_revision}}
        return {"schema_version": "muto_command_layer_v2", "skill": "search_for_object",
                "completion_proposal": "report_confirmed"}

    executive = MissionExecutive()
    result = CommanderRuntime(
        executive, CommanderAgent(planner), ToolDispatcher(RevisionBackend())
    ).run(MissionAction("scenario-rejection", "find chair", CompletionPolicy.REPORT_CONFIRMED, "chair"))
    assert result.board.confirmed_target_id == "candidate-b"
    assert result.board.rejected_candidate_ids == ("candidate-a",)
    assert result.board.visual_evidence_summary
    assert [event.event_type.value for event in executive.events].count("candidate_rejected") == 1


def test_revision_stale_inspection_cannot_replace_newer_shortlist():
    executive = MissionExecutive()
    executive.accept(MissionAction("revision", "find chair", CompletionPolicy.REPORT_CONFIRMED, "chair"))
    executive.start()
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(ToolName.QUERY_REGISTRY, success=True,
                                 candidate_ids=("old",), registry_revision="r1")
    executive.record_tool_result(ToolName.QUERY_REGISTRY, success=True,
                                 candidate_ids=("new",), registry_revision="r2")
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_ids=("old",),
        confirmed_target_id="old",
        registry_revision="r1",
        expected_registry_revision="r1",
        evidence=(CandidateEvidence(
            "old", "r1", "img-old", "fixture", 0.9,
            matched_attributes=("chair",),
        ),),
    )
    assert executive.board.shortlisted_candidate_ids == ("new",)
    assert executive.board.confirmed_target_id == ""
    assert executive.board.last_reason_code == "stale_registry_result"


def test_same_registry_refresh_preserves_rejection_and_confirmation_evidence():
    executive = MissionExecutive()
    executive.accept(MissionAction("refresh", "find chair", CompletionPolicy.REPORT_CONFIRMED, "chair"))
    executive.start()
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-a", "candidate-b"),
        registry_revision="stable",
    )
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=False,
        reason_code="candidate_rejected",
        candidate_ids=("candidate-a", "candidate-b"),
        rejected_candidate_ids=("candidate-a",),
        registry_revision="stable",
        evidence=(CandidateEvidence("candidate-a", "stable", "img-a", "fixture", 0.1),),
    )
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-a", "candidate-b"),
        registry_revision="stable",
    )
    assert executive.board.rejected_candidate_ids == ("candidate-a",)
    assert executive.board.candidate_evidence[0].candidate_id == "candidate-a"

    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=True,
        candidate_ids=("candidate-b",),
        confirmed_target_id="candidate-b",
        registry_revision="stable",
        evidence=(CandidateEvidence(
            "candidate-b", "stable", "img-b", "fixture", 0.95,
            matched_attributes=("chair",),
        ),),
    )
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=True,
        candidate_ids=("candidate-a", "candidate-b"),
        registry_revision="stable",
    )
    assert executive.board.confirmed_target_id == "candidate-b"
    assert executive.board.confirmed_registry_revision == "stable"


def test_exact_180_degree_turn_has_one_deterministic_direction():
    assert _shortest_angle(3.141592653589793) == 3.141592653589793
    assert _shortest_angle(-3.141592653589793) == 3.141592653589793


def test_cancellation_propagates_while_go_to_point_is_active():
    started = threading.Event()
    release = threading.Event()
    canceled = threading.Event()

    class BlockingBackend(RevisionBackend):
        def query_registry(self, _call, _board):
            return ToolResult(True, candidate_ids=("candidate-a",), registry_revision="r1")

        def inspect_candidates(self, call, _board):
            return ToolResult(
                True,
                candidate_ids=call.candidate_ids,
                confirmed_target_id="candidate-a",
                registry_revision="r1",
                evidence=(CandidateEvidence(
                    "candidate-a", "r1", "img-a", "fixture", 0.95,
                    matched_attributes=("chair",),
                ),),
            )

        def go_to_point(self, _call, _board):
            started.set()
            release.wait(2.0)
            return ToolResult(False, reason_code="motion_canceled")

        def cancel_active(self):
            canceled.set()
            release.set()

    def planner(board):
        if not board.shortlisted_candidate_ids:
            return {"schema_version": "muto_command_layer_v2", "skill": "search_for_object",
                    "tool": {"name": "query_registry"}}
        if not board.confirmed_target_id:
            return {"schema_version": "muto_command_layer_v2", "skill": "search_for_object",
                    "tool": {"name": "inspect_candidates", "candidate_ids": list(board.shortlisted_candidate_ids),
                             "registry_revision": board.registry_revision}}
        return {"schema_version": "muto_command_layer_v2", "skill": "approach_confirmed_object",
                "tool": {"name": "go_to_point", "point": [1.0, 0.0], "candidate_id": "candidate-a"}}

    executive = MissionExecutive()
    runtime = CommanderRuntime(
        executive, CommanderAgent(planner), ToolDispatcher(BlockingBackend())
    )
    thread = threading.Thread(
        target=lambda: runtime.run(MissionAction("cancel", "approach chair", CompletionPolicy.APPROACH_CONFIRMED, "chair")),
        daemon=True,
    )
    thread.start()
    assert started.wait(2.0)
    runtime.request_cancel()
    thread.join(2.0)
    assert canceled.is_set()
    assert executive.board.lifecycle_state.value == "canceled"
