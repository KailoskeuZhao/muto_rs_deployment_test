"""Tests for the independent registry/motion authority adapter boundary."""

from muto_command_layer_v2.backend_adapters import (
    CandidateDecision,
    MotionResult,
    RegistryCandidate,
    RegistrySnapshot,
    V2ToolBackend,
)
from muto_command_layer_v2.contracts import CompletionPolicy, MissionAction, SkillName, ToolName
from muto_command_layer_v2.contracts import CandidateEvidence
from muto_command_layer_v2.executive import MissionExecutive
from muto_command_layer_v2.tools import ToolCall, ToolDispatcher


class Registry:
    def __init__(self, decisions):
        self.decisions = tuple(decisions)

    def query(self, object_request, board):
        return RegistrySnapshot(
            revision="registry-7",
            candidates=(
                RegistryCandidate("chair-a", object_request, "registry-7"),
                RegistryCandidate("chair-b", object_request, "registry-7"),
            ),
        )

    def inspect(self, object_request, snapshot, candidate_ids, board):
        return self.decisions


class Motion:
    def observe(self, board):
        return MotionResult(True, reason_code="observed", progress_delta=0.2)

    def rotate_to_heading(self, heading, board):
        return MotionResult(True, reason_code="rotated", progress_delta=0.1)

    def go_to_point(self, point, projection_policy, board):
        return MotionResult(True, reason_code="nav2_succeeded", progress_delta=1.0)


def _board():
    executive = MissionExecutive()
    executive.accept(
        MissionAction(
            "r1", "find chair", CompletionPolicy.REPORT_CONFIRMED, "chair"
        )
    )
    executive.start()
    executive.select_skill(SkillName.SEARCH_FOR_OBJECT)
    return executive


def test_adapter_preserves_registry_revision_and_confirmation_chain():
    executive = _board()
    backend = V2ToolBackend(
        Registry((CandidateDecision("chair-a", True, 0.9), CandidateDecision("chair-b", False))),
        Motion(),
    )
    dispatcher = ToolDispatcher(backend)
    query = dispatcher.dispatch(
        ToolCall(ToolName.QUERY_REGISTRY, object_request="chair"), executive.board
    )
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=query.success,
        candidate_ids=query.candidate_ids,
        registry_revision=query.registry_revision,
    )
    inspect = dispatcher.dispatch(
        ToolCall(
            ToolName.INSPECT_CANDIDATES,
            object_request="chair",
            candidate_ids=query.candidate_ids,
            registry_revision=query.registry_revision,
        ),
        executive.board,
    )
    assert inspect.confirmed_target_id == "chair-a"
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=inspect.success,
        candidate_ids=inspect.candidate_ids,
        rejected_candidate_ids=inspect.rejected_candidate_ids,
        confirmed_target_id=inspect.confirmed_target_id,
        registry_revision=inspect.registry_revision,
        evidence=inspect.evidence,
    )
    assert executive.board.confirmed_target_id == "chair-a"
    assert executive.board.confirmed_registry_revision == "registry-7"


def test_adapter_refuses_multiple_confirmations_without_promoting_one():
    executive = _board()
    backend = V2ToolBackend(
        Registry((CandidateDecision("chair-a", True), CandidateDecision("chair-b", True))),
        Motion(),
    )
    dispatcher = ToolDispatcher(backend)
    query = dispatcher.dispatch(ToolCall(ToolName.QUERY_REGISTRY), executive.board)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=query.success,
        candidate_ids=query.candidate_ids,
        registry_revision=query.registry_revision,
    )
    result = dispatcher.dispatch(
        ToolCall(
            ToolName.INSPECT_CANDIDATES,
            candidate_ids=query.candidate_ids,
            registry_revision=query.registry_revision,
        ),
        executive.board,
    )
    assert result.success is False
    assert result.reason_code == "multiple_candidates_confirmed"
    assert result.confirmed_target_id == ""
    assert {item.candidate_id for item in result.evidence} == {"chair-a", "chair-b"}


def test_adapter_requires_revision_for_candidate_inspection():
    executive = _board()
    backend = V2ToolBackend(
        Registry((CandidateDecision("chair-a", True),)),
        Motion(),
    )
    dispatcher = ToolDispatcher(backend)
    query = dispatcher.dispatch(ToolCall(ToolName.QUERY_REGISTRY), executive.board)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=query.success,
        candidate_ids=query.candidate_ids,
        registry_revision=query.registry_revision,
    )
    result = dispatcher.dispatch(
        ToolCall(
            ToolName.INSPECT_CANDIDATES,
            candidate_ids=query.candidate_ids,
        ),
        executive.board,
    )
    assert result.success is False
    assert result.reason_code == "registry_revision_required"
