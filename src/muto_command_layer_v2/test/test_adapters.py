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
                RegistryCandidate(
                    "chair-a", object_request, "registry-7",
                    metadata={"x": "1.25", "y": "-0.5", "frame_id": "map"},
                ),
                RegistryCandidate(
                    "chair-b", object_request, "registry-7",
                    metadata={"x": "2.0", "y": "0.5", "frame_id": "map"},
                ),
            ),
        )

    def inspect(self, object_request, snapshot, candidate_ids, board):
        return self.decisions


class Motion:
    def __init__(self):
        self.calls = []

    def observe(self, board):
        return MotionResult(True, reason_code="observed", progress_delta=0.2)

    def rotate_to_heading(self, heading, board):
        return MotionResult(True, reason_code="rotated", progress_delta=0.1)

    def go_to_point(self, point, projection_policy, board):
        self.calls.append((point, projection_policy))
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


def _confirmed_approach_board(backend):
    executive = _board()
    dispatcher = ToolDispatcher(backend)
    query = dispatcher.dispatch(ToolCall(ToolName.QUERY_REGISTRY), executive.board)
    executive.record_tool_result(
        ToolName.QUERY_REGISTRY,
        success=query.success,
        candidate_ids=query.candidate_ids,
        registry_revision=query.registry_revision,
    )
    inspection = dispatcher.dispatch(
        ToolCall(
            ToolName.INSPECT_CANDIDATES,
            candidate_ids=query.candidate_ids,
            registry_revision=query.registry_revision,
        ),
        executive.board,
    )
    executive.record_tool_result(
        ToolName.INSPECT_CANDIDATES,
        success=inspection.success,
        candidate_ids=inspection.candidate_ids,
        rejected_candidate_ids=inspection.rejected_candidate_ids,
        confirmed_target_id=inspection.confirmed_target_id,
        registry_revision=inspection.registry_revision,
        evidence=inspection.evidence,
    )
    executive.select_skill(SkillName.APPROACH_CONFIRMED_OBJECT)
    return executive, dispatcher


def test_approach_resolves_the_confirmed_registry_position_when_point_is_omitted():
    motion = Motion()
    backend = V2ToolBackend(
        Registry((CandidateDecision("chair-a", True, 0.9), CandidateDecision("chair-b", False))),
        motion,
    )
    executive, dispatcher = _confirmed_approach_board(backend)
    result = dispatcher.dispatch(
        ToolCall(ToolName.GO_TO_POINT, candidate_id="chair-a"),
        executive.board,
    )
    assert result.success
    assert motion.calls == [((1.25, -0.5), "allow")]
    assert result.detail.startswith("candidate_position_resolved:chair-a")


def test_approach_fails_closed_when_confirmed_position_is_missing():
    motion = Motion()

    class MissingPositionRegistry(Registry):
        def query(self, object_request, board):
            snapshot = super().query(object_request, board)
            return RegistrySnapshot(
                revision=snapshot.revision,
                candidates=tuple(
                    RegistryCandidate(
                        item.candidate_id,
                        item.label,
                        item.registry_revision,
                    )
                    for item in snapshot.candidates
                ),
            )

    backend = V2ToolBackend(
        MissingPositionRegistry(
            (CandidateDecision("chair-a", True), CandidateDecision("chair-b", False))
        ),
        motion,
    )
    executive, dispatcher = _confirmed_approach_board(backend)
    result = dispatcher.dispatch(
        ToolCall(ToolName.GO_TO_POINT, candidate_id="chair-a"),
        executive.board,
    )
    assert not result.success
    assert result.reason_code == "candidate_position_unavailable"
    assert motion.calls == []


def test_approach_rejects_a_coordinate_that_is_not_the_confirmed_candidate():
    motion = Motion()
    backend = V2ToolBackend(
        Registry(
            (CandidateDecision("chair-a", True), CandidateDecision("chair-b", False))
        ),
        motion,
    )
    executive, dispatcher = _confirmed_approach_board(backend)
    result = dispatcher.dispatch(
        ToolCall(
            ToolName.GO_TO_POINT,
            candidate_id="chair-a",
            point=(99.0, 99.0),
        ),
        executive.board,
    )
    assert not result.success
    assert result.reason_code == "candidate_position_mismatch"
    assert motion.calls == []
