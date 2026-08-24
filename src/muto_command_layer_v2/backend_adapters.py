"""Independent authority contracts and the v2 tool adapter.

The command layer owns neither object detection, exploration, nor navigation.
Those authorities are injected through these small protocols.  ROS Humble
implementations can wrap the existing registry/frontier/Nav2 nodes without
importing the old command-layer package.
"""

from dataclasses import dataclass, field
from typing import Mapping, Protocol, Sequence, Tuple

from .contracts import CandidateEvidence, MissionBoard, ReachabilityReport
from .tools import ToolBackend, ToolCall, ToolResult


@dataclass(frozen=True)
class RegistryCandidate:
    candidate_id: str
    label: str
    registry_revision: str
    evidence_id: str = ""
    metadata: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class RegistrySnapshot:
    revision: str
    candidates: Tuple[RegistryCandidate, ...] = ()
    checked: bool = True


@dataclass(frozen=True)
class CandidateDecision:
    candidate_id: str
    confirmed: bool
    confidence: float = 0.0
    reason_code: str = ""
    source: str = ""
    evidence_id: str = ""
    observed_at_s: float = 0.0


@dataclass(frozen=True)
class MotionResult:
    success: bool
    reason_code: str = ""
    detail: str = ""
    progress_delta: float = 0.0
    reachability: ReachabilityReport = ReachabilityReport()


class RegistryAuthority(Protocol):
    def query(self, object_request: str, board: MissionBoard) -> RegistrySnapshot: ...

    def inspect(
        self,
        object_request: str,
        snapshot: RegistrySnapshot,
        candidate_ids: Sequence[str],
        board: MissionBoard,
    ) -> Sequence[CandidateDecision]: ...


class MotionAuthority(Protocol):
    def observe(self, board: MissionBoard) -> MotionResult: ...

    def rotate_to_heading(self, heading: float, board: MissionBoard) -> MotionResult: ...

    def go_to_point(
        self,
        point: Tuple[float, float],
        projection_policy: str,
        board: MissionBoard,
    ) -> MotionResult: ...


class V2ToolBackend(ToolBackend):
    """Translate the five commander tools into independent authorities."""

    def __init__(self, registry: RegistryAuthority, motion: MotionAuthority) -> None:
        self._registry = registry
        self._motion = motion
        self._snapshot = RegistrySnapshot(revision="", candidates=(), checked=False)

    def query_registry(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        snapshot = self._registry.query(call.object_request or board.object_request, board)
        self._snapshot = snapshot
        candidates = tuple(candidate.candidate_id for candidate in snapshot.candidates)
        return ToolResult(
            success=snapshot.checked,
            reason_code="registry_checked" if snapshot.checked else "registry_unavailable",
            candidate_ids=candidates,
            registry_revision=snapshot.revision,
        )

    def inspect_candidates(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        if not self._snapshot.checked or not self._snapshot.revision:
            return ToolResult(False, reason_code="registry_snapshot_unavailable")
        # Inspection is evidence-scoped.  An omitted revision is not allowed
        # to silently reuse whichever shortlist happens to be cached.
        if not call.registry_revision:
            return ToolResult(False, reason_code="registry_revision_required")
        if call.registry_revision != self._snapshot.revision:
            return ToolResult(False, reason_code="registry_revision_mismatch")
        allowed = {candidate.candidate_id for candidate in self._snapshot.candidates}
        requested = tuple(call.candidate_ids)
        if not requested or any(candidate_id not in allowed for candidate_id in requested):
            return ToolResult(False, reason_code="candidate_not_in_snapshot")
        decisions = tuple(
            self._registry.inspect(
                call.object_request or board.object_request,
                self._snapshot,
                requested,
                board,
            )
        )
        decision_ids = {decision.candidate_id for decision in decisions}
        if decision_ids != set(requested):
            return ToolResult(False, reason_code="inspection_incomplete")
        confirmed = [decision for decision in decisions if decision.confirmed]
        rejected = tuple(
            decision.candidate_id for decision in decisions if not decision.confirmed
        )
        if len(confirmed) > 1:
            return ToolResult(
                False,
                reason_code="multiple_candidates_confirmed",
                candidate_ids=requested,
                rejected_candidate_ids=rejected,
                registry_revision=self._snapshot.revision,
                evidence=tuple(
                    _candidate_evidence(decision, self._snapshot.revision)
                    for decision in decisions
                ),
            )
        if not confirmed:
            return ToolResult(
                False,
                reason_code="candidate_rejected",
                candidate_ids=requested,
                rejected_candidate_ids=rejected,
                registry_revision=self._snapshot.revision,
                evidence=tuple(_candidate_evidence(decision, self._snapshot.revision) for decision in decisions),
            )
        return ToolResult(
            True,
            reason_code="candidate_confirmed",
            candidate_ids=requested,
            confirmed_target_id=confirmed[0].candidate_id,
            rejected_candidate_ids=rejected,
            registry_revision=self._snapshot.revision,
            evidence=tuple(_candidate_evidence(decision, self._snapshot.revision) for decision in decisions),
        )

    def observe(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        return _motion_tool_result(self._motion.observe(board))

    def rotate_to_heading(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        if call.heading is None:
            return ToolResult(False, reason_code="invalid_heading")
        return _motion_tool_result(self._motion.rotate_to_heading(call.heading, board))

    def go_to_point(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        if call.point is None:
            return ToolResult(False, reason_code="invalid_point")
        return _motion_tool_result(
            self._motion.go_to_point(call.point, call.projection_policy, board)
        )

    def cancel_active(self) -> None:
        for authority in (self._motion, self._registry):
            cancel = getattr(authority, "cancel_active", None)
            if callable(cancel):
                cancel()


def _motion_tool_result(result: MotionResult) -> ToolResult:
    return ToolResult(
        success=result.success,
        reason_code=result.reason_code,
        detail=result.detail,
        progress_delta=result.progress_delta,
        reachability=result.reachability,
    )


def _candidate_evidence(decision: CandidateDecision, revision: str) -> CandidateEvidence:
    return CandidateEvidence(
        candidate_id=decision.candidate_id,
        registry_revision=revision,
        evidence_id=decision.evidence_id,
        source=decision.source,
        confidence=decision.confidence,
        observed_at_s=decision.observed_at_s or None,
        reason_code=decision.reason_code,
    )
