"""Independent authority contracts and the v2 tool adapter.

The command layer owns neither object detection nor navigation.  Registry and
POI-grid search are injected through these small protocols; Nav2 remains the
navigation authority and is never replaced by the command layer.
"""

from dataclasses import dataclass, field, replace
from math import isfinite
from typing import Mapping, Protocol, Sequence, Tuple

from .contracts import (
    CandidateEvidence,
    MissionBoard,
    ReachabilityReport,
    SkillName,
    confirmation_matches_request,
    missing_request_match_terms,
)
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
    matched_attributes: Tuple[str, ...] = ()
    unmatched_attributes: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "confirmed", bool(self.confirmed))
        object.__setattr__(
            self,
            "matched_attributes",
            _string_tuple(self.matched_attributes),
        )
        object.__setattr__(
            self,
            "unmatched_attributes",
            _string_tuple(self.unmatched_attributes),
        )


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
        object_request = call.object_request or board.object_request
        if call.object_request and call.object_request.strip() != board.object_request.strip():
            return ToolResult(False, reason_code="object_request_mismatch")
        snapshot = self._registry.query(object_request, board)
        self._snapshot = snapshot
        candidates = tuple(candidate.candidate_id for candidate in snapshot.candidates)
        return ToolResult(
            success=snapshot.checked,
            reason_code="registry_checked" if snapshot.checked else "registry_unavailable",
            candidate_ids=candidates,
            registry_revision=snapshot.revision,
        )

    def inspect_candidates(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        object_request = call.object_request or board.object_request
        if call.object_request and call.object_request.strip() != board.object_request.strip():
            return ToolResult(False, reason_code="object_request_mismatch")
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
                object_request,
                self._snapshot,
                requested,
                board,
            )
        )
        decision_ids = {decision.candidate_id for decision in decisions}
        if decision_ids != set(requested):
            return ToolResult(False, reason_code="inspection_incomplete")
        # Treat the visual authority's boolean as a claim about the complete
        # request, never as a class-only match.  Normalize an over-optimistic
        # backend result into an ordinary rejection so the executive cannot
        # promote a generic ``chair`` for a ``blue chair`` mission.
        decisions = tuple(
            _enforce_request_match(object_request, decision)
            for decision in decisions
        )
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
        point = call.point
        projection_policy = call.projection_policy
        detail_prefix = ""
        candidate = None
        if board.active_skill is SkillName.APPROACH_CONFIRMED_OBJECT:
            # The dispatcher enforces these invariants for model output, but
            # keep the backend fail-closed for direct callers and transport
            # races.  An approach may never turn an arbitrary candidate or
            # stale revision into a movement target.
            if not board.confirmed_target_id:
                return ToolResult(False, reason_code="confirmed_target_required")
            if call.candidate_id != board.confirmed_target_id:
                return ToolResult(False, reason_code="confirmed_target_mismatch")
            if (
                not self._snapshot.checked
                or not self._snapshot.revision
                or not board.confirmed_registry_revision
            ):
                return ToolResult(False, reason_code="registry_snapshot_unavailable")
            if (
                board.confirmed_registry_revision != board.registry_revision
                or board.confirmed_registry_revision != self._snapshot.revision
            ):
                return ToolResult(False, reason_code="registry_revision_mismatch")
            candidate = next(
                (
                    item for item in self._snapshot.candidates
                    if item.candidate_id == call.candidate_id
                ),
                None,
            )
            if candidate is None:
                return ToolResult(False, reason_code="candidate_not_in_snapshot")

            if candidate.registry_revision != self._snapshot.revision:
                return ToolResult(False, reason_code="candidate_revision_mismatch")
            frame_id = str(candidate.metadata.get("frame_id", "map"))
            if frame_id != "map":
                return ToolResult(False, reason_code="candidate_position_frame_invalid")
            try:
                resolved_point = (
                    float(candidate.metadata["x"]),
                    float(candidate.metadata["y"]),
                )
            except (KeyError, TypeError, ValueError):
                return ToolResult(False, reason_code="candidate_position_unavailable")
            if not all(isfinite(value) for value in resolved_point):
                return ToolResult(False, reason_code="candidate_position_unavailable")

            # A confirmed approach is bound to the registry position.  A
            # caller may omit ``point`` (the normal model form), but may not
            # smuggle an unrelated coordinate alongside the confirmed ID.
            if point is not None:
                if not all(isfinite(float(value)) for value in point):
                    return ToolResult(False, reason_code="invalid_point")
                if any(
                    abs(float(value) - float(expected)) > 1e-3
                    for value, expected in zip(point, resolved_point)
                ):
                    return ToolResult(False, reason_code="candidate_position_mismatch")
            point = resolved_point
            # A registry position is an object observation, not a guaranteed
            # free Nav2 endpoint.  The deterministic approach path may use
            # Nav2/preflight projection; ordinary point tools keep their
            # explicit reject/allow policy.
            projection_policy = "allow"
            detail_prefix = "candidate_position_resolved:{}".format(call.candidate_id)
        elif point is None:
            return ToolResult(False, reason_code="point_required")
        result = _motion_tool_result(
            self._motion.go_to_point(point, projection_policy, board)
        )
        if detail_prefix:
            detail = detail_prefix
            if result.detail:
                detail += ";" + result.detail
            result = ToolResult(
                success=result.success,
                reason_code=result.reason_code,
                detail=detail,
                candidate_ids=result.candidate_ids,
                rejected_candidate_ids=result.rejected_candidate_ids,
                confirmed_target_id=result.confirmed_target_id,
                registry_revision=result.registry_revision,
                progress_delta=result.progress_delta,
                reachability=result.reachability,
                evidence=result.evidence,
            )
        return result

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
        matched_attributes=decision.matched_attributes,
        unmatched_attributes=decision.unmatched_attributes,
    )


def _string_tuple(values) -> Tuple[str, ...]:
    if isinstance(values, str):
        values = (values,)
    if values is None:
        return ()
    return tuple(str(value).strip() for value in values if str(value).strip())


def _enforce_request_match(
    object_request: str,
    decision: CandidateDecision,
) -> CandidateDecision:
    if not decision.confirmed:
        return decision
    if confirmation_matches_request(
        object_request,
        decision.matched_attributes,
        decision.unmatched_attributes,
    ):
        return decision
    missing = missing_request_match_terms(
        object_request,
        decision.matched_attributes,
    )
    unmatched = tuple(dict.fromkeys(decision.unmatched_attributes + missing))
    return replace(
        decision,
        confirmed=False,
        reason_code="requested_attributes_unconfirmed",
        unmatched_attributes=unmatched,
    )
