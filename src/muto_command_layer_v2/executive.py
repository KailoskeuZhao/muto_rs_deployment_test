"""Deterministic v2 mission executive.

This first slice owns only contract/state transitions. ROS transport and
backend adapters will be added after these invariants are covered by tests.
"""

import math
from typing import List, Optional

from .contracts import (
    CandidateEvidence,
    CompletionPolicy,
    ContractError,
    EventType,
    LifecycleState,
    MissionAction,
    MissionBoard,
    MissionEvent,
    ReachabilityReport,
    SCHEMA_VERSION,
    SkillName,
    ToolName,
    _SKILL_TOOLS,
)


# Completion must be backed by a reason emitted by the search authority.  A
# bounded cycle ending cooperatively is useful progress, but it is not proof
# that the scenario's search space has been exhausted.
_SEARCH_EXHAUSTION_REASON_CODES = frozenset((
    "frontier_exhausted",
    "search_exhausted",
))


class ExecutiveError(RuntimeError):
    """Base error for invalid executive operations."""


class DuplicateMissionError(ExecutiveError):
    """Raised when a second mission is submitted while one is active."""


class TerminalMissionError(ExecutiveError):
    """Raised when a command is attempted after terminal state."""


class MissionExecutive:
    """Single-owner deterministic mission state machine."""

    def __init__(
        self,
        *,
        scenario_completion_policy: Optional[CompletionPolicy] = None,
        scenario_id: str = "",
    ) -> None:
        self._board = MissionBoard()
        self._events: List[MissionEvent] = []
        self._mission_counter = 0
        self._scenario_completion_policy = (
            CompletionPolicy(scenario_completion_policy)
            if scenario_completion_policy is not None
            else None
        )
        self._scenario_id = str(scenario_id)

    @property
    def board(self) -> MissionBoard:
        return self._board

    @property
    def events(self):
        return tuple(self._events)

    def accept(self, action: MissionAction) -> MissionBoard:
        if self._board.lifecycle_state not in {
            LifecycleState.IDLE,
            LifecycleState.SUCCEEDED,
            LifecycleState.CANCELED,
            LifecycleState.FAILED,
        }:
            raise DuplicateMissionError("a mission is already active")
        action = action if isinstance(action, MissionAction) else MissionAction(**action)
        completion_policy = self._scenario_completion_policy or action.completion_policy
        self._mission_counter += 1
        mission_id = "mission-{:04d}".format(self._mission_counter)
        self._board = MissionBoard(
            schema_version=SCHEMA_VERSION,
            lifecycle_state=LifecycleState.ACCEPTED,
            mission_id=mission_id,
            request_id=action.request_id,
            objective=action.objective,
            object_request=action.object_request,
            completion_policy=completion_policy,
            scenario_id=self._scenario_id,
            board_revision=1,
            last_event_type=EventType.MISSION_ACCEPTED.value,
        )
        self._emit(EventType.MISSION_ACCEPTED)
        return self._board

    def start(self) -> MissionBoard:
        self._require_state(LifecycleState.ACCEPTED)
        self._board = self._board.evolve(
            lifecycle_state=LifecycleState.RUNNING,
            last_event_type=EventType.MISSION_STARTED.value,
        )
        self._emit(EventType.MISSION_STARTED)
        return self._board

    def select_skill(self, skill: SkillName) -> MissionBoard:
        self._require_state(LifecycleState.RUNNING)
        try:
            skill = SkillName(skill)
        except (TypeError, ValueError) as exc:
            raise ContractError("unsupported skill") from exc
        current = self._board.active_skill
        if current is not None and skill != current:
            if not (
                current is SkillName.SEARCH_FOR_OBJECT
                and skill is SkillName.APPROACH_CONFIRMED_OBJECT
                and self._board.confirmed_target_id
                and self._board.confirmed_registry_revision
                and self._board.confirmed_registry_revision == self._board.registry_revision
            ):
                raise ContractError("skill handoff requires a current confirmed target")
        if skill is SkillName.APPROACH_CONFIRMED_OBJECT and not (
            self._board.confirmed_target_id
            and self._board.confirmed_registry_revision
            and self._board.confirmed_registry_revision == self._board.registry_revision
        ):
            raise ContractError("approach skill requires confirmation for the current registry revision")
        self._board = self._board.evolve(
            active_skill=skill,
            active_tool=None,
            last_event_type=EventType.SKILL_SELECTED.value,
        )
        self._emit(EventType.SKILL_SELECTED, skill=skill)
        return self._board

    def record_tool_result(
        self,
        tool: ToolName,
        *,
        success: bool,
        reason_code: str = "",
        detail: str = "",
        candidate_id: str = "",
        confirmed_target_id: str = "",
        candidate_ids=(),
        rejected_candidate_ids=(),
        registry_revision: str = "",
        progress_delta: float = 0.0,
        reachability: Optional[ReachabilityReport] = None,
        expected_registry_revision: str = "",
        evidence=(),
    ) -> MissionBoard:
        self._require_state(LifecycleState.RUNNING)
        if self._board.active_skill is None:
            raise ContractError("a skill must be selected before a tool call")
        try:
            tool = ToolName(tool)
        except (TypeError, ValueError) as exc:
            raise ContractError("unsupported tool") from exc
        if tool not in _SKILL_TOOLS[self._board.active_skill]:
            raise ContractError("tool is not allowed by the active skill")
        if progress_delta < 0.0:
            raise ContractError("progress_delta must be non-negative")
        evidence = tuple(
            item if isinstance(item, CandidateEvidence) else CandidateEvidence(**item)
            for item in evidence
        )
        if expected_registry_revision and tool is ToolName.INSPECT_CANDIDATES:
            if self._board.registry_revision != expected_registry_revision:
                return self._record_stale_registry_result(
                    tool,
                    detail or "inspection result arrived after a newer registry revision",
                    expected_registry_revision,
                )
        if tool is ToolName.INSPECT_CANDIDATES and registry_revision:
            if registry_revision != self._board.registry_revision:
                return self._record_stale_registry_result(
                    tool,
                    detail or "inspection result revision does not match the board",
                    registry_revision,
                )
            requested = tuple(candidate_ids)
            if not requested or any(
                candidate not in self._board.shortlisted_candidate_ids
                for candidate in requested
            ):
                return self._record_stale_registry_result(
                    tool,
                    detail or "inspection candidate is not in the current shortlist",
                    registry_revision,
                )
            if any(
                item.registry_revision != registry_revision
                or item.candidate_id not in requested
                for item in evidence
            ):
                return self._record_stale_registry_result(
                    tool,
                    detail or "inspection evidence is not scoped to the current shortlist",
                    registry_revision,
                )
        failures = 0 if success else self._board.consecutive_failures + 1
        search_progress = self._board.search_progress
        approach_progress = self._board.approach_progress
        if self._board.active_skill == SkillName.SEARCH_FOR_OBJECT:
            search_progress += progress_delta
        else:
            approach_progress += progress_delta
        no_progress_count = (
            self._board.no_progress_count + 1
            if success and progress_delta == 0.0
            else 0
            if success
            else self._board.no_progress_count
        )
        current_target_id = self._board.confirmed_target_id
        current_confirmed_revision = self._board.confirmed_registry_revision
        current_evidence = self._board.candidate_evidence
        shortlist = list(self._board.shortlisted_candidate_ids)
        rejected = list(self._board.rejected_candidate_ids)
        revision_changed = bool(
            registry_revision
            and self._board.registry_revision
            and registry_revision != self._board.registry_revision
        )
        if revision_changed and tool is not ToolName.QUERY_REGISTRY:
            return self._record_stale_registry_result(
                tool,
                detail or "non-query result attempted to replace the registry revision",
                registry_revision,
            )
        if revision_changed:
            shortlist = []
            rejected = []
            current_target_id = ""
            current_confirmed_revision = ""
            current_evidence = ()
        if tool is ToolName.QUERY_REGISTRY:
            shortlist = [candidate for candidate in candidate_ids if candidate]
            same_revision = bool(
                registry_revision
                and self._board.registry_revision
                and registry_revision == self._board.registry_revision
            )
            if same_revision:
                # A repeated lookup is a refresh of the same semantic
                # shortlist, not new identity evidence.  Preserve rejection,
                # confirmation, and provenance so the commander does not
                # repeatedly re-inspect the same candidates or interrupt a
                # bounded search merely because the registry was polled.
                rejected = [candidate for candidate in rejected if candidate in shortlist]
                if current_target_id and current_target_id not in shortlist:
                    current_target_id = ""
                    current_confirmed_revision = ""
                    current_evidence = ()
                elif current_target_id:
                    current_evidence = tuple(
                        item for item in current_evidence
                        if item.registry_revision == registry_revision
                        and item.candidate_id in shortlist
                    )
            else:
                rejected = []
                current_target_id = ""
                current_confirmed_revision = ""
                current_evidence = ()
        elif tool is ToolName.INSPECT_CANDIDATES:
            # Inspection can only remove rejected IDs; it cannot silently add
            # a new candidate to the registry shortlist.
            for candidate in candidate_ids:
                if candidate and candidate not in shortlist:
                    raise ContractError("inspection returned a candidate outside the shortlist")
        for candidate in rejected_candidate_ids:
            if candidate and candidate not in rejected:
                rejected.append(candidate)
            if candidate in shortlist:
                shortlist.remove(candidate)
        if confirmed_target_id:
            if not success or tool != ToolName.INSPECT_CANDIDATES:
                raise ContractError(
                    "only a successful inspect_candidates result may confirm a target"
                )
            if not candidate_ids or confirmed_target_id not in candidate_ids:
                raise ContractError("confirmed target was not in the inspected candidates")
            if (
                registry_revision
                and self._board.registry_revision
                and registry_revision != self._board.registry_revision
            ):
                raise ContractError("confirmation belongs to a different registry revision")
            current_target_id = confirmed_target_id
            current_confirmed_revision = registry_revision or self._board.registry_revision
            if not current_confirmed_revision:
                raise ContractError("confirmation requires a registry revision")
            if not any(item.candidate_id == confirmed_target_id for item in evidence):
                raise ContractError("confirmation requires candidate evidence")
            if confirmed_target_id in rejected:
                rejected.remove(confirmed_target_id)
        self._board = self._board.evolve(
            active_tool=tool,
            registry_revision=registry_revision or self._board.registry_revision,
            shortlisted_candidate_ids=tuple(shortlist),
            rejected_candidate_ids=tuple(rejected),
            confirmed_target_id=current_target_id,
            confirmed_registry_revision=current_confirmed_revision,
            search_progress=search_progress,
            approach_progress=approach_progress,
            consecutive_failures=failures,
            no_progress_count=no_progress_count,
            last_event_type=EventType.TOOL_RESULT.value,
            last_outcome="success" if success else "failure",
            last_reason_code=reason_code,
            reachability=reachability if reachability is not None else self._board.reachability,
            candidate_evidence=evidence or current_evidence,
            active_command_status="success" if success else "failure",
            visual_evidence_summary=(
                "{} candidate evidence records for registry {}".format(len(evidence), registry_revision)
                if evidence else self._board.visual_evidence_summary
            ),
            coverage_summary=(
                detail or reason_code
                if tool is ToolName.OBSERVE and success
                else self._board.coverage_summary
            ),
        )
        self._emit(
            EventType.TOOL_RESULT,
            skill=self._board.active_skill,
            tool=tool,
            outcome="success" if success else "failure",
            reason_code=reason_code,
            candidate_id=candidate_id,
            registry_revision=registry_revision,
            detail=detail,
            evidence_id=(
                next((item.evidence_id for item in evidence if item.candidate_id == confirmed_target_id), "")
                if confirmed_target_id else ""
            ),
            evidence_source=(
                next((item.source for item in evidence if item.candidate_id == confirmed_target_id), "")
                if confirmed_target_id else ""
            ),
            evidence_confidence=(
                next((item.confidence for item in evidence if item.candidate_id == confirmed_target_id), None)
                if confirmed_target_id else None
            ),
            evidence_timestamp_s=(
                next((item.observed_at_s for item in evidence if item.candidate_id == confirmed_target_id), None)
                if confirmed_target_id else None
            ),
        )
        for rejected_candidate_id in rejected_candidate_ids:
            if not rejected_candidate_id:
                continue
            self._emit(
                EventType.CANDIDATE_REJECTED,
                skill=self._board.active_skill,
                tool=tool,
                outcome="rejected",
                reason_code=reason_code or "candidate_rejected",
                candidate_id=rejected_candidate_id,
                registry_revision=registry_revision or self._board.registry_revision,
            )
        if confirmed_target_id:
            self._emit(
                EventType.CANDIDATE_CONFIRMED,
                skill=self._board.active_skill,
                tool=tool,
                outcome="confirmed",
                candidate_id=confirmed_target_id,
                registry_revision=current_confirmed_revision,
            )
        return self._board

    def record_tool_request(self, tool: ToolName, *, detail: str = "") -> MissionBoard:
        """Commit dispatch intent before an authority is invoked."""

        self._require_state(LifecycleState.RUNNING)
        try:
            tool = ToolName(tool)
        except (TypeError, ValueError) as exc:
            raise ContractError("unsupported tool") from exc
        if self._board.active_skill is None or tool not in _SKILL_TOOLS[self._board.active_skill]:
            raise ContractError("tool is not allowed by the active skill")
        self._board = self._board.evolve(
            active_tool=tool,
            active_command_status="requested",
            last_event_type=EventType.TOOL_REQUESTED.value,
            last_outcome="pending",
            last_reason_code="",
        )
        self._emit(
            EventType.TOOL_REQUESTED,
            skill=self._board.active_skill,
            tool=tool,
            outcome="pending",
            detail=detail,
        )
        return self._board

    def _record_stale_registry_result(
        self,
        tool: ToolName,
        detail: str,
        registry_revision: str,
    ) -> MissionBoard:
        """Record stale evidence while preserving the newer board snapshot."""

        self._board = self._board.evolve(
            active_tool=tool,
            active_command_status="stale_result",
            consecutive_failures=self._board.consecutive_failures + 1,
            last_event_type=EventType.TOOL_RESULT.value,
            last_outcome="failure",
            last_reason_code="stale_registry_result",
        )
        self._emit(
            EventType.TOOL_RESULT,
            skill=self._board.active_skill,
            tool=tool,
            outcome="failure",
            reason_code="stale_registry_result",
            registry_revision=registry_revision,
            detail=detail,
        )
        return self._board

    def record_recorder_status(
        self,
        *,
        available: bool,
        uri: str = "",
        reason_code: str = "",
    ) -> MissionBoard:
        """Project recorder health without making recording mission-fatal."""

        if self._board.lifecycle_state is LifecycleState.IDLE:
            return self._board
        if (
            self._board.recorder_available == bool(available)
            and self._board.recorder_uri == uri
        ):
            return self._board
        self._board = self._board.evolve(
            recorder_available=bool(available),
            recorder_uri=str(uri),
            last_event_type=EventType.BOARD_UPDATED.value,
            last_reason_code=reason_code,
        )
        self._emit(EventType.BOARD_UPDATED, detail="recorder_status:" + (reason_code or "updated"))
        return self._board

    def complete(
        self,
        *,
        confirmed_target_id: str = "",
        approach_complete: bool = False,
        search_exhausted: bool = False,
        reason_code: str = "",
    ) -> MissionBoard:
        self._require_state(LifecycleState.RUNNING)
        if (
            confirmed_target_id
            and confirmed_target_id != self._board.confirmed_target_id
        ):
            raise ContractError(
                "completion cannot promote an unconfirmed candidate"
            )
        target_id = self._board.confirmed_target_id
        policy = self._board.completion_policy
        if target_id and (
            not self._board.confirmed_registry_revision
            or self._board.confirmed_registry_revision != self._board.registry_revision
        ):
            raise ContractError("completion requires confirmation for the current registry revision")
        if policy == CompletionPolicy.REPORT_CONFIRMED and not target_id:
            raise ContractError("report_confirmed requires a confirmed target")
        if policy == CompletionPolicy.APPROACH_CONFIRMED:
            if not target_id or not approach_complete:
                raise ContractError(
                    "approach_confirmed requires target and approach completion"
                )
        if policy == CompletionPolicy.SEARCH_UNTIL_EXHAUSTED and not search_exhausted:
            raise ContractError("search_until_exhausted requires exhausted search")
        if (
            policy == CompletionPolicy.SEARCH_UNTIL_EXHAUSTED
            and self._board.last_reason_code not in _SEARCH_EXHAUSTION_REASON_CODES
        ):
            raise ContractError(
                "search_until_exhausted requires explicit exhaustion evidence"
            )
        self._board = self._board.evolve(
            lifecycle_state=LifecycleState.SUCCEEDED,
            confirmed_target_id=target_id,
            last_event_type=EventType.MISSION_SUCCEEDED.value,
            last_outcome="succeeded",
            last_reason_code=reason_code,
        )
        self._emit(
            EventType.MISSION_SUCCEEDED,
            candidate_id=target_id,
            reason_code=reason_code,
        )
        return self._board

    def record_planner_failure(self, reason_code: str, detail: str = "") -> MissionBoard:
        """Commit invalid model output as evidence without changing lifecycle."""

        self._require_state(LifecycleState.RUNNING)
        if not reason_code.strip():
            raise ContractError("planner failure reason_code must not be empty")
        self._board = self._board.evolve(
            consecutive_failures=self._board.consecutive_failures + 1,
            last_event_type=EventType.PLANNER_FAILURE.value,
            last_outcome="failure",
            last_reason_code=reason_code,
        )
        self._emit(
            EventType.PLANNER_FAILURE,
            outcome="failure",
            reason_code=reason_code,
            detail=detail,
        )
        return self._board

    def record_board_observation(
        self,
        *,
        robot_pose=None,
        coverage_summary: Optional[str] = None,
        visual_evidence_summary: Optional[str] = None,
        active_command_status: Optional[str] = None,
    ) -> MissionBoard:
        """Commit a fresh high-level observation without selecting a tool."""

        if (
            robot_pose is None
            and coverage_summary is None
            and visual_evidence_summary is None
            and active_command_status is None
        ):
            return self._board
        pose = self._board.robot_pose
        if robot_pose is not None:
            try:
                pose = tuple(float(value) for value in robot_pose)
            except (TypeError, ValueError) as exc:
                raise ContractError("robot_pose must contain numeric x, y, heading") from exc
            if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
                raise ContractError("robot_pose must contain finite x, y, heading")
        values = {
            "robot_pose": pose,
            "coverage_summary": (
                coverage_summary if coverage_summary is not None else self._board.coverage_summary
            ),
            "visual_evidence_summary": (
                visual_evidence_summary
                if visual_evidence_summary is not None
                else self._board.visual_evidence_summary
            ),
            "active_command_status": (
                active_command_status
                if active_command_status is not None
                else self._board.active_command_status
            ),
        }
        if all(getattr(self._board, key) == value for key, value in values.items()):
            return self._board
        self._board = self._board.evolve(**values, last_event_type=EventType.BOARD_UPDATED.value)
        self._emit(
            EventType.BOARD_UPDATED,
            detail="robot pose observation",
        )
        return self._board

    def configure_limits(self, *limits: str) -> MissionBoard:
        """Expose local retry/authority guards on the board, not as a budget."""

        normalized = tuple(str(limit) for limit in limits if str(limit))
        if normalized == self._board.active_limits:
            return self._board
        self._board = self._board.evolve(
            active_limits=normalized,
            last_event_type=EventType.BOARD_UPDATED.value,
        )
        self._emit(EventType.BOARD_UPDATED, detail="active_limits_configured")
        return self._board

    def cancel(self, reason_code: str = "cancel_requested") -> MissionBoard:
        if (
            self._board.lifecycle_state.terminal
            or self._board.lifecycle_state == LifecycleState.IDLE
        ):
            return self._board
        self._board = self._board.evolve(
            lifecycle_state=LifecycleState.CANCELED,
            last_event_type=EventType.MISSION_CANCELED.value,
            last_outcome="canceled",
            last_reason_code=reason_code,
        )
        self._emit(EventType.MISSION_CANCELED, reason_code=reason_code)
        return self._board

    def fail(self, reason_code: str) -> MissionBoard:
        if (
            self._board.lifecycle_state.terminal
            or self._board.lifecycle_state == LifecycleState.IDLE
        ):
            return self._board
        if not reason_code.strip():
            raise ContractError("failure reason_code must not be empty")
        self._board = self._board.evolve(
            lifecycle_state=LifecycleState.FAILED,
            last_event_type=EventType.MISSION_FAILED.value,
            last_outcome="failed",
            last_reason_code=reason_code,
        )
        self._emit(EventType.MISSION_FAILED, reason_code=reason_code)
        return self._board

    def _require_state(self, expected: LifecycleState) -> None:
        if self._board.lifecycle_state != expected:
            if self._board.lifecycle_state.terminal:
                raise TerminalMissionError("mission is already terminal")
            raise ExecutiveError(
                "expected {}, got {}".format(expected.value, self._board.lifecycle_state.value)
            )

    def _emit(
        self,
        event_type: EventType,
        *,
        skill: Optional[SkillName] = None,
        tool: Optional[ToolName] = None,
        outcome: str = "",
        reason_code: str = "",
        candidate_id: str = "",
        registry_revision: str = "",
        detail: str = "",
        evidence_id: str = "",
        evidence_source: str = "",
        evidence_confidence: Optional[float] = None,
        evidence_timestamp_s: Optional[float] = None,
    ) -> None:
        self._events.append(
            MissionEvent(
                sequence=len(self._events) + 1,
                event_type=event_type,
                mission_id=self._board.mission_id,
                request_id=self._board.request_id,
                board_revision=self._board.board_revision,
                lifecycle_state=self._board.lifecycle_state,
                skill=skill,
                tool=tool,
                outcome=outcome,
                reason_code=reason_code,
                candidate_id=candidate_id,
                registry_revision=registry_revision,
                detail=detail,
                evidence_id=evidence_id,
                evidence_source=evidence_source,
                evidence_confidence=evidence_confidence,
                evidence_timestamp_s=evidence_timestamp_s,
            )
        )
