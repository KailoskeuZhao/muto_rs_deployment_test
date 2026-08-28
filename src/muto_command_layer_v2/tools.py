"""The v2 static, typed commander tool table.

The commander never receives a ROS publisher or an arbitrary capability map.
It submits one :class:`ToolCall` to this dispatcher, which validates the
skill scope and delegates to a deterministic backend object.  Backends are
injected by the ROS integration layer; this module remains ROS-free.
"""

from dataclasses import dataclass
from math import isfinite
from threading import RLock
from typing import Optional, Protocol, Sequence, Tuple

from .contracts import (
    ContractError,
    CandidateEvidence,
    MissionBoard,
    ReachabilityReport,
    SkillName,
    ToolName,
    _SKILL_TOOLS,
)


class ToolExecutionError(RuntimeError):
    """A backend could not execute a valid tool call."""


@dataclass(frozen=True)
class ToolCall:
    """Typed envelope used internally by the commander and dispatcher."""

    tool: ToolName
    object_request: str = ""
    candidate_ids: Tuple[str, ...] = ()
    registry_revision: str = ""
    candidate_id: str = ""
    point: Optional[Tuple[float, float]] = None
    heading: Optional[float] = None
    frame_id: str = "map"
    projection_policy: str = "reject"

    def __post_init__(self) -> None:
        try:
            tool = self.tool if isinstance(self.tool, ToolName) else ToolName(self.tool)
        except (TypeError, ValueError) as exc:
            raise ContractError("unsupported tool") from exc
        object.__setattr__(self, "tool", tool)
        if self.point is not None:
            if len(self.point) != 2:
                raise ContractError("point must contain x and y")
            if not all(
                isinstance(value, (int, float)) and isfinite(float(value))
                for value in self.point
            ):
                raise ContractError("point coordinates must be finite numbers")
        if self.heading is not None and (
            not isinstance(self.heading, (int, float))
            or not isfinite(float(self.heading))
        ):
            raise ContractError("heading must be a finite number")
        if self.frame_id != "map":
            raise ContractError("motion tools require the map frame")
        if self.projection_policy not in {"reject", "allow"}:
            raise ContractError("projection_policy must be reject or allow")


@dataclass(frozen=True)
class ToolResult:
    """Typed result committed to the executive as a tool-result event."""

    success: bool
    reason_code: str = ""
    detail: str = ""
    candidate_ids: Tuple[str, ...] = ()
    rejected_candidate_ids: Tuple[str, ...] = ()
    confirmed_target_id: str = ""
    registry_revision: str = ""
    progress_delta: float = 0.0
    reachability: ReachabilityReport = ReachabilityReport()
    evidence: Tuple[CandidateEvidence, ...] = ()

    def __post_init__(self) -> None:
        if self.progress_delta < 0.0:
            raise ContractError("progress_delta must be non-negative")
        if self.confirmed_target_id and self.confirmed_target_id in self.rejected_candidate_ids:
            raise ContractError("a confirmed candidate cannot also be rejected")
        object.__setattr__(
            self,
            "evidence",
            tuple(
                item if isinstance(item, CandidateEvidence) else CandidateEvidence(**item)
                for item in self.evidence
            ),
        )


class ToolBackend(Protocol):
    """Independent authority methods used by the static dispatcher."""

    def query_registry(self, call: ToolCall, board: MissionBoard) -> ToolResult: ...

    def inspect_candidates(self, call: ToolCall, board: MissionBoard) -> ToolResult: ...

    def observe(self, call: ToolCall, board: MissionBoard) -> ToolResult: ...

    def rotate_to_heading(self, call: ToolCall, board: MissionBoard) -> ToolResult: ...

    def go_to_point(self, call: ToolCall, board: MissionBoard) -> ToolResult: ...

    def cancel_active(self) -> None: ...


# Deliberately static.  This table is the complete commander toolbox in v2.
TOOL_DISPATCH = {
    ToolName.QUERY_REGISTRY: "query_registry",
    ToolName.INSPECT_CANDIDATES: "inspect_candidates",
    ToolName.OBSERVE: "observe",
    ToolName.ROTATE_TO_HEADING: "rotate_to_heading",
    ToolName.GO_TO_POINT: "go_to_point",
}


class ToolDispatcher:
    """Validate and execute one bounded tool call at a time."""

    def __init__(self, backend: ToolBackend) -> None:
        self._backend = backend
        self._motion_active = False
        self._active_call: Optional[ToolCall] = None
        self._lock = RLock()

    @property
    def motion_active(self) -> bool:
        return self._motion_active

    def dispatch(self, call: ToolCall, board: MissionBoard) -> ToolResult:
        if board.active_skill is None:
            raise ContractError("a skill must be selected before dispatch")
        if call.tool not in _SKILL_TOOLS[board.active_skill]:
            raise ContractError(
                "{} is not allowed by {}".format(call.tool.value, board.active_skill.value)
            )
        if (
            call.tool in {ToolName.QUERY_REGISTRY, ToolName.INSPECT_CANDIDATES}
            and call.object_request
            and call.object_request.strip() != board.object_request.strip()
        ):
            raise ContractError(
                "registry tools must use the mission object_request unchanged"
            )
        if call.tool == ToolName.GO_TO_POINT:
            if board.active_skill == SkillName.APPROACH_CONFIRMED_OBJECT:
                if not board.confirmed_target_id:
                    raise ContractError("approach requires a confirmed target")
                if (
                    not board.confirmed_registry_revision
                    or board.confirmed_registry_revision != board.registry_revision
                ):
                    raise ContractError(
                        "approach requires confirmation for the current registry revision"
                    )
                if call.candidate_id != board.confirmed_target_id:
                    raise ContractError("go_to_point target is not the confirmed target")
            elif call.point is None:
                raise ContractError("go_to_point requires a map-frame point")
        if call.tool == ToolName.ROTATE_TO_HEADING and call.heading is None:
            raise ContractError("rotate_to_heading requires a heading")
        method_name = TOOL_DISPATCH[call.tool]
        method = getattr(self._backend, method_name)
        # ``observe`` is a motion tool in the production composition: the POI
        # authority selects a goal and waits for Nav2 before returning.  Keep
        # it in the same serialized lane as explicit rotation and navigation
        # so a second planner decision cannot overlap an active POI goal.
        is_motion = call.tool in {
            ToolName.OBSERVE,
            ToolName.ROTATE_TO_HEADING,
            ToolName.GO_TO_POINT,
        }
        with self._lock:
            if is_motion and self._motion_active:
                raise ToolExecutionError("a motion tool is already active")
            self._active_call = call
            if is_motion:
                self._motion_active = True
        try:
            result = method(call, board)
            if not isinstance(result, ToolResult):
                raise ToolExecutionError("backend returned a non-ToolResult")
            return result
        finally:
            with self._lock:
                if is_motion:
                    self._motion_active = False
                self._active_call = None

    def cancel_active(self) -> bool:
        """Request cancellation of an in-flight authority operation.

        The dispatcher remains the sole place that knows which tool is active;
        an authority may implement ``cancel_active`` to propagate the request
        to Nav2/VLM.  Missing cancellation support is reported to the caller
        without turning a user cancel into an exception.
        """

        with self._lock:
            active = self._active_call
        if active is None:
            return False
        cancel = getattr(self._backend, "cancel_active", None)
        if not callable(cancel):
            return False
        try:
            cancel()
        except Exception:
            return False
        return True
