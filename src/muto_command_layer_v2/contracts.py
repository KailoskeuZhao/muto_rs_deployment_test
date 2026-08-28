"""Small, dependency-free v2 contracts.

These contracts deliberately do not import the legacy command layer or ROS.
The ROS messages in this package are the wire representation; these types are
used to make executive state transitions deterministic and easy to test.
"""

from dataclasses import dataclass, field, replace
from enum import Enum
from math import isfinite
import re
from typing import Mapping, Optional, Tuple


SCHEMA_VERSION = "muto_command_layer_v2"


class ContractError(ValueError):
    """Raised when a v2 request or state violates the contract."""


class LifecycleState(str, Enum):
    IDLE = "idle"
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    FAILED = "failed"

    @property
    def terminal(self) -> bool:
        return self in {
            LifecycleState.SUCCEEDED,
            LifecycleState.CANCELED,
            LifecycleState.FAILED,
        }


class CompletionPolicy(str, Enum):
    REPORT_CONFIRMED = "report_confirmed"
    APPROACH_CONFIRMED = "approach_confirmed"
    SEARCH_UNTIL_EXHAUSTED = "search_until_exhausted"


class SkillName(str, Enum):
    SEARCH_FOR_OBJECT = "search_for_object"
    APPROACH_CONFIRMED_OBJECT = "approach_confirmed_object"


class ToolName(str, Enum):
    QUERY_REGISTRY = "query_registry"
    INSPECT_CANDIDATES = "inspect_candidates"
    OBSERVE = "observe"
    ROTATE_TO_HEADING = "rotate_to_heading"
    GO_TO_POINT = "go_to_point"


class ReachabilityState(str, Enum):
    UNKNOWN = "unknown"
    REACHABLE = "reachable"
    UNREACHABLE = "unreachable"


class EventType(str, Enum):
    MISSION_ACCEPTED = "mission_accepted"
    MISSION_STARTED = "mission_started"
    SKILL_SELECTED = "skill_selected"
    TOOL_REQUESTED = "tool_requested"
    TOOL_RESULT = "tool_result"
    PLANNER_FAILURE = "planner_failure"
    CANDIDATE_CONFIRMED = "candidate_confirmed"
    CANDIDATE_REJECTED = "candidate_rejected"
    BOARD_UPDATED = "board_updated"
    MISSION_SUCCEEDED = "mission_succeeded"
    MISSION_CANCELED = "mission_canceled"
    MISSION_FAILED = "mission_failed"


_SKILL_TOOLS = {
    # Search movement is deliberately mediated by ``observe``.  The POI-grid
    # authority chooses the reachable viewpoint; exposing a raw
    # ``go_to_point`` here would let the model bypass that authority and turn
    # search into an unbounded coordinate-control loop.
    SkillName.SEARCH_FOR_OBJECT: frozenset(
        {
            ToolName.QUERY_REGISTRY,
            ToolName.INSPECT_CANDIDATES,
            ToolName.OBSERVE,
            ToolName.ROTATE_TO_HEADING,
        }
    ),
    SkillName.APPROACH_CONFIRMED_OBJECT: frozenset(
        {
            ToolName.ROTATE_TO_HEADING,
            ToolName.GO_TO_POINT,
        }
    ),
}


def _as_enum(value, enum_type, field_name):
    if isinstance(value, enum_type):
        return value
    try:
        return enum_type(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(
            "{} must be one of {}".format(
                field_name, ", ".join(item.value for item in enum_type)
            )
        ) from exc


def _nonnegative_finite(value: Optional[float], field_name: str) -> None:
    if value is None:
        return
    if not isfinite(value) or value < 0.0:
        raise ContractError("{} must be finite and non-negative".format(field_name))


# Confirmation is intentionally conservative and dependency-free.  These are
# only grammatical words that the natural-language boundary does not treat as
# object requirements; nouns and adjectives (including colours such as
# ``blue``) remain mandatory visual terms.
_REQUEST_MATCH_STOP_WORDS = frozenset({
    "a", "an", "approach", "confirmed", "find", "for", "go",
    "identify", "look", "locate", "object", "please", "search",
    "target", "the", "to",
})


def request_match_terms(object_request: str) -> Tuple[str, ...]:
    """Return normalized terms that a visual confirmation must satisfy."""

    if not isinstance(object_request, str):
        return ()
    return tuple(dict.fromkeys(
        token
        for token in re.findall(r"[a-z0-9]+", object_request.lower())
        if token not in _REQUEST_MATCH_STOP_WORDS
    ))


def _string_tuple(values) -> Tuple[str, ...]:
    if isinstance(values, str):
        values = (values,)
    if values is None:
        return ()
    return tuple(str(value).strip() for value in values if str(value).strip())


def _attribute_terms(attributes) -> Tuple[str, ...]:
    attributes = _string_tuple(attributes)
    return tuple(dict.fromkeys(
        token
        for value in attributes
        for token in re.findall(r"[a-z0-9]+", str(value).lower())
    ))


def missing_request_match_terms(
    object_request: str,
    matched_attributes,
) -> Tuple[str, ...]:
    """Return request terms not explicitly represented by matched attributes."""

    matched = set(_attribute_terms(matched_attributes))
    return tuple(term for term in request_match_terms(object_request) if term not in matched)


def confirmation_matches_request(
    object_request: str,
    matched_attributes,
    unmatched_attributes,
) -> bool:
    """Check the fail-closed, full-request confirmation predicate.

    A model may only promote a candidate when all normalized request terms are
    represented by its matched attributes and it reports no unknown or
    contradictory attributes.  The executive and registry adapter both use
    this predicate so a direct backend cannot bypass the semantic gate.
    """

    required = set(request_match_terms(object_request))
    matched = set(_attribute_terms(matched_attributes))
    unmatched = set(_attribute_terms(unmatched_attributes))
    # An empty request is not an exact visual target.  Reject it rather than
    # allowing the set inclusion below to make confirmation vacuously true.
    return bool(required) and not unmatched and required.issubset(matched)


@dataclass(frozen=True)
class MissionAction:
    """Normalized user request accepted by the executive."""

    request_id: str
    objective: str
    completion_policy: CompletionPolicy
    object_request: str = ""
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not self.request_id.strip():
            raise ContractError("request_id must not be empty")
        if not self.objective.strip():
            raise ContractError("objective must not be empty")
        if self.schema_version != SCHEMA_VERSION:
            raise ContractError("unsupported schema_version")
        object.__setattr__(
            self,
            "completion_policy",
            _as_enum(self.completion_policy, CompletionPolicy, "completion_policy"),
        )


@dataclass(frozen=True)
class CandidateEvidence:
    """Evidence attached to one registry candidate confirmation decision.

    A candidate id by itself is not confirmation.  The evidence record keeps
    the provenance needed to audit which revision, image/observation, model
    decision, and requested attributes promoted (or rejected) a candidate.
    """

    candidate_id: str
    registry_revision: str
    evidence_id: str = ""
    source: str = ""
    confidence: Optional[float] = None
    observed_at_s: Optional[float] = None
    reason_code: str = ""
    matched_attributes: Tuple[str, ...] = field(default_factory=tuple)
    unmatched_attributes: Tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if not self.candidate_id.strip():
            raise ContractError("candidate evidence requires candidate_id")
        if not self.registry_revision.strip():
            raise ContractError("candidate evidence requires registry_revision")
        if self.confidence is not None and (
            not isfinite(float(self.confidence))
            or not 0.0 <= float(self.confidence) <= 1.0
        ):
            raise ContractError("candidate evidence confidence must be in [0, 1]")
        _nonnegative_finite(self.observed_at_s, "observed_at_s")
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
class ReachabilityReport:
    """Single preflight result; it never claims Nav2 execution success."""

    state: ReachabilityState = ReachabilityState.UNKNOWN
    reason_code: str = ""
    path_length_m: Optional[float] = None
    estimated_time_s: Optional[float] = None
    costmap_revision: Optional[int] = None
    freshness: str = "unknown"
    selected_pose: Optional[Tuple[float, float, float]] = None
    projected: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "state", _as_enum(self.state, ReachabilityState, "state"))
        _nonnegative_finite(self.path_length_m, "path_length_m")
        _nonnegative_finite(self.estimated_time_s, "estimated_time_s")
        if self.costmap_revision is not None and self.costmap_revision < 0:
            raise ContractError("costmap_revision must be non-negative")
        if self.selected_pose is not None:
            if len(self.selected_pose) != 3 or not all(
                isfinite(float(value)) for value in self.selected_pose
            ):
                raise ContractError("selected_pose must be finite x, y, heading")


@dataclass(frozen=True)
class MissionBoard:
    """Canonical current state projected to the commander and observers."""

    schema_version: str = SCHEMA_VERSION
    lifecycle_state: LifecycleState = LifecycleState.IDLE
    mission_id: str = ""
    request_id: str = ""
    objective: str = ""
    object_request: str = ""
    completion_policy: Optional[CompletionPolicy] = None
    active_skill: Optional[SkillName] = None
    active_tool: Optional[ToolName] = None
    board_revision: int = 0
    registry_revision: str = ""
    shortlisted_candidate_ids: Tuple[str, ...] = field(default_factory=tuple)
    rejected_candidate_ids: Tuple[str, ...] = field(default_factory=tuple)
    confirmed_target_id: str = ""
    confirmed_registry_revision: str = ""
    robot_pose: Optional[Tuple[float, float, float]] = None
    motion_state: str = "idle"
    search_progress: float = 0.0
    approach_progress: float = 0.0
    consecutive_failures: int = 0
    no_progress_count: int = 0
    last_event_type: str = ""
    last_outcome: str = ""
    last_reason_code: str = ""
    reachability: ReachabilityReport = field(default_factory=ReachabilityReport)
    recorder_available: bool = False
    recorder_uri: str = ""
    scenario_id: str = ""
    coverage_summary: str = ""
    visual_evidence_summary: str = ""
    active_command_status: str = ""
    active_limits: Tuple[str, ...] = field(default_factory=tuple)
    candidate_evidence: Tuple[CandidateEvidence, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ContractError("unsupported schema_version")
        object.__setattr__(self, "lifecycle_state", _as_enum(
            self.lifecycle_state, LifecycleState, "lifecycle_state"
        ))
        if self.completion_policy is not None:
            object.__setattr__(self, "completion_policy", _as_enum(
                self.completion_policy, CompletionPolicy, "completion_policy"
            ))
        if self.active_skill is not None:
            object.__setattr__(self, "active_skill", _as_enum(
                self.active_skill, SkillName, "active_skill"
            ))
        if self.active_tool is not None:
            object.__setattr__(self, "active_tool", _as_enum(
                self.active_tool, ToolName, "active_tool"
            ))
        if self.board_revision < 0:
            raise ContractError("board_revision must be non-negative")
        for field_name in (
            "search_progress",
            "approach_progress",
        ):
            value = getattr(self, field_name)
            if not isfinite(value) or value < 0.0:
                raise ContractError("{} must be finite and non-negative".format(field_name))
        if self.consecutive_failures < 0 or self.no_progress_count < 0:
            raise ContractError("failure counters must be non-negative")
        if self.robot_pose is not None:
            if len(self.robot_pose) != 3 or not all(
                isfinite(float(value)) for value in self.robot_pose
            ):
                raise ContractError("robot_pose must be finite x, y, heading")
        object.__setattr__(
            self,
            "candidate_evidence",
            tuple(
                item if isinstance(item, CandidateEvidence) else CandidateEvidence(**item)
                for item in self.candidate_evidence
            ),
        )
        object.__setattr__(self, "active_limits", tuple(str(item) for item in self.active_limits))

    def evolve(self, **changes):
        """Return a new board with one monotonically increasing revision."""

        changes.setdefault("board_revision", self.board_revision + 1)
        return replace(self, **changes)


@dataclass(frozen=True)
class MissionEvent:
    """Append-only semantic event committed with a board revision."""

    sequence: int
    event_type: EventType
    mission_id: str
    request_id: str
    board_revision: int
    lifecycle_state: LifecycleState
    skill: Optional[SkillName] = None
    tool: Optional[ToolName] = None
    outcome: str = ""
    reason_code: str = ""
    candidate_id: str = ""
    registry_revision: str = ""
    detail: str = ""
    evidence_id: str = ""
    evidence_source: str = ""
    evidence_confidence: Optional[float] = None
    evidence_timestamp_s: Optional[float] = None
    evidence_matched_attributes: Tuple[str, ...] = field(default_factory=tuple)
    evidence_unmatched_attributes: Tuple[str, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.sequence < 1:
            raise ContractError("event sequence must start at one")
        object.__setattr__(self, "event_type", _as_enum(
            self.event_type, EventType, "event_type"
        ))
        object.__setattr__(self, "lifecycle_state", _as_enum(
            self.lifecycle_state, LifecycleState, "lifecycle_state"
        ))
        if self.skill is not None:
            object.__setattr__(self, "skill", _as_enum(self.skill, SkillName, "skill"))
        if self.tool is not None:
            object.__setattr__(self, "tool", _as_enum(self.tool, ToolName, "tool"))
        if self.evidence_confidence is not None and (
            not isfinite(float(self.evidence_confidence))
            or not 0.0 <= float(self.evidence_confidence) <= 1.0
        ):
            raise ContractError("event evidence_confidence must be in [0, 1]")
        _nonnegative_finite(self.evidence_timestamp_s, "evidence_timestamp_s")
        object.__setattr__(
            self,
            "evidence_matched_attributes",
            _string_tuple(self.evidence_matched_attributes),
        )
        object.__setattr__(
            self,
            "evidence_unmatched_attributes",
            _string_tuple(self.evidence_unmatched_attributes),
        )
