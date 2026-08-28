"""Independent v2 command-layer contracts and deterministic executive."""

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
    ReachabilityState,
    SkillName,
    ToolName,
    confirmation_matches_request,
    missing_request_match_terms,
    request_match_terms,
)
from .executive import (
    DuplicateMissionError,
    ExecutiveError,
    MissionExecutive,
    TerminalMissionError,
)
from .natural_language import ActionRejection, CancellationRequest, NaturalLanguageAdapter
from .commander import CommanderAgent, CommanderDecision, PlannerFailure, parse_decision
from .tools import ToolCall, ToolDispatcher, ToolExecutionError, ToolResult
from .reachability import OccupancyGrid, ReachabilityConfig, ReachabilityPlanner
from .recorder import HighLevelRecorder, RecorderManifest
from .runtime import CommanderRuntime, RuntimeResult
from .backend_adapters import (
    CandidateDecision,
    MotionResult,
    RegistryCandidate,
    RegistrySnapshot,
    V2ToolBackend,
)

__all__ = [
    "CompletionPolicy",
    "CandidateEvidence",
    "ContractError",
    "DuplicateMissionError",
    "EventType",
    "ExecutiveError",
    "LifecycleState",
    "MissionAction",
    "MissionBoard",
    "MissionEvent",
    "MissionExecutive",
    "ReachabilityReport",
    "ReachabilityState",
    "SkillName",
    "TerminalMissionError",
    "ToolName",
    "confirmation_matches_request",
    "missing_request_match_terms",
    "request_match_terms",
    "ActionRejection",
    "CancellationRequest",
    "NaturalLanguageAdapter",
    "CommanderAgent",
    "CommanderDecision",
    "PlannerFailure",
    "parse_decision",
    "ToolCall",
    "ToolDispatcher",
    "ToolExecutionError",
    "ToolResult",
    "OccupancyGrid",
    "ReachabilityConfig",
    "ReachabilityPlanner",
    "HighLevelRecorder",
    "RecorderManifest",
    "CommanderRuntime",
    "RuntimeResult",
    "CandidateDecision",
    "MotionResult",
    "RegistryCandidate",
    "RegistrySnapshot",
    "V2ToolBackend",
]
