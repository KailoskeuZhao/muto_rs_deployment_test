"""Strict, transport-independent commander decision boundary."""

from dataclasses import dataclass
import json
from typing import Any, Callable, Mapping, Optional, Tuple

from .contracts import (
    ContractError,
    MissionBoard,
    SCHEMA_VERSION,
    SkillName,
    ToolName,
    _SKILL_TOOLS,
)
from .tools import ToolCall


class PlannerFailure(ContractError):
    """Model output was invalid and must be replanned as nonfatal evidence."""


@dataclass(frozen=True)
class CommanderDecision:
    skill: SkillName
    tool: Optional[ToolCall] = None
    completion_proposal: Optional[str] = None
    rationale: str = ""
    schema_version: str = SCHEMA_VERSION


def _strict_dict(payload: Any) -> Mapping[str, Any]:
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise PlannerFailure("planner output is not valid JSON") from exc
    if not isinstance(payload, Mapping):
        raise PlannerFailure("planner output must be a JSON object")
    allowed = {"schema_version", "skill", "tool", "completion_proposal", "rationale"}
    unknown = set(payload) - allowed
    if unknown:
        raise PlannerFailure("planner output has unknown fields: {}".format(sorted(unknown)))
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise PlannerFailure("planner output has an unsupported schema_version")
    return payload


def parse_decision(payload: Any, board: MissionBoard) -> CommanderDecision:
    """Parse one model response and enforce the active skill/tool boundary."""

    data = _strict_dict(payload)
    try:
        skill = SkillName(data["skill"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PlannerFailure("planner must choose one supported skill") from exc
    if board.active_skill is not None and skill != board.active_skill:
        # A search mission may hand off to approach only after the executive
        # has recorded an explicit visual confirmation.  The runtime performs
        # the actual board mutation; the parser merely enforces this safety
        # precondition.  This keeps skill selection model-driven without
        # allowing an unconfirmed candidate (or an arbitrary reverse handoff)
        # to enter the motion skill.
        allowed_handoff = (
            skill is SkillName.APPROACH_CONFIRMED_OBJECT
            and bool(board.confirmed_target_id)
            and bool(board.confirmed_registry_revision)
            and board.confirmed_registry_revision == board.registry_revision
        )
        if not allowed_handoff:
            raise PlannerFailure("planner changed skill without a confirmed handoff")
    tool_data = data.get("tool")
    tool = None
    if tool_data is not None:
        if not isinstance(tool_data, Mapping):
            raise PlannerFailure("tool must be a JSON object")
        if set(tool_data) - {
            "name", "object_request", "candidate_ids", "registry_revision",
            "candidate_id", "point", "heading", "frame_id", "projection_policy",
        }:
            raise PlannerFailure("tool has unknown fields")
        try:
            tool = ToolCall(
                tool=ToolName(tool_data["name"]),
                object_request=str(tool_data.get("object_request", "")),
                candidate_ids=tuple(tool_data.get("candidate_ids", ())),
                registry_revision=str(tool_data.get("registry_revision", "")),
                candidate_id=str(tool_data.get("candidate_id", "")),
                point=tuple(tool_data["point"]) if tool_data.get("point") is not None else None,
                heading=tool_data.get("heading"),
                frame_id=str(tool_data.get("frame_id", "map")),
                projection_policy=str(tool_data.get("projection_policy", "reject")),
            )
        except (KeyError, TypeError, ValueError, ContractError) as exc:
            raise PlannerFailure("planner emitted an invalid tool call") from exc
        if tool.tool not in _SKILL_TOOLS[skill]:
            raise PlannerFailure("tool is not allowed by the selected skill")
    proposal = data.get("completion_proposal")
    if proposal is not None and not isinstance(proposal, str):
        raise PlannerFailure("completion_proposal must be a string")
    if tool is not None and proposal is not None:
        raise PlannerFailure("planner must emit either a tool call or a completion proposal")
    rationale = data.get("rationale", "")
    if not isinstance(rationale, str):
        raise PlannerFailure("rationale must be a string")
    return CommanderDecision(
        skill=skill,
        tool=tool,
        completion_proposal=proposal,
        rationale=rationale,
    )


class CommanderAgent:
    """Thin model adapter; it has no lifecycle or board mutation authority."""

    def __init__(self, planner: Callable[[MissionBoard], Any]) -> None:
        self._planner = planner

    def cancel(self) -> None:
        """Propagate cancellation to a planner transport when supported."""

        cancel = getattr(self._planner, "cancel", None)
        if callable(cancel):
            cancel()

    def decide(self, board: MissionBoard) -> CommanderDecision:
        if board.lifecycle_state.value != "running":
            raise PlannerFailure("commander may plan only for a running mission")
        return parse_decision(self._planner(board), board)
