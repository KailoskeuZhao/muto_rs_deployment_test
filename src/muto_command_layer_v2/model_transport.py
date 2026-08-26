"""ROS action transport for the v2 commander planner.

This module adapts the independent ``muto_vlm_socket`` action to the small
callable expected by :class:`CommanderAgent`.  It deliberately carries only
the typed board projection and optional JPEG observations; it does not expose
ROS publishers, action clients, or arbitrary capabilities to the model.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Callable, Mapping, Optional, Sequence

from .commander import PlannerFailure
from .contracts import MissionBoard, SCHEMA_VERSION
from .backend_adapters import CandidateDecision, RegistryCandidate


def _json_value(value):
    """Convert the dependency-free board dataclasses into JSON values."""

    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {
            key: _json_value(item)
            for key, item in asdict(value).items()
        }
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def build_planner_prompt(board: MissionBoard, *, visual_available: bool) -> str:
    """Build the bounded model input from one canonical board snapshot."""

    state = _json_value(board)
    return (
        "You are the v2 CommanderAgent. Choose exactly one approved skill and "
        "at most one typed tool for the current board. The executive owns "
        "mission truth; never invent a terminal result. Candidate shortlist "
        "IDs are not confirmed targets. Use inspect_candidates only when the "
        "current registry revision and evidence support a decision. Switch "
        "from search_for_object to approach_confirmed_object only after the "
        "board contains a confirmed target for that revision. The selected "
        "completion_policy is fixed by the starting scenario and cannot be "
        "changed by the commander. Completion gates are strict: "
        "report_confirmed requires a non-empty confirmed_target_id for the "
        "current registry revision; approach_confirmed requires that same "
        "target plus successful approach evidence; search_until_exhausted "
        "requires explicit search exhaustion evidence. In particular, "
        "search_progress counts frontier goals that actually succeeded, "
        "not a percentage and not proof of exhaustion. "
        "last_reason_code=frontier_goal_succeeded means only that one frontier "
        "goal made progress; it is not frontier exhaustion. Only "
        "last_reason_code=frontier_exhausted (or an explicit "
        "search_exhausted authority result) permits search_until_exhausted. "
        "When no completion gate is satisfied, set completion_proposal to null "
        "and select another allowed tool such as query_registry, "
        "inspect_candidates, observe, or rotate_to_heading. Return "
        "only JSON matching the supplied schema.\n"
        "VISUAL_INPUT_AVAILABLE=" + ("true" if visual_available else "false") +
        "\nSTATE_JSON=" + json.dumps(state, sort_keys=True, separators=(",", ":"))
    )


def build_decision_schema() -> str:
    """Return the strict JSON schema accepted by the VLM socket."""

    schema = {
        "type": "object",
        "additionalProperties": False,
        "required": ["schema_version", "skill", "tool", "completion_proposal", "rationale"],
        # Chat Completions providers used by the Humble deployment reject a
        # root-level ``oneOf`` in strict response schemas.  Both operation
        # fields are therefore required and nullable here; the strict parser
        # in commander.py remains the authority that enforces exactly one
        # tool call or completion proposal.
        "properties": {
            "schema_version": {
                "type": "string",
                "enum": [SCHEMA_VERSION],
            },
            "skill": {
                "type": "string",
                "enum": ["search_for_object", "approach_confirmed_object"],
            },
            "tool": {
                "anyOf": [
                    {"type": "null"},
                    {
                        "type": "object",
                        "additionalProperties": False,
                        "required": [
                            "name", "object_request", "candidate_ids",
                            "registry_revision", "candidate_id", "point",
                            "heading", "frame_id", "projection_policy",
                        ],
                        "properties": {
                            "name": {
                                "type": "string",
                                "enum": [
                                    "query_registry", "inspect_candidates", "observe",
                                    "rotate_to_heading", "go_to_point",
                                ],
                            },
                            "object_request": {"type": "string"},
                            "candidate_ids": {
                                "type": "array", "items": {"type": "string"},
                            },
                            "registry_revision": {"type": "string"},
                            "candidate_id": {"type": "string"},
                            "point": {
                                "anyOf": [
                                    {"type": "null"},
                                    {
                                        "type": "array", "minItems": 2, "maxItems": 2,
                                        "items": {"type": "number"},
                                    },
                                ]
                            },
                            "heading": {"anyOf": [{"type": "null"}, {"type": "number"}]},
                            "frame_id": {
                                "type": "string",
                                "enum": ["map"],
                            },
                            "projection_policy": {
                                "type": "string",
                                "enum": ["reject", "allow"],
                            },
                        },
                    },
                ]
            },
            "completion_proposal": {
                "anyOf": [
                    {"type": "null"},
                    {
                        "type": "string",
                        "enum": [
                            "report_confirmed", "approach_confirmed",
                            "search_until_exhausted",
                        ],
                    },
                ]
            },
            "rationale": {"type": "string"},
        },
    }
    return json.dumps(schema, sort_keys=True, separators=(",", ":"))


def build_candidate_inspection_schema() -> str:
    """Return the provider-compatible schema for registry candidate checks."""

    return json.dumps({
        "type": "object",
        "additionalProperties": False,
        "required": ["schema_version", "candidate_decisions"],
        "properties": {
            "schema_version": {
                "type": "string",
                "enum": [SCHEMA_VERSION],
            },
            "candidate_decisions": {
                "type": "array",
                "items": {
                    "type": "object",
                    "additionalProperties": False,
                    "required": [
                        "candidate_id", "confirmed", "confidence", "reason_code"
                    ],
                    "properties": {
                        "candidate_id": {"type": "string"},
                        "confirmed": {"type": "boolean"},
                        "confidence": {
                            "type": "number", "minimum": 0.0, "maximum": 1.0
                        },
                        "reason_code": {"type": "string"},
                    },
                },
            },
        },
    }, sort_keys=True, separators=(",", ":"))


class VlmCommanderPlanner:
    """Synchronous callable adapter around ``muto_vlm_socket``.

    The ROS node must be driven by a multi-threaded executor because the
    planner waits for the action result while the action client's response
    callback completes on the executor.  A timeout is mandatory so a missing
    model service becomes ordinary planner evidence rather than a hung
    mission.
    """

    def __init__(
        self,
        node,
        *,
        action_name: str = "/vlm/generate",
        model: str = "",
        timeout_s: float = 30.0,
        jpeg_supplier: Optional[Callable[[], bytes]] = None,
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        try:
            from rclpy.action import ActionClient
            from rclpy.callback_groups import ReentrantCallbackGroup
            from muto_vlm_socket.action import GenerateVlm
            from muto_vlm_socket.msg import VlmContent
        except ImportError as exc:  # pragma: no cover - exercised on non-ROS host
            raise RuntimeError("muto_vlm_socket ROS interfaces are unavailable") from exc

        self._node = node
        self._model = model
        self._timeout_s = float(timeout_s)
        self._jpeg_supplier = jpeg_supplier
        self._active_goal_handle = None
        self._active_lock = threading.RLock()
        self._generate_vlm = GenerateVlm
        self._vlm_content = VlmContent
        self._client = ActionClient(
            node,
            GenerateVlm,
            action_name,
            callback_group=ReentrantCallbackGroup(),
        )

    def __call__(self, board: MissionBoard):
        if not self._client.wait_for_server(timeout_sec=self._timeout_s):
            raise PlannerFailure("v2 VLM action server unavailable")

        image_bytes = b""
        if self._jpeg_supplier is not None:
            image_bytes = self._jpeg_supplier() or b""
        prompt = self._build_prompt(board, visual_available=bool(image_bytes))
        text = self._vlm_content()
        text.type = self._vlm_content.TYPE_TEXT
        text.text = prompt
        content = [text]
        if image_bytes:
            image = self._vlm_content()
            image.type = self._vlm_content.TYPE_JPEG
            image.jpeg_data = list(image_bytes)
            content.append(image)

        goal = self._generate_vlm.Goal()
        goal.content = content
        goal.model = self._model
        goal.response_json_schema = build_decision_schema()

        send_future = self._client.send_goal_async(goal)
        goal_handle = _wait_future(send_future, self._timeout_s, "VLM goal dispatch")
        if not goal_handle.accepted:
            raise PlannerFailure("v2 VLM goal was rejected")
        with self._active_lock:
            self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        try:
            wrapped = _wait_future(result_future, self._timeout_s, "VLM result")
        finally:
            with self._active_lock:
                self._active_goal_handle = None
        result = wrapped.result
        if not result.success:
            raise PlannerFailure(result.error_message or "v2 VLM generation failed")
        if not result.response_text.strip():
            raise PlannerFailure("v2 VLM returned empty planner output")
        return result.response_text

    def cancel(self) -> None:
        with self._active_lock:
            handle = self._active_goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def _build_prompt(self, board: MissionBoard, *, visual_available: bool = False) -> str:
        return build_planner_prompt(
            board,
            visual_available=visual_available,
        )


class VlmCandidateInspector:
    """Use the same independent VLM action to inspect stored candidates.

    The inspector is deliberately separate from normal planning: it receives
    only the current shortlist and its stored evidence, and returns a decision
    for every requested candidate.  The registry adapter still enforces that
    exactly one candidate may be promoted for a revision.
    """

    def __init__(
        self,
        node,
        *,
        action_name: str = "/vlm/generate",
        model: str = "",
        timeout_s: float = 30.0,
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        try:
            from rclpy.action import ActionClient
            from rclpy.callback_groups import ReentrantCallbackGroup
            from muto_vlm_socket.action import GenerateVlm
            from muto_vlm_socket.msg import VlmContent
        except ImportError as exc:  # pragma: no cover - non-ROS host
            raise RuntimeError("muto_vlm_socket ROS interfaces are unavailable") from exc
        self._node = node
        self._model = model
        self._timeout_s = float(timeout_s)
        self._generate_vlm = GenerateVlm
        self._vlm_content = VlmContent
        self._client = ActionClient(
            node, GenerateVlm, action_name, callback_group=ReentrantCallbackGroup()
        )
        self._active_goal_handle = None
        self._active_lock = threading.RLock()

    def cancel(self) -> None:
        with self._active_lock:
            handle = self._active_goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def __call__(
        self,
        object_request: str,
        candidates: Sequence[RegistryCandidate],
        board: MissionBoard,
    ) -> Sequence[CandidateDecision]:
        if not candidates:
            return ()
        if not self._client.wait_for_server(timeout_sec=self._timeout_s):
            raise RuntimeError("v2 VLM action server unavailable for candidate inspection")
        prompt = {
            "schema_version": SCHEMA_VERSION,
            "object_request": object_request,
            "registry_revision": board.registry_revision,
            "candidates": [
                {
                    "candidate_id": candidate.candidate_id,
                    "label": candidate.label,
                    "evidence_id": candidate.evidence_id,
                    "metadata": dict(candidate.metadata),
                }
                for candidate in candidates
            ],
            "instruction": (
                "Compare each stored candidate image with the requested object. "
                "Choose at most one exact match. Return every candidate ID in "
                "candidate_decisions, with confirmed true for the one exact match "
                "and false for all others."
            ),
        }
        text = self._vlm_content()
        text.type = self._vlm_content.TYPE_TEXT
        text.text = json.dumps(prompt, sort_keys=True, separators=(",", ":"))
        content = [text]
        for candidate in candidates:
            path = candidate.metadata.get("image_path", candidate.evidence_id)
            try:
                with open(path, "rb") as stream:
                    image_bytes = stream.read()
            except (OSError, TypeError):
                raise RuntimeError(
                    "candidate evidence is unavailable: {}".format(candidate.candidate_id)
                )
            if not image_bytes:
                raise RuntimeError(
                    "candidate evidence is empty: {}".format(candidate.candidate_id)
                )
            tag = self._vlm_content()
            tag.type = self._vlm_content.TYPE_TEXT
            tag.text = "CANDIDATE_EVIDENCE candidate_id={} evidence_id={}".format(
                candidate.candidate_id, candidate.evidence_id
            )
            content.append(tag)
            image = self._vlm_content()
            image.type = self._vlm_content.TYPE_JPEG
            image.jpeg_data = list(image_bytes)
            content.append(image)
        goal = self._generate_vlm.Goal()
        goal.content = content
        goal.model = self._model
        goal.response_json_schema = build_candidate_inspection_schema()
        handle = _wait_future(
            self._client.send_goal_async(goal), self._timeout_s, "candidate inspection goal"
        )
        if not handle.accepted:
            raise RuntimeError("candidate inspection goal was rejected")
        with self._active_lock:
            self._active_goal_handle = handle
        try:
            wrapped = _wait_future(
                handle.get_result_async(), self._timeout_s, "candidate inspection result"
            )
        finally:
            with self._active_lock:
                self._active_goal_handle = None
        result = wrapped.result
        if not result.success:
            raise RuntimeError(result.error_message or "candidate inspection failed")
        try:
            payload = json.loads(result.response_text)
            if payload.get("schema_version") != SCHEMA_VERSION:
                raise ValueError("unsupported schema version")
            decisions = payload["candidate_decisions"]
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            raise RuntimeError("candidate inspection returned invalid JSON") from exc
        allowed = {candidate.candidate_id for candidate in candidates}
        if {item.get("candidate_id") for item in decisions} != allowed:
            raise RuntimeError("candidate inspection did not decide every candidate")
        return tuple(
            CandidateDecision(
                candidate_id=item["candidate_id"],
                confirmed=bool(item["confirmed"]),
                confidence=float(item["confidence"]),
                reason_code=str(item.get("reason_code", "")),
                source="vlm_candidate_inspection",
                evidence_id=next(
                    (candidate.evidence_id for candidate in candidates
                     if candidate.candidate_id == item["candidate_id"]),
                    "",
                ),
                observed_at_s=time.time(),
            )
            for item in decisions
        )


def _wait_future(future, timeout_s: float, operation: str):
    """Wait for an executor-completed future without spinning a second node."""

    done = threading.Event()
    holder = []

    def _complete(completed):
        holder.append(completed)
        done.set()

    future.add_done_callback(_complete)
    if not done.wait(timeout_s):
        raise PlannerFailure("{} timed out".format(operation))
    completed = holder[0]
    try:
        return completed.result()
    except Exception as exc:
        raise PlannerFailure("{} failed: {}".format(operation, exc)) from exc
