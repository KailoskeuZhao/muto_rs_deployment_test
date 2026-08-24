"""Deterministic natural-language boundary for the v2 mission action.

This adapter is intentionally conservative.  It recognizes the small set of
mission intents supported by v2 and returns a typed rejection for anything it
cannot normalize.  It never selects a skill or invokes a tool.
"""

from dataclasses import dataclass
import re
from typing import Optional, Union

from .contracts import CompletionPolicy, ContractError, MissionAction


@dataclass(frozen=True)
class ActionRejection:
    reason_code: str
    message: str
    clarification: str = ""


@dataclass(frozen=True)
class CancellationRequest:
    request_id: str
    reason_code: str = "cancel_requested"


NormalizedRequest = Union[MissionAction, ActionRejection, CancellationRequest]


_CANCEL_RE = re.compile(r"^(?:please\s+)?(?:stop|cancel|abort)(?:\s+the\s+mission)?[.!?]*$", re.I)
_OBJECT_RE = re.compile(
    r"^(?:please\s+)?(?:find|search\s+for|look\s+for|locate|identify|"
    r"go\s+to|approach)\s+(?:the\s+)?(.+?)[.!?]*$",
    re.I,
)


class NaturalLanguageAdapter:
    """Normalize supported user language before executive acceptance."""

    def normalize(
        self,
        text: str,
        *,
        request_id: str,
        completion_policy: Optional[CompletionPolicy] = None,
    ) -> NormalizedRequest:
        if not isinstance(text, str) or not text.strip():
            return ActionRejection(
                "unsupported_request",
                "The mission request is empty.",
                "Describe the object or search objective.",
            )
        normalized = " ".join(text.strip().split())
        if _CANCEL_RE.match(normalized):
            if not request_id.strip():
                return ActionRejection("invalid_arguments", "A cancellation needs a request id.")
            return CancellationRequest(request_id=request_id)
        match = _OBJECT_RE.match(normalized)
        if not match:
            return ActionRejection(
                "unsupported_request",
                "I could not normalize that into a v2 object mission.",
                "Try 'find the purple chair' or 'go to the confirmed chair'.",
            )
        object_request = match.group(1).strip(" .!?\t")
        if not object_request:
            return ActionRejection(
                "ambiguous_object_reference",
                "The request does not identify an object.",
                "Name the object to find or approach.",
            )
        inferred_policy = completion_policy
        if inferred_policy is None:
            verb_match = re.match(
                r"^(?:please\s+)?(find|search|look|locate|identify|go|approach)\b",
                normalized,
                re.I,
            )
            verb = verb_match.group(1).lower() if verb_match else "find"
            inferred_policy = (
                CompletionPolicy.APPROACH_CONFIRMED
                if verb in {"go", "approach"}
                else CompletionPolicy.REPORT_CONFIRMED
            )
        try:
            policy = CompletionPolicy(inferred_policy)
            return MissionAction(
                request_id=request_id,
                objective=normalized,
                object_request=object_request,
                completion_policy=policy,
            )
        except (ContractError, TypeError, ValueError) as exc:
            return ActionRejection("invalid_arguments", str(exc))
