"""Small event-driven commander/executive runtime.

This is the integration seam between a model adapter and deterministic tool
backends.  It deliberately has no ROS imports, so the same loop can be used
by Humble nodes and by trace-asserting scenario tests.
"""

from dataclasses import dataclass
from threading import Event, RLock
from typing import Callable, Optional

from .commander import CommanderAgent, PlannerFailure
from .contracts import ContractError, MissionAction, MissionBoard
from .executive import MissionExecutive
from .tools import ToolDispatcher


@dataclass(frozen=True)
class RuntimeResult:
    board: MissionBoard
    decisions: int


class CommanderRuntime:
    """Run one accepted mission until the executive reaches a terminal state."""

    def __init__(
        self,
        executive: MissionExecutive,
        commander: CommanderAgent,
        dispatcher: ToolDispatcher,
        *,
        consecutive_failure_limit: int = 3,
        no_progress_limit: int = 5,
        on_update: Optional[Callable[[MissionBoard, tuple], None]] = None,
    ) -> None:
        if consecutive_failure_limit <= 0 or no_progress_limit <= 0:
            raise ValueError("runtime failure limits must be positive")
        self.executive = executive
        self.commander = commander
        self.dispatcher = dispatcher
        self.consecutive_failure_limit = int(consecutive_failure_limit)
        self.no_progress_limit = int(no_progress_limit)
        self.on_update = on_update
        self._cancel_requested = Event()
        self._lock = RLock()

    def request_cancel(self) -> None:
        """Request cancellation and propagate it to the active authority."""

        self._cancel_requested.set()
        cancel_commander = getattr(self.commander, "cancel", None)
        if callable(cancel_commander):
            cancel_commander()
        self.dispatcher.cancel_active()

    def run(
        self,
        action: MissionAction,
        *,
        cancel_check: Optional[Callable[[], bool]] = None,
    ) -> RuntimeResult:
        self.executive.accept(action)
        self.executive.configure_limits(
            "consecutive_failures={}".format(self.consecutive_failure_limit),
            "no_progress={}".format(self.no_progress_limit),
        )
        self._notify()
        self.executive.start()
        self._notify()
        decisions = 0
        while not self.executive.board.lifecycle_state.terminal:
            if self._cancel_requested.is_set() or (
                cancel_check is not None and cancel_check()
            ):
                self.executive.cancel()
                self._notify()
                break
            decisions += 1
            try:
                decision = self.commander.decide(self.executive.board)
            except PlannerFailure as exc:
                self.executive.record_planner_failure("invalid_skill_request", str(exc))
                self._notify()
                if self._limit_reached():
                    break
                continue
            if self.executive.board.active_skill != decision.skill:
                self.executive.select_skill(decision.skill)
                self._notify()
            if decision.tool is None:
                if not decision.completion_proposal:
                    self.executive.record_planner_failure(
                        "missing_tool_or_completion", decision.rationale
                    )
                    self._notify()
                    if self._limit_reached():
                        break
                    continue
                self._apply_completion(decision.completion_proposal)
                self._notify()
                if self._limit_reached():
                    break
                continue
            result = None
            try:
                self.executive.record_tool_request(decision.tool.tool)
                self._notify()
                result = self.dispatcher.dispatch(decision.tool, self.executive.board)
                self.executive.record_tool_result(
                    decision.tool.tool,
                    success=result.success,
                    reason_code=result.reason_code,
                    detail=result.detail,
                    candidate_id=decision.tool.candidate_id,
                    confirmed_target_id=result.confirmed_target_id,
                    candidate_ids=result.candidate_ids,
                    rejected_candidate_ids=result.rejected_candidate_ids,
                    registry_revision=result.registry_revision,
                    progress_delta=result.progress_delta,
                    reachability=result.reachability,
                    expected_registry_revision=decision.tool.registry_revision,
                    evidence=result.evidence,
                )
                self._notify()
            except Exception as exc:  # backend errors become nonfatal tool evidence
                self.executive.record_tool_result(
                    decision.tool.tool,
                    success=False,
                    reason_code="tool_exception",
                    detail=str(exc),
                    candidate_id=decision.tool.candidate_id,
                    candidate_ids=getattr(result, "candidate_ids", ()),
                    rejected_candidate_ids=getattr(
                        result, "rejected_candidate_ids", ()
                    ),
                    registry_revision=(
                        getattr(result, "registry_revision", "")
                        or decision.tool.registry_revision
                    ),
                    expected_registry_revision=decision.tool.registry_revision,
                    evidence=getattr(result, "evidence", ()),
                )
                self._notify()
                if self._limit_reached():
                    break
        return RuntimeResult(self.executive.board, decisions)

    def _limit_reached(self) -> bool:
        """Turn repeated local failure/no-progress into a terminal outcome.

        This is a per-runtime guard, not a mission-wide time or distance
        budget. A successful tool resets the consecutive-failure counter, so
        ordinary backend failures remain evidence and are replanned first.
        """

        board = self.executive.board
        if board.lifecycle_state.terminal:
            return True
        if board.consecutive_failures >= self.consecutive_failure_limit:
            self.executive.fail("consecutive_failures")
            self._notify()
            return True
        if board.no_progress_count >= self.no_progress_limit:
            self.executive.fail("no_progress")
            self._notify()
            return True
        return False

    def _apply_completion(self, proposal: str) -> None:
        try:
            policy = self.executive.board.completion_policy
            if policy is None or proposal != policy.value:
                raise ContractError(
                    "completion proposal does not match the scenario policy"
                )
            if proposal == "report_confirmed":
                self.executive.complete(
                    confirmed_target_id=self.executive.board.confirmed_target_id
                )
            elif proposal == "approach_confirmed":
                self.executive.complete(
                    confirmed_target_id=self.executive.board.confirmed_target_id,
                    approach_complete=True,
                )
            elif proposal == "search_until_exhausted":
                self.executive.complete(search_exhausted=True, reason_code="search_exhausted")
            else:
                raise ValueError("unsupported completion proposal")
        except Exception as exc:
            self.executive.record_planner_failure("invalid_completion_proposal", str(exc))

    def _notify(self) -> None:
        if self.on_update is not None:
            self.on_update(self.executive.board, self.executive.events)
