"""Internal exception taxonomy for the model commander.

Keeping these types independent from the ROS node makes failure handling
explicit across the scheduler, bounded input workers, and visual codec.
"""


class CommanderCanceled(RuntimeError):
    """Raised when the parent mission is canceled."""


class CommanderFailure(RuntimeError):
    """Raised when an owned dependency cannot complete safely."""


class OwnedGoalStateUnknown(CommanderFailure):
    """Raised when an accepted owned goal cannot be confirmed stopped."""


class PlannerFailure(CommanderFailure):
    """Raised for model transport or validated-protocol failures."""


class ActiveMonitoringFailure(CommanderFailure):
    """Raised when required in-flight visual monitoring is unavailable."""


class StalePlan(RuntimeError):
    """Raised when monitored state changes during model inference."""


class ChildCompletedDuringInspection(RuntimeError):
    """Raised when the command under inspection reaches a terminal state."""


class WaitCompletedDuringInspection(RuntimeError):
    """Raised when a bounded wait ends during a visual inspection."""


class MissionBudgetExhausted(RuntimeError):
    """Raised when a locally enforced mission budget expires."""


class InputFlowFailure(RuntimeError):
    """Raised when a bounded transient-input worker cannot make progress."""
