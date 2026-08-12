"""Host-side Muto gait execution."""

from dataclasses import replace

from .base import point3d
from .leg import RealLeg
from .servo import Servo
from ..movement.gait import GaitPlan


class Hexapod:
    def __init__(
            self, serial_port, gait_step_callback=None,
            batch_gait_phase_writes=True):
        self._servo = Servo(serial_port)
        self._legs = [RealLeg(index, self._servo) for index in range(6)]
        self._batch_gait_phase_writes = bool(batch_gait_phase_writes)
        self._gait = GaitPlan()
        self._gait_step_callback = gait_step_callback
        self._command_key = None
        self._pending_command = None
        self.last_callback_error = None

    @property
    def commanded_gait_state(self):
        return self._with_pending_state(self._gait.current_state)

    def set_gait_step_callback(self, callback):
        self._gait_step_callback = callback

    def set_command(self, x_level, y_level, z_level):
        """Latch a desired motion command without advancing the gait."""
        mode, command_key = self._classify_command(x_level, y_level, z_level)

        command = (
            mode,
            command_key,
            float(x_level),
            float(y_level),
            float(z_level),
        )

        # An abrupt safety return to standby is never held behind a gait-cycle
        # boundary: the next tick commands the nominal standing pose directly,
        # rather than performing a smooth deceleration.  Starting from standby
        # is also immediate because there is no in-flight cycle to preserve.
        if mode == 'standby' or self._gait.mode == 'standby':
            changed = (
                command_key != self._command_key
                or self._pending_command is not None
            )
            self._pending_command = None
            if command_key != self._command_key:
                self._apply_command(command)
            return changed

        # A request for the command that is actually executing cancels any
        # queued replacement.  This matters when cmd_vel changes briefly and
        # returns before the current gait cycle is complete.
        if command_key == self._command_key:
            if self._pending_command is None:
                return False
            self._pending_command = None
            return True

        if (
                self._pending_command is not None
                and command_key == self._pending_command[1]):
            return False

        # Applying a new amplitude or motion mode in the middle of a tripod
        # cycle causes an instantaneous target discontinuity.  Keep only the
        # latest request and apply it before the first phase of the next cycle.
        self._pending_command = command
        return True

    def _apply_command(self, command):
        mode, command_key, x_level, y_level, z_level = command
        self._gait.configure(
            mode,
            x_level=x_level,
            y_level=y_level,
            z_level=z_level,
        )
        self._command_key = command_key

    def tick(self, notify=True):
        """Advance and command exactly one trajectory phase."""
        if self._pending_command is not None and self._gait.at_cycle_boundary:
            command = self._pending_command
            self._pending_command = None
            self._apply_command(command)

        complete, target_locations, state = self._gait.next_step()
        if self._batch_gait_phase_writes:
            phase_commands = []
            commanded_targets = []
            for index, leg in enumerate(self._legs):
                target = target_locations.get(index)
                target_world = point3d(target.x, target.y, target.z)
                angles = leg._prepare_tip_move(target_world)
                if angles is not None:
                    phase_commands.append((index, angles))
                    commanded_targets.append((leg, target_world))
            self._servo.set_leg_angles_batch(phase_commands)
            for leg, target_world in commanded_targets:
                leg._commit_tip_move(target_world)
        else:
            for index, leg in enumerate(self._legs):
                target = target_locations.get(index)
                leg.move_tip(point3d(target.x, target.y, target.z))
        state = self._with_pending_state(state)
        if notify:
            self._notify_gait_step(state)
        return complete, state

    def notify_gait_step(self, state):
        """Invoke the optional observer after serial ownership is released."""
        self._notify_gait_step(state)

    def move(self, x_level, y_level, z_level):
        """Block until the requested command reaches a complete boundary."""
        _, requested_key = self._classify_command(
            x_level, y_level, z_level)
        self.set_command(x_level, y_level, z_level)
        state = self._process_cycle()
        while self._command_key != requested_key:
            state = self._process_cycle()
        return state

    def _process_cycle(self):
        complete = False
        state = None
        while not complete:
            complete, state = self.tick()
        return state

    def _with_pending_state(self, state):
        return replace(
            state,
            replacement_pending=self._pending_command is not None,
        )

    def _notify_gait_step(self, state):
        if self._gait_step_callback is None:
            return
        try:
            self._gait_step_callback(state)
        except Exception as exc:  # Keep a diagnostics failure from stopping a gait.
            self.last_callback_error = exc

    @staticmethod
    def _classify_command(x_level, y_level, z_level):
        if x_level == 0 and y_level == 0 and z_level == 0:
            return 'standby', ('standby',)
        if z_level == 0:
            if x_level != 0:
                return 'move_x', ('move_x', _command_key_level(x_level))
            return 'move_y', ('move_y', _command_key_level(y_level))
        if x_level == 0 and y_level == 0:
            return 'turn_z', ('turn_z', _command_key_level(z_level))
        return (
            'move_xz',
            (
                'move_xz',
                _command_key_level(x_level),
                _command_key_level(y_level),
                _command_key_level(z_level),
            ),
        )


def _command_key_level(value):
    """Stable key for comparing continuous command amplitudes."""
    if value == 0:
        return 0.0
    return round(float(value), 6)
