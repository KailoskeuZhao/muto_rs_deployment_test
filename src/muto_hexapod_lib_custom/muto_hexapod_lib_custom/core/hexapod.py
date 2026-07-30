"""Host-side Muto gait execution."""

from .base import point3d
from .leg import RealLeg
from .servo import Servo
from ..movement.gait import GaitPlan


class Hexapod:
    def __init__(self, serial_port, gait_step_callback=None):
        servo = Servo(serial_port)
        self._legs = [RealLeg(index, servo) for index in range(6)]
        self._gait = GaitPlan()
        self._gait_step_callback = gait_step_callback
        self._command_key = None
        self.last_callback_error = None

    @property
    def commanded_gait_state(self):
        return self._gait.current_state

    def set_gait_step_callback(self, callback):
        self._gait_step_callback = callback

    def move(self, x_level, y_level, z_level):
        mode, command_key = self._classify_command(x_level, y_level, z_level)
        if command_key != self._command_key:
            self._gait.configure(
                mode,
                x_level=x_level,
                y_level=y_level,
                z_level=z_level,
            )
            self._command_key = command_key
        self._process_cycle()

    def _process_cycle(self):
        complete = False
        while not complete:
            complete, target_locations, state = self._gait.next_step()
            for index, leg in enumerate(self._legs):
                target = target_locations.get(index)
                leg.move_tip(point3d(target.x, target.y, target.z))
            self._notify_gait_step(state)

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
                return 'move_x', ('move_x', int(x_level))
            return 'move_y', ('move_y', int(y_level))
        if x_level == 0 and y_level == 0:
            return 'turn_z', ('turn_z', int(z_level))
        return (
            'move_xz',
            ('move_xz', int(x_level), int(y_level), int(z_level)),
        )
