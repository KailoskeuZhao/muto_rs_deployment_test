"""Generated 20-step Muto gait with commanded stance metadata."""

from collections import deque
from dataclasses import dataclass
import math

from ..core.base import locations
from ..core.config import k_standby, LEG_NAMES  # noqa: F401


GAIT_STEPS = 20
GAIT_LIFT_MM = 25.0
# Generated targets are rounded to 0.01 mm, while k_standby retains the
# original floating-point geometry.
GROUND_EPSILON_MM = 0.01


@dataclass(frozen=True)
class CommandedGaitState:
    """One emitted trajectory step; stance means nominal ground-plane target."""

    sequence: int
    mode: str
    phase_index: int
    cycle_length: int
    cycle_complete: bool
    commanded_stance: tuple
    foot_positions_mm: tuple


def _semicircle_path(stride_mm, reverse=False):
    half_steps = GAIT_STEPS // 2
    step_angle = math.pi / half_steps
    step_stride = 2.0 * stride_mm / half_steps
    result = []

    for index in range(half_steps):
        result.append((0.0, stride_mm - index * step_stride, 0.0))

    for index in range(half_steps):
        angle = math.pi - step_angle * index
        result.append((
            0.0,
            stride_mm * math.cos(angle),
            GAIT_LIFT_MM * math.sin(angle),
        ))

    result = deque(result)
    result.rotate(GAIT_STEPS // 4)
    if reverse:
        result = deque(reversed(result))
        result.rotate(1)
    return list(result)


def _rotate_path(path, angle_degrees):
    angle = math.radians(angle_degrees)
    cosine = round(math.cos(angle), 2)
    sine = round(math.sin(angle), 2)
    return [
        (
            point[0] * cosine - point[1] * sine,
            point[0] * sine + point[1] * cosine,
            point[2],
        )
        for point in path
    ]


def _half_cycle(path):
    shifted = deque(path)
    shifted.rotate(GAIT_STEPS // 2)
    return list(shifted)


def _forward_paths(level):
    path = _semicircle_path(abs(level), reverse=level < 0)
    shifted = _half_cycle(path)
    return [path, shifted, path, shifted, path, shifted]


def _lateral_paths(level):
    path = _semicircle_path(abs(level))
    path = _rotate_path(path, 90 if level >= 0 else 270)
    shifted = _half_cycle(path)
    return [path, shifted, path, shifted, path, shifted]


def _turn_paths(level):
    rotation = 0 if level >= 0 else 180
    path = _semicircle_path(abs(level))
    shifted = _half_cycle(path)
    return [
        _rotate_path(path, 45 + rotation),
        _rotate_path(shifted, rotation),
        _rotate_path(path, 315 + rotation),
        _rotate_path(shifted, 255 + rotation),
        _rotate_path(path, 180 + rotation),
        _rotate_path(shifted, 135 + rotation),
    ]


def _mixed_paths(x_level, _y_level, z_level):
    reverse = x_level < 0
    angular = round(z_level / 180.0 * math.pi, 2)
    velocity_offset = round(x_level * math.sin(angular), 2)
    left_stride = abs(int(x_level - velocity_offset))
    right_stride = abs(int(x_level + velocity_offset))

    left = _semicircle_path(left_stride, reverse=reverse)
    right = _semicircle_path(right_stride, reverse=reverse)
    shifted_left = _half_cycle(left)
    shifted_right = _half_cycle(right)
    return [right, shifted_right, right, shifted_left, left, shifted_left]


def _targets_from_offsets(offset_paths):
    return [
        [
            (
                round(k_standby[leg][0] + offset_paths[leg][phase][0], 2),
                round(k_standby[leg][1] + offset_paths[leg][phase][1], 2),
                round(k_standby[leg][2] + offset_paths[leg][phase][2], 2),
            )
            for leg in range(6)
        ]
        for phase in range(GAIT_STEPS)
    ]


class GaitPlan:
    """Stateful plan preserving Yahboom's phase progression between cycles."""

    def __init__(self):
        self.mode = 'standby'
        self._targets = [list(k_standby)]
        self._phase_index = 0
        self._sequence = 0
        self._current = locations.from_list(k_standby)

    @property
    def cycle_length(self):
        return len(self._targets)

    @property
    def current_state(self):
        return self._state_for(self._phase_index, self._current, True)

    def configure(self, mode, x_level=0, y_level=0, z_level=0):
        if mode == 'standby':
            targets = [list(k_standby)]
        elif mode == 'move_x':
            targets = _targets_from_offsets(_forward_paths(x_level))
        elif mode == 'move_y':
            targets = _targets_from_offsets(_lateral_paths(y_level))
        elif mode == 'turn_z':
            targets = _targets_from_offsets(_turn_paths(z_level))
        elif mode == 'move_xz':
            targets = _targets_from_offsets(
                _mixed_paths(x_level, y_level, z_level)
            )
        else:
            raise ValueError('unsupported gait mode: %s' % mode)

        self.mode = mode
        self._targets = targets
        self._phase_index = 0

    def next_step(self):
        self._phase_index = (self._phase_index + 1) % self.cycle_length
        self._current = locations.from_list(self._targets[self._phase_index])
        self._sequence += 1
        cycle_complete = self._phase_index == 0
        return (
            cycle_complete,
            self._current,
            self._state_for(
                self._phase_index,
                self._current,
                cycle_complete,
            ),
        )

    def _state_for(self, phase_index, foot_locations, cycle_complete):
        points = foot_locations.as_tuples()
        stance = tuple(
            abs(point[2] - k_standby[index][2]) <= GROUND_EPSILON_MM
            for index, point in enumerate(points)
        )
        return CommandedGaitState(
            sequence=self._sequence,
            mode=self.mode,
            phase_index=phase_index,
            cycle_length=self.cycle_length,
            cycle_complete=cycle_complete,
            commanded_stance=stance,
            foot_positions_mm=points,
        )
