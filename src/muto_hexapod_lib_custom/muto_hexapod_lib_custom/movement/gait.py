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
# A turn level is a common tangential foot displacement in millimetres.  For
# unequal stance radii r_i, least-squares fitting z_level to theta*r_i gives
# theta = z_level*sum(r_i)/sum(r_i**2), hence the effective radius below.  It
# is 249.94 mm for this geometry (and is identical for the two tripods).
_STANDBY_RADII_MM = tuple(
    math.hypot(point[0], point[1]) for point in k_standby
)
GAIT_TURN_EFFECTIVE_RADIUS_MM = (
    sum(radius ** 2 for radius in _STANDBY_RADII_MM)
    / sum(_STANDBY_RADII_MM)
)


@dataclass(frozen=True)
class CommandedGaitState:
    """One emitted trajectory step; stance means nominal ground-plane target."""

    sequence: int
    mode: str
    x_level: float
    y_level: float
    z_level: float
    phase_index: int
    cycle_length: int
    cycle_complete: bool
    commanded_stance: tuple
    foot_positions_mm: tuple
    replacement_pending: bool = False


def _semicircle_profile(reverse=False):
    """Return the normalized fore-aft coordinate and shared lift profile."""
    half_steps = GAIT_STEPS // 2
    step_angle = math.pi / half_steps
    step_stride = 2.0 / half_steps
    result = []

    for index in range(half_steps):
        result.append((1.0 - index * step_stride, 0.0))

    for index in range(half_steps):
        angle = math.pi - step_angle * index
        result.append((
            math.cos(angle),
            GAIT_LIFT_MM * math.sin(angle),
        ))

    result = deque(result)
    result.rotate(GAIT_STEPS // 4)
    if reverse:
        result = deque(reversed(result))
        result.rotate(1)
    return list(result)


def _semicircle_path(stride_mm, reverse=False):
    return [
        (0.0, stride_mm * q, lift_mm)
        for q, lift_mm in _semicircle_profile(reverse=reverse)
    ]


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


def _sinc(value):
    """Numerically stable sin(value) / value."""
    if abs(value) < 1e-4:
        squared = value * value
        return 1.0 - squared / 6.0 + squared * squared / 120.0
    return math.sin(value) / value


def _cosc(value):
    """Numerically stable (1 - cos(value)) / value."""
    if abs(value) < 1e-4:
        squared = value * value
        return value * (
            0.5 - squared / 24.0 + squared * squared / 720.0
        )
    return (1.0 - math.cos(value)) / value


def _forward_paths(level):
    path = _semicircle_path(abs(level), reverse=level < 0)
    shifted = _half_cycle(path)
    return [path, shifted, path, shifted, path, shifted]


def _lateral_paths(level):
    path = _semicircle_path(abs(level))
    path = _rotate_path(path, 90 if level >= 0 else 270)
    shifted = _half_cycle(path)
    return [path, shifted, path, shifted, path, shifted]


def _twist_paths(x_level, z_level):
    """Generate an exact finite planar body-twist foot trajectory.

    In ROS planar coordinates let
    ``xi = (x_level, 0, z_level / GAIT_TURN_EFFECTIVE_RADIUS_MM)``.
    For profile coordinate ``q`` we use ``tau = -q`` and the rigid transform
    ``T(tau) = Exp_SE2(tau * xi^)``.  A stationary nominal contact then has
    body-frame target ``p(q) = T(tau)^-1 p(0)``.  The same endpoint-consistent
    q profile is used for swing, with its lift added only once.
    """
    profile = _semicircle_profile()
    shifted_profile = _half_cycle(profile)
    leg_profiles = [
        profile,
        shifted_profile,
        profile,
        shifted_profile,
        profile,
        shifted_profile,
    ]
    yaw_amplitude = z_level / GAIT_TURN_EFFECTIVE_RADIUS_MM
    return [
        [
            _inverse_twist_offset(
                k_standby[leg_index],
                q,
                lift_mm,
                x_level,
                yaw_amplitude,
            )
            for q, lift_mm in leg_profile
        ]
        for leg_index, leg_profile in enumerate(leg_profiles)
    ]


def _inverse_twist_offset(
        nominal_vendor, q, lift_mm, forward_mm, yaw_amplitude):
    """Return vendor-axis offset for ``Exp_SE2(-q * twist)^-1``."""
    tau = -q
    theta = tau * yaw_amplitude
    translation_x = tau * forward_mm * _sinc(theta)
    translation_y = tau * forward_mm * _cosc(theta)

    # Vendor body axes are x=right/y=forward; ROS planar axes are
    # x=forward/y=left.
    nominal_ros_x = nominal_vendor[1]
    nominal_ros_y = -nominal_vendor[0]
    relative_x = nominal_ros_x - translation_x
    relative_y = nominal_ros_y - translation_y
    cosine = math.cos(theta)
    sine = math.sin(theta)
    target_ros_x = cosine * relative_x + sine * relative_y
    target_ros_y = -sine * relative_x + cosine * relative_y
    target_vendor_x = -target_ros_y
    target_vendor_y = target_ros_x
    return (
        target_vendor_x - nominal_vendor[0],
        target_vendor_y - nominal_vendor[1],
        lift_mm,
    )


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
        self._active_levels = (0, 0, 0)

    @property
    def cycle_length(self):
        return len(self._targets)

    @property
    def at_cycle_boundary(self):
        """Whether the last emitted phase completed the current gait cycle."""
        return self._phase_index == 0

    @property
    def current_state(self):
        return self._state_for(self._phase_index, self._current, True)

    def configure(
            self, mode, x_level=0, y_level=0, z_level=0,
            preserve_phase=False):
        previous_phase_index = self._phase_index
        previous_cycle_length = self.cycle_length
        if mode == 'standby':
            active_levels = (0, 0, 0)
            targets = [list(k_standby)]
        elif mode == 'move_x':
            active_levels = (float(x_level), 0, 0)
            targets = _targets_from_offsets(
                _forward_paths(active_levels[0]))
        elif mode == 'move_y':
            active_levels = (0, float(y_level), 0)
            targets = _targets_from_offsets(
                _lateral_paths(active_levels[1]))
        elif mode == 'turn_z':
            active_levels = (0, 0, float(z_level))
            # Use the same rigid-body construction as combined motion.  This
            # keeps the target path continuous as forward speed crosses zero
            # and avoids the inherited per-leg tangent approximation.
            targets = _targets_from_offsets(
                _twist_paths(0, active_levels[2]))
        elif mode == 'move_xz':
            # The supported combined gait is forward plus yaw.  Lateral input
            # is not active because it is not represented by this trajectory.
            active_levels = (float(x_level), 0, float(z_level))
            targets = _targets_from_offsets(
                _twist_paths(active_levels[0], active_levels[2])
            )
        else:
            raise ValueError('unsupported gait mode: %s' % mode)

        self.mode = mode
        self._active_levels = active_levels
        self._targets = targets
        if (preserve_phase and previous_cycle_length > 1
                and len(targets) > 1):
            self._phase_index = previous_phase_index % len(targets)
        else:
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
            x_level=self._active_levels[0],
            y_level=self._active_levels[1],
            z_level=self._active_levels[2],
            phase_index=phase_index,
            cycle_length=self.cycle_length,
            cycle_complete=cycle_complete,
            commanded_stance=stance,
            foot_positions_mm=points,
        )
