"""Planar Muto stance-foot odometry estimators."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GaitObservation:
    """One gait phase expressed in ROS base-frame coordinates."""

    sequence: int
    mode: str
    phase_index: int
    cycle_length: int
    stamp_sec: float
    leg_state: tuple
    foot_x_m: tuple
    foot_y_m: tuple

    def validate(self):
        arrays = (self.leg_state, self.foot_x_m, self.foot_y_m)
        if any(len(values) != 6 for values in arrays):
            raise ValueError('gait observations require six legs')
        if self.cycle_length < 1:
            raise ValueError('cycle_length must be positive')
        if not 0 <= self.phase_index < self.cycle_length:
            raise ValueError('phase_index is outside the gait cycle')
        if not math.isfinite(self.stamp_sec):
            raise ValueError('gait timestamp must be finite')
        if not all(
                math.isfinite(value)
                for values in (self.foot_x_m, self.foot_y_m)
                for value in values):
            raise ValueError('foot targets must be finite')


@dataclass(frozen=True)
class GaitIncrement:
    """Body motion from the previous phase to the current phase."""

    dx: float
    dy: float
    dyaw: float
    dt: float
    vx: float
    vy: float
    wz: float
    support_count: int
    residual_m: float


class CommandedStanceOdometry:
    """
    Fit body motion using feet commanded in stance in both phases.

    Commanded stance is treated as stationary in the odom frame. This remains
    open-loop kinematic odometry: it cannot observe contact loss or slip.
    """

    def __init__(
        self,
        stance_value=1,
        min_common_stance=3,
        max_fit_residual_m=0.01,
        max_translation_step_m=0.05,
        max_rotation_step_rad=0.2,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=2.0,
        min_sample_dt=0.001,
        max_sample_dt=2.0,
    ):
        self.stance_value = int(stance_value)
        self.min_common_stance = max(2, int(min_common_stance))
        self.max_fit_residual_m = max(0.0, float(max_fit_residual_m))
        self.max_translation_step_m = max(
            0.0, float(max_translation_step_m))
        self.max_rotation_step_rad = max(
            0.0, float(max_rotation_step_rad))
        self.max_linear_speed_mps = max(
            0.0, float(max_linear_speed_mps))
        self.max_angular_speed_radps = max(
            0.0, float(max_angular_speed_radps))
        self.min_sample_dt = max(0.0, float(min_sample_dt))
        self.max_sample_dt = max(
            self.min_sample_dt, float(max_sample_dt))
        self._previous = None

    def reset(self):
        self._previous = None

    def update(self, observation):
        observation.validate()
        previous = self._previous
        self._previous = observation

        if previous is None:
            return None
        if observation.mode == 'standby' or observation.mode != previous.mode:
            return None
        if observation.sequence != previous.sequence + 1:
            return None
        if observation.cycle_length != previous.cycle_length:
            return None
        expected_phase = (previous.phase_index + 1) % previous.cycle_length
        if observation.phase_index != expected_phase:
            return None

        dt = observation.stamp_sec - previous.stamp_sec
        if dt < self.min_sample_dt or dt > self.max_sample_dt:
            return None

        support = [
            index
            for index in range(6)
            if previous.leg_state[index] == self.stance_value
            and observation.leg_state[index] == self.stance_value
        ]
        if len(support) < self.min_common_stance:
            return None

        previous_points = [
            (previous.foot_x_m[index], previous.foot_y_m[index])
            for index in support
        ]
        current_points = [
            (observation.foot_x_m[index], observation.foot_y_m[index])
            for index in support
        ]
        dx, dy, dyaw, residual = self._fit_planar_transform(
            current_points, previous_points)

        if residual > self.max_fit_residual_m:
            return None
        if math.hypot(dx, dy) > self.max_translation_step_m:
            return None
        if abs(dyaw) > self.max_rotation_step_rad:
            return None
        if math.hypot(dx, dy) / dt > self.max_linear_speed_mps:
            return None
        if abs(dyaw) / dt > self.max_angular_speed_radps:
            return None

        # The generated pure-turn gait has millimetre-scale centroid drift
        # from rounded path rotations. It is not commanded body translation.
        if observation.mode == 'turn_z':
            dx = 0.0
            dy = 0.0

        return GaitIncrement(
            dx=dx,
            dy=dy,
            dyaw=dyaw,
            dt=dt,
            vx=dx / dt,
            vy=dy / dt,
            wz=dyaw / dt,
            support_count=len(support),
            residual_m=residual,
        )

    @staticmethod
    def _fit_planar_transform(source_points, target_points):
        """Fit target = R(source) + translation without NumPy."""
        count = len(source_points)
        source_center = (
            sum(point[0] for point in source_points) / count,
            sum(point[1] for point in source_points) / count,
        )
        target_center = (
            sum(point[0] for point in target_points) / count,
            sum(point[1] for point in target_points) / count,
        )

        dot_sum = 0.0
        cross_sum = 0.0
        for source, target in zip(source_points, target_points):
            sx = source[0] - source_center[0]
            sy = source[1] - source_center[1]
            tx = target[0] - target_center[0]
            ty = target[1] - target_center[1]
            dot_sum += sx * tx + sy * ty
            cross_sum += sx * ty - sy * tx

        yaw = math.atan2(cross_sum, dot_sum)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        dx = target_center[0] - (
            cosine * source_center[0] - sine * source_center[1])
        dy = target_center[1] - (
            sine * source_center[0] + cosine * source_center[1])

        squared_error = 0.0
        for source, target in zip(source_points, target_points):
            fitted_x = cosine * source[0] - sine * source[1] + dx
            fitted_y = sine * source[0] + cosine * source[1] + dy
            squared_error += (
                (target[0] - fitted_x) ** 2
                + (target[1] - fitted_y) ** 2
            )
        residual = math.sqrt(squared_error / count)
        return dx, dy, yaw, residual


class MeasuredStanceOdometry:
    """Fit body motion from consecutive measured joint-FK foot positions."""

    def __init__(
        self,
        stance_value=1,
        min_common_stance=3,
        max_sequence_gap=10,
        max_fit_residual_m=0.01,
        max_translation_step_m=0.05,
        max_rotation_step_rad=0.2,
        max_linear_speed_mps=1.0,
        max_angular_speed_radps=2.0,
        min_sample_dt=0.001,
        max_sample_dt=2.0,
    ):
        self.stance_value = int(stance_value)
        self.min_common_stance = max(2, int(min_common_stance))
        self.max_sequence_gap = max(1, int(max_sequence_gap))
        self.max_fit_residual_m = max(0.0, float(max_fit_residual_m))
        self.max_translation_step_m = max(
            0.0, float(max_translation_step_m))
        self.max_rotation_step_rad = max(
            0.0, float(max_rotation_step_rad))
        self.max_linear_speed_mps = max(
            0.0, float(max_linear_speed_mps))
        self.max_angular_speed_radps = max(
            0.0, float(max_angular_speed_radps))
        self.min_sample_dt = max(0.0, float(min_sample_dt))
        self.max_sample_dt = max(
            self.min_sample_dt, float(max_sample_dt))
        self._previous = None
        self.last_rejection_reason = 'waiting for first measured sample'

    def reset(self):
        self._previous = None
        self.last_rejection_reason = 'waiting for first measured sample'

    def update(self, observation, gait_history):
        """Return an increment only when support stayed planted throughout."""
        observation.validate()
        previous = self._previous
        self._previous = observation

        if previous is None:
            self.last_rejection_reason = 'waiting for second measured sample'
            return None
        if observation.mode == 'standby':
            self.last_rejection_reason = 'standby sample'
            return None
        if observation.mode != previous.mode:
            self.last_rejection_reason = 'gait mode changed between samples'
            return None

        sequence_gap = observation.sequence - previous.sequence
        if sequence_gap <= 0:
            self.last_rejection_reason = 'motor sample sequence did not advance'
            return None
        if sequence_gap > self.max_sequence_gap:
            self.last_rejection_reason = (
                f'motor samples span {sequence_gap} gait phases; maximum is '
                f'{self.max_sequence_gap}'
            )
            return None

        dt = observation.stamp_sec - previous.stamp_sec
        if dt < self.min_sample_dt or dt > self.max_sample_dt:
            self.last_rejection_reason = (
                f'motor sample interval {dt:.3f} s is outside '
                f'[{self.min_sample_dt:.3f}, {self.max_sample_dt:.3f}] s'
            )
            return None

        history_by_sequence = {
            state.sequence: state
            for state in gait_history
            if previous.sequence <= state.sequence <= observation.sequence
        }
        expected_sequences = range(
            previous.sequence, observation.sequence + 1)
        if any(sequence not in history_by_sequence
               for sequence in expected_sequences):
            self.last_rejection_reason = (
                'gait history is incomplete between measured samples'
            )
            return None

        interval = [
            history_by_sequence[sequence]
            for sequence in expected_sequences
        ]
        if any(state.mode != observation.mode for state in interval):
            self.last_rejection_reason = (
                'gait mode changed inside the measured interval'
            )
            return None

        support = [
            index
            for index in range(6)
            if all(
                state.leg_state[index] == self.stance_value
                for state in interval
            )
        ]
        if len(support) < self.min_common_stance:
            self.last_rejection_reason = (
                f'only {len(support)} feet remained in stance; need '
                f'{self.min_common_stance}'
            )
            return None

        previous_points = [
            (previous.foot_x_m[index], previous.foot_y_m[index])
            for index in support
        ]
        current_points = [
            (observation.foot_x_m[index], observation.foot_y_m[index])
            for index in support
        ]
        dx, dy, dyaw, residual = (
            CommandedStanceOdometry._fit_planar_transform(
                current_points, previous_points)
        )

        if residual > self.max_fit_residual_m:
            self.last_rejection_reason = (
                f'measured stance fit residual {residual:.4f} m exceeds '
                f'{self.max_fit_residual_m:.4f} m'
            )
            return None
        translation = math.hypot(dx, dy)
        if translation > self.max_translation_step_m:
            self.last_rejection_reason = (
                f'measured translation step {translation:.4f} m exceeds '
                f'{self.max_translation_step_m:.4f} m'
            )
            return None
        if abs(dyaw) > self.max_rotation_step_rad:
            self.last_rejection_reason = (
                f'measured rotation step {dyaw:.4f} rad exceeds '
                f'{self.max_rotation_step_rad:.4f} rad'
            )
            return None
        if translation / dt > self.max_linear_speed_mps:
            self.last_rejection_reason = (
                f'measured linear speed {translation / dt:.3f} m/s exceeds '
                f'{self.max_linear_speed_mps:.3f} m/s'
            )
            return None
        if abs(dyaw) / dt > self.max_angular_speed_radps:
            self.last_rejection_reason = (
                f'measured yaw rate {abs(dyaw) / dt:.3f} rad/s exceeds '
                f'{self.max_angular_speed_radps:.3f} rad/s'
            )
            return None

        self.last_rejection_reason = ''
        return GaitIncrement(
            dx=dx,
            dy=dy,
            dyaw=dyaw,
            dt=dt,
            vx=dx / dt,
            vy=dy / dt,
            wz=dyaw / dt,
            support_count=len(support),
            residual_m=residual,
        )
