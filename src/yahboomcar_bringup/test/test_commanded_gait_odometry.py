from dataclasses import replace
import math

from muto_hexapod_lib_custom.movement.gait import GaitPlan
import pytest
from yahboomcar_bringup.commanded_gait_odometry import (
    CommandedStanceOdometry,
    GaitObservation,
    MeasuredStanceOdometry,
)


STANCE = 1
SWING = 0


def observation(state, stamp_sec):
    # The custom library uses x=right/y=forward. The ROS message boundary
    # converts that to base_frame x=forward/y=left.
    return GaitObservation(
        sequence=state.sequence,
        mode=state.mode,
        phase_index=state.phase_index,
        cycle_length=state.cycle_length,
        stamp_sec=stamp_sec,
        leg_state=tuple(
            STANCE if in_stance else SWING
            for in_stance in state.commanded_stance
        ),
        foot_x_m=tuple(
            point[1] * 0.001 for point in state.foot_positions_mm),
        foot_y_m=tuple(
            -point[0] * 0.001 for point in state.foot_positions_mm),
    )


def gait_increments(mode, **levels):
    plan = GaitPlan()
    plan.configure(mode, **levels)
    estimator = CommandedStanceOdometry()
    increments = []
    for index in range(21):
        state = plan.next_step()[2]
        increment = estimator.update(observation(state, (index + 1) * 0.05))
        if increment is not None:
            increments.append(increment)
    return increments


@pytest.mark.parametrize(
    ('mode', 'levels', 'axis', 'expected'),
    (
        ('move_x', {'x_level': 20}, 'x', 0.08),
        ('move_x', {'x_level': -20}, 'x', -0.08),
        ('move_y', {'y_level': 20}, 'y', 0.08),
        ('move_y', {'y_level': -20}, 'y', -0.08),
    ),
)
def test_tripod_translation_uses_common_stance_feet(
        mode, levels, axis, expected):
    increments = gait_increments(mode, **levels)

    assert len(increments) == 20
    assert {increment.support_count for increment in increments} == {3}
    total_x = sum(increment.dx for increment in increments)
    total_y = sum(increment.dy for increment in increments)
    actual = total_x if axis == 'x' else total_y
    cross_axis = total_y if axis == 'x' else total_x
    assert actual == pytest.approx(expected, abs=1e-6)
    assert cross_axis == pytest.approx(0.0, abs=1e-6)
    assert sum(increment.dyaw for increment in increments) == pytest.approx(
        0.0, abs=1e-6)


def test_turn_and_mixed_gaits_produce_expected_nominal_motion():
    turn = gait_increments('turn_z', z_level=15)
    mixed = gait_increments('move_xz', x_level=20, z_level=15)

    assert sum(increment.dyaw for increment in turn) > math.radians(10.0)
    assert sum(increment.dx for increment in turn) == pytest.approx(0.0)
    assert sum(increment.dy for increment in turn) == pytest.approx(0.0)
    assert sum(increment.dx for increment in mixed) > 0.07
    assert sum(increment.dyaw for increment in mixed) > math.radians(2.0)
    assert max(increment.residual_m for increment in turn + mixed) < 0.001


def test_missing_phase_resets_the_increment_baseline():
    plan = GaitPlan()
    plan.configure('move_x', x_level=20)
    states = [plan.next_step()[2] for _ in range(4)]
    estimator = CommandedStanceOdometry()

    assert estimator.update(observation(states[0], 0.05)) is None
    assert estimator.update(observation(states[1], 0.10)) is not None
    assert estimator.update(observation(states[3], 0.20)) is None


def test_bad_stance_fit_is_rejected():
    plan = GaitPlan()
    plan.configure('move_x', x_level=20)
    first = observation(plan.next_step()[2], 0.05)
    second = observation(plan.next_step()[2], 0.10)
    stance_index = next(
        index for index, state in enumerate(second.leg_state)
        if state == STANCE and first.leg_state[index] == STANCE
    )
    bad_x = list(second.foot_x_m)
    bad_x[stance_index] += 0.05
    second = replace(second, foot_x_m=tuple(bad_x))
    estimator = CommandedStanceOdometry(max_fit_residual_m=0.005)

    assert estimator.update(first) is None
    assert estimator.update(second) is None


def test_implausible_phase_velocity_is_rejected():
    plan = GaitPlan()
    plan.configure('move_x', x_level=20)
    first = observation(plan.next_step()[2], 1.0)
    second = observation(plan.next_step()[2], 1.002)

    permissive = CommandedStanceOdometry(max_linear_speed_mps=10.0)
    assert permissive.update(first) is None
    assert permissive.update(second) is not None

    guarded = CommandedStanceOdometry(max_linear_speed_mps=1.0)
    assert guarded.update(first) is None
    assert guarded.update(second) is None


def measured_observation(sequence, stamp_sec, points, leg_state=None):
    if leg_state is None:
        leg_state = (STANCE, STANCE, STANCE, SWING, SWING, SWING)
    return GaitObservation(
        sequence=sequence,
        mode='move_x',
        phase_index=sequence % 20,
        cycle_length=20,
        stamp_sec=stamp_sec,
        leg_state=tuple(leg_state),
        foot_x_m=tuple(point[0] for point in points),
        foot_y_m=tuple(point[1] for point in points),
    )


def inverse_body_transform(points, dx, dy, dyaw):
    cosine = math.cos(-dyaw)
    sine = math.sin(-dyaw)
    return tuple(
        (
            cosine * (x - dx) - sine * (y - dy),
            sine * (x - dx) + cosine * (y - dy),
        )
        for x, y in points
    )


def test_measured_joint_estimator_fits_continuously_planted_feet():
    previous_points = (
        (0.20, 0.15), (0.22, 0.0), (0.20, -0.15),
        (-0.20, -0.15), (-0.22, 0.0), (-0.20, 0.15),
    )
    current_points = inverse_body_transform(
        previous_points, dx=0.02, dy=-0.01, dyaw=0.05)
    previous = measured_observation(10, 1.0, previous_points)
    current = measured_observation(13, 1.1, current_points)
    history = [
        measured_observation(sequence, 1.0, previous_points)
        for sequence in range(10, 14)
    ]
    estimator = MeasuredStanceOdometry(max_sequence_gap=5)

    assert estimator.update(previous, history) is None
    increment = estimator.update(current, history)

    assert increment is not None
    assert increment.support_count == 3
    assert increment.dx == pytest.approx(0.02)
    assert increment.dy == pytest.approx(-0.01)
    assert increment.dyaw == pytest.approx(0.05)
    assert increment.residual_m < 1e-12


def test_measured_joint_estimator_rejects_intervening_swing():
    points = (
        (0.20, 0.15), (0.22, 0.0), (0.20, -0.15),
        (-0.20, -0.15), (-0.22, 0.0), (-0.20, 0.15),
    )
    previous = measured_observation(10, 1.0, points)
    current = measured_observation(12, 1.1, points)
    changed_support = (STANCE, STANCE, SWING, SWING, SWING, SWING)
    history = [
        previous,
        measured_observation(11, 1.05, points, changed_support),
        current,
    ]
    estimator = MeasuredStanceOdometry(max_sequence_gap=5)

    assert estimator.update(previous, history) is None
    assert estimator.update(current, history) is None
    assert estimator.last_rejection_reason == (
        'only 2 feet remained in stance; need 3'
    )


def test_measured_joint_estimator_rejects_sparse_motor_samples():
    points = tuple((0.1 * index, 0.0) for index in range(6))
    previous = measured_observation(10, 1.0, points)
    current = measured_observation(21, 1.5, points)
    estimator = MeasuredStanceOdometry(max_sequence_gap=10)

    assert estimator.update(previous, [previous]) is None
    assert estimator.update(current, [previous, current]) is None
    assert estimator.last_rejection_reason == (
        'motor samples span 11 gait phases; maximum is 10'
    )
