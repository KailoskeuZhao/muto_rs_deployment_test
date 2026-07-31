from dataclasses import replace
import math

from muto_hexapod_lib_custom.movement.gait import GaitPlan
import pytest
from yahboomcar_bringup.commanded_gait_odometry import (
    CommandedStanceOdometry,
    GaitObservation,
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
