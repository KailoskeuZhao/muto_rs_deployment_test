import math

from muto_hexapod_lib_custom.core.base import point3d
from muto_hexapod_lib_custom.core.config import (
    k_standby,
    STANDBY_SERVO_ANGLES_DEG,
)
from muto_hexapod_lib_custom.core.leg import (
    RealLeg,
    servo_angles_to_foot_positions,
)
from muto_hexapod_lib_custom.movement.gait import (
    GaitPlan,
    LEG_NAMES,
)


TRIPOD_A = {0, 2, 4}
TRIPOD_B = {1, 3, 5}


def cycle_states(plan):
    return [plan.next_step()[2] for _ in range(plan.cycle_length)]


def stance_set(state):
    return {
        index
        for index, commanded_stance in enumerate(state.commanded_stance)
        if commanded_stance
    }


def test_leg_order_is_physical_clockwise_order():
    assert LEG_NAMES == (
        'right_front',
        'right_middle',
        'right_rear',
        'left_rear',
        'left_middle',
        'left_front',
    )


def test_forward_cycle_reports_alternating_commanded_support():
    plan = GaitPlan()
    plan.configure('move_x', x_level=10)

    states = cycle_states(plan)

    assert [state.phase_index for state in states] == list(range(1, 20)) + [0]
    assert states[-1].cycle_complete
    assert all(state.cycle_length == 20 for state in states)

    for state in states:
        support = stance_set(state)
        if state.phase_index in (5, 15):
            assert support == set(range(6))
        elif 6 <= state.phase_index <= 14:
            assert support == TRIPOD_A
        else:
            assert support == TRIPOD_B


def test_generated_modes_never_claim_fewer_than_three_support_legs():
    commands = (
        ('move_x', {'x_level': 20}),
        ('move_y', {'y_level': -20}),
        ('turn_z', {'z_level': 15}),
        ('move_xz', {'x_level': 20, 'z_level': 15}),
    )

    for mode, levels in commands:
        plan = GaitPlan()
        plan.configure(mode, **levels)
        support_counts = [
            sum(state.commanded_stance)
            for state in cycle_states(plan)
        ]
        assert set(support_counts).issubset({3, 6})


def test_standby_reports_all_six_legs_on_nominal_ground_plane():
    plan = GaitPlan()

    complete, _, state = plan.next_step()

    assert complete
    assert state.mode == 'standby'
    assert state.commanded_stance == (True,) * 6


def servo_angles_for_targets(targets):
    result = []
    for leg_index, target in enumerate(targets):
        leg = RealLeg(leg_index, servo=None)
        local = leg.translate_to_local(point3d.from_tuple(target))
        yaw, pitch, knee = leg.inverse_kinematics(local)
        result.extend((yaw, -pitch, knee))
    return result


def test_factory_standby_logical_angles_reconstruct_standby_feet():
    angles = STANDBY_SERVO_ANGLES_DEG * 6

    feet = servo_angles_to_foot_positions(angles)

    for actual, expected in zip(feet, k_standby):
        for actual_axis, expected_axis in zip(actual, expected):
            assert math.isclose(
                actual_axis, expected_axis, abs_tol=0.03)


def test_forward_kinematics_inverts_generated_gait_targets():
    plan = GaitPlan()
    plan.configure('move_xz', x_level=20, z_level=15)

    for state in cycle_states(plan):
        angles = servo_angles_for_targets(state.foot_positions_mm)
        feet = servo_angles_to_foot_positions(angles)
        for actual, expected in zip(feet, state.foot_positions_mm):
            for actual_axis, expected_axis in zip(actual, expected):
                assert math.isclose(
                    actual_axis, expected_axis, abs_tol=0.005)


def test_forward_kinematics_rejects_invalid_motor_samples():
    bad_samples = (
        [0.0] * 17,
        [0.0] * 17 + [math.nan],
        [0.0] * 17 + [91.0],
    )
    for angles in bad_samples:
        try:
            servo_angles_to_foot_positions(angles)
        except ValueError:
            continue
        raise AssertionError('invalid motor sample was accepted')
