import math

from muto_hexapod_lib_custom.core.base import point3d
from muto_hexapod_lib_custom.core.config import (
    leg_joint1_2joint2,
    leg_joint2_2joint3,
    leg_joint3_2tip,
    leg_root2joint1,
    k_standby,
    STANDBY_SERVO_ANGLES_DEG,
)
from muto_hexapod_lib_custom.core.leg import (
    MOUNT_POSITIONS,
    RealLeg,
    servo_angles_to_leg_joint_chains,
    servo_angles_to_leg_joint_positions,
    servo_angles_to_foot_positions,
)
from muto_hexapod_lib_custom.movement.gait import (
    GAIT_LIFT_MM,
    GAIT_TURN_EFFECTIVE_RADIUS_MM,
    GaitPlan,
    LEG_NAMES,
)
import pytest


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


def fit_planar_transform(source_points, target_points):
    """Fit target = R(source) + translation and return yaw and residual."""
    count = len(source_points)
    source_center = tuple(
        sum(point[axis] for point in source_points) / count
        for axis in range(2)
    )
    target_center = tuple(
        sum(point[axis] for point in target_points) / count
        for axis in range(2)
    )
    dot_sum = 0.0
    cross_sum = 0.0
    for source, target in zip(source_points, target_points):
        source_x = source[0] - source_center[0]
        source_y = source[1] - source_center[1]
        target_x = target[0] - target_center[0]
        target_y = target[1] - target_center[1]
        dot_sum += source_x * target_x + source_y * target_y
        cross_sum += source_x * target_y - source_y * target_x
    yaw = math.atan2(cross_sum, dot_sum)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translation_x = target_center[0] - (
        cosine * source_center[0] - sine * source_center[1]
    )
    translation_y = target_center[1] - (
        sine * source_center[0] + cosine * source_center[1]
    )
    squared_error = 0.0
    for source, target in zip(source_points, target_points):
        fitted_x = (
            cosine * source[0] - sine * source[1] + translation_x
        )
        fitted_y = (
            sine * source[0] + cosine * source[1] + translation_y
        )
        squared_error += (
            (target[0] - fitted_x) ** 2
            + (target[1] - fitted_y) ** 2
        )
    return yaw, math.sqrt(squared_error / count)


def nominal_cycle_fit(mode, **levels):
    """Infer nominal body yaw and worst fit residual from stance feet."""
    plan = GaitPlan()
    plan.configure(mode, **levels)
    states = [plan.next_step()[2] for _ in range(plan.cycle_length + 1)]
    cycle_yaw = 0.0
    max_residual_mm = 0.0
    for previous, current in zip(states, states[1:]):
        support = [
            index
            for index in range(6)
            if previous.commanded_stance[index]
            and current.commanded_stance[index]
        ]
        source = [
            current.foot_positions_mm[index][:2]
            for index in support
        ]
        target = [
            previous.foot_positions_mm[index][:2]
            for index in support
        ]
        yaw, residual_mm = fit_planar_transform(source, target)
        cycle_yaw += yaw
        max_residual_mm = max(max_residual_mm, residual_mm)
    return cycle_yaw, max_residual_mm


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


@pytest.mark.parametrize(
    ('mode', 'levels', 'expected_levels'),
    (
        ('standby', {}, (0, 0, 0)),
        ('move_x', {'x_level': -20}, (-20, 0, 0)),
        ('move_y', {'y_level': 15}, (0, 15, 0)),
        ('turn_z', {'z_level': -10}, (0, 0, -10)),
        (
            'move_xz',
            {'x_level': 20, 'y_level': 7, 'z_level': 15},
            (20, 0, 15),
        ),
    ),
)
def test_generated_state_reports_only_actually_active_levels(
        mode, levels, expected_levels):
    plan = GaitPlan()
    plan.configure(mode, **levels)

    state = plan.next_step()[2]

    assert (state.x_level, state.y_level, state.z_level) == expected_levels
    assert not state.replacement_pending


def test_composed_twist_retains_one_shared_lift_trajectory():
    plan = GaitPlan()
    plan.configure('move_xz', x_level=30, z_level=20)

    lift_heights = [
        point[2] - k_standby[leg_index][2]
        for state in cycle_states(plan)
        for leg_index, point in enumerate(state.foot_positions_mm)
    ]

    assert min(lift_heights) == pytest.approx(0.0, abs=0.01)
    assert max(lift_heights) == pytest.approx(GAIT_LIFT_MM, abs=0.01)


def test_composed_twist_yaw_authority_is_independent_of_forward_level():
    expected_yaw = 4.0 * 20.0 / GAIT_TURN_EFFECTIVE_RADIUS_MM

    for x_level in (-30, -10, 0, 5, 10, 20, 30):
        actual_yaw, _ = nominal_cycle_fit(
            'move_xz', x_level=x_level, z_level=20)
        # Targets are rounded to 0.01 mm before being reported.
        assert actual_yaw == pytest.approx(expected_yaw, abs=1e-4)


def test_pure_turn_uses_same_exact_path_as_zero_forward_twist():
    for z_level in (-20, -10, 10, 20):
        pure_turn = GaitPlan()
        pure_turn.configure('turn_z', z_level=z_level)
        zero_forward = GaitPlan()
        zero_forward.configure('move_xz', x_level=0, z_level=z_level)

        assert [
            state.foot_positions_mm for state in cycle_states(pure_turn)
        ] == [
            state.foot_positions_mm for state in cycle_states(zero_forward)
        ]

        actual_yaw, max_residual_mm = nominal_cycle_fit(
            'turn_z', z_level=z_level)
        expected_yaw = 4.0 * z_level / GAIT_TURN_EFFECTIVE_RADIUS_MM
        assert actual_yaw == pytest.approx(expected_yaw, abs=1e-4)
        assert max_residual_mm <= 0.01


def test_composed_twist_stance_feet_share_one_exact_rigid_transform():
    for x_level in (-30, 5, 30):
        for z_level in (-20, 10, 20):
            _, max_residual_mm = nominal_cycle_fit(
                'move_xz', x_level=x_level, z_level=z_level)
            assert max_residual_mm <= 0.01


def test_zero_yaw_composed_twist_matches_legacy_forward_path():
    for x_level in (-30, -10, 10, 30):
        forward = GaitPlan()
        forward.configure('move_x', x_level=x_level)
        twist = GaitPlan()
        twist.configure('move_xz', x_level=x_level, z_level=0)

        forward_states = cycle_states(forward)
        twist_states = cycle_states(twist)
        assert [
            state.foot_positions_mm for state in twist_states
        ] == [
            state.foot_positions_mm for state in forward_states
        ]


def test_standby_reports_all_six_legs_on_nominal_ground_plane():
    plan = GaitPlan()

    complete, _, state = plan.next_step()

    assert complete
    assert state.mode == 'standby'
    assert state.commanded_stance == (True,) * 6


def test_command_amplitude_update_can_preserve_gait_phase():
    plan = GaitPlan()
    plan.configure('move_x', x_level=10)
    first = plan.next_step()[2]

    plan.configure('move_x', x_level=20, preserve_phase=True)
    second = plan.next_step()[2]

    assert first.phase_index == 1
    assert second.phase_index == 2
    assert second.sequence == first.sequence + 1


def servo_angles_for_targets(targets):
    result = []
    for leg_index, target in enumerate(targets):
        leg = RealLeg(leg_index, servo=None)
        local = leg.translate_to_local(point3d.from_tuple(target))
        yaw, pitch, knee = leg.inverse_kinematics(local)
        result.extend((yaw, -pitch, knee))
    return result


def test_leg_commands_round_to_nearest_controller_degree(monkeypatch):
    commands = []

    class CapturingServo:
        def set_leg_angles(self, leg_index, angles):
            commands.append((leg_index, angles))

    leg = RealLeg(0, CapturingServo())
    monkeypatch.setattr(
        leg,
        'inverse_kinematics',
        lambda _target: (1.6, 2.6, -3.6),
    )

    leg.move_tip(point3d(1.0, 2.0, 3.0))

    assert commands == [(0, (2, -3, -4))]


def test_factory_standby_logical_angles_reconstruct_standby_feet():
    angles = STANDBY_SERVO_ANGLES_DEG * 6

    feet = servo_angles_to_foot_positions(angles)

    for actual, expected in zip(feet, k_standby):
        for actual_axis, expected_axis in zip(actual, expected):
            assert math.isclose(
                actual_axis, expected_axis, abs_tol=0.03)


def distance_mm(left, right):
    return math.sqrt(sum(
        (left[axis] - right[axis]) ** 2
        for axis in range(3)
    ))


def test_forward_kinematics_returns_full_nominal_leg_segment_chain():
    chain = servo_angles_to_leg_joint_positions(
        0, STANDBY_SERVO_ANGLES_DEG)

    assert len(chain) == 5
    assert chain[0] == pytest.approx(MOUNT_POSITIONS[0].as_tuple())
    assert distance_mm(chain[0], chain[1]) == pytest.approx(
        leg_root2joint1, abs=0.03)
    assert distance_mm(chain[1], chain[2]) == pytest.approx(
        leg_joint1_2joint2, abs=0.03)
    assert distance_mm(chain[2], chain[3]) == pytest.approx(
        leg_joint2_2joint3, abs=0.03)
    assert distance_mm(chain[3], chain[4]) == pytest.approx(
        leg_joint3_2tip, abs=0.03)
    assert chain[-1] == pytest.approx(k_standby[0], abs=0.03)

    chains = servo_angles_to_leg_joint_chains(
        STANDBY_SERVO_ANGLES_DEG * 6)
    assert len(chains) == 6
    for chain, expected_foot in zip(chains, k_standby):
        assert chain[-1] == pytest.approx(expected_foot, abs=0.03)


def test_inverse_kinematics_rejects_unreachable_targets():
    leg = RealLeg(0, servo=None)

    with pytest.raises(ValueError, match='unreachable leg target'):
        leg.inverse_kinematics(point3d(10000.0, 0.0, 0.0))


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


def test_supported_gait_limits_remain_inside_servo_command_range():
    commands = (
        ('move_x', {'x_level': -30}),
        ('move_x', {'x_level': 30}),
        ('move_y', {'y_level': -30}),
        ('move_y', {'y_level': 30}),
        ('turn_z', {'z_level': -20}),
        ('turn_z', {'z_level': 20}),
    ) + tuple(
        ('move_xz', {'x_level': x_level, 'z_level': z_level})
        for x_level in (-30, 30)
        for z_level in (-20, 20)
    )

    for mode, levels in commands:
        plan = GaitPlan()
        plan.configure(mode, **levels)
        for state in cycle_states(plan):
            angles = servo_angles_for_targets(state.foot_positions_mm)
            assert all(-90.0 <= angle <= 90.0 for angle in angles)


def test_every_supported_composed_level_remains_ik_safe():
    yaw_levels = tuple(range(-20, -1)) + tuple(range(2, 21))

    for x_level in range(-30, 31):
        for z_level in yaw_levels:
            plan = GaitPlan()
            plan.configure(
                'move_xz', x_level=x_level, z_level=z_level)
            for state in cycle_states(plan):
                angles = servo_angles_for_targets(state.foot_positions_mm)
                assert all(-90.0 <= angle <= 90.0 for angle in angles)


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
