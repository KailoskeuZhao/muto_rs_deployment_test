from muto_hexapod_interfaces_custom.msg import CommandedGaitState
from muto_hexapod_lib_custom.core.base import point3d
from muto_hexapod_lib_custom.core.config import k_standby
from muto_hexapod_lib_custom.core.leg import RealLeg
from muto_hexapod_lib_custom.movement.gait import GaitPlan
import pytest
from yahboomcar_bringup.foot_odometry_node import FootOdometryNode


def command_angles_for_state(state):
    angles = []
    for leg_index, target in enumerate(state.foot_positions_mm):
        leg = RealLeg(leg_index, servo=None)
        local_target = leg.translate_to_local(point3d.from_tuple(target))
        yaw, pitch, knee = leg.inverse_kinematics(local_target)
        angles.extend((int(yaw), -int(pitch), int(knee)))
    return angles


def motor_payload(state, angles):
    return {
        'angles': list(angles),
        'sample_stamp': {'sec': 12, 'nanosec': 34},
        'angle_space': 'firmware_calibrated_logical_degrees',
        'gait_state': {
            'frame_id': 'base_frame',
            'sequence': state.sequence,
            'mode': state.mode,
            'phase_index': state.phase_index,
            'cycle_length': state.cycle_length,
            'leg_state': [
                CommandedGaitState.STANCE if in_stance
                else CommandedGaitState.SWING
                for in_stance in state.commanded_stance
            ],
            'foot_x_mm': [
                target[1] for target in state.foot_positions_mm
            ],
            'foot_y_mm': [
                -target[0] for target in state.foot_positions_mm
            ],
            'foot_z_mm': [
                target[2] for target in state.foot_positions_mm
            ],
        },
    }


def moving_state():
    plan = GaitPlan()
    plan.configure('move_x', x_level=20)
    return plan.next_step()[2]


def test_calibrated_fk_tracks_generated_stance_targets():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))

    residual_m, sequence = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')

    assert sum(state.commanded_stance) == 3
    assert residual_m < 0.005
    assert sequence == state.sequence


def test_one_bad_stance_leg_exceeds_rejection_threshold():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))
    stance_index = state.commanded_stance.index(True)
    payload['angles'][stance_index * 3] = 80

    residual_m, _ = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')

    assert residual_m > 0.03


def test_swing_leg_error_does_not_corrupt_stance_validation():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))
    baseline_m, _ = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')
    swing_index = state.commanded_stance.index(False)
    payload['angles'][swing_index * 3] = 80

    residual_m, _ = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')

    assert residual_m == pytest.approx(baseline_m)


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('angle_space', 'raw_servo_degrees'),
        ('frame_id', 'base_link'),
    ),
)
def test_motor_sample_contract_rejects_wrong_space_or_frame(field, value):
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))
    if field == 'angle_space':
        payload[field] = value
    else:
        payload['gait_state'][field] = value

    with pytest.raises(ValueError):
        FootOdometryNode.motor_tracking_residual(payload, 'base_frame')


def test_standby_calibration_pose_matches_nominal_geometry():
    state = GaitPlan().current_state
    payload = motor_payload(state, [0, -30, -15] * 6)

    residual_m, _ = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')

    assert tuple(state.foot_positions_mm) == tuple(k_standby)
    assert residual_m < 0.0001


def test_motor_source_timestamp_is_validated():
    assert FootOdometryNode.motor_sample_stamp_sec({
        'sample_stamp': {'sec': 12, 'nanosec': 34},
    }) == pytest.approx(12.000000034)

    with pytest.raises(ValueError):
        FootOdometryNode.motor_sample_stamp_sec({
            'sample_stamp': {'sec': 12, 'nanosec': 1_000_000_000},
        })
