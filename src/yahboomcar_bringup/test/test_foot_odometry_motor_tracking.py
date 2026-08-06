from types import SimpleNamespace

from builtin_interfaces.msg import Time
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


def test_production_motor_poll_rate_requires_explicit_high_rate_opt_in():
    assert FootOdometryNode.validate_motor_poll_rate(2.0, False) == 2.0
    assert FootOdometryNode.validate_motor_poll_rate(10.0, True) == 10.0

    with pytest.raises(ValueError, match='production limit'):
        FootOdometryNode.validate_motor_poll_rate(10.0, False)
    with pytest.raises(ValueError, match='finite and positive'):
        FootOdometryNode.validate_motor_poll_rate(0.0, False)


def test_calibrated_fk_tracks_generated_stance_targets():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))

    residual_m, sequence, mode = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')

    assert sum(state.commanded_stance) == 3
    assert residual_m < 0.005
    assert sequence == state.sequence
    assert mode == state.mode


def test_motor_payload_produces_measured_fk_observation():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))

    residual_m, observation, stamp = (
        FootOdometryNode.measured_motor_observation(
            payload, 'base_frame')
    )

    assert residual_m < 0.005
    assert observation.sequence == state.sequence
    assert observation.mode == state.mode
    assert observation.stamp_sec == pytest.approx(12.000000034)
    assert stamp == Time(sec=12, nanosec=34)
    assert len(observation.foot_x_m) == 6
    assert len(observation.foot_y_m) == 6
    assert observation.foot_x_m != pytest.approx(
        tuple(target[1] * 0.001 for target in state.foot_positions_mm),
        abs=1e-9,
    )


def test_one_bad_stance_leg_exceeds_rejection_threshold():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))
    stance_index = state.commanded_stance.index(True)
    payload['angles'][stance_index * 3] = 80

    residual_m, _, _ = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')

    assert residual_m > 0.03


def test_swing_leg_error_does_not_corrupt_stance_validation():
    state = moving_state()
    payload = motor_payload(state, command_angles_for_state(state))
    baseline_m, _, _ = FootOdometryNode.motor_tracking_residual(
        payload, 'base_frame')
    swing_index = state.commanded_stance.index(False)
    payload['angles'][swing_index * 3] = 80

    residual_m, _, _ = FootOdometryNode.motor_tracking_residual(
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

    residual_m, _, _ = FootOdometryNode.motor_tracking_residual(
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


def test_motor_validation_cannot_cross_a_gait_mode_transition():
    node = SimpleNamespace(
        motor_tracking_accepted=True,
        last_motor_stamp_sec=12.0,
        last_motor_gait_sequence=5,
        last_motor_gait_mode='move_x',
        motor_stale_timeout=1.0,
        last_motor_tracking_residual_m=0.004,
        motor_tracking_good_residual_m=0.005,
        motor_tracking_reject_residual_m=0.03,
        clamp=FootOdometryNode.clamp,
    )
    now = SimpleNamespace(nanoseconds=12_500_000_000)

    assert FootOdometryNode.motor_tracking_confidence(
        node, now, 6, 'move_x') == 1.0
    assert FootOdometryNode.motor_tracking_confidence(
        node, now, 6, 'turn_z') is None


class FakePublisher:
    def __init__(self):
        self.message = None

    def publish(self, message):
        self.message = message


class FakeFootNode:
    set_covariance = staticmethod(FootOdometryNode.set_covariance)
    yaw_to_quaternion = staticmethod(FootOdometryNode.yaw_to_quaternion)

    def __init__(self):
        self.frame_id = 'odom'
        self.child_frame_id = 'base_frame'
        self.x = 1.25
        self.y = -0.5
        self.yaw = 0.2
        self.vx = 0.3
        self.vy = -0.1
        self.wz = 0.4
        self.unobserved_pose_variance = 1.0
        self.unobserved_twist_variance = 1.0
        self.odom_pub = FakePublisher()
        self.tf_broadcaster = None


def test_published_odometry_uses_source_stamp_and_bounded_covariance():
    node = FakeFootNode()
    stamp = Time(sec=12, nanosec=34)

    FootOdometryNode.publish_odometry(node, stamp, confidence=1.0)

    message = node.odom_pub.message
    assert message.header.stamp == stamp
    assert message.header.frame_id == 'odom'
    assert message.child_frame_id == 'base_frame'
    assert [message.pose.covariance[index] for index in (0, 7, 14, 21, 28, 35)] == [
        0.5, 0.5, 1.0, 1.0, 1.0, 0.8,
    ]
    assert [message.twist.covariance[index] for index in (0, 7, 14, 21, 28, 35)] == [
        0.2, 0.2, 1.0, 1.0, 1.0, 0.4,
    ]
    assert max(message.pose.covariance) == 1.0
    assert max(message.twist.covariance) == 1.0


class FakeEstimator:
    def __init__(self):
        self.reset_called = False

    def reset(self):
        self.reset_called = True


class FakeLogger:
    def warn(self, *_args, **_kwargs):
        pass


class FakeFrameCheckingNode:
    def __init__(self):
        self.child_frame_id = 'base_frame'
        self.estimator = FakeEstimator()

    @staticmethod
    def get_logger():
        return FakeLogger()


def test_wrong_gait_frame_resets_estimator_before_processing():
    node = FakeFrameCheckingNode()
    message = CommandedGaitState()
    message.header.frame_id = 'base_link'

    FootOdometryNode.gait_state_callback(node, message)

    assert node.estimator.reset_called
