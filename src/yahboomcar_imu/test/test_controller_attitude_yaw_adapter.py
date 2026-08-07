import math

from muto_hexapod_interfaces_custom.msg import ControllerAttitude
from muto_hexapod_interfaces_custom.msg import MotionCommandState
import pytest

from yahboomcar_imu.controller_attitude_yaw_adapter import (
    circular_mean,
    circular_span,
    controller_attitude_to_imu,
    controller_snapshot,
    ControllerAttitudeYawAdapter,
    DEFAULT_YAW_VARIANCE_RAD2,
    motion_state_is_stationary,
    StationaryAttitudeGate,
    UNUSED_ORIENTATION_VARIANCE,
    wrap_angle,
    yaw_quaternion,
)


def make_attitude(yaw_deg=0.0, roll_deg=0.0, pitch_deg=0.0):
    message = ControllerAttitude()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.roll_deg = roll_deg
    message.pitch_deg = pitch_deg
    message.yaw_deg = yaw_deg
    return message


@pytest.mark.parametrize(
    ('yaw_deg', 'expected_z', 'expected_w'),
    [
        (0.0, 0.0, 1.0),
        (90.0, math.sqrt(0.5), math.sqrt(0.5)),
        (-90.0, -math.sqrt(0.5), math.sqrt(0.5)),
        (180.0, 1.0, 0.0),
    ],
)
def test_yaw_quaternion_uses_ros_positive_counterclockwise_sign(
    yaw_deg, expected_z, expected_w
):
    x, y, z, w = yaw_quaternion(math.radians(yaw_deg))

    assert x == 0.0
    assert y == 0.0
    assert z == pytest.approx(expected_z)
    assert w == pytest.approx(expected_w, abs=1.0e-12)


def test_adapter_output_is_yaw_only_and_preserves_receive_stamp():
    source = make_attitude(
        yaw_deg=92.86,
        roll_deg=1.2,
        pitch_deg=-2.3,
    )

    output = controller_attitude_to_imu(
        source,
        'imu_link',
        DEFAULT_YAW_VARIANCE_RAD2,
    )

    assert output.header.stamp == source.header.stamp
    assert output.header.frame_id == 'imu_link'
    assert output.orientation.x == 0.0
    assert output.orientation.y == 0.0
    assert output.orientation.z == pytest.approx(
        math.sin(math.radians(92.86) * 0.5)
    )
    assert output.orientation.w == pytest.approx(
        math.cos(math.radians(92.86) * 0.5)
    )
    assert output.orientation_covariance[0] == UNUSED_ORIENTATION_VARIANCE
    assert output.orientation_covariance[4] == UNUSED_ORIENTATION_VARIANCE
    assert output.orientation_covariance[8] == pytest.approx(
        math.radians(4.0) ** 2
    )
    assert output.angular_velocity_covariance[0] == -1.0
    assert output.linear_acceleration_covariance[0] == -1.0


def test_full_euler_snapshot_distinguishes_fresh_cached_values():
    first = make_attitude(yaw_deg=10.0, roll_deg=1.0, pitch_deg=2.0)
    repeated = make_attitude(yaw_deg=10.0, roll_deg=1.0, pitch_deg=2.0)
    fresh_same_yaw = make_attitude(
        yaw_deg=10.0,
        roll_deg=1.01,
        pitch_deg=2.0,
    )

    assert controller_snapshot(first) == controller_snapshot(repeated)
    assert controller_snapshot(first) != controller_snapshot(fresh_same_yaw)


class FakePublisher:
    def __init__(self, subscription_count=1):
        self.messages = []
        self.subscription_count = subscription_count

    def get_subscription_count(self):
        return self.subscription_count

    def publish(self, message):
        self.messages.append(message)


class FakeAdapter:
    suppress_identical_snapshots = True
    stationary_gate_enabled = False
    last_snapshot = None
    received_count = 0
    changed_count = 0
    published_count = 0
    duplicate_count = 0
    waiting_for_subscriber_count = 0
    frame_id = 'imu_link'
    yaw_variance_rad2 = DEFAULT_YAW_VARIANCE_RAD2

    def __init__(self):
        self.publisher = FakePublisher()


def test_callback_suppresses_cached_packet_but_keeps_fresh_same_yaw():
    adapter = FakeAdapter()
    first = make_attitude(yaw_deg=10.0, roll_deg=1.0)
    cached = make_attitude(yaw_deg=10.0, roll_deg=1.0)
    fresh_same_yaw = make_attitude(yaw_deg=10.0, roll_deg=1.01)

    for message in (first, cached, fresh_same_yaw):
        ControllerAttitudeYawAdapter.attitude_callback(adapter, message)

    assert adapter.received_count == 3
    assert adapter.changed_count == 2
    assert adapter.published_count == 2
    assert adapter.duplicate_count == 1
    assert len(adapter.publisher.messages) == 2


def test_callback_preserves_first_anchor_until_output_subscriber_is_ready():
    adapter = FakeAdapter()
    adapter.publisher.subscription_count = 0
    first = make_attitude(yaw_deg=10.0)

    ControllerAttitudeYawAdapter.attitude_callback(adapter, first)

    assert adapter.last_snapshot is None
    assert adapter.published_count == 0
    assert adapter.waiting_for_subscriber_count == 1

    adapter.publisher.subscription_count = 1
    ControllerAttitudeYawAdapter.attitude_callback(adapter, first)

    assert adapter.last_snapshot == controller_snapshot(first)
    assert adapter.published_count == 1
    assert len(adapter.publisher.messages) == 1


@pytest.mark.parametrize('bad_yaw', [math.nan, math.inf, -math.inf])
def test_nonfinite_controller_angles_are_rejected(bad_yaw):
    with pytest.raises(ValueError, match='non-finite'):
        controller_attitude_to_imu(
            make_attitude(yaw_deg=bad_yaw),
            'imu_link',
            DEFAULT_YAW_VARIANCE_RAD2,
        )


@pytest.mark.parametrize('bad_variance', [0.0, -1.0, math.nan, math.inf])
def test_nonpositive_or_nonfinite_yaw_variance_is_rejected(bad_variance):
    with pytest.raises(ValueError, match='variance'):
        controller_attitude_to_imu(
            make_attitude(),
            'imu_link',
            bad_variance,
        )


def make_motion_state(stationary=True):
    message = MotionCommandState()
    message.mode = 'standby' if stationary else 'turn_z'
    message.active_mode = 'standby' if stationary else 'turn_z'
    if not stationary:
        message.z_level = 10
        message.active_z_level = 10
        message.requested_twist.angular.z = 0.2
    return message


def test_motion_gate_requires_selected_and_active_standby():
    message = make_motion_state(stationary=True)
    assert motion_state_is_stationary(message)

    message.active_mode = 'move_x'
    assert not motion_state_is_stationary(message)
    message.active_mode = 'standby'
    message.active_x_level = 1
    assert not motion_state_is_stationary(message)
    message.active_x_level = 0
    message.replacement_pending = True
    assert not motion_state_is_stationary(message)


def test_stationary_gate_publishes_startup_anchor_then_stable_mean():
    gate = StationaryAttitudeGate(
        stationary_settle_sec=2.0,
        motion_state_timeout_sec=0.25,
        stability_window_sec=1.0,
        minimum_distinct_snapshots=3,
        stationary_republish_interval_sec=0.0,
    )
    gate.update_motion(True, 0.0)
    yaw, reason = gate.update_attitude(math.radians(10.0), 0.0)
    assert yaw == pytest.approx(math.radians(10.0))
    assert reason == 'initial_anchor'

    for index, yaw_deg in enumerate(
        (10.1, 9.9, 10.0, 10.1, 9.9, 10.0, 10.1, 9.9, 10.0, 10.1),
        start=1,
    ):
        stamp = index * 0.2
        gate.update_motion(True, stamp)
        yaw, reason = gate.update_attitude(math.radians(yaw_deg), stamp)

    assert reason == 'stationary_correction'
    assert math.degrees(yaw) == pytest.approx(10.02, abs=0.05)

    gate.update_motion(True, 2.2)
    assert gate.update_attitude(math.radians(10.0), 2.2) == (
        None, 'correction_held'
    )


def test_stationary_gate_uses_real_relative_turn_after_motion():
    gate = StationaryAttitudeGate(
        stationary_settle_sec=0.4,
        stability_window_sec=0.5,
        minimum_distinct_snapshots=3,
        stationary_republish_interval_sec=0.0,
    )
    gate.update_motion(True, 0.0)
    assert gate.update_attitude(0.0, 0.0)[1] == 'initial_anchor'

    gate.update_motion(False, 0.2)
    assert gate.update_attitude(math.radians(45.0), 0.2) == (
        None, 'moving'
    )
    gate.update_motion(True, 1.0)
    for index, yaw_deg in enumerate((90.0, 90.1, 89.9), start=1):
        stamp = 1.0 + index * 0.2
        gate.update_motion(True, stamp)
        yaw, reason = gate.update_attitude(math.radians(yaw_deg), stamp)

    assert reason == 'stationary_correction'
    assert math.degrees(yaw) == pytest.approx(90.0, abs=0.05)


def test_stationary_gate_blocks_large_attitude_step_until_motion():
    gate = StationaryAttitudeGate(stationary_settle_sec=0.0)
    gate.update_motion(True, 0.0)
    assert gate.update_attitude(0.0, 0.0)[1] == 'initial_anchor'

    gate.update_motion(True, 0.2)
    assert gate.update_attitude(math.radians(3.0), 0.2) == (
        None, 'magnetic_guard'
    )
    assert gate.blocked_until_motion

    gate.update_motion(False, 0.3)
    assert not gate.blocked_until_motion


def test_stationary_gate_fails_closed_on_stale_motion_state():
    gate = StationaryAttitudeGate(
        stationary_settle_sec=0.0,
        motion_state_timeout_sec=0.25,
    )
    gate.update_motion(True, 0.0)

    yaw, reason = gate.update_attitude(0.0, 0.3)
    assert yaw is None
    assert reason == 'stale_motion_state'


def test_stationary_gate_refuses_relative_anchor_after_motion():
    gate = StationaryAttitudeGate()
    gate.update_motion(False, 0.0)
    gate.update_motion(True, 1.0)

    assert gate.update_attitude(math.radians(90.0), 1.0) == (
        None, 'startup_anchor_missed'
    )


@pytest.mark.parametrize(
    ('angle', 'expected'),
    [
        (0.0, 0.0),
        (2.0 * math.pi, 0.0),
        (1.5 * math.pi, -0.5 * math.pi),
    ],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


def test_circular_statistics_handle_yaw_wrap():
    angles = [math.radians(179.8), math.radians(-179.9), math.radians(180.0)]

    assert abs(abs(math.degrees(circular_mean(angles))) - 180.0) < 0.2
    assert math.degrees(circular_span(angles)) == pytest.approx(0.3)
