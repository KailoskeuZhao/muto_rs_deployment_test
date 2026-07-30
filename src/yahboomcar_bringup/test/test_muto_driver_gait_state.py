import json
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from muto_hexapod_interfaces_custom.msg import CommandedGaitState
from yahboomcar_bringup.muto_driver import yahboomcar_driver


class FakeStamp:
    def to_msg(self):
        return Time(sec=12, nanosec=34)


class FakeClock:
    def now(self):
        return FakeStamp()


class FakePublisher:
    def __init__(self):
        self.message = None

    def publish(self, message):
        self.message = message


class FakeDriver:
    def __init__(self):
        self.gait_state_frame_id = 'base_frame'
        self.gait_state_pub = FakePublisher()

    def get_clock(self):
        return FakeClock()


def test_driver_publishes_vendor_targets_in_ros_base_axes():
    state = SimpleNamespace(
        sequence=7,
        mode='move_x',
        phase_index=3,
        cycle_length=20,
        cycle_complete=False,
        commanded_stance=(True, False, True, False, True, False),
        foot_positions_mm=tuple(
            (10.0 + index, 20.0 + index, -30.0)
            for index in range(6)
        ),
    )
    driver = FakeDriver()

    yahboomcar_driver.publish_commanded_gait_state(driver, state)

    message = driver.gait_state_pub.message
    assert message.header.frame_id == 'base_frame'
    assert message.sequence == 7
    assert list(message.leg_state) == [
        CommandedGaitState.STANCE,
        CommandedGaitState.SWING,
        CommandedGaitState.STANCE,
        CommandedGaitState.SWING,
        CommandedGaitState.STANCE,
        CommandedGaitState.SWING,
    ]
    assert list(message.foot_x_mm) == [
        20.0, 21.0, 22.0, 23.0, 24.0, 25.0]
    assert list(message.foot_y_mm) == [
        -10.0, -11.0, -12.0, -13.0, -14.0, -15.0]
    assert list(message.foot_z_mm) == [-30.0] * 6


class FakeMuto:
    def __init__(self, state):
        self.commanded_gait_state = state

    @staticmethod
    def read_motor():
        return [0, -30, -15] * 6


def gait_state(mode='standby'):
    return SimpleNamespace(
        sequence=0,
        mode=mode,
        phase_index=0,
        cycle_length=1,
        cycle_complete=True,
        commanded_stance=(True,) * 6,
        foot_positions_mm=((100.0, 200.0, -90.0),) * 6,
    )


def test_standby_heartbeat_refreshes_latched_gait_state():
    driver = FakeDriver()
    driver.muto = FakeMuto(gait_state())

    yahboomcar_driver.publish_standby_gait_state_heartbeat(driver)

    assert driver.gait_state_pub.message is not None
    assert driver.gait_state_pub.message.header.stamp.sec == 12
    assert driver.gait_state_pub.message.mode == 'standby'


def test_gait_heartbeat_does_not_duplicate_moving_phases():
    driver = FakeDriver()
    driver.muto = FakeMuto(gait_state(mode='move_x'))

    yahboomcar_driver.publish_standby_gait_state_heartbeat(driver)

    assert driver.gait_state_pub.message is None


def test_motor_service_returns_synchronized_calibrated_gait_snapshot():
    state = SimpleNamespace(
        sequence=11,
        mode='move_x',
        phase_index=5,
        cycle_length=20,
        commanded_stance=(True,) * 6,
        foot_positions_mm=tuple(
            (10.0 + index, 20.0 + index, -30.0)
            for index in range(6)
        ),
    )
    driver = FakeDriver()
    driver.muto = FakeMuto(state)
    response = SimpleNamespace(success=False, message='')

    result = yahboomcar_driver.get_motor_angles_callback(
        driver, object(), response)
    payload = json.loads(result.message)

    assert result.success
    assert payload['angles'] == [0, -30, -15] * 6
    assert payload['angle_space'] == (
        'firmware_calibrated_logical_degrees')
    assert payload['standby_leg_angles_deg'] == [0.0, -30.0, -15.0]
    assert payload['sample_stamp'] == {'sec': 12, 'nanosec': 34}
    assert payload['gait_state']['frame_id'] == 'base_frame'
    assert payload['gait_state']['sequence'] == 11
    assert payload['gait_state']['leg_state'] == [
        CommandedGaitState.STANCE] * 6
    assert payload['gait_state']['foot_x_mm'] == [
        20.0, 21.0, 22.0, 23.0, 24.0, 25.0]
    assert payload['gait_state']['foot_y_mm'] == [
        -10.0, -11.0, -12.0, -13.0, -14.0, -15.0]
