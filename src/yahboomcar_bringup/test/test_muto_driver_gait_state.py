from collections import deque
import json
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from muto_hexapod_interfaces_custom.msg import CommandedGaitState
from muto_hexapod_lib_custom.movement.velocity_calibration import (
    VelocityCalibrationMapper,
    VelocityCalibrationProfile,
)
from yahboomcar_bringup import muto_driver as muto_driver_module
from yahboomcar_bringup.muto_driver import (
    ATTITUDE_TELEMETRY,
    RAW_TELEMETRY,
    TelemetryScheduler,
    yahboomcar_driver,
)


class FakeStamp:
    nanoseconds = 12_000_000_034

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
        self.last_gait_command_monotonic = 0.0
        self.gait_phase_monotonic_times = deque(maxlen=100)
        self.observed_phase_rate_hz = 0.0
        self.motors_released = False

    def get_clock(self):
        return FakeClock()

    def publish_commanded_gait_state(self, state):
        yahboomcar_driver.publish_commanded_gait_state(self, state)

    def publish_motion_command_state(self, selection, active_state=None):
        yahboomcar_driver.publish_motion_command_state(
            self, selection, active_state=active_state)

    def _standby_selection(self, *args, **kwargs):
        return yahboomcar_driver._standby_selection(self, *args, **kwargs)


def make_test_velocity_mapper():
    profile = VelocityCalibrationProfile.from_mapping({
        'schema_version': 1,
        'profile_id': 'driver-test-v1',
        'provenance': 'unit test',
        'reference_phase_rate_hz': 50.0,
        'x': {
            'positive': {5: 0.05, 30: 0.30},
            'negative': {5: 0.05, 30: 0.30},
        },
        'y': {
            'positive': {5: 0.05, 30: 0.30},
            'negative': {5: 0.05, 30: 0.30},
        },
        'yaw': {
            'positive': {10: 0.10, 20: 0.20},
            'negative': {10: 0.10, 20: 0.20},
        },
    })
    return VelocityCalibrationMapper(profile)


def test_driver_publishes_vendor_targets_in_ros_base_axes():
    state = SimpleNamespace(
        sequence=7,
        mode='move_x',
        x_level=10,
        y_level=0,
        z_level=0,
        replacement_pending=True,
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


def test_gait_phase_republishes_selected_and_active_levels_separately():
    state = SimpleNamespace(
        sequence=8,
        mode='move_xz',
        x_level=5,
        y_level=0,
        z_level=10,
        replacement_pending=True,
        phase_index=4,
        cycle_length=20,
        cycle_complete=False,
        commanded_stance=(True, False, True, False, True, False),
        foot_positions_mm=((100.0, 200.0, -90.0),) * 6,
    )
    driver = FakeDriver()
    driver.motion_command_state_pub = FakePublisher()
    driver.locomotion_update_rate_hz = 50.0
    driver.velocity_mapper = make_test_velocity_mapper()
    driver.last_velocity_selection = driver.velocity_mapper.select(
        0.10, 0.0, 0.15)

    driver.publish_commanded_gait_state(state)

    status = driver.motion_command_state_pub.message
    assert (status.x_level, status.y_level, status.z_level) == (10, 0, 15)
    assert (
        status.active_x_level,
        status.active_y_level,
        status.active_z_level,
    ) == (5, 0, 10)
    assert status.active_mode == 'move_xz'
    assert status.replacement_pending


class FakeMuto:
    def __init__(self, state):
        self.commanded_gait_state = state
        self.commands = []
        self.tick_count = 0

    def set_motion_command(self, *levels):
        self.commands.append(levels)

    def tick_motion(self):
        self.tick_count += 1

    @staticmethod
    def read_motor():
        return [0, -30, -15] * 6

    def read_motor_with_gait_state(self):
        return self.commanded_gait_state, self.read_motor()


def gait_state(mode='standby'):
    return SimpleNamespace(
        sequence=0,
        mode=mode,
        x_level=0,
        y_level=0,
        z_level=0,
        replacement_pending=False,
        phase_index=0,
        cycle_length=1,
        cycle_complete=True,
        commanded_stance=(True,) * 6,
        foot_positions_mm=((100.0, 200.0, -90.0),) * 6,
    )


def timer_driver(last_cmd_vel_monotonic):
    driver = FakeDriver()
    driver.muto = FakeMuto(gait_state())
    driver.desired_motion_levels = (10.0, 0.0, 0.0)
    driver.last_cmd_vel_monotonic = last_cmd_vel_monotonic
    driver.cmd_vel_timeout = 0.5
    driver.locomotion_update_rate_hz = 50.0
    driver.last_locomotion_tick_monotonic = None
    driver.cmd_vel_timed_out = False
    driver.locomotion_command_mapping = muto_driver_module.CALIBRATED_MAPPING
    driver.velocity_mapper = make_test_velocity_mapper()
    driver.motion_command_state_pub = FakePublisher()
    driver.warnings = []
    driver.get_logger = lambda: SimpleNamespace(
        warn=lambda message, **_kwargs: driver.warnings.append(message))
    driver.service_imu_telemetry = lambda: False
    driver.maybe_publish_imu_telemetry_status = lambda: False
    return driver


def test_locomotion_timer_advances_one_phase_from_fresh_command(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=9.8)

    yahboomcar_driver.update_locomotion(driver)

    assert driver.muto.commands == [(10.0, 0.0, 0.0)]
    assert driver.muto.tick_count == 1
    assert not driver.cmd_vel_timed_out


def test_locomotion_timer_returns_stale_command_to_standby(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=9.0)

    yahboomcar_driver.update_locomotion(driver)

    assert driver.muto.commands == [(0.0, 0.0, 0.0)]
    assert driver.muto.tick_count == 1
    assert driver.cmd_vel_timed_out
    assert driver.warnings == [
        'cmd_vel timed out; returning locomotion to standby']


def test_locomotion_timer_does_not_recommand_released_motors(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=9.8)
    driver.motors_released = True
    calls = []
    driver.service_imu_telemetry = lambda: calls.append('telemetry')
    driver.maybe_publish_imu_telemetry_status = (
        lambda: calls.append('status'))

    yahboomcar_driver.update_locomotion(driver)

    assert driver.muto.commands == []
    assert driver.muto.tick_count == 0
    assert driver.last_locomotion_tick_monotonic == 10.0
    assert calls == ['telemetry', 'status']


def test_locomotion_timer_reports_a_missed_tick_deadline(monkeypatch):
    monotonic_values = iter((10.0, 10.0, 10.005))
    monkeypatch.setattr(
        muto_driver_module.time, 'monotonic',
        lambda: next(monotonic_values),
    )
    driver = timer_driver(last_cmd_vel_monotonic=9.8)
    driver.last_locomotion_tick_monotonic = 9.95
    warnings = []
    driver.get_logger = lambda: SimpleNamespace(
        warn=lambda message, **_kwargs: warnings.append(message))

    yahboomcar_driver.update_locomotion(driver)

    assert len(warnings) == 1
    assert warnings[0].startswith('Locomotion tick delayed:')


class FakeImuPublisher:
    def __init__(self):
        self.response_timeout_sec = 0.008
        self.skipped = 0
        self.attitude_skipped = 0
        self.timeouts = []
        self.attitude_timeouts = []
        self.raw_result = True
        self.attitude_result = True
        self.poll_count = 0
        self.successful_read_count = 0
        self.failed_read_count = 0
        self.changed_snapshot_count = 0
        self.duplicate_sample_count = 0
        self.skipped_for_locomotion_count = 0
        self.attitude_poll_count = 0
        self.successful_attitude_read_count = 0
        self.failed_attitude_read_count = 0
        self.attitude_skipped_for_locomotion_count = 0

    def note_poll_skipped_for_locomotion(self):
        self.skipped += 1
        self.skipped_for_locomotion_count += 1

    def publish_imu_data(self, response_timeout_sec=None):
        self.timeouts.append(response_timeout_sec)
        self.poll_count += 1
        if self.raw_result:
            self.successful_read_count += 1
            self.changed_snapshot_count += 1
        else:
            self.failed_read_count += 1
        return self.raw_result

    def note_attitude_poll_skipped_for_locomotion(self):
        self.attitude_skipped += 1
        self.attitude_skipped_for_locomotion_count += 1

    def publish_controller_attitude(self, response_timeout_sec=None):
        self.attitude_timeouts.append(response_timeout_sec)
        self.attitude_poll_count += 1
        if self.attitude_result:
            self.successful_attitude_read_count += 1
        else:
            self.failed_attitude_read_count += 1
        return self.attitude_result


def telemetry_driver(start_monotonic=0.0, raw_rate_hz=10.0,
                     attitude_rate_hz=10.0):
    driver = SimpleNamespace(
        imu=FakeImuPublisher(),
        last_locomotion_tick_monotonic=-0.010,
        locomotion_update_rate_hz=50.0,
        imu_locomotion_guard_sec=0.003,
        telemetry_scheduler=TelemetryScheduler(
            raw_rate_hz,
            attitude_rate_hz,
            start_monotonic,
        ),
        telemetry_control_slot_count=0,
        telemetry_idle_slot_count=0,
        raw_telemetry_selected_count=0,
        attitude_telemetry_selected_count=0,
        raw_telemetry_deferred_count=0,
        attitude_telemetry_deferred_count=0,
        last_selected_telemetry_endpoint='',
    )
    driver._poll_imu_outcome = lambda: (
        yahboomcar_driver._poll_imu_outcome(driver))
    driver._poll_controller_attitude_outcome = lambda: (
        yahboomcar_driver._poll_controller_attitude_outcome(driver))
    return driver


def service_telemetry(driver, now_monotonic):
    driver.last_locomotion_tick_monotonic = now_monotonic - 0.010
    return yahboomcar_driver.service_imu_telemetry(
        driver,
        now_monotonic=now_monotonic,
    )


def test_control_slot_dispatches_gait_before_telemetry(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=9.8)
    order = []
    driver.muto.tick_motion = lambda: order.append('gait')
    driver.service_imu_telemetry = lambda: order.append('telemetry')

    yahboomcar_driver.update_locomotion(driver)

    assert order == ['gait', 'telemetry']


def test_scheduler_produces_ten_hz_attitude_and_raw(monkeypatch):
    current = [0.0]
    monkeypatch.setattr(
        muto_driver_module.time,
        'monotonic',
        lambda: current[0],
    )
    driver = telemetry_driver()

    attitude_times = []
    raw_times = []
    for slot in range(50):
        current[0] = round(slot * 0.020, 9)
        before_attitude = len(driver.imu.attitude_timeouts)
        before_raw = len(driver.imu.timeouts)
        service_telemetry(driver, current[0])
        attitude_delta = (
            len(driver.imu.attitude_timeouts) - before_attitude)
        raw_delta = len(driver.imu.timeouts) - before_raw
        assert attitude_delta + raw_delta <= 1
        if attitude_delta:
            attitude_times.append(current[0])
        if raw_delta:
            raw_times.append(current[0])

    assert len(driver.imu.attitude_timeouts) == 10
    assert len(driver.imu.timeouts) == 10
    assert driver.attitude_telemetry_selected_count == 10
    assert driver.raw_telemetry_selected_count == 10
    assert driver.telemetry_control_slot_count == 50
    assert all(
        0.080 <= later - earlier <= 0.120
        for earlier, later in zip(attitude_times, attitude_times[1:])
    )
    assert all(
        0.080 <= later - earlier <= 0.120
        for earlier, later in zip(raw_times, raw_times[1:])
    )


def test_simultaneous_due_attitude_wins_then_raw_runs_next_slot(monkeypatch):
    current = [1.0]
    monkeypatch.setattr(
        muto_driver_module.time,
        'monotonic',
        lambda: current[0],
    )
    driver = telemetry_driver(start_monotonic=1.0)
    driver.telemetry_scheduler.next_raw_monotonic = 1.0
    driver.telemetry_scheduler.next_attitude_monotonic = 1.0

    service_telemetry(driver, 1.0)

    assert len(driver.imu.attitude_timeouts) == 1
    assert driver.imu.timeouts == []
    assert driver.last_selected_telemetry_endpoint == ATTITUDE_TELEMETRY
    assert driver.raw_telemetry_deferred_count == 1
    assert driver.telemetry_scheduler.next_raw_monotonic == 1.0

    current[0] = 1.020
    service_telemetry(driver, 1.020)

    assert len(driver.imu.attitude_timeouts) == 1
    assert len(driver.imu.timeouts) == 1


def test_attitude_priority_over_older_overdue_raw(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 1.0)
    driver = telemetry_driver(start_monotonic=1.0)
    driver.telemetry_scheduler.next_raw_monotonic = 0.8
    driver.telemetry_scheduler.next_attitude_monotonic = 1.0

    service_telemetry(driver, 1.0)

    assert len(driver.imu.attitude_timeouts) == 1
    assert driver.imu.timeouts == []
    assert driver.telemetry_scheduler.next_raw_monotonic == 0.8


def test_failed_attitude_does_not_fall_through_to_raw(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 1.0)
    driver = telemetry_driver(start_monotonic=1.0)
    driver.telemetry_scheduler.next_raw_monotonic = 1.0
    driver.telemetry_scheduler.next_attitude_monotonic = 1.0
    driver.imu.attitude_result = False

    assert service_telemetry(driver, 1.0) is False
    assert len(driver.imu.attitude_timeouts) == 1
    assert driver.imu.timeouts == []
    assert driver.telemetry_scheduler.next_attitude_monotonic == 1.1


def test_deadline_skip_retries_selected_endpoint_next_slot(monkeypatch):
    current = [1.018]
    monkeypatch.setattr(
        muto_driver_module.time,
        'monotonic',
        lambda: current[0],
    )
    driver = telemetry_driver(start_monotonic=1.0)
    driver.telemetry_scheduler.next_raw_monotonic = 2.0
    driver.last_locomotion_tick_monotonic = 1.0

    assert yahboomcar_driver.service_imu_telemetry(
        driver, now_monotonic=1.018) is False
    assert driver.imu.attitude_skipped == 1
    assert driver.imu.attitude_timeouts == []
    assert driver.telemetry_scheduler.next_attitude_monotonic == 1.0

    current[0] = 1.030
    driver.last_locomotion_tick_monotonic = 1.020
    assert yahboomcar_driver.service_imu_telemetry(
        driver, now_monotonic=1.030) is True
    assert len(driver.imu.attitude_timeouts) == 1
    assert driver.telemetry_scheduler.next_attitude_monotonic == 1.1


def test_late_callback_advances_deadline_without_catchup(monkeypatch):
    current = [0.45]
    monkeypatch.setattr(
        muto_driver_module.time,
        'monotonic',
        lambda: current[0],
    )
    driver = telemetry_driver(start_monotonic=0.1)
    driver.telemetry_scheduler.next_raw_monotonic = 1.0

    service_telemetry(driver, 0.45)

    assert len(driver.imu.attitude_timeouts) == 1
    assert abs(
        driver.telemetry_scheduler.next_attitude_monotonic - 0.5
    ) < 1e-9

    current[0] = 0.46
    service_telemetry(driver, 0.46)
    assert len(driver.imu.attitude_timeouts) == 1


def test_disabled_attitude_services_only_raw(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 0.0)
    driver = telemetry_driver(attitude_rate_hz=0.0)

    service_telemetry(driver, 0.0)

    assert driver.imu.attitude_timeouts == []
    assert len(driver.imu.timeouts) == 1
    assert driver.last_selected_telemetry_endpoint == RAW_TELEMETRY


def test_telemetry_status_exposes_scheduler_and_read_counters():
    driver = telemetry_driver()
    driver.muto = FakeMuto(gait_state('move_x'))
    driver.imu_poll_rate_hz = 10.0
    driver.imu_attitude_poll_rate_hz = 10.0
    driver.imu_telemetry_status_pub = FakePublisher()
    driver.telemetry_status_publish_count = 0
    driver.next_telemetry_status_monotonic = 0.0
    driver.telemetry_control_slot_count = 50
    driver.telemetry_idle_slot_count = 7
    driver.raw_telemetry_selected_count = 12
    driver.raw_telemetry_deferred_count = 2
    driver.attitude_telemetry_selected_count = 14
    driver.attitude_telemetry_deferred_count = 3
    driver.last_selected_telemetry_endpoint = ATTITUDE_TELEMETRY
    driver.imu.poll_count = 10
    driver.imu.successful_read_count = 9
    driver.imu.failed_read_count = 1
    driver.imu.changed_snapshot_count = 2
    driver.imu.duplicate_sample_count = 7
    driver.imu.skipped_for_locomotion_count = 2
    driver.imu.attitude_poll_count = 11
    driver.imu.successful_attitude_read_count = 10
    driver.imu.failed_attitude_read_count = 1
    driver.imu.attitude_skipped_for_locomotion_count = 3

    assert yahboomcar_driver.maybe_publish_imu_telemetry_status(
        driver,
        now_monotonic=0.0,
    ) is True

    payload = json.loads(driver.imu_telemetry_status_pub.message.data)
    assert payload['schema_version'] == 1
    assert payload['scheduler_policy'] == 'gait_then_one_telemetry'
    assert payload['active_mode'] == 'move_x'
    assert payload['locomotion_update_rate_hz'] == 50.0
    assert payload['response_timeout_sec'] == 0.008
    assert payload['locomotion_guard_sec'] == 0.003
    assert payload['raw_imu']['configured_poll_rate_hz'] == 10.0
    assert payload['raw_imu'] == {
        'attempted': 10,
        'changed_snapshots': 2,
        'configured_poll_rate_hz': 10.0,
        'deadline_skips': 2,
        'deferred': 2,
        'duplicate_snapshots': 7,
        'failed_reads': 1,
        'selected': 12,
        'successful_reads': 9,
    }
    assert payload['controller_attitude']['configured_poll_rate_hz'] == 10.0
    assert payload['controller_attitude'] == {
        'attempted': 11,
        'configured_poll_rate_hz': 10.0,
        'deadline_skips': 3,
        'deferred': 3,
        'failed_reads': 1,
        'selected': 14,
        'successful_reads': 10,
    }
    assert driver.next_telemetry_status_monotonic == 1.0


def test_imu_poll_is_skipped_when_gait_deadline_is_too_close(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.018)
    driver = telemetry_driver()
    driver.last_locomotion_tick_monotonic = 10.0

    assert yahboomcar_driver.poll_imu(driver) is False
    assert driver.imu.skipped == 1
    assert driver.imu.timeouts == []


def test_imu_poll_uses_remaining_gait_budget(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.010)
    driver = telemetry_driver()
    driver.last_locomotion_tick_monotonic = 10.0

    assert yahboomcar_driver.poll_imu(driver) is True
    assert driver.imu.skipped == 0
    assert len(driver.imu.timeouts) == 1
    assert abs(driver.imu.timeouts[0] - 0.007) < 1e-9


def test_attitude_poll_is_skipped_when_gait_deadline_is_too_close(
        monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.018)
    driver = telemetry_driver()
    driver.last_locomotion_tick_monotonic = 10.0

    assert yahboomcar_driver.poll_controller_attitude(driver) is False
    assert driver.imu.attitude_skipped == 1
    assert driver.imu.attitude_timeouts == []


def test_attitude_poll_uses_remaining_gait_budget(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.010)
    driver = telemetry_driver()
    driver.last_locomotion_tick_monotonic = 10.0

    assert yahboomcar_driver.poll_controller_attitude(driver) is True
    assert driver.imu.attitude_skipped == 0
    assert len(driver.imu.attitude_timeouts) == 1
    assert abs(driver.imu.attitude_timeouts[0] - 0.007) < 1e-9


def test_cmd_vel_callback_only_updates_desired_levels(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=None)
    message = Twist()
    message.linear.x = 0.1
    message.angular.z = 0.05

    yahboomcar_driver.cmd_vel_callback(driver, message)

    assert driver.desired_motion_levels == (10, 0, 0)
    assert driver.last_cmd_vel_monotonic == 10.0
    assert driver.muto.commands == []
    assert driver.muto.tick_count == 0
    status = driver.motion_command_state_pub.message
    assert status.requested_twist.angular.z == 0.05
    assert status.predicted_twist.linear.x == 0.1
    assert status.x_level == 10
    assert status.z_level == 0
    assert status.active_x_level == 0
    assert status.active_mode == 'standby'
    assert not status.replacement_pending
    assert status.calibration_profile == 'driver-test-v1'
    assert status.quantized
    assert status.yaw_projected_to_zero
    assert not status.saturated
    assert status.calibration_phase_rate_hz == 50.0
    assert status.configured_phase_rate_hz == 50.0


def test_cmd_vel_callback_matches_vendor_angular_level_limit(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=None)
    message = Twist()
    message.angular.z = 0.3

    yahboomcar_driver.cmd_vel_callback(driver, message)

    assert driver.desired_motion_levels == (0.0, 0.0, 20)


def test_cmd_vel_callback_rejects_nonfinite_input_and_stops(monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=9.8)
    driver.desired_motion_levels = (20, 0, 0)
    message = Twist()
    message.linear.x = math.nan

    yahboomcar_driver.cmd_vel_callback(driver, message)

    assert driver.desired_motion_levels == (0, 0, 0)
    assert driver.motion_command_state_pub.message.unsupported
    assert 'invalid cmd_vel' in driver.motion_command_state_pub.message.detail


def test_legacy_mapping_is_explicit_and_reproduces_old_level_clamps(
        monkeypatch):
    monkeypatch.setattr(muto_driver_module.time, 'monotonic', lambda: 10.0)
    driver = timer_driver(last_cmd_vel_monotonic=None)
    driver.locomotion_command_mapping = muto_driver_module.LEGACY_MAPPING
    driver.velocity_mapper = None
    message = Twist()
    message.linear.x = 0.146
    message.angular.z = 0.05

    yahboomcar_driver.cmd_vel_callback(driver, message)

    assert driver.desired_motion_levels == (14, 0, 10)
    assert driver.motion_command_state_pub.message.calibration_profile == (
        'legacy_100')


def test_legacy_mapping_retains_minimum_nonzero_yaw_level():
    selection = muto_driver_module.legacy_velocity_selection(
        0.0, 0.0, 0.001, 50.0)

    assert selection.levels.z_level == 10


def test_motor_release_publishes_standby_command_state():
    driver = timer_driver(last_cmd_vel_monotonic=9.8)
    torque_off_ids = []
    driver.muto.Servo_torque_off = torque_off_ids.append
    response = SimpleNamespace(success=False, message='')

    result = yahboomcar_driver.release_motors_callback(
        driver, object(), response)

    assert result.success
    assert torque_off_ids == list(range(1, 19))
    assert driver.desired_motion_levels == (0, 0, 0)
    assert driver.motion_command_state_pub.message.mode == 'standby'
    assert driver.motion_command_state_pub.message.detail == (
        'motor torque released')


def test_motor_service_returns_synchronized_calibrated_gait_snapshot(
        monkeypatch):
    state = SimpleNamespace(
        sequence=11,
        mode='move_x',
        x_level=10,
        y_level=0,
        z_level=0,
        replacement_pending=False,
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
    monotonic_values = iter((10.0, 10.004, 10.005))
    monkeypatch.setattr(
        muto_driver_module.time, 'monotonic',
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(
        muto_driver_module.time, 'sleep',
        lambda _seconds: (_ for _ in ()).throw(
            AssertionError('motor validation must not block gait timers')
        ),
    )

    result = yahboomcar_driver.get_motor_angles_callback(
        driver, object(), response)
    payload = json.loads(result.message)

    assert result.success
    assert payload['angles'] == [0, -30, -15] * 6
    assert payload['angle_space'] == (
        'firmware_calibrated_logical_degrees')
    assert payload['standby_leg_angles_deg'] == [0.0, -30.0, -15.0]
    assert abs(payload['command_age_sec'] - 10.005) < 1e-9
    assert abs(payload['read_duration_sec'] - 0.004) < 1e-9
    assert payload['sample_stamp'] == {'sec': 12, 'nanosec': 34}
    assert payload['gait_state']['frame_id'] == 'base_frame'
    assert payload['gait_state']['sequence'] == 11
    assert payload['gait_state']['x_level'] == 10
    assert not payload['gait_state']['replacement_pending']
    assert payload['gait_state']['leg_state'] == [
        CommandedGaitState.STANCE] * 6
    assert payload['gait_state']['foot_x_mm'] == [
        20.0, 21.0, 22.0, 23.0, 24.0, 25.0]
    assert payload['gait_state']['foot_y_mm'] == [
        -10.0, -11.0, -12.0, -13.0, -14.0, -15.0]


def test_motor_service_rejects_validation_before_first_gait_tick():
    driver = FakeDriver()
    driver.muto = FakeMuto(gait_state())
    driver.last_gait_command_monotonic = None
    response = SimpleNamespace(success=False, message='')

    result = yahboomcar_driver.get_motor_angles_callback(
        driver, object(), response)

    assert not result.success
    assert json.loads(result.message)['error'] == (
        'no_commanded_gait_sample')
