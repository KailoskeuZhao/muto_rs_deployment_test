#!/usr/bin/env python3
"""ROS hardware driver for the Muto base, IMU, and locomotion state."""

from collections import deque
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from muto_hexapod_interfaces_custom.msg import (
    CommandedGaitState,
    MotionCommandState,
)
from muto_hexapod_lib_custom.core.config import STANDBY_SERVO_ANGLES_DEG
from muto_hexapod_lib_custom.core.MutoLibCore import Muto
from muto_hexapod_lib_custom.movement.velocity_calibration import (
    MotionLevels,
    PlanarVelocity,
    SaturationFlags,
    VelocityCalibrationMapper,
    VelocityCalibrationProfile,
    VelocitySelection,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from yahboomcar_imu.imu_node import ImuPublisher
import yaml


CALIBRATED_MAPPING = 'calibrated'
LEGACY_MAPPING = 'legacy_100'
DEFAULT_IMU_POLL_RATE_HZ = 10.0
DEFAULT_IMU_ATTITUDE_POLL_RATE_HZ = 10.0
DEFAULT_IMU_LOCOMOTION_GUARD_SEC = 0.003
MIN_IMU_TRANSACTION_BUDGET_SEC = 0.004
IMU_TELEMETRY_STATUS_TOPIC = '/muto/imu_telemetry_status'
IMU_TELEMETRY_STATUS_PERIOD_SEC = 1.0
RAW_TELEMETRY = 'raw_imu'
ATTITUDE_TELEMETRY = 'controller_attitude'


def _default_calibration_file():
    return os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'config',
        'muto_locomotion_provisional_20260806.yaml',
    )


def _available_telemetry_response_timeout(driver):
    """Return a serial-read budget that preserves the next gait deadline."""
    response_timeout = driver.imu.response_timeout_sec
    if driver.last_locomotion_tick_monotonic is None:
        return response_timeout

    now_monotonic = time.monotonic()
    next_locomotion_deadline = (
        driver.last_locomotion_tick_monotonic
        + 1.0 / driver.locomotion_update_rate_hz
    )
    available = (
        next_locomotion_deadline
        - now_monotonic
        - driver.imu_locomotion_guard_sec
    )
    if available < MIN_IMU_TRANSACTION_BUDGET_SEC:
        return None
    return min(response_timeout, available)


def _advance_periodic_deadline(deadline, period, now_monotonic):
    """Advance a completed periodic job without issuing catch-up bursts."""
    next_deadline = deadline + period
    if next_deadline <= now_monotonic:
        missed_periods = (
            math.floor((now_monotonic - next_deadline) / period) + 1
        )
        next_deadline += missed_periods * period
    return next_deadline


class TelemetryScheduler:
    """Choose at most one serial telemetry transaction per gait slot."""

    def __init__(self, raw_rate_hz, attitude_rate_hz, start_monotonic):
        self.raw_period_sec = 1.0 / raw_rate_hz
        self.attitude_period_sec = (
            None if attitude_rate_hz <= 0.0 else 1.0 / attitude_rate_hz
        )
        self.next_attitude_monotonic = (
            None if self.attitude_period_sec is None else start_monotonic
        )
        # Startup calibration has just read the raw endpoint. Phase the first
        # runtime raw poll between attitude polls instead of recreating two
        # aligned timers that compete for the same post-gait serial budget.
        raw_phase_offset = (
            0.0
            if self.attitude_period_sec is None
            else 0.5 * min(
                self.raw_period_sec,
                self.attitude_period_sec,
            )
        )
        self.next_raw_monotonic = start_monotonic + raw_phase_offset

    def due_endpoints(self, now_monotonic):
        raw_due = now_monotonic >= self.next_raw_monotonic
        attitude_due = (
            self.next_attitude_monotonic is not None
            and now_monotonic >= self.next_attitude_monotonic
        )
        return raw_due, attitude_due

    def select(self, now_monotonic):
        """Prioritize due attitude; raw remains due for the following slot."""
        raw_due, attitude_due = self.due_endpoints(now_monotonic)
        if not raw_due and not attitude_due:
            return None
        if attitude_due:
            return ATTITUDE_TELEMETRY
        return RAW_TELEMETRY

    def mark_attempted(self, endpoint, now_monotonic):
        """Advance only after serial I/O began; guarded skips retry next slot."""
        if endpoint == ATTITUDE_TELEMETRY:
            self.next_attitude_monotonic = _advance_periodic_deadline(
                self.next_attitude_monotonic,
                self.attitude_period_sec,
                now_monotonic,
            )
            return
        if endpoint == RAW_TELEMETRY:
            self.next_raw_monotonic = _advance_periodic_deadline(
                self.next_raw_monotonic,
                self.raw_period_sec,
                now_monotonic,
            )
            return
        raise ValueError(f'unknown telemetry endpoint: {endpoint}')


def load_velocity_calibration(path, phase_rate_hz):
    """Load and validate one explicit physical velocity profile."""
    if not path:
        raise ValueError('locomotion_calibration_file must not be empty')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    profile = VelocityCalibrationProfile.from_mapping(data)
    return VelocityCalibrationMapper(profile, phase_rate_hz=phase_rate_hz)


def legacy_velocity_selection(vx, vy, wz, phase_rate_hz):
    """Reproduce the former hidden ``cmd_vel * 100`` mapping for rollback."""
    requested = PlanarVelocity(vx, vy, wz)
    x_level = int(max(-30.0, min(30.0, vx * 100.0)))
    y_level = int(max(-30.0, min(30.0, vy * 100.0)))
    raw_z_level = max(-20.0, min(20.0, wz * 100.0))
    if 0.0 < abs(raw_z_level) < 10.0:
        z_level = 10 if raw_z_level > 0.0 else -10
    else:
        z_level = int(raw_z_level)
    levels = MotionLevels(x_level, y_level, z_level)
    unsupported = bool(y_level and (x_level or z_level))
    if y_level:
        mode = 'move_y' if not unsupported else 'standby'
    elif x_level and z_level:
        mode = 'move_xz'
    elif x_level:
        mode = 'move_x'
    elif z_level:
        mode = 'turn_z'
    else:
        mode = 'standby'
    if unsupported:
        levels = MotionLevels()
    detail = (
        'legacy_100 rollback mapping; simultaneous lateral motion is '
        'unsupported'
        if unsupported else
        'legacy_100 rollback mapping; predicted twist is not calibrated'
    )
    return VelocitySelection(
        requested=requested,
        predicted=(
            PlanarVelocity(0.0, 0.0, 0.0)
            if unsupported else
            PlanarVelocity(
                levels.x_level / 100.0,
                levels.y_level / 100.0,
                levels.z_level / 100.0,
            )
        ),
        levels=levels,
        mode=mode,
        saturation=SaturationFlags(
            x=abs(vx) > 0.30,
            y=abs(vy) > 0.30 or unsupported,
            yaw=abs(wz) > 0.20,
            unsupported_combination=unsupported,
        ),
        profile_id=LEGACY_MAPPING,
        phase_rate_hz=phase_rate_hz,
        detail=detail,
        supported=not unsupported,
        unsupported_reason=(detail if unsupported else None),
    )


class yahboomcar_driver(Node):
    """Own the shared Muto serial bus and run a fixed-rate gait loop."""

    def __init__(self, name):
        super().__init__(name)

        self.sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.sub_buzzer = self.create_subscription(
            Bool, 'Buzzer', self.Buzzercallback, 1)
        self.srv_motor_angles = self.create_service(
            Trigger, 'get_motor_angles', self.get_motor_angles_callback)
        self.srv_release_motors = self.create_service(
            Trigger, 'release_motors', self.release_motors_callback)

        self.gait_state_topic = self.declare_parameter(
            'gait_state_topic', '/muto/commanded_gait_state').value
        self.gait_state_frame_id = self.declare_parameter(
            'gait_state_frame_id', 'base_frame').value
        self.locomotion_update_rate_hz = float(self.declare_parameter(
            'locomotion_update_rate_hz', 50.0).value)
        if (not math.isfinite(self.locomotion_update_rate_hz)
                or self.locomotion_update_rate_hz <= 0.0):
            self.get_logger().warn(
                'locomotion_update_rate_hz must be positive; using 50.0')
            self.locomotion_update_rate_hz = 50.0
        self.cmd_vel_timeout = float(self.declare_parameter(
            'cmd_vel_timeout', 0.5).value)
        if not math.isfinite(self.cmd_vel_timeout) or self.cmd_vel_timeout < 0.0:
            self.get_logger().warn(
                'cmd_vel_timeout must be finite and non-negative; using 0.5')
            self.cmd_vel_timeout = 0.5
        self.locomotion_command_mapping = str(self.declare_parameter(
            'locomotion_command_mapping', CALIBRATED_MAPPING).value)
        default_calibration_file = _default_calibration_file()
        self.locomotion_calibration_file = str(self.declare_parameter(
            'locomotion_calibration_file', default_calibration_file).value)
        if self.locomotion_command_mapping == CALIBRATED_MAPPING:
            self.velocity_mapper = load_velocity_calibration(
                self.locomotion_calibration_file,
                self.locomotion_update_rate_hz,
            )
            calibration_description = (
                self.velocity_mapper.profile.profile_id
                + ' from ' + self.locomotion_calibration_file
            )
        elif self.locomotion_command_mapping == LEGACY_MAPPING:
            self.velocity_mapper = None
            calibration_description = LEGACY_MAPPING + ' rollback'
        else:
            raise ValueError(
                'locomotion_command_mapping must be calibrated or '
                'legacy_100')

        gait_state_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gait_state_pub = self.create_publisher(
            CommandedGaitState,
            self.gait_state_topic,
            gait_state_qos,
        )
        self.motion_command_state_topic = self.declare_parameter(
            'motion_command_state_topic',
            '/muto/motion_command_state',
        ).value
        self.motion_command_state_pub = self.create_publisher(
            MotionCommandState,
            self.motion_command_state_topic,
            gait_state_qos,
        )
        telemetry_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.imu_telemetry_status_pub = self.create_publisher(
            String,
            IMU_TELEMETRY_STATUS_TOPIC,
            telemetry_status_qos,
        )

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.angular_z = 0.0
        self.desired_motion_levels = (0, 0, 0)
        self.last_velocity_selection = None
        self.last_cmd_vel_monotonic = None
        self.last_gait_command_monotonic = None
        self.last_locomotion_tick_monotonic = None
        self.gait_phase_monotonic_times = deque(maxlen=100)
        self.observed_phase_rate_hz = 0.0
        self.cmd_vel_timed_out = False
        self.motors_released = False

        self.batch_gait_phase_writes = bool(self.declare_parameter(
            'batch_gait_phase_writes', True).value)
        self.muto = Muto(
            gait_step_callback=self.publish_commanded_gait_state,
            batch_gait_phase_writes=self.batch_gait_phase_writes,
        )
        self.muto.set_motion_command(0.0, 0.0, 0.0)
        self.last_velocity_selection = self._standby_selection(
            'startup standby')

        self.declare_parameter('imu_link', 'imu_link')
        imu_link = self.get_parameter(
            'imu_link').get_parameter_value().string_value
        self.declare_parameter(
            'imu_publish_rate_hz', DEFAULT_IMU_POLL_RATE_HZ)
        imu_publish_rate_hz = self.get_parameter(
            'imu_publish_rate_hz').get_parameter_value().double_value
        if (not math.isfinite(imu_publish_rate_hz)
                or imu_publish_rate_hz <= 0.0):
            self.get_logger().warn(
                'imu_publish_rate_hz must be positive; using '
                f'{DEFAULT_IMU_POLL_RATE_HZ:.1f}')
            imu_publish_rate_hz = DEFAULT_IMU_POLL_RATE_HZ
        self.imu_poll_rate_hz = imu_publish_rate_hz
        self.declare_parameter(
            'imu_attitude_publish_rate_hz',
            DEFAULT_IMU_ATTITUDE_POLL_RATE_HZ,
        )
        imu_attitude_publish_rate_hz = self.get_parameter(
            'imu_attitude_publish_rate_hz').get_parameter_value().double_value
        if (not math.isfinite(imu_attitude_publish_rate_hz)
                or imu_attitude_publish_rate_hz < 0.0):
            self.get_logger().warn(
                'imu_attitude_publish_rate_hz must be finite and '
                'non-negative; using '
                f'{DEFAULT_IMU_ATTITUDE_POLL_RATE_HZ:.1f}')
            imu_attitude_publish_rate_hz = DEFAULT_IMU_ATTITUDE_POLL_RATE_HZ
        self.imu_attitude_poll_rate_hz = imu_attitude_publish_rate_hz
        if (
            self.imu_poll_rate_hz + self.imu_attitude_poll_rate_hz
            > self.locomotion_update_rate_hz
        ):
            self.get_logger().warn(
                'Combined IMU telemetry poll rates exceed the locomotion '
                'slot rate; requested rates cannot both be guaranteed')
        self.imu_locomotion_guard_sec = float(self.declare_parameter(
            'imu_locomotion_guard_sec',
            DEFAULT_IMU_LOCOMOTION_GUARD_SEC,
        ).value)
        if (not math.isfinite(self.imu_locomotion_guard_sec)
                or self.imu_locomotion_guard_sec < 0.0):
            self.get_logger().warn(
                'imu_locomotion_guard_sec must be finite and non-negative; '
                f'using {DEFAULT_IMU_LOCOMOTION_GUARD_SEC:.3f}')
            self.imu_locomotion_guard_sec = (
                DEFAULT_IMU_LOCOMOTION_GUARD_SEC)

        # IMU calibration assumes a still robot. Dispatch a physical standby
        # phase before collecting samples; set_motion_command() only updates
        # the host-side desired state and does not itself write the servos.
        self.muto.tick_motion()
        self.imu = ImuPublisher(self, self.muto, imu_link)
        scheduler_started = time.monotonic()
        self.telemetry_scheduler = TelemetryScheduler(
            self.imu_poll_rate_hz,
            self.imu_attitude_poll_rate_hz,
            scheduler_started,
        )
        self.telemetry_control_slot_count = 0
        self.telemetry_idle_slot_count = 0
        self.raw_telemetry_selected_count = 0
        self.attitude_telemetry_selected_count = 0
        self.raw_telemetry_deferred_count = 0
        self.attitude_telemetry_deferred_count = 0
        self.last_selected_telemetry_endpoint = ''
        self.telemetry_status_publish_count = 0
        self.next_telemetry_status_monotonic = scheduler_started
        # Reassert standby after the bounded calibration interval, then start
        # the unified gait-first control-slot callback.
        self.update_locomotion()
        self.locomotion_timer = self.create_timer(
            1.0 / self.locomotion_update_rate_hz,
            self.update_locomotion,
        )
        self.get_logger().info(
            'Coordinated gait-first telemetry scheduler: raw 0x61 at '
            f'{imu_publish_rate_hz:.1f} Hz, '
            f'fused 0x60 at {imu_attitude_publish_rate_hz:.1f} Hz; '
            'at most one telemetry transaction per gait slot. Raw identical '
            'accel/gyro snapshots are '
            + (
                'suppressed'
                if self.imu.suppress_identical_snapshots
                else 'republished for diagnostics'
            ))
        if imu_attitude_publish_rate_hz > 0.0:
            self.get_logger().info(
                'Controller-fused 0x60 attitude polling enabled at '
                f'{imu_attitude_publish_rate_hz:.1f} Hz on '
                '/imu/controller_attitude; localization decides whether to '
                'use the guarded yaw adapter')
        else:
            self.get_logger().info(
                'Controller-fused 0x60 attitude polling disabled')
        self.get_logger().info(
            'Locomotion phase loop set to '
            f'{self.locomotion_update_rate_hz:.1f} Hz with '
            f'{self.cmd_vel_timeout:.2f} s cmd_vel timeout')
        self.get_logger().info(
            'Gait phase serial batching is '
            f'{"enabled" if self.batch_gait_phase_writes else "disabled"}')
        self.get_logger().info(
            'Locomotion cmd_vel mapping: ' + calibration_description)

    def update_locomotion(self):
        """Advance one gait phase from the most recent velocity command."""
        now_monotonic = time.monotonic()
        if self.motors_released:
            # Torque release suppresses leg commands, not sensor diagnostics.
            # Keep a fresh slot boundary so guarded telemetry can continue.
            self.last_locomotion_tick_monotonic = now_monotonic
            self.service_imu_telemetry()
            self.maybe_publish_imu_telemetry_status()
            return

        previous_tick = self.last_locomotion_tick_monotonic
        self.last_locomotion_tick_monotonic = now_monotonic
        tick_period = 1.0 / self.locomotion_update_rate_hz
        if (previous_tick is not None
                and now_monotonic - previous_tick > 1.5 * tick_period):
            self.get_logger().warn(
                'Locomotion tick delayed: interval '
                f'{now_monotonic - previous_tick:.4f} s exceeds the '
                f'{tick_period:.4f} s target',
                throttle_duration_sec=5.0,
            )
        command_stale = (
            self.last_cmd_vel_monotonic is None
            or (
                self.cmd_vel_timeout > 0.0
                and now_monotonic - self.last_cmd_vel_monotonic
                > self.cmd_vel_timeout
            )
        )
        levels = (
            (0, 0, 0)
            if command_stale else self.desired_motion_levels
        )
        if command_stale and not self.cmd_vel_timed_out:
            if self.last_cmd_vel_monotonic is not None:
                self.get_logger().warn(
                    'cmd_vel timed out; returning locomotion to standby')
                self.publish_motion_command_state(
                    self._standby_selection('cmd_vel timeout'),
                )
            self.cmd_vel_timed_out = True
        elif not command_stale:
            self.cmd_vel_timed_out = False

        self.muto.set_motion_command(*levels)
        dispatch_start = time.monotonic()
        self.muto.tick_motion()
        dispatch_duration = time.monotonic() - dispatch_start
        if dispatch_duration > tick_period:
            self.get_logger().warn(
                'Locomotion phase dispatch missed its budget: '
                f'{dispatch_duration:.4f} s exceeds {tick_period:.4f} s',
                throttle_duration_sec=5.0,
            )

        # One scheduler owns both serial sensor endpoints. Running it after
        # the gait phase prevents independently aligned timers from letting
        # raw 0x61 traffic starve fused 0x60 attitude while the robot moves.
        self.service_imu_telemetry()
        self.maybe_publish_imu_telemetry_status()

    def _poll_imu_outcome(self):
        """Return ``(serial_attempted, publication_succeeded)`` for raw IMU."""
        response_timeout = _available_telemetry_response_timeout(self)
        if response_timeout is None:
            self.imu.note_poll_skipped_for_locomotion()
            return False, False

        return True, self.imu.publish_imu_data(
            response_timeout_sec=response_timeout)

    def poll_imu(self):
        """Poll telemetry only when it fits before the next gait deadline."""
        _, published = self._poll_imu_outcome()
        return published

    def _poll_controller_attitude_outcome(self):
        """Return ``(serial_attempted, published)`` for fused attitude."""
        response_timeout = _available_telemetry_response_timeout(self)
        if response_timeout is None:
            self.imu.note_attitude_poll_skipped_for_locomotion()
            return False, False

        return True, self.imu.publish_controller_attitude(
            response_timeout_sec=response_timeout)

    def poll_controller_attitude(self):
        """Poll fused 0x60 attitude within the same gait deadline guard."""
        _, published = self._poll_controller_attitude_outcome()
        return published

    def service_imu_telemetry(self, now_monotonic=None):
        """Run no more than one due serial telemetry job after a gait phase."""
        if now_monotonic is None:
            now_monotonic = time.monotonic()
        self.telemetry_control_slot_count += 1
        raw_due, attitude_due = self.telemetry_scheduler.due_endpoints(
            now_monotonic)
        endpoint = self.telemetry_scheduler.select(now_monotonic)
        if endpoint is None:
            self.telemetry_idle_slot_count += 1
            return False

        if raw_due and attitude_due:
            if endpoint == ATTITUDE_TELEMETRY:
                self.raw_telemetry_deferred_count += 1
            else:
                self.attitude_telemetry_deferred_count += 1

        self.last_selected_telemetry_endpoint = endpoint
        if endpoint == ATTITUDE_TELEMETRY:
            self.attitude_telemetry_selected_count += 1
            attempted, published = (
                self._poll_controller_attitude_outcome())
        else:
            self.raw_telemetry_selected_count += 1
            attempted, published = self._poll_imu_outcome()

        if attempted:
            self.telemetry_scheduler.mark_attempted(
                endpoint,
                now_monotonic,
            )
        return published

    def maybe_publish_imu_telemetry_status(self, now_monotonic=None):
        """Publish cumulative scheduler/read counters for bag analysis."""
        if now_monotonic is None:
            now_monotonic = time.monotonic()
        if now_monotonic < self.next_telemetry_status_monotonic:
            return False

        active_state = self.muto.commanded_gait_state
        message = String()
        message.data = json.dumps({
            'schema_version': 1,
            'scheduler_policy': 'gait_then_one_telemetry',
            'active_mode': active_state.mode,
            'locomotion_update_rate_hz': self.locomotion_update_rate_hz,
            'response_timeout_sec': self.imu.response_timeout_sec,
            'locomotion_guard_sec': self.imu_locomotion_guard_sec,
            'minimum_transaction_budget_sec': (
                MIN_IMU_TRANSACTION_BUDGET_SEC),
            'control_slots': self.telemetry_control_slot_count,
            'idle_slots': self.telemetry_idle_slot_count,
            'last_selected_endpoint': self.last_selected_telemetry_endpoint,
            'raw_imu': {
                'configured_poll_rate_hz': self.imu_poll_rate_hz,
                'selected': self.raw_telemetry_selected_count,
                'deferred': self.raw_telemetry_deferred_count,
                'attempted': self.imu.poll_count,
                'successful_reads': self.imu.successful_read_count,
                'failed_reads': self.imu.failed_read_count,
                'changed_snapshots': self.imu.changed_snapshot_count,
                'duplicate_snapshots': self.imu.duplicate_sample_count,
                'deadline_skips': self.imu.skipped_for_locomotion_count,
            },
            'controller_attitude': {
                'configured_poll_rate_hz': self.imu_attitude_poll_rate_hz,
                'selected': self.attitude_telemetry_selected_count,
                'deferred': self.attitude_telemetry_deferred_count,
                'attempted': self.imu.attitude_poll_count,
                'successful_reads': (
                    self.imu.successful_attitude_read_count),
                'failed_reads': self.imu.failed_attitude_read_count,
                'deadline_skips': (
                    self.imu.attitude_skipped_for_locomotion_count),
            },
        }, separators=(',', ':'), sort_keys=True)
        self.imu_telemetry_status_pub.publish(message)
        self.telemetry_status_publish_count += 1
        self.next_telemetry_status_monotonic = _advance_periodic_deadline(
            self.next_telemetry_status_monotonic,
            IMU_TELEMETRY_STATUS_PERIOD_SEC,
            now_monotonic,
        )
        return True

    def publish_commanded_gait_state(self, state):
        """Publish the trajectory phase just sent to the motor controller."""
        msg = CommandedGaitState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gait_state_frame_id
        msg.sequence = state.sequence
        msg.mode = state.mode
        msg.phase_index = state.phase_index
        msg.cycle_length = state.cycle_length
        msg.cycle_complete = state.cycle_complete
        msg.leg_state = [
            CommandedGaitState.STANCE if in_stance
            else CommandedGaitState.SWING
            for in_stance in state.commanded_stance
        ]
        # The vendor model uses x=right/y=forward. Publish REP-103 base-frame
        # coordinates: x=forward/y=left.
        msg.foot_x_mm = [point[1] for point in state.foot_positions_mm]
        msg.foot_y_mm = [-point[0] for point in state.foot_positions_mm]
        msg.foot_z_mm = [point[2] for point in state.foot_positions_mm]
        self.last_gait_command_monotonic = time.monotonic()
        self.gait_phase_monotonic_times.append(
            self.last_gait_command_monotonic)
        if len(self.gait_phase_monotonic_times) >= 2:
            elapsed = (
                self.gait_phase_monotonic_times[-1]
                - self.gait_phase_monotonic_times[0]
            )
            if elapsed > 0.0:
                self.observed_phase_rate_hz = (
                    (len(self.gait_phase_monotonic_times) - 1) / elapsed
                )
                relative_rate_error = abs(
                    self.observed_phase_rate_hz
                    - self.locomotion_update_rate_hz
                ) / self.locomotion_update_rate_hz
                if (len(self.gait_phase_monotonic_times)
                        == self.gait_phase_monotonic_times.maxlen
                        and relative_rate_error > 0.10):
                    self.get_logger().warn(
                        'Observed gait phase rate '
                        f'{self.observed_phase_rate_hz:.1f} Hz differs from '
                        f'configured calibration condition '
                        f'{self.locomotion_update_rate_hz:.1f} Hz by more '
                        'than 10%; physical velocity prediction is degraded',
                        throttle_duration_sec=5.0,
                    )
        self.gait_state_pub.publish(msg)
        if getattr(self, 'last_velocity_selection', None) is not None:
            self.publish_motion_command_state(
                self.last_velocity_selection,
                active_state=state,
            )

    def cmd_vel_callback(self, msg):
        """Map one physical Twist and latch discrete calibrated gait levels."""
        if not isinstance(msg, Twist):
            return
        if self.motors_released:
            self.get_logger().warn(
                'Ignoring cmd_vel because joint torque was released',
                throttle_duration_sec=5.0,
            )
            return

        try:
            if self.locomotion_command_mapping == CALIBRATED_MAPPING:
                selection = self.velocity_mapper.select(
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                )
            else:
                selection = legacy_velocity_selection(
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                    self.locomotion_update_rate_hz,
                )
        except ValueError as exc:
            # A malformed Twist must stop the robot instead of leaving the
            # previous valid command latched.
            safe_requested = tuple(
                value if math.isfinite(value) else 0.0
                for value in (msg.linear.x, msg.linear.y, msg.angular.z)
            )
            selection = self._standby_selection(
                'invalid cmd_vel: %s' % exc,
                requested=safe_requested,
                supported=False,
            )

        self.vel_x = selection.levels.x_level
        self.vel_y = selection.levels.y_level
        self.angular_z = selection.levels.z_level
        self.desired_motion_levels = (
            self.vel_x, self.vel_y, self.angular_z)
        self.last_velocity_selection = selection
        self.last_cmd_vel_monotonic = time.monotonic()
        self.publish_motion_command_state(selection)

        if not selection.supported:
            self.get_logger().warn(
                'Rejected unsupported cmd_vel: ' + selection.detail,
                throttle_duration_sec=5.0,
            )
        elif selection.saturated:
            self.get_logger().warn(
                'cmd_vel projected onto calibrated gait envelope: '
                + selection.detail,
                throttle_duration_sec=5.0,
            )
        elif selection.projection.any_to_zero:
            self.get_logger().warn(
                'cmd_vel is below the minimum executable gait level: '
                + selection.detail,
                throttle_duration_sec=5.0,
            )

    def _standby_selection(
            self, detail, requested=(0.0, 0.0, 0.0), supported=True):
        profile_id = (
            self.velocity_mapper.profile.profile_id
            if self.velocity_mapper is not None else LEGACY_MAPPING
        )
        return VelocitySelection(
            requested=PlanarVelocity(*requested),
            predicted=PlanarVelocity(0.0, 0.0, 0.0),
            levels=MotionLevels(),
            mode='standby',
            saturation=SaturationFlags(
                x=not supported and requested[0] != 0.0,
                y=not supported and requested[1] != 0.0,
                yaw=not supported and requested[2] != 0.0,
                unsupported_combination=not supported,
            ),
            profile_id=profile_id,
            phase_rate_hz=self.locomotion_update_rate_hz,
            detail=detail,
            supported=supported,
            unsupported_reason=(None if supported else detail),
        )

    def publish_motion_command_state(self, selection, active_state=None):
        """Publish selected and actually active gait-command state."""
        if active_state is None:
            active_state = self.muto.commanded_gait_state
        msg = MotionCommandState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gait_state_frame_id
        msg.requested_twist.linear.x = selection.requested.linear_x_m_s
        msg.requested_twist.linear.y = selection.requested.linear_y_m_s
        msg.requested_twist.angular.z = selection.requested.angular_z_rad_s
        msg.predicted_twist.linear.x = selection.predicted.linear_x_m_s
        msg.predicted_twist.linear.y = selection.predicted.linear_y_m_s
        msg.predicted_twist.angular.z = selection.predicted.angular_z_rad_s
        msg.x_level = selection.levels.x_level
        msg.y_level = selection.levels.y_level
        msg.z_level = selection.levels.z_level
        msg.mode = selection.mode
        msg.active_x_level = active_state.x_level
        msg.active_y_level = active_state.y_level
        msg.active_z_level = active_state.z_level
        msg.active_mode = active_state.mode
        msg.replacement_pending = active_state.replacement_pending
        msg.saturated = selection.saturated
        msg.x_saturated = selection.saturation.x
        msg.y_saturated = selection.saturation.y
        msg.yaw_saturated = selection.saturation.yaw
        msg.projected = selection.projected
        msg.quantized = selection.quantized
        msg.x_projected = selection.projection.x
        msg.y_projected = selection.projection.y
        msg.yaw_projected = selection.projection.yaw
        msg.x_projected_to_zero = selection.projection.x_to_zero
        msg.y_projected_to_zero = selection.projection.y_to_zero
        msg.yaw_projected_to_zero = selection.projection.yaw_to_zero
        msg.unsupported = not selection.supported
        msg.unsupported_combination = (
            selection.saturation.unsupported_combination)
        msg.detail = selection.detail
        msg.calibration_profile = selection.profile_id
        msg.calibration_phase_rate_hz = selection.phase_rate_hz
        msg.configured_phase_rate_hz = self.locomotion_update_rate_hz
        msg.observed_phase_rate_hz = self.observed_phase_rate_hz
        self.motion_command_state_pub.publish(msg)

    def get_motor_angles_callback(self, request, response):
        """Return motor angles with the gait target active during the read."""
        del request
        if self.motors_released:
            response.success = False
            response.message = json.dumps({
                'error': 'motors_released',
                'detail': 'joint torque is disabled',
            })
            return response
        if self.last_gait_command_monotonic is None:
            response.success = False
            response.message = json.dumps({
                'error': 'no_commanded_gait_sample',
                'detail': 'locomotion loop has not emitted its first phase',
            })
            return response

        # This callback shares the driver's single-threaded executor with the
        # 50 Hz gait-slotted telemetry loop. Never sleep here to let a target
        # settle: doing so pauses both gait and sensor servicing. The motor
        # residual must reflect tracking during the normal moving gait, not an
        # artificial hold.
        read_start_monotonic = time.monotonic()
        try:
            state, angles = self.muto.read_motor_with_gait_state()
        except Exception as exc:
            response.success = False
            response.message = json.dumps({
                'error': 'read_motor_failed',
                'detail': str(exc),
            })
            return response
        read_duration_sec = time.monotonic() - read_start_monotonic

        if not angles or len(angles) != 18:
            response.success = False
            response.message = json.dumps({
                'error': 'invalid_motor_angle_data',
                'angles': angles or [],
                'expected_count': 18,
            })
            return response

        command_age = time.monotonic() - self.last_gait_command_monotonic
        stamp = self.get_clock().now().to_msg()
        leg_state = [
            CommandedGaitState.STANCE if in_stance
            else CommandedGaitState.SWING
            for in_stance in state.commanded_stance
        ]
        response.success = True
        response.message = json.dumps({
            'count': len(angles),
            'angles': angles,
            'angle_space': 'firmware_calibrated_logical_degrees',
            'standby_leg_angles_deg': list(STANDBY_SERVO_ANGLES_DEG),
            'command_age_sec': command_age,
            'read_duration_sec': read_duration_sec,
            'sample_stamp': {
                'sec': stamp.sec,
                'nanosec': stamp.nanosec,
            },
            'gait_state': {
                'frame_id': self.gait_state_frame_id,
                'sequence': state.sequence,
                'mode': state.mode,
                'x_level': state.x_level,
                'y_level': state.y_level,
                'z_level': state.z_level,
                'replacement_pending': state.replacement_pending,
                'phase_index': state.phase_index,
                'cycle_length': state.cycle_length,
                'leg_state': leg_state,
                'foot_x_mm': [
                    point[1] for point in state.foot_positions_mm
                ],
                'foot_y_mm': [
                    -point[0] for point in state.foot_positions_mm
                ],
                'foot_z_mm': [
                    point[2] for point in state.foot_positions_mm
                ],
            },
            'servo_angles': {
                str(index + 1): angle
                for index, angle in enumerate(angles)
            },
        })
        return response

    def release_motors_callback(self, request, response):
        """Disable all joint torque and stop the locomotion timer's output."""
        del request
        self.motors_released = True
        self.vel_x = 0
        self.vel_y = 0
        self.angular_z = 0
        self.desired_motion_levels = (0, 0, 0)
        self.last_cmd_vel_monotonic = None
        try:
            for servo_id in range(1, 19):
                self.muto.Servo_torque_off(servo_id)
        except Exception as exc:
            response.success = False
            response.message = json.dumps({
                'error': 'release_motors_failed',
                'detail': str(exc),
            })
            return response

        selection = self._standby_selection('motor torque released')
        self.last_velocity_selection = selection
        self.publish_motion_command_state(selection)
        response.success = True
        response.message = json.dumps({
            'released': True,
            'servo_ids': list(range(1, 19)),
            'detail': 'Torque disabled for all joint servos',
        })
        return response

    def Buzzercallback(self, msg):
        """Drive the vendor buzzer command."""
        if not isinstance(msg, Bool):
            return
        value = 255 if msg.data else 0
        for _ in range(3):
            self.muto.buzzer(value)

    def destroy_node(self):
        """Close the shared serial device before destroying the ROS node."""
        try:
            self.muto.close()
        finally:
            return super().destroy_node()


def main():
    """Run the Muto ROS hardware driver."""
    rclpy.init()
    driver = yahboomcar_driver('driver_node')
    try:
        rclpy.spin(driver)
    finally:
        driver.destroy_node()
        rclpy.shutdown()
