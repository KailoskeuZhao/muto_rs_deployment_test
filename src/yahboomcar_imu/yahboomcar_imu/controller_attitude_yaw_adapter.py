from collections import deque
import json
import math

from muto_hexapod_interfaces_custom.msg import (
    ControllerAttitude,
    MotionCommandState,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


DEFAULT_INPUT_TOPIC = '/imu/controller_attitude'
DEFAULT_OUTPUT_TOPIC = '/imu/controller_attitude_imu'
DEFAULT_MOTION_STATE_TOPIC = '/muto/motion_command_state'
DEFAULT_STATUS_TOPIC = '/muto/controller_attitude_yaw_status'
DEFAULT_FRAME_ID = 'imu_link'
# The stop-only gate removes moving 5 Hz correction steps. Four degrees gives
# the controller attitude modestly more influence than the former five-degree
# test setting without treating approximate field observations as truth.
DEFAULT_YAW_VARIANCE_RAD2 = math.radians(4.0) ** 2
DEFAULT_STATIONARY_SETTLE_SEC = 2.0
DEFAULT_MOTION_STATE_TIMEOUT_SEC = 0.25
DEFAULT_STABILITY_WINDOW_SEC = 1.0
DEFAULT_MINIMUM_DISTINCT_SNAPSHOTS = 3
DEFAULT_MAX_STATIONARY_YAW_SPAN_RAD = math.radians(1.0)
DEFAULT_MAX_ATTITUDE_STEP_RAD = math.radians(2.0)
DEFAULT_MAX_STATIONARY_ATTITUDE_DELTA_RAD = math.radians(3.0)
# Zero means one stable correction per stationary episode. All changed 5 Hz
# samples still participate in the stability decision.
DEFAULT_STATIONARY_REPUBLISH_INTERVAL_SEC = 0.0
DEFAULT_LINEAR_COMMAND_THRESHOLD = 1.0e-6
DEFAULT_ANGULAR_COMMAND_THRESHOLD = 1.0e-6
UNUSED_ORIENTATION_VARIANCE = 1.0e6


def wrap_angle(angle_rad):
    """Wrap an angle to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_quaternion(yaw_rad):
    """Return an x/y/z/w quaternion for a ROS-positive yaw angle."""
    half_yaw = 0.5 * yaw_rad
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def circular_mean(angles_rad):
    """Return a wrap-safe mean for a nonempty angle sequence."""
    if not angles_rad:
        raise ValueError('cannot average an empty angle sequence')
    sine_sum = sum(math.sin(angle) for angle in angles_rad)
    cosine_sum = sum(math.cos(angle) for angle in angles_rad)
    if math.hypot(sine_sum, cosine_sum) <= 1.0e-12:
        raise ValueError('circular mean is undefined for opposing angles')
    return math.atan2(sine_sum, cosine_sum)


def circular_span(angles_rad):
    """Return the smallest local span around the circular mean."""
    mean = circular_mean(angles_rad)
    offsets = [wrap_angle(angle - mean) for angle in angles_rad]
    return max(offsets) - min(offsets)


def controller_snapshot(message):
    """Identify a fused Euler cache snapshot without using temperature."""
    return (
        message.roll_deg,
        message.pitch_deg,
        message.yaw_deg,
    )


def motion_state_is_stationary(
    message,
    linear_threshold=DEFAULT_LINEAR_COMMAND_THRESHOLD,
    angular_threshold=DEFAULT_ANGULAR_COMMAND_THRESHOLD,
):
    """Require both selected and active locomotion states to be at rest."""
    selected_amplitudes = (
        getattr(message, 'x_amplitude', 0.0),
        getattr(message, 'y_amplitude', 0.0),
        getattr(message, 'z_amplitude', 0.0),
    )
    active_amplitudes = (
        getattr(message, 'active_x_amplitude', 0.0),
        getattr(message, 'active_y_amplitude', 0.0),
        getattr(message, 'active_z_amplitude', 0.0),
    )
    selected_compat_levels = (
        message.x_level,
        message.y_level,
        message.z_level,
    )
    active_compat_levels = (
        message.active_x_level,
        message.active_y_level,
        message.active_z_level,
    )
    requested = message.requested_twist
    return (
        message.mode == 'standby'
        and message.active_mode == 'standby'
        and not message.replacement_pending
        and all(amplitude == 0 for amplitude in selected_amplitudes)
        and all(amplitude == 0 for amplitude in active_amplitudes)
        and all(level == 0 for level in selected_compat_levels)
        and all(level == 0 for level in active_compat_levels)
        and math.hypot(requested.linear.x, requested.linear.y)
        <= linear_threshold
        and abs(requested.angular.z) <= angular_threshold
    )


def controller_attitude_to_imu(
    message,
    frame_id,
    yaw_variance_rad2,
    output_yaw_rad=None,
):
    """Convert controller yaw, or its stable circular mean, to yaw-only IMU."""
    angles = (
        message.roll_deg,
        message.pitch_deg,
        message.yaw_deg,
    )
    if not all(math.isfinite(value) for value in angles):
        raise ValueError('controller attitude contains a non-finite angle')
    if not math.isfinite(yaw_variance_rad2) or yaw_variance_rad2 <= 0.0:
        raise ValueError('yaw variance must be finite and positive')
    if output_yaw_rad is None:
        output_yaw_rad = math.radians(message.yaw_deg)
    if not math.isfinite(output_yaw_rad):
        raise ValueError('output yaw must be finite')

    output = Imu()
    output.header.stamp = message.header.stamp
    output.header.frame_id = frame_id

    quaternion = yaw_quaternion(wrap_angle(output_yaw_rad))
    output.orientation.x = quaternion[0]
    output.orientation.y = quaternion[1]
    output.orientation.z = quaternion[2]
    output.orientation.w = quaternion[3]

    # Only yaw is consumed by the EKF overlay. Large roll and pitch variances
    # make the contract explicit for any other subscriber.
    output.orientation_covariance[0] = UNUSED_ORIENTATION_VARIANCE
    output.orientation_covariance[4] = UNUSED_ORIENTATION_VARIANCE
    output.orientation_covariance[8] = yaw_variance_rad2
    output.angular_velocity_covariance[0] = -1.0
    output.linear_acceleration_covariance[0] = -1.0
    return output


class StationaryAttitudeGate:
    """Admit stable controller yaw only during confirmed stationary periods.

    The first accepted startup sample establishes robot_localization's
    relative-heading reference. Later samples are collected at the controller's
    full changed-snapshot rate, but only a stable circular mean is published
    after settling. At most one correction per configured interval is emitted.
    """

    def __init__(
        self,
        stationary_settle_sec=DEFAULT_STATIONARY_SETTLE_SEC,
        motion_state_timeout_sec=DEFAULT_MOTION_STATE_TIMEOUT_SEC,
        stability_window_sec=DEFAULT_STABILITY_WINDOW_SEC,
        minimum_distinct_snapshots=DEFAULT_MINIMUM_DISTINCT_SNAPSHOTS,
        max_stationary_yaw_span_rad=DEFAULT_MAX_STATIONARY_YAW_SPAN_RAD,
        max_attitude_step_rad=DEFAULT_MAX_ATTITUDE_STEP_RAD,
        max_stationary_attitude_delta_rad=(
            DEFAULT_MAX_STATIONARY_ATTITUDE_DELTA_RAD
        ),
        stationary_republish_interval_sec=(
            DEFAULT_STATIONARY_REPUBLISH_INTERVAL_SEC
        ),
    ):
        self.stationary_settle_sec = stationary_settle_sec
        self.motion_state_timeout_sec = motion_state_timeout_sec
        self.stability_window_sec = stability_window_sec
        self.minimum_distinct_snapshots = minimum_distinct_snapshots
        self.max_stationary_yaw_span_rad = max_stationary_yaw_span_rad
        self.max_attitude_step_rad = max_attitude_step_rad
        self.max_stationary_attitude_delta_rad = (
            max_stationary_attitude_delta_rad
        )
        self.stationary_republish_interval_sec = (
            stationary_republish_interval_sec
        )

        self.motion_stationary = False
        self.stationary_since = None
        self.latest_motion_stamp = None
        self.last_attitude_stamp = None
        self.initial_anchor_published = False
        self.startup_anchor_blocked = False
        self.blocked_until_motion = False
        self.correction_published_this_episode = False
        self.last_publish_stamp = None
        self.episode_anchor_yaw = None
        self.last_stationary_yaw = None
        self.samples = deque()

    def _reset_episode(self):
        self.correction_published_this_episode = False
        self.blocked_until_motion = False
        self.episode_anchor_yaw = None
        self.last_stationary_yaw = None
        self.samples.clear()

    def update_motion(self, stationary, stamp_sec):
        if not math.isfinite(stamp_sec):
            return False
        if (
            self.latest_motion_stamp is not None
            and stamp_sec < self.latest_motion_stamp
        ):
            return False

        state_gap = (
            None
            if self.latest_motion_stamp is None
            else stamp_sec - self.latest_motion_stamp
        )
        new_stationary_episode = stationary and (
            not self.motion_stationary
            or state_gap is None
            or state_gap > self.motion_state_timeout_sec
        )
        self.latest_motion_stamp = stamp_sec
        self.motion_stationary = stationary

        if not stationary:
            if not self.initial_anchor_published:
                # imu0_relative cannot safely establish its zero after the
                # robot has already changed heading.
                self.startup_anchor_blocked = True
            self.stationary_since = None
            self._reset_episode()
        elif new_stationary_episode:
            self.stationary_since = stamp_sec
            self._reset_episode()
        return True

    def _motion_state_is_fresh(self, sample_stamp):
        if self.latest_motion_stamp is None:
            return False
        age = sample_stamp - self.latest_motion_stamp
        return -0.05 <= age <= self.motion_state_timeout_sec

    def _append_sample(self, stamp_sec, yaw_rad):
        self.samples.append((stamp_sec, yaw_rad))
        window_start = stamp_sec - self.stability_window_sec
        while self.samples and self.samples[0][0] < window_start:
            self.samples.popleft()

    def update_attitude(self, controller_yaw_rad, stamp_sec):
        """Return ``(accepted_yaw, reason)`` for one changed snapshot."""
        if not math.isfinite(controller_yaw_rad) or not math.isfinite(stamp_sec):
            return None, 'invalid'
        if (
            self.last_attitude_stamp is not None
            and stamp_sec <= self.last_attitude_stamp
        ):
            return None, 'out_of_order'
        self.last_attitude_stamp = stamp_sec

        if self.startup_anchor_blocked and not self.initial_anchor_published:
            return None, 'startup_anchor_missed'
        if not self.motion_stationary:
            return None, 'moving'
        if not self._motion_state_is_fresh(stamp_sec):
            self.motion_stationary = False
            self.stationary_since = None
            self._reset_episode()
            return None, 'stale_motion_state'
        if self.blocked_until_motion:
            return None, 'magnetic_guard'

        yaw_rad = wrap_angle(controller_yaw_rad)
        if not self.initial_anchor_published:
            self.initial_anchor_published = True
            self.last_publish_stamp = stamp_sec
            self.episode_anchor_yaw = yaw_rad
            self.last_stationary_yaw = yaw_rad
            self._append_sample(stamp_sec, yaw_rad)
            return yaw_rad, 'initial_anchor'

        if self.episode_anchor_yaw is None:
            self.episode_anchor_yaw = yaw_rad
            self.last_stationary_yaw = yaw_rad
        else:
            step = wrap_angle(yaw_rad - self.last_stationary_yaw)
            if abs(step) > self.max_attitude_step_rad:
                self.blocked_until_motion = True
                return None, 'magnetic_guard'
            if abs(wrap_angle(yaw_rad - self.episode_anchor_yaw)) > (
                self.max_stationary_attitude_delta_rad
            ):
                self.blocked_until_motion = True
                return None, 'magnetic_guard'
            self.last_stationary_yaw = yaw_rad
        self._append_sample(stamp_sec, yaw_rad)

        if (
            self.stationary_since is None
            or stamp_sec - self.stationary_since
            < self.stationary_settle_sec
        ):
            return None, 'settling'
        if len(self.samples) < self.minimum_distinct_snapshots:
            return None, 'insufficient_samples'
        if self.correction_published_this_episode:
            if self.stationary_republish_interval_sec <= 0.0:
                return None, 'correction_held'
            if stamp_sec - self.last_publish_stamp < (
                self.stationary_republish_interval_sec
            ):
                return None, 'correction_held'

        sample_yaws = [yaw for _, yaw in self.samples]
        try:
            span = circular_span(sample_yaws)
            mean_yaw = circular_mean(sample_yaws)
        except ValueError:
            return None, 'unstable_attitude'
        if span > self.max_stationary_yaw_span_rad:
            return None, 'unstable_attitude'

        self.correction_published_this_episode = True
        self.last_publish_stamp = stamp_sec
        return mean_yaw, 'stationary_correction'


class ControllerAttitudeYawAdapter(Node):
    """Expose vendor 0x60 yaw as a guarded, yaw-only EKF input."""

    def __init__(self):
        super().__init__('controller_attitude_yaw_adapter')

        self.input_topic = self.declare_parameter(
            'input_topic', DEFAULT_INPUT_TOPIC
        ).value
        self.output_topic = self.declare_parameter(
            'output_topic', DEFAULT_OUTPUT_TOPIC
        ).value
        self.frame_id = self.declare_parameter(
            'frame_id', DEFAULT_FRAME_ID
        ).value
        self.yaw_variance_rad2 = float(self.declare_parameter(
            'yaw_variance_rad2', DEFAULT_YAW_VARIANCE_RAD2
        ).value)
        self.suppress_identical_snapshots = bool(self.declare_parameter(
            'suppress_identical_snapshots', True
        ).value)
        self.stationary_gate_enabled = bool(self.declare_parameter(
            'stationary_gate_enabled', True
        ).value)
        self.motion_state_topic = self.declare_parameter(
            'motion_state_topic', DEFAULT_MOTION_STATE_TOPIC
        ).value
        self.status_topic = self.declare_parameter(
            'status_topic', DEFAULT_STATUS_TOPIC
        ).value
        self.stationary_settle_sec = float(self.declare_parameter(
            'stationary_settle_sec', DEFAULT_STATIONARY_SETTLE_SEC
        ).value)
        self.motion_state_timeout_sec = float(self.declare_parameter(
            'motion_state_timeout_sec', DEFAULT_MOTION_STATE_TIMEOUT_SEC
        ).value)
        self.stability_window_sec = float(self.declare_parameter(
            'stability_window_sec', DEFAULT_STABILITY_WINDOW_SEC
        ).value)
        self.minimum_distinct_snapshots = int(self.declare_parameter(
            'minimum_distinct_snapshots',
            DEFAULT_MINIMUM_DISTINCT_SNAPSHOTS,
        ).value)
        self.max_stationary_yaw_span_rad = float(self.declare_parameter(
            'max_stationary_yaw_span_rad',
            DEFAULT_MAX_STATIONARY_YAW_SPAN_RAD,
        ).value)
        self.max_attitude_step_rad = float(self.declare_parameter(
            'max_attitude_step_rad', DEFAULT_MAX_ATTITUDE_STEP_RAD
        ).value)
        self.max_stationary_attitude_delta_rad = float(
            self.declare_parameter(
                'max_stationary_attitude_delta_rad',
                DEFAULT_MAX_STATIONARY_ATTITUDE_DELTA_RAD,
            ).value
        )
        self.stationary_republish_interval_sec = float(
            self.declare_parameter(
                'stationary_republish_interval_sec',
                DEFAULT_STATIONARY_REPUBLISH_INTERVAL_SEC,
            ).value
        )
        self.linear_command_threshold = float(self.declare_parameter(
            'linear_command_threshold', DEFAULT_LINEAR_COMMAND_THRESHOLD
        ).value)
        self.angular_command_threshold = float(self.declare_parameter(
            'angular_command_threshold', DEFAULT_ANGULAR_COMMAND_THRESHOLD
        ).value)
        self._normalize_parameters()

        self.last_snapshot = None
        self.received_count = 0
        self.changed_count = 0
        self.published_count = 0
        self.duplicate_count = 0
        self.waiting_for_subscriber_count = 0
        self.rejection_counts = {}
        self.last_gate_reason = 'startup'

        self.publisher = self.create_publisher(Imu, self.output_topic, 100)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_qos
        )
        self.subscription = self.create_subscription(
            ControllerAttitude,
            self.input_topic,
            self.attitude_callback,
            100,
        )

        self.gate = None
        self.motion_subscription = None
        if self.stationary_gate_enabled:
            self.gate = StationaryAttitudeGate(
                stationary_settle_sec=self.stationary_settle_sec,
                motion_state_timeout_sec=self.motion_state_timeout_sec,
                stability_window_sec=self.stability_window_sec,
                minimum_distinct_snapshots=(
                    self.minimum_distinct_snapshots
                ),
                max_stationary_yaw_span_rad=(
                    self.max_stationary_yaw_span_rad
                ),
                max_attitude_step_rad=self.max_attitude_step_rad,
                max_stationary_attitude_delta_rad=(
                    self.max_stationary_attitude_delta_rad
                ),
                stationary_republish_interval_sec=(
                    self.stationary_republish_interval_sec
                ),
            )
            motion_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                # A volatile request connects to both the live driver's
                # transient-local publisher and ordinary ros2 bag play.
                durability=DurabilityPolicy.VOLATILE,
            )
            self.motion_subscription = self.create_subscription(
                MotionCommandState,
                self.motion_state_topic,
                self.motion_state_callback,
                motion_qos,
            )

        self.status_timer = self.create_timer(1.0, self.publish_status)
        mode_description = (
            'stable stop-only relative-heading corrections'
            if self.stationary_gate_enabled
            else 'continuous relative-yaw test mode'
        )
        self.get_logger().info(
            'Converting controller-fused yaw from '
            f'{self.input_topic} -> {self.output_topic}; '
            f'frame={self.frame_id}, variance={self.yaw_variance_rad2:.6g} '
            f'rad^2, mode={mode_description}, exact cached snapshots '
            f'suppressed={self.suppress_identical_snapshots}. The output '
            'contains orientation yaw only, not angular velocity.'
        )

    def _normalize_parameters(self):
        if not self.frame_id:
            self.get_logger().warn(
                'frame_id must not be empty; using imu_link'
            )
            self.frame_id = DEFAULT_FRAME_ID
        positive_defaults = (
            ('yaw_variance_rad2', DEFAULT_YAW_VARIANCE_RAD2),
            ('motion_state_timeout_sec', DEFAULT_MOTION_STATE_TIMEOUT_SEC),
            ('stability_window_sec', DEFAULT_STABILITY_WINDOW_SEC),
            (
                'max_stationary_yaw_span_rad',
                DEFAULT_MAX_STATIONARY_YAW_SPAN_RAD,
            ),
            ('max_attitude_step_rad', DEFAULT_MAX_ATTITUDE_STEP_RAD),
            (
                'max_stationary_attitude_delta_rad',
                DEFAULT_MAX_STATIONARY_ATTITUDE_DELTA_RAD,
            ),
        )
        for name, default in positive_defaults:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                self.get_logger().warn(
                    f'{name} must be finite and positive; using {default:.6g}'
                )
                setattr(self, name, default)
        for name, default in (
            ('stationary_settle_sec', DEFAULT_STATIONARY_SETTLE_SEC),
            (
                'stationary_republish_interval_sec',
                DEFAULT_STATIONARY_REPUBLISH_INTERVAL_SEC,
            ),
            ('linear_command_threshold', DEFAULT_LINEAR_COMMAND_THRESHOLD),
            ('angular_command_threshold', DEFAULT_ANGULAR_COMMAND_THRESHOLD),
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                self.get_logger().warn(
                    f'{name} must be finite and non-negative; using '
                    f'{default:.6g}'
                )
                setattr(self, name, default)
        if self.minimum_distinct_snapshots < 2:
            self.get_logger().warn(
                'minimum_distinct_snapshots must be at least 2; using '
                f'{DEFAULT_MINIMUM_DISTINCT_SNAPSHOTS}'
            )
            self.minimum_distinct_snapshots = (
                DEFAULT_MINIMUM_DISTINCT_SNAPSHOTS
            )

    def motion_state_callback(self, message):
        stationary = motion_state_is_stationary(
            message,
            self.linear_command_threshold,
            self.angular_command_threshold,
        )
        self.gate.update_motion(
            stationary,
            stamp_seconds(message.header.stamp),
        )

    def _note_rejection(self, reason):
        self.last_gate_reason = reason
        self.rejection_counts[reason] = (
            self.rejection_counts.get(reason, 0) + 1
        )

    def attitude_callback(self, message):
        self.received_count += 1
        # Preserve the initial relative-heading anchor until the EKF has
        # discovered this publisher.
        if self.publisher.get_subscription_count() == 0:
            self.waiting_for_subscriber_count += 1
            return

        snapshot = controller_snapshot(message)
        if self.suppress_identical_snapshots and snapshot == self.last_snapshot:
            self.duplicate_count += 1
            return
        self.last_snapshot = snapshot
        self.changed_count += 1

        output_yaw_rad = None
        if self.stationary_gate_enabled:
            if not math.isfinite(message.yaw_deg):
                self._note_rejection('invalid')
                return
            output_yaw_rad, reason = self.gate.update_attitude(
                math.radians(message.yaw_deg),
                stamp_seconds(message.header.stamp),
            )
            self.last_gate_reason = reason
            if output_yaw_rad is None:
                self._note_rejection(reason)
                return

        try:
            output = controller_attitude_to_imu(
                message,
                self.frame_id,
                self.yaw_variance_rad2,
                output_yaw_rad=output_yaw_rad,
            )
        except ValueError as error:
            self._note_rejection('invalid')
            self.get_logger().warn(
                f'Rejecting controller attitude: {error}',
                throttle_duration_sec=2.0,
            )
            return

        self.publisher.publish(output)
        self.published_count += 1

    def publish_status(self):
        message = String()
        payload = {
            'schema_version': 1,
            'mode': (
                'stationary_gate'
                if self.stationary_gate_enabled else 'continuous_relative'
            ),
            'received': self.received_count,
            'changed': self.changed_count,
            'duplicates': self.duplicate_count,
            'published': self.published_count,
            'waiting_for_subscriber': self.waiting_for_subscriber_count,
            'rejected': dict(sorted(self.rejection_counts.items())),
            'last_gate_reason': self.last_gate_reason,
            'yaw_variance_rad2': self.yaw_variance_rad2,
        }
        if self.gate is not None:
            payload.update({
                'stationary': self.gate.motion_stationary,
                'initial_anchor_published': (
                    self.gate.initial_anchor_published
                ),
                'startup_anchor_blocked': (
                    self.gate.startup_anchor_blocked
                ),
                'blocked_until_motion': self.gate.blocked_until_motion,
                'buffered_samples': len(self.gate.samples),
            })
        message.data = json.dumps(
            payload, separators=(',', ':'), sort_keys=True
        )
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerAttitudeYawAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
