#!/usr/bin/env python3
"""Motor-validated commanded-stance odometry for the Muto hexapod."""

import json
import math

from geometry_msgs.msg import TransformStamped
from muto_hexapod_interfaces_custom.msg import CommandedGaitState
from muto_hexapod_lib_custom.core.leg import (
    servo_angles_to_foot_positions,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .commanded_gait_odometry import (
    CommandedStanceOdometry,
    GaitObservation,
)


class FootOdometryNode(Node):
    """
    Estimate body motion from commanded stance with motor FK validation.

    Feet commanded in stance across consecutive gait phases are assumed fixed
    in the odom frame. Calibrated logical motor readback must track the same
    kinematic targets before an odometry sample is published. This node still
    has no contact, load, or slip measurement.
    """

    def __init__(self):
        super().__init__('foot_odometry_node')

        self.motor_service_name = self.declare_parameter(
            'motor_service_name', 'get_motor_angles').value
        self.gait_state_topic = self.declare_parameter(
            'gait_state_topic', '/muto/commanded_gait_state').value
        self.odom_topic = self.declare_parameter(
            'odom_topic', '/foot_odom').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.child_frame_id = self.declare_parameter(
            'child_frame_id', 'base_frame').value
        self.publish_tf = self.declare_parameter('publish_tf', False).value

        self.motor_poll_rate = float(
            self.declare_parameter('motor_poll_rate', 2.0).value)
        self.motor_stale_timeout = max(
            0.0,
            float(self.declare_parameter(
                'motor_stale_timeout', 1.0).value),
        )
        self.gait_state_stale_timeout = max(
            0.0,
            float(self.declare_parameter(
                'gait_state_stale_timeout', 1.0).value),
        )
        self.motor_tracking_good_residual_m = max(
            0.0,
            float(self.declare_parameter(
                'motor_tracking_good_residual_m', 0.005).value),
        )
        self.motor_tracking_reject_residual_m = max(
            self.motor_tracking_good_residual_m,
            float(self.declare_parameter(
                'motor_tracking_reject_residual_m', 0.03).value),
        )
        self.max_fit_residual_m = float(
            self.declare_parameter('max_fit_residual_m', 0.01).value)
        self.max_translation_step_m = float(
            self.declare_parameter('max_translation_step_m', 0.05).value)
        self.max_rotation_step_rad = float(
            self.declare_parameter('max_rotation_step_rad', 0.2).value)
        self.max_sample_dt = float(
            self.declare_parameter('max_sample_dt', 2.0).value)

        self.estimator = CommandedStanceOdometry(
            stance_value=CommandedGaitState.STANCE,
            min_common_stance=3,
            max_fit_residual_m=self.max_fit_residual_m,
            max_translation_step_m=self.max_translation_step_m,
            max_rotation_step_rad=self.max_rotation_step_rad,
            max_sample_dt=self.max_sample_dt,
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_mode = 'standby'
        self.geometry_confidence = 0.25

        self.last_motor_stamp_sec = None
        self.last_motor_tracking_residual_m = None
        self.last_motor_gait_sequence = None
        self.motor_tracking_accepted = False
        self.motor_future = None
        self.warned_motor_service = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None)
        self.motor_client = self.create_client(
            Trigger, self.motor_service_name)
        gait_state_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gait_state_sub = self.create_subscription(
            CommandedGaitState,
            self.gait_state_topic,
            self.gait_state_callback,
            gait_state_qos,
        )

        motor_period = 1.0 / max(self.motor_poll_rate, 0.1)
        self.motor_timer = self.create_timer(
            motor_period, self.poll_motor_angles)

        self.get_logger().info(
            f'Publishing motor-validated commanded-stance odometry on '
            f'{self.odom_topic} from {self.gait_state_topic}; calibrated '
            f'logical angles come from {self.motor_service_name}')

    def gait_state_callback(self, msg):
        valid_states = {
            CommandedGaitState.SWING,
            CommandedGaitState.STANCE,
            CommandedGaitState.UNKNOWN,
        }
        if len(msg.leg_state) != 6 or any(
                state not in valid_states for state in msg.leg_state):
            self.get_logger().warn(
                'Ignoring malformed commanded gait state',
                throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        stamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if stamp_sec <= 0.0:
            stamp_sec = now.nanoseconds * 1e-9
        gait_age = now.nanoseconds * 1e-9 - stamp_sec
        if (gait_age < -0.05
                or (self.gait_state_stale_timeout > 0.0
                    and gait_age > self.gait_state_stale_timeout)):
            self.estimator.reset()
            self.get_logger().warn(
                f'Ignoring gait state delayed by {gait_age:.3f} s',
                throttle_duration_sec=5.0)
            return

        observation = GaitObservation(
            sequence=int(msg.sequence),
            mode=msg.mode,
            phase_index=int(msg.phase_index),
            cycle_length=int(msg.cycle_length),
            stamp_sec=stamp_sec,
            leg_state=tuple(msg.leg_state),
            foot_x_m=tuple(value * 0.001 for value in msg.foot_x_mm),
            foot_y_m=tuple(value * 0.001 for value in msg.foot_y_mm),
        )
        try:
            increment = self.estimator.update(observation)
        except ValueError as exc:
            self.get_logger().warn(
                f'Ignoring invalid commanded gait observation: {exc}',
                throttle_duration_sec=5.0)
            return

        self.last_mode = msg.mode
        motor_confidence = self.motor_tracking_confidence(
            now, msg.sequence)
        if msg.mode == 'standby':
            self._set_zero_twist()
            self.geometry_confidence = 1.0
            if motor_confidence is not None:
                self.publish_odometry(now, motor_confidence)
            return
        if increment is None:
            self._set_zero_twist()
            self.geometry_confidence = 0.25
            return
        if motor_confidence is None:
            self._set_zero_twist()
            return

        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        self.x += increment.dx * cosine - increment.dy * sine
        self.y += increment.dx * sine + increment.dy * cosine
        self.yaw = self.normalize_angle(self.yaw + increment.dyaw)
        self.vx = increment.vx
        self.vy = increment.vy
        self.wz = increment.wz
        if self.max_fit_residual_m > 0.0:
            fit_ratio = increment.residual_m / self.max_fit_residual_m
            self.geometry_confidence = self.clamp(
                1.0 - fit_ratio, 0.25, 1.0)
        else:
            self.geometry_confidence = 1.0

        self.publish_odometry(
            now, min(self.geometry_confidence, motor_confidence))

    def poll_motor_angles(self):
        if self.motor_future is not None and not self.motor_future.done():
            return
        if not self.motor_client.service_is_ready():
            if not self.warned_motor_service:
                self.get_logger().warn(
                    f'Motor angle service {self.motor_service_name} is not '
                    f'ready; {self.odom_topic} will remain suppressed')
                self.warned_motor_service = True
            return

        self.motor_future = self.motor_client.call_async(Trigger.Request())
        self.motor_future.add_done_callback(self.handle_motor_angles)

    def handle_motor_angles(self, future):
        self.motor_tracking_accepted = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Motor angle service call failed: {exc}')
            return

        if not response.success:
            self.get_logger().warn(
                f'Motor angle service returned failure: {response.message}',
                throttle_duration_sec=5.0)
            return

        try:
            data = json.loads(response.message)
            residual_m, sequence = self.motor_tracking_residual(
                data, self.child_frame_id)
            sample_stamp_sec = self.motor_sample_stamp_sec(data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'Could not validate calibrated motor sample: {exc}',
                throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        sample_age = now.nanoseconds * 1e-9 - sample_stamp_sec
        if (sample_age < -0.05
                or sample_age > self.motor_stale_timeout):
            self.get_logger().warn(
                f'Motor sample age {sample_age:.3f} s is outside the valid '
                f'window; suppressing {self.odom_topic}',
                throttle_duration_sec=5.0)
            return

        self.last_motor_stamp_sec = sample_stamp_sec
        self.last_motor_tracking_residual_m = residual_m
        self.last_motor_gait_sequence = sequence
        self.motor_tracking_accepted = (
            residual_m <= self.motor_tracking_reject_residual_m)
        if not self.motor_tracking_accepted:
            self.get_logger().warn(
                f'Worst stance-foot FK residual {residual_m:.3f} m exceeds '
                f'{self.motor_tracking_reject_residual_m:.3f} m; '
                f'suppressing {self.odom_topic}',
                throttle_duration_sec=5.0)

    @staticmethod
    def motor_tracking_residual(data, expected_frame_id=None):
        """Return the worst synchronized stance-foot FK error and sequence."""
        if data.get('angle_space') != (
                'firmware_calibrated_logical_degrees'):
            raise ValueError('unexpected or missing motor angle space')

        gait_state = data.get('gait_state')
        if not isinstance(gait_state, dict):
            raise ValueError('missing synchronized gait state')
        if (expected_frame_id is not None
                and gait_state.get('frame_id') != expected_frame_id):
            raise ValueError('synchronized gait state has unexpected frame')

        target_axes = (
            gait_state.get('foot_x_mm', []),
            gait_state.get('foot_y_mm', []),
            gait_state.get('foot_z_mm', []),
        )
        leg_state = gait_state.get('leg_state', [])
        if any(len(axis) != 6 for axis in target_axes) or len(leg_state) != 6:
            raise ValueError('synchronized gait state requires six feet')
        target_points = tuple(zip(*target_axes))
        if any(
                not math.isfinite(float(value))
                for point in target_points for value in point):
            raise ValueError('commanded foot targets must be finite')

        stance_indices = [
            index for index, state in enumerate(leg_state)
            if int(state) == CommandedGaitState.STANCE
        ]
        if len(stance_indices) < 3:
            raise ValueError('motor validation requires three stance legs')
        if any(
                int(state) not in (
                    CommandedGaitState.SWING,
                    CommandedGaitState.STANCE,
                )
                for state in leg_state):
            raise ValueError('motor validation has invalid leg state')

        angles = data.get('angles', [])
        actual_vendor_mm = servo_angles_to_foot_positions(angles)
        actual_ros_mm = tuple(
            (vendor_y, -vendor_x, vendor_z)
            for vendor_x, vendor_y, vendor_z in actual_vendor_mm
        )
        stance_errors_mm = [
            math.sqrt(sum(
                (actual - float(target)) ** 2
                for actual, target in zip(
                    actual_ros_mm[index], target_points[index])
            ))
            for index in stance_indices
        ]

        sequence = int(gait_state.get('sequence'))
        if sequence < 0:
            raise ValueError('gait sequence must be non-negative')
        return max(stance_errors_mm) * 0.001, sequence

    @staticmethod
    def motor_sample_stamp_sec(data):
        """Return and validate the motor sample's source timestamp."""
        stamp = data.get('sample_stamp')
        if not isinstance(stamp, dict):
            raise ValueError('missing motor sample timestamp')
        sec = int(stamp.get('sec'))
        nanosec = int(stamp.get('nanosec'))
        if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
            raise ValueError('invalid motor sample timestamp')
        result = float(sec) + float(nanosec) * 1e-9
        if result <= 0.0 or not math.isfinite(result):
            raise ValueError('invalid motor sample timestamp')
        return result

    def motor_tracking_confidence(self, now, gait_sequence):
        if (not self.motor_tracking_accepted
                or self.last_motor_stamp_sec is None):
            return None
        if int(gait_sequence) < self.last_motor_gait_sequence:
            return None
        motor_age = now.nanoseconds * 1e-9 - self.last_motor_stamp_sec
        if motor_age < -0.05 or motor_age > self.motor_stale_timeout:
            return None

        residual = self.last_motor_tracking_residual_m
        good = self.motor_tracking_good_residual_m
        reject = self.motor_tracking_reject_residual_m
        if residual <= good or reject <= good:
            return 1.0
        ratio = (residual - good) / (reject - good)
        return self.clamp(1.0 - 0.75 * ratio, 0.25, 1.0)

    def publish_odometry(self, now, confidence):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz

        multiplier = 1.0 / max(confidence, 0.05)
        self.set_covariance(
            msg.pose.covariance,
            0.5 * multiplier,
            0.5 * multiplier,
            0.8 * multiplier,
        )
        self.set_covariance(
            msg.twist.covariance,
            0.2 * multiplier,
            0.2 * multiplier,
            0.4 * multiplier,
        )
        self.odom_pub.publish(msg)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _set_zero_twist(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    @staticmethod
    def set_covariance(covariance, x_var, y_var, yaw_var):
        for index in range(36):
            covariance[index] = 0.0
        covariance[0] = x_var
        covariance[7] = y_var
        covariance[14] = 999.0
        covariance[21] = 999.0
        covariance[28] = 999.0
        covariance[35] = yaw_var

    @staticmethod
    def yaw_to_quaternion(yaw):
        from geometry_msgs.msg import Quaternion

        quaternion = Quaternion()
        half_yaw = yaw * 0.5
        quaternion.z = math.sin(half_yaw)
        quaternion.w = math.cos(half_yaw)
        return quaternion

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))


def main():
    rclpy.init()
    node = FootOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
