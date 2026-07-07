#!/usr/bin/env python3
# encoding: utf-8

import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class FootOdometryNode(Node):
    """Rough dead-reckoned odometry from Muto gait commands.

    This is not contact-sensed foot odometry. It mirrors the Muto driver gait
    selection from cmd_vel, watches reported servo angles for motion evidence,
    and publishes a deliberately high-covariance odometry estimate.
    """

    def __init__(self):
        super().__init__("foot_odometry_node")

        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "cmd_vel").value
        self.motor_service_name = self.declare_parameter(
            "motor_service_name", "get_motor_angles").value
        self.odom_topic = self.declare_parameter("odom_topic", "/foot_odom").value
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        self.child_frame_id = self.declare_parameter("child_frame_id", "base_frame").value
        self.publish_tf = self.declare_parameter("publish_tf", False).value

        self.update_rate = float(self.declare_parameter("update_rate", 30.0).value)
        self.motor_poll_rate = float(self.declare_parameter("motor_poll_rate", 2.0).value)
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.5).value)
        self.motor_stale_timeout = float(self.declare_parameter("motor_stale_timeout", 1.0).value)
        self.motor_motion_deadband_deg = float(
            self.declare_parameter("motor_motion_deadband_deg", 1.0).value)

        self.speed_scale = float(self.declare_parameter("speed_scale", 100.0).value)
        self.max_level = float(self.declare_parameter("max_level", 30.0).value)
        self.min_turn_level = float(self.declare_parameter("min_turn_level", 10.0).value)
        self.max_turn_level = float(self.declare_parameter("max_turn_level", 20.0).value)
        self.level_to_linear_velocity = float(
            self.declare_parameter("level_to_linear_velocity", 0.01).value)
        self.level_to_angular_velocity = float(
            self.declare_parameter("level_to_angular_velocity", 0.01).value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.level_x = 0.0
        self.level_y = 0.0
        self.level_z = 0.0
        self.last_cmd_time = None
        self.last_update_time = self.get_clock().now()

        self.last_motor_angles = None
        self.last_motor_stamp = None
        self.last_motor_mean_delta = 0.0
        self.motor_future = None
        self.warned_motor_service = False
        self.last_mixed_warning_time = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.motor_client = self.create_client(Trigger, self.motor_service_name)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)

        update_period = 1.0 / max(self.update_rate, 1.0)
        motor_period = 1.0 / max(self.motor_poll_rate, 0.1)
        self.update_timer = self.create_timer(update_period, self.update)
        self.motor_timer = self.create_timer(motor_period, self.poll_motor_angles)

        self.get_logger().info(
            f"Publishing rough gait odometry on {self.odom_topic} from {self.cmd_vel_topic}; "
            f"motor service {self.motor_service_name}")

    def cmd_vel_callback(self, msg):
        self.level_x = self.clamp(msg.linear.x * self.speed_scale, -self.max_level, self.max_level)
        self.level_y = self.clamp(msg.linear.y * self.speed_scale, -self.max_level, self.max_level)
        self.level_z = self.clamp(msg.angular.z * self.speed_scale, -self.max_level, self.max_level)

        if self.level_z != 0.0:
            sign = 1.0 if self.level_z > 0.0 else -1.0
            abs_level = abs(self.level_z)
            abs_level = self.clamp(abs_level, self.min_turn_level, self.max_turn_level)
            self.level_z = sign * abs_level

        self.last_cmd_time = self.get_clock().now()

    def poll_motor_angles(self):
        if self.motor_future is not None and not self.motor_future.done():
            return
        if not self.motor_client.service_is_ready():
            if not self.warned_motor_service:
                self.get_logger().warn(
                    f"Motor angle service {self.motor_service_name} is not ready; "
                    "foot odometry will use command-only confidence")
                self.warned_motor_service = True
            return

        self.motor_future = self.motor_client.call_async(Trigger.Request())
        self.motor_future.add_done_callback(self.handle_motor_angles)

    def handle_motor_angles(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Motor angle service call failed: {exc}")
            return

        if not response.success:
            self.get_logger().warn(f"Motor angle service returned failure: {response.message}")
            return

        try:
            data = json.loads(response.message)
            angles = [float(angle) for angle in data.get("angles", [])]
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Could not parse motor angle response: {exc}")
            return

        if not angles:
            return

        if self.last_motor_angles and len(self.last_motor_angles) == len(angles):
            deltas = [abs(a - b) for a, b in zip(angles, self.last_motor_angles)]
            self.last_motor_mean_delta = sum(deltas) / len(deltas)
        else:
            self.last_motor_mean_delta = 0.0

        self.last_motor_angles = angles
        self.last_motor_stamp = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        vx, vy, wz, gait_name = self.estimate_body_twist(now)

        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw = self.normalize_angle(self.yaw + wz * dt)

        confidence = self.motor_confidence(now, vx, vy, wz)
        self.publish_odometry(now, vx, vy, wz, confidence)

        if gait_name == "mixed_y_ignored":
            if self.last_mixed_warning_time is None or (
                now - self.last_mixed_warning_time).nanoseconds * 1e-9 > 2.0:
                self.get_logger().warn(
                    "cmd_vel has y and z, but Muto gen_move2 uses x/z; "
                    "rough foot odom is suppressing that motion")
                self.last_mixed_warning_time = now

    def estimate_body_twist(self, now):
        if self.last_cmd_time is None:
            return 0.0, 0.0, 0.0, "standby"
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            return 0.0, 0.0, 0.0, "standby"

        lx = self.level_x
        ly = self.level_y
        lz = self.level_z
        if lx == 0.0 and ly == 0.0 and lz == 0.0:
            return 0.0, 0.0, 0.0, "standby"

        if lz == 0.0:
            if lx != 0.0:
                return lx * self.level_to_linear_velocity, 0.0, 0.0, "move_x"
            if ly != 0.0:
                return 0.0, ly * self.level_to_linear_velocity, 0.0, "move_y"
            return 0.0, 0.0, 0.0, "standby"

        if lx == 0.0 and ly == 0.0:
            return 0.0, 0.0, lz * self.level_to_angular_velocity, "turn"
        if lx == 0.0 and ly != 0.0:
            return 0.0, 0.0, 0.0, "mixed_y_ignored"

        return (
            lx * self.level_to_linear_velocity,
            0.0,
            lz * self.level_to_angular_velocity,
            "mixed_x_z",
        )

    def motor_confidence(self, now, vx, vy, wz):
        moving_command = abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6
        if not moving_command:
            return 1.0
        if self.last_motor_stamp is None:
            return 0.5

        motor_age = (now - self.last_motor_stamp).nanoseconds * 1e-9
        if motor_age > self.motor_stale_timeout:
            return 0.35
        if self.last_motor_mean_delta >= self.motor_motion_deadband_deg:
            return 1.0
        return 0.25

    def publish_odometry(self, now, vx, vy, wz, confidence):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz

        multiplier = 1.0 / max(confidence, 0.05)
        self.set_covariance(msg.pose.covariance, 0.5 * multiplier, 0.5 * multiplier, 0.8 * multiplier)
        self.set_covariance(msg.twist.covariance, 0.2 * multiplier, 0.2 * multiplier, 0.4 * multiplier)

        self.odom_pub.publish(msg)

        if self.tf_broadcaster:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def set_covariance(covariance, x_var, y_var, yaw_var):
        for i in range(36):
            covariance[i] = 0.0
        covariance[0] = x_var
        covariance[7] = y_var
        covariance[14] = 999.0
        covariance[21] = 999.0
        covariance[28] = 999.0
        covariance[35] = yaw_var

    @staticmethod
    def yaw_to_quaternion(yaw):
        from geometry_msgs.msg import Quaternion

        q = Quaternion()
        half_yaw = yaw * 0.5
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
