import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu


def clamp_dt(dt, max_dt):
    if dt < 0.0:
        return 0.0
    if dt > max_dt:
        return max_dt
    return dt


def normalize_quaternion(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12 or not math.isfinite(norm):
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def multiply_quaternions(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def delta_quaternion_from_gyro(gx, gy, gz, dt):
    angle = math.sqrt(gx * gx + gy * gy + gz * gz) * dt
    if angle <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0

    axis_scale = math.sin(angle * 0.5) / angle
    return (
        gx * dt * axis_scale,
        gy * dt * axis_scale,
        gz * dt * axis_scale,
        math.cos(angle * 0.5),
    )


def euler_from_quaternion(q):
    x, y, z, w = q

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi * 0.5, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GyroOrientationTestNode(Node):
    def __init__(self):
        super().__init__("gyro_orientation_test_node")

        self.input_topic = self.declare_parameter("input_topic", "/imu/data_processed").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/imu/orientation_gyro_test"
        ).value
        self.euler_topic = self.declare_parameter(
            "euler_topic", "/imu/orientation_gyro_test_euler"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "imu_link").value
        self.gyro_bias_x = float(self.declare_parameter("gyro_bias_x", 0.0).value)
        self.gyro_bias_y = float(self.declare_parameter("gyro_bias_y", 0.0).value)
        self.gyro_bias_z = float(self.declare_parameter("gyro_bias_z", 0.0).value)
        self.max_dt = float(self.declare_parameter("max_dt", 0.2).value)
        self.orientation_covariance = float(
            self.declare_parameter("orientation_covariance", 999.0).value
        )

        self.normalize_parameters()

        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.last_stamp = None

        self.publisher = self.create_publisher(Imu, self.output_topic, 10)
        self.euler_publisher = self.create_publisher(Vector3Stamped, self.euler_topic, 10)
        self.subscriber = self.create_subscription(
            Imu, self.input_topic, self.imu_callback, 10
        )

        self.get_logger().info(
            f"Integrating angular_velocity only from {self.input_topic} -> {self.output_topic} "
            f"and {self.euler_topic}; "
            "orientation will drift without accel/magnetometer correction"
        )

    def normalize_parameters(self):
        if self.max_dt <= 0.0:
            self.get_logger().warn("max_dt must be positive; using 0.2")
            self.max_dt = 0.2
        if self.orientation_covariance < 0.0:
            self.get_logger().warn("orientation_covariance must be non-negative; using 999.0")
            self.orientation_covariance = 999.0

    def imu_callback(self, msg):
        stamp = self.message_time(msg)

        if self.last_stamp is not None:
            dt = clamp_dt((stamp - self.last_stamp).nanoseconds * 1e-9, self.max_dt)
            gx = msg.angular_velocity.x - self.gyro_bias_x
            gy = msg.angular_velocity.y - self.gyro_bias_y
            gz = msg.angular_velocity.z - self.gyro_bias_z

            if all(math.isfinite(value) for value in (gx, gy, gz)):
                delta = delta_quaternion_from_gyro(gx, gy, gz, dt)
                self.orientation = normalize_quaternion(
                    multiply_quaternions(self.orientation, delta)
                )
            else:
                self.get_logger().warn(
                    "Processed IMU angular_velocity contains non-finite values",
                    throttle_duration_sec=2.0,
                )

        self.last_stamp = stamp
        self.publish_orientation(msg, stamp)
        self.publish_euler(msg, stamp)

    def message_time(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return self.get_clock().now()
        return Time.from_msg(msg.header.stamp)

    def publish_orientation(self, input_msg, stamp):
        output = Imu()
        output.header.stamp = stamp.to_msg()
        output.header.frame_id = self.frame_id or input_msg.header.frame_id

        output.orientation.x = self.orientation[0]
        output.orientation.y = self.orientation[1]
        output.orientation.z = self.orientation[2]
        output.orientation.w = self.orientation[3]

        for index in (0, 4, 8):
            output.orientation_covariance[index] = self.orientation_covariance
        output.angular_velocity_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0

        self.publisher.publish(output)

    def publish_euler(self, input_msg, stamp):
        roll, pitch, yaw = euler_from_quaternion(self.orientation)

        output = Vector3Stamped()
        output.header.stamp = stamp.to_msg()
        output.header.frame_id = self.frame_id or input_msg.header.frame_id
        output.vector.x = roll
        output.vector.y = pitch
        output.vector.z = yaw

        self.euler_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GyroOrientationTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
