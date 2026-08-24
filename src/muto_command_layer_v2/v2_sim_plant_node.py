#!/usr/bin/env python3
"""Small reactive 2-D plant for v2/Nav2 validation.

This is an environment fixture, not a command-layer backend.  It consumes the
normal Nav2 ``/cmd_vel`` output, integrates a bounded planar robot, and
publishes the map, odometry, TF, clock, and LiDAR topics expected by the
existing Nav2 pipeline.  It has no dependency on the legacy command layer.
"""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class V2SimPlant(Node):
    def __init__(self) -> None:
        super().__init__("muto_command_layer_v2_sim_plant")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("map_width", 80)
        self.declare_parameter("map_height", 80)
        self.declare_parameter("map_resolution", 0.1)
        self.declare_parameter("map_origin_x", -4.0)
        self.declare_parameter("map_origin_y", -4.0)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("robot_radius_m", 0.26)
        self.declare_parameter("obstacles_json", "[]")

        self._rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self._dt = 1.0 / self._rate_hz
        self._width = int(self.get_parameter("map_width").value)
        self._height = int(self.get_parameter("map_height").value)
        self._resolution = float(self.get_parameter("map_resolution").value)
        self._origin_x = float(self.get_parameter("map_origin_x").value)
        self._origin_y = float(self.get_parameter("map_origin_y").value)
        self._radius = float(self.get_parameter("robot_radius_m").value)
        self._x = float(self.get_parameter("start_x").value)
        self._y = float(self.get_parameter("start_y").value)
        self._yaw = float(self.get_parameter("start_yaw").value)
        self._linear = 0.0
        self._angular = 0.0
        self._sim_time = 0.0
        self._obstacles = self._parse_obstacles(
            str(self.get_parameter("obstacles_json").value)
        )

        self._clock_pub = self.create_publisher(Clock, "/clock", 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self._odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 20)
        self._scan_pub = self.create_publisher(
            LaserScan, "/lidar/filtered_laserscan", qos_profile_sensor_data
        )
        self._scan_full_pub = self.create_publisher(
            LaserScan,
            "/lidar/filtered_laserscan_no_downsample",
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel, 20)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._map = self._build_map()
        self._publish_static_tf()
        self._timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info("v2 reactive simulation plant ready")

    def _parse_obstacles(self, encoded: str):
        values = json.loads(encoded)
        if not isinstance(values, list):
            raise ValueError("obstacles_json must be a list")
        result = []
        for value in values:
            if not isinstance(value, list) or len(value) != 4:
                raise ValueError("each obstacle must be [xmin, xmax, ymin, ymax]")
            xmin, xmax, ymin, ymax = (float(item) for item in value)
            if xmin >= xmax or ymin >= ymax:
                raise ValueError("obstacle bounds must be increasing")
            result.append((xmin, xmax, ymin, ymax))
        return tuple(result)

    def _stamp(self):
        stamp = Clock().clock
        whole = int(self._sim_time)
        stamp.sec = whole
        stamp.nanosec = int((self._sim_time - whole) * 1e9)
        return stamp

    def _build_map(self):
        values = [0] * (self._width * self._height)
        for row in range(self._height):
            for col in range(self._width):
                if row in (0, self._height - 1) or col in (0, self._width - 1):
                    values[row * self._width + col] = 100
                    continue
                x = self._origin_x + (col + 0.5) * self._resolution
                y = self._origin_y + (row + 0.5) * self._resolution
                if any(xmin <= x <= xmax and ymin <= y <= ymax
                       for xmin, xmax, ymin, ymax in self._obstacles):
                    values[row * self._width + col] = 100
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info.resolution = self._resolution
        message.info.width = self._width
        message.info.height = self._height
        message.info.origin.position.x = self._origin_x
        message.info.origin.position.y = self._origin_y
        message.info.origin.orientation.w = 1.0
        message.data = values
        return message

    def _publish_static_tf(self):
        stamp = self._stamp()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        sensor = TransformStamped()
        sensor.header.stamp = stamp
        sensor.header.frame_id = "base_frame"
        sensor.child_frame_id = "lidar_link"
        sensor.transform.translation.z = 0.2
        sensor.transform.rotation.w = 1.0
        # Publish both static links in one latched TF message.  Sending them
        # in separate calls would leave only the second transform in the
        # transient-local cache, making ``map -> base_frame`` unavailable to
        # late-joining Nav2/readiness clients.
        self._static_tf.sendTransform([transform, sensor])

    def _cmd_vel(self, message: Twist):
        self._linear = max(-0.25, min(0.25, float(message.linear.x)))
        self._angular = max(-0.8, min(0.8, float(message.angular.z)))

    def _tick(self):
        self._sim_time += self._dt
        next_yaw = self._wrap(self._yaw + self._angular * self._dt)
        next_x = self._x + self._linear * math.cos(next_yaw) * self._dt
        next_y = self._y + self._linear * math.sin(next_yaw) * self._dt
        if self._free(next_x, next_y):
            self._x, self._y = next_x, next_y
        else:
            self._linear = 0.0
        self._yaw = next_yaw
        stamp = self._stamp()
        clock = Clock()
        clock.clock = stamp
        self._clock_pub.publish(clock)
        self._map.header.stamp = stamp
        self._map_pub.publish(self._map)
        self._publish_odom(stamp)
        scan = self._scan(stamp)
        self._scan_pub.publish(scan)
        self._scan_full_pub.publish(scan)

    def _free(self, x: float, y: float) -> bool:
        left = self._origin_x + self._radius
        right = self._origin_x + self._width * self._resolution - self._radius
        bottom = self._origin_y + self._radius
        top = self._origin_y + self._height * self._resolution - self._radius
        if not (left <= x <= right and bottom <= y <= top):
            return False
        return not any(
            xmin - self._radius <= x <= xmax + self._radius
            and ymin - self._radius <= y <= ymax + self._radius
            for xmin, xmax, ymin, ymax in self._obstacles
        )

    def _publish_odom(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_frame"
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = math.sin(self._yaw / 2.0)
        transform.transform.rotation.w = math.cos(self._yaw / 2.0)
        self._tf.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_frame"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = transform.transform.rotation.z
        odom.pose.pose.orientation.w = transform.transform.rotation.w
        odom.twist.twist.linear.x = self._linear
        odom.twist.twist.angular.z = self._angular
        self._odom_pub.publish(odom)

    def _scan(self, stamp):
        count = 360
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "lidar_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / count
        scan.range_min = 0.05
        scan.range_max = 8.0
        scan.ranges = [self._ray(scan.angle_min + index * scan.angle_increment)
                       for index in range(count)]
        return scan

    def _ray(self, angle: float) -> float:
        angle += self._yaw
        dx, dy = math.cos(angle), math.sin(angle)
        distances = []
        if abs(dx) > 1e-9:
            for x in (self._origin_x, self._origin_x + self._width * self._resolution):
                t = (x - self._x) / dx
                if t > 0:
                    distances.append(t)
        if abs(dy) > 1e-9:
            for y in (self._origin_y, self._origin_y + self._height * self._resolution):
                t = (y - self._y) / dy
                if t > 0:
                    distances.append(t)
        for xmin, xmax, ymin, ymax in self._obstacles:
            for x in (xmin, xmax):
                if abs(dx) > 1e-9:
                    t = (x - self._x) / dx
                    y = self._y + t * dy
                    if t > 0 and ymin <= y <= ymax:
                        distances.append(t)
            for y in (ymin, ymax):
                if abs(dy) > 1e-9:
                    t = (y - self._y) / dy
                    x = self._x + t * dx
                    if t > 0 and xmin <= x <= xmax:
                        distances.append(t)
        return max(0.05, min(8.0, min(distances) if distances else 8.0))

    @staticmethod
    def _wrap(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))


def main(args=None):
    rclpy.init(args=args)
    node = V2SimPlant()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
