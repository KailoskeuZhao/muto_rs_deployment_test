#!/usr/bin/env python3
"""ROS hardware driver for the Muto base, IMU, and locomotion state."""

import json
import time

from geometry_msgs.msg import Twist
from muto_hexapod_interfaces_custom.msg import CommandedGaitState
from muto_hexapod_lib_custom.core.config import STANDBY_SERVO_ANGLES_DEG
from muto_hexapod_lib_custom.core.MutoLibCore import Muto
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from yahboomcar_imu.imu_node import ImuPublisher


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
        if self.locomotion_update_rate_hz <= 0.0:
            self.get_logger().warn(
                'locomotion_update_rate_hz must be positive; using 50.0')
            self.locomotion_update_rate_hz = 50.0
        self.cmd_vel_timeout = max(
            0.0,
            float(self.declare_parameter('cmd_vel_timeout', 0.5).value),
        )

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

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.angular_z = 0.0
        self.speed_scale = 100.0
        self.desired_motion_levels = (0.0, 0.0, 0.0)
        self.last_cmd_vel_monotonic = None
        self.last_gait_command_monotonic = None
        self.last_locomotion_tick_monotonic = None
        self.cmd_vel_timed_out = False
        self.motors_released = False

        self.muto = Muto(gait_step_callback=self.publish_commanded_gait_state)
        self.muto.set_motion_command(0.0, 0.0, 0.0)

        self.declare_parameter('imu_link', 'imu_link')
        imu_link = self.get_parameter(
            'imu_link').get_parameter_value().string_value
        self.declare_parameter('imu_publish_rate_hz', 50.0)
        imu_publish_rate_hz = self.get_parameter(
            'imu_publish_rate_hz').get_parameter_value().double_value
        if imu_publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'imu_publish_rate_hz must be positive; using 50.0')
            imu_publish_rate_hz = 50.0

        # IMU calibration assumes a still robot. Enforce standby immediately
        # after calibration and before ROS timers begin dispatching callbacks.
        self.imu = ImuPublisher(self, self.muto, imu_link)
        self.update_locomotion()
        self.locomotion_timer = self.create_timer(
            1.0 / self.locomotion_update_rate_hz,
            self.update_locomotion,
        )
        self.imu_timer = self.create_timer(
            1.0 / imu_publish_rate_hz,
            self.imu.publish_imu_data,
        )
        self.get_logger().info(
            f'IMU publish rate set to {imu_publish_rate_hz:.1f} Hz')
        self.get_logger().info(
            'Locomotion phase loop set to '
            f'{self.locomotion_update_rate_hz:.1f} Hz with '
            f'{self.cmd_vel_timeout:.2f} s cmd_vel timeout')

    def update_locomotion(self):
        """Advance one gait phase from the most recent velocity command."""
        if self.motors_released:
            return

        now_monotonic = time.monotonic()
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
            (0.0, 0.0, 0.0)
            if command_stale else self.desired_motion_levels
        )
        if command_stale and not self.cmd_vel_timed_out:
            if self.last_cmd_vel_monotonic is not None:
                self.get_logger().warn(
                    'cmd_vel timed out; returning locomotion to standby')
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
        self.gait_state_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        """Latch desired levels; the locomotion timer performs all actuation."""
        if not isinstance(msg, Twist):
            return
        if self.motors_released:
            self.get_logger().warn(
                'Ignoring cmd_vel because joint torque was released',
                throttle_duration_sec=5.0,
            )
            return

        self.vel_x = max(-30, min(30, msg.linear.x * self.speed_scale))
        self.vel_y = max(-30, min(30, msg.linear.y * self.speed_scale))
        # The vendor gait accepts angular levels only in [-20, 20]. Clamp at
        # the ROS boundary as well so the reported/latching value is the value
        # the library actually executes instead of relying on hidden clipping.
        self.angular_z = max(
            -20, min(20, msg.angular.z * self.speed_scale))
        if 0.0 < abs(self.angular_z) < 10.0:
            self.angular_z = 10 if self.angular_z > 0.0 else -10

        self.desired_motion_levels = (
            self.vel_x, self.vel_y, self.angular_z)
        self.last_cmd_vel_monotonic = time.monotonic()

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
        # 50 Hz gait and IMU timers. Never sleep here to let a target settle:
        # doing so pauses both real-time loops. The motor residual must reflect
        # tracking during the normal moving gait, not an artificial hold.
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
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.angular_z = 0.0
        self.desired_motion_levels = (0.0, 0.0, 0.0)
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
