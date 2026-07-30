#!/usr/bin/env python
# encoding: utf-8

import json

from muto_hexapod_lib_custom.core.MutoLibCore import Muto
from muto_hexapod_lib_custom.core.config import (
	STANDBY_SERVO_ANGLES_DEG,
)
from muto_hexapod_interfaces_custom.msg import CommandedGaitState
from yahboomcar_imu.imu_node import ImuPublisher


#ros lib
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)

		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_buzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,1)
		self.srv_motor_angles = self.create_service(Trigger, "get_motor_angles", self.get_motor_angles_callback)
		self.srv_release_motors = self.create_service(Trigger, "release_motors", self.release_motors_callback)
		
		self.gait_state_topic = self.declare_parameter(
			"gait_state_topic", "/muto/commanded_gait_state").value
		self.gait_state_frame_id = self.declare_parameter(
			"gait_state_frame_id", "base_frame").value
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

		self.muto = Muto(gait_step_callback=self.publish_commanded_gait_state)
		self.publish_commanded_gait_state(self.muto.commanded_gait_state)

		self.vel_x = 0.0
		self.vel_y = 0.0
		self.angular_z = 0.0
		
		# 速度映射参数，与MutoLibCore保持一致
		# Speed mapping parameters, consistent with MutoLibCore
		# MutoLibCore使用: 档位(-30~30) -> 真实速度(m/s), 默认每档位0.01m/s
		# MutoLibCore uses: level(-30~30) -> real speed(m/s), default 0.01m/s per level
		self.speed_scale = 100.0  # 将m/s转换为档位的缩放因子 (1/0.01 = 100)

		self.declare_parameter("imu_link", "imu_link")
		imu_link = self.get_parameter("imu_link").get_parameter_value().string_value
		self.declare_parameter("imu_publish_rate_hz", 50.0)
		imu_publish_rate_hz = self.get_parameter("imu_publish_rate_hz").get_parameter_value().double_value
		if imu_publish_rate_hz <= 0.0:
			self.get_logger().warn("imu_publish_rate_hz must be positive; using 50.0")
			imu_publish_rate_hz = 50.0

		self.imu = ImuPublisher(self, self.muto, imu_link)
		self.imu_timer = self.create_timer(1.0 / imu_publish_rate_hz, self.imu.publish_imu_data)
		self.get_logger().info("IMU publish rate set to {:.1f} Hz".format(imu_publish_rate_hz))

	def publish_commanded_gait_state(self, state):
		"""Publish nominal gait support; this is not measured foot contact."""
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
		self.gait_state_pub.publish(msg)

	def cmd_vel_callback(self,msg):
		if not isinstance(msg, Twist): return
		
		# 获取ROS2的速度命令 (单位: m/s 和 rad/s)
		# Get ROS2 velocity commands (units: m/s and rad/s)
		ros_vel_x = msg.linear.x
		ros_vel_y = msg.linear.y
		ros_angular_z = msg.angular.z
		
		# 转换为MutoLibCore的档位系统
		# Convert to MutoLibCore level system
		# 速度映射: m/s -> 档位 (每档位0.01m/s)
		# Speed mapping: m/s -> level (0.01m/s per level)
		self.vel_x = ros_vel_x * self.speed_scale
		self.vel_y = ros_vel_y * self.speed_scale
		self.angular_z = ros_angular_z * self.speed_scale
		
		# 限制档位范围到[-30, 30]
		# Limit level range to [-30, 30]
		self.vel_x = max(-30, min(30, self.vel_x))
		self.vel_y = max(-30, min(30, self.vel_y))
		self.angular_z = max(-30, min(30, self.angular_z))
		
		# 角速度特殊处理：确保在有效范围[10-30]内，或为0
		# Special handling for angular velocity: ensure within valid range [10-30] or 0
		if self.angular_z != 0:
			if abs(self.angular_z) < 10:
				self.angular_z = 10 if self.angular_z > 0 else -10
			elif abs(self.angular_z) > 30:
				self.angular_z = 30 if self.angular_z > 0 else -30
		
		self.get_logger().info("ROS vel: x={:.3f}, y={:.3f}, z={:.3f} -> Muto levels: x={:.1f}, y={:.1f}, z={:.1f}".format(
			ros_vel_x, ros_vel_y, ros_angular_z, self.vel_x, self.vel_y, self.angular_z))
		
		# 发送到MutoLibCore
		# Send to MutoLibCore
		self.muto.move(self.vel_x, self.vel_y, self.angular_z)

	def get_motor_angles_callback(self, request, response):
		del request
		try:
			angles = self.muto.read_motor()
		except Exception as exc:
			response.success = False
			response.message = json.dumps({
				"error": "read_motor_failed",
				"detail": str(exc)
			})
			return response

		if not angles or len(angles) != 18:
			response.success = False
			response.message = json.dumps({
				"error": "invalid_motor_angle_data",
				"angles": angles or [],
				"expected_count": 18,
			})
			return response

		state = self.muto.commanded_gait_state
		stamp = self.get_clock().now().to_msg()
		leg_state = [
			CommandedGaitState.STANCE if in_stance
			else CommandedGaitState.SWING
			for in_stance in state.commanded_stance
		]
		response.success = True
		response.message = json.dumps({
			"count": len(angles),
			"angles": angles,
			"angle_space": "firmware_calibrated_logical_degrees",
			"standby_leg_angles_deg": list(STANDBY_SERVO_ANGLES_DEG),
			"sample_stamp": {
				"sec": stamp.sec,
				"nanosec": stamp.nanosec,
			},
			"gait_state": {
				"frame_id": self.gait_state_frame_id,
				"sequence": state.sequence,
				"mode": state.mode,
				"phase_index": state.phase_index,
				"cycle_length": state.cycle_length,
				"leg_state": leg_state,
				"foot_x_mm": [
					point[1] for point in state.foot_positions_mm
				],
				"foot_y_mm": [
					-point[0] for point in state.foot_positions_mm
				],
				"foot_z_mm": [
					point[2] for point in state.foot_positions_mm
				],
			},
			"servo_angles": {
				str(index + 1): angle for index, angle in enumerate(angles)
			},
		})
		return response
	
	def release_motors_callback(self, request, response):
		del request
		try:
			self.vel_x = 0.0
			self.vel_y = 0.0
			self.angular_z = 0.0
			self.motion_command_time_sec = None
			for servo_id in range(1, 19):
				self.muto.Servo_torque_off(servo_id)
		except Exception as exc:
			response.success = False
			response.message = json.dumps({
				"error": "release_motors_failed",
				"detail": str(exc)
			})
			return response

		response.success = True
		response.message = json.dumps({
			"released": True,
			"servo_ids": list(range(1, 19)),
			"detail": "Torque disabled for all joint servos"
		})
		return response
	
	#控制蜂鸣器
	#Control buzzer
	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			# 255表示一直响
			for i in range(3): 
				self.muto.buzzer(255)
		else:
			for i in range(3): 
				self.muto.buzzer(0)

	def destroy_node(self):
		try:
			self.muto.close()
		finally:
			return super().destroy_node()
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)
	driver.destroy_node()
	rclpy.shutdown()

		
		
