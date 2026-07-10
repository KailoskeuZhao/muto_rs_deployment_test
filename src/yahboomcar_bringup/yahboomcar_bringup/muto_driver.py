#!/usr/bin/env python
# encoding: utf-8

import json

#public lib
#from MutoLib import Muto 
from muto_hexapod_lib.core.MutoLibCore import Muto
from yahboomcar_imu.imu_node import ImuPublisher


#ros lib
import rclpy
from rclpy.node import Node
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
		
		self.muto = Muto()

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
		
		# 角速度特殊处理：确保在有效范围[10-20]内，或为0
		# Special handling for angular velocity: ensure within valid range [10-20] or 0
		if self.angular_z != 0:
			if abs(self.angular_z) < 10:
				self.angular_z = 10 if self.angular_z > 0 else -10
			elif abs(self.angular_z) > 20:
				self.angular_z = 20 if self.angular_z > 0 else -20
		
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

		if not angles:
			response.success = False
			response.message = json.dumps({
				"error": "no_motor_angle_data",
				"angles": []
			})
			return response

		response.success = True
		response.message = json.dumps({
			"count": len(angles),
			"angles": angles,
			"servo_angles": {
				str(index + 1): angle for index, angle in enumerate(angles)
			}
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
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)
	driver.destroy_node()
	rclpy.shutdown()

		
		
