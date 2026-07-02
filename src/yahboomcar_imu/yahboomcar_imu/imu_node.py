# imu_node.py
import math

from sensor_msgs.msg import Imu

LSB_PER_DPS = 131.0  # approx for ±250 dps

class ImuPublisher:
    def __init__(self, node, muto, imu_link="imu_link"):
        self.node = node
        self.muto = muto
        self.imu_link = imu_link
        self.publisher = node.create_publisher(Imu, "/imu/data_raw",100)
        self.publisher_1 = node.create_publisher(Imu, "/imu/data_processed", 100)

    def publish_imu_data(self):
        imu = Imu()
        ax, ay, az, gx, gy, gz, _, _, _ = self.muto.read_IMU_Raw()

        stamp = self.node.get_clock().now().to_msg()
        
        imu.header.stamp = stamp
        imu.header.frame_id = "raw_imu_link"
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0

        imu.orientation_covariance[0] = -1  # Orientation not provided

        self.publisher.publish(imu)

        imu2 = Imu()
        imu2.header.stamp = stamp
        imu2.header.frame_id = self.imu_link
        imu2.linear_acceleration.x = ax * 9.8 / 8500.0
        imu2.linear_acceleration.y = ay * 9.8 / 8500.0
        imu2.linear_acceleration.z = az * 9.8 / 8500.0
        imu2.angular_velocity.x = (gx -31.0) / LSB_PER_DPS * math.pi / 180.0
        imu2.angular_velocity.y = (gy +1.0) / LSB_PER_DPS * math.pi / 180.0
        imu2.angular_velocity.z = (gz +17.0) / LSB_PER_DPS * math.pi / 180.0

        imu2.orientation_covariance[0] = -1  # Orientation not provided

        self.publisher_1.publish(imu2)
