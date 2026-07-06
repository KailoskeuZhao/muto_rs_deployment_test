# imu_node.py
import math

from sensor_msgs.msg import Imu, MagneticField

LSB_PER_DPS = 131.0  # approx for ±250 dps
ANGULAR_VELOCITY_COVARIANCE = 6.9e-6
LINEAR_ACCELERATION_COVARIANCE = 5.1e-4
MAGNETIC_FIELD_COVARIANCE = 3.0e-4
RAW_IMU_FRAME = "raw_imu_link"


def set_imu_covariance(imu):
    imu.orientation_covariance[0] = -1  # Orientation not provided

    for index in (0, 4, 8):
        imu.angular_velocity_covariance[index] = ANGULAR_VELOCITY_COVARIANCE
        imu.linear_acceleration_covariance[index] = LINEAR_ACCELERATION_COVARIANCE


def set_magnetic_field_covariance(mag):
    for index in (0, 4, 8):
        mag.magnetic_field_covariance[index] = MAGNETIC_FIELD_COVARIANCE


def read_imu_raw(node, muto):
    data = muto.read_IMU_Raw()
    if data is None:
        node.get_logger().warn("IMU raw read returned no data", throttle_duration_sec=5.0)
        return None
    if len(data) < 9:
        node.get_logger().warn(
            f"IMU raw read returned {len(data)} values, expected at least 9",
            throttle_duration_sec=5.0,
        )
        return None

    try:
        return tuple(float(value) for value in data[:9])
    except (TypeError, ValueError) as exc:
        node.get_logger().warn(f"Invalid IMU raw data: {exc}", throttle_duration_sec=5.0)
        return None


class ImuPublisher:
    def __init__(self, node, muto, imu_link="imu_link"):
        self.node = node
        self.muto = muto
        self.imu_link = imu_link
        self.publisher = node.create_publisher(Imu, "/imu/data_raw", 100)
        self.mag_raw_publisher = node.create_publisher(MagneticField, "/imu/mag_raw", 100)
        self.publisher_1 = node.create_publisher(Imu, "/imu/data_processed", 100)

    def publish_imu_data(self):
        raw = read_imu_raw(self.node, self.muto)
        if raw is None:
            return

        ax, ay, az, gx, gy, gz, mx, my, mz = raw

        stamp = self.node.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = RAW_IMU_FRAME
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0

        set_imu_covariance(imu)

        self.publisher.publish(imu)

        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = RAW_IMU_FRAME
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz
        set_magnetic_field_covariance(mag)

        self.mag_raw_publisher.publish(mag)

        imu2 = Imu()
        imu2.header.stamp = stamp
        imu2.header.frame_id = self.imu_link
        imu2.linear_acceleration.x = ax * 9.8 / 8500.0
        imu2.linear_acceleration.y = ay * 9.8 / 8500.0
        imu2.linear_acceleration.z = az * 9.8 / 8500.0
        imu2.angular_velocity.x = (gx -31.0) / LSB_PER_DPS * math.pi / 180.0
        imu2.angular_velocity.y = (gy +1.0) / LSB_PER_DPS * math.pi / 180.0
        imu2.angular_velocity.z = (gz +17.0) / LSB_PER_DPS * math.pi / 180.0

        set_imu_covariance(imu2)

        self.publisher_1.publish(imu2)
