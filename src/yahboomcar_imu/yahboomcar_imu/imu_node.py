# imu_node.py
import math
import time

from sensor_msgs.msg import Imu, MagneticField

DEFAULT_GYRO_LSB_PER_DPS = 16.4
LSB_PER_DPS = DEFAULT_GYRO_LSB_PER_DPS
GRAVITY_MPS2 = 9.80665
DEFAULT_ACCEL_COUNTS_PER_G = 8500.0
DEFAULT_CALIBRATION_SAMPLE_COUNT = 1200
DEFAULT_CALIBRATION_MAX_READS = 3600
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


def trimmed_mean(values, trim_fraction=0.1):
    if not values:
        return 0.0

    ordered = sorted(values)
    trim_count = int(len(ordered) * trim_fraction)
    if trim_count > 0 and trim_count * 2 < len(ordered):
        ordered = ordered[trim_count:-trim_count]

    return sum(ordered) / len(ordered)


def population_stddev(values):
    if len(values) < 2:
        return 0.0

    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance)


class ImuPublisher:
    def __init__(self, node, muto, imu_link="imu_link"):
        self.node = node
        self.muto = muto
        self.imu_link = imu_link
        self.publisher = node.create_publisher(Imu, "/imu/data_raw", 100)
        self.mag_raw_publisher = node.create_publisher(MagneticField, "/imu/mag_raw", 100)
        self.publisher_1 = node.create_publisher(Imu, "/imu/data_processed", 100)

        self.accel_counts_per_g = float(
            node.declare_parameter("imu_accel_counts_per_g", DEFAULT_ACCEL_COUNTS_PER_G).value
        )
        self.gyro_lsb_per_dps = float(
            node.declare_parameter("imu_gyro_lsb_per_dps", DEFAULT_GYRO_LSB_PER_DPS).value
        )
        self.gyro_bias_x = float(node.declare_parameter("imu_gyro_bias_x", 0.0).value)
        self.gyro_bias_y = float(node.declare_parameter("imu_gyro_bias_y", 0.0).value)
        self.gyro_bias_z = float(node.declare_parameter("imu_gyro_bias_z", 0.0).value)
        self.calibrate_on_startup = bool(
            node.declare_parameter("imu_calibrate_on_startup", True).value
        )
        self.calibration_sample_count = int(
            node.declare_parameter(
                "imu_calibration_sample_count", DEFAULT_CALIBRATION_SAMPLE_COUNT
            ).value
        )
        self.calibration_max_reads = int(
            node.declare_parameter(
                "imu_calibration_max_reads", DEFAULT_CALIBRATION_MAX_READS
            ).value
        )
        self.calibration_read_interval = float(
            node.declare_parameter("imu_calibration_read_interval", 0.005).value
        )
        self.calibration_gyro_stddev_limit = float(
            node.declare_parameter("imu_calibration_gyro_stddev_limit", 80.0).value
        )
        self.calibration_accel_norm_stddev_limit = float(
            node.declare_parameter("imu_calibration_accel_norm_stddev_limit", 250.0).value
        )

        self.normalize_calibration_parameters()
        if self.calibrate_on_startup:
            self.calibrate_from_startup_samples()

    def normalize_calibration_parameters(self):
        if self.accel_counts_per_g <= 0.0:
            self.node.get_logger().warn(
                "imu_accel_counts_per_g must be positive; using default "
                f"{DEFAULT_ACCEL_COUNTS_PER_G:.1f}"
            )
            self.accel_counts_per_g = DEFAULT_ACCEL_COUNTS_PER_G
        if self.gyro_lsb_per_dps <= 0.0:
            self.node.get_logger().warn(
                "imu_gyro_lsb_per_dps must be positive; using default "
                f"{DEFAULT_GYRO_LSB_PER_DPS:.1f}"
            )
            self.gyro_lsb_per_dps = DEFAULT_GYRO_LSB_PER_DPS
        if self.calibration_sample_count < 1:
            self.node.get_logger().warn("imu_calibration_sample_count must be positive; using 1")
            self.calibration_sample_count = 1
        if self.calibration_max_reads < self.calibration_sample_count:
            self.node.get_logger().warn(
                "imu_calibration_max_reads is smaller than imu_calibration_sample_count; "
                "using sample count"
            )
            self.calibration_max_reads = self.calibration_sample_count
        if self.calibration_read_interval < 0.0:
            self.node.get_logger().warn(
                "imu_calibration_read_interval must be non-negative; using 0.0"
            )
            self.calibration_read_interval = 0.0
        if self.calibration_gyro_stddev_limit <= 0.0:
            self.node.get_logger().warn(
                "imu_calibration_gyro_stddev_limit must be positive; using 80.0"
            )
            self.calibration_gyro_stddev_limit = 80.0
        if self.calibration_accel_norm_stddev_limit <= 0.0:
            self.node.get_logger().warn(
                "imu_calibration_accel_norm_stddev_limit must be positive; using 250.0"
            )
            self.calibration_accel_norm_stddev_limit = 250.0

    def calibrate_from_startup_samples(self):
        self.node.get_logger().info(
            "Calibrating IMU from startup raw samples; keep the robot still"
        )

        samples = []
        for _ in range(self.calibration_max_reads):
            raw = read_imu_raw(self.node, self.muto)
            if raw is not None:
                samples.append(raw)
                if len(samples) >= self.calibration_sample_count:
                    break
            if self.calibration_read_interval > 0.0:
                time.sleep(self.calibration_read_interval)

        min_required_samples = max(10, self.calibration_sample_count // 2)
        if len(samples) < min_required_samples:
            self.node.get_logger().warn(
                "IMU startup calibration skipped: collected "
                f"{len(samples)} valid samples, need at least {min_required_samples}. "
                "Using configured IMU scale/bias parameters."
            )
            return

        ax_values = [sample[0] for sample in samples]
        ay_values = [sample[1] for sample in samples]
        az_values = [sample[2] for sample in samples]
        gx_values = [sample[3] for sample in samples]
        gy_values = [sample[4] for sample in samples]
        gz_values = [sample[5] for sample in samples]

        gyro_stddev = max(
            population_stddev(gx_values),
            population_stddev(gy_values),
            population_stddev(gz_values),
        )
        accel_norms = [
            math.sqrt(ax * ax + ay * ay + az * az)
            for ax, ay, az in zip(ax_values, ay_values, az_values)
        ]
        accel_norm_stddev = population_stddev(accel_norms)

        if gyro_stddev > self.calibration_gyro_stddev_limit:
            self.node.get_logger().warn(
                "IMU startup calibration rejected: gyro stddev "
                f"{gyro_stddev:.2f} raw counts exceeds "
                f"{self.calibration_gyro_stddev_limit:.2f}. "
                "Robot may have moved during startup."
            )
            return
        if accel_norm_stddev > self.calibration_accel_norm_stddev_limit:
            self.node.get_logger().warn(
                "IMU startup calibration rejected: accel norm stddev "
                f"{accel_norm_stddev:.2f} raw counts exceeds "
                f"{self.calibration_accel_norm_stddev_limit:.2f}. "
                "Robot may have moved during startup."
            )
            return

        accel_counts_per_g = trimmed_mean(accel_norms)
        if accel_counts_per_g <= 0.0:
            self.node.get_logger().warn(
                "IMU startup calibration rejected: invalid accel scale estimate. "
                "Using configured IMU scale/bias parameters."
            )
            return

        self.accel_counts_per_g = accel_counts_per_g
        self.gyro_bias_x = trimmed_mean(gx_values)
        self.gyro_bias_y = trimmed_mean(gy_values)
        self.gyro_bias_z = trimmed_mean(gz_values)

        self.node.get_logger().info(
            "IMU startup calibration accepted: "
            f"samples={len(samples)}, accel_counts_per_g={self.accel_counts_per_g:.2f}, "
            f"gyro_bias=({self.gyro_bias_x:.2f}, {self.gyro_bias_y:.2f}, "
            f"{self.gyro_bias_z:.2f}), gyro_stddev={gyro_stddev:.2f}, "
            f"accel_norm_stddev={accel_norm_stddev:.2f}"
        )

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
        imu2.linear_acceleration.x = ax * GRAVITY_MPS2 / self.accel_counts_per_g
        imu2.linear_acceleration.y = ay * GRAVITY_MPS2 / self.accel_counts_per_g
        imu2.linear_acceleration.z = az * GRAVITY_MPS2 / self.accel_counts_per_g
        imu2.angular_velocity.x = (gx - self.gyro_bias_x) / self.gyro_lsb_per_dps * math.pi / 180.0
        imu2.angular_velocity.y = (gy - self.gyro_bias_y) / self.gyro_lsb_per_dps * math.pi / 180.0
        imu2.angular_velocity.z = (gz - self.gyro_bias_z) / self.gyro_lsb_per_dps * math.pi / 180.0

        set_imu_covariance(imu2)

        self.publisher_1.publish(imu2)
