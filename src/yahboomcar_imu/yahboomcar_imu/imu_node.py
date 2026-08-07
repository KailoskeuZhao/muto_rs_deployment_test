# imu_node.py
import math
import time

from muto_hexapod_interfaces_custom.msg import ControllerAttitude
from sensor_msgs.msg import Imu, MagneticField

DEFAULT_GYRO_LSB_PER_DPS = 16.4
LSB_PER_DPS = DEFAULT_GYRO_LSB_PER_DPS
GRAVITY_MPS2 = 9.80665
DEFAULT_ACCEL_COUNTS_PER_G = 8500.0
DEFAULT_CALIBRATION_SAMPLE_COUNT = 10
DEFAULT_CALIBRATION_MAX_READS = 150
DEFAULT_CALIBRATION_TIMEOUT_SEC = 15.0
DEFAULT_CALIBRATION_READ_INTERVAL_SEC = 0.1
DEFAULT_RESPONSE_TIMEOUT_SEC = 0.008
DEFAULT_STALE_WARNING_SEC = 2.0
CONTROLLER_DATA_RETURN_TYPE = 0x12
DEFAULT_YAW_RATE_DEADBAND_RAD_S = 0.03
# odom_test_001 stationary raw gyro noise: 2.73 counts at 16.4 counts/(deg/s).
ANGULAR_VELOCITY_COVARIANCE = 8.5e-6
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


def read_imu_raw(node, muto, response_timeout_sec=None):
    if response_timeout_sec is None:
        data = muto.read_IMU_Raw()
    else:
        data = muto.read_IMU_Raw(response_timeout=response_timeout_sec)
    if data is None:
        diagnostic = getattr(muto, "last_read_diagnostic", "no diagnostic available")
        node.get_logger().warn(
            f"IMU raw read returned no data: {diagnostic}",
            throttle_duration_sec=5.0,
        )
        return None
    if len(data) < 9:
        node.get_logger().warn(
            f"IMU raw read returned {len(data)} values, expected at least 9",
            throttle_duration_sec=5.0,
        )
        return None

    try:
        result = tuple(float(value) for value in data[:9])
    except (TypeError, ValueError) as exc:
        node.get_logger().warn(f"Invalid IMU raw data: {exc}", throttle_duration_sec=5.0)
        return None

    response_type = getattr(muto, "last_response_type", None)
    if (
        response_type is not None
        and response_type != CONTROLLER_DATA_RETURN_TYPE
    ):
        node.get_logger().warn(
            "Unexpected controller IMU response type "
            f"0x{response_type:02x}; expected data-return type "
            f"0x{CONTROLLER_DATA_RETURN_TYPE:02x}",
            throttle_duration_sec=60.0,
        )
    return result


def read_controller_attitude(node, muto, response_timeout_sec=None):
    """Read and validate one vendor-fused ``0x60`` attitude response."""
    if response_timeout_sec is None:
        data = muto.read_IMU()
    else:
        data = muto.read_IMU(response_timeout=response_timeout_sec)
    if data is None:
        diagnostic = getattr(muto, "last_read_diagnostic", "no diagnostic available")
        node.get_logger().warn(
            f"Controller attitude read returned no data: {diagnostic}",
            throttle_duration_sec=5.0,
        )
        return None
    if len(data) < 4:
        node.get_logger().warn(
            "Controller attitude read returned "
            f"{len(data)} values, expected at least 4",
            throttle_duration_sec=5.0,
        )
        return None

    try:
        roll_deg, pitch_deg, yaw_deg = (
            float(value) for value in data[:3]
        )
        temperature_raw = int(data[3])
    except (TypeError, ValueError) as exc:
        node.get_logger().warn(
            f"Invalid controller attitude data: {exc}",
            throttle_duration_sec=5.0,
        )
        return None
    if not all(math.isfinite(value) for value in (
        roll_deg, pitch_deg, yaw_deg
    )):
        node.get_logger().warn(
            "Controller attitude contains a non-finite angle",
            throttle_duration_sec=5.0,
        )
        return None
    if not 0 <= temperature_raw <= 255:
        node.get_logger().warn(
            "Controller attitude temperature byte is outside [0, 255]",
            throttle_duration_sec=5.0,
        )
        return None

    response_type = getattr(muto, "last_response_type", None)
    if (
        response_type is not None
        and response_type != CONTROLLER_DATA_RETURN_TYPE
    ):
        node.get_logger().warn(
            "Unexpected controller attitude response type "
            f"0x{response_type:02x}; expected data-return type "
            f"0x{CONTROLLER_DATA_RETURN_TYPE:02x}",
            throttle_duration_sec=60.0,
        )
    return roll_deg, pitch_deg, yaw_deg, temperature_raw


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
        self.controller_attitude_publisher = node.create_publisher(
            ControllerAttitude,
            "/imu/controller_attitude",
            100,
        )

        self.accel_counts_per_g = float(
            node.declare_parameter("imu_accel_counts_per_g", DEFAULT_ACCEL_COUNTS_PER_G).value
        )
        self.gyro_lsb_per_dps = float(
            node.declare_parameter("imu_gyro_lsb_per_dps", DEFAULT_GYRO_LSB_PER_DPS).value
        )
        self.gyro_bias_x = float(node.declare_parameter("imu_gyro_bias_x", 0.0).value)
        self.gyro_bias_y = float(node.declare_parameter("imu_gyro_bias_y", 0.0).value)
        self.gyro_bias_z = float(node.declare_parameter("imu_gyro_bias_z", 0.0).value)
        self.yaw_rate_deadband_rad_s = float(
            node.declare_parameter(
                "imu_yaw_rate_deadband_rad_s", DEFAULT_YAW_RATE_DEADBAND_RAD_S
            ).value
        )
        self.calibrate_on_startup = bool(
            node.declare_parameter("imu_calibrate_on_startup", True).value
        )
        self.suppress_identical_snapshots = bool(
            node.declare_parameter(
                "imu_suppress_identical_snapshots", True
            ).value
        )
        self.response_timeout_sec = float(
            node.declare_parameter(
                "imu_response_timeout_sec", DEFAULT_RESPONSE_TIMEOUT_SEC
            ).value
        )
        self.stale_warning_sec = float(
            node.declare_parameter(
                "imu_stale_warning_sec", DEFAULT_STALE_WARNING_SEC
            ).value
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
        self.calibration_timeout_sec = float(
            node.declare_parameter(
                "imu_calibration_timeout_sec", DEFAULT_CALIBRATION_TIMEOUT_SEC
            ).value
        )
        self.calibration_read_interval = float(
            node.declare_parameter(
                "imu_calibration_read_interval",
                DEFAULT_CALIBRATION_READ_INTERVAL_SEC,
            ).value
        )
        self.calibration_gyro_stddev_limit = float(
            node.declare_parameter("imu_calibration_gyro_stddev_limit", 80.0).value
        )
        self.calibration_accel_norm_stddev_limit = float(
            node.declare_parameter("imu_calibration_accel_norm_stddev_limit", 250.0).value
        )

        self.poll_count = 0
        self.successful_read_count = 0
        self.changed_snapshot_count = 0
        self.duplicate_sample_count = 0
        self.skipped_for_locomotion_count = 0
        self.attitude_poll_count = 0
        self.successful_attitude_read_count = 0
        self.attitude_skipped_for_locomotion_count = 0
        self.last_observed_motion_sample = None
        self.last_changed_monotonic = None
        self.last_read_duration_sec = 0.0

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
        if self.yaw_rate_deadband_rad_s < 0.0:
            self.node.get_logger().warn(
                "imu_yaw_rate_deadband_rad_s must be non-negative; using 0.0"
            )
            self.yaw_rate_deadband_rad_s = 0.0
        if (
            not math.isfinite(self.response_timeout_sec)
            or self.response_timeout_sec <= 0.0
        ):
            self.node.get_logger().warn(
                "imu_response_timeout_sec must be positive; using "
                f"{DEFAULT_RESPONSE_TIMEOUT_SEC:.3f}"
            )
            self.response_timeout_sec = DEFAULT_RESPONSE_TIMEOUT_SEC
        if (
            not math.isfinite(self.stale_warning_sec)
            or self.stale_warning_sec < 0.0
        ):
            self.node.get_logger().warn(
                "imu_stale_warning_sec must be non-negative; using "
                f"{DEFAULT_STALE_WARNING_SEC:.1f}"
            )
            self.stale_warning_sec = DEFAULT_STALE_WARNING_SEC
        if self.calibration_sample_count < 1:
            self.node.get_logger().warn("imu_calibration_sample_count must be positive; using 1")
            self.calibration_sample_count = 1
        if self.calibration_max_reads < self.calibration_sample_count:
            self.node.get_logger().warn(
                "imu_calibration_max_reads is smaller than imu_calibration_sample_count; "
                "using sample count"
            )
            self.calibration_max_reads = self.calibration_sample_count
        if self.calibration_timeout_sec <= 0.0:
            self.node.get_logger().warn(
                "imu_calibration_timeout_sec must be positive; using "
                f"{DEFAULT_CALIBRATION_TIMEOUT_SEC:.1f}"
            )
            self.calibration_timeout_sec = DEFAULT_CALIBRATION_TIMEOUT_SEC
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
        attempts = 0
        duplicate_reads = 0
        previous_motion_sample = None
        started = time.monotonic()
        deadline = started + self.calibration_timeout_sec
        stop_reason = "maximum read attempts reached"
        for _ in range(self.calibration_max_reads):
            if time.monotonic() >= deadline:
                stop_reason = "wall-clock timeout reached"
                break
            attempts += 1
            # Startup calibration happens before ROS timers start, so retain
            # the vendor-compatible 50 ms read timeout here. The shorter
            # runtime timeout is specifically a locomotion deadline guard.
            raw = read_imu_raw(self.node, self.muto)
            if raw is not None:
                motion_sample = raw[:6]
                if (
                    previous_motion_sample is not None
                    and motion_sample == previous_motion_sample
                ):
                    duplicate_reads += 1
                else:
                    samples.append(raw)
                    previous_motion_sample = motion_sample
                    if len(samples) >= self.calibration_sample_count:
                        stop_reason = "target changed-snapshot count reached"
                        break
            if self.calibration_read_interval > 0.0:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    stop_reason = "wall-clock timeout reached"
                    break
                time.sleep(min(self.calibration_read_interval, remaining))

        elapsed = time.monotonic() - started

        min_required_samples = min(
            self.calibration_sample_count,
            max(10, self.calibration_sample_count // 2),
        )
        if len(samples) < min_required_samples:
            self.node.get_logger().warn(
                "IMU startup calibration skipped: collected "
                f"{len(samples)} changed snapshots from {attempts} attempts "
                f"({duplicate_reads} cached duplicates) in "
                f"{elapsed:.2f} s, need at least {min_required_samples}; "
                f"{stop_reason}. "
                "Using configured IMU scale/bias parameters."
            )
            return

        if len(samples) < self.calibration_sample_count:
            self.node.get_logger().warn(
                "IMU startup calibration proceeding with "
                f"{len(samples)}/{self.calibration_sample_count} changed snapshots "
                f"after {attempts} attempts ({duplicate_reads} cached duplicates) "
                f"in {elapsed:.2f} s; {stop_reason}"
            )

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
            f"changed_snapshots={len(samples)}, attempts={attempts}, "
            f"cached_duplicates={duplicate_reads}, elapsed={elapsed:.2f}s, "
            f"accel_counts_per_g={self.accel_counts_per_g:.2f}, "
            f"gyro_bias=({self.gyro_bias_x:.2f}, {self.gyro_bias_y:.2f}, "
            f"{self.gyro_bias_z:.2f}), gyro_stddev={gyro_stddev:.2f}, "
            f"accel_norm_stddev={accel_norm_stddev:.2f}"
        )

    def note_poll_skipped_for_locomotion(self):
        """Record an IMU poll omitted to protect the gait dispatch deadline."""
        self.skipped_for_locomotion_count += 1

    def note_attitude_poll_skipped_for_locomotion(self):
        """Record a fused-attitude poll omitted for the gait deadline."""
        self.attitude_skipped_for_locomotion_count += 1

    def publish_controller_attitude(self, response_timeout_sec=None):
        """Publish every valid controller-fused response, including repeats.

        Repeated samples are intentionally retained so recorded bags expose
        the controller cache/update cadence. This vendor Euler channel remains
        diagnostic-only until its frame, signs, wrap, and reference are known.
        """
        self.attitude_poll_count += 1
        attitude = read_controller_attitude(
            self.node,
            self.muto,
            response_timeout_sec=(
                self.response_timeout_sec
                if response_timeout_sec is None else response_timeout_sec
            ),
        )
        if attitude is None:
            return False

        self.successful_attitude_read_count += 1
        roll_deg, pitch_deg, yaw_deg, temperature_raw = attitude
        message = ControllerAttitude()
        message.header.stamp = self.node.get_clock().now().to_msg()
        # Deliberately empty: the vendor Euler convention has not been mapped
        # to imu_link/base_frame yet.
        message.header.frame_id = ""
        message.roll_deg = roll_deg
        message.pitch_deg = pitch_deg
        message.yaw_deg = yaw_deg
        message.temperature_raw = temperature_raw
        self.controller_attitude_publisher.publish(message)
        return True

    def publish_imu_data(self, response_timeout_sec=None):
        """Poll once and conservatively suppress identical motion snapshots.

        The stock protocol has no controller sequence or acquisition timestamp.
        Exact equality of accel and gyro values is therefore only a cache
        heuristic, not proof that the sensor itself failed to acquire a sample.
        """
        self.poll_count += 1
        read_started = time.monotonic()
        raw = read_imu_raw(
            self.node,
            self.muto,
            response_timeout_sec=(
                self.response_timeout_sec
                if response_timeout_sec is None else response_timeout_sec
            ),
        )
        self.last_read_duration_sec = time.monotonic() - read_started
        if raw is None:
            return False

        self.successful_read_count += 1
        now_monotonic = time.monotonic()
        motion_sample = raw[:6]
        identical_snapshot = (
            self.last_observed_motion_sample is not None
            and motion_sample == self.last_observed_motion_sample
        )
        if identical_snapshot:
            self.duplicate_sample_count += 1
            if self.suppress_identical_snapshots:
                if (
                    self.stale_warning_sec > 0.0
                    and self.last_changed_monotonic is not None
                    and now_monotonic - self.last_changed_monotonic
                    >= self.stale_warning_sec
                ):
                    self.node.get_logger().warn(
                        "Controller IMU snapshot unchanged for "
                        f"{now_monotonic - self.last_changed_monotonic:.2f} s; "
                        "suppressing duplicate ROS publications",
                        throttle_duration_sec=5.0,
                    )
                return False
        else:
            self.last_observed_motion_sample = motion_sample
            self.last_changed_monotonic = now_monotonic
            self.changed_snapshot_count += 1

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
        yaw_rate = (gz - self.gyro_bias_z) / self.gyro_lsb_per_dps * math.pi / 180.0
        if abs(yaw_rate) < self.yaw_rate_deadband_rad_s:
            yaw_rate = 0.0
        imu2.angular_velocity.z = yaw_rate

        set_imu_covariance(imu2)

        self.publisher_1.publish(imu2)
        return True
