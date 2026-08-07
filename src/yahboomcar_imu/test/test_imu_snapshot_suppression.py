from builtin_interfaces.msg import Time

from yahboomcar_imu import imu_node


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_kwargs):
        self.warnings.append(message)


class FakeNow:
    def to_msg(self):
        return Time()


class FakeClock:
    def now(self):
        return FakeNow()


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return FakeClock()


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_publisher():
    publisher = imu_node.ImuPublisher.__new__(imu_node.ImuPublisher)
    publisher.node = FakeNode()
    publisher.muto = object()
    publisher.imu_link = 'imu_link'
    publisher.publisher = RecordingPublisher()
    publisher.mag_raw_publisher = RecordingPublisher()
    publisher.publisher_1 = RecordingPublisher()
    publisher.accel_counts_per_g = imu_node.DEFAULT_ACCEL_COUNTS_PER_G
    publisher.gyro_lsb_per_dps = imu_node.DEFAULT_GYRO_LSB_PER_DPS
    publisher.gyro_bias_x = 0.0
    publisher.gyro_bias_y = 0.0
    publisher.gyro_bias_z = 0.0
    publisher.yaw_rate_deadband_rad_s = 0.0
    publisher.suppress_identical_snapshots = True
    publisher.response_timeout_sec = imu_node.DEFAULT_RESPONSE_TIMEOUT_SEC
    publisher.stale_warning_sec = 0.0
    publisher.poll_count = 0
    publisher.successful_read_count = 0
    publisher.failed_read_count = 0
    publisher.changed_snapshot_count = 0
    publisher.duplicate_sample_count = 0
    publisher.skipped_for_locomotion_count = 0
    publisher.last_observed_motion_sample = None
    publisher.last_changed_monotonic = None
    publisher.last_read_duration_sec = 0.0
    return publisher


def test_identical_accel_gyro_snapshot_is_not_retimestamped(monkeypatch):
    snapshots = iter([
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0),
        # Magnetometer-only movement must not make cached EKF motion fresh.
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 8.0, 9.0),
        (1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 8.0, 8.0, 9.0),
    ])
    monkeypatch.setattr(
        imu_node,
        'read_imu_raw',
        lambda *_args, **_kwargs: next(snapshots),
    )
    publisher = make_publisher()

    assert publisher.publish_imu_data() is True
    assert publisher.publish_imu_data() is False
    assert publisher.publish_imu_data() is True

    assert publisher.poll_count == 3
    assert publisher.successful_read_count == 3
    assert publisher.changed_snapshot_count == 2
    assert publisher.duplicate_sample_count == 1
    assert len(publisher.publisher.messages) == 2
    assert len(publisher.mag_raw_publisher.messages) == 2
    assert len(publisher.publisher_1.messages) == 2


def test_failed_read_is_distinct_from_suppressed_duplicate(monkeypatch):
    sample = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)
    snapshots = iter((sample, sample, None))
    monkeypatch.setattr(
        imu_node,
        'read_imu_raw',
        lambda *_args, **_kwargs: next(snapshots),
    )
    publisher = make_publisher()

    assert publisher.publish_imu_data() is True
    assert publisher.publish_imu_data() is False
    assert publisher.publish_imu_data() is False

    assert publisher.poll_count == 3
    assert publisher.successful_read_count == 2
    assert publisher.duplicate_sample_count == 1
    assert publisher.failed_read_count == 1


def test_identical_snapshot_can_be_republished_for_diagnostics(monkeypatch):
    raw = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)
    monkeypatch.setattr(
        imu_node,
        'read_imu_raw',
        lambda *_args, **_kwargs: raw,
    )
    publisher = make_publisher()
    publisher.suppress_identical_snapshots = False

    assert publisher.publish_imu_data() is True
    assert publisher.publish_imu_data() is True
    assert publisher.changed_snapshot_count == 1
    assert publisher.duplicate_sample_count == 1
    assert len(publisher.publisher.messages) == 2
