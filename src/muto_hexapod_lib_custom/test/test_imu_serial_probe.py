import argparse

from muto_hexapod_lib_custom.tools.imu_serial_probe import (
    build_argument_parser,
    parse_endpoints,
    parse_rates,
    read_endpoint,
    summarize_attitude_trial,
    summarize_trial,
)
import pytest


def test_parse_rates_requires_positive_finite_values():
    assert parse_rates('2, 10,50') == [2.0, 10.0, 50.0]
    for invalid in ('', '0', '-1', 'nan', '10,nope'):
        with pytest.raises(argparse.ArgumentTypeError):
            parse_rates(invalid)


def test_parse_endpoints_accepts_fused_alias_and_rejects_unknown_names():
    assert parse_endpoints('raw, fused') == ['raw', 'attitude']
    assert parse_endpoints('attitude,attitude') == ['attitude']
    for invalid in ('', 'imu', 'raw,nope'):
        with pytest.raises(argparse.ArgumentTypeError):
            parse_endpoints(invalid)


def test_probe_defaults_to_both_documented_imu_endpoints():
    args = build_argument_parser().parse_args([])

    assert args.endpoints == ['raw', 'attitude']


def test_summary_separates_motion_and_magnetometer_transitions():
    first = (1, 2, 3, 4, 5, 6, 7, 8, 9)
    mag_only = (1, 2, 3, 4, 5, 6, 8, 8, 9)
    motion_change = (1, 2, 3, 4, 5, 7, 8, 8, 9)
    observations = [
        {
            'time_sec': 0.1,
            'latency_sec': 0.003,
            'raw': first,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.2,
            'latency_sec': 0.004,
            'raw': first,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.3,
            'latency_sec': 0.005,
            'raw': mag_only,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.4,
            'latency_sec': 0.004,
            'raw': motion_change,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.5,
            'latency_sec': 0.050,
            'raw': None,
            'response_type': None,
        },
    ]

    result = summarize_trial(10.0, 1.0, 0.0, 1.0, observations)

    assert result['polls'] == 5
    assert result['successful_reads'] == 4
    assert result['timeouts_or_invalid_frames'] == 1
    assert result['accel_gyro_value_transitions'] == 1
    assert result['full_packet_value_transitions'] == 2
    assert result['accel_gyro_transition_interval_sample_count'] == 0
    assert result['accel_gyro_transition_interval_mean_ms'] is None
    assert result['full_packet_transition_interval_sample_count'] == 1
    assert result['full_packet_transition_interval_mean_ms'] == pytest.approx(
        100.0)
    assert result['response_types'] == [0x12]
    assert result['response_types_hex'] == ['0x12']


def test_attitude_summary_separates_orientation_and_temperature_changes():
    first = (-1.0, 2.0, 3.0, 30)
    temperature_only = (-1.0, 2.0, 3.0, 31)
    orientation_change = (-1.0, 2.0, 3.5, 31)
    observations = [
        {
            'time_sec': 0.1,
            'latency_sec': 0.002,
            'attitude': first,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.2,
            'latency_sec': 0.003,
            'attitude': first,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.3,
            'latency_sec': 0.004,
            'attitude': temperature_only,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.4,
            'latency_sec': 0.003,
            'attitude': orientation_change,
            'response_type': 0x12,
        },
        {
            'time_sec': 0.5,
            'latency_sec': 0.050,
            'attitude': None,
            'response_type': None,
        },
    ]

    result = summarize_attitude_trial(
        10.0, 1.0, 0.0, 1.0, observations)

    assert result['endpoint'] == 'attitude'
    assert result['endpoint_address_hex'] == '0x60'
    assert result['polls'] == 5
    assert result['successful_reads'] == 4
    assert result['orientation_value_transitions'] == 1
    assert result['full_attitude_value_transitions'] == 2
    assert result['orientation_transition_interval_sample_count'] == 0
    assert result['orientation_transition_interval_mean_ms'] is None
    assert result['full_attitude_transition_interval_sample_count'] == 1
    assert result['full_attitude_transition_interval_mean_ms'] == pytest.approx(
        100.0)


def test_raw_summary_reports_inter_transition_interval_distribution():
    values = [
        (0, 0, 0, 0, 0, gyro_z, 0, 0, 0)
        for gyro_z in range(4)
    ]
    observations = [
        {
            'time_sec': sample_time,
            'latency_sec': 0.003,
            'raw': value,
            'response_type': 0x12,
        }
        for sample_time, value in zip((0.1, 0.3, 0.5, 0.8), values)
    ]

    result = summarize_trial(20.0, 1.0, 0.0, 1.0, observations)

    assert result['accel_gyro_transition_interval_sample_count'] == 2
    assert result['accel_gyro_transition_interval_mean_ms'] == pytest.approx(
        250.0)
    assert result['accel_gyro_transition_interval_median_ms'] == pytest.approx(
        250.0)
    assert result['accel_gyro_transition_interval_p95_ms'] == pytest.approx(
        295.0)
    assert result['accel_gyro_transition_interval_min_ms'] == pytest.approx(
        200.0)
    assert result['accel_gyro_transition_interval_max_ms'] == pytest.approx(
        300.0)
    assert (
        result['full_packet_transition_interval_mean_ms']
        == pytest.approx(250.0)
    )


def test_attitude_summary_reports_inter_transition_interval_distribution():
    values = [
        (0.0, 0.0, yaw, 30)
        for yaw in range(4)
    ]
    observations = [
        {
            'time_sec': sample_time,
            'latency_sec': 0.003,
            'attitude': value,
            'response_type': 0x12,
        }
        for sample_time, value in zip((0.1, 0.25, 0.45, 0.70), values)
    ]

    result = summarize_attitude_trial(
        20.0, 1.0, 0.0, 1.0, observations)

    assert result['orientation_transition_interval_sample_count'] == 2
    assert result['orientation_transition_interval_mean_ms'] == pytest.approx(
        225.0)
    assert result['orientation_transition_interval_median_ms'] == pytest.approx(
        225.0)
    assert result['orientation_transition_interval_p95_ms'] == pytest.approx(
        247.5)
    assert result['orientation_transition_interval_min_ms'] == pytest.approx(
        200.0)
    assert result['orientation_transition_interval_max_ms'] == pytest.approx(
        250.0)
    assert (
        result['full_attitude_transition_interval_mean_ms']
        == pytest.approx(225.0)
    )


def test_endpoint_reader_calls_raw_and_fused_library_methods():
    class FakeRobot:
        def __init__(self):
            self.calls = []

        def read_IMU_Raw(self, response_timeout):
            self.calls.append(('raw', response_timeout))
            return [1] * 9

        def read_IMU(self, response_timeout):
            self.calls.append(('attitude', response_timeout))
            return [1.0, 2.0, 3.0, 30]

    robot = FakeRobot()

    assert read_endpoint(robot, 'raw', 0.01) == [1] * 9
    assert read_endpoint(robot, 'attitude', 0.02) == [1.0, 2.0, 3.0, 30]
    assert robot.calls == [('raw', 0.01), ('attitude', 0.02)]
    with pytest.raises(ValueError, match='unsupported IMU endpoint'):
        read_endpoint(robot, 'unknown', 0.01)
