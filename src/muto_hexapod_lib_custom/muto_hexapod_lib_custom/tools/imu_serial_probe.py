"""Measure controller IMU response and snapshot-change rates without ROS."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from muto_hexapod_lib_custom.core.MutoLibCore import Muto


ENDPOINT_ADDRESSES = {
    'raw': 0x61,
    'attitude': 0x60,
}
ENDPOINT_ALIASES = {
    'raw': 'raw',
    'attitude': 'attitude',
    'fused': 'attitude',
}


def parse_rates(value):
    """Parse a comma-separated list of positive host polling rates."""
    rates = []
    for item in value.split(','):
        try:
            rate = float(item.strip())
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f'invalid polling rate {item!r}'
            ) from exc
        if not math.isfinite(rate) or rate <= 0.0:
            raise argparse.ArgumentTypeError(
                'polling rates must be finite and positive'
            )
        rates.append(rate)
    if not rates:
        raise argparse.ArgumentTypeError('at least one polling rate is required')
    return rates


def parse_endpoints(value):
    """Parse endpoint names, accepting ``fused`` as an attitude alias."""
    endpoints = []
    for item in value.split(','):
        name = item.strip().lower()
        endpoint = ENDPOINT_ALIASES.get(name)
        if endpoint is None:
            raise argparse.ArgumentTypeError(
                f'unknown IMU endpoint {item!r}; use raw or attitude'
            )
        if endpoint not in endpoints:
            endpoints.append(endpoint)
    if not endpoints:
        raise argparse.ArgumentTypeError('at least one endpoint is required')
    return endpoints


def percentile(values, fraction):
    """Return one linearly interpolated percentile without NumPy."""
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def transition_interval_metrics(prefix, transition_times):
    """Return observed intervals between consecutive value transitions."""
    intervals_ms = [
        (current - previous) * 1000.0
        for previous, current in zip(
            transition_times, transition_times[1:])
    ]
    return {
        f'{prefix}_interval_sample_count': len(intervals_ms),
        f'{prefix}_interval_mean_ms': (
            statistics.fmean(intervals_ms) if intervals_ms else None
        ),
        f'{prefix}_interval_median_ms': (
            statistics.median(intervals_ms) if intervals_ms else None
        ),
        f'{prefix}_interval_p95_ms': percentile(intervals_ms, 0.95),
        f'{prefix}_interval_min_ms': min(intervals_ms) if intervals_ms else None,
        f'{prefix}_interval_max_ms': max(intervals_ms) if intervals_ms else None,
    }


def summarize_trial(rate_hz, duration_sec, started, ended, observations):
    """Summarize successful reads and exact snapshot transitions."""
    successful = [item for item in observations if item['raw'] is not None]
    latencies = [item['latency_sec'] for item in observations]
    motion_transition_times = []
    packet_transition_times = []
    previous_motion = None
    previous_packet = None
    for item in successful:
        raw = item['raw']
        motion = tuple(raw[:6])
        packet = tuple(raw)
        if previous_motion is not None and motion != previous_motion:
            motion_transition_times.append(item['time_sec'])
        if previous_packet is not None and packet != previous_packet:
            packet_transition_times.append(item['time_sec'])
        previous_motion = motion
        previous_packet = packet

    elapsed = max(0.0, ended - started)
    response_types = sorted({
        item['response_type']
        for item in successful
        if item['response_type'] is not None
    })
    return {
        'endpoint': 'raw',
        'endpoint_address': ENDPOINT_ADDRESSES['raw'],
        'endpoint_address_hex': '0x61',
        'requested_poll_rate_hz': rate_hz,
        'requested_duration_sec': duration_sec,
        'elapsed_sec': elapsed,
        'polls': len(observations),
        'successful_reads': len(successful),
        'timeouts_or_invalid_frames': len(observations) - len(successful),
        'achieved_poll_rate_hz': (
            len(observations) / elapsed if elapsed > 0.0 else 0.0
        ),
        # The stock protocol has no sequence/timestamp. These are conservative
        # exact-value transition rates, not authoritative sensor ODRs.
        'accel_gyro_value_transitions': len(motion_transition_times),
        'accel_gyro_transition_rate_hz': (
            len(motion_transition_times) / elapsed if elapsed > 0.0 else 0.0
        ),
        'full_packet_value_transitions': len(packet_transition_times),
        'full_packet_transition_rate_hz': (
            len(packet_transition_times) / elapsed if elapsed > 0.0 else 0.0
        ),
        **transition_interval_metrics(
            'accel_gyro_transition', motion_transition_times),
        **transition_interval_metrics(
            'full_packet_transition', packet_transition_times),
        'read_latency_mean_ms': (
            statistics.fmean(latencies) * 1000.0 if latencies else None
        ),
        'read_latency_p95_ms': (
            percentile(latencies, 0.95) * 1000.0 if latencies else None
        ),
        'response_types': response_types,
        'response_types_hex': [
            f'0x{response_type:02x}' for response_type in response_types
        ],
    }


def summarize_attitude_trial(
        rate_hz, duration_sec, started, ended, observations):
    """Summarize ``0x60`` fused-orientation and temperature transitions."""
    successful = [
        item for item in observations if item['attitude'] is not None
    ]
    latencies = [item['latency_sec'] for item in observations]
    orientation_transition_times = []
    full_transition_times = []
    previous_orientation = None
    previous_attitude = None
    for item in successful:
        attitude = item['attitude']
        orientation = tuple(attitude[:3])
        packet = tuple(attitude)
        if (
                previous_orientation is not None
                and orientation != previous_orientation):
            orientation_transition_times.append(item['time_sec'])
        if previous_attitude is not None and packet != previous_attitude:
            full_transition_times.append(item['time_sec'])
        previous_orientation = orientation
        previous_attitude = packet

    elapsed = max(0.0, ended - started)
    response_types = sorted({
        item['response_type']
        for item in successful
        if item['response_type'] is not None
    })
    return {
        'endpoint': 'attitude',
        'endpoint_address': ENDPOINT_ADDRESSES['attitude'],
        'endpoint_address_hex': '0x60',
        'requested_poll_rate_hz': rate_hz,
        'requested_duration_sec': duration_sec,
        'elapsed_sec': elapsed,
        'polls': len(observations),
        'successful_reads': len(successful),
        'timeouts_or_invalid_frames': len(observations) - len(successful),
        'achieved_poll_rate_hz': (
            len(observations) / elapsed if elapsed > 0.0 else 0.0
        ),
        # Exact-value transitions are lower bounds because 0x60 also has no
        # acquisition timestamp or sequence number.
        'orientation_value_transitions': len(orientation_transition_times),
        'orientation_transition_rate_hz': (
            len(orientation_transition_times) / elapsed
            if elapsed > 0.0 else 0.0
        ),
        'full_attitude_value_transitions': len(full_transition_times),
        'full_attitude_transition_rate_hz': (
            len(full_transition_times) / elapsed if elapsed > 0.0 else 0.0
        ),
        **transition_interval_metrics(
            'orientation_transition', orientation_transition_times),
        **transition_interval_metrics(
            'full_attitude_transition', full_transition_times),
        'read_latency_mean_ms': (
            statistics.fmean(latencies) * 1000.0 if latencies else None
        ),
        'read_latency_p95_ms': (
            percentile(latencies, 0.95) * 1000.0 if latencies else None
        ),
        'response_types': response_types,
        'response_types_hex': [
            f'0x{response_type:02x}' for response_type in response_types
        ],
    }


def read_endpoint(robot, endpoint, response_timeout_sec):
    """Read one documented controller IMU endpoint."""
    if endpoint == 'raw':
        return robot.read_IMU_Raw(response_timeout=response_timeout_sec)
    if endpoint == 'attitude':
        return robot.read_IMU(response_timeout=response_timeout_sec)
    raise ValueError(f'unsupported IMU endpoint: {endpoint!r}')


def run_trial(
        robot, rate_hz, duration_sec, response_timeout_sec, endpoint='raw'):
    """Poll at one requested host rate for a bounded duration."""
    if endpoint not in ENDPOINT_ADDRESSES:
        raise ValueError(f'unsupported IMU endpoint: {endpoint!r}')
    period = 1.0 / rate_hz
    started = time.monotonic()
    deadline = started + duration_sec
    next_poll = started
    observations = []
    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        if now < next_poll:
            time.sleep(min(next_poll, deadline) - now)
        poll_started = time.monotonic()
        if poll_started >= deadline:
            break
        sample = read_endpoint(robot, endpoint, response_timeout_sec)
        completed = time.monotonic()
        observation = {
            'time_sec': completed,
            'latency_sec': completed - poll_started,
            'response_type': getattr(robot, 'last_response_type', None),
        }
        observation[endpoint] = sample
        observations.append(observation)
        next_poll += period
        if next_poll < completed:
            # Do not issue catch-up bursts after a slow controller response.
            next_poll = completed
    ended = time.monotonic()
    summarizer = (
        summarize_trial if endpoint == 'raw' else summarize_attitude_trial)
    return summarizer(rate_hz, duration_sec, started, ended, observations)


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            'Sweep the raw 0x61 and fused-attitude 0x60 IMU endpoints and '
            'report response and exact value-change rates. Stop muto_driver '
            'first: this tool requires exclusive access to /dev/myserial.'
        )
    )
    parser.add_argument('--port', default='/dev/myserial')
    parser.add_argument(
        '--rates',
        type=parse_rates,
        default=parse_rates('2,5,10,20,50'),
        help='comma-separated host polling rates in Hz',
    )
    parser.add_argument(
        '--endpoints',
        type=parse_endpoints,
        default=parse_endpoints('raw,attitude'),
        help='comma-separated endpoints: raw, attitude (or fused)',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=8.0,
        help='seconds to test each rate',
    )
    parser.add_argument(
        '--response-timeout',
        type=float,
        default=0.05,
        help='maximum seconds to wait for each response',
    )
    parser.add_argument(
        '--settle',
        type=float,
        default=0.5,
        help='idle seconds between rate steps',
    )
    parser.add_argument(
        '--output',
        type=Path,
        help='optional JSON output path',
    )
    return parser


def main(argv=None):
    args = build_argument_parser().parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit('--duration must be finite and positive')
    if (
        not math.isfinite(args.response_timeout)
        or args.response_timeout <= 0.0
    ):
        raise SystemExit('--response-timeout must be finite and positive')
    if not math.isfinite(args.settle) or args.settle < 0.0:
        raise SystemExit('--settle must be finite and non-negative')

    report = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'port': args.port,
        'endpoints': args.endpoints,
        'response_timeout_sec': args.response_timeout,
        'note': (
            'Value transitions are lower bounds because the stock protocol '
            'does not expose a sensor sequence number or acquisition timestamp. '
            'Each endpoint is measured in a separate trial.'
        ),
        'trials': [],
    }
    robot = None
    try:
        robot = Muto(port=args.port)
        trial_index = 0
        for rate_hz in args.rates:
            for endpoint in args.endpoints:
                if trial_index and args.settle > 0.0:
                    time.sleep(args.settle)
                trial = run_trial(
                    robot,
                    rate_hz=rate_hz,
                    duration_sec=args.duration,
                    response_timeout_sec=args.response_timeout,
                    endpoint=endpoint,
                )
                report['trials'].append(trial)
                print(json.dumps(trial, indent=2, sort_keys=True))
                trial_index += 1
    finally:
        if robot is not None:
            robot.close()

    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output is not None:
        args.output.write_text(rendered, encoding='utf-8')
        print(f'wrote {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
