import struct

from muto_hexapod_lib_custom.core import MutoLibCore
from muto_hexapod_lib_custom.core import servo as servo_module
from muto_hexapod_lib_custom.core.MutoLibCore import Muto
import pytest


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.read_buffer = b''
        self.is_open = True
        self.flush_count = 0

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def reset_input_buffer(self):
        pass

    def flush(self):
        self.flush_count += 1

    def read_all(self):
        data = self.read_buffer
        self.read_buffer = b''
        return data

    def close(self):
        self.is_open = False


class RecordingRLock:
    def __init__(self):
        self.depth = 0
        self.max_depth = 0
        self.enter_count = 0

    def __enter__(self):
        self.depth += 1
        self.enter_count += 1
        self.max_depth = max(self.max_depth, self.depth)
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.depth -= 1


def response_packet(
        address, payload, response_type=MutoLibCore.DATA_RETURN_COMMAND):
    payload = bytes(payload)
    packet_length = len(payload) + 8
    checksum = 255 - (
        packet_length
        + response_type
        + address
        + sum(payload)
    ) % 256
    return bytes([
        MutoLibCore.HEADER,
        MutoLibCore.DEVICE_ID,
        packet_length,
        response_type,
        address,
        *payload,
        checksum,
        *MutoLibCore.TRAILER,
    ])


def disable_protocol_delays(monkeypatch):
    monkeypatch.setattr(MutoLibCore.time, 'sleep', lambda _seconds: None)
    monkeypatch.setattr(servo_module.time, 'sleep', lambda _seconds: None)


def written_protocol_packets(serial):
    """Split one or more contiguous serial writes into vendor frames."""
    packets = []
    for write in serial.writes:
        offset = 0
        while offset < len(write):
            packet_length = write[offset + 2]
            packet = write[offset:offset + packet_length]
            assert len(packet) == packet_length
            packets.append(packet)
            offset += packet_length
    return packets


def test_move_emits_one_callback_per_vendor_gait_step(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.move(10, 0, 0)

    leg_packets = [
        packet for packet in written_protocol_packets(serial)
        if packet[4] == 0x41
    ]
    assert len(states) == 20
    assert len(leg_packets) == 20 * 6
    assert len(serial.writes) == 20
    assert states[-1].phase_index == 0
    assert states[-1].cycle_complete
    assert all(packet[:2] == b'\x55\x00' for packet in leg_packets)
    assert all(packet[2] == 14 for packet in leg_packets)
    assert all(packet[-2:] == b'\x00\xaa' for packet in leg_packets)
    for phase_start in range(0, len(leg_packets), 6):
        assert [
            packet[5] for packet in leg_packets[phase_start:phase_start + 6]
        ] == list(range(1, 7))


def test_blocking_move_finishes_queued_command_when_called_mid_cycle(
        monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)
    robot.set_motion_command(10, 0, 0)
    for _ in range(5):
        robot.tick_motion()
    call_start = len(states)

    result = robot.move(20, 0, 15)

    emitted = states[call_start:]
    assert len(emitted) == 15 + 20
    assert all(state.mode == 'move_x' for state in emitted[:15])
    assert all(state.x_level == 10 for state in emitted[:15])
    assert all(state.replacement_pending for state in emitted[:15])
    assert all(state.mode == 'move_xz' for state in emitted[15:])
    assert all(
        (state.x_level, state.y_level, state.z_level) == (20, 0, 15)
        for state in emitted[15:]
    )
    assert not any(
        state.replacement_pending for state in emitted[15:])
    assert result is emitted[-1]
    assert result.cycle_complete
    assert result.phase_index == 0


def test_latched_motion_advances_one_phase_per_tick(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    changed = robot.set_motion_command(10, 0, 0)

    assert changed
    assert states == []
    assert serial.writes == []

    robot.tick_motion()

    leg_packets = [
        packet for packet in written_protocol_packets(serial)
        if packet[4] == 0x41
    ]
    assert len(states) == 1
    assert states[0].phase_index == 1
    assert len(leg_packets) == 6
    assert len(serial.writes) == 1
    assert serial.writes[0] == b''.join(leg_packets)


def test_latched_phase_holds_one_outer_serial_transaction(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    lock = RecordingRLock()
    callback_lock_depths = []
    robot = Muto(
        serial_port=serial,
        gait_step_callback=lambda _state: callback_lock_depths.append(
            lock.depth),
    )
    robot.set_motion_command(10, 0, 0)
    robot._serial_lock = lock

    robot.tick_motion()

    # One outer phase lock plus one nested re-entrant write for the whole phase.
    assert lock.enter_count == 2
    assert lock.max_depth == 2
    assert lock.depth == 0
    assert callback_lock_depths == [0]


def test_latched_phase_sleeps_once_after_contiguous_batch(monkeypatch):
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    robot.set_motion_command(10, 0, 0)
    sleeps = []
    monkeypatch.setattr(servo_module.time, 'sleep', sleeps.append)

    robot.tick_motion()

    packets = written_protocol_packets(serial)
    assert len(serial.writes) == 1
    assert len(packets) == 6
    assert [packet[5] for packet in packets] == list(range(1, 7))
    assert serial.flush_count == 1
    assert sleeps == [0.001]


def test_latched_phase_rollback_uses_original_six_writes(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(
        serial_port=serial,
        batch_gait_phase_writes=False,
    )
    robot.set_motion_command(10, 0, 0)

    robot.tick_motion()

    assert len(serial.writes) == 6
    assert all(len(write) == 14 for write in serial.writes)
    assert [write[5] for write in serial.writes] == list(range(1, 7))


def test_initial_standby_tick_commands_hardware_then_heartbeats(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.set_motion_command(0, 0, 0)
    robot.tick_motion()
    first_write_count = len(serial.writes)
    first_packet_count = len(written_protocol_packets(serial))
    robot.tick_motion()

    assert first_packet_count == 6
    assert first_write_count == 1
    assert len(serial.writes) == first_write_count
    assert [state.sequence for state in states] == [1, 2]
    assert all(state.mode == 'standby' for state in states)


def test_nonzero_command_update_waits_for_completed_cycle(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.set_motion_command(10, 0, 0)
    robot.tick_motion()
    assert robot.set_motion_command(20, 0, 15)
    queued_state = robot.commanded_gait_state
    assert (queued_state.x_level, queued_state.z_level) == (10, 0)
    assert queued_state.replacement_pending

    for _ in range(19):
        robot.tick_motion()

    assert [state.phase_index for state in states] == list(range(1, 20)) + [0]
    assert all(state.mode == 'move_x' for state in states)
    assert all(state.x_level == 10 for state in states)
    assert not states[0].replacement_pending
    assert all(state.replacement_pending for state in states[1:])
    assert states[-1].cycle_complete

    robot.tick_motion()

    assert states[-1].mode == 'move_xz'
    assert (states[-1].x_level, states[-1].z_level) == (20, 15)
    assert not states[-1].replacement_pending
    assert states[-1].phase_index == 1
    assert states[-1].sequence == 21


def test_latest_queued_nonzero_command_wins(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)

    robot.set_motion_command(10, 0, 0)
    robot.tick_motion()
    assert robot.set_motion_command(20, 0, 0)
    assert robot.set_motion_command(30, 0, 0)

    for _ in range(20):
        robot.tick_motion()

    assert robot._hexapod._command_key == ('move_x', 30)
    assert robot.commanded_gait_state.mode == 'move_x'
    assert robot.commanded_gait_state.phase_index == 1


def test_return_to_active_command_cancels_queued_change(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.set_motion_command(10, 0, 0)
    robot.tick_motion()
    assert robot.set_motion_command(20, 0, 15)
    assert robot.set_motion_command(10, 0, 0)

    for _ in range(20):
        robot.tick_motion()

    assert all(state.mode == 'move_x' for state in states)
    assert robot._hexapod._command_key == ('move_x', 10)


def test_standby_transition_is_immediate_and_clears_pending_command(
        monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.set_motion_command(10, 0, 0)
    robot.tick_motion()
    robot.set_motion_command(20, 0, 15)
    assert robot.set_motion_command(0, 0, 0)

    robot.tick_motion()

    assert states[-1].mode == 'standby'
    assert (
        states[-1].x_level,
        states[-1].y_level,
        states[-1].z_level,
    ) == (0, 0, 0)
    assert not states[-1].replacement_pending
    assert states[-1].phase_index == 0
    assert robot._hexapod._pending_command is None


def test_leg_packet_encodes_three_signed_angles_and_runtime(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    servo = servo_module.Servo(serial)

    servo.set_leg_angles(2, (-10, 20, -30))

    assert len(serial.writes) == 1
    packet = serial.writes[0]
    assert packet[:5] == bytes([0x55, 0x00, 14, 0x01, 0x41])
    assert packet[5:11] == bytes([3, 246, 20, 226, 0, 0])
    assert packet[-3] == 255 - sum(packet[2:-3]) % 256
    assert packet[-2:] == b'\x00\xaa'


def test_leg_batch_validates_every_packet_before_serial_write(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    servo = servo_module.Servo(serial)

    with pytest.raises(ValueError, match='leg angles'):
        servo.leg_batch((
            (1, (0, 0, 0)),
            (2, (0, 91, 0)),
        ), runtime=0)

    assert serial.writes == []


def test_leg_batch_preserves_individual_packet_bytes_and_order(monkeypatch):
    disable_protocol_delays(monkeypatch)
    commands = tuple(
        (leg_id, (leg_id, -leg_id, leg_id * 2))
        for leg_id in range(1, 7)
    )
    individual_serial = FakeSerial()
    individual_servo = servo_module.Servo(individual_serial)
    for leg_id, angles in commands:
        individual_servo.leg(leg_id, angles, runtime=0)

    batch_serial = FakeSerial()
    batch_servo = servo_module.Servo(batch_serial)
    batch_servo.leg_batch(commands, runtime=0)

    assert len(individual_serial.writes) == 6
    assert len(batch_serial.writes) == 1
    assert batch_serial.writes[0] == b''.join(individual_serial.writes)


def test_single_joint_packet_remains_available_for_compatibility(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    servo = servo_module.Servo(serial)

    servo.set_angle(2, 1, -30)

    assert serial.writes[0][4:9] == bytes([0x40, 8, 226, 0, 0])


@pytest.mark.parametrize(
    ('call', 'message'),
    (
        (lambda servo: servo.motor(0, 0), 'servo_id'),
        (lambda servo: servo.motor(1, 91), 'servo angle'),
        (lambda servo: servo.leg(0, (0, 0, 0)), 'leg_id'),
        (lambda servo: servo.leg(1, (0, -91, 0)), 'leg angles'),
        (lambda servo: servo.set_angle(0, 3, 0), 'part_index'),
    ),
)
def test_servo_writer_rejects_unsafe_addresses_and_angles(call, message):
    serial = FakeSerial()
    servo = servo_module.Servo(serial, write_delay=0.0)

    with pytest.raises(ValueError, match=message):
        call(servo)

    assert serial.writes == []


def test_read_motor_decodes_signed_joint_angles(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    expected = list(range(-9, 9))
    serial.read_buffer = response_packet(
        MutoLibCore.COMMANDS['MOTOR_ANGLE'],
        [value & 0xFF for value in expected],
    )

    assert robot.read_motor() == expected


def test_motor_snapshot_pairs_feedback_with_locked_gait_state(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    robot.set_motion_command(10, 0, 0)
    robot.tick_motion()
    expected = list(range(-9, 9))
    serial.read_buffer = response_packet(
        MutoLibCore.COMMANDS['MOTOR_ANGLE'],
        [value & 0xFF for value in expected],
    )

    state, angles = robot.read_motor_with_gait_state()

    assert state.sequence == 1
    assert state.phase_index == 1
    assert angles == expected


def test_serial_read_returns_without_fixed_sleep_when_packet_is_ready(
        monkeypatch):
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    serial.read_buffer = response_packet(
        MutoLibCore.COMMANDS['MOTOR_ANGLE'], [0] * 18)
    sleeps = []
    monkeypatch.setattr(MutoLibCore.time, 'sleep', sleeps.append)

    assert robot.read_motor() == [0] * 18
    assert sleeps == []


def test_imu_read_rejects_nonfinite_or_negative_timeout():
    robot = Muto(serial_port=FakeSerial())

    for invalid in (float('nan'), float('inf'), -0.001):
        with pytest.raises(ValueError, match='response_timeout'):
            robot.read_IMU_Raw(response_timeout=invalid)
        with pytest.raises(ValueError, match='response_timeout'):
            robot.read_IMU(response_timeout=invalid)


def test_response_parser_accepts_data_return_frame_when_valid():
    robot = Muto(serial_port=FakeSerial())
    payload = [0] * 18
    packet_length = len(payload) + 8
    address = MutoLibCore.COMMANDS['MOTOR_ANGLE']
    checksum = 255 - (
        packet_length
        + MutoLibCore.DATA_RETURN_COMMAND
        + address
        + sum(payload)
    ) % 256
    vendor_type_packet = bytes([
        MutoLibCore.HEADER,
        MutoLibCore.DEVICE_ID,
        packet_length,
        MutoLibCore.DATA_RETURN_COMMAND,
        address,
        *payload,
        checksum,
        *MutoLibCore.TRAILER,
    ])

    assert robot._extract_payload(
        vendor_type_packet,
        address,
        expected_payload_length=18,
    ) == bytes(payload)
    assert robot.last_response_type == MutoLibCore.DATA_RETURN_COMMAND


def test_response_parser_rejects_write_or_read_request_frame_types():
    robot = Muto(serial_port=FakeSerial())
    address = MutoLibCore.COMMANDS['IMU_RAW']
    for response_type in (
            MutoLibCore.WRITE_COMMAND, MutoLibCore.READ_COMMAND):
        packet = response_packet(
            address, [0] * 18, response_type=response_type)
        assert robot._extract_payload(
            packet,
            address,
            expected_payload_length=18,
        ) is None


def test_response_parser_rejects_wrong_payload_length():
    robot = Muto(serial_port=FakeSerial())
    packet = response_packet(
        MutoLibCore.COMMANDS['IMU_RAW'], [0],
        response_type=MutoLibCore.DATA_RETURN_COMMAND,
    )

    assert robot._extract_payload(
        packet,
        MutoLibCore.COMMANDS['IMU_RAW'],
        expected_payload_length=18,
    ) is None


def test_failed_read_reports_whether_controller_returned_bytes(monkeypatch):
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    timestamps = iter((0.0, 0.1))
    monkeypatch.setattr(MutoLibCore.time, 'monotonic', lambda: next(timestamps))

    assert robot.read_IMU_Raw() is None
    assert robot.last_read_diagnostic == (
        'no bytes received for address 0x61 within 0.050s')


def test_read_imu_raw_decodes_big_endian_words(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    expected = (-1000, -20, 0, 20, 1000, 2000, -2000, 32767, -32768)
    payload = b''.join(struct.pack('>h', value) for value in expected)
    serial.read_buffer = response_packet(
        MutoLibCore.COMMANDS['IMU_RAW'],
        payload,
    )

    assert robot.read_IMU_Raw() == list(expected)


def test_read_fused_imu_decodes_documented_attitude_endpoint(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    payload = struct.pack('>hhhB', -1234, 25, 17999, 42)
    serial.read_buffer = response_packet(
        MutoLibCore.COMMANDS['ATTITUDE_ANGLE'],
        payload,
    )

    assert robot.read_IMU() == [-12.34, 0.25, 179.99, 42]
    assert serial.writes == [bytes([
        0x55, 0x00, 0x09, 0x02, 0x60, 0x07, 0x8D, 0x00, 0xAA,
    ])]


def test_buzzer_and_torque_packets_retain_vendor_addresses(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)

    robot.buzzer(255)
    robot.Servo_torque_off(3)

    assert serial.writes[0][4:6] == bytes([
        MutoLibCore.COMMANDS['BUZZER'],
        255,
    ])
    assert serial.writes[1][4:6] == bytes([
        MutoLibCore.COMMANDS['TORQUE_OFF'],
        3,
    ])
