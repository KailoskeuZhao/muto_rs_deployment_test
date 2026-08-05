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

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def reset_input_buffer(self):
        pass

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


def response_packet(address, payload, response_type=MutoLibCore.READ_COMMAND):
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


def test_move_emits_one_callback_per_vendor_gait_step(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.move(10, 0, 0)

    leg_packets = [packet for packet in serial.writes if packet[4] == 0x41]
    assert len(states) == 20
    assert len(leg_packets) == 20 * 6
    assert states[-1].phase_index == 0
    assert states[-1].cycle_complete
    assert all(packet[:2] == b'\x55\x00' for packet in leg_packets)
    assert all(packet[2] == 14 for packet in leg_packets)
    assert all(packet[-2:] == b'\x00\xaa' for packet in leg_packets)
    for phase_start in range(0, len(leg_packets), 6):
        assert [
            packet[5] for packet in leg_packets[phase_start:phase_start + 6]
        ] == list(range(1, 7))


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

    leg_packets = [packet for packet in serial.writes if packet[4] == 0x41]
    assert len(states) == 1
    assert states[0].phase_index == 1
    assert len(leg_packets) == 6


def test_latched_phase_holds_one_outer_serial_transaction(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    robot = Muto(serial_port=serial)
    robot.set_motion_command(10, 0, 0)
    lock = RecordingRLock()
    robot._serial_lock = lock

    robot.tick_motion()

    # One outer phase lock plus one nested re-entrant write lock per leg.
    assert lock.enter_count == 7
    assert lock.max_depth == 2
    assert lock.depth == 0


def test_initial_standby_tick_commands_hardware_then_heartbeats(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.set_motion_command(0, 0, 0)
    robot.tick_motion()
    first_packet_count = len(serial.writes)
    robot.tick_motion()

    assert first_packet_count == 6
    assert len(serial.writes) == first_packet_count
    assert [state.sequence for state in states] == [1, 2]
    assert all(state.mode == 'standby' for state in states)


def test_nonzero_command_updates_do_not_restart_the_gait(monkeypatch):
    disable_protocol_delays(monkeypatch)
    serial = FakeSerial()
    states = []
    robot = Muto(serial_port=serial, gait_step_callback=states.append)

    robot.set_motion_command(10, 0, 0)
    robot.tick_motion()
    robot.set_motion_command(20, 0, 0)
    robot.tick_motion()

    assert [state.phase_index for state in states] == [1, 2]
    assert [state.sequence for state in states] == [1, 2]


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


def test_response_parser_accepts_vendor_response_type_when_frame_is_valid():
    robot = Muto(serial_port=FakeSerial())
    payload = [0] * 18
    packet_length = len(payload) + 8
    address = MutoLibCore.COMMANDS['MOTOR_ANGLE']
    checksum = 255 - (
        packet_length
        + MutoLibCore.WRITE_COMMAND
        + address
        + sum(payload)
    ) % 256
    vendor_type_packet = bytes([
        MutoLibCore.HEADER,
        MutoLibCore.DEVICE_ID,
        packet_length,
        MutoLibCore.WRITE_COMMAND,
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
    assert robot.last_response_type == MutoLibCore.WRITE_COMMAND


def test_response_parser_rejects_wrong_payload_length():
    robot = Muto(serial_port=FakeSerial())
    packet = response_packet(
        MutoLibCore.COMMANDS['IMU_RAW'], [0],
        response_type=MutoLibCore.WRITE_COMMAND,
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
