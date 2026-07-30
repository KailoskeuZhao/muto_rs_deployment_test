import struct

from muto_hexapod_lib_custom.core import MutoLibCore
from muto_hexapod_lib_custom.core import servo as servo_module
from muto_hexapod_lib_custom.core.MutoLibCore import Muto


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


def response_packet(address, payload):
    payload = bytes(payload)
    packet_length = len(payload) + 8
    checksum = 255 - (
        packet_length
        + MutoLibCore.READ_COMMAND
        + address
        + sum(payload)
    ) % 256
    return bytes([
        MutoLibCore.HEADER,
        MutoLibCore.DEVICE_ID,
        packet_length,
        MutoLibCore.READ_COMMAND,
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

    motor_packets = [packet for packet in serial.writes if packet[4] == 0x40]
    assert len(states) == 20
    assert len(motor_packets) == 20 * 18
    assert states[-1].phase_index == 0
    assert states[-1].cycle_complete
    assert all(packet[:2] == b'\x55\x00' for packet in motor_packets)
    assert all(packet[-2:] == b'\x00\xaa' for packet in motor_packets)


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
