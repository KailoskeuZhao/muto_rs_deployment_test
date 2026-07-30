"""Focused Muto serial protocol used by the ROS hardware driver."""

import struct
import threading
import time

import serial

from .hexapod import Hexapod


__version__ = '1.2.3-ros1'


WRITE_COMMAND = 0x01
READ_COMMAND = 0x02
DEVICE_ID = 0x00
HEADER = 0x55
TRAILER = (0x00, 0xAA)

COMMANDS = {
    'BUZZER': 0x18,
    'TORQUE_ON': 0x26,
    'TORQUE_OFF': 0x27,
    'MOTOR_ANGLE': 0x50,
    'IMU_RAW': 0x61,
}


class Muto:
    """Muto serial device plus the host-generated gait used by this workspace."""

    def __init__(
        self,
        port='/dev/myserial',
        debug=False,
        speed_mapping=None,
        gait_step_callback=None,
        serial_port=None,
    ):
        del speed_mapping  # Retained for constructor compatibility.
        self.ser = serial_port or serial.Serial(port, 115200, timeout=0.05)
        self._debug = bool(debug)
        self._serial_lock = threading.RLock()
        self._hexapod = Hexapod(self, gait_step_callback=gait_step_callback)

    @property
    def commanded_gait_state(self):
        return self._hexapod.commanded_gait_state

    def set_gait_step_callback(self, callback):
        self._hexapod.set_gait_step_callback(callback)

    def write(self, data):
        """Serialize gait and protocol writes through one serial connection."""
        with self._serial_lock:
            return self.ser.write(bytes(data))

    def close(self):
        with self._serial_lock:
            if getattr(self.ser, 'is_open', True):
                self.ser.close()

    def buzzer(self, timeout):
        timeout = max(0, min(255, int(timeout)))
        self._send(COMMANDS['BUZZER'], [timeout])

    def Servo_torque_on(self, servo_id=0):
        self._set_servo_torque(COMMANDS['TORQUE_ON'], servo_id)

    def Servo_torque_off(self, servo_id=0):
        self._set_servo_torque(COMMANDS['TORQUE_OFF'], servo_id)

    def move(self, x, y, z):
        x_level = max(-30, min(30, int(x)))
        y_level = max(-30, min(30, int(y)))
        z_level = max(-20, min(20, int(z)))
        if z_level != 0 and abs(z_level) < 10:
            z_level = 10 if z_level > 0 else -10
        self._hexapod.move(x_level, y_level, z_level)

    def read_motor(self):
        payload = self._read(COMMANDS['MOTOR_ANGLE'], 18, 0.1)
        if payload is None or len(payload) < 18:
            return None
        return [struct.unpack('b', payload[index:index + 1])[0] for index in range(18)]

    def read_IMU_Raw(self):
        payload = self._read(COMMANDS['IMU_RAW'], 18, 0.05)
        if payload is None or len(payload) < 18:
            return None
        return [
            struct.unpack('>h', payload[index:index + 2])[0]
            for index in range(0, 18, 2)
        ]

    def _set_servo_torque(self, command, servo_id):
        servo_id = int(servo_id)
        if servo_id < 0 or servo_id > 18:
            return
        self._send(command, [0xFE if servo_id == 0 else servo_id])

    def _send(self, address, values):
        values = [int(value) & 0xFF for value in values]
        packet_length = len(values) + 0x08
        checksum = 255 - (
            packet_length + WRITE_COMMAND + address + sum(values)
        ) % 256
        packet = bytes(
            [HEADER, DEVICE_ID, packet_length, WRITE_COMMAND, address]
            + values
            + [checksum, *TRAILER]
        )
        self.write(packet)
        if self._debug:
            print('send:', list(packet))
        time.sleep(0.001)

    def _read(self, address, parameter, response_delay):
        with self._serial_lock:
            self._reset_input_buffer()
            packet_length = 0x09
            checksum = 255 - (
                packet_length + READ_COMMAND + address + parameter
            ) % 256
            packet = bytes([
                HEADER,
                DEVICE_ID,
                packet_length,
                READ_COMMAND,
                address,
                parameter,
                checksum,
                *TRAILER,
            ])
            self.ser.write(packet)
            if self._debug:
                print('read:', list(packet))
            time.sleep(response_delay)
            data = self._read_available()
        return self._extract_payload(data, address)

    def _reset_input_buffer(self):
        if hasattr(self.ser, 'reset_input_buffer'):
            self.ser.reset_input_buffer()
        elif hasattr(self.ser, 'flushInput'):
            self.ser.flushInput()

    def _read_available(self):
        if hasattr(self.ser, 'read_all'):
            return bytes(self.ser.read_all())
        waiting = getattr(self.ser, 'in_waiting', 0)
        if callable(waiting):
            waiting = waiting()
        return bytes(self.ser.read(waiting)) if waiting else b''

    def _extract_payload(self, data, expected_address):
        for start in range(max(0, len(data) - 8)):
            if data[start:start + 2] != bytes([HEADER, DEVICE_ID]):
                continue
            if start + 3 > len(data):
                continue
            packet_length = data[start + 2]
            end = start + packet_length
            if packet_length < 8 or end > len(data):
                continue
            packet = data[start:end]
            if packet[-2:] != bytes(TRAILER):
                continue
            if packet[4] != expected_address:
                continue
            expected_checksum = 255 - sum(packet[2:-3]) % 256
            if packet[-3] != expected_checksum:
                continue
            payload = packet[5:-3]
            if self._debug:
                print('receive:', list(packet))
            return payload
        if self._debug and data:
            print('invalid receive:', list(data))
        return None
