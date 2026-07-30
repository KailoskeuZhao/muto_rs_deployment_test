"""Low-level bus-servo packet writer."""

import struct
import time


class Servo:
    def __init__(self, serial_port, write_delay=0.001):
        self._serial = serial_port
        self._write_delay = max(float(write_delay), 0.0)

    def motor(self, servo_id, angle, runtime=100):
        runtime_bytes = struct.pack('>h', int(runtime))
        values = [
            int(servo_id),
            int(angle) & 0xFF,
            runtime_bytes[0],
            runtime_bytes[1],
        ]
        packet_length = len(values) + 0x08
        checksum = 255 - (
            packet_length + 0x01 + 0x40 + sum(values)
        ) % 256
        packet = bytes(
            [0x55, 0x00, packet_length, 0x01, 0x40]
            + values
            + [checksum, 0x00, 0xAA]
        )
        self._serial.write(packet)
        if self._write_delay:
            time.sleep(self._write_delay)

    def set_angle(self, leg_index, part_index, kinematic_angle):
        servo_id = leg_index * 3 + part_index + 1
        self.motor(servo_id, int(kinematic_angle), 0)
