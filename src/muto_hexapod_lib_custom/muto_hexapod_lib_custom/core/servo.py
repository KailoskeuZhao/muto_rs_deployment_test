"""Low-level bus-servo packet writer."""

import struct
import time


HEADER = 0x55
DEVICE_ID = 0x00
WRITE_COMMAND = 0x01
MOTOR_ADDRESS = 0x40
LEG_ADDRESS = 0x41
TRAILER = (0x00, 0xAA)


class Servo:
    def __init__(self, serial_port, write_delay=0.001):
        self._serial = serial_port
        self._write_delay = max(float(write_delay), 0.0)

    @staticmethod
    def _command_packet(address, values):
        values = [int(value) & 0xFF for value in values]
        packet_length = len(values) + 0x08
        checksum = 255 - (
            packet_length + WRITE_COMMAND + address + sum(values)
        ) % 256
        return bytes(
            [HEADER, DEVICE_ID, packet_length, WRITE_COMMAND, address]
            + values
            + [checksum, *TRAILER]
        )

    def _write_packets(self, packets):
        packets = tuple(packets)
        if not packets:
            return
        # The controller protocol is a byte stream of self-delimiting frames.
        # Joining a gait phase here retains every frame byte and its order while
        # avoiding six Python/serial writes and six inter-frame sleeps.
        self._serial.write(b''.join(packets))
        if self._write_delay:
            time.sleep(self._write_delay)

    def _write_command(self, address, values):
        self._write_packets((self._command_packet(address, values),))

    @classmethod
    def _leg_packet(cls, leg_id, angles, runtime):
        leg_id = int(leg_id)
        if leg_id < 1 or leg_id > 6:
            raise ValueError('leg_id must be in [1, 6]')
        if len(angles) != 3:
            raise ValueError('angles must contain exactly three values')
        angles = tuple(int(angle) for angle in angles)
        if any(angle < -90 or angle > 90 for angle in angles):
            raise ValueError('leg angles must be in [-90, 90]')
        runtime_bytes = struct.pack(
            '>h', max(0, min(2000, int(runtime))))
        return cls._command_packet(LEG_ADDRESS, [
            leg_id,
            *(angle & 0xFF for angle in angles),
            runtime_bytes[0],
            runtime_bytes[1],
        ])

    def motor(self, servo_id, angle, runtime=100):
        """Command one joint through the vendor ``MOTOR`` address."""
        servo_id = int(servo_id)
        angle = int(angle)
        if servo_id < 1 or servo_id > 18:
            raise ValueError('servo_id must be in [1, 18]')
        if angle < -90 or angle > 90:
            raise ValueError('servo angle must be in [-90, 90]')
        runtime_bytes = struct.pack(
            '>h', max(0, min(2000, int(runtime))))
        self._write_command(MOTOR_ADDRESS, [
            servo_id,
            angle & 0xFF,
            runtime_bytes[0],
            runtime_bytes[1],
        ])

    def leg(self, leg_id, angles, runtime=100):
        """Command one leg's three joints in one vendor ``LEG`` packet."""
        self._write_packets((self._leg_packet(leg_id, angles, runtime),))

    def leg_batch(self, commands, runtime=100):
        """Command multiple legs in one contiguous serial write.

        ``commands`` contains ``(leg_id, angles)`` pairs using the same
        one-based leg IDs and angle validation as :meth:`leg`.  All packets are
        built before the write so an invalid command cannot emit a partial
        gait phase.
        """
        packets = tuple(
            self._leg_packet(leg_id, angles, runtime)
            for leg_id, angles in commands
        )
        self._write_packets(packets)

    def set_angle(self, leg_index, part_index, kinematic_angle):
        if leg_index < 0 or leg_index >= 6:
            raise ValueError('leg_index must be in [0, 5]')
        if part_index < 0 or part_index >= 3:
            raise ValueError('part_index must be in [0, 2]')
        servo_id = leg_index * 3 + part_index + 1
        self.motor(servo_id, int(kinematic_angle), 0)

    def set_leg_angles(self, leg_index, kinematic_angles):
        """Command all three logical joint angles for a zero-based leg."""
        if leg_index < 0 or leg_index >= 6:
            raise ValueError('leg_index must be in [0, 5]')
        self.leg(
            leg_index + 1,
            tuple(int(angle) for angle in kinematic_angles),
            0,
        )

    def set_leg_angles_batch(self, commands):
        """Command zero-based ``(leg_index, angles)`` pairs as one phase."""
        normalized = []
        for leg_index, kinematic_angles in commands:
            if leg_index < 0 or leg_index >= 6:
                raise ValueError('leg_index must be in [0, 5]')
            normalized.append((
                leg_index + 1,
                tuple(int(angle) for angle in kinematic_angles),
            ))
        self.leg_batch(normalized, 0)
