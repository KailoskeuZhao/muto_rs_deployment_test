"""Focused Muto serial protocol used by the ROS hardware driver."""

import math
import struct
import threading
import time

import serial

from .hexapod import Hexapod


__version__ = '0.3.0'


WRITE_COMMAND = 0x01
READ_COMMAND = 0x02
DATA_RETURN_COMMAND = 0x12
DEVICE_ID = 0x00
HEADER = 0x55
TRAILER = (0x00, 0xAA)
DEFAULT_IMU_RESPONSE_TIMEOUT_SEC = 0.05

COMMANDS = {
    'BUZZER': 0x18,
    'TORQUE_ON': 0x26,
    'TORQUE_OFF': 0x27,
    'MOTOR_ANGLE': 0x50,
    'ATTITUDE_ANGLE': 0x60,
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
        batch_gait_phase_writes=True,
    ):
        del speed_mapping  # Retained for constructor compatibility.
        self.ser = serial_port or serial.Serial(port, 115200, timeout=0.05)
        self._debug = bool(debug)
        self._serial_lock = threading.RLock()
        self._hexapod = Hexapod(
            self,
            gait_step_callback=gait_step_callback,
            batch_gait_phase_writes=batch_gait_phase_writes,
        )
        self.last_read_diagnostic = 'no serial read attempted'
        self.last_response_type = None

    @property
    def commanded_gait_state(self):
        with self._serial_lock:
            return self._hexapod.commanded_gait_state

    def set_gait_step_callback(self, callback):
        self._hexapod.set_gait_step_callback(callback)

    def write(self, data):
        """Serialize one write and wait until its bytes leave the host queue.

        A batched 84-byte gait phase occupies about 7.3 ms at 115200 baud.
        Draining it here prevents a following timed IMU request from spending
        most of its response budget behind gait bytes still queued by Linux.
        """
        with self._serial_lock:
            packet = bytes(data)
            result = self.ser.write(packet)
            if result is not None and result != len(packet):
                raise IOError(
                    f'partial serial write: {result}/{len(packet)} bytes')
            flush = getattr(self.ser, 'flush', None)
            if callable(flush):
                flush()
            return result

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
        """Compatibility call that sends one complete gait cycle."""
        x_level, y_level, z_level = self._normalize_motion_command(x, y, z)
        with self._serial_lock:
            return self._hexapod.move(x_level, y_level, z_level)

    def set_motion_command(self, x, y, z):
        """Latch motion levels for subsequent :meth:`tick_motion` calls."""
        x_level, y_level, z_level = self._normalize_motion_command(x, y, z)
        with self._serial_lock:
            return self._hexapod.set_command(x_level, y_level, z_level)

    def tick_motion(self):
        """Advance the latched gait by one atomic six-leg trajectory phase."""
        # The current ROS driver uses a single-threaded executor, but keep the
        # six packets belonging to one phase together even if that changes.
        # ``write()`` takes the same RLock for each nested packet write.
        with self._serial_lock:
            result = self._hexapod.tick(notify=False)
        # ROS publication and other observers do not belong to the serial
        # transaction. Keeping them outside the lock prevents future executor
        # concurrency from extending gait bus ownership.
        self._hexapod.notify_gait_step(result[1])
        return result

    @staticmethod
    def _normalize_motion_command(x, y, z):
        x_level = max(-30, min(30, int(x)))
        y_level = max(-30, min(30, int(y)))
        z_level = max(-20, min(20, int(z)))
        # The inherited library forced every non-zero turn to level 10.  The
        # custom exact-SE(2) gait has useful whole-degree servo changes from
        # level 2 onward; level 1 quantizes to the same packets as zero.
        if z_level != 0 and abs(z_level) < 2:
            z_level = 2 if z_level > 0 else -2
        return x_level, y_level, z_level

    def read_motor(self):
        payload = self._read(COMMANDS['MOTOR_ANGLE'], 18, 0.1)
        if payload is None or len(payload) < 18:
            return None
        return [struct.unpack('b', payload[index:index + 1])[0] for index in range(18)]

    def read_motor_with_gait_state(self):
        """Read joints and return the immutable gait state held for that read."""
        with self._serial_lock:
            state = self._hexapod.commanded_gait_state
            return state, self.read_motor()

    def read_IMU_Raw(self, response_timeout=None):
        """Read one controller-exported IMU snapshot.

        ``response_timeout`` bounds how long this synchronous transaction may
        hold the shared gait/telemetry serial bus.  The vendor-compatible
        default remains 50 ms; the ROS driver supplies a shorter deadline-aware
        budget while locomotion is active.
        """
        if response_timeout is None:
            response_timeout = DEFAULT_IMU_RESPONSE_TIMEOUT_SEC
        response_timeout = float(response_timeout)
        if not math.isfinite(response_timeout) or response_timeout < 0.0:
            raise ValueError(
                'response_timeout must be finite and non-negative')
        payload = self._read(
            COMMANDS['IMU_RAW'], 18, response_timeout)
        if payload is None or len(payload) < 18:
            return None
        return [
            struct.unpack('>h', payload[index:index + 2])[0]
            for index in range(0, 18, 2)
        ]

    def read_IMU(self, response_timeout=None):
        """Read the controller's fused roll, pitch, yaw, and temperature.

        This is the documented baseboard ``0x60`` endpoint. Angles are
        returned in degrees using the controller's signed centidegree scale;
        temperature is the unsigned byte supplied by the controller.
        """
        if response_timeout is None:
            response_timeout = DEFAULT_IMU_RESPONSE_TIMEOUT_SEC
        response_timeout = float(response_timeout)
        if not math.isfinite(response_timeout) or response_timeout < 0.0:
            raise ValueError(
                'response_timeout must be finite and non-negative')
        payload = self._read(
            COMMANDS['ATTITUDE_ANGLE'], 7, response_timeout)
        if payload is None or len(payload) < 7:
            return None
        roll, pitch, yaw = struct.unpack('>hhh', payload[:6])
        return [roll / 100.0, pitch / 100.0, yaw / 100.0, payload[6]]

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

    def _read(self, address, parameter, response_timeout):
        with self._serial_lock:
            self.last_response_type = None
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
            self.write(packet)
            if self._debug:
                print('read:', list(packet))
            deadline = time.monotonic() + max(0.0, response_timeout)
            data = bytearray()
            while True:
                data.extend(self._read_available())
                payload = self._extract_payload(
                    bytes(data), address, expected_payload_length=parameter)
                if payload is not None:
                    self.last_read_diagnostic = (
                        f'ok: received {len(data)} bytes, response_type='
                        f'0x{self.last_response_type:02x}'
                    )
                    return payload
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    if data:
                        raw_preview = bytes(data[:64]).hex(' ')
                        self.last_read_diagnostic = (
                            f'received {len(data)} bytes within '
                            f'{response_timeout:.3f}s but no valid frame for '
                            f'address 0x{address:02x} with {parameter} payload '
                            f'bytes; raw={raw_preview}'
                        )
                    else:
                        self.last_read_diagnostic = (
                            f'no bytes received for address 0x{address:02x} '
                            f'within {response_timeout:.3f}s'
                        )
                    return None
                time.sleep(min(0.001, remaining))

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

    def _extract_payload(
        self, data, expected_address, expected_payload_length=None
    ):
        # Eight bytes is the shortest valid frame. Include the last possible
        # start offset so a minimal frame or a frame after leading noise is not
        # skipped.
        for start in range(max(0, len(data) - 7)):
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
            if packet[3] != DATA_RETURN_COMMAND:
                continue
            if packet[4] != expected_address:
                continue
            expected_checksum = 255 - sum(packet[2:-3]) % 256
            if packet[-3] != expected_checksum:
                continue
            payload = packet[5:-3]
            if (
                expected_payload_length is not None and
                len(payload) != expected_payload_length
            ):
                continue
            # Muto's published baseboard protocol defines 0x12 as the data
            # return instruction. It is distinct from the 0x02 read request.
            self.last_response_type = packet[3]
            if self._debug:
                print('receive:', list(packet))
            return payload
        if self._debug and data:
            print('invalid receive:', list(data))
        return None
