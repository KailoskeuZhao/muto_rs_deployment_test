"""Muto leg coordinate transforms and inverse kinematics."""

import math

from .base import point3d
from .config import (
    leg_joint1_2joint2,
    leg_joint2_2joint3,
    leg_joint3_2tip,
    leg_mount_left_right_x,
    leg_mount_other_x,
    leg_mount_other_y,
    leg_root2joint1,
)
from .math_utils import (
    rotate0,
    rotate135,
    rotate180,
    rotate225,
    rotate315,
    rotate45,
)


LOCAL_ROTATIONS = (rotate315, rotate0, rotate45, rotate135, rotate180, rotate225)
WORLD_ROTATIONS = (rotate45, rotate0, rotate315, rotate225, rotate180, rotate135)
MOUNT_POSITIONS = (
    point3d(leg_mount_other_x, leg_mount_other_y, 0.0),
    point3d(leg_mount_left_right_x, 0.0, 0.0),
    point3d(leg_mount_other_x, -leg_mount_other_y, 0.0),
    point3d(-leg_mount_other_x, -leg_mount_other_y, 0.0),
    point3d(-leg_mount_left_right_x, 0.0, 0.0),
    point3d(-leg_mount_other_x, leg_mount_other_y, 0.0),
)


def servo_angles_to_foot_positions(servo_angles_deg):
    """Convert 18 calibrated logical motor angles to vendor-body feet.

    Firmware calibration maps logical zero onto each physical servo center.
    Values returned by read_motor() are consumed directly; raw calibration
    offsets must not be applied again here.
    """
    angles = tuple(float(value) for value in servo_angles_deg)
    if len(angles) != 18:
        raise ValueError('servo angle input must contain 18 values')
    if any(not math.isfinite(value) for value in angles):
        raise ValueError('servo angles must be finite')
    if any(abs(value) > 90.0 for value in angles):
        raise ValueError('logical servo angles must be within [-90, 90]')

    return tuple(
        servo_angles_to_foot_position(
            leg_index,
            angles[leg_index * 3:(leg_index + 1) * 3],
        )
        for leg_index in range(6)
    )


def servo_angles_to_foot_position(leg_index, servo_angles_deg):
    """Convert one leg's logical yaw/pitch/knee angles to a body point."""
    if leg_index < 0 or leg_index >= 6:
        raise ValueError('leg_index must be in [0, 5]')
    if len(servo_angles_deg) != 3:
        raise ValueError('one leg requires three servo angles')

    yaw_command, pitch_command, knee_command = servo_angles_deg
    joint_yaw = math.radians(yaw_command)
    joint_pitch = math.radians(-pitch_command)
    joint_knee = math.radians(knee_command)

    distal_angle = joint_pitch + joint_knee - math.pi / 2.0
    planar_reach = (
        leg_joint1_2joint2
        + leg_joint2_2joint3 * math.cos(joint_pitch)
        + leg_joint3_2tip * math.cos(distal_angle)
    )
    local_point = point3d(
        leg_root2joint1 + planar_reach * math.cos(joint_yaw),
        planar_reach * math.sin(joint_yaw),
        leg_joint2_2joint3 * math.sin(joint_pitch)
        + leg_joint3_2tip * math.sin(distal_angle),
    )
    world_point = (
        WORLD_ROTATIONS[leg_index](local_point)
        + MOUNT_POSITIONS[leg_index]
    )
    return world_point.as_tuple()


class RealLeg:
    def __init__(self, leg_index, servo):
        if leg_index < 0 or leg_index >= 6:
            raise ValueError('leg_index must be in [0, 5]')
        self._leg_index = leg_index
        self._servo = servo
        self._mount_position = MOUNT_POSITIONS[leg_index]
        self._local_rotation = LOCAL_ROTATIONS[leg_index]
        self._world_rotation = WORLD_ROTATIONS[leg_index]
        # Motor position is unknown until this process sends its first target.
        # Assuming standby here caused the initial standby command to be
        # optimized away even when the physical leg was left in another pose.
        self._tip_position = None

    def translate_to_local(self, world_point):
        return self._local_rotation(world_point - self._mount_position)

    def inverse_kinematics(self, target_point):
        x = target_point.x - leg_root2joint1
        y = target_point.y
        angle0 = math.degrees(math.atan2(y, x))

        x = math.sqrt(x * x + y * y) - leg_joint1_2joint2
        y = target_point.z
        angle_to_target = math.atan2(y, x)
        radius_squared = x * x + y * y
        radius = math.sqrt(radius_squared)
        if radius <= 1e-9:
            raise ValueError('invalid leg target at joint origin')

        first = self._clamp_acos(
            (radius_squared + leg_joint2_2joint3 ** 2 - leg_joint3_2tip ** 2)
            / (2 * leg_joint2_2joint3 * radius)
        )
        second = self._clamp_acos(
            (radius_squared - leg_joint2_2joint3 ** 2 + leg_joint3_2tip ** 2)
            / (2 * leg_joint3_2tip * radius)
        )
        angle1 = math.degrees(angle_to_target + math.acos(first))
        angle2 = 90.0 - math.degrees(math.acos(first) + math.acos(second))
        return (angle0, angle1, angle2)

    def move_tip(self, target_world):
        if (
                self._tip_position is not None
                and target_world == self._tip_position):
            return
        target_local = self.translate_to_local(target_world)
        angles = self.inverse_kinematics(target_local)
        # The controller accepts whole logical degrees.  Nearest-degree
        # quantization avoids the systematic toward-zero stride loss caused by
        # int() truncation while retaining the firmware's integer interface.
        servo_angles = (
            int(round(angles[0])),
            int(round(-angles[1])),
            int(round(angles[2])),
        )
        self._servo.set_leg_angles(self._leg_index, servo_angles)
        self._tip_position = target_world

    @staticmethod
    def _clamp_acos(value):
        return max(-1.0, min(1.0, value))
