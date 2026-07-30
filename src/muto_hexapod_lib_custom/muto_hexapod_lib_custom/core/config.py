"""Mechanical dimensions and nominal Muto foot locations in millimetres."""

from .math_utils import COS15, COS30, COS45, SIN15, SIN30, SIN45


leg_mount_left_right_x = 53.4
leg_mount_other_x = 31.12
leg_mount_other_y = 83.07

mount_position = (
    (leg_mount_other_x, leg_mount_other_y, 0.0),
    (leg_mount_left_right_x, 0.0, 0.0),
    (leg_mount_other_x, -leg_mount_other_y, 0.0),
    (-leg_mount_other_x, -leg_mount_other_y, 0.0),
    (-leg_mount_left_right_x, 0.0, 0.0),
    (-leg_mount_other_x, leg_mount_other_y, 0.0),
)

leg_root2joint1 = 27.5
leg_joint1_2joint2 = 50.59
leg_joint2_2joint3 = 72.60
leg_joint3_2tip = 134.5

default_angle = (-45, 0, 45, 135, 180, 225)
angleLimitation = ((-45, 45), (-45, 75), (-60, 60))

# Logical motor angles for the factory standing pose. Firmware calibration
# maps these command-space values onto each servo's physical center.
STANDBY_SERVO_ANGLES_DEG = (0.0, -30.0, -15.0)

standby_z = leg_joint3_2tip * COS15 - leg_joint2_2joint3 * SIN30
left_right_x = (
    leg_mount_left_right_x
    + leg_root2joint1
    + leg_joint1_2joint2
    + leg_joint2_2joint3 * COS30
    + leg_joint3_2tip * SIN15
)
other_reach = (
    leg_root2joint1
    + leg_joint1_2joint2
    + leg_joint2_2joint3 * COS30
    + leg_joint3_2tip * SIN15
)
other_x = leg_mount_other_x + other_reach * COS45
other_y = leg_mount_other_y + other_reach * SIN45

k_standby = (
    (other_x, other_y, -standby_z),
    (left_right_x, 0.0, -standby_z),
    (other_x, -other_y, -standby_z),
    (-other_x, -other_y, -standby_z),
    (-left_right_x, 0.0, -standby_z),
    (-other_x, other_y, -standby_z),
)

# Names follow the ordering in Yahboom's locations type and mechanical model.
LEG_NAMES = (
    'right_front',
    'right_middle',
    'right_rear',
    'left_rear',
    'left_middle',
    'left_front',
)
