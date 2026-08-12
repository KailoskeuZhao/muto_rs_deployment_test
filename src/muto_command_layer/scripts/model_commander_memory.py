"""Pure mission-memory records and pose accounting for the model commander."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotPoseSnapshot:
    """Latest robot odometry pose included in high-level decision memory."""

    frame_id: str
    child_frame_id: str
    stamp_seconds: float
    receipt_monotonic: float
    x: float
    y: float
    z: float
    yaw: float


def angle_delta(end_yaw, start_yaw):
    """Return the wrapped signed yaw change in radians."""
    return math.atan2(
        math.sin(end_yaw - start_yaw),
        math.cos(end_yaw - start_yaw),
    )


def pose_delta_context(start_pose, end_pose):
    """Summarize measured translation and rotation between two pose records."""
    if start_pose is None or end_pose is None:
        return None
    dx = float(end_pose['x']) - float(start_pose['x'])
    dy = float(end_pose['y']) - float(start_pose['y'])
    dz = float(end_pose['z']) - float(start_pose['z'])
    dyaw = angle_delta(
        float(end_pose['yaw_rad']),
        float(start_pose['yaw_rad']),
    )
    return {
        'dx': round(dx, 4),
        'dy': round(dy, 4),
        'dz': round(dz, 4),
        'distance_xy': round(math.hypot(dx, dy), 4),
        'dyaw_rad': round(dyaw, 6),
        'dyaw_abs_rad': round(abs(dyaw), 6),
    }


def primitive_memory_entry(
        max_message_characters, primitive, outcome, message, world_revision,
        started_pose, ended_pose, **fields):
    """Build the bounded record supplied to later planning decisions."""
    entry = {
        'primitive': primitive,
        'outcome': outcome,
        'message': message[:max_message_characters],
        'world_revision': world_revision,
        'started_pose': started_pose,
        'ended_pose': ended_pose,
        'delta_pose': pose_delta_context(started_pose, ended_pose),
    }
    for key, value in fields.items():
        if value is not None:
            entry[key] = value
    return entry
