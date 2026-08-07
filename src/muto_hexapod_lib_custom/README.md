# muto_hexapod_lib_custom

This ROS 2 `ament_python` package vendors the part of Yahboom's Muto library
used by this workspace. It provides:

- the `/dev/myserial` protocol used by the Muto driver;
- the generated 20-step walking and turning trajectories;
- a latched-command, one-phase-per-tick locomotion API;
- leg inverse kinematics and one vendor `LEG` packet per leg and phase; and
- a callback containing the continuous pre-quantization foot targets and
  nominal stance/swing classification for every emitted gait step.

The stance classification is command-derived. It does not measure foot
contact, load, or slip.

The compatibility import remains:

```python
from muto_hexapod_lib_custom.core.MutoLibCore import Muto
```

The ROS driver uses the time-driven API:

```python
robot.set_motion_command(x_level, y_level, yaw_level)
robot.tick_motion()  # exactly one of the 20 trajectory phases
```

Changing a nonzero command queues the latest trajectory until the next complete
20-phase boundary. This prevents a controller publishing faster than the gait
cycle from changing foot targets mid-stride. Transitioning to zero is an abrupt
safety return: the next tick directly commands the one-step nominal standby
pose, not a smooth deceleration trajectory. The first standby tick always sends
all 18 joint targets because motor position is unknown when a process starts;
later identical standby ticks still emit gait state but do not resend unchanged
servo targets.

Each Python `CommandedGaitState` reports the levels active in that emitted
phase as `x_level`, `y_level`, and `z_level`. `replacement_pending` remains
true while a newer nonzero command is waiting for the cycle boundary.

`move(...)` remains as a compatibility wrapper that sends one complete gait
cycle synchronously. New control loops should latch a command and call
`tick_motion()` at a fixed rate instead.

Serial reads poll for a complete response and return immediately when it
arrives, up to the existing IMU or motor timeout. This avoids holding the
shared serial bus for a fixed sleep on every sensor read.

Moving phases use address `0x41` to send the three joint angles of each leg in
one packet. Six 14-byte leg packets replace eighteen 12-byte single-joint
packets, reducing a moving phase from 216 to 84 serial bytes while retaining
the vendor's 115200-baud link and zero-runtime joint targets. The single-joint
`0x40` writer remains available for compatibility but is not used by the gait
loop. Continuous IK results are rounded to the nearest whole controller degree
instead of truncated toward zero. Each six-packet phase is protected as one
serial transaction, command
updates use the same lock, and `read_motor_with_gait_state()` holds that lock
while pairing a gait snapshot with joint feedback. A sensor read therefore
cannot be inserted between legs and a phase cannot change during the paired
read if the caller later becomes multithreaded.

The raw angular input is a gait level, not a calibrated angular velocity. It
is constrained to `[-20, -10]`, zero, or `[10, 20]`. The ROS driver now maps
`m/s` and `rad/s` through an explicit, sign-specific calibration profile and
publishes the selected level and feed-forward prediction on
`/muto/motion_command_state`. The installed 2026-08-06 profile is provisional:
its straight axes are geometric predictions and its yaw data is transferred
from RF2O observations of the inherited turn path, not an external measurement
of the corrected gait.

The inherited `x * sin(z)` combined gait has been removed. Forward-plus-yaw
targets now apply the exact finite planar body transform to each nominal foot:
`p_body = Exp_SE2(body_stride)^-1 * p_world`. This preserves yaw authority as
forward speed approaches zero and accounts for the unequal 229--259 mm Muto
foot radii. The existing alternating-tripod phase pattern and 25 mm lift are
retained, and pure turn uses the same transform as zero-forward mixed motion.
Pure lateral motion remains supported, but lateral motion combined with forward
or yaw is rejected; current Nav2 commands `linear.y = 0`.

The ROS-independent mapping API is available for tests and offline tools:

```python
from muto_hexapod_lib_custom.movement.velocity_calibration import (
    VelocityCalibrationMapper,
    VelocityCalibrationProfile,
)

profile = VelocityCalibrationProfile.from_mapping(profile_data)
selection = VelocityCalibrationMapper(profile).select(vx, vy, wz)
```

See `docs/locomotion_calibration.md` in the deployment workspace for the
model, provisional-profile caveats, and controlled calibration procedure.

The runtime subset is intentionally narrower than the upstream distribution.
It supports motion command/tick operations, `read_motor`, `read_IMU_Raw`,
`buzzer`, and joint torque enable/disable because those are the operations used
by the ROS driver.
