# muto_hexapod_lib_custom

This ROS 2 `ament_python` package vendors the part of Yahboom's Muto library
used by this workspace. It provides:

- the `/dev/myserial` protocol used by the Muto driver;
- the generated 20-step walking and turning trajectories;
- a latched-command, one-phase-per-tick locomotion API;
- leg inverse kinematics and one vendor `LEG` packet per leg and phase; and
- a callback containing the exact commanded foot targets and nominal
  stance/swing classification for every emitted gait step.

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

Changing a nonzero command updates the trajectory amplitude without resetting
its phase. Transitioning to zero selects the one-step standby trajectory. The
first standby tick always sends all 18 joint targets because motor position is
unknown when a process starts; later identical standby ticks still emit gait
state but do not resend unchanged servo targets.

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
loop. Each six-packet phase is protected as one serial transaction, command
updates use the same lock, and `read_motor_with_gait_state()` holds that lock
while pairing a gait snapshot with joint feedback. A sensor read therefore
cannot be inserted between legs and a phase cannot change during the paired
read if the caller later becomes multithreaded.

The angular input is a vendor gait level, not a calibrated angular velocity.
It is constrained to `[-20, -10]`, zero, or `[10, 20]`. The ROS driver accepts
`rad/s` commands but the physical mapping must be measured on the robot; code
must not interpret one angular level as `0.01 rad/s`.

The inherited combined gait uses forward `x` and yaw `z`; it does not apply
lateral `y` while yaw is nonzero. Pure lateral motion remains supported. The
current Nav2 configuration commands `linear.y = 0`, but a holonomic teleop
interface must not assume simultaneous lateral translation and turning works.

The runtime subset is intentionally narrower than the upstream distribution.
It supports motion command/tick operations, `read_motor`, `read_IMU_Raw`,
`buzzer`, and joint torque enable/disable because those are the operations used
by the ROS driver.
