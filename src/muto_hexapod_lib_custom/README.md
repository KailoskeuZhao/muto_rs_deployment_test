# muto_hexapod_lib_custom

This ROS 2 `ament_python` package vendors the part of Yahboom's Muto library
used by this workspace. It provides:

- the `/dev/myserial` protocol used by the Muto driver;
- the generated 20-step walking and turning trajectories;
- leg inverse kinematics and servo packet output; and
- a callback containing the exact commanded foot targets and nominal
  stance/swing classification for every emitted gait step.

The stance classification is command-derived. It does not measure foot
contact, load, or slip.

The compatibility import remains:

```python
from muto_hexapod_lib_custom.core.MutoLibCore import Muto
```

The runtime subset is intentionally narrower than the upstream distribution.
It currently supports `move`, `read_motor`, `read_IMU_Raw`, `buzzer`, and
joint torque enable/disable because those are the operations used by the ROS
driver.
