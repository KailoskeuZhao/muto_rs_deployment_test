# Muto odometry bag

This package records the hardware-originated inputs to the Muto odometry
pipeline and replays them into the normal ROS interfaces. It does not contain
an alternative odometry implementation.

During replay, the existing packages still perform every calculation:

1. `lidar_pointcloud_filter` filters the virtual raw scan.
2. The external `rf2o_laser_odometry` package produces `scan_odom_raw`.
3. The existing deadband node produces `scan_odom`.
4. `yahboomcar_bringup/foot_odometry_node` calls the virtual
   `get_motor_angles` service and produces `foot_odom`.
5. The controller-yaw adapter admits stable, stop-only heading corrections.
6. `robot_localization/ekf_node` produces `odometry/filtered` and
   `odom -> base_frame`.

## Source bag contract

| Bag topic | Type | Reason |
| --- | --- | --- |
| `/lidar/raw_laserscan` | `sensor_msgs/msg/LaserScan` | Input to the existing LiDAR filter and RF2O |
| `/imu/data_processed` | `sensor_msgs/msg/Imu` | Existing EKF IMU input |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Original IMU sample for later calibration and processing changes |
| `/imu/controller_attitude` | `muto_hexapod_interfaces_custom/msg/ControllerAttitude` | Every successful vendor-fused `0x60` roll/pitch/yaw poll plus its raw temperature byte |
| `/muto/imu_telemetry_status` | `std_msgs/msg/String` | One-second cumulative scheduler selection, deferral, attempt, success, failure, duplicate, and deadline-skip counters |
| `/muto/controller_attitude_yaw_status` | `std_msgs/msg/String` | One-second stationary-gate state, acceptance/rejection counters, and active yaw variance |
| `/muto/commanded_gait_state` | `muto_hexapod_interfaces_custom/msg/CommandedGaitState` | Backward-compatible commanded stance/swing and continuous foot targets |
| `/muto/motion_command_state` | `muto_hexapod_interfaces_custom/msg/MotionCommandState` | Requested twist, selected and active levels, pending/projection flags, prediction, and profile |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Existing RF2O deadband gate input |
| `/muto/measured_motor_state` | `std_msgs/msg/String` | Optional representation of each successful `get_motor_angles` response when motor recording is enabled |
| `/muto/odometry_test_event` | `std_msgs/msg/String` | Timestamped JSON start/end and measured field-pose markers |
| `/muto/odometry_recording_metadata` | `std_msgs/msg/String` | Recorder build git revision, dirty state, bag schema, and telemetry-status capture contract |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Exact static sensor geometry offered during recording |

The recorder does not poll `get_motor_angles` by default. Set
`record_motor_angles:=true` only for a controlled joint-feedback diagnostic;
each such service call is an additional blocking serial read while recording.
The recorder also does not poll `0x60`: the sole-owner hardware driver creates
`/imu/controller_attitude`, and this package only subscribes. The message uses
host receive time, preserves the controller's degree-valued Euler fields and
raw temperature byte, and deliberately retains identical consecutive replies
so cache cadence remains measurable. Its frame is intentionally unset and it
is never fused directly. The stop-only adapter consumes it and publishes the
sparse yaw-only EKF input.
The same driver publishes `/muto/imu_telemetry_status`; the recorder stores it
without initiating any serial transaction. Its counters distinguish controller
read failures from gait-deadline skips and valid raw snapshots suppressed as
duplicates.

The source bag intentionally excludes `scan_odom_raw`, `scan_odom`,
`foot_odom`, `odometry/filtered`, and dynamic `/tf`. Those are results under
test and must be recomputed. Static sensor transforms are captured for
provenance. Replay uses the current `tf2_publisher` geometry by default so TF
fixes can be evaluated against the same recorded sensor data; an explicit
launch switch can instead replay the recorded `/tf_static` messages.

The metadata message is written automatically when the recorder starts. Its
git revision and dirty flag describe the source tree from which the recorder
binary was most recently built. Rebuild the package before a field session so
this identifies the code actually under test.

The default foot estimator derives motion from measured motor-angle FK, not
from commanded foot displacement. The commanded gait state supplies the full
50 Hz stance history needed to reject a motor-sample interval if any selected
support foot swung between its endpoints. Commanded stance is still not
measured contact, so the estimator cannot detect foot slip or a foot that has
lost contact.

When motor recording is explicitly enabled, the conservative 2 Hz rate is
suitable for residual and timing diagnostics, but it is generally too sparse
for moving foot odometry. The
estimator suppresses such intervals rather than treating a complete gait cycle
as one planted-foot transform. The 2026-08-05 10 Hz hardware test delayed gait
and IMU processing and still produced severely under-scaled measured foot
motion. Rates above 2 Hz now require an explicit experimental opt-in, and 20 Hz
must not be attempted with the current blocking serial service. See
[`../../docs/odometry_10hz_mini_test_2026-08-05.md`](../../docs/odometry_10hz_mini_test_2026-08-05.md).

## Record

Build and source the workspace, start the normal hardware/localization
pipeline, then attach the recorder:

```bash
ros2 launch muto_odometry_bag record_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001
```

The default recording above contains LiDAR, raw/processed IMU, controller
attitude, gait, commands, events, and static TF without adding serial requests
from the recorder. A controlled motor experiment must first opt in. A
high-rate experiment must state the opt-in, rate, and safety override:

```bash
ros2 launch muto_odometry_bag record_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_experimental \
  record_motor_angles:=true \
  motor_poll_rate:=10.0 \
  allow_experimental_high_rate_motor_polling:=true
```

This override is not approval for normal deployment; it only prevents an
experimental rate from being selected accidentally.

Stop the recorder with `Ctrl-C` so rosbag2 writes its final metadata. If
`bag_path` is omitted, the node creates a timestamped directory in its current
working directory. An existing bag directory is never overwritten.

Inspect the source bag:

```bash
ros2 bag info /data/bags/muto_odom_001
```

For the controller-attitude rerun on the deployed workspace, use:

### Terminal 1

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

### Terminal 2

```bash
ros2 launch muto_odometry_bag record_odometry_bag_launch.py \
  bag_path:=/opt/muto_rs_ws/bags/muto_odometry_attitude_002 \
  record_motor_angles:=false
```

### Terminal 3

```bash
ros2 topic hz /imu/controller_attitude
ros2 topic echo /muto/imu_telemetry_status --once \
  --qos-durability transient_local
```

The hardware launch requests `0x60` at 10 Hz to avoid aliasing the measured
roughly 5 Hz controller producer and retains raw `0x61` polling at 10 Hz so
the comparison does not worsen its host-observed transition timestamps. A
single gait-slotted scheduler sends the gait first and then
services at most one endpoint; requests that cannot fit before the next gait
deadline remain due for the next slot. Set
`imu_attitude_publish_rate_hz:=0.0` on the pipeline
launch for rollback. After stopping the recorder with `Ctrl-C`, require
`ros2 bag info` to show nonzero `/imu/controller_attitude` and
`/muto/imu_telemetry_status` counts. With localization running, also require a
nonzero `/muto/controller_attitude_yaw_status` count.

This rerun leaves motor polling off so the additional `0x60` traffic can be
evaluated without the known blocking motor-read confounder. Replay it with
`launch_foot_odometry:=false`. If measured-foot comparison is also required,
record a separate controlled bag with `record_motor_angles:=true` at 2 Hz and
treat its gait timing as a different experimental condition.

## Mark measured field endpoints

Publish an event after the robot is settled at each measured start or end
pose. The bag receive timestamp is the event time, so the JSON does not need a
ROS header. Keep angles in radians and identify the physical reference used to
measure position:

```bash
ros2 topic pub --once /muto/odometry_test_event std_msgs/msg/String \
  '{data: "{\"trial\":\"straight_forward_1m\",\"event\":\"end\",\"x_m\":0.987,\"y_m\":-0.014,\"yaw_rad\":0.021,\"accumulated_yaw_rad\":0.021,\"reference\":\"base_frame_floor_projection\"}"}'
```

Recommended keys are `trial`, `event`, `x_m`, `y_m`, `yaw_rad`,
`accumulated_yaw_rad`, `reference`, `position_uncertainty_m`,
`yaw_uncertainty_rad`, and an optional `note`. For a full turn, use a final
`yaw_rad` near the starting heading but set `accumulated_yaw_rad` to
`+6.283185` or `-6.283185`.

## Replay through the original stack

Do not run the hardware pipeline in the same ROS domain during replay. The
replay launch owns the original source topic names and the
`get_motor_angles` service.

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  playback_rate:=1.0 \
  launch_foot_odometry:=false
```

Controller-yaw fusion is enabled by default, so this replay requires nonempty
`/imu/controller_attitude` and `/muto/motion_command_state` topics. For an
older bag, explicitly use `fuse_controller_attitude_yaw:=false`.

The replayed test-event and recording-metadata topics are available to
analysis tools. Raw IMU, controller attitude, and IMU telemetry status are also
republished when present. Controller attitude and telemetry status remain
optional in the file format, so schema-1/2 bags remain replayable with the
explicit raw-gyro rollback above. To
test a revised IMU processor, launch that processor separately and suppress the
previously processed samples so it is the only publisher of
`/imu/data_processed`:

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  replay_processed_imu:=false
```

Start the revised processor before this replay command. In this mode the
replayer requires both a non-empty `/imu/data_raw` recording and a live raw-IMU
subscriber before playback begins.

The default replay replaces the sparse raw-gyro EKF input with stable,
stop-only relative controller yaw:

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odometry_attitude_002 \
  playback_rate:=2.0 \
  launch_foot_odometry:=false
```

This mode requires nonempty `/imu/controller_attitude` and
`/muto/motion_command_state` recordings and waits for the motion-state gate,
yaw adapter, and downstream yaw-EKF subscriptions before playback. It
deliberately does not wait for the replaced processed-IMU input.
Comparison mode waits for both the baseline IMU path and this attitude path.
The adapter publishes
`/imu/controller_attitude_imu`, suppresses exact cached packets, preserves the
recorded host-receive stamp, and supplies yaw only. It publishes one startup
anchor, observes every changed 5 Hz snapshot, waits for two seconds of strict
selected-and-active standby, and publishes one circular-mean correction per
stop. It publishes nothing during motion. The EKF zero-aligns the first
observation with `imu0_relative: true`; it does not differentiate Euler angles
into angular velocity. Accepted corrections use `(4 deg)^2` variance and an
independent EKF innovation threshold of `1.0`.

To reproduce the exact recorded static transforms instead of testing with the
current transform publishers:

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  replay_recorded_tf_static:=true
```

This disables the current static TF launch for that replay, preventing two
publishers from claiming the same sensor transforms. Older source bags remain
replayable with the default current-TF mode; recorded-TF mode requires a
non-empty `/tf_static` topic. The replayer caches and publishes all recorded
static transforms before releasing the first sensor message. This is required
because a recorder can discover transient `/tf_static` only after scans have
already entered the bag; replaying that discovery order made RF2O initialize
without `base_frame` in `muto_odometry_attitude_002`.

The replay node waits for the original LiDAR, IMU, command, and optional foot
consumers before releasing the first message. It publishes `/clock`, and all
original odometry nodes run with `use_sim_time:=true`. Message headers and ROS
time therefore retain the recorded timing even when `playback_rate` changes
the wall-clock duration. The RF2O submodule stays unmodified. Replay runs its
stock polling loop at `64 * playback_rate` Hz, four times the expected wall-clock
scan arrival rate. CPU scheduling can still drop work at high acceleration:
the `_002` comparison retained 99.73% of scans at 2x but only about 76% in one
8x run. Use 2x or slower for quantitative comparisons and verify output counts.
Normal robot startup remains at 16 Hz.

Set `launch_foot_odometry:=false` to test only RF2O plus IMU EKF:

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  launch_foot_odometry:=false
```

## Compare RF2O covariance profiles

Replay the same source bag with foot odometry disabled to isolate covariance
changes in the LiDAR plus IMU path:

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  playback_rate:=2.0 \
  launch_foot_odometry:=false \
  rf2o_covariance_profile:=measured
```

Repeat with `relaxed`, `conservative`, and, for regression only,
`legacy_zero`. The parent-owned output wrapper applies these profiles; they
change the uncertainty supplied to the EKF, not RF2O's scan-matching pose.
Record `/scan_odom_raw`, `/scan_odom`, and
`/odometry/filtered` when comparing EKF covariance, rejection, and prediction
behavior.

## Replay all odometry variants together

The comparison launch runs the LiDAR estimator once and the baseline EKF
variants over the same replay clock. Foot odometry and the controller-attitude
candidate are independent, default-off options. The parent wrapper uses the
selected covariance profile for every variant, so the default `measured` run
changes only the fused sensor inputs:

```bash
ros2 launch muto_odometry_bag replay_odometry_comparison_launch.py \
  bag_path:=/data/bags/muto_odometry_attitude_002 \
  playback_rate:=2.0 \
  launch_foot_odometry:=false \
  compare_controller_attitude:=true
```

It publishes these independently inspectable topics, subject to the enabled
optional branches:

| Topic | Inputs |
| --- | --- |
| `/scan_odom_raw` | Raw LiDAR RF2O pose before deadband/jump filtering |
| `/scan_odom` | LiDAR only, after the existing RF2O deadband filter |
| `/foot_odom` | Measured motor-angle FK with commanded-stance continuity gating |
| `/odometry/lidar_only` | Filtered LiDAR pose through the EKF, without IMU or feet |
| `/odometry/lidar_imu` | LiDAR plus IMU EKF |
| `/odometry/lidar_controller_attitude` | LiDAR plus stable, stop-only relative `0x60` yaw; sparse raw gyro replaced |
| `/odometry/raw_lidar_imu` | Raw RF2O plus IMU EKF through a covariance-only wrapper branch, bypassing deadband and jump rejection |
| `/odometry/filtered` | Normal launch result; LiDAR plus raw gyro, and optional foot input when explicitly enabled |

Only the fused `/odometry/filtered` EKF publishes `odom -> base_frame`. The
comparison EKFs have TF publication disabled, preventing multiple estimators
from claiming the same transform.

To save a particular replay's derived results for comparison, run a normal
output-only recorder in another terminal:

```bash
ros2 bag record -o /data/bags/muto_odom_001_results \
  /scan_odom_raw /scan_odom /foot_odom /odometry/lidar_only \
  /odometry/lidar_imu /odometry/lidar_controller_attitude \
  /odometry/raw_lidar_imu /odometry/filtered \
  /imu/controller_attitude_imu \
  /muto/controller_attitude_yaw_status \
  /tf /muto/odometry_test_event \
  /muto/odometry_recording_metadata
```

This package follows the ROS 2 Humble
[C++ writer](https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html)
and
[C++ reader](https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html)
APIs rather than shelling out to `ros2 bag play`.
