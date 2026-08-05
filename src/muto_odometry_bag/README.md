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
5. `robot_localization/ekf_node` produces `odometry/filtered` and
   `odom -> base_frame`.

## Source bag contract

| Bag topic | Type | Reason |
| --- | --- | --- |
| `/lidar/raw_laserscan` | `sensor_msgs/msg/LaserScan` | Input to the existing LiDAR filter and RF2O |
| `/imu/data_processed` | `sensor_msgs/msg/Imu` | Existing EKF IMU input |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Original IMU sample for later calibration and processing changes |
| `/muto/commanded_gait_state` | `muto_hexapod_interfaces_custom/msg/CommandedGaitState` | Commanded stance/swing and foot targets |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Existing RF2O deadband gate input |
| `/muto/measured_motor_state` | `std_msgs/msg/String` | Baggable representation of each successful `get_motor_angles` response |
| `/muto/odometry_test_event` | `std_msgs/msg/String` | Timestamped JSON start/end and measured field-pose markers |
| `/muto/odometry_recording_metadata` | `std_msgs/msg/String` | Recorder build git revision, dirty state, and bag schema |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Exact static sensor geometry offered during recording |

The recorder polls `get_motor_angles` at the same default 2 Hz rate used by
foot odometry. This is an additional service read while recording.

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

The conservative 2 Hz recorder rate is suitable for residual and timing
diagnostics, but it is generally too sparse for moving foot odometry. The
estimator suppresses such intervals rather than treating a complete gait cycle
as one planted-foot transform. Record at 10 Hz or 20 Hz only after that rate is
validated against the 50 Hz gait and IMU loops on hardware.

## Record

Build and source the workspace, start the normal hardware/localization
pipeline, then attach the recorder:

```bash
ros2 launch muto_odometry_bag record_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001
```

Stop the recorder with `Ctrl-C` so rosbag2 writes its final metadata. If
`bag_path` is omitted, the node creates a timestamped directory in its current
working directory. An existing bag directory is never overwritten.

Inspect the source bag:

```bash
ros2 bag info /data/bags/muto_odom_001
```

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
  launch_foot_odometry:=true
```

The replayed test-event and recording-metadata topics are available to
analysis tools. Raw IMU is also republished when present. To test a revised IMU
processor, launch that processor separately and suppress the previously
processed samples so it is the only publisher of `/imu/data_processed`:

```bash
ros2 launch muto_odometry_bag replay_odometry_bag_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  replay_processed_imu:=false
```

Start the revised processor before this replay command. In this mode the
replayer requires both a non-empty `/imu/data_raw` recording and a live raw-IMU
subscriber before playback begins.

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
non-empty `/tf_static` topic.

The replay node waits for the original LiDAR, IMU, command, and optional foot
consumers before releasing the first message. It publishes `/clock`, and all
original odometry nodes run with `use_sim_time:=true`. Message headers and ROS
time therefore retain the recorded timing even when `playback_rate` changes
the wall-clock duration. The RF2O submodule stays unmodified. Replay runs its
stock polling loop at `64 * playback_rate` Hz, four times the expected wall-clock
scan arrival rate, to prevent accelerated playback from replacing an
unprocessed scan. Normal robot startup remains at 16 Hz.

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
  playback_rate:=4.0 \
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

The comparison launch runs the LiDAR and foot estimators once and four EKF
instances over the same replay clock. The parent wrapper uses the selected
covariance profile for every variant, so the default `measured` run changes
only the fused sensor inputs:

```bash
ros2 launch muto_odometry_bag replay_odometry_comparison_launch.py \
  bag_path:=/data/bags/muto_odom_001 \
  playback_rate:=1.0
```

It publishes six independently inspectable results:

| Topic | Inputs |
| --- | --- |
| `/scan_odom_raw` | Raw LiDAR RF2O pose before deadband/jump filtering |
| `/scan_odom` | LiDAR only, after the existing RF2O deadband filter |
| `/foot_odom` | Measured motor-angle FK with commanded-stance continuity gating |
| `/odometry/lidar_only` | Filtered LiDAR pose through the EKF, without IMU or feet |
| `/odometry/lidar_imu` | LiDAR plus IMU EKF |
| `/odometry/raw_lidar_imu` | Raw RF2O plus IMU EKF through a covariance-only wrapper branch, bypassing deadband and jump rejection |
| `/odometry/filtered` | LiDAR plus leg plus IMU EKF |

Only the fused `/odometry/filtered` EKF publishes `odom -> base_frame`. The
comparison EKFs have TF publication disabled, preventing multiple estimators
from claiming the same transform.

To save a particular replay's derived results for comparison, run a normal
output-only recorder in another terminal:

```bash
ros2 bag record -o /data/bags/muto_odom_001_results \
  /scan_odom_raw /scan_odom /foot_odom /odometry/lidar_only \
  /odometry/lidar_imu /odometry/raw_lidar_imu /odometry/filtered \
  /tf /muto/odometry_test_event \
  /muto/odometry_recording_metadata
```

This package follows the ROS 2 Humble
[C++ writer](https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html)
and
[C++ reader](https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html)
APIs rather than shelling out to `ros2 bag play`.
