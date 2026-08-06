# Odometry And Localization Notes

Date: 2026-07-28

This document describes the odometry-related behavior in this workspace. It is
intended to be a working engineering note for the Muto RS deployment, not a
general ROS localization tutorial.

The robot deployment target for this stack is ROS 2 Humble.

## Why Odometry Matters

After the robot's physical frames and sensor mounting geometry are known, the
robot still needs to estimate how `base_frame` moves through the world. In this
workspace that local motion estimate lives in the `odom` frame. The normal TF
chain for mapping and navigation is:

```text
map -> odom -> base_frame -> sensor frames
```

`odom -> base_frame` is the locally continuous robot pose estimate. It drifts
over time, but should not jump. `map -> odom` is produced by localization or
SLAM to relate that drifting local odometry frame to the map frame. Fixed sensor
mounts such as `base_frame -> lidar_frame` are static TFs.

## Normal Startup Path

The normal robot bringup is split across three launches:

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
ros2 launch tf2_publisher all_tf2_publishers_launch.py
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

The one-shot `muto_nav2_pipeline_launch.py` starts these same layers with
readiness gates before mapping and Nav2.

The normal odometry flow is:

```text
TG30 hardware
  -> /lidar/raw_laserscan                 (LaserScan, frame: lidar_frame)
  -> lidar_laserscan_filter_node
  -> /lidar/filtered_laserscan            (downsampled LaserScan for RF2O)
  -> rf2o_laser_odometry_node
  -> scan_odom_raw                        (raw RF2O Odometry)
  -> odometry_translation_deadband_node
  -> scan_odom                            (filtered LiDAR odometry)
  -> robot_localization ekf_node
  -> /odometry/filtered and odom -> base_frame TF
```

The EKF is the authoritative publisher of `odom -> base_frame` in this normal
pipeline.

## Key Files

| File | Role |
| --- | --- |
| [`src/yahboomcar_bringup/launch/muto_hardware_launch.py`](../src/yahboomcar_bringup/launch/muto_hardware_launch.py) | Starts the TG30 LiDAR, Orbbec camera launch, and Muto base driver/IMU publisher. |
| [`src/tf2_publisher/launch/all_tf2_publishers_launch.py`](../src/tf2_publisher/launch/all_tf2_publishers_launch.py) | Starts static sensor TF publishers; optional odom TF publisher is disabled by default. |
| [`src/lidar_tg30/src/lidar_node.cpp`](../src/lidar_tg30/src/lidar_node.cpp) | Publishes the raw TG30 LaserScan. |
| [`src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py`](../src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py) | Starts LiDAR scan filtering, RF2O, and the odometry deadband wrapper. |
| [`src/lidar_pointcloud_filter/src/lidar_laserscan_filter_node.cpp`](../src/lidar_pointcloud_filter/src/lidar_laserscan_filter_node.cpp) | Filters raw LiDAR LaserScan into RF2O/Nav2 and SLAM scan topics. |
| [`src/lidar_pointcloud_filter/src/odometry_translation_deadband_node.cpp`](../src/lidar_pointcloud_filter/src/odometry_translation_deadband_node.cpp) | Applies RF2O deadbands and jump rejection before publishing `scan_odom`. |
| [`src/yahboomcar_bringup/launch/ekf_imu_lidar_launch.py`](../src/yahboomcar_bringup/launch/ekf_imu_lidar_launch.py) | Main EKF launch for LiDAR plus IMU odometry. |
| [`src/yahboomcar_bringup/config/ekf_lidar_imu.yaml`](../src/yahboomcar_bringup/config/ekf_lidar_imu.yaml) | Default EKF fusion configuration. |
| [`src/yahboomcar_imu/yahboomcar_imu/imu_node.py`](../src/yahboomcar_imu/yahboomcar_imu/imu_node.py) | Publishes raw and processed IMU messages. |
| [`src/yahboomcar_bringup/yahboomcar_bringup/foot_odometry_node.py`](../src/yahboomcar_bringup/yahboomcar_bringup/foot_odometry_node.py) | Continuity-gated measured-joint odometry, enabled as a low-trust EKF velocity input by default. |
| [`src/muto_slam_mapping/config/mapper_params_online_async.yaml`](../src/muto_slam_mapping/config/mapper_params_online_async.yaml) | SLAM Toolbox frame and scan-topic settings. |

## Frames And TF Ownership

Sensor mount transforms are published by `tf2_publisher`:

| Transform | Publisher | Notes |
| --- | --- | --- |
| `base_frame -> lidar_frame` | `base_to_lidar_publisher` | Static TF. Translation `x=-0.02`, `y=0.0`, `z=0.0`; RPY roughly `(0, -pi, 0.20)`. |
| `base_frame -> imu_link` | `base_to_imu_publisher` | Static TF. Translation `x=0.07`, `y=0.0`, `z=0.0`. |
| `base_frame -> camera_link` | `base_to_camera_publisher` | Static TF. Translation `x=0.13`, `y=0.0`, `z=0.115`; RPY `(0, 0.18325, 0)`. |

The Orbbec SDK on the real robot is expected to publish the internal camera
frame tree for camera optical/depth frames. The local camera publisher only owns
`base_frame -> camera_link`.

`tf2_publisher/odom_publisher` can republish an odometry topic as TF, but
`all_tf2_publishers_launch.py` keeps it disabled by default with
`publish_odom_tf:=false`. Do not enable it while the EKF is publishing
`odom -> base_frame`, or the tree will have duplicate dynamic TF publishers.

## LiDAR Input

`muto_hardware_launch.py` starts `lidar_tg30/lidar_node`. The driver publishes
`/lidar/raw_laserscan` as `sensor_msgs/msg/LaserScan` in `lidar_frame`. This is
the only TG30 data path; downstream filtering produces the two scans needed by
RF2O/Nav2 and SLAM.

## LiDAR Scan Filtering

`filter_lidar_odometry_launch.py` starts
`lidar_pointcloud_filter/lidar_laserscan_filter_node` unconditionally.

It consumes `/lidar/raw_laserscan` and publishes two scans:

| Topic | Purpose | Default filtering |
| --- | --- | --- |
| `/lidar/filtered_laserscan` | RF2O odometry input | `range_min=0.05`, `range_max=10.0`, full circle, downsample factor `2`. |
| `/lidar/filtered_laserscan_no_downsample` | SLAM Toolbox input | Full resolution, `range_min=0.05`, `range_max=15.0`. |

The node preserves input timestamps by default. `scan_restamp_output:=false` is
intentional; restamping should only be used when a driver is known to publish bad
timestamps while the data itself is fresh.

## RF2O LiDAR Odometry

The active LiDAR odometry node is `rf2o_laser_odometry_node`, launched from
`filter_lidar_odometry_launch.py`.

Default launch parameters:

| Parameter | Value |
| --- | --- |
| `laser_scan_topic` | `/lidar/filtered_laserscan` |
| `odom_topic` | `scan_odom_raw` |
| `odom_frame_id` | `odom` |
| `base_frame_id` | `base_frame` |
| `freq` | `16.0` Hz |
| ROS log level | `error` |
| `publish_tf` | `false` inside this launch |

RF2O uses TF2 to look up the transform from the scan frame to `base_frame` on
the first scan. This lets the raw scan remain in `lidar_frame`; the odometry
result is still expressed for `base_frame`.

RF2O publishes its original `scan_odom_raw` message and does not own the final
`odom -> base_frame` TF in the normal EKF pipeline. The RF2O submodule remains
unmodified. Covariance, deadbanding, and jump rejection are parent-owned
post-processing in `odometry_translation_deadband_node`.

The covariance profiles are provisional starting points derived from the first
recorded field bag:

| Profile | X/Y pose variance | Yaw pose variance | Intended use |
| --- | --- | --- | --- |
| `measured` | `2.5e-4 m^2` | `1.0e-4 rad^2` | Default bag-derived estimate; about 1.6 cm and 0.57 deg standard deviation. |
| `relaxed` | `1.0e-3 m^2` | `4.0e-4 rad^2` | Twice the default standard deviation. |
| `conservative` | `2.5e-3 m^2` | `2.7416e-3 rad^2` | About 5 cm and 3 deg standard deviation. |
| `legacy_zero` | `0` | `0` | Regression comparison only; not a valid uncertainty model. |

The profile is forwarded through the top-level pipeline as
`rf2o_covariance_profile`. The parent wrapper writes that covariance into
`scan_odom`; RF2O itself is unaware of the profile. A `custom` profile is also
available through the wrapper's `custom_*_variance` parameters for later
calibration work.

### First bag comparison

`odom_test_001` contains 4,189 scans. RF2O uses the first scan for
initialization. Accelerated replay raises the stock RF2O polling frequency to
four times the wall-clock scan arrival rate instead of patching RF2O's callback. The
four event labels are approximate measured field checkpoints, not motion-capture
ground truth, so these results are suitable for initial tuning but not final
covariance calibration.

With foot odometry disabled, the current LiDAR-plus-IMU pipeline produced:

| Input path | Checkpoint position RMSE | Checkpoint yaw RMSE | Final position error | Final yaw error |
| --- | --- | --- | --- | --- |
| Raw RF2O, before deadband | `0.0892 m` | `6.226 deg` | `0.0394 m` | `0.405 deg` |
| Current filtered RF2O and EKF | `0.1223 m` | `6.490 deg` | `0.0384 m` | `2.383 deg` |

Feeding the same filtered trajectory and IMU messages to all covariance
profiles isolates the confidence change from scan-matching timing:

| Profile | Position RMSE | Yaw RMSE | Position sigma at final marker | Yaw sigma at final marker |
| --- | --- | --- | --- | --- |
| `measured` | `0.122337 m` | `6.490 deg` | `0.0437 m` | `2.569 deg` |
| `relaxed` | `0.122337 m` | `6.490 deg` | `0.0503 m` | `2.734 deg` |
| `conservative` | `0.122338 m` | `6.490 deg` | `0.0590 m` | `3.510 deg` |
| `legacy_zero` | `0.122336 m` | `6.490 deg` | `0.0407 m` | `2.506 deg` |

As expected with RF2O as the only absolute pose source, covariance selection
hardly changes the checkpoint trajectory. It changes the EKF's reported
confidence. `measured` is therefore the default. The raw-versus-filtered result
motivated the production deadband change below.

### Measured-profile fusion comparison

The original full comparison launch was run with `rf2o_covariance_profile`
fixed to `measured`. Each EKF saw the same replay clock and RF2O trajectory;
only its sensor inputs changed. At that time, the legacy foot implementation
derived displacement from commanded gait targets and used the 2 Hz motor
samples only as a tracking check.

| Variant | Checkpoint position RMSE | Checkpoint yaw RMSE | Final-marker position error | Final-marker yaw error |
| --- | --- | --- | --- | --- |
| Raw RF2O reference | `0.08900 m` | `6.2256 deg` | `0.03939 m` | `0.4045 deg` |
| Filtered RF2O reference | `0.12234 m` | `6.4902 deg` | `0.03837 m` | `2.3827 deg` |
| Filtered LiDAR-only EKF | `0.12234 m` | `6.4887 deg` | `0.03837 m` | `2.3834 deg` |
| Filtered LiDAR plus IMU EKF | `0.12234 m` | `6.4902 deg` | `0.03837 m` | `2.3827 deg` |
| Raw RF2O plus IMU EKF | `0.08990 m` | `6.2346 deg` | `0.03907 m` | `0.3711 deg` |
| Filtered LiDAR plus IMU plus foot EKF | `0.12234 m` | `6.4902 deg` | `0.03837 m` | `2.3827 deg` |

The raw-RF2O-plus-IMU variant was the strongest tested fusion on these sparse
field markers. Adding IMU yaw rate to filtered LiDAR did not change checkpoint
accuracy, but it changed inter-scan yaw prediction by `0.225 deg` RMS and
slightly reduced the EKF yaw uncertainty, so it remains useful as the angular
velocity source.

Adding the legacy command-derived foot velocity changed the LiDAR-plus-IMU
trajectory by only `0.0025 m` RMS and did not improve a checkpoint. Its
standalone estimate reached about `5.95 m` when the robot had returned close to
its start. This was not a covariance problem: commanded gait displacement was
being mistaken for measured body displacement. The legacy mode is now retained
only as `foot_odometry_source:=commanded_targets` for regression testing.

The comparison was rerun after making `measured_joints` the default. The bag has
525 joint snapshots over 262.6 seconds, approximately 2 Hz. Consecutive moving
snapshots span 22 to 24 of the 50 Hz gait phases, crossing tripod changes. The
continuity gate therefore accepted zero moving increments. During replay,
`/foot_odom` produced 403 valid standby zero-velocity messages, zero nonzero
twists, and an unchanged pose. The fused trajectory and LiDAR-plus-IMU
trajectory differed by `0.000033 m` on average; the largest brief position
difference was `0.0092 m`. This is the intended safe result for an undersampled
bag. Measured foot displacement requires non-blocking, verified physical-joint
feedback; simply increasing the blocking service rate has now been tested and
rejected.

No RF2O increment in this bag exceeded the existing `0.03 m` translation or
`5 deg` yaw jump limits. Consequently, the production defaults now set only the
small RF2O deadbands to zero while retaining jump rejection. This preserves the
better raw trajectory observed in the trial without removing the safety guards.

## RF2O Deadband And Jump Rejection

`odometry_translation_deadband_node` wraps RF2O output:

```text
scan_odom_raw -> scan_odom
```

Current default filters:

| Setting | Default | Meaning |
| --- | --- | --- |
| `translation_deadband` | `0.0` m | Production default preserves all RF2O translation increments. Set explicitly only for a stationary-drift experiment. |
| `yaw_deadband` | `0.0` rad | Production default preserves all RF2O yaw increments. Set explicitly only for a stationary-drift experiment. |
| `translation_jump_rejection_threshold` | `0.03` m | Reject RF2O XY updates above 3 cm per update in every motion state; retained with deadbands disabled. |
| `max_translation_rate` | `0.0` m/s | Disabled so translation jump rejection uses only the per-update 3 cm cap. |
| `yaw_jump_rejection_threshold` | `0.087266` rad | Reject RF2O yaw updates above 5 deg per update in every motion state; retained with deadbands disabled. |
| `max_yaw_rate` | `0.0` rad/s | Disabled so yaw jump rejection uses only the per-update 5 deg cap. |
| `use_cmd_vel_gate` | `true` | Apply RF2O deadbands per axis only when recent `cmd_vel` for that axis is near zero; jump rejection is always active. |
| `cmd_vel_timeout` | `0.5` s | If no fresh `cmd_vel` is seen, assume stationary and apply the filters. |
| `cmd_vel_stationary_linear_threshold` | `0.03` m/s | Translation deadbanding applies at or below this commanded planar speed. |
| `cmd_vel_stationary_angular_threshold` | `0.03` rad/s | Yaw deadbanding applies at or below this commanded yaw rate. |

In normal Nav2 operation, the controller publishes `/cmd_vel_nav` and the
lifecycle-managed velocity smoother publishes the follow-path `/cmd_vel`
consumed by both the Muto driver and this guard. The guard therefore sees the
bounded command presented to the hardware, not the controller's unsmoothed
request.
Recovery behaviors are the exception: they currently publish directly to
`/cmd_vel`, so the guard still sees their actual command but no velocity-smoother
stage precedes it.

In standalone mode, `filter_lidar_odometry_launch.py` defaults
`rf2o_publish_tf:=true`, so the deadband wrapper can publish `odom -> base_frame`
for testing without an EKF.

When launched through `ekf_imu_lidar_launch.py`, that argument is forced to
`false`, so the wrapper publishes only `scan_odom` and the EKF owns TF.

## IMU Processing

`muto_hardware_launch.py` starts `yahboomcar_bringup/muto_driver`, and the driver
instantiates `yahboomcar_imu.imu_node.ImuPublisher`.

IMU topics:

| Topic | Frame | Meaning |
| --- | --- | --- |
| `/imu/data_raw` | `raw_imu_link` | Raw accelerometer and gyro counts published as an IMU message for inspection. |
| `/imu/mag_raw` | `raw_imu_link` | Raw magnetometer values. |
| `/imu/data_processed` | `imu_link` | Scaled accelerometer and gyro data used by localization experiments. |

`/imu/data_processed` does not provide orientation. Its orientation covariance is
set to `-1`, which tells consumers that orientation is unavailable.
The current angular-velocity variance is `8.5e-6 (rad/s)^2`, estimated from
stationary raw-gyro samples in `odom_test_001`.

Startup calibration is enabled by default. While the robot is still, the node
collects raw IMU samples to estimate:

- accelerometer counts per 1 g from the norm of the accelerometer vector;
- gyro biases for x, y, and z;
- a yaw-rate deadband before publishing `angular_velocity.z`.

The EKF currently consumes only IMU yaw rate:

```text
/imu/data_processed angular_velocity.z
```

It does not fuse IMU linear acceleration, roll/pitch, or absolute orientation in
the normal configuration.

## EKF Fusion

The default EKF launch is:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

It starts the LiDAR odometry path unless `launch_lidar_odometry:=false`, then
starts the installed ROS Humble `robot_localization/ekf_node` with
`ekf_lidar_imu.yaml`.

Important EKF frame settings:

| Setting | Value |
| --- | --- |
| `map_frame` | `map` |
| `odom_frame` | `odom` |
| `base_link_frame` | `base_frame` |
| `world_frame` | `odom` |
| `two_d_mode` | `true` |
| `frequency` | `30.0` Hz |
| `publish_tf` | `true` |

The EKF fuses:

| Source | Topic | Fused fields |
| --- | --- | --- |
| RF2O filtered odometry | `/scan_odom` | `x`, `y`, and yaw pose. |
| Processed IMU | `/imu/data_processed` | yaw rate only. |

This means LiDAR odometry dominates translation and absolute yaw. The IMU is a
secondary yaw-rate source, not the source of absolute orientation.

The EKF publishes the filtered odometry topic and the authoritative
`odom -> base_frame` TF.

## Measured-Joint Foot Odometry

Foot/gait odometry is enabled by default. It can be disabled with:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py launch_foot_odometry:=false
```

`foot_odometry_node` is measured-joint kinematic odometry, but it is not
contact-sensed odometry. The custom driver latches `/cmd_vel` and advances
exactly one trajectory phase per 50 Hz locomotion tick. It publishes gait state
only after sending that phase's motor targets; changing a nonzero command
preserves phase instead of restarting the 20-step cycle. A command older than
0.5 seconds actively returns the gait to standby. The node:

- retains `/muto/commanded_gait_state` as a 50 Hz history of which feet were
  commanded in stance or swing;
- polls `get_motor_angles`, whose response pairs all 18 motor values with the
  gait target and stance mask that were current during that serial read;
- samples without an artificial settle sleep; the serial read itself still
  holds the shared bus and can delay the 50 Hz timers if its response is slow;
- converts calibrated logical motor angles through the custom library's forward
  kinematics to obtain six measured foot positions in `base_frame`;
- requires a complete gait history, one unchanged gait mode, no more than ten
  skipped gait phases, and at least three feet that remained in commanded
  stance throughout the interval between motor snapshots;
- fits the planar body transform from the current measured support-foot
  positions to the previous measured support-foot positions;
- rejects stale or malformed input, incomplete support history, FK residuals
  above `0.01 m`, steps above `0.05 m` or `0.2 rad`, inferred rates above
  `1.0 m/s` or `2.0 rad/s`, and command-tracking residuals above `0.03 m`;
- publishes valid measured increments and weak standby zero-velocity updates,
  stamping each output with the motor sample's source timestamp.

`odometry_source:=commanded_targets` preserves the previous command-derived
estimator only for regression. It must not be used as measured odometry.

Motor values are already in firmware-calibrated logical degrees. The node does
not read or subtract raw servo calibration offsets a second time. The factory
standing command is `(0, -30, -15)` degrees per leg. That pose and generated
gait commands are regression-tested against the custom model's `27.5`, `50.59`,
`72.60`, and `134.5` mm leg dimensions.

The motor check uses the worst sampled stance-foot FK error. It does not supply
the motion increment; it rejects samples whose stance classification may no
longer match the commanded geometry. Errors up to 5 mm
retain full motor confidence; confidence decreases from 5 mm to 30 mm; an error
above 30 mm suppresses output. Missing, stale, malformed, wrong-frame, or
wrong-angle-space samples also suppress output. A future-sequence motor sample
cannot validate replayed older gait phases.

### RViz Covariance Display

`/foot_odom` is a planar estimate. Its operational EKF configuration consumes
only `vx` and `vy`; it does not consume the foot pose, Z, roll, pitch, or yaw
rate. The message nevertheless requires complete 6x6 pose and twist covariance
arrays.

The node uses bounded variance `1.0` for unsupported Z, roll, and pitch entries.
An earlier value of `999` produced a standard deviation of about `31.6` metres
or radians. RViz's Odometry display retains one covariance visual per accepted
pose according to its `Keep` setting, so those entries appeared as multiple
world-scale disks even when the underlying planar pose was well behaved.

For trajectory inspection in RViz, set the Odometry display to `Keep: 1` or
turn its Covariance property off. With covariance enabled, repeated bounded
ellipses are expected and do not by themselves indicate duplicate TF or an
exploding pose. Check the numeric message separately:

```bash
ros2 topic echo /foot_odom --once
ros2 topic hz /foot_odom
ros2 run tf2_ros tf2_echo odom base_frame
```

The current node should report `header.frame_id: odom`,
`child_frame_id: base_frame`, finite planar pose/twist values, and no covariance
diagonal above the configured planar confidence-scaled values or bounded
unsupported-axis variances. The node itself does not publish TF in the normal
pipeline; the EKF remains the sole `odom -> base_frame` owner.

The ROS foot-odometry boundary converts vendor-model `x=right, y=forward` foot
coordinates to `base_frame` axes (`x=forward, y=left`). A stance label means
only that the target is in the gait's support portion. The estimator therefore
assumes those feet are static relative to the ground throughout the interval.
Servo tracking can expose gross command, calibration, or actuator errors, but
it still cannot measure ground contact, load, foot slip, or body motion caused
by external forces.

The first locomotion tick after driver startup commands the factory standby
pose instead of assuming that the physical servos are already there. Support
the robot and keep its legs clear when starting the hardware driver. Sensor
reads share the same serial bus as gait writes; the custom protocol now polls
until a complete response arrives and returns early rather than sleeping for a
fixed 50 or 100 ms on every read. Each moving phase uses six vendor `LEG`
packets instead of eighteen individual joint packets, reducing its serial
traffic from 216 to 84 bytes. The motor read rate remains at the conservative
2 Hz production default. The 2026-08-05 10 Hz hardware test measured a
`25.67 ms` median motor response, about `40 ms` p95 gait and IMU intervals, and
an IMU average of only `41.44 Hz`; 10 Hz is therefore rejected for production
with the current blocking serial service. The localization and top-level
pipeline launches expose this as `foot_motor_poll_rate`; each successful
`get_motor_angles` response includes `read_duration_sec` so serial response
latency can be measured directly.

Normal production startup keeps the 2 Hz limit:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py \
  launch_mapping:=false launch_nav2:=false foot_motor_poll_rate:=2.0

ros2 topic hz /muto/commanded_gait_state
ros2 topic hz /imu/data_processed
ros2 topic hz /foot_odom
ros2 service call /get_motor_angles std_srvs/srv/Trigger "{}"
```

Any rate above 2 Hz now requires the explicit
`allow_experimental_high_rate_motor_polling:=true` launch argument. Do not use
that override on the deployed robot until feedback is streamed or otherwise
decoupled from the single-threaded gait and IMU serial path. In particular, do
not proceed to 20 Hz with the current controller interface. The complete test
record is in
[`odometry_10hz_mini_test_2026-08-05.md`](odometry_10hz_mini_test_2026-08-05.md).

The offline byte budget at 115200 baud, including 8-N-1 framing, is:

| Joint poll rate | Gait + IMU + joint traffic | UART utilization |
| --- | ---: | ---: |
| 2 Hz | 6,020 bytes/s | 52.3% |
| 20 Hz | 6,650 bytes/s | 57.7% |
| 50 Hz | 7,700 bytes/s | 66.8% |

This shows that 20 Hz fits the average byte budget, but the 10 Hz hardware
result proves that average bandwidth is not the acceptance criterion and 20 Hz
is not safe with the current blocking service. A synthetic delayed-reply test
on the host measured a moving gait phase at about `6.7 ms`. With both sensor
reads due, the combined callback time was about `17.6 ms` for a `5 ms` reply
and `22.0 ms` for a `7 ms` reply. Therefore a slow or timed-out response can
still delay the single-threaded 50 Hz gait loop even though average UART
capacity is available. The real `read_duration_sec` distribution and topic
rates are the acceptance criteria. The driver also logs `Locomotion tick
delayed` or `Locomotion phase dispatch missed its budget` when the 20 ms target
is violated.

No source in the supplied vendor library defines a controller baud-change
command or a supported rate other than 115200. Changing only the Jetson serial
port to 1 Mbaud would therefore break communication. Keep 115200 unless
Yahboom supplies a controller-side procedure or the controller firmware is
available and explicitly confirms another rate.

The measured-joint estimator associates every snapshot pair with all
intervening gait phases and rejects the interval unless support continuity is
provable. At the 2 Hz production default, moving snapshots are normally
separated by more than one complete 20-phase gait cycle, so motion output is
deliberately suppressed. The 10 Hz test produced moving updates, but their
straight and yaw displacement were severely under-scaled and the blocking
reads delayed the gait and IMU loops. Moving foot velocity is therefore not
validated for production. Joint quantization, uncertain feedback semantics,
commanded rather than sensed contact, and unmeasured slip remain limitations.
Foot-derived yaw rate is not fused; RF2O supplies yaw and the IMU supplies
`wz`.

`ekf_lidar_imu_with_foot.yaml` is loaded as an overlay on the primary
`ekf_lidar_imu.yaml` configuration. `/foot_odom` contributes only planar body
velocity (`vx`, `vy`). Pose and yaw still come from RF2O, yaw rate still comes
from the IMU, and the foot node runs with `publish_tf:=false`.

## IMU-Only EKF Test

`ekf_imu_lidar_launch.py imu_only:=true` starts an EKF with
`ekf_imu_only.yaml`. That configuration only fuses IMU yaw rate. It is useful as
a wiring test for `/imu/data_processed`, but it is not a complete mobile-base
odometry source because it has no translational input and no absolute yaw input.

## Removed Legacy LiDAR Paths

`src/Simple-2D-LiDAR-Odometry` and the TG30/PCL PointCloud2 odometry branch
were removed from the active workspace. The current odometry pipeline is the
TG30 `LaserScan` path through `lidar_pointcloud_filter`,
`rf2o_laser_odometry`, the deadband wrapper, and the EKF.

## Mapping And Nav2 Relationship

SLAM and Nav2 rely on odometry but do not replace the EKF odom source.

`muto_slam_mapping/config/mapper_params_online_async.yaml` configures
SLAM Toolbox with:

```text
odom_frame: odom
map_frame: map
base_frame: base_frame
scan_topic: /lidar/filtered_laserscan_no_downsample
```

`online_async_mapping_launch.py` starts SLAM Toolbox on the full-resolution
filtered LiDAR topic. It also starts camera depth-to-scan projection by default,
but that camera topic is an independent Nav2 obstacle source and is not an input
to SLAM or the EKF.

Nav2 costmaps are configured around the same frame chain:

- local costmap: `global_frame=odom`, `robot_base_frame=base_frame`;
- global costmap: `global_frame=map`, `robot_base_frame=base_frame`;
- both consume `/lidar/filtered_laserscan` as the required LiDAR source and
  `/camera/filtered_laserscan` as an optional camera source.

The Nav2 controller and BT navigator read odometry from
`/odometry/filtered`. Command routing is:

```text
controller_server /cmd_vel_nav
  -> nav2_velocity_smoother
  -> /cmd_vel
  -> Muto driver + RF2O command-aware deadband/jump guard

behavior_server recovery command
  -> /cmd_vel
  -> Muto driver + RF2O command-aware deadband/jump guard
```

The smoother is configured open-loop at 20 Hz, so it shapes commands from its
last commanded velocity. `/odometry/filtered` remains its configured odometry
topic but is not used as closed-loop feedback unless `feedback` is changed.

## Duplicate TF Publisher Rules

Only one node should publish any dynamic `odom -> base_frame` transform at a
time.

Normal EKF pipeline:

| Node | Publishes `odom -> base_frame` TF? |
| --- | --- |
| RF2O node | No. Forced `publish_tf=false`. |
| Deadband wrapper | No. `ekf_imu_lidar_launch.py` passes `rf2o_publish_tf=false`. |
| Foot odometry | No. Launched with `publish_tf=false`. |
| `tf2_publisher/odom_publisher` | No. Disabled unless `publish_odom_tf:=true`. |
| EKF | Yes. This is the authoritative publisher. |

Standalone LiDAR odometry test:

| Node | Publishes `odom -> base_frame` TF? |
| --- | --- |
| RF2O node | No. |
| Deadband wrapper | Yes by default, unless `rf2o_publish_tf:=false`. |

If the EKF is running, keep every other odometry TF publisher disabled.

## Timing And Stamps

The odometry pipeline prefers real sensor timestamps over restamping:

- TG30 LaserScan messages are stamped with the driver node clock.
- LiDAR scan filtering preserves the input stamp by default.
- RF2O timestamps odometry using the scan time it processed.
- The deadband wrapper preserves the incoming odometry stamp.
- The EKF uses those stamps for fusion and TF publication.

The filtering nodes warn when input stamps are far from the node clock and can
drop data when the age exceeds `max_input_age`. Large timestamp gaps are clock or
driver-stamping problems, not map update-rate problems.

Occasional Nav2 message-filter drops during startup can be normal TF cache
behavior. Continuous drops after the system has been running indicate a real
time/TF problem.

## Current Known Risks

- LiDAR odometry can drift when the scan geometry is poor, when a person stands
  close to the LiDAR, or when the robot rotates in a feature-poor area.
- RF2O yaw can still jump if scan matching fails badly; the deadband wrapper now
  rejects sudden large translation and yaw updates using threshold plus rate
  checks.
- The IMU is not providing absolute orientation; it only helps as a yaw-rate
  source.
- Foot odometry uses measured joint FK but still lacks contact and slip sensing;
  it should remain low trust, and moving output is unavailable at 2 Hz.
- Depth camera information adds forward obstacle observations to Nav2, but it is
  not currently an EKF odometry input.

## Useful Runtime Checks

Check the raw and filtered odometry topics:

```bash
ros2 topic echo /scan_odom --once
ros2 topic echo /odometry/filtered --once
ros2 topic hz /lidar/filtered_laserscan
ros2 topic hz /scan_odom
```

Check the authoritative TF:

```bash
ros2 run tf2_ros tf2_echo odom base_frame
ros2 run tf2_ros tf2_echo base_frame lidar_frame
ros2 run tf2_ros tf2_echo base_frame imu_link
```

Check for duplicate odom TF publishers:

```bash
ros2 topic echo /tf
```

There should be only one active source for `odom -> base_frame` in the normal
EKF pipeline.

## Future Work

- Test odometry by teleoperating the robot through loops and returning to the
  start pose.
- Compare RF2O behavior with any separately reintroduced point-cloud ICP
  experiment only after the normal RF2O/EKF baseline is stable.
- Revisit EKF covariances after collecting repeatable bag data.
- Replace or decouple the blocking joint-read path, confirm that feedback is
  physical present position, then repeat the 10 Hz test with numeric endpoints
  before considering any increase in foot-odometry weight. Do not proceed to
  20 Hz on the current controller interface.
- Consider depth-camera odometry only as a separate future experiment; the
  current depth-camera path is an independent Nav2 costmap source.
