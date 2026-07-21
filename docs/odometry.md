# Odometry And Localization Notes

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
| [`src/lidar_tg30/src/lidar_node.cpp`](../src/lidar_tg30/src/lidar_node.cpp) | Publishes raw TG30 LaserScan and optional legacy PointCloud2. |
| [`src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py`](../src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py) | Starts LiDAR scan filtering, RF2O, and the odometry deadband wrapper. |
| [`src/lidar_pointcloud_filter/src/lidar_laserscan_filter_node.cpp`](../src/lidar_pointcloud_filter/src/lidar_laserscan_filter_node.cpp) | Filters raw LiDAR LaserScan into RF2O and fusion scan topics. |
| [`src/lidar_pointcloud_filter/src/odometry_translation_deadband_node.cpp`](../src/lidar_pointcloud_filter/src/odometry_translation_deadband_node.cpp) | Applies RF2O deadbands and jump rejection before publishing `scan_odom`. |
| [`src/yahboomcar_bringup/launch/ekf_imu_lidar_launch.py`](../src/yahboomcar_bringup/launch/ekf_imu_lidar_launch.py) | Main EKF launch for LiDAR plus IMU odometry. |
| [`src/yahboomcar_bringup/config/ekf_lidar_imu.yaml`](../src/yahboomcar_bringup/config/ekf_lidar_imu.yaml) | Default EKF fusion configuration. |
| [`src/yahboomcar_imu/yahboomcar_imu/imu_node.py`](../src/yahboomcar_imu/yahboomcar_imu/imu_node.py) | Publishes raw and processed IMU messages. |
| [`src/yahboomcar_bringup/yahboomcar_bringup/foot_odometry_node.py`](../src/yahboomcar_bringup/yahboomcar_bringup/foot_odometry_node.py) | Optional command/gait dead-reckoned odometry. |
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

`muto_hardware_launch.py` starts `lidar_tg30/lidar_node`. The TG30 node can
publish both:

| Topic | Type | Frame | Current role |
| --- | --- | --- | --- |
| `/lidar/raw_laserscan` | `sensor_msgs/LaserScan` | `lidar_frame` | Normal RF2O input before filtering. |
| `lidar/PointCloud` | `sensor_msgs/PointCloud2` | `lidar_frame` | Legacy point-cloud path. |

The LaserScan path is the current default. The point-cloud path remains
available for comparison and older experiments.

## LiDAR Scan Filtering

`filter_lidar_odometry_launch.py` starts
`lidar_pointcloud_filter/lidar_laserscan_filter_node` when
`use_laserscan_pipeline:=true`, which is the default.

It consumes `/lidar/raw_laserscan` and publishes two scans:

| Topic | Purpose | Default filtering |
| --- | --- | --- |
| `/lidar/filtered_laserscan` | RF2O odometry input | `range_min=0.05`, `range_max=10.0`, full circle, downsample factor `2`. |
| `/lidar/filtered_laserscan_no_downsample` | Mapping/fused scan LiDAR input | Full resolution, `range_min=0.05`, `range_max=15.0`. |

The node preserves input timestamps by default. `scan_restamp_output:=false` is
intentional; restamping should only be used when a driver is known to publish bad
timestamps while the data itself is fresh.

The older point-cloud branch can be selected with:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py use_laserscan_pipeline:=false
```

That branch transforms `lidar/PointCloud` into `base_frame`, filters it with PCL,
and converts the filtered point cloud into a LaserScan for RF2O. It is not the
normal startup path.

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
| `freq` | `20.0` Hz |
| `publish_tf` | `false` inside this launch |

RF2O uses TF2 to look up the transform from the scan frame to `base_frame` on
the first scan. This lets the raw scan remain in `lidar_frame`; the odometry
result is still expressed for `base_frame`.

RF2O publishes `scan_odom_raw`. It does not own the final `odom -> base_frame`
TF in the normal EKF pipeline.

## RF2O Deadband And Jump Rejection

`odometry_translation_deadband_node` wraps RF2O output:

```text
scan_odom_raw -> scan_odom
```

Current default filters:

| Setting | Default | Meaning |
| --- | --- | --- |
| `translation_deadband` | `0.0025` m | Suppress tiny per-update XY drift; at 20 Hz RF2O, this accepts roughly `>=5 cm/s`. |
| `yaw_deadband` | `0.001` rad | Suppress tiny per-update yaw drift. |
| `translation_jump_rejection_threshold` | `0.03` m | Reject RF2O XY updates above 3 cm per update while commanded translation is near zero. |
| `max_translation_rate` | `0.0` m/s | Disabled so translation jump rejection uses only the per-update 3 cm cap. |
| `yaw_jump_rejection_threshold` | `0.087266` rad | Reject RF2O yaw updates above 5 deg per update while commanded yaw is near zero. |
| `max_yaw_rate` | `0.0` rad/s | Disabled so yaw jump rejection uses only the per-update 5 deg cap. |
| `use_cmd_vel_gate` | `true` | Apply RF2O deadbands and jump caps per axis only when recent `cmd_vel` for that axis is near zero. |
| `cmd_vel_timeout` | `0.5` s | If no fresh `cmd_vel` is seen, assume stationary and apply the filters. |
| `cmd_vel_stationary_linear_threshold` | `0.03` m/s | Translation filters apply at or below this commanded planar speed. |
| `cmd_vel_stationary_angular_threshold` | `0.03` rad/s | Yaw filters apply at or below this commanded yaw rate. |

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

## Optional Foot Odometry

Foot/gait odometry is disabled by default. It can be launched with:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py launch_foot_odometry:=true
```

`foot_odometry_node` is not contact-sensed foot odometry. It:

- listens to `cmd_vel`;
- mirrors the Muto gait command mapping;
- polls the `get_motor_angles` service for rough motion evidence;
- integrates a high-covariance dead-reckoned pose;
- publishes `/foot_odom`.

In `ekf_lidar_imu_with_foot.yaml`, `/foot_odom` contributes only planar body
velocity (`vx`, `vy`). Pose and yaw still come from RF2O, and yaw rate still
comes from the IMU. The node is launched with `publish_tf:=false`.

## IMU-Only EKF Test

`ekf_imu_lidar_launch.py imu_only:=true` starts an EKF with
`ekf_imu_only.yaml`. That configuration only fuses IMU yaw rate. It is useful as
a wiring test for `/imu/data_processed`, but it is not a complete mobile-base
odometry source because it has no translational input and no absolute yaw input.

## Removed Legacy LiDAR Odometry Package

`src/Simple-2D-LiDAR-Odometry` was removed from the active workspace. The
current odometry pipeline is the TG30 `LaserScan` path through
`lidar_pointcloud_filter`, `rf2o_laser_odometry`, the deadband wrapper, and the
EKF.

## Mapping And Nav2 Relationship

SLAM and Nav2 rely on odometry but do not replace the EKF odom source.

`muto_slam_mapping/config/mapper_params_online_async.yaml` configures
SLAM Toolbox with:

```text
odom_frame: odom
map_frame: map
base_frame: base_frame
scan_topic: /fused/laserscan
```

`online_async_mapping_launch.py` starts fused LaserScan generation by default.
That fused scan combines:

```text
/camera/filtered_laserscan
/lidar/filtered_laserscan_no_downsample
```

into:

```text
/fused/laserscan
```

The fused scan is used for mapping and Nav2 costmaps. It is not currently fused
into the EKF.

Nav2 costmaps are configured around the same frame chain:

- local costmap: `global_frame=odom`, `robot_base_frame=base_frame`;
- global costmap: `global_frame=map`, `robot_base_frame=base_frame`;
- both consume `/fused/laserscan`.

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
- Foot odometry is only command/dead-reckoned and should remain low trust.
- Depth camera information improves the fused scan for mapping/Nav2, but it is
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
- Decide whether `/foot_odom` should remain optional or be removed if it does
  not improve robustness.
- Consider depth-camera odometry only as a separate future experiment; the
  current depth-camera path is for scan fusion, mapping, and Nav2 costmaps.
