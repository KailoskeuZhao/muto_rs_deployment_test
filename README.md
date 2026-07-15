# Muto RS Deployment Test

This is a ROS 2 workspace for experimental deployment on a Muto RS / Yahboom-based robot platform. The current focus is local sensing, TF wiring, LiDAR/depth-camera scan generation, LiDAR odometry, EKF experiments, and SLAM mapping.

## Status

This repository is a work in progress.

Expect launch files, topic names, frames, calibration values, and filtering assumptions to change while the robot is being tested. This is not a polished upstream distribution and should not be treated as a stable reference implementation yet.

Hardware-specific assumptions are currently embedded in several places, especially frame names, sensor poses, IMU calibration constants, LiDAR/depth-camera filtering ranges, and SLAM/EKF parameters.

Detailed odometry and localization notes are in [docs/odometry.md](docs/odometry.md).
Launch-file roles and example startup sequences are in [docs/launches.md](docs/launches.md).

## Package Origins

Some packages in this workspace are original deployment glue, while others were imported or forked and then modified.

| Path | Notes |
| --- | --- |
| `src/yahboomcar_bringup` | Came from the Muto RS tutorial material and has been modified for this deployment. |
| `src/yahboomcar_ctrl` | Came from the Muto RS tutorial material and has been modified for this deployment. |
| `src/Simple-2D-LiDAR-Odometry` | Forked/imported from another GitHub repository and adapted for this workspace. Check the package's own README and license. |
| `src/simple_vlm` | Also carried as external/forked GitHub code in this workspace. Check the package's own README and license. |
| `src/lidar_pointcloud_filter` | Local filtering and scan-conversion utilities for this deployment. |
| `src/muto_slam_mapping` | Local SLAM launch/config package for this deployment. |
| `src/tf2_publisher` | Local TF publisher package for robot sensor frames. |
| `src/yahboomcar_imu` | IMU publishing package used by the robot. |
| `src/lidar_tg30` | TG30 LiDAR driver/package used by the robot. |
| `src/robot_localization` | Vendor/upstream `robot_localization` package kept inside this workspace. |

Because this workspace mixes local code, tutorial-derived code, and forked external packages, check package-level license files before redistributing any part of it.

## Build

From the workspace root:

```bash
cd ~/Documents/testground/muto_rs_deployment_test
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If your robot workspace lives somewhere else, run the same commands from that workspace root.

## Normal Startup Sequence

Use this sequence for the normal robot bringup. Do not launch
`camera_pointcloud_to_laserscan_launch.py` as a separate normal-startup step.
That launch is a component/test launch; mapping launches it internally when a
fused scan is needed.

Start the TG30 LiDAR, Orbbec depth camera, and Muto base driver:

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
```

This launch starts:

- `lidar_tg30/lidar_node`
- `orbbec_camera/astra_pro_plus.launch.py`
- `yahboomcar_bringup/muto_driver`

Start the sensor TF publishers:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

Start LiDAR scan filtering, RF2O laser odometry, and the EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

This launch includes `filter_lidar_odometry_launch.py` by default. The LiDAR
odom path is:

- `/lidar/raw_laserscan`
- `/lidar/filtered_laserscan` for RF2O
- `scan_odom_raw`
- `scan_odom`
- EKF output and EKF-owned `odom -> base_frame` TF

When launched through `ekf_imu_lidar_launch.py`, RF2O/deadband odometry TF
publishing is disabled and the EKF publishes the odom TF instead.

## Mapping And Nav2 Add-Ons

After the normal startup sequence is running, start online async SLAM Toolbox
mapping:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

This launch starts fused LaserScan generation for mapping. It assumes the normal
LiDAR odom/filter path is already running and producing
`/lidar/filtered_laserscan_no_downsample`. It does not start hardware, sensor
TF, LiDAR odometry, or the EKF.

If `/fused/laserscan` is already running:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py launch_fused_laserscan:=false
```

Run the Nav2 BT/planner/controller bringup after mapping, TF, EKF, and
`/fused/laserscan` are available:

```bash
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

This launch follows Nav2's normal server ownership model: `bt_navigator` hosts
the NavigateToPose action, `behavior_server` hosts recovery actions,
`controller_server` creates the local costmap, `planner_server` creates the
global costmap, and `smoother_server` hosts path smoothing. It lifecycle-manages
only those servers, so it does not start AMCL, route, waypoint, docking, or full
`nav2_bringup`. The default config is `nav2_params.yaml`.

Mapping/TF/scan inputs should already be running. The global costmap expects
`map -> base_frame`, and the local costmap expects `odom -> base_frame`. Both
costmaps use `/fused/laserscan`, `base_frame`, `odom`, and `map`.

## Experimental And Test Launches

Run the LiDAR odometry path without the EKF:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py
```

By default this uses the raw TG30 `LaserScan` path. It publishes:

- `/lidar/filtered_laserscan`, a downsampled filtered scan for RF2O
- `/lidar/filtered_laserscan_no_downsample`, a full-resolution filtered scan for fusion
- `scan_odom_raw`, RF2O output before deadband/jump filtering
- `scan_odom`, filtered odometry output

`filter_lidar_odometry_launch.py` keeps RF2O itself unmodified: RF2O publishes
raw odometry on `scan_odom_raw`, then
`lidar_pointcloud_filter/odometry_translation_deadband_node` republishes the
EKF-facing `scan_odom`. The wrapper applies a small per-update planar
translation deadband by default (`rf2o_translation_deadband:=0.0025`) so
stationary scan-match drift does not feed `/scan_odom`. At the default 20 Hz
RF2O rate, this accepts roughly `>=5 cm/s` planar motion. Set
`rf2o_translation_deadband:=0.0` to disable it, or tune the value in meters for
the robot. By default, these RF2O deadbands and jump caps are gated per axis by
recent `cmd_vel`: translation filters apply while commanded planar motion is
near zero, and yaw filters apply while commanded yaw is near zero.

Standalone filtered odometry publishes `odom -> base_frame` TF by default. Use
`rf2o_publish_tf:=false` when an EKF or another localization node owns odom TF.

To force the older PointCloud2 odom path for comparison:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py use_laserscan_pipeline:=false
```

Directly test camera scan conversion plus scan fusion:

```bash
ros2 launch lidar_pointcloud_filter camera_pointcloud_to_laserscan_launch.py
```

This is not part of the normal startup sequence. It expects an existing
`/lidar/filtered_laserscan_no_downsample` topic, normally produced by
`filter_lidar_odometry_launch.py` or `ekf_imu_lidar_launch.py`. It converts
`/camera/depth/points` to `/camera/filtered_laserscan`, then fuses that with the
LiDAR scan into `/fused/laserscan`.

If `/scan_odom` is already being produced by another launch and you only want
the EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py launch_lidar_odometry:=false
```

Rough gait/cmd_vel dead-reckoned odometry is disabled by default. To launch it
and fuse `/foot_odom` into the EKF as a low-trust planar velocity source:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py launch_foot_odometry:=true
```

To test the EKF with only `/imu/data_processed` and no LiDAR or foot odometry:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py imu_only:=true
```

The smaller `nav2_costmap_params.yaml` is still kept and can be passed to Nav2
with `params_file:=...` if needed.

YAML files such as `ekf_lidar_imu.yaml` and `mapper_params_online_async.yaml` are parameter files, not launch files. Launch the matching `.py` file and let it load the YAML.

## Main Topics

| Topic | Purpose |
| --- | --- |
| `/lidar/raw_laserscan` | Raw TG30 `LaserScan`; default input to LiDAR scan filtering. |
| `/lidar/filtered_laserscan` | Downsampled filtered LiDAR `LaserScan`; default input to RF2O. |
| `/lidar/filtered_laserscan_no_downsample` | Full-resolution filtered LiDAR `LaserScan`; default LiDAR input for scan fusion. |
| `/lidar/PointCloud` | Legacy raw LiDAR `PointCloud2`, still published by default for compatibility. |
| `/lidar/PointCloudFiltered` | Legacy filtered and voxel-downsampled LiDAR `PointCloud2`. |
| `/lidar/PointCloudFilteredNoDownsample` | Legacy filtered LiDAR `PointCloud2` before voxel downsampling. |
| `/camera/depth/points` | Depth camera `PointCloud2`. |
| `/camera/filtered_laserscan` | Intermediate camera `LaserScan` generated from `/camera/depth/points`. |
| `/fused/laserscan` | Fused `LaserScan` generated from `/camera/filtered_laserscan` and `/lidar/filtered_laserscan_no_downsample`. |
| `/imu/data_processed` | Processed IMU message used by localization experiments. |
| `scan_odom` | LiDAR odometry output topic used by downstream localization. |
| `/foot_odom` | Rough command/gait odometry from `cmd_vel` and motor-angle activity. Optional EKF input when `launch_foot_odometry:=true`. |

## Fused LaserScan Notes

`camera_pointcloud_to_laserscan_launch.py` is a component/test launch. It
currently builds `/fused/laserscan` from:

- `/camera/depth/points`, converted to `/camera/filtered_laserscan`
- `/lidar/filtered_laserscan_no_downsample`

The no-downsample LiDAR scan is used so the final scan has enough angular
samples. The downsampled `/lidar/filtered_laserscan` topic is reserved for RF2O.

Current defaults:

| Setting | Default | Notes |
| --- | --- | --- |
| `output_topic` | `/fused/laserscan` | Final synthetic scan. |
| `camera_scan_topic` | `/camera/filtered_laserscan` | Intermediate camera scan. |
| `lidar_scan_topic` | `/lidar/filtered_laserscan_no_downsample` | Existing LiDAR scan consumed by fusion. |
| `processing_frame` | `base_frame` | Camera cloud is transformed here before scan projection. |
| `fused_scan_frame` | `base_frame` | Final fused scan frame. |
| `angle_min` / `angle_max` | `-pi` / `pi` | Full-circle scan output. |
| `range_max` | `3.0` | Depth camera points are capped at 3 m. |
| `lidar_range_max` | `15.0` | Fused output range cap for LiDAR scan points. |
| `min_z` / `max_z` | `-0.2` / `0.05` | Z slice applied in `processing_frame`. |
| `input_point_stride` | `8` | Process every 8th depth-camera point before scan projection. |
| `require_lidar_scan` | `true` | Wait for a timestamp-matched LiDAR scan before publishing fused output. |

When `require_lidar_scan:=true`, fusion waits until a valid LiDAR scan is
available instead of publishing camera-only scans during startup.
If a live depth-camera cloud contains no points, it is still converted into an
empty all-infinity `/camera/filtered_laserscan` with the cloud timestamp. Fusion
then combines that camera scan with the latest timestamp-compatible LiDAR scan,
so temporary depth-camera no-return frames do not block `/fused/laserscan`.

## Frame Notes

The current TF setup expects robot sensor frames such as:

| Frame | Notes |
| --- | --- |
| `base_frame` | Robot base frame used by several launch/config files. |
| `camera_link` | Camera body frame used for depth-cloud processing. |
| `camera_depth_optical_frame` | Depth camera optical frame where depth points may originate. |
| `lidar_frame` | LiDAR frame used by the TG30 point cloud. |
| `imu_link` | IMU frame. |

The depth camera point cloud may arrive in `camera_depth_optical_frame`. The
fusion path uses TF2 to transform sampled points into the configured
`processing_frame`, which is `base_frame` by default, before applying the
deployment-specific z/range bounds and projecting to LaserScan.

## Development Notes

- Keep TF publishers running before debugging point-cloud fusion or odometry.
- If a transform appears missing on startup, wait a moment and re-check with `tf2_echo`; some nodes may start before the TF buffer has received all frames.
- Prefer launching `.py` launch files. Parameter YAML files are loaded by launch files or nodes.
- Calibration and filtering values are still experimental. Re-check them on the actual robot before relying on mapping or navigation results.
