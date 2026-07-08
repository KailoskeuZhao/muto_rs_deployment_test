# Muto RS Deployment Test

This is a ROS 2 workspace for experimental deployment on a Muto RS / Yahboom-based robot platform. The current focus is local sensing, TF wiring, LiDAR/depth-camera scan generation, LiDAR odometry, EKF experiments, and SLAM mapping.

## Status

This repository is a work in progress.

Expect launch files, topic names, frames, calibration values, and filtering assumptions to change while the robot is being tested. This is not a polished upstream distribution and should not be treated as a stable reference implementation yet.

Hardware-specific assumptions are currently embedded in several places, especially frame names, sensor poses, IMU calibration constants, LiDAR/depth-camera filtering ranges, and SLAM/EKF parameters.

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

## Common Launches

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

Run LiDAR PointCloud filtering, filtered-cloud LaserScan conversion, and RF2O laser odometry:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py
```

Standalone RF2O publishes `odom -> base_frame` TF by default. When launched through `ekf_imu_lidar_launch.py`, RF2O TF publishing is disabled and the EKF publishes the odom TF instead.

Convert camera depth points plus LiDAR points into a fused `LaserScan`:

```bash
ros2 launch lidar_pointcloud_filter camera_pointcloud_to_laserscan_launch.py
```

This launch does not start LiDAR filtering. It expects an existing LiDAR `PointCloud2` topic, defaulting to `/lidar/PointCloudFilteredNoDownsample`. Start `filter_lidar_odometry_launch.py` or `ekf_imu_lidar_launch.py` first if that topic is not already running.

To consume a different existing LiDAR cloud, override `lidar_topic`.

Run LiDAR PointCloud filtering, RF2O laser odometry, and the EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

If `/scan_odom` is already being produced by another launch:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py launch_lidar_odometry:=false
```

Rough gait/cmd_vel dead-reckoned odometry is disabled by default. To launch it and fuse `/foot_odom` into the EKF as a low-trust planar velocity source:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py launch_foot_odometry:=true
```

To test the EKF with only `/imu/data_processed` and no LiDAR or foot odometry:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py imu_only:=true
```

Run online async SLAM Toolbox mapping plus fused LaserScan conversion:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

This launch assumes the upstream LiDAR filtered cloud already exists, usually `/lidar/PointCloudFilteredNoDownsample` from `filter_lidar_odometry_launch.py` or `ekf_imu_lidar_launch.py`. It does not start LiDAR filtering.

It launches fused LaserScan generation by default because `mapper_params_online_async.yaml` uses `/fused/laserscan` as `scan_topic`. If `/fused/laserscan` is already running:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py launch_fused_laserscan:=false
```

Run standalone Nav2 global/local costmaps:

```bash
ros2 launch muto_slam_mapping nav2_costmaps_launch.py
```

This launch starts only two standalone `nav2_costmap_2d` lifecycle nodes plus a lifecycle manager. It does not start `controller_server`, `planner_server`, BT navigation, AMCL, or full `nav2_bringup`. The default config is the fuller `nav2_params.yaml`, but only the costmap sections are used by this launch. The costmaps use `/fused/laserscan`, `base_frame`, `odom`, and `map`.

The smaller `nav2_costmap_params.yaml` is still kept and can be passed with `params_file:=...` if needed.

Mapping/TF/scan inputs should already be running. The global costmap expects `map -> base_frame`, and the local costmap expects `odom -> base_frame`.

YAML files such as `ekf_lidar_imu.yaml` and `mapper_params_online_async.yaml` are parameter files, not launch files. Launch the matching `.py` file and let it load the YAML.

## Main Topics

| Topic | Purpose |
| --- | --- |
| `/lidar/PointCloud` | Raw LiDAR `PointCloud2`. |
| `/lidar/PointCloudFiltered` | Filtered and voxel-downsampled LiDAR `PointCloud2`, currently using `voxel_leaf_size:=0.02` by default. This is the default LiDAR input for RF2O scan conversion. |
| `/lidar/PointCloudFilteredNoDownsample` | Filtered LiDAR `PointCloud2` before voxel downsampling. This is the default LiDAR input for fused LaserScan generation. |
| `/lidar/filtered_laserscan` | LiDAR-only synthetic `LaserScan` generated from `/lidar/PointCloudFiltered`; default input to RF2O. |
| `/camera/depth/points` | Depth camera `PointCloud2`. |
| `/fused/laserscan` | Synthetic/fused `LaserScan` generated from camera depth points and LiDAR points. |
| `/imu/data_processed` | Processed IMU message used by localization experiments. |
| `scan_odom` | LiDAR odometry output topic used by downstream localization. |
| `/foot_odom` | Rough command/gait odometry from `cmd_vel` and motor-angle activity. Optional EKF input when `launch_foot_odometry:=true`. |

## Fused LaserScan Notes

`camera_pointcloud_to_laserscan_launch.py` currently builds `/fused/laserscan` from:

- `/camera/depth/points`
- `/lidar/PointCloudFilteredNoDownsample`

The no-downsample LiDAR topic is used so the final scan has enough angular samples. The downsampled `/lidar/PointCloudFiltered` topic still exists for workflows that want a lighter cloud.

Current defaults:

| Setting | Default | Notes |
| --- | --- | --- |
| `output_topic` | `/fused/laserscan` | Final synthetic scan. |
| `processing_frame` | `camera_link` | Camera and LiDAR clouds are transformed here before scan projection. |
| `angle_min` / `angle_max` | `-pi` / `pi` | Full-circle scan output. |
| `range_max` | `3.0` | Depth camera points are capped at 3 m. |
| `lidar_range_max` | `15.0` | LiDAR points are capped at 15 m. |
| `min_z` / `max_z` | `-0.4` / `0.2` | Z slice applied in `processing_frame`. |
| `lidar_topic` | `/lidar/PointCloudFilteredNoDownsample` | Existing LiDAR cloud consumed by the scan converter. |

When `use_lidar:=true`, the scan node waits until a valid LiDAR cloud is available instead of publishing camera-only scans during startup.

## Frame Notes

The current TF setup expects robot sensor frames such as:

| Frame | Notes |
| --- | --- |
| `base_frame` | Robot base frame used by several launch/config files. |
| `camera_link` | Camera body frame used for depth-cloud processing. |
| `camera_depth_optical_frame` | Depth camera optical frame where depth points may originate. |
| `lidar_frame` | LiDAR frame used by the TG30 point cloud. |
| `imu_link` | IMU frame. |

The depth camera point cloud may arrive in `camera_depth_optical_frame`, but filtering/projection logic may transform it into `camera_link` before applying deployment-specific bounds.

## Development Notes

- Keep TF publishers running before debugging point-cloud fusion or odometry.
- If a transform appears missing on startup, wait a moment and re-check with `tf2_echo`; some nodes may start before the TF buffer has received all frames.
- Prefer launching `.py` launch files. Parameter YAML files are loaded by launch files or nodes.
- Calibration and filtering values are still experimental. Re-check them on the actual robot before relying on mapping or navigation results.
