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

Start the sensor TF publishers:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

Run LiDAR PointCloud filtering plus 2D LiDAR odometry:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py
```

Convert camera depth points plus LiDAR points into a fused `LaserScan`:

```bash
ros2 launch lidar_pointcloud_filter camera_pointcloud_to_laserscan_launch.py
```

Run the IMU + LiDAR odometry EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

Run online async SLAM Toolbox mapping:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

YAML files such as `ekf_lidar_imu.yaml` and `mapper_params_online_async.yaml` are parameter files, not launch files. Launch the matching `.py` file and let it load the YAML.

## Main Topics

| Topic | Purpose |
| --- | --- |
| `/lidar/PointCloud` | Raw LiDAR `PointCloud2`. |
| `/lidar/PointCloudFiltered` | Filtered LiDAR `PointCloud2`, usually produced by `lidar_pointcloud_filter_node`. |
| `/camera/depth/points` | Depth camera `PointCloud2`. |
| `/fused/laserscan` | Synthetic/fused `LaserScan` generated from camera depth points and LiDAR points. |
| `/imu/data_processed` | Processed IMU message used by localization experiments. |
| `scan_odom` | LiDAR odometry output topic used by downstream localization. |

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
