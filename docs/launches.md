# Launch Reference

Date: 2026-07-17

This document summarizes the launch files that matter for the current Muto RS
workspace. It separates the normal robot sequence from experimental and
component launches.

## Removed Packages

`src/Simple-2D-LiDAR-Odometry` and `src/simple_vlm` were removed from the
active workspace. The current launch files do not include them, and no active
package declares them as a dependency.

## Launch File Summary

`robot_localization` is an external ROS Humble dependency. The workspace launch
files invoke its installed `ekf_node`; this workspace does not keep
`robot_localization` under `src/`.

| Launch file | What it starts | Usual role |
| --- | --- | --- |
| `yahboomcar_bringup/launch/muto_hardware_launch.py` | `lidar_tg30/lidar_node`, Orbbec `astra_pro_plus.launch.py`, and `yahboomcar_bringup/muto_driver`. | Hardware source layer. Run first on the robot. |
| `tf2_publisher/launch/all_tf2_publishers_launch.py` | Static TF publishers for `base_frame -> camera_link`, `base_frame -> lidar_frame`, and `base_frame -> imu_link`. Optional odom TF publisher is off by default. | Sensor TF layer. Needed before scan conversion, RF2O, mapping, and Nav2. |
| `lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | Default path filters `/lidar/raw_laserscan` into `/lidar/filtered_laserscan` and `/lidar/filtered_laserscan_no_downsample`, then runs RF2O and the odometry deadband/jump wrapper. | LiDAR odometry chain. Direct standalone launch lets the wrapper publish `odom -> base_frame` by default. |
| `yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Includes the LiDAR odometry launch with odom TF disabled, optionally starts `/foot_odom`, then runs the installed `robot_localization/ekf_node`. | Preferred odometry/localization layer. EKF owns `odom -> base_frame`. |
| `lidar_pointcloud_filter/launch/camera_pointcloud_to_laserscan_launch.py` | Converts `/camera/depth/points` to `/camera/filtered_laserscan`, fuses it with `/lidar/filtered_laserscan_no_downsample`, and publishes `/fused/laserscan`. Can also run a legacy LiDAR PointCloud2-to-LaserScan utility path. | Component/test launch. Mapping includes it internally when `launch_fused_laserscan:=true`; do not launch separately during normal startup unless testing. |
| `muto_slam_mapping/launch/online_async_mapping_launch.py` | Starts fused LaserScan generation by default, then starts SLAM Toolbox online async mapping. | Mapping layer. Uses `/fused/laserscan` and the EKF odom TF to maintain the map relationship. |
| `muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Starts `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, and lifecycle manager. | Current Nav2 planner/controller/action bringup. Requires mapping, TF, EKF, and `/fused/laserscan` already running. |
| `muto_slam_mapping/launch/nav2_costmaps_launch.py` | Wrapper around `nav2_planner_controller_launch.py` using the same Nav2 params. | Compatibility/alias launch. Despite the name, it is no longer costmaps-only. |
| `yahboomcar_bringup/launch/bringup_launch.py` | Starts only `yahboomcar_bringup/muto_driver`. Does not start robot description TF, LiDAR, Orbbec, or the current odom/mapping stack. | Older/simple driver launch. Not the normal robot hardware launch for this workspace. |
| `yahboomcar_ctrl/launch/yahboomcar_joy_launch.py` | Starts `joy_node` and `yahboom_joy`. | Joystick teleop. |

## Main Ownership Graph

```text
muto_hardware_launch.py
  -> raw LiDAR, camera SDK launch, Muto driver, IMU topics

all_tf2_publishers_launch.py
  -> base_frame -> lidar_frame
  -> base_frame -> imu_link
  -> base_frame -> camera_link

filter_lidar_odometry_launch.py
  -> /lidar/filtered_laserscan
  -> /lidar/filtered_laserscan_no_downsample
  -> scan_odom_raw
  -> scan_odom

ekf_imu_lidar_launch.py
  -> /odometry/filtered
  -> authoritative odom -> base_frame TF

online_async_mapping_launch.py
  -> /camera/filtered_laserscan
  -> /fused/laserscan
  -> slam_toolbox map relation

nav2_planner_controller_launch.py
  -> local_costmap and global_costmap
  -> planner/controller/smoother/behavior/bt_navigator servers
```

Only one node should publish dynamic `odom -> base_frame` at a time. In the
normal EKF sequence, that node is the EKF.

## Normal Startup Sequence

Use separate terminals and source the workspace in each one:

```bash
cd ~/fast_vivo_deployment_ws
. install/setup.bash
```

Start hardware:

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
```

Start static sensor TF:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

Start LiDAR odometry and EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

Start mapping:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

Start Nav2 planner/controller/costmaps:

```bash
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

The older command below is still available as a wrapper, but the clearer name is
`nav2_planner_controller_launch.py`:

```bash
ros2 launch muto_slam_mapping nav2_costmaps_launch.py
```

## Example Sequences

### Full Mapping Plus Nav2

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
ros2 launch tf2_publisher all_tf2_publishers_launch.py
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
ros2 launch muto_slam_mapping online_async_mapping_launch.py
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

Expected high-level outputs:

- `/lidar/raw_laserscan`
- `/lidar/filtered_laserscan`
- `/lidar/filtered_laserscan_no_downsample`
- `/imu/data_processed`
- `/scan_odom`
- `/odometry/filtered`
- `/camera/filtered_laserscan`
- `/fused/laserscan`
- `odom -> base_frame` from EKF
- `map` relation from SLAM Toolbox
- Nav2 local and global costmaps

### LiDAR Odometry Only

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
ros2 launch tf2_publisher all_tf2_publishers_launch.py
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py
```

This direct launch is useful for RF2O testing without the EKF. The deadband
wrapper publishes `odom -> base_frame` by default in this mode. If any other
localization node is publishing odom TF, use:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py rf2o_publish_tf:=false
```

### EKF Odometry Only

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
ros2 launch tf2_publisher all_tf2_publishers_launch.py
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

This is the preferred odometry-only sequence. It starts RF2O and the EKF, with
the EKF owning `odom -> base_frame`.

### Fused Scan Test

This requires hardware, static TF, and the LiDAR odometry/filter path already
running so `/lidar/filtered_laserscan_no_downsample` exists.

```bash
ros2 launch lidar_pointcloud_filter camera_pointcloud_to_laserscan_launch.py
ros2 topic hz /fused/laserscan
```

The component launch publishes:

- `/camera/filtered_laserscan`
- `/fused/laserscan`

It should not be launched separately during normal mapping if
`online_async_mapping_launch.py` is already running with
`launch_fused_laserscan:=true`.

### Mapping Without Starting Fused Scan

Use this only if another process is already publishing `/fused/laserscan`:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py launch_fused_laserscan:=false
```

SLAM Toolbox will still subscribe to `/fused/laserscan`.

### Costmaps/Nav2 After SLAM Is Running

Preferred:

```bash
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

Compatibility wrapper:

```bash
ros2 launch muto_slam_mapping nav2_costmaps_launch.py
```

Expected nodes include:

- `/controller_server`
- `/planner_server`
- `/smoother_server`
- `/behavior_server`
- `/bt_navigator`
- `/lifecycle_manager_costmaps`
- `/local_costmap/local_costmap`
- `/global_costmap/global_costmap`

This launch is not a full `nav2_bringup` replacement with AMCL, route server,
waypoint follower, docking, or other optional Nav2 servers.

### Joystick Teleop

```bash
ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py
```

This starts joystick input and the Yahboom joystick command node. Hardware
bringup still needs to be running for `cmd_vel` to move the robot.

## Common Checks

Check odometry and TF:

```bash
ros2 topic echo /scan_odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_frame
```

Check mapping scan:

```bash
ros2 topic hz /fused/laserscan
ros2 topic echo /fused/laserscan/header --once
```

Check Nav2 servers:

```bash
ros2 node list | grep -E 'controller|planner|smoother|behavior|bt_navigator|costmap'
```

If TF message filters occasionally drop scan messages during startup, that can
be ordinary TF cache timing. If drops continue after startup, inspect scan
timestamps and `odom -> base_frame` / `map -> base_frame` availability.
