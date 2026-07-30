# Launch Reference

Date: 2026-07-28

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
| `muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Includes hardware, static sensor TF, LiDAR odometry/EKF, online async mapping, fused scan generation, and Nav2 planner/controller/action servers with minimum delays and topic/TF readiness gates. | One-shot full robot Nav2 pipeline. Use this for normal bringup once the robot dependencies are installed. |
| `yahboomcar_bringup/launch/muto_hardware_launch.py` | `lidar_tg30/lidar_node`, Orbbec `astra_pro_plus.launch.py`, and `yahboomcar_bringup/muto_driver`. | Hardware source layer. Run first on the robot. |
| `tf2_publisher/launch/all_tf2_publishers_launch.py` | Static TF publishers for `base_frame -> camera_link`, `base_frame -> lidar_frame`, and `base_frame -> imu_link`. Optional odom TF publisher is off by default. | Sensor TF layer. Needed before scan conversion, RF2O, mapping, and Nav2. |
| `lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | Default path filters `/lidar/raw_laserscan` into `/lidar/filtered_laserscan` and `/lidar/filtered_laserscan_no_downsample`, then runs RF2O and the odometry deadband/jump wrapper. | LiDAR odometry chain. Direct standalone launch lets the wrapper publish `odom -> base_frame` by default. |
| `yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Includes the LiDAR odometry launch with odom TF disabled, starts motor-validated commanded-stance `/foot_odom` by default, then runs the installed `robot_localization/ekf_node`. Set `launch_foot_odometry:=false` to disable the foot input. | Preferred odometry/localization layer. EKF owns `odom -> base_frame`. |
| `lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Converts `/camera/depth/image_raw` plus CameraInfo to `/camera/filtered_laserscan` and merges it into the LiDAR-driven `/fused/laserscan`; missing or stale camera scans automatically produce LiDAR-only output. | Component/test launch. Mapping includes it internally when `launch_fused_laserscan:=true`; do not launch separately during normal startup unless testing. |
| `muto_slam_mapping/launch/online_async_mapping_launch.py` | Starts fused LaserScan generation by default, then starts SLAM Toolbox online async mapping. | Mapping layer. Uses `/fused/laserscan` and the EKF odom TF to maintain the map relationship. |
| `muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Starts `controller_server`, `planner_server`, path `smoother_server`, `velocity_smoother`, `behavior_server`, `bt_navigator`, and lifecycle manager. | Current Nav2 planner/controller/action bringup. Requires mapping, TF, EKF, and `/fused/laserscan` already running. |
| `muto_slam_mapping/launch/frontier_exploration_launch.py` | Starts the submodule's `frontier_explorer` with the Muto-specific map, costmap, TF, Nav2 action, QoS, and bounded-DP configuration. | Optional autonomous exploration client. Start only after mapping and Nav2 are ready; it is not included by the one-shot Nav2 launch. |
| `muto_command_layer/launch/object_pipeline_launch.py` | Starts the SAM2 image annotator, C++ object registry, VLM socket, go-to-object server, and natural-language object-search server. | Independent object/perception pipeline. It consumes camera/TF and delegates `/go_to_object` to an already-running Nav2 `/navigate_to_pose`; it is not included by the Nav2 launch. |
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
  -> planner/path-smoother/velocity-smoother/behavior/bt_navigator servers
  -> controller /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  -> recovery behaviors -------------------------------> /cmd_vel

frontier_exploration_launch.py                 (optional, separate process)
  -> /map + local/global costmaps + TF
  -> /navigate_to_pose

object_pipeline_launch.py                      (independent process group)
  -> YOLO/SAM2 -> instance cloud -> object registry
  -> /find_object through /vlm/generate
  -> /go_to_object through Nav2 /navigate_to_pose
```

Only one node should publish dynamic `odom -> base_frame` at a time. In the
normal EKF sequence, that node is the EKF.

## Normal Startup Sequence

Source the deployed workspace before running either the one-shot launch or the
layer-by-layer debug commands:

```bash
cd /opt/muto_rs_ws
. install/setup.bash
```

Full one-shot pipeline:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

The localization, mapping, and Nav2 includes start only after their required live
topics and TF chains are available. The delay arguments are minimum offsets; the
`*_readiness_timeout` arguments bound each wait and shut down the pipeline on
failure.

The one-shot launch ends at Nav2. It deliberately does **not** start SAM2, the
object registry, the VLM bridge, the command layer, or frontier exploration.
This keeps navigation usable without the GPU or a network VLM provider and
prevents unrelated action clients from being started implicitly.

Subsystem includes are configuration-scoped. Generic child argument names such as
`input_topic` and `lidar_scan_topic` therefore cannot inherit unrelated values
from hardware or localization launches. Mapping explicitly binds camera input to
`/camera/depth/image_raw` plus `/camera/depth/camera_info` and fusion input to
`/lidar/filtered_laserscan_no_downsample`.

Layer-by-layer startup for debugging:

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

Optionally start frontier exploration only after Nav2 is active:

```bash
ros2 launch muto_slam_mapping frontier_exploration_launch.py
```

Start the independent object pipeline in another terminal:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer object_pipeline_launch.py
```

The checked-in object-pipeline VLM profile uses a plain-HTTP provider. Keep it
on a trusted network, VPN, or tunnel, or replace it with an HTTPS endpoint.

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
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
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

Expected nodes include:

- `/controller_server`
- `/planner_server`
- `/smoother_server`
- `/velocity_smoother`
- `/behavior_server`
- `/bt_navigator`
- `/lifecycle_manager_costmaps`
- `/local_costmap/local_costmap`
- `/global_costmap/global_costmap`

This launch is not a full `nav2_bringup` replacement with AMCL, route server,
waypoint follower, docking, or other optional Nav2 servers.

The controller publishes `/cmd_vel_nav`; the velocity smoother publishes the
normal follow-path `/cmd_vel` sent to the Muto driver. Recovery behaviors are
also lifecycle-managed but currently publish directly to `/cmd_vel`, bypassing
the smoother while retaining their own conservative velocity and acceleration
limits.

### Object Perception And Commands

Run this beside, not inside, the Nav2 pipeline:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer object_pipeline_launch.py
```

Expected high-level interfaces include:

- `/sam2/instance_pointcloud`
- `/sam2/stored_objects`
- `/sam2/stored_object_markers`
- `/sam2/get_stored_objects`
- `/vlm/generate`
- `/find_object`
- `/go_to_object`

Detection, registry query, visualization, and `/find_object` can be diagnosed
independently of Nav2. `/go_to_object` requires the separate
`/navigate_to_pose` server and the complete `map -> base_frame` TF chain.

### Frontier Exploration

After mapping and Nav2 are active:

```bash
ros2 launch muto_slam_mapping frontier_exploration_launch.py
```

The launch enables the package CLI control service by default. The
`src/frontier_exploration` source is a Git submodule; deployment-specific
parameters remain in
`muto_slam_mapping/config/frontier_exploration_params.yaml`.

```bash
frontier_exploration_ctl stop
frontier_exploration_ctl start
frontier_exploration_ctl stop -q
```

The Muto profile autostarts exploration. `stop` enters cold idle, `start`
resumes after fresh map/costmap data arrive, and `stop -q` also terminates the
explorer process without stopping the parent Nav2 pipeline.

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

Check final command routing:

```bash
ros2 topic info /cmd_vel_nav --verbose
ros2 topic info /cmd_vel --verbose
ros2 lifecycle get /velocity_smoother
```

If TF message filters occasionally drop scan messages during startup, that can
be ordinary TF cache timing. If drops continue after startup, inspect scan
timestamps and `odom -> base_frame` / `map -> base_frame` availability.
