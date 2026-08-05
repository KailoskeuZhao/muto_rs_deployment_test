# Launch Reference

Date: 2026-08-05

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
| `muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Includes hardware, static sensor TF, LiDAR odometry/EKF, online async mapping, camera depth projection, and Nav2 planner/controller/action servers with minimum delays and topic/TF readiness gates. | One-shot full robot Nav2 pipeline. Use this for normal bringup once the robot dependencies are installed. |
| `yahboomcar_bringup/launch/muto_hardware_launch.py` | `lidar_tg30/lidar_node`, Orbbec `astra_pro_plus.launch.py`, and the fixed-rate `yahboomcar_bringup/muto_driver` locomotion loop. | Hardware source layer. Run first on the robot. The initial driver tick commands standby, so support the robot and keep its legs clear. |
| `tf2_publisher/launch/all_tf2_publishers_launch.py` | Static TF publishers for `base_frame -> camera_link`, `base_frame -> lidar_frame`, and `base_frame -> imu_link`. Optional odom TF publisher is off by default. | Sensor TF layer. Needed before scan conversion, RF2O, mapping, and Nav2. |
| `lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | Default path filters `/lidar/raw_laserscan` into `/lidar/filtered_laserscan` and `/lidar/filtered_laserscan_no_downsample`, then runs RF2O and the odometry deadband/jump wrapper. | LiDAR odometry chain. Direct standalone launch lets the wrapper publish `odom -> base_frame` by default. |
| `yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Includes the LiDAR odometry launch with odom TF disabled, starts motor-validated commanded-stance `/foot_odom` by default, then runs the installed `robot_localization/ekf_node`. Set `launch_foot_odometry:=false` to disable the foot input. | Preferred odometry/localization layer. EKF owns `odom -> base_frame`. |
| `muto_odometry_bag/launch/record_odometry_bag_launch.py` | Records raw LiDAR, raw and processed IMU, commanded gait, `cmd_vel`, endpoint events, build metadata, `/tf_static`, and polled motor-service snapshots through `rosbag2_cpp::Writer`. | Attach to a live hardware pipeline to capture only odometry source data and field-test provenance. |
| `muto_odometry_bag/launch/replay_odometry_bag_launch.py` | Publishes the recorded source topics and `/clock`, recreates `get_motor_angles`, and starts the normal LiDAR/foot/EKF launch. It uses current static sensor TF by default or recorded `/tf_static` on request. | Offline repeatable odometry run through the original nodes; no hardware, mapping, or Nav2. |
| `muto_odometry_bag/launch/replay_odometry_comparison_launch.py` | Replays one source bag through LiDAR-only, leg-only, LiDAR-plus-IMU EKF, and LiDAR-plus-leg-plus-IMU EKF paths concurrently. Only the fully fused EKF publishes odom TF. | Side-by-side odometry comparison under one replay clock. |
| `lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Converts raw depth plus CameraInfo into the narrow, NaN-masked `/camera/filtered_laserscan`. It does not subscribe to LiDAR. | Independent camera preprocessing component. The top-level pipeline includes it when `launch_camera_obstacle_scan:=true`. |
| `muto_slam_mapping/launch/online_async_mapping_launch.py` | Starts SLAM Toolbox online asynchronous mapping. | Mapping-only layer. SLAM uses `/lidar/filtered_laserscan_no_downsample` and does not own camera preprocessing. |
| `muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Starts `controller_server`, `planner_server`, path `smoother_server`, `velocity_smoother`, `behavior_server`, `bt_navigator`, and lifecycle manager. | Requires mapping, TF, EKF, and `/lidar/filtered_laserscan`; camera observations are optional. |
| `muto_slam_mapping/launch/frontier_exploration_launch.py` | Starts the submodule's `frontier_explorer` with the Muto-specific map, costmap, TF, Nav2 action, QoS, and bounded-DP configuration. | Optional autonomous exploration client. Start only after mapping and Nav2 are ready; it is not included by the one-shot Nav2 launch. |
| `muto_command_layer/launch/command_layer_launch.py` | Starts the lower object pipeline, typed object commands, validated natural-language router, and the Muto frontier explorer in cold idle. | Independent command stack. Motion delegates to already-running Nav2 actions. |
| `muto_command_layer/launch/object_pipeline_launch.py` | Starts the SAM2 image annotator, C++ object registry, and VLM socket. | Lower object-identification layer; it does not start command actions or exploration. |
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

camera_depth_to_laserscan_launch.py
  -> /camera/filtered_laserscan (optional Nav2 source)

online_async_mapping_launch.py
  -> slam_toolbox map relation from /lidar/filtered_laserscan_no_downsample

nav2_planner_controller_launch.py
  -> local_costmap and global_costmap
  -> planner/path-smoother/velocity-smoother/behavior/bt_navigator servers
  -> controller /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  -> recovery behaviors -------------------------------> /cmd_vel

command_layer_launch.py                       (independent process group)
  -> object_pipeline_launch.py
     -> YOLO/SAM2 -> instance cloud -> object registry
     -> VLM socket -> /vlm/generate
  -> /find_object and /find_something
  -> /go_to_object through Nav2 /navigate_to_pose
  -> /natural_language_command -> validated typed command dispatch
  -> cold-idle frontier explorer -> /explore -> /navigate_to_pose
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
`/camera/depth/image_raw` plus `/camera/depth/camera_info`. SLAM independently
subscribes to `/lidar/filtered_laserscan_no_downsample` through its parameter file.

Layer-by-layer startup for debugging:

Start hardware:

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
```

The driver consumes `/cmd_vel` as a desired command rather than executing a
whole gait inside the subscription callback. Defaults are a 50 Hz trajectory
phase loop and a 0.5 s command timeout. These are exposed as
`locomotion_update_rate_hz` and `cmd_vel_timeout` by both the hardware and
one-shot launches. Motor feedback never sleeps to settle inside the driver,
because that process also owns the gait and IMU timers.
Startup IMU calibration targets 300 valid samples, permits at most 600 serial
attempts, and has a 30 s wall-clock limit; the same defaults are exposed by the
hardware and one-shot launches.
The top-level pipeline and localization launch also expose
`foot_motor_poll_rate`, which remains at 2 Hz until higher feedback rates are
validated on the controller.

Start static sensor TF:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

Start LiDAR odometry and EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

Start the optional independent camera obstacle source:

```bash
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
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

Start the independent command stack in another terminal:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

The checked-in object-pipeline VLM profile uses a plain-HTTP provider. Keep it
on a trusted network, VPN, or tunnel, or replace it with an HTTPS endpoint.

## Example Sequences

### Full Mapping Plus Nav2

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
ros2 launch tf2_publisher all_tf2_publishers_launch.py
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
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

### Camera Obstacle Scan Test

This requires the depth camera and static camera TF. LiDAR is not an input to the
component launch.

```bash
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
ros2 topic hz /camera/filtered_laserscan
ros2 topic echo /camera/filtered_laserscan --once
```

The output covers only the declared 58.4-degree horizontal camera sector.
Unobserved bins are NaN so Nav2 does not treat angles outside observed depth as
clearing rays. Do not launch a second copy while the top-level pipeline owns the topic.

### Running Without Camera Projection

For one-shot startup, disable the independent source explicitly:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py \
  launch_camera_obstacle_scan:=false
```

For layer-by-layer startup, simply omit the camera component launch.

SLAM Toolbox still subscribes directly to
`/lidar/filtered_laserscan_no_downsample`. Nav2 retains its required LiDAR
observation source.

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
ros2 launch muto_command_layer command_layer_launch.py
```

Expected high-level interfaces include:

- `/sam2/instance_pointcloud`
- `/sam2/stored_objects`
- `/sam2/stored_object_markers`
- `/sam2/get_stored_objects`
- `/vlm/generate`
- `/find_object`
- `/find_something`
- `/go_to_object`
- `/explore`
- `/explore_and_record`

Detection, registry query, visualization, and `/find_object` can be diagnosed
independently of Nav2. `/go_to_object` requires the separate
`/navigate_to_pose` server and the complete `map -> base_frame` TF chain.
`/find_something` first performs the same no-motion registry query, then uses
the existing `/explore_and_record` action until a newly confirmed static object
matches or predicted coverage completes.

### Frontier Exploration

The normal object-command launch already owns a cold-idle explorer. After
mapping and Nav2 are active, start and stop it through the public command:

```bash
ros2 service call /explore std_srvs/srv/SetBool "{data: true}"
ros2 service call /explore std_srvs/srv/SetBool "{data: false}"
```

The stop command enters cold idle and keeps the process reusable. To run the
explorer independently instead, set `launch_frontier_explorer:=false` on the
command launch, then use:

```bash
ros2 launch muto_slam_mapping frontier_exploration_launch.py
frontier_exploration_ctl stop
frontier_exploration_ctl start
frontier_exploration_ctl stop -q
```

The standalone Muto wrapper autostarts exploration. Its `stop -q` option also
terminates the explorer process without stopping the parent Nav2 pipeline.

For command-layer-controlled exploration and object recording, use the
`/explore_and_record` action. It periodically pauses frontier navigation, uses
Nav2 `/spin` for eight 45-degree turns, dwells after every step while the
existing perception pipeline records static objects, checkpoints the registry
after the complete 360-degree scan, and resumes exploration. The action owns
command-layer navigation until it succeeds, aborts, or is canceled.

When frontier exploration reports completion, the action snapshots `/map` and
Nav2's global costmap and visits viewpoints selected by a 2-D line-of-sight
model. Its default `0.98` completion ratio is predicted observable free-space
and occupied-boundary coverage. Successful navigation and spin steps receive
the model's predicted visibility credit; RGB, depth, detector, and registry
results are not coverage inputs. Treat this as a geometric mission-progress
estimate, not measured camera coverage or proof that every object was seen.

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

Check SLAM and costmap scans:

```bash
ros2 topic hz /lidar/filtered_laserscan_no_downsample
ros2 topic echo /lidar/filtered_laserscan_no_downsample --once
ros2 topic hz /lidar/filtered_laserscan
ros2 topic hz /camera/filtered_laserscan
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
