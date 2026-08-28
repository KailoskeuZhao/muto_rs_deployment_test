# Launch Reference

Date: 2026-08-06

This document summarizes the launch files that matter for the current Muto RS
workspace. It separates the normal robot sequence from experimental and
component launches.

For the default v2 mission/Nav2 monitoring layers, optional deep Nav2
and odometry captures, shared retention policy, manual notes, and replay safety,
see [Default Bags And Mission Monitoring](bags.md).

## Removed Packages

`src/Simple-2D-LiDAR-Odometry` and `src/simple_vlm` were removed from the
active workspace. The current launch files do not include them, and no active
package declares them as a dependency.

## Launch File Summary

`robot_localization` is an external ROS 2 dependency. The workspace launch
files use the installed Humble or Jazzy `ekf_node`; this workspace does not keep
`robot_localization` under `src/`.

| Launch file | What it starts | Usual role |
| --- | --- | --- |
| `muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Includes hardware, static sensor TF, LiDAR odometry/EKF, online async mapping, camera depth projection, Nav2 servers, and the compact Nav2 session recorder after readiness succeeds. | One-shot full robot Nav2 pipeline. Use `launch_nav2_bag:=false` only when recording is intentionally disabled. |
| `yahboomcar_bringup/launch/muto_hardware_launch.py` | `lidar_tg30/lidar_node`, Orbbec `astra_pro_plus.launch.py`, and the fixed-rate `yahboomcar_bringup/muto_driver` locomotion loop. The driver also publishes raw/processed IMU, diagnostic controller-fused `0x60` attitude, and cumulative IMU scheduler status. | Hardware source layer. Run first on the robot. The initial driver tick commands standby, so support the robot and keep its legs clear. |
| `tf2_publisher/launch/all_tf2_publishers_launch.py` | Static TF publishers for `base_frame -> camera_link`, `base_frame -> lidar_frame`, and `base_frame -> imu_link`. Optional odom TF publisher is off by default. | Sensor TF layer. Needed before scan conversion, RF2O, mapping, and Nav2. |
| `lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | Default path filters `/lidar/raw_laserscan` into `/lidar/filtered_laserscan` and `/lidar/filtered_laserscan_no_downsample`, then runs RF2O and the odometry deadband/jump wrapper. | LiDAR odometry chain. Direct standalone launch lets the wrapper publish `odom -> base_frame` by default. |
| `yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Includes LiDAR odometry, the default stop-only controller-yaw gate, and `robot_localization/ekf_node`. Measured-joint `/foot_odom` remains default-off. | Preferred odometry/localization layer. EKF owns `odom -> base_frame`. |
| `muto_odometry_bag/launch/record_odometry_bag_launch.py` | Records raw LiDAR, raw/processed IMU, controller attitude, telemetry and yaw-gate status, motion/gait state, `cmd_vel`, endpoint events, build metadata, `/tf_static`, and optional motor snapshots. | Attach to a live hardware pipeline to capture odometry source data and field-test provenance. |
| `muto_odometry_bag/launch/replay_odometry_bag_launch.py` | Publishes recorded source topics and `/clock`, recreates `get_motor_angles`, and starts the normal LiDAR/optional-foot/EKF launch. Stop-only controller yaw is enabled by default; recorded static TF can be pre-published before the first scan. | Offline repeatable odometry run through the original nodes; no hardware, mapping, or Nav2. |
| `muto_odometry_bag/launch/replay_odometry_comparison_launch.py` | Replays one source bag through LiDAR-only, raw-gyro, raw-RF2O, and optional relative-controller-yaw EKF branches concurrently; foot input is independently optional. Only `/odometry/filtered` publishes odom TF. | Side-by-side odometry comparison under one replay clock. Use 2x or slower for quantitative work. |
| `muto_command_layer_v2/high_level_recorder_node.py` | Opens one mission-scoped high-level MCAP with board, decisions, registry evidence, POI-grid result, lifecycle, and terminal outcome. | Automatic with the v2 mission launch; raw sensor capture remains opt-in through the dedicated diagnostic bags. |
| `muto_nav2_bag/launch/record_nav2_bag_launch.py` | Opens a navigation-only MCAP with TF, maps, compact pose/scan evidence, goals, paths, commands, action state, diagnostics, and config snapshots. | Included by the normal pipeline. It can also be launched separately; select `nav2_bag_full.yaml` for a short costmap/action-feedback deep dive. |
| `lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Converts raw depth plus CameraInfo into the narrow, NaN-masked `/camera/filtered_laserscan`. It does not subscribe to LiDAR. | Independent camera preprocessing component. The top-level pipeline includes it when `launch_camera_obstacle_scan:=true`. |
| `muto_slam_mapping/launch/online_async_mapping_launch.py` | Starts SLAM Toolbox online asynchronous mapping. | Mapping-only layer. SLAM uses `/lidar/filtered_laserscan_no_downsample` and does not own camera preprocessing. |
| `muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Starts `controller_server`, `planner_server`, path `smoother_server`, `velocity_smoother`, `behavior_server`, `bt_navigator`, and lifecycle manager. | Requires mapping, TF, EKF, and `/lidar/filtered_laserscan`; camera observations are optional. |
| `muto_command_layer_v2/launch/v2_hardware_smoke_launch.py` | Starts the v2 field composition: the production pipeline, registry, deterministic POI-grid/Nav2 authorities, mission executive, and high-level MCAP recorder. | Use for a supervised Humble hardware smoke. |
| `muto_command_layer_v2/launch/v2_command_layer_launch.py` | Starts only the v2 executive and high-level recorder. | Use when the independent authorities are already running. |
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
  -> recovery behaviors --> /cmd_vel_nav -> velocity_smoother -> /cmd_vel

muto_command_layer_v2 (independent process group)
  -> YOLO/SAM2 -> instance cloud -> object registry
  -> VLM socket -> /vlm/generate
  -> /muto/mission -> MissionExecutive -> CommanderAgent
       -> registry/POI-grid/Nav2 authorities -> typed bounded tools
  -> high-level mission recorder -> MCAP
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
object registry, the VLM bridge, the v2 mission executive, or POI-grid search.
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
one-shot launches. `locomotion_command_mapping:=geometric` derives the
requested gait amplitude directly from the custom trajectory geometry and
cadence. `calibrated` optionally loads `locomotion_calibration_file`, while
`legacy_100` is explicit rollback only. Every mode publishes the requested and
projected twist plus selected float amplitudes on `/muto/motion_command_state`;
rounded level fields remain as compatibility diagnostics.
Nonzero amplitude changes wait for the next complete gait
boundary; selected and active amplitudes plus pending state are carried
separately by `/muto/motion_command_state`. Stop and timeout directly command nominal
stance and are abrupt rather than a smooth deceleration. Motor feedback never
sleeps to settle inside the driver, because that process also owns the
gait-slotted telemetry scheduler. Each gait phase normally batches six vendor
leg frames into one serial write; `batch_gait_phase_writes:=false` is the
hardware rollback.

After startup, the host polls the controller's cached raw IMU packet at 10 Hz
and suppresses consecutive identical accel/gyro values. Although the cache
changes at only about 1.033 Hz, retaining the previous host rate keeps its
transition timestamp uncertainty comparable during the `0x60` experiment.
Startup calibration is independent: it targets ten
changed snapshots, permits 150 serial attempts, and has a 15 s wall-clock
limit. See [`imu_serial_pipeline_2026-08-07.md`](imu_serial_pipeline_2026-08-07.md)
for the controller-rate probe and firmware finding.

The same 50 Hz control callback requests the vendor-fused `0x60` attitude at a
10 Hz host rate and publishes every valid response on
`/imu/controller_attitude`. The higher request rate avoids aliasing the
roughly 5 Hz controller producer; it does not make the controller compute at
10 Hz. Each control slot sends the gait phase first and then services at most
one due telemetry endpoint. Raw reads are phased between attitude reads so
they no longer compete as aligned timers. Both paths retain the gait-deadline
guard. Cumulative scheduling and read counters are published on
`/muto/imu_telemetry_status` for bag analysis. The second marked bag confirmed
10.01 Hz successful polling throughout active gait and about 5.04 Hz changed
snapshots. Set `imu_attitude_publish_rate_hz:=0.0` to disable the path.

Production localization now replaces the sparse raw yaw rate with guarded
`0x60` relative heading. The adapter observes all changed 5 Hz samples but
publishes only a startup anchor and one circular-mean correction after each
two-second stationary dwell. Both selected and active locomotion states must
be standby, the motion-state sample must be fresh, and the one-second attitude
window must span no more than one degree. Accepted corrections use `(4 deg)^2`
variance; `/muto/controller_attitude_yaw_status` exposes every gate decision.
RF2O remains the only moving-yaw source. Set
`fuse_controller_attitude_yaw:=false` for the raw-gyro rollback.

Foot odometry is disabled by default. The top-level pipeline and localization
launch retain `foot_motor_poll_rate:=2.0` for explicit diagnostic use. The
2026-08-07 bag correlated 651 of 652 motor reads with gait intervals over
30 ms, so `launch_foot_odometry:=true` is not a normal-deployment setting with
the current blocking service.

Start static sensor TF:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

Start LiDAR odometry and EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

Raw-gyro rollback:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py \
  fuse_controller_attitude_yaw:=false
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

Start the v2 command layer only after Nav2 readiness has been observed:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer_v2 v2_hardware_smoke_launch.py
```

Start the independent command stack in another terminal:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer_v2 v2_hardware_smoke_launch.py
```

The checked-in object-pipeline VLM profile uses a plain-HTTP provider. Keep it
on a trusted network, VPN, or tunnel, or replace it with an HTTPS endpoint.
`/muto/mission` sends bounded planning requests through this connection; the
executive supplies the current board and one fresh camera snapshot at each
decision boundary.

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

The controller, behavior server, and v2 direct-rotate primitive
publish to `/cmd_vel_nav`; the velocity smoother publishes the final
`/cmd_vel` sent to the Muto driver. This keeps one Humble-compatible hard
velocity/acceleration limiter before hardware.

The navigation behavior trees select Humble's Savitzky-Golay path smoother for
small NavFn artifacts and retain `SimpleSmoother` as a configured rollback.
RPP uses a fixed `0.40 m` lookahead, beyond the approximately `0.26 m` robot
radius, to reduce left/right turn-direction flipping when a new path starts
behind the robot.

### Object Perception And Commands

Run the v2 field composition beside the normal operator console:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer_v2 v2_hardware_smoke_launch.py
```

The v2 surface is one typed `/muto/mission` action. Natural-language input is
normalized into a `Mission` goal, the `CommanderAgent` selects one of the two
v2 skills from the mission board, and deterministic registry/POI-grid/Nav2
authorities execute the bounded tools. The mission-scoped MCAP recorder
captures the board, decisions, rejections, registry evidence, POI result, and
terminal result.

```bash
ros2 action send_goal /muto/mission \
  muto_command_layer_v2/action/Mission \
  "{request_id: 'field-001', objective: 'find the red mug beside the kettle', object_request: 'red mug', completion_policy: 'approach_confirmed', schema_version: 'muto_command_layer_v2'}" \
  --feedback
```

Do not send the retired v1 actions or direct competing search/navigation
goals during a mission. Nav2 remains the final navigation and obstacle-
avoidance authority; the commander only requests bounded typed tools.

### POI-grid search

The v2 executive owns the search boundary. The POI-grid authority samples only
known-free, footprint-safe cells in the current connected map component, emits
one typed result on `/muto/poi_grid/result`, and hands the selected pose to
Nav2. Commander receives control only after the Nav2 result, exhaustion,
staleness, no-reachable-goal, or cancellation outcome is recorded. There is no
independent search process or manual search service. Inspect the
selected pose and typed result directly:

```bash
ros2 topic echo /muto/poi_grid/selected_pose geometry_msgs/msg/PoseStamped
ros2 topic echo /muto/poi_grid/result muto_command_layer_v2/msg/PoiGridResult
```

The high-level recorder writes the board/events and POI-grid diagnostics to one
unique MCAP per mission. It does not record raw camera, LiDAR, IMU, or
point-cloud streams.

### Nav2 Diagnostic Bag

The normal Nav2 pipeline starts the compact recorder by default. To run the
recorder separately against an already-running Nav2 graph:

```bash
ros2 launch muto_nav2_bag record_nav2_bag_launch.py
```

It writes under `/opt/muto_rs_ws/bags/muto_nav2_<timestamp>_<session-id>` and
continues until `Ctrl-C` or:

```bash
ros2 service call /muto/nav2_bag/stop std_srvs/srv/Trigger "{}"
```

Read `/muto/nav2_bag/path` with transient-local durability for the exact bag
directory. Add a timestamped field note with a short string on
`/muto/nav2_bag/event`. The directory includes a manifest and snapshots of the
Nav2 and SLAM parameter files plus both Muto behavior trees used by the
recorder launch.

The Humble deployment cannot capture ordinary Nav2 action goal/result service
payloads unless service introspection is enabled. The compact default retains
plans, action status, RViz goals, POI/object goal mirrors, and operator
events; annotate a goal sent directly through an action client when its target
matters. Use `nav2_bag_full.yaml` for short captures that also require action
feedback and complete costmaps.

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
