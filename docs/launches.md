# Launch Reference

Date: 2026-08-06

This document summarizes the launch files that matter for the current Muto RS
workspace. It separates the normal robot sequence from experimental and
component launches.

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
| `muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Includes hardware, static sensor TF, LiDAR odometry/EKF, online async mapping, camera depth projection, and Nav2 planner/controller/action servers with minimum delays and topic/TF readiness gates. | One-shot full robot Nav2 pipeline. Use this for normal bringup once the robot dependencies are installed. |
| `yahboomcar_bringup/launch/muto_hardware_launch.py` | `lidar_tg30/lidar_node`, Orbbec `astra_pro_plus.launch.py`, and the fixed-rate `yahboomcar_bringup/muto_driver` locomotion loop. The driver also publishes raw/processed IMU, diagnostic controller-fused `0x60` attitude, and cumulative IMU scheduler status. | Hardware source layer. Run first on the robot. The initial driver tick commands standby, so support the robot and keep its legs clear. |
| `tf2_publisher/launch/all_tf2_publishers_launch.py` | Static TF publishers for `base_frame -> camera_link`, `base_frame -> lidar_frame`, and `base_frame -> imu_link`. Optional odom TF publisher is off by default. | Sensor TF layer. Needed before scan conversion, RF2O, mapping, and Nav2. |
| `lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | Default path filters `/lidar/raw_laserscan` into `/lidar/filtered_laserscan` and `/lidar/filtered_laserscan_no_downsample`, then runs RF2O and the odometry deadband/jump wrapper. | LiDAR odometry chain. Direct standalone launch lets the wrapper publish `odom -> base_frame` by default. |
| `yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Includes LiDAR odometry, the default stop-only controller-yaw gate, and `robot_localization/ekf_node`. Measured-joint `/foot_odom` remains default-off. | Preferred odometry/localization layer. EKF owns `odom -> base_frame`. |
| `muto_odometry_bag/launch/record_odometry_bag_launch.py` | Records raw LiDAR, raw/processed IMU, controller attitude, telemetry and yaw-gate status, motion/gait state, `cmd_vel`, endpoint events, build metadata, `/tf_static`, and optional motor snapshots. | Attach to a live hardware pipeline to capture odometry source data and field-test provenance. |
| `muto_odometry_bag/launch/replay_odometry_bag_launch.py` | Publishes recorded source topics and `/clock`, recreates `get_motor_angles`, and starts the normal LiDAR/optional-foot/EKF launch. Stop-only controller yaw is enabled by default; recorded static TF can be pre-published before the first scan. | Offline repeatable odometry run through the original nodes; no hardware, mapping, or Nav2. |
| `muto_odometry_bag/launch/replay_odometry_comparison_launch.py` | Replays one source bag through LiDAR-only, raw-gyro, raw-RF2O, and optional relative-controller-yaw EKF branches concurrently; foot input is independently optional. Only `/odometry/filtered` publishes odom TF. | Side-by-side odometry comparison under one replay clock. Use 2x or slower for quantitative work. |
| `muto_exploration_bag/launch/record_exploration_bag_launch.py` | Arms a standalone compact MCAP recorder that retains navigation, odometry, structured object results, action traffic, and operator events while excluding bulky image and point-cloud payloads. | Automatic with the command launch, or launch separately before an exploration action. |
| `muto_nav2_bag/launch/record_nav2_bag_launch.py` | Immediately opens a manually controlled, navigation-only MCAP with TF, maps, odometry, scans, goals, paths, costmaps, commands, Nav2 action progress, and diagnostic context. | Attach to any live Nav2 run; stop with its service or `Ctrl-C`. It is independent of the mission recorder. |
| `lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Converts raw depth plus CameraInfo into the narrow, NaN-masked `/camera/filtered_laserscan`. It does not subscribe to LiDAR. | Independent camera preprocessing component. The top-level pipeline includes it when `launch_camera_obstacle_scan:=true`. |
| `muto_slam_mapping/launch/online_async_mapping_launch.py` | Starts SLAM Toolbox online asynchronous mapping. | Mapping-only layer. SLAM uses `/lidar/filtered_laserscan_no_downsample` and does not own camera preprocessing. |
| `muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Starts `controller_server`, `planner_server`, path `smoother_server`, `velocity_smoother`, `behavior_server`, `bt_navigator`, and lifecycle manager. | Requires mapping, TF, EKF, and `/lidar/filtered_laserscan`; camera observations are optional. |
| `muto_slam_mapping/launch/frontier_exploration_launch.py` | Starts the submodule's `frontier_explorer` with the Muto-specific map, costmap, TF, Nav2 action, QoS, and bounded-DP configuration. | Optional autonomous exploration client. Start only after mapping and Nav2 are ready; it is not included by the one-shot Nav2 launch. |
| `muto_command_layer/launch/command_layer_launch.py` | Starts the lower object pipeline, typed object commands, persistent event-driven model commander, validated natural-language router, standalone exploration recorder, and the Muto frontier explorer in cold idle. | Independent command stack. Motion delegates to already-running Nav2 actions. |
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
  -> recovery behaviors --> /cmd_vel_nav -> velocity_smoother -> /cmd_vel

command_layer_launch.py                       (independent process group)
  -> object_pipeline_launch.py
     -> YOLO/SAM2 -> instance cloud -> object registry
     -> VLM socket -> /vlm/generate
  -> /find_object and deterministic /find_something
  -> model commander -> /look_for_object
       -> check registry, schedule one bounded command, monitor, replan
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
`/look_for_object` sends fresh full-camera JPEGs through this connection for
each model scheduling decision and for slow periodic inspections while a
bounded exploration or wait command is active.

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

The controller, behavior server, and command-layer direct-rotate primitive
publish to `/cmd_vel_nav`; the velocity smoother publishes the final
`/cmd_vel` sent to the Muto driver. This keeps one Humble-compatible hard
velocity/acceleration limiter before hardware.

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
- `/look_for_object`
- `/model_commander/status`
- `/go_to_object`
- `/explore`
- `/explore_and_record`

Detection, registry query, visualization, and `/find_object` can be diagnosed
independently of Nav2. `/go_to_object` requires the separate
`/navigate_to_pose` server and the complete `map -> base_frame` TF chain.
`/find_something` first performs the same no-motion registry query, then uses
the existing `/explore_and_record` action until a newly confirmed static object
matches or predicted coverage completes.

`/look_for_object` is the persistent highest-level alternative. Its resident
commander first checks `/find_object`, then asks the model at scheduling points
to inspect a newly received bounded RGB frame plus mission state and choose a
registry recheck, a bounded `/explore_and_record` program, a bounded wait, or a
valid no-match finish. During a long exploration or wait, a fresh snapshot is
inspected after a 20-second cooldown by default; inference latency is additional.
That restricted monitor can only leave
the owned command running or request that local code stop it, confirm the stop,
check the registry, and replan. Changed confirmed-object identities interrupt
stale decisions or child work. The raw camera subscription is snapshot-scoped;
frames do not invoke the model at camera rate. The model cannot name ROS
interfaces or declare that an object was found; visual evidence remains
advisory until `/find_object` confirms a registry match. Possible or likely
evidence forces that check before further motion. Nav2 remains responsible for
real-time collision handling. A single periodic frame is not a batch of all six
scan headings.

```bash
ros2 action send_goal /look_for_object \
  muto_command_layer/action/LookForObject \
  "{prompt: 'the red mug beside the kettle', max_duration: 0.0, max_planning_steps: 0}" \
  --feedback
```

A zero duration or planning-step value selects the finite configured defaults
of 1800 seconds and 64 decisions. `/find_something` remains the deterministic
rollback command. The stack does not yet enforce a graph-wide motion lease, so
do not send competing direct Nav2 or exploration goals during either mission.

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
The wrapper disables visibility-gain goal preemption so SLAM refreshes cannot
repeatedly cancel and resend the same active Nav2 goal. Blocked-goal skipping
and the independent `0.25 m` close-enough completion guard remain enabled.

For command-layer-controlled exploration and object recording, use the
`/explore_and_record` action. It periodically pauses frontier navigation, uses
Nav2 `/spin` for six 60-degree turns, waits for three fresh detector results at
each heading (with a three-second timeout), checkpoints the registry after the
complete 360-degree scan, and resumes exploration. A cycle's ten-second
exploration interval is a minimum: an active frontier trip is allowed to finish
before the scan begins. The action owns
command-layer navigation until it succeeds, aborts, or is canceled.

The launch starts the standalone `muto_exploration_bag` recorder by default.
It opens one MCAP per mission under `/opt/muto_rs_ws/bags`, retains the
navigation, odometry, structured object-result, log, lifecycle, and hidden
action graph, and finalizes on success, cancellation, or abort. Raw camera
images, derived SAM2 images, and point clouds are excluded by default. Read the
exact transient-local path from
`/explore_and_record/last_bag_path`. Publish a short manual note on
`/explore_and_record/operator_event` while the action is active. Clear
`exploration_bag_exclude_regex` only for a dedicated raw-perception capture, or
use `exploration_bag_topics_regex` for a narrower recording contract.

When frontier exploration reports completion, the action snapshots `/map` and
Nav2's global costmap and queries a 2-D line-of-sight calculator for current
coverage and ranked observation points of interest. The legacy action visits
the top-ranked points; the same read-only report is available at
`/command_layer/visibility_coverage`. Its default `0.98` completion ratio is
predicted observable free-space and occupied-boundary coverage. Successful
navigation and spin steps receive the model's predicted visibility credit;
RGB, depth, detector, and registry results are not coverage inputs. Treat this
as a geometric mission-progress estimate, not measured camera coverage or
proof that every object was seen.

### Standalone Nav2 Diagnostic Bag

To inspect navigation without recording perception imagery or the full ROS
graph, start this in a separate terminal after the Nav2 pipeline:

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
Nav2, frontier, and SLAM parameter files plus both Muto behavior trees used by
the recorder launch.

The Humble deployment cannot capture ordinary Nav2 action goal/result service
payloads unless service introspection is enabled. Plans, feedback/status, RViz
goals, frontier/object goal mirrors, and operator events are retained; annotate
a goal sent directly through an action client when its target matters.

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
