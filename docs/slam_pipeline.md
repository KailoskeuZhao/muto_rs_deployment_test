# SLAM and Nav2 Pipeline

Date: 2026-08-05

This runbook describes the active ROS 2 Humble mapping and navigation pipeline
for the Muto RS deployment on aarch64. It is derived from the current launch
files and configuration, not from the removed PointCloud-based experiments.

The normal entry point is:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

Use the component launches later in this document only to isolate one layer
during debugging.

## Active Architecture

```text
TG30 LiDAR
  -> /lidar/raw_laserscan
  -> lidar_laserscan_filter_node
       -> /lidar/filtered_laserscan
       |    -> RF2O -> /scan_odom_raw -> odometry guard -> /scan_odom
       |    -> robot_localization EKF <- /imu/data_processed
       |    -> /odometry/filtered + odom -> base_frame
       |    -> Nav2 local and global obstacle layers (lidar source)
       |
       -> /lidar/filtered_laserscan_no_downsample
            -> SLAM Toolbox -> /map + map -> odom

depth image + depth CameraInfo
  -> camera_depth_to_laserscan_node
  -> /camera/filtered_laserscan
  -> Nav2 local and global obstacle layers (camera source)

Nav2 planner/path smoother/controller
  -> /cmd_vel_nav -> velocity smoother -> /cmd_vel
  -> Muto desired command latch -> 50 Hz one-phase gait loop
  -> /muto/commanded_gait_state -> /foot_odom -> EKF planar velocity overlay
```

The LiDAR and camera enter Nav2 as separate standard obstacle-layer observation
sources. They are not alternated or merged into a synthetic `LaserScan`. SLAM
uses only the stable full-resolution LiDAR stream. Camera loss therefore does not
change SLAM scan geometry and does not block navigation startup.

Static sensor transforms provide:

```text
base_frame -> lidar_frame
base_frame -> imu_link
base_frame -> camera_link -> camera optical frames
```

The complete runtime TF chain is:

```text
map -> odom -> base_frame -> sensor frames
```

## TF Ownership

Only one node may publish each dynamic transform.

| Transform | Normal owner | Notes |
| --- | --- | --- |
| `map -> odom` | SLAM Toolbox | Relates the map to locally continuous odometry while mapping. |
| `odom -> base_frame` | `robot_localization/ekf_node` | Authoritative local robot pose in the normal pipeline. |
| `base_frame -> lidar_frame` | `tf2_publisher` | Static sensor mount. |
| `base_frame -> imu_link` | `tf2_publisher` | Static sensor mount. |
| `base_frame -> camera_link` | `tf2_publisher` | Static camera-body mount. |
| Camera internal frames | Orbbec driver | Optical and depth-frame relationships. |

The normal EKF launch forces RF2O and the odometry wrapper not to publish TF.
The optional `tf2_publisher/odom_publisher` is also disabled. Enabling either
while the EKF runs creates competing `odom -> base_frame` publishers.

## Build

On the Humble target:

```bash
cd /opt/muto_rs_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

External ROS dependencies, including `robot_localization`, `slam_toolbox`,
and Nav2, must be installed in the sourced Humble environment. They are not
vendored into this workspace.

## Normal Startup

Start the complete stack:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

This entry point owns hardware, TF, localization, mapping, and Nav2 only. The
GPU perception/object command pipeline remains an independent launch. By
default that command launch owns a frontier explorer process in cold idle.

The launch combines minimum delays with observable readiness checks. A delay
does not declare a stage ready; it only determines when that stage begins
checking.

| Stage | Minimum delay | Readiness timeout | Required state before launch |
| --- | ---: | ---: | --- |
| Static sensor TF | 1 s | n/a | Timer only. |
| Localization | 3 s | 120 s | `/lidar/raw_laserscan`, `base_frame <- lidar_frame`, and `base_frame <- imu_link`. |
| Mapping | 8 s | 90 s | `/odometry/filtered`, `/lidar/filtered_laserscan_no_downsample`, and `odom <- base_frame`. |
| Nav2 | 12 s | 120 s | `/map`, `/lidar/filtered_laserscan`, and `map <- base_frame`. |

A failed readiness gate shuts down the pipeline instead of launching its
downstream stage against incomplete topics or TF. This matters on aarch64,
where driver, camera, and mapping startup time can vary.

Useful top-level switches include:

| Argument | Default | Effect |
| --- | --- | --- |
| `launch_hardware` | `true` | Starts the TG30, Orbbec launch, and Muto driver. |
| `launch_sensor_tf` | `true` | Starts static sensor mounts. |
| `launch_localization` | `true` | Starts LiDAR filtering, RF2O, and EKF. |
| `launch_mapping` | `true` | Starts SLAM Toolbox only. |
| `launch_nav2` | `true` | Starts the current Nav2 server set. |
| `launch_camera_obstacle_scan` | `true` | Starts the independent camera costmap source in the top-level pipeline. |
| `camera_scan_max_publish_rate` | `7.0` | Caps camera depth-to-scan processing. |
| `locomotion_update_rate_hz` | `50.0` | Advances one custom-library gait phase per driver tick. |
| `batch_gait_phase_writes` | `true` | Sends the six unchanged leg frames in one contiguous write; `false` restores per-leg pacing. |
| `cmd_vel_timeout` | `0.5` | Returns the gait to standby when velocity commands stop. |
| `locomotion_command_mapping` | `geometric` | Derives gait amplitudes from the custom exact-SE(2) trajectory; `calibrated` loads a measured profile and `legacy_100` is rollback-only. |
| `locomotion_calibration_file` | `muto_locomotion_provisional_20260806.yaml` | Optional profile used only when `locomotion_command_mapping:=calibrated`. |
| `imu_publish_rate_hz` | `10.0` | Runtime host poll rate for the roughly 1.033 Hz controller-cached raw snapshot; retaining 10 Hz preserves its prior transition-time observation window for the `0x60` comparison. |
| `imu_attitude_publish_rate_hz` | `10.0` | Host poll rate for fused `0x60` attitude; the gait-slotted scheduler observes roughly 5 Hz changed snapshots without starving during motion. `0.0` disables it. |
| `imu_suppress_identical_snapshots` | `true` | Avoids assigning repeated accel/gyro values new ROS timestamps. |
| `imu_response_timeout_sec` | `0.008` | Runtime serial budget; polls too close to a gait deadline are skipped. |
| `imu_calibration_sample_count` | `10` | Changed stationary accel/gyro snapshots targeted at startup. |
| `imu_calibration_max_reads` | `150` | Maximum serial attempts allowed for that calibration. |
| `imu_calibration_timeout_sec` | `15.0` | Hard wall-clock cap on startup calibration. |
| `launch_foot_odometry` | `false` | Measured-joint foot odometry is diagnostic-only while reads block gait dispatch. |
| `foot_motor_poll_rate` | `2.0` | Conservative rate when foot diagnostics are explicitly enabled. |
| `allow_experimental_high_rate_motor_polling` | `false` | Required explicit opt-in above 2 Hz; never enable for normal deployment with the current blocking serial service. |
| `fuse_controller_attitude_yaw` | `true` | Replace the sparse raw gyro with a startup anchor and stable stop-only relative `0x60` yaw corrections. Set `false` for rollback. |
| `controller_attitude_yaw_variance` | `0.0048738787` | `(4 deg)^2` variance for accepted stop corrections. |
| `controller_attitude_stationary_settle_sec` | `2.0` | Required strict-standby dwell before correction. |
| `controller_attitude_republish_interval_sec` | `0.0` | One averaged correction per stationary episode. |

If a prerequisite stage is disabled, any enabled downstream stage must already
have equivalent topics and TF supplied externally.

For a complete deployment with object commands, run the command stack in a
second terminal:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

The command-layer launch starts SAM2/YOLO, the object registry, and the VLM
socket as its lower layer, together with `/find_object`,
`/find_something`, `/go_to_object`, `/explore`,
`/explore_and_record`, and a cold-idle frontier explorer. `/find_something`
composes registry lookup with the existing autonomous mission and cancels that
mission when a newly confirmed static object matches.
After frontier exhaustion, `/explore_and_record` may continue through
costmap-reachable viewpoints until its 2-D model reaches the configured
predicted observable free-space and boundary ratio. This estimate is based on
occupancy-grid line of sight after successful Nav2 scans; camera frames, depth,
detections, and registry growth do not gate completion.
After Nav2 is active, call `/explore` with `data: true` to start autonomous map
exploration and `data: false` to stop it. Do not run exploration concurrently
with another navigation command client unless preemption is intentional.

## Layer-By-Layer Debug Startup

Use separate terminals, sourcing the Humble installation and workspace in each.

Hardware:

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
```

Static sensor TF:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

LiDAR odometry and EKF:

```bash
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
```

Optional independent camera obstacle scan:

```bash
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
```

Mapping:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

Nav2 planner, controller, path smoother, velocity smoother, behavior, and
navigator servers:

```bash
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

Optional frontier exploration after Nav2 is ready:

```bash
ros2 launch muto_slam_mapping frontier_exploration_launch.py
```

The Muto frontier profile keeps an accepted Nav2 goal stable while the robot is
driving. Visibility-gain goal preemption is disabled because live SLAM refreshes
were canceling and redispatching effectively identical goals. A frontier can
still be skipped when it becomes blocked, and arrival within `0.25 m` still
counts as complete when Nav2 has not yet reported success.

`online_async_mapping_launch.py` owns only SLAM Toolbox. The top-level pipeline
owns camera preprocessing independently when `launch_camera_obstacle_scan:=true`.
For layer-by-layer startup, launch the camera component once when depth obstacles
are wanted, or omit it without changing the mapping command.

## Hardware And Sensor Frames

`muto_hardware_launch.py` starts:

- `lidar_tg30/lidar_node`
- `orbbec_camera/astra_pro_plus.launch.py`
- `yahboomcar_bringup/muto_driver`

The important live sensor inputs are:

| Topic | Expected type/frame |
| --- | --- |
| `/lidar/raw_laserscan` | `sensor_msgs/msg/LaserScan`, normally `lidar_frame`. |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image`, encoding `16UC1`. |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` matching the depth profile. |
| `/imu/data_processed` | `sensor_msgs/msg/Imu`, frame `imu_link`; yaw rate is retained for rollback and `imu_only` tests. |
| `/imu/controller_attitude` | `muto_hexapod_interfaces_custom/msg/ControllerAttitude`; host-stamped vendor Euler degrees with frame intentionally unset. |
| `/imu/controller_attitude_imu` | Sparse default yaw-only `sensor_msgs/msg/Imu` in `imu_link`; startup anchor plus one stable correction per stop. |
| `/muto/imu_telemetry_status` | `std_msgs/msg/String`; one-second cumulative raw/attitude scheduler, attempt, success, failure, duplicate, deferral, and gait-deadline-skip counters. |
| `/muto/controller_attitude_yaw_status` | `std_msgs/msg/String`; one-second gate mode, acceptance/rejection counters, and startup-anchor state. |

Current Astra Pro Plus launch defaults request color at `640x480 @ 30 Hz` and
depth at `320x240 @ 30 Hz`. The downstream depth-to-scan and SAM2 projection
branches each cap their own processing at 7 Hz, so the supported 30 Hz hardware
profile does not imply 30 heavy projections per second. Orbbec point-cloud and
IR publication are disabled; current consumers use raw depth directly. The
serial-specific color calibration fallback is
`yahboomcar_bringup/config/astra_pro_plus_acrf35300kr_color_640x480.yaml`.

The Orbbec launch must publish the internal transform from `camera_link` to
the depth optical frame. The local TF package owns only the camera-body mount.

## LiDAR Filtering And Localization

The active LiDAR path is entirely `LaserScan` based:

| Stage | Input | Output | Important defaults |
| --- | --- | --- | --- |
| Scan filter | `/lidar/raw_laserscan` | `/lidar/filtered_laserscan` | Range `0.05-10.0 m`, downsample factor `2`. |
| Full-resolution filter output | Same raw scan | `/lidar/filtered_laserscan_no_downsample` | Range up to `15.0 m`; used directly by SLAM Toolbox. |
| RF2O | `/lidar/filtered_laserscan` | `/scan_odom_raw` | `16 Hz`, frames `odom` and `base_frame`, internal TF output disabled. |
| Deadband/jump wrapper | `/scan_odom_raw` | `/scan_odom` | Suppresses stationary drift and implausible jumps. |
| EKF | `/scan_odom` plus one selected IMU input | `/odometry/filtered` and `odom -> base_frame` | `30 Hz`, 2D mode; RF2O while moving plus sparse stop-only relative `0x60` yaw checks. |

The EKF fuses RF2O planar position and yaw. The default yaw adapter observes
all changed controller samples but publishes only a startup reference and one
stable circular-mean correction after each two-second stop. Selected and
active locomotion state must both be standby, and it publishes nothing while
moving. RF2O therefore remains the moving-yaw source. The raw controller topic
is never fused directly. Set `fuse_controller_attitude_yaw:=false` to restore
the sparse raw-yaw-rate branch.

The optional, default-off foot input is generated from a fixed-rate locomotion state stream.
`/cmd_vel` changes the desired command; the driver advances one trajectory phase
per 50 Hz tick and publishes the phase after sending its motor targets. A newer
nonzero command waits for the next complete gait boundary. The accompanying
motion-command state reports selected and active float amplitudes separately,
keeps rounded legacy level diagnostics, and states whether a replacement is
pending.
A stale or zero command returns directly to nominal stance instead of performing
a smooth deceleration. Motor service reads use the gait target current during
that serial read and add no artificial settling delay. The read still holds the
shared serial bus, so slow responses can delay the gait-slotted telemetry loop.
The 2026-08-07 bag showed this for 651 of 652 motor-overlapping gait intervals;
leave `launch_foot_odometry:=false` in normal deployment.

Raw `0x61` and fused-attitude `0x60` no longer use independent ROS timers. The
50 Hz locomotion callback sends its gait phase first, selects at most one due
telemetry endpoint, and applies the existing response-budget guard. The
defaults phase 10 Hz raw polling between 10 Hz attitude polling. A transaction
that begins advances its endpoint deadline even if the controller returns no
valid packet; a pre-I/O deadline skip remains due for the next control slot.

The deadband wrapper applies its translation and yaw guards per axis when
recent `cmd_vel` indicates that axis is stationary. Standalone
`filter_lidar_odometry_launch.py` lets the wrapper publish
`odom -> base_frame` by default for testing. When the EKF is running, keep
`rf2o_publish_tf:=false`.

## Depth-To-Scan And Nav2 Observations

The active camera path consumes the raw depth image directly. It does not use
`/camera/depth/points`.

```text
/camera/depth/image_raw + /camera/depth/camera_info
  -> sampled depth back-projection
  -> reject rays outside H 58.4 degrees and V 45.5 degrees
  -> TF into base_frame
  -> z/range filtering
  -> /camera/filtered_laserscan
  -> Nav2 obstacle layers as source `camera`
```

Current defaults:

| Setting | Default | Meaning |
| --- | --- | --- |
| `processing_frame` | `base_frame` | Frame for projection, height filtering, and output. |
| `horizontal_fov` | `58.4 deg` | Declared horizontal camera field of view. |
| `vertical_fov` | `45.5 deg` | Declared vertical camera field of view. |
| `angle_min/max` | `-29.2 / 29.2 deg` | Narrow output scan sector. |
| `angle_increment` | approximately `1 deg` | Uniform spacing is derived while preserving both FOV endpoints. |
| `range_min/max` | `0.30 / 3.0 m` | Camera observation range. |
| `min_z/max_z` | `-0.07 / 0.18 m` | Camera depth slice in `base_frame`; nominal floor is excluded. |
| `camera_min_x` | `0.30 m` | Rejects points too near or behind the robot origin. |
| `pixel_stride_x/y` | `4 / 4` | Projects the nearest valid depth pixel in each 4x4 block. |
| `max_publish_rate` | `7.0 Hz` | Camera conversion cap for aarch64 load control. |
| `restamp_output` | `false` | Preserves depth-image timestamps. |

Unobserved and filtered bins are NaN. Nav2 configures this source with
`inf_is_valid: false`, so the camera does not create clearing rays outside its
actual sector. Both local and global obstacle layers independently consume:

- `lidar`: `/lidar/filtered_laserscan`, required, 360-degree, marking and clearing;
- `camera`: `/camera/filtered_laserscan`, optional, forward-sector, marking and clearing.

The camera source has no nonzero `expected_update_rate`, so missing camera data
does not make a costmap non-current. The required LiDAR source retains a 0.2 s
update expectation.

## SLAM Toolbox

`online_async_mapping_launch.py` includes the installed SLAM Toolbox
`online_async_launch.py` with
`config/mapper_params_online_async.yaml`.

Current parameters:

| Parameter | Value |
| --- | --- |
| `mode` | `mapping` |
| `scan_topic` | `/lidar/filtered_laserscan_no_downsample` |
| `base_frame` | `base_frame` |
| `odom_frame` | `odom` |
| `map_frame` | `map` |
| `resolution` | `0.04 m/cell` |
| `transform_publish_period` | `0.05 s` |
| `map_update_interval` | `3.0 s` |

SLAM Toolbox consumes the full-resolution filtered LiDAR scan and the existing
`odom -> base_frame` relationship. It publishes the map and maintains
`map -> odom`. It does not replace the EKF's local odometry.

This launch is online mapping, not saved-map localization. Starting it creates
or extends a map from the current run.

When `muto_command_layer` is running alongside this pipeline, export the
current occupancy map through its sanitized SLAM Toolbox wrapper:

```bash
ros2 service call /save_map slam_toolbox/srv/SaveMap \
  "{name: {data: warehouse}}"
```

The default output prefix is `$HOME/.ros/maps/warehouse`; an empty request name
uses `muto_map`. This writes the occupancy-map YAML and image used by Nav2's
map server. It does not serialize the SLAM pose graph or change this pipeline
into a saved-map localization launch. The checked-in mapper profile explicitly
keeps `use_map_saver: true`, which owns `/slam_toolbox/save_map`.

## Nav2 Relationship

The current Nav2 launch starts:

- `controller_server`
- `planner_server`
- `smoother_server`
- `velocity_smoother`
- `behavior_server`
- `bt_navigator`
- the associated lifecycle manager and local/global costmaps

After the top-level Nav2 readiness gate succeeds,
`muto_nav2_pipeline_launch.py` also starts the compact `muto_nav2_bag`
session recorder by default. It records the target-plan-command-response chain,
filtered obstacle scans, action/lifecycle state, and config snapshots under
`/opt/muto_rs_ws/bags`. It deliberately omits continuous costmaps, raw sensor
streams, and high-rate action feedback; use `nav2_bag_full.yaml` only for a
short deep-dive capture. Set `launch_nav2_bag:=false` to opt out.

The local costmap uses `odom`; the global costmap uses `map`. Both use
`base_frame` as the robot frame. Each obstacle layer consumes the downsampled
LiDAR scan and the optional narrow camera scan as separate observation sources. The path
smoother's collision checker also explicitly uses `base_frame` with the global
raw costmap and published footprint; it does not rely on Nav2's upstream
`base_link` default.

This is the current planner/controller/navigation-action stack, not every
optional Nav2 server. It does not start AMCL, waypoint following, route,
docking, or a full saved-map localization workflow.

The Humble behavior trees explicitly compute a Navfn path, apply Humble's
fixed seven-point Savitzky-Golay filter, collision-check the result, and feed
that `smoothed_path` to Regulated Pure Pursuit. `SimpleSmoother` remains loaded
as an explicit rollback but is not selected by either behavior tree. They
replan at 1 Hz. The controller reads `/odometry/filtered`, uses a fixed `0.40 m`
lookahead and requests up to 0.20 m/s linear motion. Keeping the carrot beyond
the approximately `0.26 m` robot radius reduces turn-direction sensitivity
when the path initially lies behind the robot. Its 0.30
rad/s yaw request is a conservative physical envelope for the current Muto
gait: higher geometric commands are possible, but field bags showed poor
achieved yaw and weak simultaneous forward-turn response. Humble RPP's own
`max_angular_accel` is intentionally `12.0 rad/s^2`: this is not a physical
limit. RPP otherwise clamps every rotate-to-path request around the measured
odometry yaw rate, which reduced a requested `0.30 rad/s` turn to approximately
`0.03 rad/s` in the 2026-08-13 field bag. The downstream velocity smoother is
the physical acceleration authority and still ramps yaw at `0.6 rad/s^2`.
Odometry remains authoritative for achieved motion and completion. Its
output is remapped to `/cmd_vel_nav`; the lifecycle-managed
velocity smoother publishes the normal follow-path `/cmd_vel` at 20 Hz.
Recovery behavior output and model-commander direct-rotate output are also
sent to `/cmd_vel_nav`, so the velocity smoother remains the final limiter
before the Muto driver.

NavFn plans a point-center path and therefore cannot itself guarantee that the
complete circular footprint clears an obstacle. Both costmaps use a gradual
`0.42 m` inflation field with scaling factor `6.0`. This leaves a modest
`0.15 m` soft-clearance band outside the effective `0.27 m` robot radius while
still allowing narrow routes in the cluttered deployment room. The smoother
and RPP retain full-footprint collision checking; inflation is a planning
preference, not permission to ignore a collision.

`inflate_around_unknown` is disabled. This is required for frontier
navigation: Humble otherwise uses every unknown map cell as an inflation
source, contradicting NavFn's `allow_unknown: true`. The 2026-08-13 diagnostic
bag showed the consequence of that mismatch: 58,256 unknown SLAM cells became
51,696 lethal and 7,294 inscribed cells in the global costmap. Unknown space
now remains unknown while mapped obstacle cells retain inflation and
full-footprint collision checks.

Recovery is bounded and failure-specific. A planning failure clears only the
global costmap, waits `2 s` for its 1 Hz static/sensor layers to repopulate, and
retries once. Savitzky-Golay smoothing remains collision checked, but its
failure is non-fatal: Nav2 preserves and follows the original NavFn path. This
matters at a frontier because Humble's generic smoothing collision check treats
unknown cost as blocked even though NavFn is intentionally configured with
`allow_unknown: true`.

Synthetic-map regression tests preserve the reason for that setting. With the
optimized frontier decision map enabled, one-cell free-space dilation can place
the selected endpoint in a cell that is still unknown in the raw SLAM map.
Disabling unknown traversal therefore rejects some current frontier goals. Even
with decision-map optimization disabled, the selected known-free endpoint is
immediately adjacent to unknown space: at 0.04 m map resolution, the 0.26 m
robot footprint still overlaps unknown cells and the smoother's full-footprint
check rejects it. The behavior tree consequently treats smoothing as an
optional quality improvement while retaining controller/local-costmap collision
checking during execution.

The explorer's reachability search is point-based rather than footprint-based.
Its map BFS admits a cell when any cell in its 8-connected neighborhood is free,
and its final blocked-goal check samples only the endpoint cost. Sparse diagonal
free cells from a scan can therefore form a reachable-looking one-cell halo
through unknown space even though no continuous robot-sized known-free corridor
exists. Decision-map filtering and one-cell free dilation can widen that effect;
turning optimization off does not remove the underlying point-versus-footprint
distinction. The Muto endpoint adapter is the final full-footprint guard.

Frontier navigation now has a Muto-owned endpoint adapter between the explorer
and Nav2. The upstream explorer still selects and reasons about the actual
frontier target, but sends its `NavigateToPose` request to
`/frontier/navigate_to_pose`. `frontier_goal_adapter` checks the raw `/map` and
uses `map <- base_frame` to find the robot's current connected known-free
component. It selects the frontier-nearest cell in that component whose complete
`0.27 m` effective circular footprint is known free, forwards that staged
endpoint to Nav2, and faces the final pose toward the original frontier. The old
arbitrary `0.80 m` projection cutoff is disabled by the default
`maximum_projection_distance: 0.0`: a distant unsafe frontier can therefore
produce a useful safe advance, allow the map to grow, and be reconsidered on
the next frontier cycle. The adapter rejects only when it cannot find any
footprint-safe cell connected to a safe seed near the robot, when the required
TF/map is unavailable, or when an optional nonzero projection cap explicitly
forbids every reachable candidate. It also rejects a displaced endpoint that
would advance less than `0.20 m`; this prevents Nav2's goal tolerance from
turning a local impasse into a false successful frontier visit.

Frontier suppression is active immediately for commander-owned bounded steps.
One confirmed planner/controller failure, adapter no-progress rejection, or
eight seconds without meaningful navigation progress temporarily suppresses
that robot-width frontier region for `90 s`. There is no post-failure settle
delay: the explorer immediately filters the bad region, selects another
frontier, and sends the replacement goal inside the same primitive. The former
`15 s` suppression startup grace was longer than a normal `10-15 s` primitive
and restarted with every primitive, so it effectively disabled this escape
path. If no other footprint-safe frontier exists, the primitive reports that
fact rather than inventing unsafe motion.

This is intentionally scoped only to frontier exploration. Object approach and
operator navigation continue to use `/navigate_to_pose` directly. The compact
and full Nav2 bag profiles record the original goal, projected goal, adapter
status, and adapter action status, so endpoint projection is visible during bag
analysis.
A controller/progress failure clears only the local costmap, waits `1 s` for
fresh 5 Hz obstacle updates, and retries FollowPath once. If the complete
navigation pipeline still fails, it performs one stationary `2 s` wait and
returns failure to the frontier selector. It does not spin or back up merely
because a path could not be planned; the frontier layer can suppress that goal
and select another. Humble's Wait BT input is integer seconds, so these delays
deliberately avoid sub-second literals. Footprint collision checking remains
enabled throughout.

## Main Runtime Contract

| Topic or transform | Owner | Consumer |
| --- | --- | --- |
| `/lidar/raw_laserscan` | TG30 driver | LiDAR scan filter. |
| `/lidar/filtered_laserscan` | LiDAR scan filter | RF2O and both Nav2 obstacle layers. |
| `/lidar/filtered_laserscan_no_downsample` | LiDAR scan filter | SLAM Toolbox. |
| `/scan_odom_raw` | RF2O | Odometry guard wrapper. |
| `/scan_odom` | Odometry guard wrapper | EKF. |
| `/odometry/filtered` | EKF | Mapping readiness and inspection. |
| `odom -> base_frame` | EKF | SLAM Toolbox and Nav2. |
| `/camera/filtered_laserscan` | Depth-to-scan node | Both Nav2 obstacle layers. |
| `/map` and `map -> odom` | SLAM Toolbox | Nav2 global planning and TF. |
| `/cmd_vel_nav` | Nav2 controller, behavior server, and model commander direct rotate | Nav2 velocity smoother. |
| `/cmd_vel` | Nav2 velocity smoother | Muto driver and RF2O command-aware guard. |
| `/muto/commanded_gait_state` | Fixed-rate Muto locomotion loop | Backward-compatible stance/swing and continuous foot targets for foot odometry and motor validation. |
| `/muto/motion_command_state` | Muto driver | Requested twist, selected/active levels, pending and projection flags, profile, and feed-forward prediction. |
| `/foot_odom` | Commanded-stance estimator with motor FK validation | EKF planar velocity overlay. |

## Runtime Checks

Check hardware and static TF:

```bash
ros2 topic hz /lidar/raw_laserscan
ros2 topic echo /camera/depth/camera_info --once
ros2 topic hz /imu/data_processed
ros2 topic hz /imu/controller_attitude
ros2 topic echo /muto/imu_telemetry_status --once \
  --qos-durability transient_local
ros2 topic echo /muto/controller_attitude_yaw_status --once \
  --qos-durability transient_local
ros2 run tf2_ros tf2_echo base_frame lidar_frame
ros2 run tf2_ros tf2_echo base_frame imu_link
ros2 run tf2_ros tf2_echo base_frame camera_depth_optical_frame
```

`/imu/controller_attitude_imu` is intentionally sparse: one startup anchor and
one accepted correction per stationary episode. Use the yaw-status topic above
instead of `ros2 topic hz` to inspect its 5 Hz source cadence and gate counts.

Check LiDAR odometry and EKF:

```bash
ros2 topic hz /lidar/filtered_laserscan
ros2 topic hz /scan_odom_raw
ros2 topic hz /scan_odom
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_frame
```

Check camera conversion and both Nav2 scan sources:

```bash
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/filtered_laserscan
ros2 topic hz /lidar/filtered_laserscan
ros2 topic hz /lidar/filtered_laserscan_no_downsample
ros2 topic echo /camera/filtered_laserscan --once
```

Check mapping and Nav2:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_frame
ros2 node list
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /velocity_smoother
```

## Failure Isolation

### Localization readiness timeout

Check, in order:

1. `/imu/data_raw` and `/imu/data_processed` produce changed snapshots after
   the bounded startup calibration. Identical controller-cached accel/gyro
   replies are suppressed by default. A read warning distinguishes no
   controller bytes from bytes that failed frame validation.
2. `/lidar/raw_laserscan` is live and stamped near the ROS clock.
3. Its `header.frame_id` is `lidar_frame` or has a valid transform to
   `base_frame`.
4. `base_frame <- imu_link` exists.
5. The LiDAR filter, RF2O, and EKF processes stay alive after launch.

### Mapping readiness timeout

Check:

1. `/lidar/filtered_laserscan_no_downsample` is live.
2. `/scan_odom_raw` and `/scan_odom` are live.
3. `/odometry/filtered` is live.
4. Exactly one publisher owns `odom -> base_frame`.

The mapping gate does not require camera depth. SLAM uses LiDAR directly, so
mapping proceeds if the camera branch is unavailable.

### No camera obstacle scan

Check:

1. `camera_depth_to_laserscan_node` is running.
2. `base_frame <- camera_depth_optical_frame` exists.
3. Depth encoding is `16UC1` and CameraInfo dimensions match.
4. Sensor timestamps are not being dropped by `max_input_age`.
5. `launch_camera_obstacle_scan` was not intentionally disabled.

Camera loss does not stop SLAM or the LiDAR costmap source. Diagnose it as a
reduced-obstacle-coverage condition, not as a mapping failure.

### LiDAR scan is live but no map

Check:

1. The no-downsample LiDAR scan header frame connects to `base_frame`.
2. `odom -> base_frame` is continuous and timestamp-compatible.
3. The scan contains enough finite ranges.
4. SLAM Toolbox is active and subscribed to
   `/lidar/filtered_laserscan_no_downsample`.
5. The robot is moving through observable geometry rather than spinning in a
   feature-poor or highly dynamic area.

### Nav2 readiness timeout

Check:

1. `/map` and `/lidar/filtered_laserscan` are live.
2. Both `map -> odom` and `odom -> base_frame` are available.
3. `map -> base_frame` resolves at current sensor timestamps.
4. SLAM has published at least one usable map before Nav2 activation.

## Timing Rules

The live pipeline preserves sensor timestamps by default:

- LiDAR filtering preserves the TG30 scan stamp.
- RF2O uses the processed scan stamp.
- The odometry wrapper preserves RF2O timestamps.
- Depth conversion preserves sensor time.
- The EKF and TF consumers depend on those timestamps.

Do not enable restamping merely to hide a clock or driver problem. Large age
warnings, continuous TF extrapolation failures, or message-filter drops after
startup should be fixed at the clock/stamp source.

Short message-filter drops during startup can occur while TF buffers fill.
Continuous drops after readiness gates pass indicate a real TF or timing
problem.

## Tuning Order

Tune from upstream to downstream:

1. Verify static sensor calibration and timestamps.
2. Verify raw and filtered LiDAR scan quality.
3. Verify RF2O and EKF odometry while driving known paths.
4. Verify depth projection and both independent scan topics in RViz.
5. Tune SLAM resolution and scan matching only after odometry and scans are
   stable.
6. Tune Nav2 costmaps and planners only after the complete TF chain is stable.

Changing SLAM parameters cannot repair bad sensor TF, stale timestamps, sparse
scans, or discontinuous odometry.

For aarch64, profile before increasing camera processing above `7 Hz`,
reducing the 4x4 depth stride, or decreasing the `0.04 m` map resolution.
Those changes increase CPU, memory, or both.

## Current Limits

- The environment should be mostly static while mapping. Moving people or
  objects can produce transient scan geometry and map artifacts.
- RF2O can drift or jump in feature-poor geometry; the wrapper reduces but
  cannot eliminate failed scan matches.
- The default `0x60` branch supplies sparse stop-time relative heading checks
  with an arbitrary startup zero. It is not a globally referenced heading;
  power-cycle and magnetic-disturbance tests remain required.
- Camera depth augments Nav2 costmaps but is not a SLAM or EKF odometry source.
- SLAM and LiDAR costmap coverage continue when camera data is unavailable.
- Online mapping does not provide a saved-map localization workflow.
- Calibration and filter bounds remain deployment-specific and must be checked
  on the physical robot.
- Nav2's current `0.16 m` radius models the central body, not the roughly
  `0.295 m` zero-pose leg envelope measured from the reference Yahboom tutorial
  URDF. Validate the swept gait envelope before using narrow clearances.

## Removed Legacy Paths

The active pipeline does not use:

- `/lidar/PointCloud`
- `/lidar/PointCloudFiltered`
- `/lidar/PointCloudFilteredNoDownsample`
- `/camera/depth/points`
- `lidar_pointcloud_filter_node`
- `camera_pointcloud_to_laserscan_node`
- `camera_pointcloud_to_laserscan_launch.py`

Do not reintroduce these names into launch files or deployment instructions.
The current implementation uses TG30 `LaserScan`, raw `16UC1` depth images,
and `camera_depth_to_laserscan_node`.

## Key Files

| File | Role |
| --- | --- |
| `src/muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Full readiness-gated startup, including the compact default Nav2 session recorder. |
| `src/muto_nav2_bag/config/nav2_bag.yaml` | Compact default navigation evidence profile. |
| `src/muto_nav2_bag/config/nav2_bag_full.yaml` | Opt-in broad costmap/action-feedback diagnostic profile. |
| `src/muto_slam_mapping/launch/online_async_mapping_launch.py` | SLAM Toolbox mapping only. |
| `src/muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Current Nav2 server set. |
| `src/muto_slam_mapping/launch/frontier_exploration_launch.py` | Optional exploration client launch; not part of one-shot Nav2 startup. |
| `src/muto_slam_mapping/config/frontier_exploration_params.yaml` | Muto frame, topic, QoS, and bounded-DP exploration profile. |
| `src/muto_slam_mapping/config/mapper_params_online_async.yaml` | SLAM frames, topic, and map timing. |
| `src/muto_slam_mapping/config/nav2_params.yaml` | Humble planner, path smoother, controller, velocity smoother, behavior, and costmap settings. |
| `src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | LiDAR scan filtering, RF2O, and odometry guard. |
| `src/lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Depth-to-scan component launch. |
| `src/yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Normal localization layer. |
| `src/yahboomcar_bringup/config/ekf_lidar_imu.yaml` | EKF frame and source configuration. |
| `src/tf2_publisher/launch/all_tf2_publishers_launch.py` | Static sensor mounts. |
| `src/muto_command_layer/launch/command_layer_launch.py` | Full command stack; includes the lower object pipeline. |
| `src/muto_command_layer/launch/object_pipeline_launch.py` | Lower SAM2/registry/VLM startup only. |
| `src/yahboomcar_description/README.md` | Reference-only Yahboom tutorial URDF boundary and footprint measurements. |
