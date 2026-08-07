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
| `cmd_vel_timeout` | `0.5` | Returns the gait to standby when velocity commands stop. |
| `locomotion_command_mapping` | `calibrated` | Uses an explicit physical velocity profile; `legacy_100` is rollback-only. |
| `locomotion_calibration_file` | `muto_locomotion_provisional_20260806.yaml` | Provisional gait-level mapping; replace after marked-field trials. |
| `imu_calibration_sample_count` | `300` | Valid stationary IMU samples targeted at startup. |
| `imu_calibration_max_reads` | `600` | Maximum serial attempts allowed for that calibration. |
| `imu_calibration_timeout_sec` | `30.0` | Hard wall-clock cap on startup calibration. |
| `foot_motor_poll_rate` | `2.0` | Production limit for synchronized 18-joint feedback. The tested 10 Hz rate delayed gait and IMU processing. |
| `allow_experimental_high_rate_motor_polling` | `false` | Required explicit opt-in above 2 Hz; never enable for normal deployment with the current blocking serial service. |

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
| `/imu/data_processed` | `sensor_msgs/msg/Imu`, frame `imu_link`; yaw rate is the active EKF field. |

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
| EKF | `/scan_odom` plus `/imu/data_processed` | `/odometry/filtered` and `odom -> base_frame` | `30 Hz`, 2D mode. |

The EKF fuses RF2O planar position and yaw. The IMU contributes yaw rate only;
it does not provide absolute orientation or translational odometry.

The optional foot input is generated from a fixed-rate locomotion state stream.
`/cmd_vel` changes the desired command; the driver advances one trajectory phase
per 50 Hz tick and publishes the phase after sending its motor targets. A newer
nonzero command waits for the next complete gait boundary. The accompanying
motion-command state reports selected and active raw levels separately and
whether a replacement is pending.
A stale or zero command returns directly to nominal stance instead of performing
a smooth deceleration. Motor service reads use the gait target current during
that serial read and add no artificial settling delay. The read still holds the
shared serial bus, so slow responses can delay the gait and IMU timers.

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

The local costmap uses `odom`; the global costmap uses `map`. Both use
`base_frame` as the robot frame. Each obstacle layer consumes the downsampled
LiDAR scan and the optional narrow camera scan as separate observation sources. The path
smoother's collision checker also explicitly uses `base_frame` with the global
raw costmap and published footprint; it does not rely on Nav2's upstream
`base_link` default.

This is the current planner/controller/navigation-action stack, not every
optional Nav2 server. It does not start AMCL, waypoint following, route,
docking, or a full saved-map localization workflow.

The Humble behavior trees explicitly compute a Navfn path, collision-check a
Simple Smoother result, and feed that `smoothed_path` to Regulated Pure
Pursuit. They replan at 1 Hz. The controller reads `/odometry/filtered`, uses a
fixed 0.25 m lookahead, requests up to 0.25 m/s linear motion, and is capped at
0.18 rad/s angular motion by the provisional yaw envelope transferred from the
2026-08-06 pure-turn observation. Its
output is remapped to `/cmd_vel_nav`; the lifecycle-managed
velocity smoother publishes the normal follow-path `/cmd_vel` at 20 Hz.
Recovery behaviors currently publish directly to `/cmd_vel` and therefore
bypass the smoother, while retaining the behavior server's conservative
rotation limits and the BT's bounded backup speed.

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
| `/cmd_vel_nav` | Nav2 controller | Nav2 velocity smoother. |
| `/cmd_vel` | Nav2 velocity smoother or recovery behavior server | Muto driver and RF2O command-aware guard. |
| `/muto/commanded_gait_state` | Fixed-rate Muto locomotion loop | Backward-compatible stance/swing and continuous foot targets for foot odometry and motor validation. |
| `/muto/motion_command_state` | Muto driver | Requested twist, selected/active levels, pending and projection flags, profile, and feed-forward prediction. |
| `/foot_odom` | Commanded-stance estimator with motor FK validation | EKF planar velocity overlay. |

## Runtime Checks

Check hardware and static TF:

```bash
ros2 topic hz /lidar/raw_laserscan
ros2 topic echo /camera/depth/camera_info --once
ros2 topic hz /imu/data_processed
ros2 run tf2_ros tf2_echo base_frame lidar_frame
ros2 run tf2_ros tf2_echo base_frame imu_link
ros2 run tf2_ros tf2_echo base_frame camera_depth_optical_frame
```

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

1. `/imu/data_raw` and `/imu/data_processed` are live after the bounded startup
   calibration. A read warning distinguishes no controller bytes from bytes
   that failed frame validation.
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
- The IMU supplies yaw rate only, not absolute heading.
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
| `src/muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Full readiness-gated startup. |
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
