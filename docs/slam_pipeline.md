# SLAM and Nav2 Pipeline

Date: 2026-07-24

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
       |    -> RF2O
       |    -> /scan_odom_raw
       |    -> odometry deadband/jump filter
       |    -> /scan_odom
       |    -> robot_localization EKF <- /imu/data_processed
       |    -> /odometry/filtered + odom -> base_frame
       |
       -> /lidar/filtered_laserscan_no_downsample
            |
            +-------------------------------------+
                                                  |
depth image + depth CameraInfo                    |
  -> camera_depth_to_laserscan_node               |
  -> /camera/filtered_laserscan                   |
            |                                     |
            +-> laserscan_fusion_node <-----------+
                   -> /fused/laserscan
                   -> SLAM Toolbox
                        -> /map + map -> odom
                        -> Nav2 planner/controller/costmaps
```

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
cd ~/Documents/testground/muto_rs_deployment_test
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

The launch combines minimum delays with observable readiness checks. A delay
does not declare a stage ready; it only determines when that stage begins
checking.

| Stage | Minimum delay | Readiness timeout | Required state before launch |
| --- | ---: | ---: | --- |
| Static sensor TF | 1 s | n/a | Timer only. |
| Localization | 3 s | 60 s | `/lidar/raw_laserscan`, `base_frame <- lidar_frame`, and `base_frame <- imu_link`. |
| Mapping | 8 s | 90 s | `/odometry/filtered`, `/lidar/filtered_laserscan_no_downsample`, and `odom <- base_frame`. |
| Nav2 | 12 s | 120 s | `/map`, `/fused/laserscan`, and `map <- base_frame`. |

A failed readiness gate shuts down the pipeline instead of launching its
downstream stage against incomplete topics or TF. This matters on aarch64,
where driver, camera, and mapping startup time can vary.

Useful top-level switches include:

| Argument | Default | Effect |
| --- | --- | --- |
| `launch_hardware` | `true` | Starts the TG30, Orbbec launch, and Muto driver. |
| `launch_sensor_tf` | `true` | Starts static sensor mounts. |
| `launch_localization` | `true` | Starts LiDAR filtering, RF2O, and EKF. |
| `launch_mapping` | `true` | Starts fused scan generation and SLAM Toolbox. |
| `launch_nav2` | `true` | Starts the current Nav2 server set. |
| `launch_fused_laserscan` | `true` | Lets the mapping launch own scan fusion. |
| `fused_scan_max_publish_rate` | `7.0` | Caps camera depth-to-scan processing. |
| `fused_scan_camera_max_age` | `0.5` | Maximum camera/LiDAR timestamp difference. |

If a prerequisite stage is disabled, any enabled downstream stage must already
have equivalent topics and TF supplied externally.

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

Fused scan and mapping:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

Nav2 planner, controller, behavior, smoother, and navigator servers:

```bash
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

`online_async_mapping_launch.py` starts scan fusion by default. Do not also
launch `camera_depth_to_laserscan_launch.py` during normal mapping. If another
process already owns `/fused/laserscan`, use:

```bash
ros2 launch muto_slam_mapping online_async_mapping_launch.py \
  launch_fused_laserscan:=false
```

To test only depth conversion and scan fusion after hardware, static TF, and
the LiDAR filter are running:

```bash
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
```

Stop this component launch before starting normal mapping, or start mapping with
`launch_fused_laserscan:=false`.

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

The Orbbec launch must publish the internal transform from `camera_link` to
the depth optical frame. The local TF package owns only the camera-body mount.

## LiDAR Filtering And Localization

The active LiDAR path is entirely `LaserScan` based:

| Stage | Input | Output | Important defaults |
| --- | --- | --- | --- |
| Scan filter | `/lidar/raw_laserscan` | `/lidar/filtered_laserscan` | Range `0.05-10.0 m`, downsample factor `2`. |
| Full-resolution filter output | Same raw scan | `/lidar/filtered_laserscan_no_downsample` | Range up to `15.0 m`; reserved for fusion. |
| RF2O | `/lidar/filtered_laserscan` | `/scan_odom_raw` | `16 Hz`, frames `odom` and `base_frame`, internal TF output disabled. |
| Deadband/jump wrapper | `/scan_odom_raw` | `/scan_odom` | Suppresses stationary drift and implausible jumps. |
| EKF | `/scan_odom` plus `/imu/data_processed` | `/odometry/filtered` and `odom -> base_frame` | `30 Hz`, 2D mode. |

The EKF fuses RF2O planar position and yaw. The IMU contributes yaw rate only;
it does not provide absolute orientation or translational odometry.

The deadband wrapper applies its translation and yaw guards per axis when
recent `cmd_vel` indicates that axis is stationary. Standalone
`filter_lidar_odometry_launch.py` lets the wrapper publish
`odom -> base_frame` by default for testing. When the EKF is running, keep
`rf2o_publish_tf:=false`.

## Depth-To-Scan And Fusion

The active camera path consumes the raw depth image directly. It does not use
`/camera/depth/points`.

```text
/camera/depth/image_raw + /camera/depth/camera_info
  -> sampled depth back-projection
  -> TF into base_frame
  -> deployment z/range filtering
  -> /camera/filtered_laserscan
```

That camera scan is merged with
`/lidar/filtered_laserscan_no_downsample` into
`/fused/laserscan`.

Current defaults:

| Setting | Default | Meaning |
| --- | --- | --- |
| `processing_frame` | `base_frame` | Frame for depth projection and filtering. |
| `fused_scan_frame` | `base_frame` | Header frame of the final scan. |
| `angle_min/max` | `-pi / pi` | Full-circle fused scan. |
| `camera_angle_increment` | `1 deg` | Intermediate camera scan resolution. |
| `fused_angle_increment` | `0.25 deg` | Final fused scan resolution. |
| `range_max` | `3.0 m` | Depth-camera range cap. |
| `lidar_range_max` | `15.0 m` | LiDAR range cap in the fused scan. |
| `min_z/max_z` | `-0.10 / 0.18 m` | Camera depth slice in `base_frame`. |
| `camera_min_x` | `0.30 m` | Rejects camera points too near/behind the robot origin. |
| `pixel_stride_x/y` | `4 / 4` | Projects the nearest valid depth pixel in each 4x4 block. |
| `max_publish_rate` | `7.0 Hz` | Camera conversion cap for aarch64 load control. |
| `max_lidar_age` | `0.5 s` | Maximum camera/LiDAR timestamp difference. |
| `require_lidar_scan` | `true` | LiDAR is required and drives fused output. |
| `restamp_output` | `false` | Preserves sensor timestamps. |

LiDAR remains the fallback source. A missing or stale camera scan is omitted
while LiDAR-only `/fused/laserscan` output continues. A valid depth frame with
no usable pixels produces an empty camera scan and does not block LiDAR fusion.

The no-downsample LiDAR topic is intentional: RF2O uses a lighter scan, while
fusion retains the TG30 angular density.

## SLAM Toolbox

`online_async_mapping_launch.py` includes the installed SLAM Toolbox
`online_async_launch.py` with
`config/mapper_params_online_async.yaml`.

Current parameters:

| Parameter | Value |
| --- | --- |
| `mode` | `mapping` |
| `scan_topic` | `/fused/laserscan` |
| `base_frame` | `base_frame` |
| `odom_frame` | `odom` |
| `map_frame` | `map` |
| `resolution` | `0.04 m/cell` |
| `transform_publish_period` | `0.08 s` |
| `map_update_interval` | `3.0 s` |

SLAM Toolbox consumes the fused scan and the existing
`odom -> base_frame` relationship. It publishes the map and maintains
`map -> odom`. It does not replace the EKF's local odometry.

This launch is online mapping, not saved-map localization. Starting it creates
or extends a map from the current run.

## Nav2 Relationship

The current Nav2 launch starts:

- `controller_server`
- `planner_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`
- the associated lifecycle manager and local/global costmaps

The local costmap uses `odom`; the global costmap uses `map`. Both use
`base_frame` as the robot frame and consume `/fused/laserscan`.

This is the current planner/controller/navigation-action stack, not every
optional Nav2 server. It does not start AMCL, waypoint following, route,
docking, or a full saved-map localization workflow.

## Main Runtime Contract

| Topic or transform | Owner | Consumer |
| --- | --- | --- |
| `/lidar/raw_laserscan` | TG30 driver | LiDAR scan filter. |
| `/lidar/filtered_laserscan` | LiDAR scan filter | RF2O. |
| `/lidar/filtered_laserscan_no_downsample` | LiDAR scan filter | Scan fusion. |
| `/scan_odom_raw` | RF2O | Odometry guard wrapper. |
| `/scan_odom` | Odometry guard wrapper | EKF. |
| `/odometry/filtered` | EKF | Mapping readiness and inspection. |
| `odom -> base_frame` | EKF | SLAM Toolbox and Nav2. |
| `/camera/filtered_laserscan` | Depth-to-scan node | Scan fusion. |
| `/fused/laserscan` | Scan fusion | SLAM Toolbox and Nav2 costmaps. |
| `/map` and `map -> odom` | SLAM Toolbox | Nav2 global planning and TF. |

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

Check camera conversion and fusion:

```bash
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/filtered_laserscan
ros2 topic hz /lidar/filtered_laserscan_no_downsample
ros2 topic hz /fused/laserscan
ros2 topic echo /fused/laserscan --once
```

Check mapping and Nav2:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_frame
ros2 node list
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

## Failure Isolation

### Localization readiness timeout

Check, in order:

1. `/lidar/raw_laserscan` is live and stamped near the ROS clock.
2. Its `header.frame_id` is `lidar_frame` or has a valid transform to
   `base_frame`.
3. `base_frame <- imu_link` exists.
4. The LiDAR filter, RF2O, and EKF processes stay alive after launch.

### Mapping readiness timeout

Check:

1. `/lidar/filtered_laserscan_no_downsample` is live.
2. `/scan_odom_raw` and `/scan_odom` are live.
3. `/odometry/filtered` is live.
4. Exactly one publisher owns `odom -> base_frame`.

The mapping gate does not require camera depth. Mapping can proceed with the
LiDAR-driven fused scan even if the camera branch is temporarily unavailable.

### No fused scan

Check:

1. The no-downsample LiDAR scan is live.
2. `camera_depth_to_laserscan_node` and `laserscan_fusion_node` are running.
3. `base_frame <- camera_depth_optical_frame` exists.
4. Depth encoding and `CameraInfo` dimensions are valid.
5. Sensor timestamps are not being dropped by `max_input_age`.

Because LiDAR drives fusion, complete loss of `/fused/laserscan` usually
points first to the LiDAR input, fusion process, or timestamps rather than to
missing camera returns.

### Fused scan is live but no map

Check:

1. The fused scan header frame connects to `base_frame`.
2. `odom -> base_frame` is continuous and timestamp-compatible.
3. The scan contains enough finite ranges.
4. SLAM Toolbox is active and subscribed to `/fused/laserscan`.
5. The robot is moving through observable geometry rather than spinning in a
   feature-poor or highly dynamic area.

### Nav2 readiness timeout

Check:

1. `/map` and `/fused/laserscan` are live.
2. Both `map -> odom` and `odom -> base_frame` are available.
3. `map -> base_frame` resolves at current sensor timestamps.
4. SLAM has published at least one usable map before Nav2 activation.

## Timing Rules

The live pipeline preserves sensor timestamps by default:

- LiDAR filtering preserves the TG30 scan stamp.
- RF2O uses the processed scan stamp.
- The odometry wrapper preserves RF2O timestamps.
- Depth conversion and fusion preserve sensor time.
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
4. Verify depth projection and the fused scan in RViz.
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
- Camera depth augments mapping and costmaps but is not an EKF odometry source.
- LiDAR-only fusion is intentionally supported when camera data is stale or
  unavailable.
- Online mapping does not provide a saved-map localization workflow.
- Calibration and filter bounds remain deployment-specific and must be checked
  on the physical robot.

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
`camera_depth_to_laserscan_node`, and `laserscan_fusion_node`.

## Key Files

| File | Role |
| --- | --- |
| `src/muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Full readiness-gated startup. |
| `src/muto_slam_mapping/launch/online_async_mapping_launch.py` | Fused scan plus SLAM Toolbox mapping. |
| `src/muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Current Nav2 server set. |
| `src/muto_slam_mapping/config/mapper_params_online_async.yaml` | SLAM frames, topic, and map timing. |
| `src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | LiDAR scan filtering, RF2O, and odometry guard. |
| `src/lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Depth conversion and scan fusion component launch. |
| `src/yahboomcar_bringup/launch/ekf_imu_lidar_launch.py` | Normal localization layer. |
| `src/yahboomcar_bringup/config/ekf_lidar_imu.yaml` | EKF frame and source configuration. |
| `src/tf2_publisher/launch/all_tf2_publishers_launch.py` | Static sensor mounts. |
