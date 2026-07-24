# Muto Mapping and Costmap Notes

Date: 2026-07-24

This document describes the current mapping and Nav2 costmap design for the
Muto RS ROS 2 Humble deployment. It focuses on how the fused scan becomes a
SLAM map and how that map plus live observations become local and global
costmaps.

For startup order, readiness gates, odometry, and failure isolation, use
[slam_pipeline.md](slam_pipeline.md).

## System View

```text
/lidar/filtered_laserscan
  -> RF2O
  -> EKF
  -> odom -> base_frame

/lidar/filtered_laserscan_no_downsample -----------+
                                                    |
/camera/depth/image_raw + CameraInfo                |
  -> /camera/filtered_laserscan --------------------+
                                                    |
                                                    v
                                           /fused/laserscan
                                              |          |
                                              |          +-> local costmap
                                              |          +-> global costmap
                                              |
                                              +-> SLAM Toolbox
                                                   -> /map
                                                   -> map -> odom
```

The two LiDAR filter outputs have separate jobs:

| Topic | Role |
| --- | --- |
| `/lidar/filtered_laserscan` | Downsampled LiDAR-only scan used by RF2O odometry. |
| `/lidar/filtered_laserscan_no_downsample` | Full-resolution LiDAR scan used by scan fusion. |
| `/fused/laserscan` | LiDAR plus current usable depth-camera observations used by SLAM and both Nav2 costmaps. |

The active path is `LaserScan` based. It does not use the removed TG30
PointCloud filtering or `/camera/depth/points` pipeline.

## Frame Model And Ownership

```text
map
  -> odom
    -> base_frame
      -> lidar_frame
      -> imu_link
      -> camera_link
        -> camera optical frames
```

| Transform | Owner in the normal pipeline |
| --- | --- |
| `map -> odom` | SLAM Toolbox while online mapping is active. |
| `odom -> base_frame` | `robot_localization/ekf_node`. |
| `base_frame -> lidar_frame` | Static TF publisher. |
| `base_frame -> imu_link` | Static TF publisher. |
| `base_frame -> camera_link` | Static TF publisher. |
| Camera body to optical frames | Orbbec driver. |

The local costmap uses `odom` because it needs a continuous local frame. The
global costmap uses `map` because global plans and the SLAM occupancy grid are
expressed there.

Only one node may publish `odom -> base_frame`. The normal EKF launch disables
RF2O-wrapper TF output. Standalone LiDAR odometry may publish this transform,
but it must be disabled before starting the EKF.

## Scan Fusion Contract

The mapping launch starts
`lidar_pointcloud_filter/camera_depth_to_laserscan_launch.py` by default.

Camera conversion:

```text
/camera/depth/image_raw (16UC1)
/camera/depth/camera_info
  -> nearest valid depth sample from each 4x4 block
  -> back-project with camera intrinsics
  -> transform to base_frame
  -> apply camera range and height filters
  -> /camera/filtered_laserscan
```

Fusion:

```text
/camera/filtered_laserscan
/lidar/filtered_laserscan_no_downsample
  -> laserscan_fusion_node
  -> /fused/laserscan (frame: base_frame)
```

Current defaults:

| Parameter | Value | Effect |
| --- | ---: | --- |
| `processing_frame` | `base_frame` | Camera projection and filtering frame. |
| `fused_scan_frame` | `base_frame` | Final scan frame. |
| Camera range | `3.0 m` | Maximum depth-camera contribution. |
| LiDAR range | `15.0 m` | Maximum LiDAR contribution to the fused scan. |
| Camera z slice | `-0.10 to 0.18 m` | Keeps depth points near the navigation obstacle plane. |
| `camera_min_x` | `0.30 m` | Rejects camera points too near or behind the robot origin. |
| Camera angle increment | `1 deg` | Intermediate camera scan density. |
| Fused angle increment | `0.25 deg` | Final scan density. |
| Depth block size | `4 x 4` | One nearest valid depth sample is projected per block. |
| Camera conversion cap | `7.0 Hz` | Bounds CPU use on aarch64. |
| Camera/LiDAR tolerance | `0.5 s` | Older camera data is excluded. |
| `require_lidar_scan` | `true` | LiDAR drives fused output. |
| `restamp_output` | `false` | Preserves sensor timestamps. |

LiDAR-only output continues when camera depth is missing, stale, or has no valid
returns. Complete loss of `/fused/laserscan` therefore usually indicates a
missing LiDAR scan, a fusion-process failure, or invalid timestamps rather than
an empty camera image.

## Mapping Ownership

`online_async_mapping_launch.py` starts:

- scan fusion, unless `launch_fused_laserscan:=false`;
- installed SLAM Toolbox `online_async_launch.py`;
- `mapper_params_online_async.yaml`.

Current SLAM settings:

| Parameter | Value |
| --- | --- |
| `mode` | `mapping` |
| `scan_topic` | `/fused/laserscan` |
| `map_frame` | `map` |
| `odom_frame` | `odom` |
| `base_frame` | `base_frame` |
| `resolution` | `0.04 m/cell` |
| `transform_publish_period` | `0.08 s` |
| `map_update_interval` | `3.0 s` |

SLAM Toolbox owns the persistent occupancy estimate for the current run and
publishes `map -> odom`. It consumes the existing EKF
`odom -> base_frame` relationship; it does not replace local odometry.

The mapping launch is online mapping, not AMCL or saved-map localization. Nav2
runs against the live SLAM map.

## Costmap Ownership

The costmaps are created inside their Nav2 servers:

| Costmap | Owning server | Global frame |
| --- | --- | --- |
| Local costmap | `controller_server` | `odom` |
| Global costmap | `planner_server` | `map` |

`nav2_planner_controller_launch.py` lifecycle-manages:

- `controller_server`
- `planner_server`
- `smoother_server`
- `behavior_server`
- `bt_navigator`

It does not launch AMCL or the full optional Nav2 server set.

Both costmaps use:

- `robot_base_frame: base_frame`;
- `/fused/laserscan` as a live obstacle source;
- `resolution: 0.04`;
- `transform_tolerance: 0.2`;
- `robot_radius: 0.16`;
- `footprint_padding: 0.01`;
- marking and raytracing;
- the same inflation settings.

## Robot Footprint Assumption

> **Important:** The costmaps model a body-and-fixed-sensor radius of
> `0.16 m` with `0.01 m` footprint padding. The approximately `0.30 m`
> full zero-pose leg collision radius is intentionally not represented.

This configuration assumes navigation clearance is based on the compact body,
not every possible leg pose or gait sweep. Inflation does not correct an
undersized physical footprint: it changes obstacle costs around the configured
footprint, while collision checks still depend on the footprint model.

Before autonomous operation near furniture, walls, people, or narrow passages,
validate the real swept leg envelope for the active gait. Increase the modeled
footprint or impose equivalent clearance elsewhere if leg contact is possible.

## Local Costmap

The local costmap is a rolling obstacle map for control and collision checking.

| Setting | Value |
| --- | --- |
| `global_frame` | `odom` |
| `rolling_window` | `true` |
| Width / height | `3.0 m / 3.0 m` |
| Resolution | `0.04 m/cell` |
| Update frequency | `5.0 Hz` |
| Publish frequency | `2.0 Hz` |
| Plugins | obstacle, inflation |
| `always_send_full_costmap` | `true` |

There is no static layer in the local costmap. Its purpose is to represent the
robot's immediate surroundings in a continuous frame without carrying the full
SLAM map.

Because the window is only 3 m square, obstacles leave the local map as the
robot moves away. The global map remains responsible for long-range route
structure.

## Global Costmap

The global costmap combines the SLAM occupancy grid with current sensor
observations.

| Setting | Value |
| --- | --- |
| `global_frame` | `map` |
| Resolution | `0.04 m/cell` |
| Update frequency | `1.0 Hz` |
| Publish frequency | `1.0 Hz` |
| `track_unknown_space` | `true` |
| Plugins | static, obstacle, inflation |
| `always_send_full_costmap` | `true` |

The static layer subscribes transient-local to `/map` and accepts map updates.
This lets the global costmap follow the live SLAM map without restarting Nav2.

The obstacle layer overlays current `/fused/laserscan` observations on top of
the static map. A transient obstacle can therefore affect global planning even
before it becomes part of the SLAM occupancy grid.

## Obstacle Layer

Both costmaps use the same obstacle-source policy:

| Setting | Value |
| --- | ---: |
| Topic | `/fused/laserscan` |
| Data type | `LaserScan` |
| Expected update period | `0.1 s` |
| Obstacle marking range | `2.5 m` |
| Raytrace clearing range | `3.0 m` |
| Minimum obstacle range | `0.0 m` |
| Minimum raytrace range | `0.0 m` |
| Marking | enabled |
| Clearing | enabled |
| Footprint clearing | enabled |
| `inf_is_valid` | `false` |
| Combination method | `1` |

The fused scan can contain LiDAR returns out to 15 m and camera returns out to
3 m, but the costmaps mark obstacles only within 2.5 m and clear only within
3 m. Long-range LiDAR data can still help SLAM even though the costmap obstacle
layer ignores it beyond those limits.

`expected_update_rate: 0.1` is a period in seconds, not 0.1 Hz. It expects
observations roughly every 100 ms. Sustained fused-scan output below 10 Hz may
produce stale-source warnings even though the camera conversion itself is
capped at 7 Hz; the LiDAR-driven fusion output is expected to continue at the
LiDAR rate.

With `inf_is_valid: false`, infinite ranges are not treated as explicit
clearing observations. Clearing relies on valid finite ray endpoints and
raytracing behavior.

## Inflation Layer

Both costmaps use:

| Setting | Value |
| --- | ---: |
| `inflation_radius` | `0.4 m` |
| `cost_scaling_factor` | `12.0` |
| `inflate_unknown` | `false` |
| `inflate_around_unknown` | `true` |

The inflation radius defines how far obstacle costs extend. The relatively high
cost-scaling factor makes those costs decay steeply away from the lethal
footprint.

Inflation is a planning preference and collision-cost field. It is not a
substitute for an accurate physical footprint or a guaranteed 0.4 m clearance.

## Planner And Controller Interaction

The global planner is Navfn:

| Setting | Value |
| --- | --- |
| Planner plugin | `nav2_navfn_planner/NavfnPlanner` |
| `use_astar` | `false` |
| `allow_unknown` | `true` |
| Goal tolerance | `0.5 m` |
| Expected planner frequency | `20 Hz` |

Because both `track_unknown_space` and `allow_unknown` are enabled, the
planner may route through unknown global-map space when a path is otherwise
valid. Disable `allow_unknown` if autonomous planning must remain only in
observed free space.

The controller is Regulated Pure Pursuit:

| Setting | Value |
| --- | --- |
| Desired linear velocity | `0.3 m/s` |
| Lookahead distance | `0.2 m` |
| Collision detection | enabled |
| Maximum collision lookahead | `1.0 s` |
| Rotate to heading | enabled |
| Reversing | disabled |
| Cost-regulated velocity scaling | disabled |

The controller's collision checks depend on the local costmap and configured
footprint. A clean global plan does not make an unsafe or stale local costmap
acceptable.

## Static And Dynamic Environment Assumptions

SLAM assumes the environment is mostly static. Persistent walls, furniture, and
other fixed geometry should dominate the map. Moving people, doors, chairs, or
other objects can create temporary scan inconsistencies and occupancy artifacts.

The live costmap obstacle layers can respond to moving obstacles, but they are
not object trackers. They mark and clear scan geometry; they do not estimate
velocity, identity, or future motion.

A dynamic obstacle can therefore:

- temporarily enter both costmaps;
- be incorporated into the SLAM map if observed repeatedly;
- leave stale occupancy if clearing rays are unavailable;
- invalidate a path between planner updates.

Do not interpret the map or costmap as proof that a moving object is stationary.

## Startup

Preferred full pipeline:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

Layer-by-layer debugging:

```bash
ros2 launch yahboomcar_bringup muto_hardware_launch.py
ros2 launch tf2_publisher all_tf2_publishers_launch.py
ros2 launch yahboomcar_bringup ekf_imu_lidar_launch.py
ros2 launch muto_slam_mapping online_async_mapping_launch.py
ros2 launch muto_slam_mapping nav2_planner_controller_launch.py
```

`online_async_mapping_launch.py` owns scan fusion by default. Do not launch
`camera_depth_to_laserscan_launch.py` separately at the same time unless
mapping is started with `launch_fused_laserscan:=false`.

## Runtime Checks

Mapping inputs and TF:

```bash
ros2 topic hz /fused/laserscan
ros2 topic echo /fused/laserscan --once
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_frame
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_frame
```

Costmap topics:

```bash
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 topic echo /local_costmap/published_footprint --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
```

Lifecycle state:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

Discover the exact namespaced costmap nodes before inspecting parameters:

```bash
ros2 node list
ros2 param list /local_costmap/local_costmap
ros2 param list /global_costmap/global_costmap
```

RViz should use `map` as the fixed frame when inspecting the global costmap,
SLAM map, and complete robot path. Useful displays are:

- Map on `/map`;
- LaserScan on `/fused/laserscan`;
- Map on `/local_costmap/costmap`;
- Map on `/global_costmap/costmap`;
- TF;
- robot footprint and planned path.

## Failure Patterns

### Map exists but global costmap is empty

Check:

1. `planner_server` is active.
2. The static layer receives `/map`.
3. `map_subscribe_transient_local` remains enabled.
4. `map -> base_frame` resolves.
5. Global costmap and map resolutions are both `0.04`.

### Local costmap is empty or frozen

Check:

1. `controller_server` is active.
2. `/fused/laserscan` is live near the expected rate.
3. The fused scan frame transforms to `odom`.
4. `odom -> base_frame` is continuous.
5. Observation-source warnings are not reporting missed update deadlines.

### Obstacles never clear

Check:

1. The fused scan contains finite rays through newly free space.
2. `clearing: true` and `footprint_clearing_enabled: true`.
3. The obstacle lies within the 3 m raytrace range.
4. TF and timestamps allow the clearing observation to enter the layer.
5. Camera or LiDAR occlusion is not preventing a free-space ray.

### Robot marks itself

Check:

1. Static sensor transforms and camera projection geometry.
2. `camera_min_x`, z limits, and minimum ranges.
3. The published footprint and real body dimensions.
4. Whether leg or sensor returns fall outside the configured 0.16 m radius.
5. Whether footprint clearing removes only the modeled body while leaving leg
   returns as obstacles.

### Planner enters unknown space

This is expected with `allow_unknown: true`. Set it to `false` only after
deciding that navigation should be restricted to observed free space.

### Paths pass too close to obstacles

Check the physical footprint first. Then tune:

1. `robot_radius` and `footprint_padding`;
2. `inflation_radius`;
3. `cost_scaling_factor`;
4. planner/controller behavior.

Increasing inflation while leaving an undersized footprint does not make the
collision model physically correct.

### CPU load is too high on aarch64

Measure before changing behavior. The main cost drivers are:

- 0.04 m SLAM and costmap resolution;
- full-costmap publication;
- local 5 Hz updates;
- planner expected at 20 Hz;
- depth conversion rate and 4x4 sampling;
- fused scan density.

Reduce visualization/publication load first. Then consider update rates, depth
conversion rate, or resolution while checking navigation behavior after every
change.

## Tuning Order

1. Validate sensor extrinsics and all three TF layers.
2. Validate LiDAR-only odometry and `odom -> base_frame`.
3. Validate the fused scan geometry in `base_frame`.
4. Validate the SLAM map before starting Nav2.
5. Validate the real robot footprint, including the active gait envelope.
6. Tune obstacle marking and clearing ranges.
7. Tune inflation.
8. Tune planner and controller behavior.
9. Profile CPU and timing on the target.

Changing costmap parameters cannot repair stale timestamps, invalid TF,
incorrect camera projection, or discontinuous odometry.

## Key Files

| File | Role |
| --- | --- |
| `src/muto_slam_mapping/config/mapper_params_online_async.yaml` | SLAM frames, resolution, scan topic, and map timing. |
| `src/muto_slam_mapping/config/nav2_params.yaml` | Costmaps, planner, controller, behaviors, and BT navigator parameters. |
| `src/muto_slam_mapping/launch/online_async_mapping_launch.py` | Scan fusion plus online mapping. |
| `src/muto_slam_mapping/launch/nav2_planner_controller_launch.py` | Nav2 server and lifecycle ownership. |
| `src/muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Readiness-gated full startup. |
| `src/lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Raw depth conversion and scan fusion. |
| `src/lidar_pointcloud_filter/launch/filter_lidar_odometry_launch.py` | LiDAR scans and RF2O input. |
