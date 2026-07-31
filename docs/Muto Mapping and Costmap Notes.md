# Muto Mapping and Costmap Notes

Date: 2026-07-30

This document describes the active ROS 2 Humble mapping and Nav2 costmap path
for the aarch64 Muto deployment. It is the current runtime contract.

## System View

```text
/lidar/raw_laserscan
  -> lidar_laserscan_filter_node
       -> /lidar/filtered_laserscan
       |    -> RF2O
       |    -> Nav2 local obstacle layer: source lidar
       |    -> Nav2 global obstacle layer: source lidar
       |
       -> /lidar/filtered_laserscan_no_downsample
            -> SLAM Toolbox
                 -> /map
                 -> map -> odom

/camera/depth/image_raw + /camera/depth/camera_info
  -> camera_depth_to_laserscan_node
       -> /camera/filtered_laserscan
            -> Nav2 local obstacle layer: source camera
            -> Nav2 global obstacle layer: source camera
```

LiDAR and camera observations are not merged. Nav2's `ObstacleLayer` consumes
them as separate named sources. SLAM consumes only the stable full-resolution
LiDAR topic, so camera latency, loss, or geometry cannot change SLAM scan
metadata.

The complete dynamic TF path is:

```text
map -> odom -> base_frame -> sensor frames
```

| Transform | Owner |
| --- | --- |
| `map -> odom` | SLAM Toolbox |
| `odom -> base_frame` | `robot_localization/ekf_node` |
| `base_frame -> lidar_frame` | Static TF publisher |
| `base_frame -> camera_link` | Static TF publisher |
| Camera internal optical transforms | Orbbec driver |

Do not enable RF2O TF output or the optional odometry-to-TF republisher while
the EKF owns `odom -> base_frame`.

## Scan Ownership

### LiDAR for SLAM

`/lidar/filtered_laserscan_no_downsample` is the SLAM Toolbox input. Its active
mapping parameter is:

```yaml
scan_topic: /lidar/filtered_laserscan_no_downsample
```

This topic preserves the TG30 angular density and has a 15 m filter cap. Camera
data never enters SLAM Toolbox.

### LiDAR for costmaps

`/lidar/filtered_laserscan` is downsampled by a factor of two and capped at 10 m.
It already exists for RF2O, so the costmaps reuse it instead of processing the
higher-density topic. Nav2 applies its own shorter obstacle and raytrace ranges.

This source is required for normal navigation:

| Parameter | Value |
| --- | --- |
| Source name | `lidar` |
| Topic | `/lidar/filtered_laserscan` |
| Type | `LaserScan` |
| Marking / clearing | `true / true` |
| Obstacle range | `0.0-2.5 m` |
| Raytrace range | `0.0-3.0 m` |
| `expected_update_rate` | `0.2 s` |
| `inf_is_valid` | `true` |

Positive-infinity LiDAR returns are valid clearing observations. The 0.2 s
expectation allows roughly three nominal 16 Hz scan periods before Nav2 reports
the source stale.

### Camera for costmaps

`camera_depth_to_laserscan_node` converts sampled `16UC1` depth pixels using
CameraInfo and TF. It publishes only `/camera/filtered_laserscan`; LiDAR is not an input to the
component.

| Parameter | Value |
| --- | --- |
| Source name | `camera` |
| Topic | `/camera/filtered_laserscan` |
| Type | `LaserScan` |
| Output frame | `base_frame` |
| Horizontal FOV | 58.4 degrees (`-29.2` to `29.2`) |
| Vertical FOV | 45.5 degrees |
| Angular increment | approximately 1 degree; exact FOV endpoints are preserved |
| Camera range | `0.30-3.0 m` |
| Obstacle range in Nav2 | `0.30-2.5 m` |
| Raytrace range in Nav2 | `0.30-3.0 m` |
| Height slice in `base_frame` | `-0.07` to `0.18 m` |
| Maximum conversion rate | `7 Hz` |
| Pixel stride | `4 x 4` |
| Marking / clearing | `true / true` |
| `inf_is_valid` | `false` |

The lower height bound excludes the nominal floor, which is around `z=-0.094 m`
for the current standby geometry. Validate that threshold on the physical robot.

Unobserved, invalid, and filtered camera bins are NaN. With
`inf_is_valid: false`, Nav2 ignores those bins instead of clearing space outside
the camera's measured sector. Finite camera observations still mark obstacles
and clear the ray up to the observation.

The camera source intentionally has no nonzero `expected_update_rate`. It is an
optional enhancement: loss of camera data reduces forward obstacle coverage but
does not make the costmap non-current or interrupt LiDAR-based navigation.

## Why Sensors Stay Separate

Publishing alternating partial scans on one topic is not a valid replacement for multiple
Nav2 observation sources. Consumers can cache scan geometry by frame, and two
messages with different angular extents under one frame/topic are not one stable
sensor contract. Time-separated front and rear messages also produce clearing
and staleness behavior that depends on callback ordering.

The standard Nav2 model already solves this problem:

```yaml
observation_sources: lidar camera
lidar:
  topic: /lidar/filtered_laserscan
camera:
  topic: /camera/filtered_laserscan
```

Each consumer therefore receives one stable sensor contract: SLAM receives the
full-resolution LiDAR topic, while Nav2 receives two named observation sources.

## Mapping Ownership

`online_async_mapping_launch.py` owns one action: launch the installed SLAM
Toolbox `online_async_launch.py`. It does not own camera preprocessing.

The top-level pipeline starts camera depth-to-scan independently when
`launch_camera_obstacle_scan:=true`. In a layer-by-layer startup, use:

```bash
ros2 launch lidar_pointcloud_filter camera_depth_to_laserscan_launch.py
ros2 launch muto_slam_mapping online_async_mapping_launch.py
```

Omitting the camera command does not change SLAM; it still uses
`/lidar/filtered_laserscan_no_downsample`.

Current SLAM settings include:

| Parameter | Value |
| --- | --- |
| Mode | `mapping` |
| `map_frame` | `map` |
| `odom_frame` | `odom` |
| `base_frame` | `base_frame` |
| Resolution | `0.04 m/cell` |
| Scan queue | `1` |
| TF publish period | `0.05 s` |
| TF timeout | `0.5 s` |
| Map update interval | `3.0 s` |

Online mapping creates or extends a map for the current run. This launch is not
a saved-map localization workflow.

## Local Costmap

The local costmap is a rolling obstacle view used by the controller and recovery
behaviors.

| Setting | Value |
| --- | --- |
| Global frame | `odom` |
| Robot frame | `base_frame` |
| Size | `3 x 3 m` |
| Resolution | `0.04 m/cell` |
| Update / publish | `5 / 2 Hz` |
| Plugins | obstacle, inflation |
| Robot radius | `0.16 m` |
| Inflation radius | `0.40 m` |
| Cost scaling factor | `12.0` |
| Transform tolerance | `0.2 s` |

Both `lidar` and `camera` are declared under the same obstacle layer. They update
the same rolling grid but retain independent topics, timestamps, FOVs, and
valid-return semantics.

## Global Costmap

The global costmap combines the SLAM map with current sensor observations.

| Setting | Value |
| --- | --- |
| Global frame | `map` |
| Robot frame | `base_frame` |
| Resolution | `0.04 m/cell` |
| Update / publish | `1 / 1 Hz` |
| Track unknown | `true` |
| Plugins | static, obstacle, inflation |
| Robot radius | `0.16 m` |
| Inflation radius | `0.40 m` |
| Transform tolerance | `0.2 s` |

The static layer consumes `/map`. The obstacle layer adds current LiDAR and
camera observations. The inflation layer expands lethal costs for planning.

## Footprint Assumption

The configured `robot_radius: 0.16` models the central body and fixed sensor
package. It does not cover the complete swept leg envelope. The reference
zero-pose leg collision radius is roughly `0.30 m`, and walking can change the
shape further.

This is a deployment risk, not a cosmetic tuning choice. Before autonomous
navigation near narrow gaps:

1. Measure the maximum swept gait envelope on the physical robot.
2. Decide whether a conservative radius or a polygon footprint is appropriate.
3. Validate forward, turning, backup, and recovery motions against that shape.
4. Retune inflation only after the collision footprint is defensible.

## Static And Dynamic Assumptions

SLAM assumes the dominant environment geometry is static. Moving people,
carried objects, doors, and furniture can create transient scan matches and map
artifacts. Nav2 obstacle layers are intentionally dynamic and should represent
current hazards without writing every transient camera return into the map.

That separation is another reason camera depth belongs in Nav2 costmaps instead
of the SLAM scan.

## Startup

Normal startup:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

The relevant readiness gates are:

| Stage | Required state |
| --- | --- |
| Mapping | `/odometry/filtered`, `/lidar/filtered_laserscan_no_downsample`, `odom <- base_frame` |
| Nav2 | `/map`, `/lidar/filtered_laserscan`, `map <- base_frame` |

Camera depth is optional and is not a readiness prerequisite. The top-level
camera controls are:

```text
launch_camera_obstacle_scan:=true
camera_scan_max_publish_rate:=7.0
```

## Runtime Checks

Check the two LiDAR products and camera source separately:

```bash
ros2 topic hz /lidar/filtered_laserscan
ros2 topic hz /lidar/filtered_laserscan_no_downsample
ros2 topic hz /camera/filtered_laserscan
ros2 topic echo /camera/filtered_laserscan --once
```

The camera message should report:

- `header.frame_id: base_frame`;
- angles near `-0.5096` to `0.5096` radians;
- about 59 bins at one-degree resolution;
- NaN in unobserved bins, not positive infinity.

Check map and TF:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo odom base_frame
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_frame
```

Inspect costmap subscriptions and lifecycle state:

```bash
ros2 topic info /lidar/filtered_laserscan --verbose
ros2 topic info /camera/filtered_laserscan --verbose
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

## Failure Patterns

### Camera scan is absent

Verify the depth image, matching CameraInfo, `16UC1` encoding, camera optical TF,
and `max_input_age`. Navigation should still have LiDAR observations. An absent
camera topic is a degraded-coverage condition, not a SLAM failure.

### Camera marks the floor

Inspect the camera scan in RViz and tighten `min_z` only after checking the real
camera mount and standby height. Do not hide a bad camera transform with a very
narrow height slice.

### LiDAR source is stale

Check `/lidar/filtered_laserscan` frequency and timestamps first. This source has
a 0.2 s update expectation and can make the obstacle layer non-current when it
stops.

### Map is absent

Check `/lidar/filtered_laserscan_no_downsample`, `odom -> base_frame`, and SLAM
Toolbox state. Camera scan availability is irrelevant to map production.

### Obstacles do not clear

Confirm LiDAR positive-infinity returns survive filtering and that
`inf_is_valid: true` remains set only for LiDAR. Camera NaNs are deliberately not
clearing rays. Also check TF at each scan timestamp and verify the sensor is not
seeing the robot's own body or legs.

### CPU load is high on aarch64

Measure before changing behavior. In order:

1. Keep camera conversion at `7 Hz`.
2. Keep the `4 x 4` nearest-depth block stride.
3. Keep the downsampled LiDAR topic on Nav2 costmaps.
4. Reduce visualization and debug publishers.
5. Only then consider lower costmap update rates or coarser resolution.

Do not downsample the SLAM topic without retesting map quality and scan matching.

## Tuning Order

1. Verify static camera and LiDAR transforms.
2. Verify sensor timestamps and TF availability at those timestamps.
3. Verify LiDAR filtering and RF2O/EKF stability.
4. Validate camera FOV, height slice, self-filtering, and NaN output.
5. Validate SLAM using LiDAR only.
6. Validate each Nav2 obstacle source independently in RViz.
7. Validate the physical footprint and gait envelope.
8. Tune obstacle ranges, inflation, planner, and controller.
9. Profile CPU and memory on the target aarch64 system.

## Key Files

| File | Role |
| --- | --- |
| `src/muto_slam_mapping/config/mapper_params_online_async.yaml` | SLAM LiDAR topic, frames, and map timing. |
| `src/muto_slam_mapping/config/nav2_params.yaml` | Local/global costmaps and named observation sources. |
| `src/muto_slam_mapping/launch/online_async_mapping_launch.py` | SLAM Toolbox ownership only. |
| `src/muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Readiness-gated complete startup. |
| `src/lidar_pointcloud_filter/src/lidar_laserscan_filter_node.cpp` | LiDAR products for RF2O/Nav2 and SLAM. |
| `src/lidar_pointcloud_filter/src/camera_depth_to_laserscan_node.cpp` | Narrow camera scan generation. |
| `src/lidar_pointcloud_filter/launch/camera_depth_to_laserscan_launch.py` | Camera-only component launch. |
