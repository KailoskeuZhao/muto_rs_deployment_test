# TF2 Frames And Ownership

Date: 2026-07-28

This document defines the active TF2 contract for the Muto RS ROS 2 Humble
workspace. It covers frame names, transform ownership, static calibration,
timestamp behavior, startup readiness, and debugging.

For the complete startup sequence, see [slam_pipeline.md](slam_pipeline.md).

## Non-Negotiable Rules

1. The deployed robot frame is `base_frame`. The reference-only
   `yahboomcar_description` URDF uses `base_link`, but no active launch starts
   its robot-state publisher or introduces that frame into the deployed tree.
2. Every child frame must have one parent in the active TF tree.
3. Every dynamic transform must have one authoritative publisher.
4. Sensor data must be transformed at a timestamp compatible with its header.
5. Do not publish an identity transform to hide a missing localization layer.
6. Do not restamp sensor data merely to hide a clock or TF problem.

Violating these rules can produce plausible-looking topics with geometrically
incorrect data.

## Direction Notation

This document uses two related notations.

Tree ownership:

```text
parent -> child
```

For example, `base_frame -> lidar_frame` means `base_frame` is the parent
and `lidar_frame` is the child published in TF.

Lookup direction:

```text
target <- source
```

A lookup asks TF2 for coordinates expressed in `source` to be represented in
`target`:

```text
lookup_transform(target_frame, source_frame, stamp)
```

The Buffer API order is target then source. The `tf2_echo` CLI retains the
historical argument labels `source_frame target_frame`; its help notes that the
returned transform converts target-frame data into source-frame coordinates.

To inspect `base_frame <- lidar_frame`, run:

```bash
ros2 run tf2_ros tf2_echo base_frame lidar_frame
```

Read the first argument as the frame in which the second frame should be
expressed. The underlying tree edge remains `base_frame -> lidar_frame`.

## Current Frame Tree

During normal online mapping:

```text
map
  -> odom
    -> base_frame
      -> lidar_frame
      -> imu_link
      -> camera_link
        -> camera optical and stream frames
```

Important qualifications:

- `map` exists only when SLAM or another map-localization source is active.
- `odom` remains locally continuous but can drift.
- `map -> odom` corrects that local drift against the map.
- The Orbbec driver owns the internal camera-frame tree below `camera_link`.
- Raw IMU inspection messages use `raw_imu_link`. The active navigation tree
  and processed EKF input use `imu_link`.
- No active package should introduce `base_link`.
- `src/yahboomcar_description` is copied Yahboom tutorial reference material,
  not an active TF source.

## Transform Ownership

| Transform | Normal owner | Behavior |
| --- | --- | --- |
| `map -> odom` | SLAM Toolbox | Dynamic transform while online mapping is active. |
| `odom -> base_frame` | `robot_localization/ekf_node` | Dynamic local pose in the normal pipeline. |
| `base_frame -> lidar_frame` | `tf2_publisher/base_to_lidar_publisher` | Static hard-coded sensor mount. |
| `base_frame -> camera_link` | `tf2_publisher/base_to_camera_publisher` | Static hard-coded camera-body mount. |
| `base_frame -> imu_link` | `tf2_publisher/base_to_imu_publisher` | Static hard-coded IMU mount. |
| `camera_link -> camera optical frames` | Orbbec driver | Driver-owned static/internal camera tree. |

Consumers such as Nav2, depth projection, SAM2, and the object registry query TF;
they do not own these tree edges.

There is no current `publish_map_to_odom_tf` launch argument and no identity
`map -> odom` helper. SLAM Toolbox owns that transform during mapping.

## Static Sensor Calibration

The local static transforms are hard-coded in C++:

| Parent -> child | Translation xyz, metres | RPY, radians |
| --- | --- | --- |
| `base_frame -> lidar_frame` | `(-0.02, 0.0, 0.0)` | `(0.0, -3.1415, 0.20)` |
| `base_frame -> camera_link` | `(0.13, 0.0, 0.115)` | `(0.0, 0.18325, 0.0)` |
| `base_frame -> imu_link` | `(0.07, 0.0, 0.0)` | `(0.0, 0.0, 0.0)` |

Start them directly with:

```bash
ros2 launch tf2_publisher all_tf2_publishers_launch.py
```

They are published through `StaticTransformBroadcaster` on `/tf_static`.
Static transforms are transient-local, so late subscribers can receive them.

These calibration values are deployment-specific and currently require a source
change and rebuild. Verify physical translation, axis direction, and rotation on
the robot before trusting filtered scans, odometry, maps, or costmaps.

## Optional Odometry Republisher

`all_tf2_publishers_launch.py` has:

```text
publish_odom_tf:=false
odom_topic:=scan_odom
```

When explicitly enabled, `tf2_publisher/odom_publisher` copies an Odometry
message into TF using:

- the message timestamp;
- `header.frame_id` as the parent;
- `child_frame_id` as the child;
- the message pose as the transform.

It does not choose or repair frame names. It is disabled by default and must
remain disabled while the EKF or standalone odometry wrapper publishes the same
dynamic edge.

## Normal Dynamic Ownership

The normal localization path is:

```text
/lidar/filtered_laserscan
  -> RF2O
  -> /scan_odom_raw
  -> odometry deadband/jump wrapper
  -> /scan_odom
  -> robot_localization EKF <- /imu/data_processed
  -> /odometry/filtered
  -> odom -> base_frame
```

The EKF configuration uses:

```text
map_frame: map
odom_frame: odom
base_link_frame: base_frame
world_frame: odom
publish_tf: true
```

Because `world_frame` is `odom`, the EKF publishes
`odom -> base_frame`. Setting `map_frame` does not make this EKF own
`map -> odom`.

The normal EKF launch forces `rf2o_publish_tf:=false`. Foot odometry, when
enabled, also publishes no TF.

## Standalone LiDAR Odometry

For RF2O testing without the EKF:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py
```

RF2O itself has TF publication disabled by the wrapper launch. The downstream
odometry guard publishes `odom -> base_frame` by default in this standalone
mode.

Before starting the EKF, stop the standalone launch or use:

```bash
ros2 launch lidar_pointcloud_filter filter_lidar_odometry_launch.py \
  rf2o_publish_tf:=false
```

Never run the standalone TF owner and EKF TF owner together.

## SLAM And Nav2

SLAM Toolbox is configured with:

```text
map_frame: map
odom_frame: odom
base_frame: base_frame
scan_topic: /lidar/filtered_laserscan_no_downsample
```

It consumes the existing `odom -> base_frame` transform and publishes
`map -> odom` while mapping.

Nav2 does not replace either transform. Its local costmap consumes
`odom -> base_frame`; its global costmap and BT navigator require a complete
`map -> odom -> base_frame` chain.

## Topic And Frame Contract

| Product | Expected header frames |
| --- | --- |
| `/lidar/raw_laserscan` | `lidar_frame`. |
| `/lidar/filtered_laserscan` | Preserves the LiDAR scan frame. |
| `/lidar/filtered_laserscan_no_downsample` | Preserves the LiDAR scan frame. |
| `/scan_odom_raw` | Parent `odom`, child `base_frame`. |
| `/scan_odom` | Parent `odom`, child `base_frame`. |
| `/odometry/filtered` | Parent `odom`, child `base_frame`. |
| `/imu/data_processed` | `imu_link`. |
| `/imu/data_raw` and `/imu/mag_raw` | `raw_imu_link`; inspection data, not the active EKF frame. |
| `/camera/depth/image_raw` | Normally a depth optical frame from the Orbbec driver. |
| `/camera/depth/camera_info` | Must describe the depth profile and frame. |
| `/camera/filtered_laserscan` | `base_frame` by default. |
| `/map` | `map`. |
| SAM2 image and mask outputs | Preserve the color image frame. |
| `/sam2/instance_pointcloud` | Depth optical frame and depth timestamp. |
| Stored perception positions | Registry target frame, normally `map`. |

Always inspect the actual message header. Driver profiles and remappings can
change camera optical frame names.

## Lookup Behavior By Component

### Readiness gates

The top-level pipeline checks latest transform availability before launching
each downstream stage:

| Stage | Required TF lookup |
| --- | --- |
| Localization | `base_frame <- lidar_frame` and `base_frame <- imu_link`. |
| Mapping | `odom <- base_frame`. |
| Nav2 | `map <- base_frame`. |

The gate proves that a chain currently exists. It does not prove that every
historical sensor timestamp can be transformed.

### RF2O

On its first scan, RF2O requests the latest
`base_frame <- <scan frame>` transform and stores the LiDAR mounting pose.
This should resolve through the static `base_frame -> lidar_frame` edge.

### Depth-image to LaserScan

The depth converter requests:

```text
base_frame <- depth_image_frame
```

at the depth-image timestamp. If that fails, it tries the latest transform.
The fallback is intended to tolerate static camera-extrinsic timing. Do not rely
on latest-transform fallback to conceal a missing or stale dynamic
`map/odom/base_frame` relationship.

The output camera scan keeps the depth-image timestamp but changes its frame to
the configured processing frame, normally `base_frame`.

### Nav2 obstacle sources

The LiDAR costmap source preserves `lidar_frame`. Nav2 transforms it through the
static lookup:

```text
base_frame <- lidar_frame
```

The camera converter requests `base_frame <- depth_image_frame` at the image
timestamp and publishes the resulting narrow scan in `base_frame`. The two
sources remain separate; there is no output-frame lookup for a combined scan.

### SLAM And Nav2

SLAM and Nav2 consume dynamic transforms at sensor/message times. Continuous
`odom -> base_frame` and `map -> odom` history matters. A transform that
exists only at the latest time can still cause message-filter drops or
extrapolation errors for older scans.

### SAM2 depth projection

The image annotator requests:

```text
color_optical_frame <- depth_optical_frame
```

at the depth timestamp and then permits a latest-transform fallback. These are
normally fixed camera extrinsics.

The published instance cloud remains in the depth frame. Transforming points
for color projection does not change the cloud's declared frame.

### Stored perception positions

The registry requests:

```text
target_frame <- instance_cloud_frame
```

at the cloud timestamp, where `target_frame` normally defaults to `map`.
It does not fall back to the latest transform. If the timestamped chain is not
ready, the registry retains the already-paired cloud and detection messages for
up to `tf_retry_window` (1 second by default) and retries that same historical
timestamp at `tf_retry_rate` (20 Hz by default). The queue is bounded by
`sync_queue_size`; the observation is rejected if its exact transform remains
unavailable when the retry window expires.

Persisted positions assume objects are static in the target frame. TF only
places an observation in that frame; it does not detect or compensate for
object motion.

### Object command layer

The go-to-object server receives a registry centroid already expressed in the
registry target frame, normally `map`. If that differs from its configured
Nav2 global frame, it transforms the stored point using the latest available
frame relationship; a persistent object has no single live sensor timestamp.
It also requests the latest:

```text
global_frame <- base_frame
```

to seed reachability from the robot's current cell. The command requests
Nav2's master global costmap, uses its static, obstacle, footprint, and
inflation result directly, searches outward around the object for a reachable
cell at or beyond the configured radius, and orients that cell toward the
centroid. TF does not choose the approach side. The command publishes no TF
edge. Object search and the VLM socket use registry metadata/images and do not
query TF.

## Pipeline Startup

Preferred startup:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py
```

The launch starts hardware immediately, static sensor TF after a minimum
one-second delay, then uses topic and TF gates before localization, mapping, and
Nav2.

The relevant TF checks are:

```text
localization: base_frame <- lidar_frame
              base_frame <- imu_link

mapping:      odom <- base_frame

Nav2:         map <- base_frame
```

A gate timeout shuts down the pipeline rather than launching the next layer
with an incomplete tree.

The perception and command nodes are deliberately outside that launch. Start
them separately when needed:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer object_pipeline_launch.py
```

That launch consumes the existing camera optical frames and, for registry and
navigation functions, the existing `map -> odom -> base_frame` chain. It does
not create a second TF tree. Frontier exploration is also a separate Nav2
client and only consumes `map <- base_frame`.

## Basic Verification

Start with the expected static edges:

```bash
ros2 run tf2_ros tf2_echo base_frame lidar_frame
ros2 run tf2_ros tf2_echo base_frame camera_link
ros2 run tf2_ros tf2_echo base_frame imu_link
```

Check the camera chain using the actual frame from CameraInfo:

```bash
ros2 topic echo /camera/depth/camera_info --once
ros2 run tf2_ros tf2_echo base_frame camera_depth_optical_frame
```

Check dynamic localization and mapping:

```bash
ros2 run tf2_ros tf2_echo odom base_frame
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_frame
```

Inspect TF topics and publishers:

```bash
ros2 topic echo /tf_static --once
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

Generate a tree snapshot when `tf2_tools` is installed:

```bash
ros2 run tf2_tools view_frames
```

Open the generated `frames.pdf` and verify that each child appears once under
the expected parent.

## Message Header Checks

Verify frame and timestamp fields before blaming TF:

```bash
ros2 topic echo /lidar/raw_laserscan --once
ros2 topic echo /imu/data_processed --once
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /lidar/filtered_laserscan_no_downsample --once
ros2 topic echo /camera/filtered_laserscan --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo /map --once
```

For every sensor-processing path, verify:

1. The topic is live.
2. `header.frame_id` is nonempty and expected.
3. The timestamp uses the same clock as TF publishers.
4. `target <- source` resolves at that timestamp.
5. The output frame matches the node's configured processing frame.

## Duplicate-Publisher Audit

The highest-risk duplicate is `odom -> base_frame`.

Normal EKF mode:

| Possible publisher | Expected state |
| --- | --- |
| EKF | Enabled and authoritative. |
| RF2O node | TF disabled. |
| Odometry deadband wrapper | TF disabled by the EKF parent launch. |
| `tf2_publisher/odom_publisher` | Disabled. |
| Foot odometry | TF disabled. |

Standalone LiDAR odometry mode:

| Possible publisher | Expected state |
| --- | --- |
| Odometry deadband wrapper | Enabled and authoritative. |
| EKF | Not running. |
| `tf2_publisher/odom_publisher` | Disabled. |

For `map -> odom`, SLAM Toolbox is the only normal owner during online
mapping. Do not add an identity publisher beside it.

Useful audit commands:

```bash
ros2 node list
ros2 topic info /tf --verbose
ros2 topic echo /tf
```

If the same child alternates between poses, jumps while stationary, or appears
under multiple parents in `view_frames`, stop the duplicate publisher before
tuning any filter.

## Common Failures

### Invalid frame ID

Typical causes:

- the expected publisher is not running;
- the driver uses a different frame name;
- the Orbbec internal frame tree has not started;
- a message has an empty `frame_id`;
- an old launch or remapping still expects a removed frame.

Check the exact message header and `view_frames`.

### Extrapolation into the future

The sensor timestamp is newer than available dynamic TF. Check clock
synchronization, `use_sim_time`, publication latency, and whether the dynamic
publisher is still running.

### Extrapolation into the past

The message is older than the TF buffer history or processing is backlogged.
Check stale inputs, queue growth, CPU load, and inappropriate restamping.

### Lookup works with latest time but processing still drops messages

`tf2_echo` normally shows the current transform. The failing node may need the
transform at an older sensor timestamp. Inspect the message stamp and dynamic TF
history rather than assuming the current lookup proves timestamp compatibility.

### Camera obstacle scan missing

Check:

1. `camera_depth_to_laserscan_node` is running.
2. The camera depth frame resolves to `base_frame`.
3. Depth encoding and CameraInfo dimensions are valid.
4. Sensor stamps are not rejected as stale.
5. Only one converter owns `/camera/filtered_laserscan`.

Missing camera TF stops camera scan publication but does not stop the LiDAR
costmap source or SLAM Toolbox.

### Map missing or not updating

Check:

1. `/lidar/filtered_laserscan_no_downsample` is live and its `lidar_frame`
   resolves to `base_frame`.
2. `odom <- base_frame` exists continuously.
3. SLAM Toolbox is running with `map_frame=map`.
4. No other node publishes `map -> odom`.
5. Scan and odometry timestamps are compatible.

### Nav2 transform timeout

Check the complete timestamped chain:

```text
map -> odom -> base_frame -> sensor frame
```

A latest `map <- base_frame` lookup can succeed while a costmap still rejects
an older scan. Inspect continuous message-filter warnings and sensor stamps.

### Geometry looks rotated or mirrored

A connected TF tree can still contain bad calibration. Inspect the hard-coded
RPY values, sensor axis conventions, and a known physical target in RViz.
Connectivity does not prove geometric correctness.

## Simulated Time And Clocks

Dynamic publishers and sensor sources must use the same clock. If some nodes
use `/clock` and others use wall time, timestamped lookups fail even when frame
names and topology are correct.

The main pipeline propagates `use_sim_time` through localization, mapping, and
Nav2. Static transforms are time-independent after publication, but dynamic
odometry and map transforms are not.

## Removed Legacy Assumptions

The active TF contract does not use:

- `base_link`;
- `/lidar/PointCloud` or filtered PointCloud topics;
- `/camera/depth/points`;
- `camera_link` as the depth-scan output frame;
- an identity `map -> odom` launch helper;
- `publish_map_to_odom_tf`.

Current camera projection uses raw `16UC1` depth and `base_frame` as its
processing/output frame. SLAM uses the TG30 no-downsample LaserScan directly.

## Key Files

| File | Role |
| --- | --- |
| `src/tf2_publisher/launch/all_tf2_publishers_launch.py` | Static sensor TF and disabled optional odometry republisher. |
| `src/tf2_publisher/src/base_to_lidar_publisher.cpp` | LiDAR mount calibration. |
| `src/tf2_publisher/src/base_to_camera_publisher.cpp` | Camera-body mount calibration. |
| `src/tf2_publisher/src/base_to_imu_publisher.cpp` | IMU mount calibration. |
| `src/tf2_publisher/src/odom_publisher.cpp` | Optional Odometry-to-TF relay. |
| `src/yahboomcar_bringup/config/ekf_lidar_imu.yaml` | Normal dynamic odom TF owner. |
| `src/muto_slam_mapping/config/mapper_params_online_async.yaml` | SLAM frame contract. |
| `src/muto_slam_mapping/launch/muto_nav2_pipeline_launch.py` | Topic and TF readiness gates. |
| `src/muto_command_layer/launch/object_pipeline_launch.py` | Independent object consumers of camera and map TF. |
| `src/lidar_pointcloud_filter/src/camera_depth_to_laserscan_node.cpp` | Depth-frame lookup and output-frame behavior. |
| `src/yahboomcar_description/README.md` | Boundary between reference `base_link` URDF data and deployed `base_frame` TF. |
