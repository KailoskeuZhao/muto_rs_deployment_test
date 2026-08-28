# SAM2 + YOLO + Object Registry And Command Pipeline

Date: 2026-07-28

This document describes the complete perception-to-persistence path implemented
by `sam2_image_annotator` and `sam2_object_registry`. It covers YOLO
detection, SAM2 segmentation, RGB/depth geometry, the instance-marked point
cloud, TF2 centroid localization, temporal confirmation, indexed object storage,
RViz visualization, services, and YAML persistence. The v2 mission executive
uses this registry as an independent authority for shortlist lookup and visual
candidate confirmation; it does not add another object-command package.

## Static-Object Assumption

> **Critical:** Every detected object is assumed to be static in the world.
> The robot and its cameras may move, but an observed object is expected to
> remain at one fixed map-frame position during capture and across later
> observations.

The pipeline does not estimate object velocity, track moving objects, or
motion-compensate RGB/depth pairs. Registry association interprets repeated
same-label observations near one map position as measurements of the same
stationary object and averages them into a persistent position. Temporal
confirmation checks whether observations are repeated and spatially
consistent; it does **not** establish that an object is static.

Using this pipeline for people, vehicles, carried items, or furniture that is
moved can produce mask/depth misalignment, biased point clouds, drifting
averaged positions, duplicate records, or a persisted position that is no
longer valid.

## Data Flow

```text
/camera/color/image_raw ----> latest-RGB worker slot ----> YOLO
                                                        |
                                                        v
                                             confidence/box guard
                                                        |
                                                        v
                                             SAM2 box refinement
                                                        |
                         +------------------------------+----------------+
                         |                              |                |
                         v                              v                v
                 annotated image                 mask products    typed metadata
                                                        |                |
/camera/depth/image_raw --> bounded depth buffer        |                |
/depth CameraInfo ----------> depth back-projection     |                |
/color CameraInfo ----------> color projection <--------+                |
TF: color <- depth ---------> extrinsic transform                        |
                                  |                                      |
                                  v                                      |
                      /sam2/instance_pointcloud                           |
                                  |                                      |
                                  +--------------+-----------------------+
                                                 | timestamp pairing
                                                 v
                                      C++ object registry
                                                 |
                              per-instance centroid + TF: map <- cloud
                                                 |
                         +-----------------------+-----------------------+
                         |                                               |
               existing confirmed object                       tentative candidate
                         |                                               |
                  running update                    count/time/gap/confidence policy
                         |                                               |
                         +-----------------------+-----------------------+
                                                 | confirmed objects only
                         +-----------------------+-----------------------+
                         |                       |                       |
                         v                       v                       v
               /sam2/stored_objects     RViz MarkerArray       sam2_objects.yaml
                         |                                      + object JPEG
                         +---- image_path ------------------------------+
                                                services: query / save / clear
                         |
                         +-> v2 MissionExecutive <-> /vlm/generate
                               registry shortlist -> candidate confirmation
                               -> Nav2 /navigate_to_pose when confirmed
```

The Python annotator owns image inference and depth-to-mask projection. The C++
registry consumes the typed detections and instance cloud, estimates one
map-frame centroid per instance, rejects transient detections, and owns the
long-lived object database. A failure in the optional 3D branch does not stop
the annotated image, masks, or metadata outputs.

## Runtime Requirements

The annotator package builds without loading the inference stack because SAM2,
PyTorch, and Ultralytics are imported lazily. The registry is a normal C++ ROS
node. End-to-end runtime requires:

- ROS 2 Humble with this workspace built and sourced.
- SAM2 and a compatible CUDA-enabled PyTorch build for aarch64.
- Ultralytics and a compatible YOLO weights file.
- A SAM2 checkpoint matching `model_cfg`.
- Color images for 2D output.
- A `16UC1` depth image, camera calibration, and a depth-to-color TF for 3D
  output.
- A complete timestamped TF tree from the depth optical frame through the
  configured registry `target_frame`, normally `map`.
- The `sam2_object_registry` C++ package and yaml-cpp for registry and
  persistence features.
- For v2 mission planning and candidate confirmation: `muto_vlm_socket`, a
  reachable configured VLM endpoint, and the `HKU_API_KEY` environment
  variable used by the checked-in provider profile.
- For a v2 approach tool: an independently running Nav2
  `/navigate_to_pose` action and a current `map <- base_frame` transform.

Pre-stage both model files on the robot. A named Ultralytics model can trigger
a download when it is not cached.

The current hardware launch requests `640x480 @ 30 Hz` RGB and
`320x240 @ 30 Hz` depth, while this node caps processing starts at 7 Hz. It
loads the serial-specific 640x480 color calibration fallback from
`yahboomcar_bringup`; the depth driver currently supplies valid native
320x240 intrinsics. Color intrinsics are used for depth-point projection into
the RGB SAM mask, not for YOLO or SAM2's 2D inference itself. Invalid color
intrinsics therefore leave 2D outputs alive but block the instance-cloud and
registry observation path.

## Start

The v2 field composition starts the annotator, registry, VLM socket, mission
executive, deterministic POI-grid authority, Nav2, and high-level recorder:

```bash
source /opt/ros/humble/setup.bash
cd /opt/muto_rs_ws
source install/setup.bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer_v2 v2_hardware_smoke_launch.py
```

For a registry-only diagnostic, launch the annotator and registry directly as
described below. For a mission, use the v2 launch rather than composing the
retired action servers by hand.

For component debugging, perception and registry can still be started in
separate terminals:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py
ros2 launch sam2_object_registry object_registry_launch.py
```

Use absolute model paths for repeatable deployment:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  checkpoint:=/opt/sam2/checkpoints/sam2.1_hiera_base_plus.pt \
  yolo_model:=/opt/models/yolo26m.pt
```

The relative checkpoint is checked from the launch working directory and the
installed or editable SAM2 project root. The default model configuration is
`configs/sam2.1/sam2.1_hiera_b+.yaml`.

## Per-Frame Processing

1. The color callback replaces a single pending RGB slot. A dedicated worker
   always takes the newest pending image, so inference cannot build an unbounded
   RGB backlog. `max_publish_rate` limits processing start times; inference time is
   not followed by another cooldown.
2. The worker snapshots the cached color/depth `CameraInfo` messages and
   selects the nearest timestamp from a bounded recent depth deque. The 3D
   branch is blocked when the nearest depth exceeds `depth_sync_tolerance`.
3. The image is converted to BGR for YOLO and RGB for SAM2.
4. YOLO runs with its candidate confidence set to `0.0`. The node then applies
   `yolo_confidence`, rejects non-finite scores, clips boxes to the image, and
   drops invalid boxes.
5. SAM2 sets the RGB image once, then predicts one mask per accepted box. If
   multimask output is enabled, the highest-scoring mask is selected.
6. The node composes the overlay, union mask, instance mask, and metadata. Each
   typed detection also carries a JPEG crop of its original, unannotated YOLO
   box for possible registry promotion.
7. When valid depth inputs and TF are available, depth pixels are projected
   into the color mask to create the labeled point cloud.

Instance IDs start at `1` and restart on every processed frame. They are
frame-local IDs, not tracking IDs. If masks overlap, the earlier accepted
segment owns the overlap; later masks fill only pixels that are still zero.

## Topic Interface

Inputs:

| Default topic | Type | Purpose |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | YOLO and SAM2 color input. |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color calibration. |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Raw `16UC1` depth. |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth calibration. |
| TF | `tf2` | Depth optical frame to color optical frame transform. |

Outputs:

| Default topic | Type | Purpose |
| --- | --- | --- |
| `/sam2/annotated_image` | `sensor_msgs/msg/Image` | BGR boxes, labels, scores, and mask overlays. |
| `/sam2/mask` | `sensor_msgs/msg/Image` | `mono8` union of refined masks. |
| `/sam2/instance_mask` | `sensor_msgs/msg/Image` | `16UC1` frame-local instance IDs. |
| `/sam2/segments` | `std_msgs/msg/String` | JSON image and segment metadata. |
| `/sam2/detections` | `sam2_object_registry/msg/DetectedObjectArray` | Structured segment metadata and JPEG object crops. |
| `/sam2/instance_pointcloud` | `sensor_msgs/msg/PointCloud2` | Depth-frame XYZ, instance ID, and color. |

Image and metadata outputs retain the color image header. The point cloud uses
the depth image timestamp and depth optical frame. Sensor subscriptions are
best-effort and volatile; output publishers have queue depth `2`.

## Metadata

`/sam2/segments` contains the color header, image dimensions, and an
`objects` array. Each segment has:

| Field | Meaning |
| --- | --- |
| `instance_id` | Value used in the instance mask and point cloud. |
| `class_id` | YOLO class index. |
| `label` | YOLO class label. |
| `confidence` | YOLO confidence after the node guard. |
| `box_xyxy` | Clipped box in `[x1, y1, x2, y2]` order. |
| `sam_score` | Score of the selected SAM2 mask. |
| `mask_area` | Refined mask area in pixels. |

A typed `DetectedObject` additionally contains `crop`, a
`sensor_msgs/msg/CompressedImage` JPEG cut from the original color image at the
clipped YOLO box. The JSON `/sam2/segments` representation intentionally omits
the binary crop.

A no-detection frame publishes valid empty image-sized masks and empty
metadata.

## Instance Point Cloud

The 3D branch requires:

- Depth encoding exactly `16UC1`.
- Color/depth timestamp offset no greater than `depth_sync_tolerance`.
- `CameraInfo` dimensions matching the images.
- Positive focal lengths in both camera matrices.
- Nonempty depth and color optical frame IDs.
- A usable depth-to-color transform.

The selected depth frame may differ from the RGB timestamp by as much as
`depth_sync_tolerance`. The object is assumed not to move during that interval;
there is no per-object motion compensation before the RGB mask is applied to
the projected depth samples.

TF is first requested at the depth timestamp and then, if that fails, at the
latest available time:

```text
lookup_transform(color_frame, depth_frame, depth_stamp)
```

The mask is not resized or directly copied into the depth image. Sampling starts
from the depth grid independently of the mask:

1. Global row-major depth pixel indices are decimated by
   `pointcloud_stride`. A stride of 6 retains approximately one-sixth of the
   depth grid, not one-sixth along each image axis.
2. Zero depth values are discarded and `depth_scale` converts `uint16` units
   to metres.
3. `cv2.undistortPoints` applies depth intrinsics and distortion to obtain a
   normalized depth ray.
4. A sampled depth pixel `(u_d, v_d, Z)` becomes a depth-frame 3D point:

```text
X_d = normalized_x * Z
Y_d = normalized_y * Z
P_d = [X_d, Y_d, Z]
```

5. TF2 supplies the depth-to-color extrinsic transform:

```text
P_c = R_color_depth * P_d + t_color_depth
```

6. Color intrinsics and distortion project `P_c` into `(u_c, v_c)`.
7. The depth point is retained only when that color pixel contains a nonzero
   trimmed SAM instance ID.
8. When multiple sampled depth points land on one RGB pixel, a nearest-Z test
   retains the visible point.

This forward depth-to-color direction is intentional. A color pixel defines a
ray but cannot identify a unique depth pixel without a depth measurement.

Depth pixels are undistorted and back-projected in the depth frame. Points are
transformed for color-mask projection and nearest-Z visibility testing, while
published XYZ remains in the depth frame. Processing is split into 64-row
chunks to limit temporary allocations at high resolution.

Each mask is trimmed inward by `pointcloud_mask_trim_ratio` before 3D
selection. This reduces boundary depth contamination and does not alter the
published 2D mask. The stable depth grid is sampled using `pointcloud_stride`
before intrinsic and projection work.

The unorganized cloud has `height=1`, `point_step=20`, and these fields:

| Field | Offset | Type |
| --- | ---: | --- |
| `x` | 0 | `float32` |
| `y` | 4 | `float32` |
| `z` | 8 | `float32` |
| `instance_id` | 12 | `uint16` |
| `rgb` | 16 | `uint32` |

## Registry Input Pairing

The registry subscribes to two products from the same annotator result:

| Input | Role |
| --- | --- |
| `/sam2/detections` | Label, class, confidence, box, SAM score, mask area, and frame-local `instance_id`. |
| `/sam2/instance_pointcloud` | Depth-frame XYZ points carrying the same `instance_id`. |

Both subscriptions use best-effort QoS and bounded queues. The registry searches
the two queues for the pair with the smallest absolute timestamp difference and
accepts it only when the difference does not exceed
`metadata_sync_tolerance`. Consumed messages are removed from both queues.

The typed metadata path is authoritative for registry labels. The JSON
`/sam2/segments` topic is intended for inspection and is not parsed by the C++
registry. Manual SAM prompts publish no typed object metadata, so manual mode
can produce a mask and cloud but does not create registry entries.

## Centroid Extraction and Map Localization

For every accepted pair, the registry:

1. Repeats the `yolo_confidence` check defensively.
2. Builds a lookup from frame-local `instance_id` to typed metadata.
3. Requests `target_frame <- cloud_frame` from TF2 at the cloud timestamp.
4. If that exact historical transform is not ready, retains the already-paired
   messages for at most `tf_retry_window` and retries at `tf_retry_rate`.
5. Scans the cloud once and accumulates finite XYZ values for each known ID.
6. Rejects an instance with fewer than `min_points` retained cloud points and
   computes the arithmetic mean XYZ centroid for every remaining instance.
7. Applies that transform to the centroid and submits the resulting map-frame
   observation to the registry.

The registry does not substitute the latest transform when the timestamped
map-frame lookup fails. Its retry queue is bounded by `sync_queue_size`, and the
default one-second window uses the TF buffer's stored dynamic history to match
the original cloud time even though processing happens later. This differs from
the annotator depth-to-color lookup: camera extrinsics are normally static, so
that lookup first tries the depth timestamp and then permits the latest
transform as a fallback.

The point cloud keeps XYZ in the depth optical frame. It is transformed only
for centroid localization; the published cloud itself is not rewritten into
`map`.

## Association and Temporal Confirmation

The registry has two state layers:

```text
one observation
      |
      v
tentative candidate -- insufficient evidence --> expires
      |
      | enough repeated evidence
      v
confirmed object --> published, queryable, visualized, YAML eligible
```

### Existing confirmed objects

A new map-frame observation first searches confirmed objects with the same
YOLO label. Confirmed records are stored in a 3D spatial hash whose cell size is
`duplicate_distance_threshold`. Only the 27 neighboring cells need to be
checked.

If the nearest same-label object is within the threshold, the observation
updates that record immediately:

- Position becomes an observation-count running mean.
- `observation_count` increments.
- `point_count`, `last_confidence`, and `last_seen` take the latest values.
- A spatial-cell change updates the spatial index.

The default threshold is `0.25 m` in full 3D distance. Small centroid motion
caused by depth noise, mask variation, or viewpoint changes is therefore
tolerated. A larger shift can start a separate tentative candidate.

### New tentative candidates

When no confirmed match exists, the observation is associated with the nearest
same-label tentative candidate within the same distance threshold, or creates a
new candidate. Tentative candidates exist only in memory and are not returned
by queries, published as stored markers, or written to YAML.

Each candidate stores a bounded deque of timestamped positions, point counts,
and YOLO confidence values. Repeated data with the same timestamp counts only
once; if a duplicate timestamp has a higher confidence, it replaces the earlier
sample.

Before evaluating a candidate, observations are ordered by timestamp and
normalized:

- Observations older than `confirmation_window` relative to the newest sample
  are removed.
- If a gap exceeds `confirmation_max_gap`, only the run after the most recent
  excessive gap is retained.
- The candidate position is recomputed as the mean of the retained positions.
- Inactive candidates are also pruned by the registry timer.

Promotion requires all of the following default conditions:

| Requirement | Default |
| --- | ---: |
| Distinct timestamped observations | at least 3 |
| Rolling time window | 3.0 s |
| Maximum gap inside the retained run | 1.5 s |
| Mean YOLO confidence | at least 0.6 |
| Per-observation confidence guard | at least 0.4 |
| Spatial association distance | at most 0.25 m |

For example, one high-confidence false `bus` observation expires without
creating a stored object. Three spatially consistent observations with
confidences `0.72`, `0.68`, and `0.75` have mean confidence `0.717` and can be
promoted if their timing also passes.

On promotion, the candidate mean becomes the initial confirmed position, the
candidate observation count is preserved, and the most recent point count,
confidence, and timestamp are copied into the confirmed record.

## Object Identity and Naming

The identifiers serve different lifetimes:

| Identifier | Lifetime and meaning |
| --- | --- |
| `instance_id` | Starts at 1 for each processed RGB frame. Connects one mask, one metadata item, and cloud points in that frame only. |
| `class_id` | Numeric YOLO class. |
| `label` | YOLO class name used for spatial association, such as `chair`. |
| `name` | Stable registry key allocated only after confirmation, such as `chair`, `chair_2`, or `chair_3`. |
| Internal record ID | Process-local integer used by C++ indexes; not part of the ROS interface or YAML contract. |

Instance IDs are not tracker IDs and must never be used to associate objects
between frames. Cross-frame association is based on label and map-frame
distance.

## In-Memory Registry Structure

Confirmed state is organized as:

- An ID-to-record hash map containing the authoritative object records.
- A constant-time exact-name index.
- A label-to-ID index for class queries.
- A per-label 3D spatial hash for nearby-object association.
- A used-name set and suffix counters for deterministic unique naming.

Tentative state has a candidate map plus a label index and bounded observation
deques. Each candidate retains only the highest-confidence JPEG from its
current confirmation run; it does not accumulate a picture per observation. A
two-thread ROS executor permits sensor callbacks and services to
overlap; queue, registry, publication, and persistence state use separate
locks.

The clear service increments an observation generation, clears pending paired
inputs, and invalidates work that began before the clear boundary. Snapshot
publication is serialized so an older nonempty marker snapshot cannot race
after the empty clear snapshot.

## Registry Topics and Messages

Final registry outputs are:

| Default topic | Type | Behavior |
| --- | --- | --- |
| `/sam2/stored_objects` | `sam2_object_registry/msg/StoredObjectArray` | Full confirmed-object snapshot; reliable, transient-local, and change-driven. |
| `/sam2/stored_object_markers` | `visualization_msgs/msg/MarkerArray` | Confirmed centroids and names; reliable, transient-local, and republished at `snapshot_publish_rate` for late-joining RViz displays. |

`StoredObject` contains:

| Field | Meaning |
| --- | --- |
| `name` | Unique persistent registry name. |
| `label` | YOLO label used for association. |
| `class_id` | YOLO class index. |
| `position` | Confirmed centroid in the array header frame. |
| `image_path` | Absolute path to the stored representative JPEG, or empty when image storage failed or was disabled. |
| `image_confidence` | YOLO confidence associated with the representative crop, or `-1` when unavailable. |
| `observation_count` | Number of observations incorporated into the record. |
| `point_count` | Retained cloud points in the latest observation. |
| `last_confidence` | YOLO confidence of the latest observation. |
| `last_seen` | Timestamp of the latest accepted point-cloud observation. |

The array header frame is `target_frame`, normally `map`. The marker array
starts with `DELETEALL`, then adds one green sphere-list point per confirmed
object and one text marker per name. Marker snapshots are republished
periodically even when the database is unchanged, so a volatile RViz subscriber
can join late.

Tentative candidates never appear on either final topic.

## Registry Services

### Query

```bash
ros2 service call /sam2/get_stored_objects sam2_object_registry/srv/GetStoredObjects "{name: '', label: ''}"
```

The request supports an optional exact `name` and optional `label`:

- Empty name and label return every confirmed object.
- A name performs a constant-time exact lookup.
- A label uses the label index.
- Supplying both requires the named object to have the requested label.

The query reads current in-memory state and does not wait for a topic snapshot
or YAML save.

### Save

```bash
ros2 service call /sam2/save_stored_objects std_srvs/srv/Trigger "{}"
```

This checkpoints dirty confirmed state to YAML. Tentative candidates are never
saved.

### Clear

```bash
ros2 service call /sam2/clear_stored_objects std_srvs/srv/Trigger "{}"
```

Clear is destructive and removes:

- Confirmed records.
- Tentative candidates.
- Name, label, and spatial indexes.
- Pending point-cloud and metadata messages.
- Name suffix and internal ID state.
- Representative JPEGs owned by records under `image_directory`.

It invalidates observations already processing before the service boundary,
immediately publishes empty object and marker snapshots, and atomically writes
a valid YAML database with an empty object sequence. If the disk write fails,
the service reports failure but the in-memory registry remains cleared. New
camera observations arriving after the clear can begin fresh tentative
confirmation normally.

## VLM Search And Object Navigation

The v2 mission executive uses the registry as an independent authority. It
never treats a name/metadata match as final confirmation and never consumes a
tentative candidate for approach.

### Find an object from a prompt

```bash
ros2 action send_goal /muto/mission \
  muto_command_layer_v2/action/Mission \
  "{request_id: 'registry-001', objective: 'find the red cup', object_request: 'red cup', completion_policy: 'report_confirmed', schema_version: 'muto_command_layer_v2'}" \
  --feedback
```

The executive queries confirmed registry records and gives the commander a
revision-scoped shortlist. If more than one candidate remains, the commander
requests a second VLM pass over each candidate's stored JPEG. Only the
candidate decision tied to the current registry revision may promote one ID to
the confirmed target.

Both VLM passes request strict structured output. Local enum, ID, revision, and
evidence validation rejects malformed, stale, or incomplete model responses.

The VLM socket is configured by
`muto_command_layer_v2/config/v2_vlm_socket.yaml`. The checked-in profile reads
the credential from `HKU_API_KEY`; no key is stored in YAML.

### Search and approach through v2

The v2 mission action is the only command surface:

```bash
ros2 action send_goal /muto/mission \
  muto_command_layer_v2/action/Mission \
  "{request_id: 'approach-001', objective: 'find and approach chair_2', object_request: 'chair', completion_policy: 'approach_confirmed', schema_version: 'muto_command_layer_v2'}" \
  --feedback
```

The commander may query the registry, inspect each shortlisted stored JPEG,
observe or rotate through the deterministic POI-grid authority, and approach only
one target confirmed for the current registry revision. The deterministic
motion authority selects a footprint-safe stance and delegates the actual path
and obstacle avoidance to Nav2. Tool failures and stale evidence are recorded
on the mission board for replanning; they do not silently promote a candidate.

### v2 interfaces

| Default name | Type | Purpose |
| --- | --- | --- |
| `/muto/mission` | `muto_command_layer_v2/action/Mission` | Normalized mission request, commander loop, and terminal result. |
| `/vlm/generate` | `muto_vlm_socket/action/GenerateVlm` | Planner and candidate-confirmation transport. |
| `/sam2/get_stored_objects` | `sam2_object_registry/srv/GetStoredObjects` | Confirmed registry snapshot. |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Final path-following and obstacle-avoidance authority. |
| `/muto/mission_board` | `muto_command_layer_v2/msg/MissionBoard` | Current board projection. |
| `/muto/mission_event` | `muto_command_layer_v2/msg/MissionEvent` | Decision, tool, evidence, and lifecycle trail. |

The high-level recorder subscribes to these projections and the POI-grid
result's compact diagnostics, writing one unique mission-scoped MCAP without
raw camera, LiDAR, IMU, or point-cloud streams.

## YAML Persistence

Only confirmed objects are persistent. The default empty `output_yaml` resolves
to `sam2_objects.yaml` using this order:

1. `ROS_WORKSPACE` when explicitly set.
2. The workspace root inferred from `COLCON_PREFIX_PATH` or
   `AMENT_PREFIX_PATH`.
3. The node current working directory.

For an installation under `/opt/muto_rs_ws/install`, the usual result is:

```text
/opt/muto_rs_ws/sam2_objects.yaml
/opt/muto_rs_ws/sam2_object_images/
```

When a candidate is promoted, the registry atomically writes its
highest-confidence crop to `image_directory`; the default empty directory is
`sam2_object_images` beside the YAML. The image is written immediately at
promotion, while YAML still follows the configured manual/shutdown checkpoint
policy. Existing legacy records with no image acquire one on their next matched
observation.

The file contains the target frame, duplicate threshold, and sorted confirmed
records. A representative entry is:

```yaml
frame_id: map
duplicate_distance_threshold: 0.25
image_directory: /opt/muto_rs_ws/sam2_object_images
objects:
  - name: chair
    label: chair
    class_id: 56
    position:
      x: 1.24
      y: -0.31
      z: 0.48
    image_path: /opt/muto_rs_ws/sam2_object_images/chair_1.jpg
    image_confidence: 0.86
    observation_count: 7
    point_count: 284
    last_confidence: 0.81
    last_seen:
      sec: 1784792210
      nanosec: 900261120
```

Saving uses a temporary file in the destination directory, full write, file
`fsync`, close, atomic rename, and best-effort directory `fsync`. Concurrent
changes remain dirty when they occur after the saved snapshot.

The complete confirmed database is saved on clean shutdown when
`save_on_shutdown` is enabled. `SIGKILL` and sudden power loss cannot run the
shutdown hook, so the manual save service is required for an immediate
checkpoint.

When the workspace YAML does not exist, default-path mode can load the legacy
`~/.ros/sam2_objects.yaml` and migrate it on the next save. A malformed YAML or
a mismatched `frame_id` disables ordinary persistence to protect the file while
leaving the in-memory registry available. The explicit clear service is allowed
to replace such a file with a valid empty registry.

Objects loaded from YAML are trusted as already confirmed. The temporal policy
does not retroactively remove an older false record.

## Frames and Timestamps

| Product or operation | Timestamp | Frame |
| --- | --- | --- |
| Annotated image, union mask, instance mask, typed detections | RGB image timestamp | RGB image frame |
| Instance point cloud | Selected depth image timestamp | Depth optical frame |
| Depth-to-color projection TF | Depth timestamp, then latest fallback | `color_optical <- depth_optical` |
| Registry cloud/metadata pairing | Absolute header timestamp difference | No frame conversion during pairing |
| Registry centroid TF | Point-cloud timestamp only; may be retried later against TF history | `target_frame <- cloud_frame` |
| Stored object array and markers | Publish time | `target_frame` |

This separation matters: the depth-to-color transform is camera calibration
geometry, while the registry transform places an observation into the robot
world frame.

## RViz Visualization

Use these displays:

| Display | Topic | Fixed-frame notes |
| --- | --- | --- |
| Image | `/sam2/annotated_image` | No TF requirement. |
| PointCloud2 | `/sam2/instance_pointcloud` | Use `RGB8` color transformer when available; RViz needs TF from the depth optical frame to its fixed frame. |
| MarkerArray | `/sam2/stored_object_markers` | Set the RViz fixed frame to `map` or the configured registry `target_frame`. |

The instance cloud visualizes the current masked surfaces. The marker array
visualizes only confirmed stored centroids. If the query service returns an
empty object array, an empty marker display is expected even when YOLO and SAM
are detecting tentative objects.

## Annotator Defaults

| Parameter | Default | Effect |
| --- | --- | --- |
| `device` | `cuda` | SAM2 device. |
| `use_autocast` | `true` | Enables SAM2 autocast. |
| `autocast_dtype` | `bfloat16` | SAM2 autocast type. |
| `yolo_model` | `yolo26m.pt` | YOLO weights name or path. |
| `yolo_device` | `0` | YOLO CUDA device. |
| `yolo_confidence` | `0.4` | Minimum score before SAM2. |
| `yolo_iou` | `0.7` | YOLO NMS IoU. |
| `yolo_imgsz` | `960` | YOLO square inference size used for the current RGB stream. |
| `yolo_max_detections` | `20` | Maximum boxes refined per frame. |
| `detection_crop_jpeg_quality` | `90` | Representative crop JPEG quality; `0` disables crop encoding. |
| `yolo_classes` | empty | Allows every detector class. |
| `yolo_quantize` | `fp16` | YOLO CUDA precision; disabled on CPU. |
| `max_publish_rate` | `7.0` | Maximum processing starts per second; slower inference naturally lowers throughput. |
| `depth_scale` | `0.001` | Converts depth units to metres. |
| `depth_sync_tolerance` | `0.2` | Maximum color/depth offset in seconds. |
| `depth_buffer_size` | `30` | Recent depth frames retained for nearest-timestamp matching. |
| `pointcloud_stride` | `6` | Selects every sixth stable depth-grid pixel before intrinsic and projection work. |
| `pointcloud_mask_trim_ratio` | `0.1` | Trims boundaries for 3D selection. |
| `tf_timeout` | `0.1` | TF timeout in seconds. |
| `queue_size` | `2` | Input queue depth. |

The launch exposes normal model, topic, detector, prompt, depth, TF, and rate
settings. The YAML also records internal defaults such as autocast, overlay
styling, passthrough behavior, and queue depth; the launch does not
automatically load that YAML.

Filter to selected class IDs:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  yolo_classes:="0,2"
```

## Registry Defaults

| Parameter | Default | Effect |
| --- | --- | --- |
| `pointcloud_topic` | `/sam2/instance_pointcloud` | Instance-marked cloud input. |
| `detections_topic` | `/sam2/detections` | Typed metadata input. |
| `objects_topic` | `/sam2/stored_objects` | Confirmed-object snapshot output. |
| `marker_topic` | `/sam2/stored_object_markers` | Confirmed centroid and name markers. |
| `query_service` | `/sam2/get_stored_objects` | Indexed confirmed-object query service. |
| `save_service` | `/sam2/save_stored_objects` | Manual YAML checkpoint service. |
| `clear_service` | `/sam2/clear_stored_objects` | Destructive memory and YAML clear service. |
| `target_frame` | `map` | Frame used for confirmed positions, outputs, markers, and YAML. |
| `duplicate_distance_threshold` | `0.25` | Maximum same-label 3D association distance in metres. |
| `metadata_sync_tolerance` | `0.2` | Maximum cloud/metadata timestamp difference in seconds. |
| `sync_queue_size` | `10` | Queue depth for each registry input. |
| `min_points` | `20` | Minimum retained cloud points needed for a centroid observation. |
| `yolo_confidence` | `0.4` | Defensive minimum confidence at the registry boundary. |
| `confirmation_min_observations` | `3` | Distinct observations required for promotion. |
| `confirmation_min_average_confidence` | `0.6` | Minimum mean confidence over the retained run. |
| `confirmation_window` | `3.0` | Rolling candidate window in seconds. |
| `confirmation_max_gap` | `1.5` | Maximum consecutive-observation gap in seconds. |
| `tf_timeout` | `0.1` | Timestamped cloud-to-target TF timeout. |
| `tf_retry_window` | `1.0` | Maximum wall-clock time to retain a paired observation for exact historical TF. Set to `0` to disable retries. |
| `tf_retry_rate` | `20.0` | Exact timestamped-TF retry rate in Hz. |
| `tf_cache_time` | `30.0` | Registry TF buffer history in seconds. |
| `snapshot_publish_rate` | `2.0` | Changed-object snapshot coalescing and periodic marker rate. |
| `marker_scale` | `0.12` | Stored centroid sphere diameter in metres. |
| `marker_text_height` | `0.12` | RViz name text height. |
| `marker_text_offset` | `0.15` | Vertical offset of name text from the centroid. |
| `output_yaml` | empty | Resolves to the active workspace `sam2_objects.yaml`. |
| `image_directory` | empty | Resolves to `sam2_object_images` beside `output_yaml`. |
| `store_images` | `true` | Writes one representative JPEG when a candidate is confirmed. |
| `save_on_shutdown` | `true` | Writes dirty confirmed state during clean shutdown. |

The registry launch exposes every row above, including topic and service names.

## Current Guarantees and Limits

- **All objects are assumed static in the world.** The registry represents a
  fixed landmark position, not a moving target state. Its running mean smooths
  measurement noise; it must not be interpreted as motion tracking.
- The pipeline is observation-based, not a multi-object tracker. Same-label
  map-frame distance is the cross-frame association signal.
- Two same-label objects closer than `duplicate_distance_threshold` can merge
  into one record. A centroid shift beyond the threshold can create another
  candidate.
- The centroid is the mean of the sampled visible depth surface, not a full
  physical-object center. Viewpoint, occlusion, stride, mask erosion, and depth
  noise can move it slightly.
- Small centroid motion is tolerated by the `0.25 m` association gate.
- The temporal count and mean-confidence policy applies only before initial
  promotion. Later updates to a confirmed object still require the defensive
  `0.4` confidence guard and spatial match, but do not repeat promotion.
- Confirmed objects do not expire automatically. They remain in memory and YAML
  until the clear service or an external database-editing workflow removes
  them.
- A label change is treated as another object class because association requires
  the same label.
- The requested `max_publish_rate` is an upper bound on processing starts, not a
  guaranteed detector rate. Actual throughput is limited by YOLO, SAM2,
  projection, and scheduling time.

## aarch64 Notes

- `device:=cuda` and `yolo_device:=0` require compatible CUDA-enabled
  PyTorch and Ultralytics installations.
- The launch sets `LD_PRELOAD` to
  `/usr/lib/aarch64-linux-gnu/libopenblas.so.0` for this node. Override
  `openblas_preload` if the compatible library is elsewhere.
- Keep `max_publish_rate` bounded initially. Inference runs in a dedicated
  latest-wins worker; superseded RGB frames are dropped rather than queued.
- YOLO defaults to `fp16`; SAM2 separately uses `bfloat16` autocast.
- Keep checkpoints and weights on local storage.

## Failure Behavior

- Import or model initialization failures are logged while the node keeps
  spinning.
- With `publish_passthrough_on_error:=true`, the node publishes an unavailable
  overlay with empty masks and metadata.
- Missing, stale, or invalid depth data, calibration mismatch, and TF failures
  skip only point-cloud publication.
- Rejected YOLO scores are reported in throttled batch and cumulative logs.
  The cumulative value is a node-lifetime count of discarded raw candidates,
  not a count of unique objects or queued work, so it only increases until the
  node restarts.
- When an accepted 2D detection cannot enter the 3D branch, the annotator emits
  a throttled `Skipping instance point cloud` report containing the current
  missing calibration, timestamp, encoding, or TF reasons.
- A valid no-detection frame publishes image-sized empty masks, empty typed
  metadata, and an empty instance cloud when the 3D inputs are available.
- The registry waits when no cloud/metadata pair falls inside
  `metadata_sync_tolerance`; bounded queues eventually discard older messages.
- Registry observations below the defensive confidence threshold or
  `min_points` are ignored.
- A missing timestamped `target_frame <- cloud_frame` transform retains the
  paired observation for up to `tf_retry_window`; it is rejected only if the
  exact historical transform remains unavailable. No latest-TF fallback is
  used at this stage.
- A valid new object remains invisible on stored topics and markers until its
  temporal candidate is promoted.
- A malformed or frame-incompatible YAML disables ordinary persistence but not
  the in-memory registry or query service.
- A clear-service disk failure leaves memory empty and reports `success: false`;
  later save or clean shutdown can retry the YAML write.

## Diagnostics

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /sam2/annotated_image
ros2 topic echo /sam2/segments --once
ros2 topic info /sam2/instance_pointcloud --verbose
ros2 run tf2_ros tf2_echo <color_optical_frame> <depth_optical_frame>

ros2 node list
ros2 service list
ros2 topic hz /sam2/detections
ros2 topic echo /sam2/detections --once
ros2 topic echo /sam2/instance_pointcloud --once
ros2 topic echo /sam2/stored_objects --once
ros2 topic echo /sam2/stored_object_markers --once
ros2 service call /sam2/get_stored_objects sam2_object_registry/srv/GetStoredObjects "{name: '', label: ''}"
ros2 param get /object_registry target_frame
ros2 param get /object_registry confirmation_min_average_confidence
ros2 action info /vlm/generate
ros2 action info /muto/mission
ros2 topic echo /muto/mission_board --once
ros2 topic echo /muto/mission_event --once
ros2 topic echo /muto/mission_board --once
```

Manual prompts can isolate detector problems from SAM2 or camera problems:

```bash
ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  prompt_mode:=manual point_coords:="320,240" point_labels:="1"

ros2 launch sam2_image_annotator sam2_image_annotator_launch.py \
  prompt_mode:=manual box:="120,80,520,420" default_prompt:=none
```
