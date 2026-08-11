# Object pipeline functions

`command_layer_launch.py` connects camera perception, 3D instance geometry,
the persistent object registry, a multimodal VLM bridge, and the command layer.
This document describes the functions made available by that launch and their
runtime contracts.

## Scope and process graph

The launch starts the following cooperating nodes (the legacy
`active_object_search` node is opt-in):

```text
RGB + depth + CameraInfo + TF
               |
               v
  sam2_image_annotator
    YOLO detection -> SAM 2 masks -> instance point cloud
               |
               v
  sam2_object_registry
    temporal confirmation -> spatial merge -> YAML/JPEG persistence
               |
               +-----------------------------+
               |                             |
               v                             v
       object_search                     command_layer
   registry + VLM selection    registry + TF + Nav2 + SLAM Toolbox
               |                             |
               v                             +--> /go_to_object
         /find_object                        +--> /save_map
                                             +--> mission lifecycle events

  muto_exploration_bag
    mission lifecycle -> one action-scoped diagnostic MCAP

  muto_command_bag
    /look_for_object lifecycle -> one parent decision/context MCAP

  active_object_search (disabled compatibility node)
    /find_object + /explore_and_record + /sam2/stored_objects
               |
               v
       /find_something

  model_commander
    event-driven decisions + primitive history/blackboard
               |
               v
       /look_for_object
       |-- /command_primitives/explore_frontier
       |-- /spin
       |-- /sam2/detection_heartbeat (only while observing)
       `-- /sam2/save_stored_objects

  vlm_socket provides /vlm/generate to object_search
               natural_language_command_router, and model_commander

  natural_language_command_router
    strict intent enum -> existing typed command interfaces
               |
               v
      /natural_language_command
```

The launch does **not** start the camera driver, TF publishers, Nav2, or a VLM
provider. Those are external dependencies. The VLM socket is an HTTP bridge;
the configured provider must already be reachable.

Start the complete object pipeline with:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

This command uses `config/object_pipeline_vlm.yaml`, whose checked-in profile
selects the tested `hkuproxy` chat-completions endpoint, `gpt-5.6-sol`, and the
`HKU_API_KEY` environment-variable name. `vlm_model` remains the socket default,
`object_search_vlm_model` keeps object matching on `gpt-5.6-sol`,
`natural_language_vlm_model` defaults to `gpt-5.3-codex-spark`, and
`model_commander_vlm_model` defaults to `gpt-5.6-luna`. When selecting a
provider file for another backend, also set `vlm_base_url`, `vlm_wire_api`, and
the relevant model arguments.

> **Transport warning:** the checked-in proxy URL uses plain HTTP. The API key
> is not stored in ROS configuration, but its bearer header is not encrypted
> in transit. Model-supervised search also sends fresh full-camera JPEGs, not
> only stored candidate crops. Use a trusted network, VPN, or secure tunnel,
> or configure an HTTPS endpoint.

For a full robot deployment, run the independent Nav2 and object pipelines
alongside each other:

```bash
# Terminal 1
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py

# Terminal 2
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

The Nav2 pipeline does not start the annotator, registry, VLM socket, or command
layer. The object pipeline owns `/find_object`, `/look_for_object`, and
`/go_to_object`; its active-search and go-to-object
commands consume the independent Nav2 stack through existing command actions.
`/find_something` is available only when its compatibility launch option is
enabled.

## Functions enabled by the pipeline

### 1. Detect and segment objects

The annotator runs YOLO on the RGB stream and uses each accepted bounding box
as a SAM 2 prompt. The default detector is `yolo26m.pt`. Detections below
`yolo_confidence` (`0.4` by default) are stopped before SAM, depth projection,
and registry processing.

The annotator publishes:

| Output | Type | Function |
| --- | --- | --- |
| `/sam2/annotated_image` | `sensor_msgs/msg/Image` | RGB image with boxes and mask overlays. |
| `/sam2/mask` | `sensor_msgs/msg/Image` | Mono8 union of accepted instance masks. |
| `/sam2/instance_mask` | `sensor_msgs/msg/Image` | 16UC1 mask whose nonzero pixels are instance IDs. |
| `/sam2/segments` | `std_msgs/msg/String` | JSON segmentation metadata for inspection. |
| `/sam2/detections` | `sam2_object_registry/msg/DetectedObjectArray` | Typed detections, confidence, boxes, mask statistics, and JPEG crops. |
| `/sam2/detection_heartbeat` | `std_msgs/msg/Header` | Crop-free completion signal for each published detector frame. |

Processing starts at no more than `max_publish_rate` (`7 Hz` by default). The
camera may publish faster; only the newest pending RGB work item is retained.

### 2. Generate instance-marked 3D surfaces

For each processed RGB frame, the annotator selects the closest buffered depth
frame within the configured timestamp tolerance. It then:

1. samples the depth grid with `pointcloud_stride` (default `6`);
2. back-projects sampled pixels using depth intrinsics;
3. obtains the depth-optical to color-optical transform from TF2;
4. projects each transformed point into the RGB image;
5. retains points that land inside a SAM instance mask; and
6. publishes XYZ, RGB, and `instance_id` fields on
   `/sam2/instance_pointcloud`.

The default 10% inward mask trim rejects boundary depth pixels that are most
likely to belong to the background. Sampling happens before the expensive
projection work.

This function requires valid depth and color `CameraInfo`, sufficiently aligned
RGB/depth timestamps, and the relevant optical-frame transform. A failure here
does not stop 2D detection outputs, but it prevents that frame from contributing
3D observations to the registry.

### 3. Build a stable object registry

The C++ registry synchronizes typed detections with instance point clouds,
computes one centroid per instance, and transforms it into `target_frame`
(`map` by default) using TF2 at the observation timestamp. If the exact
historical transform has not arrived yet, the paired observation is retained
for up to `registry_tf_retry_window` (1 second by default) and retried at
`registry_tf_retry_rate` (20 Hz by default); the current/latest robot pose is
never substituted for that historical pose.

New observations are not immediately treated as real objects. Under the
registry defaults, a tentative object needs:

- three distinct observations;
- all observations within a three-second window;
- no gap longer than 1.5 seconds; and
- mean YOLO confidence of at least 0.6.

This filters isolated confidence spikes. Confirmed observations with the same
YOLO label and a nearby centroid are merged. Distinct locations receive stable
IDs such as `chair`, `chair_2`, and `chair_3`. The label is semantic class data;
the generated name is the ID used by command actions.

Confirmed objects are exposed on the transient-local
`/sam2/stored_objects` snapshot. Tentative candidates are not queryable,
published, or written to YAML.

### 4. Store representative pictures and persistent object data

Each confirmed object can retain the highest-confidence JPEG crop seen during
confirmation. By default, images are written to `sam2_object_images` beside the
registry YAML and their absolute paths are stored in the object record.

An empty `registry_output_yaml` resolves to `sam2_objects.yaml` in the active
ROS workspace root. By default, startup clears the previous map-frame object
state. Pass `load_existing_map:=true` only when restoring its saved map; then
existing objects are loaded at startup and new objects are
merged in memory, and the complete registry is atomically rewritten on clean
shutdown.

Manual persistence functions are also available:

```bash
# Write the current confirmed registry now.
ros2 service call /sam2/save_stored_objects std_srvs/srv/Trigger "{}"

# Destructively remove confirmed and tentative state, owned JPEGs, and YAML data.
ros2 service call /sam2/clear_stored_objects std_srvs/srv/Trigger "{}"
```

Use the save service before a power cut when the latest registry state matters;
SIGKILL and sudden power loss cannot execute the shutdown checkpoint.

### 5. Query objects by ID or label

`/sam2/get_stored_objects` provides indexed access to confirmed objects.

```bash
# Return every confirmed object.
ros2 service call /sam2/get_stored_objects \
  sam2_object_registry/srv/GetStoredObjects \
  "{name: '', label: ''}"

# Exact persistent ID lookup.
ros2 service call /sam2/get_stored_objects \
  sam2_object_registry/srv/GetStoredObjects \
  "{name: 'chair_2', label: ''}"

# Return all objects with one YOLO label.
ros2 service call /sam2/get_stored_objects \
  sam2_object_registry/srv/GetStoredObjects \
  "{name: '', label: 'chair'}"
```

Each `StoredObject` contains its ID, label, class ID, centroid, representative
image path and confidence, observation count, point count, latest confidence,
and last-seen time.

### 6. Visualize registered and live objects

Two complementary 3D views are available:

- `/sam2/instance_pointcloud` shows the current masked object surfaces in the
  depth optical frame.
- `/sam2/stored_object_markers` shows confirmed centroids and ID labels in the
  registry target frame.

For the registry view, add a `MarkerArray` display in RViz, select
`/sam2/stored_object_markers`, and use `map` as the fixed frame when
`global_frame:=map`. Marker snapshots are periodically republished, so RViz can
join after the object was registered.

### 7. Find stored objects using natural language and images

The `/find_object` action accepts a natural-language prompt and returns zero or
more `ObjectMatch` records.

```bash
ros2 action send_goal /find_object \
  muto_command_layer/action/FindObject \
  "{prompt: 'find the red cup'}" --feedback
```

Selection is performed in two bounded stages:

1. A text-only VLM request receives the prompt plus registered IDs, labels, and
   class IDs. It produces a shortlist.
2. When one or more candidates remain, the pipeline loads every shortlisted
   JPEG and asks the VLM to verify the images against the original prompt.

Both stages use strict JSON Schemas whose ID enums contain only the eligible
registry IDs. The command layer also validates the returned JSON and exact IDs
locally. Incomplete, filtered, failed, or refused VLM responses fail the action
instead of returning partial selections.

The action result contains the complete match batch. Each match is also
published on `/object_search/matches` with a query ID, rank, batch size, exact
object ID, label, and VLM description:

```bash
ros2 topic echo /object_search/matches \
  muto_command_layer/msg/ObjectMatch
```

Zero candidates is a successful no-match result. Exactly one candidate still
receives visual verification, so requested attributes such as color are not
accepted from class metadata alone.

### 8. Legacy active-search compatibility

`/find_something` wires together `/find_object`, `/explore_and_record`, and the
transient-local `/sam2/stored_objects` snapshot. It first searches without
moving. If no registered object matches, it starts the normal exploration,
six-step scans, recording, and predicted-visibility mission. Each changed set
of confirmed registry IDs triggers another `/find_object` query. A match
cancels the mission and is returned as the existing `ObjectMatch` type.

```bash
ros2 action send_goal /find_something \
  muto_command_layer/action/FindSomething \
  "{prompt: 'find a red mug'}" --feedback
```

If the predictive mission completes first, the action performs a final query
and returns a successful result with an empty match list. Re-observing an
existing ID does not trigger another VLM query. The command assumes registered
objects are static, does not pursue moving targets, and does not automatically
approach a match.

This compatibility node is disabled by default. New object-search clients
should use `/look_for_object`, whose commander schedules smaller primitives.

### 9. Run a persistent model-supervised object search

`/look_for_object` is an always-available mission server above the existing
typed commands. For each goal it first calls `/find_object` without moving. If
there is no match, it waits for a new color-camera frame, converts it to a
bounded JPEG, and asks the VLM to inspect that frame plus compact mission state.
The VLM then chooses exactly one locally validated next primitive:
`verify_registry`, bounded `explore_frontier`, in-place `rotate`, stationary
`observe`, `checkpoint_registry`, bounded `wait`, or `finish_not_found`.

`explore_frontier` runs frontier travel for a bounded interval and stops without
rotating. `rotate` owns one bounded Nav2 Spin goal. `observe` creates a
temporary `/sam2/detection_heartbeat` subscription and waits for fresh crop-free
frame completions without motion. The subscription is destroyed when the
primitive ends. `checkpoint_registry` atomically saves
the current registry. Their outcomes are appended to the mission blackboard
before replanning, so their order is not fixed. The legacy
`/explore_and_record` endpoint remains available for compatibility but is no
longer visible to the commander.

```bash
ros2 action send_goal /look_for_object \
  muto_command_layer/action/LookForObject \
  "{prompt: 'the red mug beside the kettle', max_duration: 0.0, max_planning_steps: 0}" \
  --feedback
```

The node monitors child completion and the transient-local object-registry
snapshot. A changed confirmed-object identity set cancels an in-flight model
decision, wait, or exploration program, then forces a fresh `/find_object` check
before replanning. Position, confidence, and repeat-observation updates for the
same identity deliberately do not churn this revision. This prevents a decision
made against an older object inventory from being executed.

The VLM also inspects a fresh forward-camera frame after a 20-second cooldown by
default while an exploration primitive or a model-directed wait remains active.
Inference time is additional; this is strategic periodic monitoring, not a
real-time camera loop. This
restricted monitor can only continue the current bounded command or request a
safe stop and replan. `possible` or `likely` evidence requires the latter. Local
code confirms the child is terminal, checks the registry, and then runs the
normal planner; the monitor cannot dispatch a replacement itself. Nav2 remains
responsible for real-time collision response.

Every inspection requires a camera frame received after the previous one. A
raw-image subscription is created only while taking that snapshot; normal frame
arrival does not invoke the model or cancel an in-flight decision. Newly
received does not prove newly captured: a camera driver that republishes frozen
pixels cannot be distinguished from a genuinely unchanged scene here.

Local controls allow one mission and one owned child at a time, impose finite
duration, planning-step, command-dispatch, wait, exploration-cycle, retry, and
no-progress limits, and cancel the owned child when the parent goal is canceled.
The strict decision schema cannot name ROS interfaces, poses, arbitrary
parameters, or code. Its response must report a bounded visual summary and
target evidence (`not_visible`, `possible`, `likely`, or `unclear`). A model
cannot declare an object found: only an exact `/find_object` result can do that.
`possible` or `likely` evidence forces a registry check before any further
motion. It also cannot end a no-match mission until the configured stationary
observation count, cumulative completed rotation, registry checkpoint count,
and frontier evidence have been followed by a current identity-set check.
Primitive order is not fixed.
An object that never enters the YOLO/SAM registry therefore cannot be returned
as found from VLM pixels alone.

A missing, stale, unconvertible, or oversized planning frame causes bounded
no-motion retries and then aborts the model mission. During an owned motion,
isolated monitor failures are retried; the configured consecutive-failure limit
stops the child and aborts rather than continuing without visual supervision.
Each request currently contains one forward-view JPEG; it is not proof that
every direction observed during a 360-degree scan was inspected by the VLM.
If an accepted moving child cannot be confirmed stopped, the commander retains
its ownership state, publishes `ownership_uncertain`, and rejects new missions
until its process is restarted.

```bash
ros2 topic echo /model_commander/status --once \
  --qos-durability transient_local
```

The direct action remains active until it finds a match, reaches a valid
no-match decision, exhausts a budget, fails, or is canceled. A zero goal budget
selects the configured finite defaults of 1800 seconds and 64 planning steps.
`/find_something` remains an opt-in deterministic compatibility fallback.
Each model-dispatched frontier primitive keeps the existing action-scoped MCAP
handshake, so a multi-step model mission can produce multiple child bags.
The separate `muto_command_bag` recorder also keeps one continuous parent bag
from the accepted `/look_for_object` goal through terminal status. It records
planning inputs, validated decisions, command outcomes, sampled inspected
JPEGs, waits, registry checks, primitive traffic, and manual operator events.
The parent path is published on `/model_commander/last_bag_path`.

### 10. Explore and record with predicted visibility

`/explore_and_record` alternates frontier navigation with six-step,
360-degree observation scans and registry checkpoints. When frontier
exploration reports completion, the command snapshots the SLAM occupancy map
and Nav2 global costmap, generates robot-reachable viewpoints, and continues
scanning until its predicted observable free-space and occupied-boundary
ratios reach `visibility_completion_ratio`.

This completion value is a 2-D geometric estimate. After Nav2 reaches a
viewpoint and completes the requested spin steps, the planner credits cells
predicted visible by occupancy-grid line of sight. Fresh detector messages end
each observation wait early, but their contents and registry growth are not
coverage evidence; a detector timeout warns and proceeds. The default `0.98`
therefore means 98 percent of the planner's predicted coverable free and
boundary cells, not measured camera coverage or detector recall.

The prediction is consistent with the pipeline's static-object mission but
does not establish that every static object was observed. Navigation failures
discard the selected viewpoint without predicted-visibility credit.

The separate `muto_exploration_bag` process owns each rosbag2 recording. Its
default compact MCAP retains hidden action status and feedback, `/tf_static`,
scans, structured object results, maps, odometry, logs, lifecycle events, and
any plain-text `/explore_and_record/operator_event`. It excludes raw camera
images, derived mask/annotation images, and point clouds. The finalized path is
published on `/explore_and_record/last_bag_path`; metadata includes the action
goal context, recorder build git state, and active exclusion regex. Bags are
finalized on success, cancel, and abort.

### 11. Navigate to and face a registered object

The `/go_to_object` action accepts an exact persistent object ID:

```bash
ros2 action send_goal /go_to_object \
  muto_command_layer/action/GoToObject \
  "{object_id: 'chair_2'}" --feedback
```

The command node queries the registry, transforms the centroid into the Nav2
global frame, reads the current robot pose from TF2, and requests Nav2's master
global costmap through `/global_costmap/get_costmap`. That costmap already
contains Nav2's static, obstacle, and inflation layers. The command treats
costs through `approach_maximum_cost` as traversable; its default of `252`
blocks inscribed, lethal, and unknown cells. Starting at
`max(approach_distance, approach_robot_radius)`, it searches outward around the
object. The first costmap-resolution ring containing robot-reachable candidates
is preferred; within that ring, the shortest reachable path wins. The
resulting orientation faces the centroid.

The selected pose is returned in action feedback/result and published with
transient-local durability on `/object_navigation/target_pose` for RViz.
Canceling `/go_to_object` forwards cancellation to its active Nav2 goal.

This command approaches the stored centroid; it does not visually re-identify
the object after arrival. It aborts before dispatch when no reachable approach
cell exists. Nav2 still performs final planning and execution and can reject a
cell if the costmap changes after the service snapshot.

### 12. Save the live occupancy map

The command layer wraps SLAM Toolbox's `/slam_toolbox/save_map` service as
`/save_map`:

```bash
ros2 service call /save_map slam_toolbox/srv/SaveMap \
  "{name: {data: warehouse}}"
```

The request field is a basename, not a path. The wrapper trims whitespace,
rejects path separators, traversal, and unsupported characters, then prefixes
the configured `map_save_directory`. An empty request selects
`default_map_name`. With the default empty directory parameter, output goes
under `$HOME/.ros/maps`; the directory is created on demand. Custom directory
components are restricted to a shell-safe ASCII subset because Humble's map
saver invokes `map_saver_cli` through a system command.

This is an occupancy-map export for later Nav2/map-server use. It does not
serialize the SLAM Toolbox pose graph. The wrapper returns SLAM Toolbox's
standard result code and reports failure when no map has been received, the
underlying service is unavailable, or the bounded save timeout expires.

### 13. Trigger typed commands through natural language

The `/natural_language_command` action accepts one request such as `search the
map for a red chair`, `go to the red chair`, `start exploring`, `run the
mapping and recording mission`, `save the map as warehouse`, or `cancel the
active command`.

```bash
ros2 action send_goal /natural_language_command \
  muto_command_layer/action/NaturalLanguageCommand \
  "{query: 'run exploration and record static objects'}" --feedback
```

The VLM is a classifier, not a ROS executor. It must return one strict JSON
object whose command is one of `find_object`, `find_something`,
`look_for_object`, `go_to_object`, `start_exploration`, `stop_exploration`,
`explore_and_record`, `save_map`, `cancel_active_command`, or `unsupported`.
The command router independently checks the exact object shape, argument types,
and configured numeric bounds.
It then dispatches only the corresponding compiled action or service client;
model-provided ROS names, arbitrary parameters, and code cannot reach the ROS
graph.

Object descriptions sent to `go_to_object` are resolved through
`/find_object`. Exactly one persistent registry ID must match before the router
calls `/go_to_object`. This pipeline assumes mapped objects remain static.

Registry-only `find_object` waits for its result. Deterministic active search,
model-supervised search, navigation, and explore-and-record return from the
natural-language action after the typed child accepts the goal, while the router
retains the child handle. A subsequent natural-language cancel can therefore
stop the long-running command. Call `/look_for_object` directly when the caller
must wait for its terminal result.

Exact, unambiguous cancellation phrases are recognized locally before VLM
classification. They can therefore cancel a router-owned model mission while
the single-request VLM socket is occupied. Ambiguous or compound phrases still
use the normal validated interpretation path.

### 14. Use the VLM bridge directly

The launch also exposes the general `/vlm/generate` action. Other ROS nodes may
send one ordered message containing text and/or JPEG parts and may optionally
supply a strict response JSON Schema. Object search uses this action internally,
but it is not limited to registry queries.

Only one VLM request is processed at a time. Credentials are read from the
environment variable named by `api_key_env`; API keys are not launch arguments
or ROS parameters.

## Public interface summary

| Name | Kind and type | Purpose |
| --- | --- | --- |
| `/natural_language_command` | Action: `muto_command_layer/action/NaturalLanguageCommand` | Validate one VLM-classified request and dispatch a fixed typed command. |
| `/natural_language_command/decision_event` | Topic: `std_msgs/msg/String` | Transient-local original query, interpretation source, validated intent, and dispatch result for command-bag correlation. |
| `/find_object` | Action: `muto_command_layer/action/FindObject` | Select registered objects from a natural-language prompt. |
| `/find_something` | Compatibility action: `muto_command_layer/action/FindSomething` | Opt-in fixed search sequence retained for rollback. |
| `/look_for_object` | Action: `muto_command_layer/action/LookForObject` | Persistently schedule bounded primitives and replan until a described static object is found or the mission ends. |
| `/command_primitives/explore_frontier` | Internal action: `muto_command_layer/action/ExploreAndRecord` | Run bounded frontier travel, stop, and return its outcome without scanning. |
| `/spin` | Internal action: `nav2_msgs/action/Spin` | Execute one model-selected bounded in-place rotation; owned and canceled by the commander. |
| `/sam2/detection_heartbeat` | Topic: `std_msgs/msg/Header` | Crop-free detector-frame clock subscribed only during the stationary `observe` primitive. |
| `/model_commander/status` | Topic: `std_msgs/msg/String` | Transient-local bounded JSON heartbeat for the current or most recent model-supervised mission. |
| `/model_commander/decision_event` | Topic: `std_msgs/msg/String` | Append-only planning request, validated decision, and command-result context retained by the parent command bag. |
| `/model_commander/inspected_image` | Topic: `sensor_msgs/msg/CompressedImage` | Exact bounded JPEG supplied to each commander planning or active-monitoring request. |
| `/model_commander/recording_event` | Topic: `std_msgs/msg/String` | Complete `/look_for_object` mission start and terminal lifecycle for `muto_command_bag`. |
| `/model_commander/bag_status` | Topic: `std_msgs/msg/String` | Parent command-recorder ready, finishing, finalized, or error status. |
| `/model_commander/last_bag_path` | Topic: `std_msgs/msg/String` | Latest `muto_command_*` mission directory. |
| `/model_commander/operator_event` | Topic: `std_msgs/msg/String` | Manual high-level mission milestone retained in the active parent bag. |
| `/go_to_object` | Action: `muto_command_layer/action/GoToObject` | Approach and face one exact object ID through Nav2. |
| `/global_costmap/get_costmap` | Service: `nav2_msgs/srv/GetCostmap` | Current Nav2 master costmap used for object approach and visibility traversal. |
| `/explore_and_record` | Legacy action: `muto_command_layer/action/ExploreAndRecord` | Compatibility composite: alternate frontier exploration and scans, then pursue predicted visibility coverage. |
| `/explore_and_record/recording_event` | Topic: `std_msgs/msg/String` | Command-layer JSON mission start and terminal events that control the standalone recorder. |
| `/explore_and_record/bag_status` | Topic: `std_msgs/msg/String` | Transient recorder-ready, finishing, finalized, or error acknowledgement. |
| `/explore_and_record/last_bag_path` | Topic: `std_msgs/msg/String` | Transient-local path of the latest per-mission MCAP bag. |
| `/explore_and_record/operator_event` | Topic: `std_msgs/msg/String` | Plain-text manual observation or milestone retained in the active bag. |
| `/explore` | Service: `std_srvs/srv/SetBool` | Start (`true`) or stop (`false`) Muto frontier exploration. |
| `/save_map` | Service: `slam_toolbox/srv/SaveMap` | Save the live occupancy map beneath the configured output directory. |
| `/vlm/generate` | Action: `muto_vlm_socket/action/GenerateVlm` | Ordered text/JPEG VLM request with optional structured output. |
| `/sam2/get_stored_objects` | Service: `sam2_object_registry/srv/GetStoredObjects` | Query confirmed objects by exact ID and/or label. |
| `/sam2/save_stored_objects` | Service: `std_srvs/srv/Trigger` | Atomically checkpoint the registry YAML. |
| `/sam2/clear_stored_objects` | Service: `std_srvs/srv/Trigger` | Destructively clear dynamic and persistent registry state. |
| `/object_search/matches` | Topic: `muto_command_layer/msg/ObjectMatch` | One publication per final natural-language match. |
| `/object_navigation/target_pose` | Topic: `geometry_msgs/msg/PoseStamped` | Latest computed object approach pose. |
| `/sam2/stored_objects` | Topic: `sam2_object_registry/msg/StoredObjectArray` | Confirmed registry snapshot. |
| `/sam2/stored_object_markers` | Topic: `visualization_msgs/msg/MarkerArray` | RViz centroid and ID markers. |
| `/sam2/instance_pointcloud` | Topic: `sensor_msgs/msg/PointCloud2` | Current per-instance masked 3D surfaces. |

Topic and action names shown above are defaults. The endpoints routed through
`command_layer_launch.py` can be renamed with launch arguments.

## External requirements and degraded operation

| Missing dependency | Effect |
| --- | --- |
| RGB camera | No YOLO/SAM processing or new observations. Existing registry matches can still be returned, but `/look_for_object` cannot make a new VLM scheduling decision and aborts after bounded no-motion retries. |
| Depth image, valid intrinsics, or depth-to-color TF | 2D outputs continue, but no instance point cloud or new 3D registry entries. |
| TF into `target_frame` | The registry retries the exact observation time for a bounded window, then skips observations it still cannot transform. |
| VLM endpoint or credential | `/find_object` fails and `/look_for_object` defers then aborts after bounded retries; perception, registry queries, visualization, and `/go_to_object` remain usable. |
| Nav2 global-costmap service | `/go_to_object` cannot select an approach cell and post-frontier predicted-visibility coverage cannot start; object search and registry functions remain usable. |
| Nav2 or global/base TF | `/go_to_object` fails; perception, registry, and `/find_object` remain usable. |
| Nav2 Spin behavior | `rotate` and legacy `/explore_and_record` abort before rotating; frontier-only exploration remains available. |
| Frontier control service | `/explore` fails; object perception and commands remain usable. |
| SLAM Toolbox save-map service | `/save_map` fails; navigation and other command-layer functions remain usable. |
| Stored candidate JPEGs | Metadata search works; ambiguous visual refinement normally aborts. |

Each high-level action accepts only one active goal at a time and uses bounded
dependency waits. The model commander owns at most one child and invokes
`/find_object`, model planning, and one bounded primitive sequentially. The VLM
socket itself remains single-request, so unrelated direct `/vlm/generate`
clients can still cause bounded deferral or failure. There is not yet a global
motion lease across every ROS client: direct Nav2, frontier, or command-action
clients can compete with a commander-owned motion. Operators should send motion
through one high-level owner at a time. The natural-language router serializes
interpretation and short operations, but releases its action after a long
motion goal is accepted so a later natural-language cancel can be interpreted.

## Main launch controls

The top-level launch exposes the controls most likely to vary by deployment:

- camera topics: `image_topic`, `depth_topic`,
  `depth_camera_info_topic`, and `color_camera_info_topic`;
- inference: `yolo_model`, `yolo_device`, `yolo_confidence`,
  `detection_crop_jpeg_quality`, and `max_publish_rate`;
- registry: `registry_output_yaml`, `registry_image_directory`,
  `registry_store_images`, `load_existing_map`, `registry_tf_retry_window`,
  `registry_tf_retry_rate`, and `global_frame`;
- VLM: `vlm_params_file`, `vlm_action`, `vlm_base_url`, `vlm_wire_api`,
  `vlm_model`, `object_search_vlm_model`, `natural_language_vlm_model`, and
  `model_commander_vlm_model`; and
- commands: `natural_language_command_action`,
  `launch_natural_language_command`, `launch_model_commander`,
  `look_for_object_action`, `model_commander_status_topic`, `action_name`,
  `find_object_action`, `object_match_topic`, `navigate_to_pose_action`,
  `global_costmap_service`, `robot_base_frame`, `approach_distance`,
  `approach_robot_radius`, `approach_start_snap_distance`,
  `approach_maximum_cost`, `global_costmap_timeout`, `explore_service`,
  `save_map_service`, `slam_toolbox_save_map_service`, `map_save_directory`,
  `default_map_name`, `save_map_timeout`, `explore_and_record_action`,
  `spin_action`, cycle timing, and
  `launch_frontier_explorer`.

Use `ros2 launch muto_command_layer command_layer_launch.py --show-args` for
the complete argument list. Lower-level tuning such as point-cloud stride,
mask trimming, registry confirmation thresholds, and spatial merge distance
currently remains in the child package launch/configuration surfaces; it is not
forwarded by this top-level launch.
