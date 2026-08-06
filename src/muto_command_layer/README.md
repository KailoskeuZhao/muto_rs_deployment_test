# Muto command layer

`muto_command_layer` exposes robot-level commands above perception, the object
registry, the VLM socket, and Nav2. It provides `/find_object` registry lookup,
cancellable `/find_something` active search and `/go_to_object` navigation,
`/explore` start/stop, a sanitized `/save_map` wrapper, and a validated
`/natural_language_command` action that routes natural language to those typed
interfaces.

See [Object pipeline functions](docs/object_pipeline_functions.md) for the
complete functional interface, runtime dependencies, operator commands, and
degraded-operation behavior of `command_layer_launch.py`.

## Complete command-layer launch

Start the command layer together with its SAM 2 image annotator, object
registry, and VLM socket:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

Credentials remain environment-only. The launch shares the annotator detection
and point-cloud topics with the registry, and shares the registry service and
VLM action endpoints with the command layer. Each included package is scoped so
similarly named child launch arguments cannot leak into another package.

The deployment-level VLM settings live in
`config/object_pipeline_vlm.yaml`. It contains the provider URL, wire API,
model, request limits, and the *name* of the credential environment variable.
The checked-in profile selects the tested `hkuproxy` Responses endpoint,
`gpt-5.6-sol`, and `HKU_API_KEY`. Select another file at launch time with
`vlm_params_file:=/absolute/path/to/vlm.yaml`. The top-level launch always sets
the provider URL, wire API, and model from `vlm_base_url`, `vlm_wire_api`, and
`vlm_model`; their defaults mirror the checked-in profile. When selecting a
file for a different provider, override those three launch arguments as well.

> **Transport warning:** the current proxy URL uses plain HTTP. Although the
> key is absent from launch arguments and YAML, its bearer header is not
> encrypted in transit. Use this endpoint only through a trusted network,
> VPN, or secure tunnel, or replace it with an HTTPS endpoint.

The included processes are launched together; dependency readiness is checked
when an action goal is executed rather than through arbitrary startup delays.
The `/go_to_object` action requires Nav2's global-costmap service, navigation
action, and global/base TF.
`/find_object` requires the registry and VLM socket; camera and TF inputs are
additionally needed to create new registry entries. The Muto-configured
frontier explorer is also started in cold idle so `/explore` can activate it
without process startup latency.

`muto_nav2_pipeline_launch.py` independently owns hardware, localization,
mapping, and Nav2. It does not start the annotator, registry, VLM socket, or
command layer. For a complete deployment, run the Nav2 and object pipelines
alongside each other:

```bash
# Terminal 1: hardware, localization, mapping, and Nav2.
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py

# Terminal 2: perception, registry, VLM, and both object commands.
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

## Explore command

Start autonomous frontier exploration after mapping and Nav2 are active:

```bash
ros2 service call /explore std_srvs/srv/SetBool "{data: true}"
```

Stop exploration and cancel its active Nav2 goal:

```bash
ros2 service call /explore std_srvs/srv/SetBool "{data: false}"
```

Stopping returns `frontier_explorer` to cold idle; it does not terminate the
process, so exploration can be started again. The command forwards to
`/control_exploration` with zero delay and never requests `quit_after_stop`.
Set `launch_frontier_explorer:=false` only when another launch already owns the
frontier explorer process.

## Save-map command

Save the current SLAM Toolbox occupancy map with an explicit basename:

```bash
ros2 service call /save_map slam_toolbox/srv/SaveMap \
  "{name: {data: warehouse}}"
```

An empty name uses `default_map_name` (`muto_map`). The wrapper accepts only a
basename containing letters, numbers, `.`, `_`, or `-`; callers cannot select
an arbitrary path. It creates `map_save_directory` when needed and forwards an
absolute output prefix to `/slam_toolbox/save_map`. An empty configured
directory resolves to `$HOME/.ros/maps`. Custom directory components use the
wrapper's shell-safe ASCII subset: letters, numbers, `.`, `_`, and `-`.

This saves the occupancy-map YAML and image produced by SLAM Toolbox. It does
not serialize the SLAM pose graph for resuming a mapping session. The live
online mapper must have received a map before this command can succeed.

## Explore-and-record program

`/explore_and_record` is the command-layer coordinator for autonomous mapping
and object collection. Its default cycle is:

1. Run frontier exploration for at least 10 seconds.
2. If Nav2 is still driving to a frontier, let that travel goal finish; then
   pause exploration at the goal boundary.
3. Wait 0.25 seconds for the robot to settle.
4. Perform six positive 60-degree in-place turns through Nav2 `/spin`, for a
   complete 360-degree scan.
5. After every step, wait for three fresh `/sam2/detections` messages. Three
   seconds remains the timeout if perception is slow or unavailable.
6. Checkpoint confirmed objects through `/sam2/save_stored_objects`.
7. Resume exploration and repeat.
8. When frontier exploration reports exhaustion, snapshot `/map`, request the
   Nav2 global costmap, and switch to adaptive predicted-visibility coverage.
9. Navigate to reachable viewpoints selected for new free-space and obstacle-
   boundary visibility, perform the same six-step scan, checkpoint, and
   repeat until the configured predicted-coverage ratio is reached.

The registry assumes observed objects are static. The program does not create a
second object recorder; it coordinates motion around the continuously running
annotator and registry, then explicitly persists the registry after each
completed observation cycle.

Run through frontier exploration and post-frontier predicted visibility:

```bash
ros2 action send_goal /explore_and_record \
  muto_command_layer/action/ExploreAndRecord \
  "{exploration_duration: 0.0, observation_duration: 0.0, scan_step_count: 0, max_cycles: 0}" \
  --feedback
```

Zero-valued durations and scan step count select the configured defaults. The
step angle is always `360 / scan_step_count`; the default is six 60-degree
steps. `exploration_cycle_duration` is a minimum rather than a travel timeout,
so a frontier trip already in progress may extend a cycle. `observation_duration`
is a per-step maximum: observation normally ends as soon as
`observation_min_detection_frames` fresh detector results arrive. Set that
frame count to zero to restore a fixed-duration dwell. A nonzero `max_cycles`
makes a bounded run, where one cycle is one full
360-degree scan during frontier exploration. It completes before the
post-frontier phase if that limit is reached. With the default `max_cycles: 0`,
frontier exhaustion starts predicted-visibility coverage. Canceling the action
cancels any active navigation or spin, stops exploration, and makes a
best-effort final registry checkpoint.
`/go_to_object` goals and direct `/explore` calls are rejected while this
program owns navigation.

### Automatic mission bag

The standalone `muto_exploration_bag` package opens one MCAP rosbag for each
`/explore_and_record` goal and finalizes it after success, cancellation, or
failure. `command_layer_launch.py` starts that recorder by default; this
command node only publishes mission lifecycle events and waits for the
recorder-ready acknowledgement. By default the recorder captures all
discovered topics, hidden action topics, and available service-event topics.
Metadata stores the action goal context, recorder git revision, and dirty flag.

The default parent directory is
`$HOME/.ros/bags/explore_and_record`; on the root-run robot this is
`/root/.ros/bags/explore_and_record`. The exact path is logged and published
transient-local:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /explore_and_record/last_bag_path
ros2 bag info /root/.ros/bags/explore_and_record/muto_explore_<timestamp>_<goal>
```

Replay the complete bag on a development machine or an isolated ROS domain:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock
```

Do not replay the unfiltered all-topic bag on the live robot domain: it may
contain motion and command topics. Use `ros2 bag play --topics ...` when only a
specific sensor or diagnostic path is needed.

`/explore_and_record/recording_event` marks recording start, terminal outcome,
and resolved action settings inside the bag. Use the separate operator topic
for a short manual note—JSON is unnecessary:

```bash
ros2 topic pub --once /explore_and_record/operator_event std_msgs/msg/String \
  "{data: 'observation: chair visible left of the doorway'}"
```

The default all-topic mode includes raw camera and point-cloud streams and can
grow quickly. For longer deployments, set an explicit `topics` list in
`muto_exploration_bag/config/exploration_bag.yaml`, or pass
`exploration_bag_topics_regex` and `exploration_bag_exclude_regex` to the
command launch.
`exploration_bag_enabled:=false` omits the standalone recorder from the command
launch. `exploration_bag_required:=true` makes a missing/error recorder-ready
acknowledgement abort the mission instead of continuing with a prominent
error. See the
[`muto_exploration_bag` README](../muto_exploration_bag/README.md) for the
standalone launch and operator-event quick reference.

Post-frontier coverage is a geometric prediction over two aligned layers. The
latest transient-local `/map` defines free-space and occupied-boundary targets.
Nav2's master global costmap defines traversable candidate cells and
connectivity after its static, obstacle, footprint, and inflation processing.
The command resamples costmap values at occupancy-cell centers, so the grids
may have different origins or dimensions. Cells outside the costmap and costs
above `visibility_maximum_cost` are blocked. Nav2 remains authoritative for
final path planning and collision checking. A rejected pose is discarded
without receiving predicted-visibility credit.

After Nav2 reaches a viewpoint and all configured spin steps complete, the
planner credits the free and occupied-boundary cells that its 2-D line-of-sight
model predicts were visible. The command does not use RGB delivery, depth
points, detections, or registry changes as coverage evidence. Camera and
perception failures can therefore reduce actual observations without reducing
the reported estimate. Detector messages shorten the observation wait, but a
timeout warns and proceeds rather than claiming or denying coverage.

Completion requires both predicted observable free-space coverage and
predicted observable boundary coverage to reach
`visibility_completion_ratio`. The default `0.98` means 98 percent of cells
predicted coverable from the generated candidate set; it is not a measured
camera-coverage or detector-recall guarantee. The separate map-coverage value
in logs compares credited free cells with all connected target free cells and
is diagnostic only.

The prediction model is deliberately 2-D and assumes registered objects do not
move during the mission. It prioritizes newly visible occupied boundaries,
where static objects and room structure are likely to appear. Each selected
pose is published transient-local on `/explore/visibility_target_pose` for
RViz inspection.

## Go-to-object pipeline

```text
GoToObject(object_id: "chair_2")
             |
             v
/sam2/get_stored_objects  -- exact StoredObject.name lookup
             |
             v
registry centroid -- TF --> map
             |
             +---- /global_costmap/get_costmap
             +---- current map -> base_frame TF
             |
             v
search the robot-reachable component of Nav2's master costmap
             |
             v
nearest reachable ring cell at or beyond the required radius
             |
             v
/navigate_to_pose (Nav2)
```

The registry's unique `StoredObject.name` is the persistent object ID. Examples
are `chair` and `chair_2`; a YOLO class label alone is not an ID.

The command requests Nav2's current master global costmap, whose static,
obstacle, and inflation layers already encode the configured footprint and
clearance. Raw costs through `approach_maximum_cost` are eligible; the default
`252` keeps Nav2's inscribed (`253`), lethal (`254`), and unknown (`255`) cells
blocked. It computes the robot-reachable component without diagonal corner
cutting. Starting at `max(approach_distance, approach_robot_radius)`, it
searches outward in costmap-resolution rings and selects the shortest reachable
candidate in the first populated ring. The target yaw faces the object. Object
height is deliberately omitted from the planar Nav2 goal.

This command means “reach a peripheral pose and face the stored object.” It does
not yet perform visual re-identification after arrival. If no suitable costmap
cell exists, the command aborts before Nav2 dispatch. Nav2 remains authoritative
for final path planning, smoothing, and execution, and can reject the selected
cell if its costmap changes after the service snapshot.

## Launch and call

The command-layer launch now owns its object-identification lower layer. Nav2
must still be running separately before navigation commands can succeed:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer command_layer_launch.py
```

List the registry first if the ID is unknown:

```bash
ros2 service call /sam2/get_stored_objects \
  sam2_object_registry/srv/GetStoredObjects "{name: '', label: ''}"
```

Send a goal and show progress:

```bash
ros2 action send_goal /go_to_object \
  muto_command_layer/action/GoToObject \
  "{object_id: 'chair_2'}" --feedback
```

Only one command is accepted at a time. Canceling this action forwards a cancel
request to its active Nav2 goal. A separate Nav2 client, such as frontier
exploration, can still preempt or compete with this package; call `/explore`
with `data: false` before issuing a go-to-object command.

The computed goal is also published with transient-local durability on
`/object_navigation/target_pose` for RViz inspection.

## Natural-language command router

`/natural_language_command` accepts one natural-language request and asks the
VLM to classify it into a fixed command enum. The VLM does not select ROS names
or execute code. Its strict JSON response is validated locally, including all
argument types and bounds, before the router can call an existing typed
service/action.

Supported commands are:

- `find_object`
- `find_something`
- `go_to_object`
- `start_exploration`
- `stop_exploration`
- `explore_and_record`
- `save_map`
- `cancel_active_command`
- `unsupported`, which never dispatches anything

Send a request through the normal object pipeline:

```bash
ros2 action send_goal /natural_language_command \
  muto_command_layer/action/NaturalLanguageCommand \
  "{query: 'go to the red chair'}" --feedback
```

For `go_to_object`, the router first calls `/find_object` with the interpreted
description. Navigation is dispatched only when exactly one static registry ID
matches; no VLM-generated ID is sent directly to navigation. For
`find_object`, the action waits for search completion and returns the exact IDs.

Long-running `find_something`, `go_to_object`, and `explore_and_record`
commands return once the typed child action accepts the goal. This leaves the
router available for a later request such as `cancel the active command`; the
child result is tracked asynchronously. Manual exploration start/stop waits for
the `/explore` service response, while map saving waits for a bounded
`/save_map` result. The router uses a two-thread executor so one thread can wait
on a bounded child operation while the other processes ROS progress and cancel
responses.

## Find-object pipeline

```text
FindObject(prompt)
       |
       v
read all stored IDs, labels, and class IDs from the registry
       |
       v
VLM pass 1: text only, return an exact-ID shortlist
       |
       +-- zero candidates --> successful no-match result
       |
       +-- one candidate ---> publish ID + metadata-based description
       |
       +-- multiple candidates
                  |
                  v
        load each candidate's stored JPEG
                  |
                  v
        VLM pass 2: tagged ID/JPEG pairs + original prompt
                  |
                  v
        publish final exact IDs + visual descriptions
```

The first pass never receives images or filesystem paths. It receives only the
user prompt and a JSON inventory containing each registered object's exact ID,
YOLO label, and class ID. The command layer sends a strict provider JSON Schema
whose ID enum contains only the current registry IDs. It then requires the
response to be exactly one JSON object and checks every returned ID against the
same registry snapshot before it can proceed.

Visual refinement happens only when the shortlist contains more than one
candidate. The command layer reads the representative `image_path` stored by
the registry and sends ordered `candidate ID tag -> JPEG` pairs through
`muto_vlm_socket`. By default, a missing, oversized, or invalid candidate JPEG
aborts an ambiguous search so candidates are not silently discarded.
This pass receives a separate strict schema limited to the IDs whose JPEGs were
actually included.

Every final result is returned in the `FindObject` action result and published
individually as `muto_command_layer/msg/ObjectMatch` on
`/object_search/matches`. Each message contains a generated `query_id`, rank,
batch total, exact registry ID, registry label, and VLM-produced description.

The command-layer launch starts the VLM socket automatically. To run only the
lower YOLO/SAM 2, registry, and VLM layer without command actions, launch:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer object_pipeline_launch.py
```

Search using natural language and inspect the per-object publications:

```bash
ros2 action send_goal /find_object \
  muto_command_layer/action/FindObject \
  "{prompt: 'find the chair with a red back'}" --feedback

ros2 topic echo /object_search/matches \
  muto_command_layer/msg/ObjectMatch
```

The registry and VLM socket remain separate dependency nodes. If either is
unavailable, the action aborts with a bounded error instead of waiting forever.

## Active find-something pipeline

`/find_something` composes the existing registry search and autonomous mission;
it does not add another detector or navigation algorithm:

1. Call `/find_object` before moving.
2. Return immediately when an already registered static object matches.
3. Otherwise start `/explore_and_record` with its configured defaults.
4. Re-run `/find_object` whenever `/sam2/stored_objects` reports a changed set
   of confirmed IDs.
5. Cancel `/explore_and_record` and return its exact `ObjectMatch` records when
   a match appears.
6. If predictive exploration and visibility coverage finish first, make one
   final registry query and return a successful no-match result.

```bash
ros2 action send_goal /find_something \
  muto_command_layer/action/FindSomething \
  "{prompt: 'find a red mug'}" --feedback
```

The registry is explicitly a static-object model. The command can revisit the
map to discover a missing object, but it does not track moving targets and does
not prove camera coverage. It rechecks only when confirmed registry IDs change,
so repeated observations of the same object do not repeatedly invoke the VLM.
Finding and approaching are separate operations; use `/go_to_object` with one
returned ID when movement to the object is wanted.

## Important parameters

- `approach_distance`: minimum center-to-centroid approach radius in metres;
  default `0.75`.
- `global_costmap_service`: Nav2 master-costmap query service; default
  `/global_costmap/get_costmap`.
- `approach_robot_radius`: lower bound for the object-centered approach ring;
  default `0.16`, matching the checked-in Nav2 robot radius. It does not
  duplicate Nav2 footprint inflation.
- `approach_maximum_cost`: largest raw Nav2 cost considered traversable;
  default `252`.
- `approach_start_snap_distance`: maximum distance used to recover the current
  robot pose onto a traversable costmap cell; default `0.5` metres.
- `global_frame`: Nav2 planning frame; default `map`.
- `robot_base_frame`: robot frame used to seed costmap reachability; default
  `base_frame`.
- `registry_timeout`, `nav_server_timeout`, `global_costmap_timeout`, and
  `tf_timeout`: bounded dependency waits in seconds.
- `navigation_timeout`: complete-goal timeout; `0.0` means unlimited.
- `map_save_directory` and `default_map_name`: confined occupancy-map output
  location and empty-request basename; defaults are `$HOME/.ros/maps` and
  `muto_map`.
- `save_map_timeout`: total wait for `/slam_toolbox/save_map`; default `10`
  seconds.
- `behavior_tree`: optional Nav2 BT XML path; empty uses Nav2's configured
  default tree.
- `visibility_coverage_enabled`: run adaptive predicted-visibility viewpoint
  coverage after frontier exhaustion; default `true`.
- `visibility_maximum_cost`: largest resampled Nav2 cost considered
  traversable; default `252`.
- `visibility_candidate_spacing` and `visibility_range`: geometry used to
  sample candidates and predict 2-D line-of-sight coverage; defaults `0.5`
  and `2.5` metres. `visibility_robot_clearance` is the standalone fallback
  when no navigation-cost layer is supplied to the planner library.
- `visibility_completion_ratio`: required predicted observable free-space and
  boundary coverage; default `0.98`. It does not measure camera or detector
  success.
- `visibility_max_viewpoints`: optional hard viewpoint limit; `0` is unlimited.
- `vlm_action` and `vlm_model`: child action endpoint and optional model
  override for object search and command interpretation.
- `max_query_characters`, `max_object_query_characters`, and
  `max_map_name_characters`: local input/output limits for natural-language
  routing; defaults `4096`, `1024`, and `128`.
- `max_exploration_duration`, `max_observation_duration`,
  `max_scan_step_count`, and `max_cycles`: bounds applied both in the VLM JSON
  Schema and again by the local command parser.
- `max_shortlist_size`: maximum number of metadata candidates allowed into
  visual refinement; default `8`.
- `require_all_candidate_images`: abort refinement when any shortlisted image
  is unavailable; default `true`.
- `vlm_result_timeout`: upper bound for either VLM inference pass; default
  `180` seconds.
- `log_vlm_judgements`: emit validated metadata-shortlist and visual-filter
  decisions as single-line JSON; default `true`.
- `max_log_description_characters` and `max_log_filtered_ids`: bound judgement
  log volume; defaults are `240` and `32`.
