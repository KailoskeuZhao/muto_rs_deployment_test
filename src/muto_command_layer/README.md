# Muto command layer

`muto_command_layer` exposes robot-level commands above perception, the object
registry, the VLM socket, and Nav2. It provides `/find_object` registry lookup,
persistent model-supervised `/look_for_object` missions, `/go_to_object`
navigation, `/explore` start/stop, a sanitized `/save_map` wrapper, and a
validated `/natural_language_command` action. The older `/find_something` and
`/explore_and_record` endpoints remain available as compatibility interfaces,
but they are no longer normal natural-language or commander choices.

The command reform is now active. The commander schedules independent bounded
primitives instead of one fixed exploration program:

```text
verify_registry | refine_registry_selection | explore_frontier
navigate_to_observation_poi | rotate | observe
checkpoint_registry | approach_object | wait | finish_not_found
        -> record result -> update mission state -> choose again
```

The default launch also arms a mission-wide command bag and a child-scoped
exploration bag. Together they are the normal high-level monitor, but they open
files only during relevant missions rather than recording continuously. See
[Default Bags And Mission Monitoring](../../docs/bags.md) for scope, paths,
retention, manual notes, the default Nav2 session bag, and odometry captures.

`/command_primitives/explore_frontier` performs bounded frontier travel and
confirms it stopped. `rotate` sends one bounded executable yaw command and
verifies the achieved angle from `/odometry/filtered`; `observe`
holds the robot stationary while fresh `/sam2/detection_heartbeat` messages
arrive, and
`checkpoint_registry` calls `/sam2/save_stored_objects`. Rotation, observation,
and persistence are no longer hidden inside one model-visible scan command.
The heartbeat contains only the detector frame header. The commander subscribes
to it only for the lifetime of `observe`; the bounded observation clock starts
after the first heartbeat confirms DDS matching. It never deserializes JPEG-bearing
`/sam2/detections` messages and has no detector subscription while idle.

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

This default is a fresh-map startup and clears persisted object records. When
the navigation stack is restoring the corresponding saved map, preserve its
registry explicitly:

```bash
ros2 launch muto_command_layer command_layer_launch.py load_existing_map:=true
```

Credentials remain environment-only. The launch shares the annotator detection
and point-cloud topics with the registry, and shares the registry service and
VLM action endpoints with the command layer. Each included package is scoped so
similarly named child launch arguments cannot leak into another package.

The deployment-level VLM settings live in
`config/object_pipeline_vlm.yaml`. It contains the provider URL, wire API,
default model, request limits, and the *name* of the credential environment variable.
The checked-in profile selects the tested `hkuproxy` chat-completions endpoint,
`gpt-5.6-sol`, and `HKU_API_KEY`. `vlm_model` remains the VLM socket default
and object matching keeps `object_search_vlm_model:=gpt-5.6-sol` by default.
Natural-language routing uses
`natural_language_vlm_model:=gpt-5.3-codex-spark`, while persistent commander
planning keeps `model_commander_vlm_model:=gpt-5.6-luna`. Select another
provider file with `vlm_params_file:=/absolute/path/to/vlm.yaml`; when doing
so, also override `vlm_base_url`, `vlm_wire_api`, and the relevant model
arguments.

> **Transport warning:** the current proxy URL uses plain HTTP. Although the
> key is absent from launch arguments and YAML, its bearer header is not
> encrypted in transit. `/look_for_object` also sends fresh full-camera JPEGs
> for model inspection, in addition to selected registry crops used by
> `/find_object`. Use this endpoint only through a trusted network, VPN, or
> secure tunnel, or replace it with an HTTPS endpoint.

The included processes are launched together; dependency readiness is checked
when an action goal is executed rather than through arbitrary startup delays.
The `/go_to_object` action requires Nav2's global-costmap service, navigation
action, and global/base TF.
`/find_object` requires the registry and VLM socket; camera and TF inputs are
additionally needed to create new registry entries. `/look_for_object` also
requires fresh RGB frames whenever a registry miss needs a new model scheduling
decision. The Muto-configured frontier explorer is started in cold idle so
`/explore` can activate it
without process startup latency.

`muto_nav2_pipeline_launch.py` independently owns hardware, localization,
mapping, and Nav2. It does not start the annotator, registry, VLM socket, or
command layer. For a complete deployment, run the Nav2 and object pipelines
alongside each other:

```bash
# Terminal 1: hardware, localization, mapping, and Nav2.
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py

# Terminal 2: perception, registry, VLM, and the command stack.
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
5. After every step, wait for three fresh `/sam2/detection_heartbeat` messages.
   Three seconds remains the timeout if perception is slow or unavailable.
6. Checkpoint confirmed objects through `/sam2/save_stored_objects`.
7. Resume exploration and repeat.
8. When frontier exploration reports exhaustion, snapshot `/map`, request the
   Nav2 global costmap, and switch to adaptive predicted-visibility coverage.
9. Query the visibility calculator for current coverage and ranked observation
   points of interest, navigate to the selected point, perform the same
   six-step scan, checkpoint, and repeat until the configured
   predicted-coverage ratio is reached.

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

### Automatic exploration child bag

The standalone `muto_exploration_bag` package opens one child-scoped MCAP for
each compatible exploration action or primitive and finalizes it after success,
cancellation, or failure. The separate parent `muto_command_bag` remains the
overall monitor for a complete commander mission; this exploration bag is the
deeper record of one child interval. `command_layer_launch.py` starts both
recorder processes by default, but neither writes while idle. The command node
only publishes mission lifecycle events and waits for the recorder-ready
acknowledgement. By default the exploration recorder discovers the graph but
filters out raw camera images, derived mask/annotation images, and point clouds.
It retains navigation, odometry, hidden action, structured object result,
lifecycle, operator-event, and available service-event topics.
Each bag's `muto_recording_manifest.json` stores the action goal context, ROS
distribution, recorder Git revision, dirty flag, and topic scope. Newer
rosbag2 releases also copy these fields into `metadata.yaml`; ROS 2 Humble does
not provide that storage API. Humble also lacks the newer `all_services`
selector, but all-topic mode records service-event topics exposed by server
introspection as ordinary topics.

The default parent directory is `/opt/muto_rs_ws/bags`. The exact path is
logged and published transient-local:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /explore_and_record/last_bag_path
ros2 bag info /opt/muto_rs_ws/bags/muto_explore_<timestamp>_<goal>
```

Replay the complete bag on a development machine or an isolated ROS domain:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock
```

Do not replay a complete mission bag on the live robot domain: it may contain
motion and command topics. Use `ros2 bag play --topics ...` when only a specific
sensor or diagnostic path is needed.

`/explore_and_record/recording_event` marks recording start, terminal outcome,
and resolved action settings inside the bag. Use the separate operator topic
for a short manual note—JSON is unnecessary:

```bash
ros2 topic pub --once /explore_and_record/operator_event std_msgs/msg/String \
  "{data: 'observation: chair visible left of the doorway'}"
```

The default compact profile excludes raw camera images, derived SAM2 image
outputs, and point clouds. Clear `exploration_bag_exclude_regex` only for a
dedicated perception-replay capture; use `exploration_bag_topics_regex` or an
explicit `topics` list for an even narrower contract.
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

The visibility component is a calculator, not a fixed mission script. For the
current robot cell, it reports coverage statistics and ranks observation
points of interest by predicted new free-space and occupied-boundary
visibility per travel-plus-scan cost. The same information is exposed through
`/command_layer/visibility_coverage`
(`muto_command_layer/srv/GetVisibilityCoverage`). The legacy
`/explore_and_record` action uses the top-ranked point for compatibility, but
a higher-level commander can query the report to decide whether to observe,
move elsewhere, or defer.

The model commander treats this report as optional post-progress context. It
does not query coverage before the first frontier step because a mapped
line-of-sight estimate is not evidence that the current object-search mission
has explored anything. Later context queries are bounded to 0.5 seconds and
run before camera capture, so a missing or slow costmap cannot invalidate the
fresh JPEG used for the scheduling decision. A model-selected
`navigate_to_observation_poi` rechecks the helper immediately before motion
with the normal endpoint deadline.

Within one `/look_for_object` mission, every successful `observe` primitive
also records the measured map pose, camera heading, configured horizontal FOV,
and number of detector heartbeat frames. These bounded observation records are
replayed into each later coverage query. The resulting `covered_*` fields and
POI gains therefore describe remaining inspection work for that mission rather
than restarting from zero on every request. Observations with no detector
frames, an incompatible frame, or an invalid/unreachable pose are rejected and
counted separately. Starting a new mission starts a new inspection history.

Frontier exploration remains an unmodified independent map-expansion tool. The
commander summarizes its measured outcomes as `frontier_search`: untried,
productive, stalled, exhausted, or uncertain. The model compares that state
with remaining visibility POIs when choosing its next primitive; neither helper
silently dispatches the other.

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
VLM for a bounded mission spec, not a user-visible command enum. For an object
mission, the spec separates the target description from the desired end state:
report the confirmed object, or approach it after confirmation. The VLM does
not select ROS names, primitive commands, or code. Its strict JSON response is
validated locally before the router adapts the mission to the existing typed
service/action surface.

Supported mission types are:

- `locate_object`
- `locate_and_approach_object`
- `approach_known_object`
- `query_object_registry`
- `start_manual_exploration`
- `stop_manual_exploration`
- `save_current_map`
- `cancel_active_mission`
- `unsupported`, which never dispatches anything

Send a request through the normal object pipeline:

```bash
ros2 action send_goal /natural_language_command \
  muto_command_layer/action/NaturalLanguageCommand \
  "{query: 'go to the red chair'}" --feedback
```

For `approach_known_object`, the router first calls `/find_object` with the
interpreted description. Navigation is dispatched only when exactly one static
registry ID matches; no VLM-generated ID is sent directly to navigation. For
`query_object_registry`, the action waits for search completion and returns the
exact IDs.

Long-running object missions return once the typed child action accepts the
goal. This leaves the router available for a later request such as `cancel the
active command`; the child result is tracked asynchronously. To wait for the
complete model-supervised outcome, call `/look_for_object` directly. Manual
exploration start/stop waits for the `/explore` service response, while map
saving waits for a bounded `/save_map` result. The router uses a two-thread
executor so one thread can wait on a bounded child operation while the other
processes ROS progress and cancel responses.

Plain `find X`, `look for X`, and `search for X` all enter
`/look_for_object` as `locate_object` or `locate_and_approach_object`.
Registry-only lookup is deliberately explicit: `check registry for X` or
`query registry for X`. Unambiguous phrases such as `cancel the active command`
are recognized locally
and do not use the single-request VLM socket. This keeps cancellation available
while the model commander is planning. An object search followed by approaching
that same object is one declarative mission rather than two router-dispatched
commands. For example, `find a green chair and then go near it` enters
`/look_for_object` as `locate_and_approach_object`. The commander then
assembles smaller primitives internally. Unrelated compound requests remain
unsupported. The `NaturalLanguageCommand` result still includes the legacy
`command` field for CLI and integration compatibility; `arguments_json` and the
decision-event topic carry the mission spec fields.

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
       +-- one or more candidates
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

Visual refinement happens for every non-empty shortlist, including exactly one
candidate. This prevents an attribute-rich request such as a color from being
accepted from class metadata alone. The command layer reads each representative
`image_path` stored by
the registry and sends ordered `candidate ID tag -> JPEG` pairs through
`muto_vlm_socket`. By default, a missing, oversized, or invalid candidate JPEG
aborts the search so candidates are not silently discarded.
This pass receives a separate strict schema limited to the IDs whose JPEGs were
actually included. Requested visual attributes are mandatory. A general color
description must be dominant on the object's primary visible body or upholstery;
small hardware, armrests, reflections, tinted lighting, and nearby objects do
not qualify unless the request explicitly names that part. Occluded or ambiguous
candidates are rejected.

`FindObject` also accepts optional `candidate_ids`. When supplied, the server
skips whole-registry metadata shortlisting and visually refines only those
exact confirmed registry IDs using their stored JPEGs. The commander uses this
as `refine_registry_selection` when an approach mission has multiple confirmed
matches and needs one exact object ID before `/go_to_object`.

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

## Legacy find-something compatibility pipeline

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

This compatibility node is disabled by default
(`launch_active_object_search:=false`). New callers should use
`/look_for_object`.

## Persistent model-supervised search

`/look_for_object` is the highest command-layer object-search mission. The
`model_commander` node stays alive with the command stack, but model inference
is event-driven: it runs only when a mission needs a new decision. It does not
poll the model continuously.

The implementation is split by responsibility:

- `model_commander_node.py` owns ROS endpoints and coordinates the mission.
- `model_commander_protocol.py` owns the strict model input/output contract.
- `model_commander_config.py` owns parameter defaults and startup validation.
- `model_commander_inputs.py` owns bounded subscriptions and camera encoding.
- `model_commander_memory.py` owns pose deltas and bounded primitive records.
- `model_commander_errors.py` defines the failure and ownership taxonomy.

The support modules contain no scheduling policy. In particular, moving them
out of the node does not weaken the fail-closed rule: if an accepted moving
child cannot be confirmed stopped, the node keeps ownership latched and rejects
new missions until restart.

```text
check the current registry
          |
          +-- match + report completion ----------------> return it
          +-- match + approach completion --------------> retain exact ID
          |
          v
capture a newly received bounded RGB frame
          |
          v
VLM inspects frame + state and selects one bounded next primitive
  verify_registry | refine_registry_selection | explore_frontier
  navigate_to_observation_poi | rotate | observe
  checkpoint_registry | approach_object | wait | finish_not_found
          |
          v
monitor the child result and confirmed-object set
          |
          +-- frontier active ---> run one separate /find_object check
          |                         coalesce newer registry revisions
          |                           | no match --> keep moving
          |                           | match ----> stop, confirm, replan
          +-- target approach changed -> stop stale approach
          +-- rotate/observe active ---> coalesce revisions, finish the
          |                              bounded primitive, then recheck
          +-- child completed ---> recheck, then replan
          +-- 20 s elapsed ------> VLM inspects a fresh frame while the
                                    bounded child remains active
                                      | continue current command
                                      | stop, verify stopped, recheck, replan
```

The planner selects only from that bounded primitive enum.
`refine_registry_selection` reuses `/find_object` with caller-provided
candidate IDs, so stored registry JPEGs can narrow multiple confirmed matches
without searching outside that set. `approach_object` is unavailable until
the current registry revision contains exactly one confirmed target ID; camera
evidence alone can never unlock it. The
commander owns the `/go_to_object` child and records its target ID plus
before/after pose in the same primitive history. A registry change stops the
approach before replanning. `navigate_to_observation_poi` reuses the
read-only visibility helper: the model asks for a semantic move, but local code
re-queries `/command_layer/visibility_coverage` immediately before dispatch
and sends Nav2 only to the current top-ranked POI. Each operation otherwise
has one purpose: frontier travel, POI navigation, in-place rotation,
stationary detector dwell, or registry persistence.
Navigation to a visibility POI is canceled after 12 seconds without 0.05 m of
odometry progress. Two consecutive stalls abort the mission rather than
dispatching more POIs to a physically trapped robot.
The recent primitive history and outcome are returned in
the next mission-state request so the model may change order, defer, retry, or
choose another primitive. Every scheduling request contains
a newly received `/camera/color/image_raw` frame encoded as a bounded JPEG. The
strict response must include a short visual observation and one of
`not_visible`, `possible`, `likely`, or `unclear`. Local code owns every action
client, limits duration and planning/dispatch counts, and rejects
`finish_not_found` until the configured counts of stationary observations,
odometry-measured rotation, registry checkpoints, and translational travel are
followed by a fresh registry check. Frontier time alone is not coverage: a step
moving less than 0.10 m is reported as `no_spatial_progress`, and the default
no-match gate requires at least 0.50 m of measured travel. The other defaults
are four observations, one full turn, and one checkpoint. The primitive order
remains unrestricted. `possible`
  or `likely` target evidence
  forces another registry check before any further motion or no-match finish.
  Visual evidence never sets `found`; only `/find_object` can confirm a registry
  match. During `explore_frontier`, motion and registry verification use
  separate ROS action handles and are polled concurrently. Only one of each may
  be active; intermediate registry revisions are coalesced instead of stopping
  the robot for every newly confirmed unrelated object. A background match is
  never accepted directly: motion is first confirmed stopped and a normal
  foreground `/find_object` check confirms the latest registry snapshot.
  Accepted bounded `rotate` and stationary `observe` primitives likewise finish
  while identity revisions accumulate; their command result records the start
  revision, end revision, and coalesced-update count. The commander then runs
  the ordinary foreground registry check at that primitive boundary. Registry
  freshness remains a strict pre-dispatch condition, and target-specific
  `approach_object` is still canceled when its registry revision becomes stale.
  Therefore a target that never enters the YOLO/SAM registry cannot
  complete this mission from VLM pixels alone. The model cannot provide ROS
  names, poses, parameters, or code. Only one commander mission and one owned
  motion child run at a time.

While a motion primitive or a model-directed wait remains active, the VLM also
inspects a fresh forward-camera snapshot after each
`active_inspection_period` cooldown (20 seconds by default). Model latency is
additional; this is not a real-time 20 Hz/20-second control guarantee. Its
separate strict monitor response can only
`continue_current_command` or `interrupt_and_replan`. `possible` or `likely`
target evidence must interrupt. Local code then confirms the owned child has
stopped, checks `/find_object`, and only afterward invokes the normal planner.
The VLM is a strategic monitor, not a real-time collision controller; Nav2 keeps
that responsibility.

The raw-image subscription exists only while one snapshot is being acquired, so
the idle resident commander does not deserialize the full camera stream. Camera
and crop-free detector-heartbeat subscriptions are owned by dedicated
single-thread executors: create, spin, and destroy occur sequentially on the
same worker thread, outside the commander's multithread executor. Each worker
accepts one bounded request at a time, has no unbounded queue, and is canceled
without holding commander state locks. This prevents the Humble wait-set handle
race and bounds shutdown if an input stalls. Each inspection requires a frame
received after the preceding inspection. Ordinary 30 Hz camera updates neither
trigger inference nor invalidate an in-flight model request. "Fresh" here means
newly received with a bounded monotonic receipt age; it cannot detect a camera
driver that republishes frozen pixels under new messages. A missing, stale,
invalid, or oversized planning frame causes bounded no-motion retries. Repeated
active-monitor failures stop the owned motion and abort rather than letting it
run indefinitely without the promised monitor. One snapshot is the current
forward view, not proof of complete 360-degree visual coverage.

If the stop state of an accepted moving child cannot be confirmed, the
commander fails closed: it retains the ownership handle, latches its status as
`ownership_uncertain`, and rejects new missions until the commander process is
restarted. This prevents replacement motion from being dispatched beside a
possibly live old child.

```bash
ros2 action send_goal /look_for_object \
  muto_command_layer/action/LookForObject \
  "{prompt: 'the red mug beside the kettle', max_duration: 0.0, max_planning_steps: 0, completion_mode: 0}" \
  --feedback
```

`completion_mode: 0` reports the confirmed object; `completion_mode: 1`
requires the commander to approach the one unambiguous confirmed object before
success. Zeros select the configured finite defaults: 30 minutes and 64 model planning
steps. A no-match or budget-limited run is a completed action whose `found`
field is false; dependency or repeated model failures abort the action. Cancel
the direct action with `Ctrl-C`, or dispatch it through natural language and
send `cancel the active command`.

The commander's last state is published as bounded JSON with transient-local
durability:

```bash
ros2 topic echo /model_commander/status --once \
  --qos-durability transient_local
```

`/find_something` remains available as an opt-in deterministic rollback path. It
always runs one fixed registry-search then explore-and-record sequence and does
not ask the model to schedule steps.

Every dispatched frontier primitive retains the existing action-scoped
bag handshake. In addition, the standalone `muto_command_bag` recorder opens
one parent `muto_command_<timestamp>_<mission-id>` MCAP before the initial
registry check and closes it only when the complete `/look_for_object` mission
finishes. It therefore spans planning, waits, registry checks, all scheduled
primitives, replanning, cancellation, and terminal status.

The parent bag retains append-only JSON on
`/model_commander/decision_event`, the exact bounded JPEG supplied for each
inspection on `/model_commander/inspected_image`, commander status, primitive
and object-search action traffic, registry changes, navigation context, and
logs. The decision trace includes the mission blackboard given to the model,
the validated response, model latency, command outcomes, and the original
natural-language decision event when the mission came through the router.
Continuous raw camera images and high-bandwidth point-cloud/mask products stay
excluded. The latest parent path is published on
`/model_commander/last_bag_path`. The command recorder keeps the newest 20
recognized Muto bag directories in the parent directory by default. Set
`command_bag_max_directories:=0` to disable pruning.

This parent bag is the default overall command monitor. It is mission-scoped,
not a continuous system-wide recorder. Direct `/go_to_object`, `/explore`,
joystick, or arbitrary Nav2 goals are instead retained by the compact Nav2 bag
started with the normal Nav2 pipeline. See
[Default Bags And Mission Monitoring](../../docs/bags.md) for that boundary.

Add a manual mission observation with:

```bash
ros2 topic pub --once /model_commander/operator_event std_msgs/msg/String \
  "{data: 'observation: target-like chair visible beside the white desk'}"
```

## Important parameters

- `approach_distance`: minimum center-to-centroid approach radius in metres;
  default `0.75`.
- `global_costmap_service`: Nav2 master-costmap query service; default
  `/global_costmap/get_costmap`.
- `approach_robot_radius`: lower bound for the object-centered approach ring;
  default `0.26`, matching the checked-in Nav2 walking-envelope radius. It does not
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
- `visibility_coverage_service`: read-only coverage-state and observation-POI
  service; default `/command_layer/visibility_coverage`.
- `visibility_coverage_max_points`: maximum ranked observation POIs returned
  by one service query; default `8`.
- `visibility_completion_ratio`: required predicted observable free-space and
  boundary coverage; default `0.98`. It does not measure camera or detector
  success.
- `visibility_max_viewpoints`: optional hard viewpoint limit; `0` is unlimited.
- `vlm_action`: shared GenerateVlm action endpoint.
- `vlm_model`: VLM socket default model.
- `object_search_vlm_model`: model used by registry object matching; default
  `gpt-5.6-sol`.
- `natural_language_vlm_model`: faster command interpretation model; default
  `gpt-5.3-codex-spark`.
- `model_commander_vlm_model`: persistent command-planning model; default
  `gpt-5.6-luna`.
- `default_max_duration` and `default_max_planning_steps`: finite defaults for
  `/look_for_object` goals that request zero; defaults are `1800` seconds and
  `64` planning steps. Hard caps are `7200` seconds, `256` planning steps, and
  `128` child-command dispatches.
- `max_exploration_cycles`, `max_wait_seconds`, planner retry limits, command
  failure limits, and repeated-no-progress limits: local bounds on every model
  decision and recovery path. Background registry checks consume the same
  command-dispatch budget; they are not an unbounded side channel.
- `visual_observation_topic`, `visual_observation_timeout`, and
  `visual_observation_max_age`: camera source and freshness requirements for
  every model planning step; defaults are `/camera/color/image_raw`, `5.0`, and
  `2.0` seconds.
- `visibility_context_timeout`: deadline for optional coverage context added
  before a planning snapshot; default `0.5` seconds. Initial planning skips
  the helper until frontier exploration or measured search progress exists.
- `visibility_observation_horizontal_fov_rad` and
  `visibility_max_observations`: geometry and bounded mission-memory capacity
  used to credit completed detector-backed `observe` primitives; defaults are
  `1.019272` radians (58.4 degrees) and `64` records.
- `visual_observation_jpeg_quality`, `visual_observation_max_width`,
  `visual_observation_max_height`, and `visual_observation_max_jpeg_bytes`:
  bound the transmitted planning snapshot; defaults are `80`, `960`, `720`,
  and `1048576` bytes.
- `active_visual_monitoring`, `active_inspection_period`,
  `active_inspection_timeout`, and `active_inspection_max_decision_age`: enable
  strategic visual checks during long commands and bound their rate, inference
  time, and result age; defaults are `true`, `20`, `30`, and `90` seconds.
- `max_consecutive_active_inspection_failures` and `max_visual_interrupts`:
  stop unmonitored motion after three consecutive monitor failures and bound a
  mission to eight model-requested stop/replan cycles.
- `visual_observation_max_source_width`,
  `visual_observation_max_source_height`, and
  `visual_observation_max_source_bytes`: reject unreasonable raw messages before
  direct `bgr8`/`rgb8`/`mono8`/`bgra8`/`rgba8` conversion; defaults are `8192`,
  `8192`, and `67108864` bytes.
- `max_query_characters`, `max_object_query_characters`, and
  `max_map_name_characters`: local input/output limits for natural-language
  routing; defaults `4096`, `1024`, and `128`.
- `max_rotation_radians`, `max_observation_seconds`, `spin_time_allowance`,
  `rotate_executable_yaw_velocity`, `rotate_timeout_reference_yaw_velocity`,
  `checkpoint_timeout`, and `observation_min_detection_frames`: local bounds
  for the separated rotate, observe, and checkpoint primitives. Rotate uses
  Nav2 Spin so the local costmap checks the 0.27 m effective footprint, then
  accepts completion only after odometry verifies the requested yaw.
- `navigation_progress_distance_m`, `navigation_no_progress_timeout`, and
  `max_consecutive_navigation_no_progress`: stop stalled POI navigation after
  `0.05 m`/`12 s` and abort after two consecutive stalls.
- `minimum_explore_progress_distance_m`: displacement required before a
  bounded frontier interval counts as useful travel.
- `minimum_useful_exploration_speed_mps`: blackboard threshold for measured
  frontier travel divided by requested exploration time; default `0.03 m/s`.
  Below it, the VLM still receives and interprets the current JPEG, but the
  prompt biases the next decision toward another bounded frontier attempt
  unless the image contains possible or likely target evidence.
- `minimum_no_match_travel_distance_m`, `minimum_no_match_observations`,
  `minimum_no_match_rotation_radians`, and
  `minimum_no_match_checkpoints`: measured evidence gates enforced before a
  model-requested no-match finish.
- `max_shortlist_size`: maximum number of metadata candidates allowed into
  visual refinement; default `7`. This keeps the tagged JPEG request within the
  VLM socket's 16-part limit.
- `require_all_candidate_images`: abort refinement when any shortlisted image
  is unavailable; default `true`.
- `vlm_result_timeout`: upper bound for VLM inference. Object search defaults
  to `180` seconds, model commander planning to `60`, and natural-language
  routing to `45`.
- `log_vlm_judgements`: emit validated metadata-shortlist and visual-filter
  decisions as single-line JSON; default `true`.
- `max_log_description_characters` and `max_log_filtered_ids`: bound judgement
  log volume; defaults are `240` and `32`.
