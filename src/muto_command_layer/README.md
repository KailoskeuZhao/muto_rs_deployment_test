# Muto command layer

`muto_command_layer` exposes robot-level commands above perception, the object
registry, the VLM socket, and Nav2. It provides cancellable `/go_to_object` and
`/find_object` actions.

See [Object pipeline functions](docs/object_pipeline_functions.md) for the
complete functional interface, runtime dependencies, operator commands, and
degraded-operation behavior of `object_pipeline_launch.py`.

## Complete object pipeline launch

Start the SAM 2 image annotator, object registry, VLM socket, and command layer
from one launch file:

```bash
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer object_pipeline_launch.py
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
The `/go_to_object` action still requires Nav2 to be running. `/find_object`
requires the registry and VLM socket; camera and TF inputs are additionally
needed to create new registry entries.

`muto_nav2_pipeline_launch.py` independently owns hardware, localization,
mapping, and Nav2. It does not start the annotator, registry, VLM socket, or
command layer. For a complete deployment, run the Nav2 and object pipelines
alongside each other:

```bash
# Terminal 1: hardware, localization, mapping, and Nav2.
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py

# Terminal 2: perception, registry, VLM, and both object commands.
export HKU_API_KEY='your-key'
ros2 launch muto_command_layer object_pipeline_launch.py
```

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
             +---- current map -> base_frame TF
             |
             v
robot-facing standoff pose, oriented toward the centroid
             |
             v
/navigate_to_pose (Nav2)
```

The registry's unique `StoredObject.name` is the persistent object ID. Examples
are `chair` and `chair_2`; a YOLO class label alone is not an ID.

The target is `approach_distance` metres from the 2-D object centroid on the
side currently facing the robot. Its orientation looks back at the centroid.
The object's measured height is deliberately not copied into the planar Nav2
goal. TF supplies all frame conversion; there is no hard-coded frame offset.

This command means “reach a peripheral pose and face the stored object.” It does
not yet perform visual re-identification after arrival. Nav2 remains responsible
for collision checking, planning, smoothing, and control. If the selected
standoff point is occupied or unreachable, Nav2 can reject or abort the goal.

## Launch and call

When the registry and Nav2 are already running, the command layer can also be
started by itself:

```bash
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
exploration, can still preempt or compete with this package; stop exploration
before issuing a go-to-object command.

The computed goal is also published with transient-local durability on
`/object_navigation/target_pose` for RViz inspection.

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

Start the VLM socket alongside the command layer:

```bash
export HKU_API_KEY='your-key'
vlm_share="$(ros2 pkg prefix muto_command_layer)/share/muto_command_layer"
vlm_params="${vlm_share}/config/object_pipeline_vlm.yaml"
ros2 launch muto_vlm_socket vlm_socket_launch.py \
  params_file:="$vlm_params" \
  base_url:=http://43.165.176.234:8080/ \
  wire_api:=responses \
  default_model:=gpt-5.6-sol
ros2 launch muto_command_layer command_layer_launch.py
```

The explicit provider arguments are required here because the lower-level
`muto_vlm_socket` launch has independent generic defaults. Prefer
`object_pipeline_launch.py` for the normal integrated deployment.

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

## Important parameters

- `approach_distance`: centroid standoff in metres; default `0.75`.
- `global_frame`: Nav2 planning frame; default `map`.
- `robot_base_frame`: robot frame used to select an approach side; default
  `base_frame`.
- `registry_timeout`, `nav_server_timeout`, and `tf_timeout`: bounded dependency
  waits in seconds.
- `navigation_timeout`: complete-goal timeout; `0.0` means unlimited.
- `behavior_tree`: optional Nav2 BT XML path; empty uses Nav2's configured
  default tree.
- `vlm_action` and `vlm_model`: child action endpoint and optional model
  override for object search.
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
