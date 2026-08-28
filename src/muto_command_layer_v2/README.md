# muto_command_layer_v2

This package is the active v2 command-layer contract and deterministic
mission-executive slice. The retired v1 command-layer package is not present
in the workspace; v2 owns the only mission command surface.

AI handoff: read
`docs/the-new-command-layer-8f3a1182-d276-4717-aff9-bab5f0bdee9c.md` and
`docs/commander_stack_reference.md` before changing this package. The latter
is the current v2 orientation note and must be checked against source and
runtime evidence.

The package currently contains:

- the v2 `Mission` action;
- the natural-language frontend path (an action goal with an empty
  `object_request` is normalized before executive acceptance);
- `MissionBoard`, `MissionEvent`, and `ReachabilityReport` messages;
- dependency-free typed contracts; and
- deterministic executive state-transition tests;
- a strict natural-language normalization boundary;
- a strict JSON commander decision parser;
- one static, skill-scoped typed tool dispatcher;
- conservative costmap reachability preflight;
- a lifecycle-managed high-level recorder that opens one mission-scoped bag,
  writes a manifest and recorder status, and closes on terminal outcome; and
- an event-driven commander/executive runtime loop;
- a strict `muto_vlm_socket` planner transport with schema-checked output;
- ROS authority adapters for registry snapshots, deterministic POI-grid
  observation,
  and Nav2 motion results;
- an independent ROS composition and validation launch;
- ROS projections and a single v2 mission action transport node.

For a v2-only Nav2 integration smoke in Humble, use
`ros2 launch muto_command_layer_v2 v2_nav2_sim_launch.py`. It starts the
reactive `v2_sim_plant_node.py` fixture and the independent
`muto_slam_mapping` Nav2 pipeline with hardware/localization/mapping disabled;
it never starts the legacy command layer. The VLM and registry authority
endpoints remain explicit launch parameters, while the POI-grid authority is
owned by v2, so this smoke does not
pretend that a missing backend is a successful mission.

For a supervised physical Humble smoke, use
`ros2 launch muto_command_layer_v2 v2_hardware_smoke_launch.py`. This is the
v2-only field composition: it enables the existing hardware/localization/SLAM
and Nav2 pipeline, launches the direct SAM2 annotator and object registry, and
starts the VLM socket plus v2 executive with its map-backed POI-grid planner.
It does not include the retired command-layer launch or an external search
process. Keep the robot lifted or in a clear test area until the
readiness gates pass and the first mission is intentionally sent.

The launch starts the v2 mission recorder by default and exposes the existing
odometry input recorder as an opt-in:

- the v2 high-level recorder writes only bounded mission/POI diagnostics
  (board, events, rejection, selected POI pose, typed POI results, recorder
  status, and manifest) to a unique MCAP directory under
  `/opt/muto_rs_ws/bags/muto_command_v2_<run>_<mission>`; and
- `muto_odometry_bag` is available as an opt-in lower-level diagnostic
  (`record_odometry_bag:=true`) and does not poll motor angles by default
  (`odometry_record_motor_angles:=false`), so the normal smoke capture does
  not add sensor subscriptions or blocking controller reads to the gait loop.

Override `bag_output_uri`, `bag_run_id`, or `odometry_bag_path` when a test
needs a known destination. `HKU_API_KEY` must be present for the configured
VLM provider, and the SAM2 checkpoint/model paths must exist in the robot
container. This launch is a smoke harness, not a claim that hardware,
network, perception, or Nav2 are healthy; inspect readiness-gate output,
`/muto/mission_board`, `/muto/mission_event`, and the high-level bag (plus
the optional odometry or Nav2 diagnostic bags when enabled with
`record_odometry_bag:=true` or `launch_nav2_bag:=true`).

Use the installed `v2_nav2_smoke.py` client for live motion checks instead of
`ros2 action send_goal` when `/clock` is active. The client explicitly follows
sim time, gates on `/bt_navigator` reaching `active`, and reports the real
`NavigateToPose`/`Spin` result status:

```bash
ros2 run muto_command_layer_v2 v2_nav2_smoke.py --goal-x 1.5 --goal-y 0.0 --timeout-s 90
ros2 run muto_command_layer_v2 v2_nav2_smoke.py --spin-yaw 3.141592653589793 --timeout-s 60
```

This helper is only a Nav2 diagnostic client. It does not publish velocity,
perform reachability classification, or replace the v2 motion authority.

For object-position sandbox cases, use `v2_navigation_sandbox.py`. It reports
the live map/TF preflight and then the real v2 motion-authority result. An
object on an occupied cell should return `unreachable` without dispatching a
Nav2 goal; `--projection-policy allow` explicitly tests the selected-pose
projection path:

```bash
ros2 run muto_command_layer_v2 v2_navigation_sandbox.py \
  --target-x 0.6 --target-y 0.0 --object-id chair_inside_obstacle
ros2 run muto_command_layer_v2 v2_navigation_sandbox.py \
  --target-x 0.6 --target-y 0.0 --object-id chair_inside_obstacle \
  --projection-policy allow
```

The sandbox and v2 composition use a 0.26 m footprint radius by default,
matching the Nav2 costmap and reactive plant. If a deployment changes the
robot footprint, pass the same value as both `robot_radius_m` and
`footprint_radius_m`; otherwise preflight can disagree with Nav2 at narrow
gaps.

The production composition also gates motion dispatch on the configured Nav2
lifecycle state service (active by default at `/bt_navigator/get_state`). This
prevents an action server discovered during startup from being mistaken for a
ready navigation authority; isolated transport tests may leave that option
empty when they provide their own readiness fixture.

The ROS action/message interfaces, authority transport, and high-level
rosbag2 recorder are verified in a ROS 2 Humble container. The recorder uses
an explicit bounded allowlist: the mission board/events/manifest/status,
typed mission rejection, selected POI pose, and typed POI result. It does not
subscribe to raw camera,
LiDAR, IMU, scan, or point-cloud streams, and recorder failures are non-fatal
to a mission. The POI-grid planner only selects known-free reachable map
cells and hands the selected pose to Nav2; it does not replace Nav2's planner,
controller, recovery, or obstacle avoidance. A full scenario launch that
connects a real VLM, registry, POI grid, and Nav2 remains a cutover gate. The
standalone transport node fails closed with
`commander_unavailable` until independent commander and backend instances are
injected by deployment composition.

The Humble graph test also runs the real v2 composition with deterministic ROS
authority servers: natural-language input reaches the VLM action, registry
name/label shortlist, stored-candidate inspection, and terminal mission result.
The mission action endpoint is configurable through the `mission_action`
launch argument so v2 can be validated side-by-side before cutover.

The commander may switch from search to approach only after a current-revision
candidate confirmation is on the board; the parser rejects unconfirmed or
arbitrary skill changes. Confirmation evidence is source-tagged and carries an
evidence id, observation time, and registry revision, so a late inspection
result cannot replace a newer shortlist. Nav2 remains the navigation and
obstacle-avoidance authority; the v2 executive only performs a fresh,
conservative reachability preflight and records the resulting report. The
reachability revision changes when grid contents change, while repeated
unchanged map timestamps only refresh freshness; this keeps the final
preflight-to-Nav2 revision check useful on live costmaps.

Tool permissions are skill-scoped: search may query/inspect, observe through
the POI grid, or rotate; approach may rotate or send the exact confirmed
candidate to `go_to_point`. Search cannot select a raw coordinate, and approach
cannot start another POI search.

For an approach tool call, the confirmed candidate ID is mandatory. If the
model omits a raw point, the adapter resolves that candidate's map position
from the same registry revision and permits only the deterministic preflight
projection policy; it does not perform a new lookup or choose another object.

Registry revisions use the same stability rule: identity, label/class,
evidence path, and centimetre-scale pose are semantic revision inputs;
detector ``last_seen`` timestamps, observation counters, and millimetre-scale
pose jitter are freshness/noise and do not invalidate a same-object visual
decision or interrupt a POI-grid step. A repeated lookup with the same
revision preserves prior rejection/confirmation evidence; only a genuinely
new shortlist revision resets it.

For deployments, set `scenario_id` and `scenario_completion_policy` in the
launch configuration. The policy is scenario/test metadata, not a model choice:
`report_confirmed` ends after confirmation, `approach_confirmed` requires a
successful approach, and `search_until_exhausted` requires the configured
search completion signal. Invalid or impossible individual decisions are
recorded as non-fatal rejections when they can be replanned; cancellation is
propagated to the active authority and produces a terminal canceled mission.
