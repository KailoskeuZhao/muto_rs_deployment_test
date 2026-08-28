# Commander stack reference

This note describes the command stack currently mounted in the workspace.
It is an orientation aid for agents, not an authority above source, ROS
interfaces, launch wiring, installed artifacts, or runtime evidence.

## Active composition

The retired v1 `muto_command_layer` package is absent. The active command
surface is the independent v2 package:

```text
natural-language request
  -> v2 Mission action (/muto/mission)
  -> MissionExecutive
  -> CommanderAgent
  -> scoped typed tool
  -> independent registry/POI-grid/Nav2 authority
  -> MissionBoard + MissionEvent + high-level MCAP
```

The v2 composition owns a deterministic POI-grid search authority. It remains
cold until the executive requests one observation step, selects only a
known-free reachable map cell, publishes a typed result on
`/muto/poi_grid/result`, and waits for Nav2 to finish before Commander chooses
the next tool. Nav2 remains the final navigation and obstacle-avoidance
authority.

For a supervised robot composition use:

```bash
ros2 launch muto_command_layer_v2 v2_hardware_smoke_launch.py
```

When the production authorities are already running, the smaller composition
is:

```bash
ros2 launch muto_command_layer_v2 v2_command_layer_launch.py
```

Neither launch starts the retired v1 action servers or imports their state
machine. The v2 high-level recorder is sensor-free: it records mission board,
events, typed rejection, registry evidence, POI selection/result, and recorder
lifecycle into one unique MCAP per mission. Raw camera, LiDAR, IMU,
and point-cloud recording is opt-in through the dedicated diagnostic bags.

## Commander contract

The commander has exactly two skills:

- `search_for_object`: use registry lookup, revision-scoped candidate
  confirmation, one-goal POI observation/rotation steps, and search as needed;
- `approach_confirmed_object`: approach exactly one candidate confirmed for the
  current registry revision.

The commander-facing tool table is fixed and typed:

- `query_registry`
- `inspect_candidates`
- `observe`
- `rotate_to_heading`
- `go_to_point`

The table is statically scoped by skill. Search may query/inspect the registry,
observe through the deterministic POI grid, or rotate. Approach may rotate or
send the exact confirmed-candidate `go_to_point`; it cannot invoke POI search
or choose an arbitrary search coordinate.

The model receives the objective, completion policy, registry revision and
shortlist evidence, robot pose/readiness, current board/progress, recent tool
results, and a bounded camera snapshot. It selects a skill and at most one
tool call. It cannot publish raw ROS commands or mutate mission state.

The VLM transport keeps `tool` and `completion_proposal` as required nullable
fields because the deployed Chat Completions provider rejects a root-level
`oneOf`. The commander parser still rejects both fields being set (or neither
being set), so provider compatibility does not weaken the bounded decision
contract.

## Confirmation chain

Registry name/metadata matching creates only a shortlist. Every candidate
inspection is tied to the exact registry revision and stored-JPEG evidence.
The complete `object_request` is a conjunction of requirements: the object
class and every stated attribute (for example, `blue` **and** `chair`) must
match the candidate evidence. A class-only registry label is never sufficient.
The candidate inspector must return explicit matched and unmatched attributes;
`confirmed=true` is valid only when every requested term is matched and no
term is unknown or contradictory. At most one candidate may be confirmed. A
target is approach-eligible only when its confirmation revision equals the
board's current registry revision and its attribute evidence satisfies the
same request.

The executive and backend both enforce this rule. A malformed, incomplete, or
attribute-mismatching inspection is an ordinary nonfatal rejection, not a
confirmed target. The board and high-level recorder retain the matched and
unmatched attribute lists so an operator can audit why a candidate was (or
was not) promoted.

Repeated unchanged lookups preserve the existing evidence; they do not reset
the search or interrupt an active POI observation.
An approach `go_to_point` call must carry that exact confirmed candidate ID.
The normal form omits the raw point; the backend resolves the candidate's map
position from the same revision and uses the deterministic projection policy.
If a point is supplied, it must match that resolved position; the backend
never performs a new lookup or substitutes another candidate.

## Reachability and navigation

The deterministic motion authority produces a `ReachabilityReport` containing
state, reason, path length, estimated time, costmap revision, freshness, and
selected pose. It treats unknown/occupied cells and insufficient footprint
clearance as unsafe preflight results. This is a conservative estimate, not a
claim of execution success. Nav2 performs global planning, local control,
recovery, dynamic obstacle avoidance, and final success/failure.

## Failure and evidence

Malformed planner output, rejected preconditions, stale evidence, and ordinary
tool failures become board-visible nonfatal evidence for the next decision.
Cancellation, safety/lifecycle corruption, infrastructure failure, and the
scenario's terminal policy may end a mission. POI exhaustion is authoritative
only for `search_until_exhausted`; inspect the typed POI result,
`/navigate_to_pose`, motion state, and the mission bag.

## Verification checklist

Before relying on this note:

1. Confirm `ROS_DISTRO=humble` and inspect the installed Humble interfaces.
2. Confirm the installed package contains `muto_command_layer_v2` and no
   `muto_command_layer` package or launch process.
3. Run the v2 unit/transport tests and the Humble launch-description smoke.
4. For hardware, inspect the live graph and the mission MCAP rather than
   inferring readiness from source or simulation alone.
5. Include an attribute-mismatch trace (for example, a red `chair` returned
   for `blue chair`) and verify that no `confirmed_target_id` is committed.
