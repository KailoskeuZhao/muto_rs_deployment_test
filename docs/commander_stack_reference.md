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
  -> independent registry/frontier/Nav2 authority
  -> MissionBoard + MissionEvent + high-level MCAP
```

The standalone `frontier_exploration_ros2` package remains an exploration
authority because v2 adapts its control service directly; it is not a command
layer and must remain cold-idle until the executive requests a bounded
observation session. Nav2 remains the final navigation and obstacle-avoidance
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
events, typed rejection, registry evidence, frontier selection/projection, and
recorder lifecycle into one unique MCAP per mission. Raw camera, LiDAR, IMU,
and point-cloud recording is opt-in through the dedicated diagnostic bags.

## Commander contract

The commander has exactly two skills:

- `search_for_object`: use registry lookup, revision-scoped candidate
  confirmation, bounded observation/rotation, and exploration as needed;
- `approach_confirmed_object`: approach exactly one candidate confirmed for the
  current registry revision.

The commander-facing tool table is fixed and typed:

- `query_registry`
- `inspect_candidates`
- `observe`
- `rotate_to_heading`
- `go_to_point`

The model receives the objective, completion policy, registry revision and
shortlist evidence, robot pose/readiness, current board/progress, recent tool
results, and a bounded camera snapshot. It selects a skill and at most one
tool call. It cannot publish raw ROS commands or mutate mission state.

## Confirmation chain

Registry name/metadata matching creates only a shortlist. Every candidate
inspection is tied to the exact registry revision and stored-JPEG evidence.
At most one candidate may be confirmed. A target is approach-eligible only
when its confirmation revision equals the board's current registry revision.
Repeated unchanged lookups preserve the existing evidence; they do not reset
the search or interrupt an active bounded observation.

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
scenario's terminal policy may end a mission. A frontier start/completion
event is not navigation evidence by itself; inspect the frontier adapter,
`/navigate_to_pose`, motion state, and the mission bag.

## Verification checklist

Before relying on this note:

1. Confirm `ROS_DISTRO=humble` and inspect the installed Humble interfaces.
2. Confirm the installed package contains `muto_command_layer_v2` and no
   `muto_command_layer` package or launch process.
3. Run the v2 unit/transport tests and the Humble launch-description smoke.
4. For hardware, inspect the live graph and the mission MCAP rather than
   inferring readiness from source or simulation alone.
