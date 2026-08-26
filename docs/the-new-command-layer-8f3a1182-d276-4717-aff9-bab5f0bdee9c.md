# New Command Layer Design Note

This note records the v2 command-layer design and cutover decisions. It is not
the operational as-is reference; for the current stack, read
`docs/commander_stack_reference.md` and verify it against source before
changing behavior.

## Cutover status (2026-08-24)

The retired v1 `muto_command_layer` package and its v1-only
`muto_command_bag`/`muto_exploration_bag` recorders have been removed from the
workspace. The v2 package is the only command-layer implementation and no v2
runtime imports the deleted actions, launch files, or state machine. The
standalone `frontier_exploration_ros2` backend remains intentionally mounted:
it is the v2 `observe` authority, kept cold-idle until the commander selects
the search skill. Removing that backend would be a separate v2 behavior change,
not legacy cleanup.

The target remains ROS 2 Humble on the Muto robot. The new stack should be
agentic, but the agent must not become the mission lifecycle owner.

## Agent Handoff: Target Topology

The intended high-level topology is deliberately small and should be the first
architecture check for any future agent:

```text
Natural-language adapter + MissionExecutive
                  |
            CommanderAgent
                  |
       independent registry/frontier/Nav2 backends
                  |
          high-level event recorder
```

This is the target shape, not a request to add one process per box. The
natural-language adapter and executive may live in one node; the commander
may be a separate node or component; registry, frontier, and Nav2 are
independent deterministic authorities; and the recorder is an observing
subscriber, not a commander capability. Any backend currently embedded in the
old command layer must be extracted or reimplemented before cutover.
Safety/emergency stop remains a separate bypass path. New abstractions or
processes must justify their existence against this topology.

## Squashed v2 Contract (Normative)

This is the compact contract future agents and implementation work must use.
The ADRs below preserve design rationale; if an older ADR conflicts with this
section, this section wins.

### Runtime shape

```text
Natural language
      -> MissionAction
      -> MissionExecutive
      -> CommanderAgent
      -> Skill + typed tool call
      -> independent registry/frontier/Nav2 backends
      -> MissionBoard + MissionEvent
      -> action feedback, status, and high-level bag
```

- `MissionExecutive` is the only mission-truth and lifecycle authority. The
  natural-language adapter may live in the same node and only normalizes or
  rejects user requests.
- `CommanderAgent` chooses the next skill and, while that skill is active, a
  bounded typed tool call. It cannot publish raw ROS commands or mutate board
  state.
- Registry, frontier, motion, and Nav2 are independent deterministic
  authorities. No v2 runtime dependency may come from the old command-layer
  package, actions, launch graph, or state machine. Code currently trapped in
  that package must be extracted or reimplemented before cutover.
- The recorder passively subscribes to mission events and lifecycle signals.
  It is not a commander tool and cannot affect mission success.
- Emergency stop remains outside this path.

### Skills and tools

The v2 commander has exactly two high-level skills:

- `search_for_object`: registry lookup, candidate confirmation, observation,
  rotation, and deterministic exploration as needed;
- `approach_confirmed_object`: approach one explicitly confirmed candidate.

Not-found is an executive completion proposal, not a skill. The initial
commander-visible tools are exactly:

- `query_registry`;
- `inspect_candidates`;
- `observe`;
- `rotate_to_heading`; and
- `go_to_point`.

Frontier selection, viewpoint generation, checkpointing, and recorder writes
are deterministic internals. A static typed dispatch table is sufficient; no
dynamic capability/plugin registry is part of v2.

The commander may hand off from `search_for_object` to
`approach_confirmed_object` only after the executive has recorded an explicit
candidate confirmation for the current registry revision. Unconfirmed or
arbitrary reverse skill changes are invalid planner output and are replanned
as board-visible failures.

### State, completion, and navigation

- One current `MissionBoard` and one append-only `MissionEvent` stream are the
  canonical observability interfaces. ROS action feedback and status topics
  are transport projections, not additional state models.
- The mission lifecycle is `IDLE -> ACCEPTED -> RUNNING -> SUCCEEDED`,
  `CANCELED`, or `FAILED`. Skill/tool phases are events.
- User-facing goals contain an objective, optional object request, and a
  normalized completion intent. The starting scenario/harness resolves that
  intent to a fixed policy; simulation/test harnesses add `scenario_id` as
  internal run metadata.
- Completion policies are a fixed v2 set: `report_confirmed`,
  `approach_confirmed`, and `search_until_exhausted`. The commander cannot
  redefine the selected policy.
- Child failures normally become board-visible evidence and trigger replanning.
  Cancellation, safety/lifecycle corruption, infrastructure failure, and
  scenario-defined terminal conditions may end the mission. There is no
  mission-wide wall-clock, distance, energy, or tool-call budget.
- `ReachabilityReport` is the single reachability result. It is a preflight
  estimate; Nav2 remains authoritative for planning, obstacle avoidance,
  recovery, and final navigation success.

### Replacement and versioning

- The v2 action/tool protocol has one v2 schema version. Skill names, manifests,
  and compatibility registries are not independently versioned frameworks.
- Side-by-side launch is allowed only for isolated validation. After the
  simulation and controlled-robot gates pass, the old command layer is
  removed from the image, launch, dependencies, and runtime graph.

## Design Goal

Build a replacement command layer v2 that is simpler to reason about than the
current command stack. During validation it may be launched side-by-side with
the old layer, but side-by-side operation is temporary and is not the target
runtime:

- one deterministic mission executive owns truth, lifecycle, local safety
  limits, retries, cancellation, final outcomes, and board updates;
- one commander agent chooses the next skill, then uses only that skill's
  scoped tools from the `MissionBoard` and current visual inputs;
- deterministic skill runtimes and tool backends own low-level behavior such as
  rotation, observation timing, Nav2 calls, frontier selection, registry
  checking, checkpointing, and retry timing;
- normal command failures are nonfatal evidence for the next agent decision;
  terminal failure is reserved for cancellation, safety/lifecycle corruption,
  infrastructure failure, or a scenario-defined terminal condition.

## Settled Decisions

The ADRs in this section are the interview history and rationale. They are
not a second interface specification; the Squashed v2 Contract above is
normative.

### ADR-001: Mission Truth Is Deterministic

The deterministic mission executive owns mission truth. The commander agent
does not decide whether the mission is complete by itself. It selects the next
skill and, while that skill is active, may make scoped tool calls. The executive
validates each call, runs it, records the result, updates the board, and
determines whether the mission has reached a valid terminal condition.

### ADR-002: The Commander Invokes Skills, Not Configuration Flows

The commander chooses a reusable high-level skill. A skill may contain a
multi-step scheme (for example, finding and approaching an identified object),
but that scheme is an implementation detail of the skill, not a strict global
mission flow encoded in the executive or in the prompt. The same skill can be
used in different mission contexts and can return control to the executive at
its natural completion, interruption, or failure boundary.

The initial skill toolbox is intentionally small:

- `search_for_object`: expand the searchable space using deterministic
  frontier and viewpoint policy;
- `approach_confirmed_object`: use the confirmed object ID and deterministic
  approach/navigation policy to reach it.

The search skill includes registry lookup, candidate imagery, visual
confirmation, observation, rotation, and exploration as needed. A valid
not-found result is an executive completion proposal, not a third skill.

The skill runtime uses deterministic internals such as viewpoint selection,
frontier selection, Nav2, and checkpoint writing. Only the five typed tools are
commander-facing. The executive still validates safety, local limits,
cancellation, confirmation invariants, and terminal outcomes, but it does not
prescribe a universal sequence such as
`check_registry -> inspect_candidates -> explore -> approach`.

### ADR-003: Object Confirmation Has One Valid Chain

The only valid final-object path is:

```text
registry name/metadata lookup
  -> candidate shortlist
  -> stored candidate images
  -> commander visual confirmation
  -> confirmed exact ID or rejected candidates
```

A registry shortlist is not a final object. Live camera evidence may motivate
a registry check, but it does not directly create a final confirmed object.

### ADR-004: Exploration Is Intent From The Agent, Policy In Code

The commander may choose search tools, but it does not pick raw cells,
viewpoints, or frontier goals directly. Deterministic exploration code chooses
reachable cells, viewpoints, frontier goals, scan strategy, and fallback
behavior. The board reports what was attempted, why, and what progress or
failure resulted.

### ADR-005: Command Failures Are Usually Nonfatal

Impossible or failed commands normally end that command and update the board.
They should not terminate the mission unless the executive identifies a
terminal condition. This keeps the agent loop useful while preventing local
errors from corrupting mission state.

### ADR-006: The Board Is Typed And Small

The commander receives a compact typed board, not an unbounded debug dump.
The board should contain:

- mission goal and completion mode;
- robot pose and motion status;
- map/search coverage summary;
- current candidate shortlist;
- confirmed and rejected candidate IDs;
- last command result;
- visible evidence summary;
- active local limits and retry state;
- active command status.

Full traces, bags, raw action feedback, and debug details remain available for
operators and tests, but they are not the main commander input.

### ADR-007: Side By Side Is Validation Only

Launch v2 side-by-side with the old command layer only for isolated validation,
using separate names and ROS domains where necessary. This is a migration
technique, not a production architecture. Once the replacement gate passes,
the old command layer is removed from the mounted launch, package, and runtime
graph; production must not retain a fallback to it.

### ADR-008: Documentation Split

This file is the living v2 design note. `commander_stack_reference.md`
describes the current stack as-is for future AI agents. If v2 changes current
behavior, update both the implementation and the relevant documentation.

### ADR-009: Exploration Is Not Interrupted By Candidate Discovery

`search_for_object` runs as a deterministic search skill. Each `observe` tool
call requests one explorer-owned frontier goal and returns a typed semantic
result to Commander. New registry identities and newly shortlisted target
candidates are recorded as evidence during the command, but they do not stop
that frontier step early. The executive returns the evidence to the board
when the step ends and Commander chooses what happens next.

Allowed early-stop reasons for `search_for_object`:

- `frontier_no_reachable_goal`;
- `frontier_goals_blocked`;
- `frontier_goal_succeeded`;
- `frontier_exhausted`;
- `frontier_no_progress`;
- `frontier_goal_failed`;
- `frontier_goal_canceled`;
- `frontier_safety_watchdog_elapsed` (emergency only).

The explorer owns single-step goal shutdown, so Commander does not race a
second stop request against Nav2 result delivery. Candidate confirmation
happens after the executive regains control and updates the board. There is no
mission-wide time budget; the watchdog is only a local safety deadman.

### ADR-010: Skills Use One Generic Invocation Contract

Internally, the commander emits a `SkillCall` envelope containing a skill name,
typed arguments, and execution constraints under the single v2 protocol
schema. Individual skills have typed argument and result fields but no
separate compatibility protocol. External clients use the mission action,
not skill-specific ROS actions.

### ADR-011: Skills Are Bounded Deterministic Runtimes

The commander chooses among the approved tools exposed by the active skill.
The skill runtime executes those tools as a bounded deterministic scheme; it
cannot create an independent mission loop, mutate canonical board state, or
bypass executive limits. The executive remains the only owner of lifecycle,
cancellation, local safety limits, confirmation, and terminal state.

### ADR-012: Skills Do Not Recursively Invoke Skills In v2

Skill runtimes call deterministic internals directly. Unrestricted nested skill
calls are deferred because they create hidden recursive command trees and make
ownership and cancellation difficult to observe. Explicit bounded subskills can
be added later when a concrete use case justifies them.

### ADR-013: Skill Progress Is Typed Semantic Events

Skills report structured events such as `candidate_seen`, `candidate_rejected`,
`viewpoint_completed`, `coverage_increased`, `target_approach_progress`, and
`no_progress`. State transitions do not parse free-form model text or raw ROS
feedback strings.

### ADR-014: Evidence And Confirmation Are Distinct

Model judgments are stored as evidence with source, timestamp, candidate ID,
and confidence. Only an explicit confirmation result can promote a candidate
to `ConfirmedTarget`.

### ADR-015: Candidate Identity Is Revision-Scoped

Candidate decisions are keyed by `{candidate_id, registry_revision, mission_id}`.
A rejection applies to that mission and registry revision. A genuinely new
registry revision may reintroduce the identity for reconsideration.

### ADR-016: Completion Comes From The Starting Scenario

The starting scenario supplies a typed `CompletionPolicy` describing whether
success means reporting a confirmed object, approaching it, exhausting search,
or another scenario-specific predicate. The executive evaluates this policy;
the commander may propose completion but cannot redefine it.

### ADR-017: Validation Uses A Deterministic Trace

Every v2 scenario records skill selection, tool/backend events, board revisions,
confirmation state, interruptions, and the scenario-specific terminal
predicate. A passing scenario must prove the trace, not merely observe a final
boolean result.

### ADR-018: The Commander Has Scoped Tool Access Inside A Skill

While a skill is active, the commander may invoke approved typed tools through
the static dispatch table. The initial tools are `go_to_point`,
`rotate_to_heading`, `observe`, `query_registry`, and `inspect_candidates`.
These are tool calls, not a globally prescribed mission sequence.

Every tool call is mediated by the executive and static dispatch table. The
executive validates frames, reachability, confirmation prerequisites, timeout,
cancellation, safety, and local limits before dispatch. Tool results are typed events
that update the board.

For example, `approach_confirmed_object` may request `go_to_point` as part of
its approach strategy, while `search_for_object` may request observation or a
heading change. The commander may choose the next tool based on the board, but
it cannot publish raw ROS messages, bypass validation, or invent unbounded
low-level control loops.

`go_to_point` accepts a validated map-frame point or a board-provided viewpoint
ID. `rotate_to_heading` accepts a map-frame heading; the deterministic motion
backend handles angle wrapping, shortest-turn policy, and the 180-degree case.

### ADR-019: One Canonical State Has Multiple Projections

The executive owns one canonical typed `MissionBoard` and one append-only
`MissionEvent` stream. Action feedback and operator status are transport views
of those two interfaces, not additional state models.

### ADR-020: The Executive Uses A Compact Mission Lifecycle

The mission follows the compact lifecycle:

```text
IDLE -> ACCEPTED -> RUNNING -> SUCCEEDED
                         -> CANCELED
                         -> FAILED
```

Skill and tool phases are typed events. The completion policy determines the
successful terminal meaning (`report_confirmed`, `approach_confirmed`, or
`search_until_exhausted`).

### ADR-021: Skill Results Are Structured

A skill returns a typed result containing status, evidence delta, progress
events, tool/backend failures, an optional completion proposal, and a human
summary. The executive decides whether the completion proposal is valid.

### ADR-022: Tools Use A Static Typed Dispatch Table

Skills invoke tools through a static typed dispatch table. Each tool declares
typed arguments, typed events, timeout and cancellation behavior, and
ownership metadata. Skills do not contain direct ROS wiring.

### ADR-023: Invalid Model Output Is Nonfatal

The commander output is checked against a strict versioned JSON schema and
skill enum. Unsupported or malformed output becomes an `invalid_skill_request`
board event and triggers replanning. The system never guesses a command or
silently substitutes another skill.

### ADR-024: Interruption Has Explicit Semantics

Safety and lifecycle faults halt immediately. Explicit cancellation and local
tool/skill limit expiry stop cooperatively at the current tool boundary.
Candidate discovery never interrupts exploration. Ordinary tool/backend
failures are committed and replanned rather than automatically becoming
mission failure.

### ADR-025: Migration Reuses Authorities, Not The Old Command Layer

V2 owns its executive, skills, tools, lifecycle, and command implementations.
It must not depend on the old command-layer package, old command actions,
old launch wiring, old board/state machine, or compatibility wrappers around
those implementations. V2 may call independent authorities such as Nav2 and
the object registry. A backend currently embedded in the old command layer
must be extracted into an independent package or reimplemented behind the v2
interfaces before cutover.

### ADR-026: Replacement Requires Scenario Traces

Before v2 can replace the current stack, it must pass trace-asserting scenarios
for multiple candidate rejection, 90-degree off-axis targets, occlusion,
180-degree turns, discovery during exploration, fresh registry revisions,
Nav2 recovery, cancellation during motion, and valid not-found exhaustion.

### ADR-027: The Design Note Starts With An AI Handoff

Future agents must read this note first for the target platform, authority
model, skill/tool boundary, confirmation and cancellation invariants,
current implementation status, and unresolved decisions. The legacy stack is
documented separately in `docs/commander_stack_reference.md` and must not be
treated as the v2 design.

### ADR-028: The Initial Tool Set Is Small And Typed

The first v2 tool set is `go_to_point`, `rotate_to_heading`, `observe`,
`query_registry`, and `inspect_candidates`. Frontier selection, viewpoint
generation, checkpointing, and recording are deterministic internals. Raw
velocity commands, arbitrary ROS topic publishing, and direct unmediated Nav2
access are not commander tools.

### ADR-029: Tool Geometry Uses The Map Frame

Movement tools accept map-frame coordinates or board-provided viewpoint IDs;
headings are expressed in radians. Frame conversion and coordinate validation
are deterministic runtime responsibilities, not model responsibilities.

### ADR-030: Tool Permissions Are Skill-Scoped

The static skill/tool table defines each skill's allowlist and local limits,
such as maximum movement distance, rotation count, and bounded tool retries.
The executive rejects calls outside the active skill's scope.

### ADR-031: Capability Transport Is Hidden

The static dispatch table presents one typed interface. Long-running operations
use ROS actions behind it; quick reads use services or in-process calls. The
commander does not depend on transport details.

### ADR-032: Motion Is Serialized

Only one motion tool/backend operation may be active at a time. Read-only
perception may run asynchronously, but motion, cancellation, and canonical
board commits are serialized by the executive.

### ADR-033: Model Decisions Are Event-Driven

The commander is invoked at skill start, after a typed tool result, after a
meaningful board/evidence change, or on an executive interrupt. Sensor-rate
updates do not independently trigger model calls.

### ADR-034: Retry Budgets Have Two Layers

Capabilities may perform a small bounded local retry. The executive owns
per-skill no-progress and failure thresholds, but there is no mission-wide
wall-clock, distance, energy, or tool-call quota. The commander receives the
final typed failure and chooses the next skill or tool.

### ADR-035: Skills And Schemas Are Explicitly Versioned

The mission/tool protocol carries one v2 schema version. Skill names are
ordinary enum values such as `approach_confirmed_object`; per-skill version
registries and compatibility adapters are not part of the v2 runtime.

### ADR-036: V2 Replacement Requires The Full Scenario Gate

V2 must pass the trace-asserting simulated scenarios first, then a robot dry
run using the same interfaces. Until both gates pass, v2 remains side-by-side
for validation only. After the gate passes, remove the legacy command layer
from the production image, launch, package dependencies, and active graph;
there is no runtime fallback to the old command layer.

### ADR-037: Reachability Is A Deterministic Planner Predicate

The commander does not decide whether a cell is reachable. The runtime derives
reachability from the latest navigation data using this sequence:

1. Convert the request into the costmap/map grid and reject invalid frames or
   out-of-bounds cells.
2. Treat only known-free cells below the configured occupancy/cost threshold as
   traversable. Unknown and occupied cells are not safe endpoints.
3. Apply robot-footprint clearance. Every map cell touched by the effective
   footprint must remain known-free; inflated navigation cost may provide the
   clearance mask when available.
4. Snap the robot pose to a nearby footprint-safe start cell if necessary.
5. Flood-fill the traversable mask from that start cell. Diagonal moves are
   allowed only when both intervening cardinal cells are also traversable, so
   the robot cannot pass through an artificial diagonal corner.
6. A cell is reachable only if it belongs to that connected component. Rank
   reachable candidates by planner path distance and deterministic tie-breakers.
7. Submit the selected pose to Nav2, because the costmap may change between
   the snapshot and execution. Nav2 acceptance/execution is the final runtime
   confirmation.

For frontier goals, the unknown frontier cell is a discovery boundary, not a
safe navigation endpoint. The frontier adapter projects it to the nearest
reachable footprint-safe known-free cell and may choose a further staged cell
when the nearest projection would produce no meaningful advance. For object
approach, the helper selects a preflight-admissible stance outside the minimum
standoff radius and orients the robot toward the object. Nav2 still owns the
actual path, controller, obstacle avoidance, recovery, and success or abort
decision.

The runtime keeps a full internal reachability result for diagnostics and
trace assertions. The commander receives only a compact projection rather than
a planner dump:

- `state`: `reachable`, `unreachable`, or `unknown`;
- `reason_code`: stable machine-readable reason;
- `path_length_m` and `estimated_time_s` when available;
- `map_revision` plus freshness state;
- `executed_pose` only when the runtime projected or changed the requested
  pose, otherwise the commander already knows the requested pose.

Raw costmaps, cell masks, planner internals, and full diagnostic text remain
operator/test data. A stale or unavailable costmap is `unknown`, not reachable.

### ADR-038: Nav2 Is The Navigation And Obstacle-Avoidance Authority

The v2 command layer does not replace or override Nav2. Its costmap checks,
footprint masks, connected-component tests, stance selection, and frontier
projection are conservative preflight helpers for choosing or rejecting goals.
Nav2 remains authoritative for global planning, local planning, controller
execution, dynamic obstacle avoidance, recovery behavior, and final navigation
success or failure. A preflight `reachable` result is therefore an estimate,
not a guarantee that Nav2 will complete the motion.

### ADR-039: Preflight And Nav2 Reasons Are Distinct

Commander-facing reason codes distinguish command-layer filtering from Nav2
execution results:

```text
preflight_invalid_frame, preflight_out_of_bounds,
preflight_unknown_space, preflight_high_cost,
preflight_insufficient_clearance, preflight_disconnected,
preflight_goal_projected, costmap_stale, nav2_unavailable,
nav2_rejected, nav2_aborted, dynamic_obstacle_blocked, canceled
```

### ADR-040: Freshness Uses Scenario-Configured Limits

Reachability freshness compares costmap timestamp, map revision, and required
TF freshness against scenario-configured limits. The result is `fresh`,
`stale`, or `unknown`; there is no universal hard-coded timeout.

### ADR-041: Dynamic Obstacle Handling Belongs To Nav2

Dynamic obstacle avoidance remains in Nav2's local costmap, controller, and
recovery path. The command layer reports the resulting typed outcome but does
not implement a competing obstacle-avoidance loop.

### ADR-042: Preflight Does Not Imply Navigation Success

A preflight `reachable` result permits dispatch only. It never permits the
skill or commander to assume success; the Nav2 action result remains
authoritative.

### ADR-043: Projection Requires Explicit Permission

Deterministic search internals may project frontier or viewpoint goals.
Ordinary `go_to_point` rejects an unsafe requested point unless its explicit
`projection_policy` allows adjustment.

### ADR-044: Navigation Revisions Trigger Revalidation

If the costmap or map revision changes before dispatch, preflight is recomputed
against the new data. Changes during execution are handled by Nav2 and reported
through its action outcome.

### ADR-045: ETA Is Advisory

`estimated_time_s` is derived from deterministic path length and nominal speed
and may rank alternatives. It cannot override Nav2 timeouts or determine
mission success.

### ADR-046: Reachability Has Dedicated Fixtures

The v2 test suite covers disconnected free regions, diagonal corner blocking,
footprint clearance, unknown cells, projection, stale revisions, dynamic
blockage, Nav2 rejection, and successful Nav2 execution.

### ADR-047: There Is No Mission-Wide Budget

V2 does not impose a global wall-clock, distance, energy, or tool-call quota.
Exploration may continue until the starting scenario's completion policy,
explicit cancellation, safety/lifecycle failure, or infrastructure failure
ends it. Local limits still exist for individual tool timeouts, bounded retries,
skill no-progress/failure thresholds, and storage/resource safety.

### ADR-048: Every Mission Has A High-Level Rosbag Profile

An independent rosbag2/MCAP recorder is available for every v2 mission and is
started and finalized by mission lifecycle events. The default profile records
mid- and high-level data only, including:

- mission, skill, tool, board, decision, and terminal events;
- registry revisions, candidate IDs, confirmation/rejection evidence, and
  object summaries;
- frontier/viewpoint selections, projected goals, Nav2 goal/status/result,
  controller/recovery status, and compact motion progress;
- compact robot pose, map/search coverage, scenario configuration, recorder
  status, and operator annotations;
- recording manifest, schema version, build revision, and terminal reason.

It excludes raw camera images, camera point clouds, LiDAR point clouds,
LaserScan/IMU streams, detector masks, and other high-bandwidth sensor input.
Raw sensor capture remains a separate explicitly selected diagnostic profile.
The recorder is a passive observability subscriber, not a mission-completion
decision maker or commander tool. It follows mission lifecycle events emitted
by the executive. Storage retention/pruning is an operational policy and is
not a mission budget.

### ADR-049: Skills Use A Static Scoped Tool Table

The v2 implementation contains one static table of skill allowlists, argument
schemas, local limits, and completion conditions. There is no dynamic or
independently versioned skill-manifest protocol.

### ADR-050: Motion Tool Calls Are Serialized

A model response may request one motion tool at a time. Read-only calls may be
batched only when they cannot race with motion or mutate canonical state.

### ADR-051: Skill Handoff Uses Typed Outcomes

Skill and tool results contain typed status, evidence, progress, and optional
completion proposals. The executive maps those fields to the compact mission
lifecycle; there is no separate public `SkillOutcome` model.

### ADR-052: Progress Is Deterministic

The executive evaluates progress using skill-specific metrics such as distance
advanced, new map coverage, new evidence, candidate-state change, or approach
distance reduction. Free-form model prose cannot establish progress.

### ADR-053: Candidate Inspection Has Revision-Scoped Evidence

Candidate inspection receives the object request, candidate IDs, registry
metadata, one stored image per candidate, image timestamps, and registry
revision. The response contains typed per-candidate decisions; the executive
validates IDs and revision before committing them.

### ADR-054: Interrupted Skills Restart From Canonical State

V2 does not resume hidden skill-internal state after interruption. The executive
commits the interruption result, and the next skill starts from the canonical
board.

### ADR-055: External Clients Use One Mission Action

External callers invoke one v2 mission action. Skill calls and tools remain
internal interfaces; recording is passive and has no commander API.

### ADR-056: Recorder Lifecycle Follows Mission Ownership

The high-level recorder observes the executive's accepted, terminal, canceled,
and owner-loss events and finalizes accordingly. Recorder failure cannot
control mission completion.

### ADR-057: The High-Level Bag Uses An Explicit Allowlist

The default recorder uses an explicit topic profile generated from the v2
observability schema. It does not discover every topic and then attempt to
exclude sensors with a broad regex.

### ADR-058: Recorder Readiness Is Nonfatal

The executive requests recorder readiness and records a startup event, but a
recorder that is unavailable does not block mission execution. The board and
trace receive `recording_unavailable`.

### ADR-059: Recorder Failure Does Not Stop The Mission

Disk-full, writer, or recorder-process failures stop recording and publish a
typed recorder failure. The mission continues unless its starting scenario
explicitly requires evidence capture.

### ADR-060: High-Level Bags Are Diagnostic Replays

The high-level bag reconstructs decisions, state, and outcomes; it is not a
sensor replay source for driving the robot. Full sensor replay uses a separate
diagnostic profile.

### ADR-061: Retention Is Operational

Bag pruning is limited to managed prefixes and an operational directory/disk
policy. It never acts as a mission budget or deletes unrelated directories.

### ADR-062: Evidence Images Are Opt-In

The default high-level bag records candidate image metadata and evidence IDs,
not image payloads. Bounded inspected images require an explicitly enabled
evidence profile.

### ADR-063: The Mission Action Is Typed

The external v2 mission action contains `objective`, optional `object_request`,
the scenario-selected `completion_policy`, and optional initial constraints.
It does not expose `scenario_id`, internal skill names, tool calls, planner
details, or recorder topic configuration.

### ADR-064: Natural Language Remains The User Front End

Users continue to submit natural-language commands. A deterministic
natural-language adapter validates and normalizes the request into the typed
v2 mission action. The commander then receives the normalized objective and
board; the front end never directly chooses tools, bypasses the executive, or
mutates mission state.

### ADR-065: Invalid Or Ambiguous Language Is Rejected Before Acceptance

The natural-language adapter may reject a request before creating an active
mission. The rejection is typed action feedback with a stable reason such as
`ambiguous_object_reference`, `missing_completion_policy`,
`unsupported_request`, or `invalid_arguments`. It may include a concise
clarification prompt, but no commander skill, tool, navigation, or registry
mutation starts until the user submits an accepted request.

### ADR-066: User Feedback Is A Compact Mission Projection

The frontend exposes current mission phase, active skill, concise progress,
candidate status, and terminal reason. Raw tool traffic and planner internals
remain operator/test data.

### ADR-067: Cancellation Bypasses Model Planning

User requests such as “stop” or “cancel” map directly to executive
cancellation. They do not require another natural-language interpretation or
commander decision once recognized as cancellation.

### ADR-068: Clarification Happens Before Acceptance

Clarification may resolve an unaccepted request, but it cannot mutate an active
mission. An active mission can only be canceled, completed, or continue under
its accepted `MissionAction`.

### ADR-069: Parser And Commander Have Separate Contracts

Natural-language interpretation creates a validated `MissionAction`. Commander
planning begins only after acceptance and consumes the mission plus board. The
two stages do not share one unconstrained model call.

### ADR-070: Unsupported Requests Are Rejected Without Partial Execution

Requests outside the supported mission/action schema receive typed rejection
feedback before mission start. No partial skill, tool, registry, navigation,
or recorder mission is launched.

### ADR-071: Accepted Missions Have Stable Identity

Every accepted request receives a unique `request_id` and `mission_id`. Board
events, bags, skill results, and user feedback reference those identities.
Simulation/test harness metadata may additionally attach a `scenario_id`.

### ADR-072: Feedback Has Action And Status Projections

The active client receives ROS action feedback, while observers receive a
transient status topic. Both are transport projections of the `MissionBoard`
and `MissionEvent` interfaces; there is no separate feedback state model.

### ADR-073: Duplicate Requests Are Explicitly Rejected

A duplicate request does not implicitly start another mission. The frontend
returns the active mission identity or a typed `mission_already_active`
rejection.

### ADR-074: Emergency Stop Is A Separate Safety Path

Emergency stop bypasses commander and executive planning and is handled by the
safety/motion system. Normal user cancellation remains an executive lifecycle
operation.

### ADR-075: Restart Is Fail-Closed

If the executive process is lost, the recorder finalizes with `owner_lost` and
the next process requires a new accepted mission. Hidden skill state is never
reconstructed after restart.

### ADR-076: Terminal Feedback Is Typed And Useful

Mission completion returns a typed terminal outcome, concise explanation,
confirmed target or not-found evidence, `mission_id`, and the bag path when
recording succeeded.

### ADR-077: Natural Language Enters Through The V2 Mission Action

Long-running natural-language requests use the v2 mission action. A
small immediate acceptance/rejection endpoint may validate the request first,
but the action remains the source of mission feedback and cancellation.

### ADR-078: Mission Goals Are Strictly Typed

The normalized mission goal requires `objective`, optional `object_request`,
the fixed `completion_policy` selected by the starting scenario, and the v2
schema version. Unknown or ambiguous fields are rejected before acceptance. A
simulation/test harness may attach `scenario_id` internally; it is not a
normal user goal field.

### ADR-079: Completion Policies Are A Fixed V2 Set

The initial completion policies are the fixed enum `report_confirmed`,
`approach_confirmed`, and `search_until_exhausted`. They are not a dynamic
registry. Cancellation and infrastructure failure are terminal outcomes, not
completion policies.

### ADR-080: Scenarios Own Initial Conditions

The launch or test harness supplies `scenario_id`, initial world assumptions,
and the selected completion policy. The natural-language frontend supplies the
objective and completion intent; it does not select a runtime scenario or
dynamic scenario policy.

### ADR-081: Request And Mission Correlation Is End-To-End

`request_id` and `mission_id` are propagated through acceptance events, action
goals, board events, and recorder manifests. `scenario_id` is propagated only
when supplied by a simulation/test harness.

### ADR-082: Status And Trace QoS Are Deliberate

`MissionBoard` snapshots use transient-local delivery for late observers.
Append-only `MissionEvent` records use reliable delivery. Action feedback
remains tied to the active goal.

### ADR-083: One Active Mission Controls Mutations

The executive accepts at most one active mission. Other clients may observe,
but only the active action client or explicit operator authority may cancel or
mutate it. This is an executive invariant, not a separate lease protocol.

### ADR-084: Reproduction Uses Typed Mission Inputs

Reproduction records and reuses the normalized mission action, v2 schema/config
revision, optional scenario ID, tool results, board revisions, Nav2 outcomes,
and recorder metadata. It does not depend on replaying model prose.

### ADR-085: Commander Provenance Is Recorded

The mission manifest records the commander model identifier, prompt/schema
version, v2 tool-table/configuration revision, and build revision.

### ADR-086: Review Before Implementation

The v2 design frontier is complete enough for a dedicated review pass. No
production implementation should begin until the ADRs, domain model, public
interfaces, authority boundaries, scenario policies, and trace obligations are
reviewed together. Review may revise these decisions; implementation follows
the reviewed version rather than silently filling gaps.

### ADR-087: Simplification Review Baseline

The review removes framework abstractions that do not change the target
behavior. The v2 baseline is:

- two commander skills: `search_for_object` and
  `approach_confirmed_object`; not-found is an executive completion proposal;
- five commander-visible tools: `query_registry`, `inspect_candidates`,
  `observe`, `rotate_to_heading`, and `go_to_point`;
- one current `MissionBoard` plus one append-only `MissionEvent` stream;
- one compact mission lifecycle, with skill and tool phases represented as
  typed events;
- a static typed dispatch table and one v2 schema version instead of dynamic
  capability plugins and per-skill version registries;
- one `ReachabilityReport`, projected into the board without a second domain
  result type;
- scenario identity supplied by simulation/test configuration, not ordinary
  user requests; and
- a lifecycle-managed high-level recorder subscribed to the explicit mission
  board/event/rejection and frontier-adapter diagnostic allowlist plus its own
  manifest/status topics, not a commander capability. It opens one
  mission-scoped bag on acceptance, closes it on a terminal outcome, and never
  records raw sensor streams.

These cuts preserve the confirmation chain, deterministic executive
authority, Nav2 authority, cancellation semantics, and trace requirements.

### ADR-088: Clean Replacement Cutover

The old command layer is permitted only as a temporary validation reference.
The new command layer must be independently implemented and must not reuse the
old command-layer package, action servers, launch graph, board/state machine,
or compatibility wrappers as runtime dependencies. Once v2 passes the
simulation and controlled-robot confirmation gates, the old layer is removed
from the mounted system and its package dependencies. The deployed graph then
contains only v2 plus its independent authorities and safety path.

### ADR-089: Review Complete; Writing Stage Authorized

The architecture review confirms that the Squashed v2 Contract is the design
principle set. Package names, concrete ROS message definitions, backend
extraction mechanics, and test implementation details are now writing-stage
decisions constrained by that contract, not additional architecture branches.
Implementation may begin with the v2 interface and contract-test layer. No
legacy command-layer dependency is permitted.

## Domain Model Draft

`MissionExecutive`: the deterministic mission-truth and lifecycle owner. It
accepts or rejects `MissionAction`, validates and dispatches skill/tool calls,
commits the board and event stream, handles cancellation and local limits, and
decides terminal outcomes.

`MissionBoard`: the single current typed snapshot supplied to the commander
and projected to operators.

`MissionEvent`: an append-only typed event describing decisions, tool results,
evidence, state changes, and terminal outcomes.

`CommanderAgent`: the model-backed decision maker that reads the board, chooses
one of the two skills, and makes bounded tool calls while that skill is active.

`Skill`: one of `search_for_object` or `approach_confirmed_object`; a bounded
high-level scheme whose deterministic runtime executes validated tools.

`Tool`: one of `query_registry`, `inspect_candidates`, `observe`,
`rotate_to_heading`, or `go_to_point`.

`SkillCall`: the internal v2-protocol envelope containing a skill and typed
arguments. It uses the single v2 schema version; skills do not have separate
compatibility registries.

`ToolCall`: a typed invocation of one commander-visible tool, validated by the
executive before dispatch.

`Candidate`: a registry object returned by name/metadata lookup.

`ConfirmedTarget`: exactly one candidate accepted by visual confirmation from
the stored candidate evidence for the current registry revision.

`Evidence`: a source-tagged observation or model judgment that is not yet
canonical mission truth.

`CompletionPolicy`: one of the fixed v2 policies `report_confirmed`,
`approach_confirmed`, or `search_until_exhausted`.

`ReachabilityReport`: the single deterministic preflight result containing
state, reason, path estimate, freshness/revision, and selected or projected
pose. Nav2 execution remains authoritative.

`MissionAction`: the typed v2 request containing objective, optional object
request, the scenario-selected `CompletionPolicy`, and schema version.
Simulation/test metadata such as `scenario_id` is added internally by the
harness.

`NaturalLanguageAdapter`: the frontend parser/validator that produces a
`MissionAction` or an `ActionRejection`; it does not select tools or own state.

`ActionRejection`: typed pre-acceptance feedback for ambiguous, invalid, or
unsupported user input.

`HighLevelBagProfile`: the explicit rosbag2/MCAP allowlist for canonical
mission board/event/rejection projections, recorder manifest/status, selected
frontier poses, and frontier-adapter original/projected/status diagnostics.
Those bounded projections carry the compact registry, reachability,
navigation-result, and progress fields needed for diagnostics; the profile
excludes raw sensor streams.

The following names are historical and are not v2 interfaces: `InformationBoard`,
`ProgressBoard`, `ReachabilitySummary`, `RecordingMarker`, `SkillManifest`,
`SkillOutcome`, `MissionLease`, `SemanticCommand`, and `CommandImplementation`.

## Open Design Questions

The design tree is now reduced to implementation review. The exact v2 command
contracts, board schema, and state machine must be derived from the Squashed v2
Contract, not reopened as separate competing abstractions. Remaining review
work is limited to backend extraction boundaries, ROS interface details,
scenario fixtures, and the replacement/cutover test gate.

The earlier interview rounds established the confirmation chain, nonfatal
command-failure policy, event-driven commander, Nav2 authority, reachability
preflight, cancellation semantics, high-level recording, natural-language
rejection, and replacement gate. Their detailed entries remain above as
rationale, but the names and interfaces from the Squashed v2 Contract replace
the earlier round-specific terminology.

Design status:

- The architecture review is complete; the normative compact contract is at
  the top of this note.
- The simplification baseline and clean-replacement boundary are recorded in
  ADR-087 and ADR-088.
- Writing-stage implementation is authorized under ADR-089.

Implementation status (2026-08-22):

- The independent `muto_command_layer_v2` package now contains the v2 ROS
  action/messages, typed executive contracts, natural-language rejection
  boundary, strict commander JSON parser, static skill-scoped tool table,
  conservative reachability preflight, revision-scoped visual evidence,
  cooperative cancellation, lifecycle-managed high-level recorder, the
  event-driven commander/executive loop, ROS projections, independent
  registry/Nav2 authority adapters, an independent ROS composition/launch,
  and a `muto_vlm_socket` action planner transport with a strict decision
  schema.
- The package builds and its eleven Humble CTest targets pass in the
  `ros:humble-ros-base` container. The host Jazzy build is not the deployment
  authority.
- A full v2 ROS graph test now drives a natural-language mission through the
  real v2 composition, VLM action transport, registry name/label shortlist,
  stored-candidate confirmation, and terminal board/event result using
  deterministic authority servers.
- The trace tests also cover revision-scoped rejection events, candidate
  evidence provenance, cooperative ROS cancellation at a tool boundary, exact
  180-degree heading normalization, and conservative reachability failures.
- The board projection carries candidate-id/evidence-id/source/confidence,
  reason, and observation-time arrays so a multi-candidate visual decision is
  auditable without consulting an out-of-band image table. Costmap revisions
  advance on semantic grid changes rather than repeated unchanged map
  timestamps; freshness still uses the latest message time. Registry
  revisions likewise ignore detector ``last_seen``/observation-count churn
  and millimetre-scale pose jitter, so a stable shortlist does not repeatedly
  invalidate confirmation or interrupt bounded exploration. A same-revision
  lookup preserves prior rejection/confirmation evidence; only a genuinely
  new shortlist revision resets it.
- The v2 composition now adapts one explorer-owned frontier step through the
  independent `frontier_exploration_ros2` control service and typed
  `/explore/frontier_goal_result` event, and the high-level rosbag2 writer opens per-mission output, writes a typed
  manifest/status record, and records only the bounded board/event/rejection/
  frontier-adapter allowlist. The writer defaults to unique persistent MCAP
  output under `/opt/muto_rs_ws/bags` and accepts explicit URI/run-id
  overrides. No old command-layer package or action is imported by v2.
- A v2-only `v2_nav2_sim_launch.py` now supplies a reactive 2-D map/odometry/
  LiDAR plant and starts the existing independent Nav2 pipeline with hardware,
  localization, mapping, and Nav2-bag recording disabled. Humble startup
  smoke reached the Nav2 readiness gate and launched the controller, planner,
  smoother, behavior, BT navigator, and lifecycle manager without legacy
  command-layer processes. This validates transport and authority startup,
  not a completed object mission: real VLM, registry, frontier, and mission
  scenario fixtures remain the final cutover gate.
- With the same launch, a real `/navigate_to_pose` goal to `(0.8, 0.0)` was
  accepted and finished `SUCCEEDED` in the reactive plant; this verifies the
  Nav2 controller/plant feedback loop, not merely action-server discovery.
- The traced v2 mission then completed the full simulated path with a
  deterministic external VLM/registry authority: `query_registry` →
  `inspect_candidates` → confirmed-target handoff to
  `approach_confirmed_object` → real `go_to_point`/Nav2 → `MISSION_SUCCEEDED`
  for `chair-1`. The test also caught and fixed the previously unreachable
  skill-handoff rule; an unconfirmed handoff is still rejected.
- The v2 launch exposes `scenario_id`, `scenario_completion_policy`, raw
  camera input, map/TF freshness limits, and recorder output explicitly. The
  camera adapter degrades to `visual_unavailable` when optional host image
  conversion libraries are unavailable instead of crashing the command node;
  Humble deployment still needs a valid camera/registry/VLM path for visual
  confirmation.
- A v2-only `v2_hardware_smoke_launch.py` now composes the existing production
  hardware/localization/SLAM/Nav2 pipeline with the independent frontier
  explorer, direct SAM2 annotator, object registry, VLM socket, v2 executive,
  and high-level MCAP recorder. An optional odometry input bag can be enabled
  separately (`record_odometry_bag:=true`); it keeps motor-angle polling
  disabled by default. The launch keeps frontier idle until the executive
  starts a bounded observation session and is ready for a supervised Humble
  robot smoke; the connected hardware/perception/network result remains an
  operational gate rather than a source-level claim. Raw Nav2/odometry bag
  profiles remain explicit opt-ins.

Cutover verification (2026-08-24):

- `colcon list` in the Humble container reports `muto_command_layer_v2` and no
  deleted v1 command or recorder packages.
- The v2 and Nav2 contract suites pass on the host (`44 passed, 5 skipped`);
  Python syntax compilation passes for the v2 and affected launch modules.
- The standalone frontier backend remains in the graph because v2 imports its
  typed control service. It is not an old v1 command-layer process.
