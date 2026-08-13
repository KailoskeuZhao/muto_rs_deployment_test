# Muto command bag

`muto_command_bag` records one MCAP bag for the complete
`/look_for_object` model-commander mission. It starts before the initial
registry check and closes after success, cancellation, abort, or a fail-closed
`ownership_uncertain` terminal state, so waits and replanning gaps remain in
the same timeline.

This is the primary default overall monitor for high-level autonomous command
missions. “Overall” means the complete parent mission lifecycle, not continuous
recording from robot startup. The exploration recorder may open a second,
child-scoped bag during an exploration primitive. See the workspace-wide
[Default Bags And Mission Monitoring](../../docs/bags.md) overview.

The default recorder also watches the transient `/model_commander/status`
heartbeat. If the commander process crashes or its executor stops publishing
for 10 seconds while a bag is active, the independent recorder finalizes the
MCAP with reason `owner_heartbeat_timeout`. This prevents a lost terminal
lifecycle event from leaving an unbounded recording behind. The timeout and
topic are configurable as `owner_heartbeat_timeout` and
`owner_heartbeat_topic`; an empty topic disables the watchdog.

The package reuses the Humble/Jazzy-compatible recorder engine from
`muto_exploration_bag` with an independent lifecycle, output prefix, manifest
schema, status topic, and operator-event topic.

The normal command-layer launch starts it automatically. To start it
separately:

```bash
ros2 launch muto_command_bag record_command_bag_launch.py
```

Bags are saved under:

```text
/opt/muto_rs_ws/bags/muto_command_<timestamp>_<mission-id>
```

By default, the recorder keeps the newest 20 recognized Muto bag directories in
that parent directory across command, exploration, nav2, and odometry bags.
Older Muto bags are pruned after a new bag finalizes. Unrelated directories are
not touched. Set `max_bag_directories:=0` to disable pruning.

Find the current or latest path:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /model_commander/last_bag_path
```

Append a manual observation while a command mission is active:

```bash
ros2 topic pub --once /model_commander/operator_event std_msgs/msg/String \
  "{data: 'observation: green chair visible beside the white desk'}"
```

The default profile records the append-only decision trace, including a final
`mission_result`, commander status,
the exact bounded JPEGs inspected by the commander, registry/object changes,
hidden action traffic, primitive lifecycle events, navigation context, TF,
odometry, maps, scans, logs, and operator events. Each decision trace event
also includes the latest `/odometry/filtered` pose snapshot, compact motion
progress, perception readiness, navigation ownership, and visibility-coverage
POI context when available, so high-level replay can answer where the robot was
when it planned, dispatched, or finished a primitive and what helper state the
agent saw. It excludes continuous raw
camera images, camera point clouds, SAM masks/annotations/instance point
clouds, and legacy LiDAR point clouds.

Important topics are:

- `/model_commander/recording_event`: parent mission start/terminal lifecycle;
- `/model_commander/decision_event`: planning inputs, validated decisions, and
  command outcomes as append-only JSON;
- `/model_commander/inspected_image`: exact bounded JPEG used for each model
  inspection;
- `/model_commander/status`: latest commander state snapshot;
- `/model_commander/operator_event`: manual milestones and observations;
- `/model_commander/bag_status`: recorder readiness/finalization status;
- `/model_commander/last_bag_path`: latest bag directory.

Every directory contains `muto_recording_manifest.json` with schema
`command_mission_v1`, the full bounded start event, ROS distribution, Git
revision/dirty state of the shared recorder engine, topic scope, and exclusion
profile.

Replay only on a development machine or isolated ROS domain because the bag
may contain command and velocity topics.
