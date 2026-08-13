# Muto exploration bag

This package records one diagnostic MCAP bag for each
`/explore_and_record` action. It is a standalone recorder: the command layer
publishes mission lifecycle events and does not own rosbag2 or write files.

In the normal command launch this is the child-detail layer of the default
monitor. The separate command bag spans the entire commander mission; this bag
opens only for the exploration interval. See [Default Bags And Mission
Monitoring](../../docs/bags.md) for the combined monitoring contract.

`command_layer_launch.py` starts this recorder by default. To keep it in a
separate terminal, arm it first:

```bash
ros2 launch muto_exploration_bag record_exploration_bag_launch.py
```

Then suppress only the command launch's duplicate recorder; its lifecycle
handshake remains enabled:

```bash
ros2 launch muto_command_layer command_layer_launch.py \
  launch_exploration_bag_recorder:=false
```

The recorder waits without writing until it receives a `mission_started`
event. It then discovers the topic graph and records the compact diagnostic
profile described below, including hidden action topics and service-event
topics where server introspection is enabled. Success, cancellation, and abort
events close the current bag after a short terminal capture delay.

The recorder supports the ROS 2 Humble API used on the Orin and the newer
Jazzy API used for development. Every bag directory contains
`muto_recording_manifest.json` with the goal context, resolved start event,
ROS distribution, Git revision, dirty flag, and topic scope. Newer rosbag2
versions also mirror these fields into `metadata.yaml`. The manifest records
the active exclusion regex. Humble has no native `all_services` selector; topic
discovery still captures service-event topics that servers expose through
introspection and that pass the filter.

The default directory on the robot is:

```text
/opt/muto_rs_ws/bags/muto_explore_<timestamp>_<goal-id>
```

By default, the recorder keeps the newest 20 recognized Muto bag directories in
that parent directory across command, exploration, nav2, and odometry bags.
Older Muto bags are pruned only after a new bag finalizes. Unrelated
directories are not touched. Set `max_bag_directories:=0` to disable pruning.

Read the latest path and inspect it with:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /explore_and_record/bag_status
ros2 topic echo --once --qos-durability transient_local \
  /explore_and_record/last_bag_path
ros2 bag info <bag-directory>
cat <bag-directory>/muto_recording_manifest.json
```

`recording_ready` means the action-scoped bag is open;
`recording_finalized` means its metadata and MCAP have been closed.

## Manual observation or milestone

While the action is active, publish a short plain-text operator event:

```bash
ros2 topic pub --once /explore_and_record/operator_event std_msgs/msg/String \
  "{data: 'milestone: entered the second room'}"
```

For an observation:

```bash
ros2 topic pub --once /explore_and_record/operator_event std_msgs/msg/String \
  "{data: 'observation: chair visible left of the doorway'}"
```

JSON is optional. The bag receive timestamp supplies the event time, so a
short string is sufficient. The recorder logs a warning if an operator event
arrives while no mission bag is active.

## Topic selection

The default compact profile discovers all topics but excludes:

- raw camera images and their transport variants;
- camera point clouds;
- SAM2 annotated images, masks, instance masks, and instance point clouds;
- legacy LiDAR point-cloud topics.

It retains camera calibration, the derived camera obstacle scan, LiDAR scans,
TF, maps, odometry, action traffic, logs, lifecycle/operator events,
`/sam2/detections`, `/sam2/segments`, `/sam2/stored_objects`, and object markers.

To record raw perception payloads for a dedicated perception trial, clear the
default exclusion:

```bash
ros2 launch muto_exploration_bag record_exploration_bag_launch.py \
  exclude_regex:=''
```

For a narrower contract, configure an explicit `topics` list in
`config/exploration_bag.yaml` or pass `topics_regex`. `topics` and
`topics_regex` are mutually exclusive.

Replay complete bags only on a development machine or isolated ROS domain;
they may contain `/cmd_vel` and other command topics:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock
```
