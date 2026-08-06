# Muto exploration bag

This package records one diagnostic MCAP bag for each
`/explore_and_record` action. It is a standalone recorder: the command layer
publishes mission lifecycle events and does not own rosbag2 or write files.

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
event. It then records the discovered topic graph, hidden action topics, and
service-event topics where server introspection is enabled. Success,
cancellation, and abort events close the current bag after a short terminal
capture delay.

The default directory on the root-run robot is:

```text
/root/.ros/bags/explore_and_record/muto_explore_<timestamp>_<goal-id>
```

Read the latest path and inspect it with:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /explore_and_record/bag_status
ros2 topic echo --once --qos-durability transient_local \
  /explore_and_record/last_bag_path
ros2 bag info <bag-directory>
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

The default all-topic mode is intended for diagnosis and may grow quickly from
camera and point-cloud traffic. Configure an explicit `topics` list in
`config/exploration_bag.yaml`, or select/exclude topics at launch:

```bash
ros2 launch muto_exploration_bag record_exploration_bag_launch.py \
  topics_regex:='(/tf.*|/map|/odom|/scan|/sam2/.*)' \
  exclude_regex:='/camera/.*/image_raw'
```

`topics` and `topics_regex` are mutually exclusive.

Replay complete bags only on a development machine or isolated ROS domain;
they may contain `/cmd_vel` and other command topics:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock
```
