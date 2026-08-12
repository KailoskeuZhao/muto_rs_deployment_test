# Muto Nav2 bag

This package records a standalone, compact diagnostic bag for navigation. It
starts recording as soon as its launch file runs and is independent of the
exploration mission recorder.

The default profile keeps TF, maps, fused and source odometry, processed IMU,
filtered laser scans, goals, paths, costmaps, controller commands, Nav2 action
progress, frontier targets, diagnostics, and lifecycle events. It deliberately
does not record camera images, point clouds, SAM output, or `/bond`. MCAP's
`zstd_fast` preset compresses the repeated maps and costmaps by default; use
`storage_preset:=none` only when measuring uncompressed recorder throughput.

## Record

Start the normal robot/Nav2 pipeline first. In another sourced terminal:

```bash
ros2 launch muto_nav2_bag record_nav2_bag_launch.py
```

The default output is:

```text
/opt/muto_rs_ws/bags/muto_nav2_<timestamp>_<session-id>
```

Read the current path or status:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /muto/nav2_bag/path
ros2 topic echo --once --qos-durability transient_local \
  /muto/nav2_bag/status
```

The bag also contains a manifest plus snapshots of the active Nav2, frontier,
SLAM, and both behavior-tree files. The default geometric locomotion mapping
has no external profile; `/muto/motion_command_state` records its profile ID,
selected amplitudes, and configured/observed phase rates. If the pipeline used
`locomotion_command_mapping:=calibrated`, pass that measured profile explicitly:

```bash
ros2 launch muto_nav2_bag record_nav2_bag_launch.py \
  nav2_params_file:=/absolute/path/to/active_nav2_params.yaml \
  locomotion_calibration_file:=/absolute/path/to/active_muto_profile.yaml
```

## Add a manual event

A short string is enough; the bag supplies the timestamp:

```bash
ros2 topic pub --once /muto/nav2_bag/event std_msgs/msg/String \
  "{data: 'goal: doorway at the far end of the room'}"
```

Other useful examples are `observation: robot hesitated at chair`,
`milestone: entered room 2`, and `result: reached target`.

## Stop cleanly

Press `Ctrl-C` in the recorder terminal, or finalize it from another terminal:

```bash
ros2 service call /muto/nav2_bag/stop std_srvs/srv/Trigger "{}"
```

After the service reports success, the recorder process remains available for
status queries but that session is closed; start a new recorder process for a
new bag. Check it with:

```bash
ros2 bag info /opt/muto_rs_ws/bags/muto_nav2_<timestamp>_<session-id>
```

## What is and is not captured

The complete allowlist is in `config/nav2_bag.yaml`. Two command topics are
kept intentionally: `/cmd_vel_nav` is the controller output and `/cmd_vel` is
the final command after velocity smoothing. `/muto/motion_command_state`
records how that SI command was projected into integer gait levels, the
feed-forward achievable twist, projection flags, and the profile ID. It is
republished with every motor phase so selected and active levels plus the
pending flag remain time-aligned. `/muto/commanded_gait_state` retains its
backward-compatible stance/swing and continuous-foot-target schema. Both
translated occupancy-grid costmaps
and exact Nav2 raw costmaps are retained; their duplicate grid data compresses
efficiently. If the exploration mission recorder is also active, its transient
status and bag path are retained to cross-link both recordings.

On the deployed ROS 2 Humble system, ordinary action goal/result requests are
services and are not recordable unless service introspection is enabled. Nav2
feedback/status, generated plans, and this stack's goal-mirror topics are
recorded. For a direct `NavigateToPose` action sent by another client, publish a
short `/muto/nav2_bag/event` describing the target as well.

Replay bags only on an isolated ROS domain because they contain `/cmd_vel`:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock
```
