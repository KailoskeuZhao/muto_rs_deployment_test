# Muto Nav2 bag

This package records a compact session-level diagnostic bag for navigation. It
starts after the normal Nav2 pipeline passes its readiness gate and remains
independent of the v2 mission recorder.

It is enabled by default by `muto_nav2_pipeline_launch.py`, covering direct
Nav2 goals and the navigation portions of commander missions. Set
`launch_nav2_bag:=false` only when recording is intentionally disabled. See
[Default Bags And Mission Monitoring](../../docs/bags.md) for how the layers
relate.

The default profile keeps TF, map state, fused pose and scan-odometry response,
the two filtered obstacle scans, goals, paths, controller commands, compact
action state, selected POI/object targets, diagnostics, and lifecycle
events. It deliberately omits full costmaps, raw sensors, high-rate action
feedback, camera images, point clouds, SAM output, and `/bond`.

For a short deep-dive capture, opt into the preserved broad profile:

```bash
ros2 launch muto_nav2_bag record_nav2_bag_launch.py \
  params_file:=$(ros2 pkg prefix muto_nav2_bag)/share/muto_nav2_bag/config/nav2_bag_full.yaml
```

## Record

The normal pipeline starts it automatically. For a standalone interval against
an already-running Nav2 graph:

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

When included by the pipeline, the bag also contains a manifest plus snapshots
of the active Nav2, POI-grid, SLAM, and both behavior-tree files. The default
geometric locomotion mapping
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
records how that SI command was projected into gait output, the feed-forward
achievable twist, projection flags, and profile ID. The default bag uses scans
and paths instead of continuously duplicating full local/global costmaps. Use
`nav2_bag_full.yaml` when exact costmap replay and action feedback are required.

On the deployed ROS 2 Humble system, ordinary action goal/result requests are
services and are not recordable unless service introspection is enabled. Nav2
status, generated plans, and this stack's goal-mirror topics are recorded. For
a direct `NavigateToPose` action sent by another client, publish a short
`/muto/nav2_bag/event` describing the target as well.

The recorder participates directly in the shared default retention cap of 20
recognized Muto bag directories and prunes after clean finalization. Set
`max_bag_directories:=0` only when retention is managed externally.

Replay bags only on an isolated ROS domain because they contain `/cmd_vel`:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock
```
