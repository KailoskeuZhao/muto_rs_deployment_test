# Default Bags And Mission Monitoring

This is the authoritative overview of recording in the Muto deployment. The
normal Nav2 pipeline records one compact navigation session, while the v2
mission launch opens one high-level mission-scoped recorder. Together they
reconstruct robot motion and agent decisions without recording the full sensor
graph.

All standard Muto bags belong under:

```text
/opt/muto_rs_ws/bags
```

## Default Monitoring Layers

The normal startup launches enable two default layers:

| Layer | Opens when | Closes when | Main question it answers |
| --- | --- | --- | --- |
| Nav2 session bag, `muto_nav2_*` | `muto_nav2_pipeline_launch.py` passes its Nav2 readiness gate. | The pipeline shuts down or `/muto/nav2_bag/stop` is called. | What goals, paths, obstacle scans, velocity commands, poses, action states, and navigation diagnostics occurred during this robot session? |
| v2 mission bag, `muto_command_v2_*` | A `/muto/mission` goal is accepted. | The v2 mission succeeds, is canceled, or fails terminally. | What did the agent know, decide, dispatch, observe, and achieve over the complete mission? |

The v2 bag spans registry checks, model requests and validated decisions,
bounded tool dispatch/results, reachability reports, navigation context, and
the final mission result. Its decision events contain the current board and
progress evidence. Raw camera frames are not recorded by this profile.

Direct Nav2 goals, joystick activity, and `/save_map` without a mission are
outside the v2 mission lifecycle, but their navigation effects are still
captured by the default Nav2 session bag.

## Default Scope

The automatic profiles retain the information needed to reconstruct high-level
behavior:

- append-only v2 decisions, mission status, and typed tool outcomes;
- registry revisions, candidate IDs, confirmation evidence, and object summaries;
- POI selection, reachability result, Nav2 action state, goals, paths, and command topics;
- map state, TF, LiDAR and derived camera obstacle scans, fused/scan odometry, and
  relevant diagnostics;
- lifecycle events, recorder manifests, Git/build provenance, and manual
  operator notes.

They exclude continuous raw camera images, camera point clouds, SAM masks and
annotations, instance point clouds, and legacy LiDAR point clouds. This keeps
the monitor useful without recreating the earlier oversized perception bags.
Use a dedicated perception capture when raw-image replay is the purpose of the
test.

Each directory contains `muto_recording_manifest.json`. Inspect that file
before analysis: it records the schema, resolved mission context, ROS
distribution, recorder revision and dirty state, and the active topic filter.

## Find The Active Or Latest Bags

v2 mission:

```bash
ros2 topic echo --once --qos-durability transient_local \
  /muto/mission_recorder_manifest
ros2 topic echo --once --qos-durability transient_local \
  /muto/mission_recorder_status
```

`recording_ready` means the scoped MCAP is open. `recording_finalized` means
the recorder has closed the file and written final metadata.

## Add A Human Observation

Operator annotations are currently external to the v2 high-level allowlist;
annotate the MCAP or use the Nav2 diagnostic event topic for a field note.

## Optional Diagnostic Bags

These modes are not part of the compact defaults:

| Recorder | Start command | Purpose |
| --- | --- | --- |
| Full-context Nav2 bag, `muto_nav2_*` | `ros2 launch muto_nav2_bag record_nav2_bag_launch.py params_file:=$(ros2 pkg prefix muto_nav2_bag)/share/muto_nav2_bag/config/nav2_bag_full.yaml` | Short, manually bounded deep dive adding full costmaps, raw/source odometry, and high-rate action feedback. Do not run this beside the automatic recorder unless duplicate capture is intentional. |
| Odometry bag, `muto_odometry_*` | `ros2 launch muto_odometry_bag record_odometry_bag_launch.py bag_path:=<path>` | Controlled source-data capture for replaying and comparing odometry configurations. It intentionally excludes derived odometry outputs under test. |

Stop the Nav2 recorder with `Ctrl-C` or:

```bash
ros2 service call /muto/nav2_bag/stop std_srvs/srv/Trigger "{}"
```

Stop an odometry recorder with `Ctrl-C` so rosbag2 finalizes its metadata.

## Retention

Both automatic recorder profiles default to `max_bag_directories: 20`.
After any automatic bag finalizes cleanly, it counts recognized directories with
the prefixes `muto_command_v2_`, `muto_nav2_`, and
`muto_odometry_` under the shared parent and removes the oldest until only 20
remain. Unrelated directories are untouched.

This is a total shared count, not 20 bags per recorder type. The Nav2 recorder
now prunes directly after finalization; odometry bags participate when a later
automatic bag prunes. Set `max_bag_directories` to `0` only when retention is
managed externally.

## Replay Safety

Complete mission and navigation bags may contain `/cmd_vel` and action topics.
Replay them only on a development machine or isolated ROS domain, and select
only the required topics when possible:

```bash
export ROS_DOMAIN_ID=77
ros2 bag play <bag-directory> --clock --topics <topic> [<topic> ...]
```

Do not replay a complete bag on the live robot domain.

Package-specific details remain in the
[`muto_command_layer_v2`](../src/muto_command_layer_v2/README.md),
[`muto_nav2_bag`](../src/muto_nav2_bag/README.md), and
[`muto_odometry_bag`](../src/muto_odometry_bag/README.md) READMEs.
