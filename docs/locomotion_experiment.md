# Locomotion experiment

This branch is a bounded diagnostic for the Humble/aarch64 Muto deployment. It
does not change production defaults and it does not invoke the commander or
object perception path.

## Composition

```text
locomotion_experiment_launch.py
  -> v2 hardware composition
  -> base + LiDAR + camera + localization + mapping + Nav2
  -> low-level odometry source bag (cmd_vel, gait, IMU, scans, TF static)
  -> Nav2 source bag (TF, pose, plans, costmaps, cmd_vel, action feedback)
```

Motion is sent only through `v2_nav2_smoke.py`, which uses the real Nav2
`/spin` and `/navigate_to_pose` actions and reports their terminal statuses.
Each probe has a wall-clock timeout. The experiment records both the command
and the evidence needed to distinguish a base-driver stop from localization,
costmap, or Nav2 progress failure.

## Probe order

1. Wait for `/bt_navigator/get_state` to report `active`.
2. Send a bounded +90-degree spin.
3. Send a bounded 180-degree spin.
4. Send a short, preflight-checked `navigate_to_pose` goal in a known-clear
   area, if the map snapshot provides one.
5. Stop the composition and inspect both bags before any object-search test.

The experiment is a diagnostic only: a successful Nav2 action does not prove
the full mission stack or object confirmation path is successful.

