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

## Hardware run: 2026-08-28

The branch revision `faf4206` was pulled into the Humble/aarch64 robot
container and rebuilt with `--symlink-install` (17 packages finished).  The
experiment composition reached an active `/bt_navigator`; LiDAR, camera,
localization, mapping, Nav2, and both diagnostic recorders were alive.

The +90-degree probe was accepted by `/spin`, emitted non-zero angular
`/cmd_vel`, and ended `ABORTED` after 29.7 s.  The behavior server reported
`Collision Ahead - Exiting Spin`.  The recorded filtered/scan odometry over
the probe changed by approximately `(-0.10 m, +0.60 m, -0.62 rad)` even though
the request was a positive 1.57-rad turn.  This is not an acceptable in-place
turn and points at the hardware locomotion/odometry path in addition to the
costmap safety stop.

The 180-degree probe was also accepted and ended `ABORTED` after 4.7 s with
the same collision reason.  Its measured change was approximately
`(-0.00 m, +0.04 m, -0.13 rad)`.  A point-navigation probe was deliberately
not sent: the global costmap contained lethal cells about 0.2 m from the
current robot cell, so issuing a goal would not have been a safe locomotion
test.

The logs also reported an observed gait phase rate of about 9.2 Hz versus the
configured 50 Hz, locomotion tick/dispatch delays up to roughly 37 ms, and
`cmd_vel` timeout returning the base to standby after each abort.  The
low-level source bag contains 108,717 messages (835.0 s), including 650
`/cmd_vel` and 41,252 commanded-gait states.  The Nav2 source bag contains
142,663 messages (819.9 s), including 24,574 filtered odometry messages,
13,068 scan-odometry messages, 316 `/cmd_vel_nav` messages, and four `/spin`
action-status messages.

Artifacts on the robot:

* `/opt/muto_rs_ws/bags/muto_locomotion_experiment_20260828_072415_859035`
* `/opt/muto_rs_ws/bags/muto_nav2_20260828_072431_f8723294`

The composition was stopped cleanly after the probes.  This run does not
clear the hardware-locomotion blocker; object-search or commander validation
should wait until the physical gait cadence, turn direction, and in-place
translation are corrected and re-tested in an open area.
