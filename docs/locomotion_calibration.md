# Muto physical velocity mapping

## Why the old mapping was wrong

The original driver multiplied every `geometry_msgs/Twist` component by 100
and passed the results to Yahboom's gait generator. Those three values do not
share one physical unit:

- `x` and `y` are foot-path stride amplitudes in millimetres;
- `z` is a tangential half-stride converted into a body-yaw stride using the
  six nominal foot radii;
- the inherited combined gait treated `z` as a degree-like steering value in
  `x * sin(z)` and therefore lost all yaw authority as `x` approached zero.

With 20 phases at exactly 50 phases/s, the commanded-foot geometry predicts
`v_x = 0.01 * x_level` m/s for a pure straight gait. That coincidence does not
calibrate yaw, servo tracking, foot slip, battery/load effects, or delayed gait
phases.

The 2026-08-06 Nav2 bag confirmed the semantic problem. Commands reached the
gait classifier, but the old mixed gait realized only about 10.5% of its
commanded-foot translation and 19.6% of its commanded-foot yaw. The pure-turn
level-20 response was about 0.18--0.19 rad/s. These are RF2O-derived diagnostic
figures, not external ground truth.

## Current correction

The corrected command path is:

```text
/cmd_vel in m/s and rad/s
        |
        v
validated, sign-specific calibration curves
        |
        v
integer x/y/z gait levels + predicted achievable Twist
        |
        v
tripod foot targets -> per-leg inverse kinematics -> controller packets
```

The mapper never extrapolates beyond a calibrated table. It chooses the
closest executable integer level, separately reports quantization,
projection-to-zero, and upper-envelope saturation, and rejects a lateral
command combined with forward or yaw motion. The selected request and
feed-forward prediction are published on `/muto/motion_command_state`. The same
message is emitted for every motor phase with selected and active raw levels
kept separate and a replacement-pending flag. `/muto/commanded_gait_state`
retains its older wire schema so existing bags remain replayable. Neither topic
is measured odometry.

For simultaneous forward/yaw motion, the gait applies the requested finite
planar body transform to every nominal foot location while preserving the
existing alternating-tripod stance and 25 mm swing lift. A planted target is
computed from

```text
p_body(s) = Exp_SE(2)(s * body_stride)^-1 * p_world
```

before the existing per-leg IK. The same construction is used for pure turn,
so the target path remains continuous when forward speed crosses zero. This
makes yaw independent of forward stride and accounts for each leg's actual
radius from the body. Generated extrema are checked against the custom
library's link dimensions and joint limits. IK results are rounded to the
nearest whole controller degree instead of being truncated toward zero.

The common yaw level is converted with the least-squares stance radius
`R_eff = sum(r_i^2) / sum(r_i) = 249.944 mm`. One complete 20-phase cycle has
nominal commanded-foot displacement `4*x_level` mm and yaw
`4*z_level/R_eff` rad. These are kinematics, not proof of ground motion.

Nonzero command changes are queued until the next complete 20-phase boundary.
This prevents Nav2's 20 Hz updates from rebuilding a trajectory several times
inside one roughly 0.4 s gait cycle. A stop, timeout, or transition to standby
still directly commands nominal stance; this is an abrupt safety return, not a
smooth stopping trajectory.

## Provisional profile

The deployed default file is
`yahboomcar_bringup/config/muto_locomotion_provisional_20260806.yaml`.

- Straight and lateral curves are commanded-foot kinematic predictions.
- Positive yaw transfers a descriptive RF2O fit from the inherited pure-turn
  path recorded in the 2026-08-06 bag. Pure turn now uses the exact body-twist
  path, so this curve has not measured the gait that will execute.
- Negative yaw currently mirrors the positive curve.
- No axis has yet been validated against an external position/heading
  reference.

The profile is deliberately named `provisional`. It is valid only with the
configured 50 Hz gait condition; the mapper does not assume that empirical
speed scales linearly with another cadence. The rolling observed phase rate is
telemetry only. A Nav2 bag records the selected/active split in
`/muto/motion_command_state`, retains `/muto/commanded_gait_state`, and copies
the exact calibration YAML beside the MCAP file.

Because levels are discrete, Nav2 limits are request-side limits rather than
strict achieved caps: for example, `0.18 rad/s` currently selects level 19,
whose provisional prediction is `0.18295 rad/s`. The structured projection
fields make that difference explicit.

Use the former conversion only for rollback or raw-level field trials:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py \
  locomotion_command_mapping:=legacy_100
```

`legacy_100` is explicit in telemetry and must not be described as a physical
calibration.

## Controlled field calibration

Disable Nav2 and support the robot while starting. Use a flat marked field,
the normal payload, a recorded battery voltage/state, and three repeats per
condition. Record the odometry bag plus externally measured endpoints.

Run raw-level trials with `locomotion_command_mapping:=legacy_100` so a command
of `0.20` selects level 20. For example, pure forward level 20 is:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.20}, angular: {z: 0.0}}'
```

Pure positive turn level 15 is:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.15}}'
```

Stop immediately after each trial:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

Measure at least:

- straight `x = +/-10, +/-20, +/-30`;
- pure turn `z = +/-10, +/-15, +/-20`;
- mixed `x = 10, 20, 30` with `z = +/-10, +/-20`;
- lateral `y = +/-10, +/-20, +/-30`;
- straight level 20 and turn level 15 at 20, 30, 40, and 50 phases/s.

Hold each case for an integer 20--30 complete cycles after settling. Record
start/end tape displacement and externally referenced yaw, elapsed time,
completed cycles, surface, payload, battery state, and profile/git revision.
Fit displacement and yaw per completed cycle first. Only then convert to
velocity at the tested cadence. Servo lag means the relationship with phase
frequency must be measured rather than assumed linear.

After updating the YAML, rebuild/restart and verify:

```bash
ros2 topic echo /muto/motion_command_state
```

Nav2 velocity limits must remain within the new measured envelope. Do not add
a fast RF2O PI loop at the gait phase rate; if closed-loop trim is later added,
it should be slow, bounded, and evaluated only at complete gait cycles.

## Technical references

- Gao et al., *Constrained Predictive Tracking Control for Unmanned Hexapod
  Robot with Tripod Gait*, Drones 6(9), 246 (2022),
  <https://doi.org/10.3390/drones6090246>.
- Tam et al., *OpenSHC: A Versatile Multilegged Robot Controller*, which
  accompanies the open Syropod high-level controller's
  body-twist/foot-stride implementation,
  <https://arxiv.org/abs/2006.04424>.
