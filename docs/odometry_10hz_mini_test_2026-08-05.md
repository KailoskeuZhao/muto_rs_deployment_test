# 10 Hz Joint-Feedback Odometry Mini-Test — 2026-08-05

## Purpose

This test checked whether reading all 18 leg joints at 10 Hz makes measured
stance-foot odometry useful, and whether those blocking controller reads are
compatible with the 50 Hz gait and IMU loops.

The result is negative for production deployment. The higher feedback rate
does create moving `/foot_odom` samples, but their displacement is severely
under-scaled. Meanwhile, the serial reads delay the gait and IMU loops. The
foot input must therefore retain its low EKF trust, and hardware polling above
2 Hz remains experimental.

## Recording

| Item | Value |
| --- | --- |
| Bag | `odom_test_10hz_001` |
| Duration | `370.799 s` |
| Messages | `61,095` |
| Source revision | `04083053e7af20e40815665fbcf604d9f7523e1a` |
| Source working tree | Dirty, according to the recorded metadata |
| Motor snapshots | `3,675` |
| LiDAR scans | `5,890` |
| Raw and processed IMU samples | `15,240` each |
| Commanded gait states | `17,893` |

The bag contains four plain-text markers:

1. `start`
2. `90 degree clockwise approximately`
3. `90 degree counter clockwise to originial pose approximately`
4. `360 degree counter clockwise to originial pose approximately, small drift to negative y direction observed`

These labels are useful for approximate turn closure. They are not numeric
surveyed endpoints, so this test cannot produce ground-truth RMSE or a new
calibrated covariance profile.

## Hardware Timing

| Stream | Mean rate | Median interval | 95th-percentile interval | Maximum interval |
| --- | ---: | ---: | ---: | ---: |
| Motor snapshots | `9.992 Hz` | `100.01 ms` | `102.16 ms` | `420.23 ms` |
| LiDAR | `16.000 Hz` | `62.50 ms` | `62.72 ms` | `335.88 ms` |
| Commanded gait state | `48.647 Hz` | `19.89 ms` | `40.39 ms` | `343.19 ms` |
| Raw IMU | `41.439 Hz` | `20.12 ms` | `40.50 ms` | `334.31 ms` |

The controller motor response took `25.67 ms` at the median and `26.61 ms` at
the 95th percentile. Of the normal loop intervals:

- `3,675 / 17,892` gait intervals exceeded `30 ms`;
- `3,677 / 15,239` raw-IMU intervals exceeded `30 ms`;
- LiDAR timing remained stable apart from one long interruption.

The average UART byte rate is not the limiting factor. A synchronous motor
transaction occupies the shared serial path longer than the gait loop's
`20 ms` period.

## Replay Method

The bag was replayed at 4× wall-clock speed through the current stack:

- stock, unmodified RF2O;
- the parent-owned RF2O covariance/deadband wrapper;
- the `measured` RF2O covariance profile;
- measured-joint foot odometry polling the virtual service at 10 Hz;
- LiDAR-only, LiDAR-plus-IMU, raw-LiDAR-plus-IMU, and
  LiDAR-plus-IMU-plus-foot EKFs.

The replay produced `5,877` messages on each RF2O output branch from `5,890`
source scans. One scan initializes RF2O; another 12 source scans did not produce
odometry under the loaded 4× comparison replay. The filtered scan topic was not
recorded in this derived bag, so the loss cannot be assigned conclusively to
the filter or RF2O polling. It is a replay-scheduling limitation rather than a
field-sensor loss. The missing fraction is about `0.2%` and does not explain
the foot-odometry scale error. An offline pass using every recorded motor
sample confirms the same conclusion.

## Approximate Checkpoints

All positions and accumulated yaw below are relative to the `start` marker.

| Marker | Estimator | X | Y | Accumulated yaw |
| --- | --- | ---: | ---: | ---: |
| Approximate 90° clockwise | Raw RF2O | `+1.009 m` | `-0.003 m` | `-89.15°` |
|  | Filtered RF2O | `+1.006 m` | `-0.030 m` | `-89.78°` |
|  | Measured foot | `+0.073 m` | `+0.054 m` | `-4.84°` |
| Approximate return to original heading | Raw RF2O | `+1.025 m` | `-0.055 m` | `-3.50°` |
|  | Filtered RF2O | `+1.016 m` | `-0.061 m` | `-3.38°` |
|  | Measured foot | `+0.069 m` | `+0.036 m` | `-2.17°` |
| Approximate 360° counter-clockwise | Raw RF2O | `+0.955 m` | `-0.050 m` | `+358.96°` |
|  | Filtered RF2O | `+0.948 m` | `-0.080 m` | `+358.90°` |
|  | Measured foot | `+0.069 m` | `-0.018 m` | `+28.83°` |

The best-case offline foot pass, using every recorded 10 Hz motor sample,
reached only `0.146 m` and `-6.87°` at the first turn marker, and `0.175 m`
with `+37.41°` at the final marker. Accelerated replay is therefore not the
root cause of the under-scaling.

## Foot-Odometry Gate Results

| Gait mode | Recorded samples | Tracking residual median | Samples within 30 mm | Published moving samples during replay |
| --- | ---: | ---: | ---: | ---: |
| `move_x` | `168` | `19.4 mm` | `164` | `70` |
| `turn_z` | `321` | `16.4 mm` | `319` | `122` |
| `standby` | `3,186` | `3.5 mm` | `2,888` | `0` |

The 30 mm command-tracking gate accepts almost every moving snapshot. It is
not the main cause of the scale error. Across all recorded samples, 139
intervals crossed a tripod transition and had zero feet continuously in
stance. Even after accounting for those rejected intervals, the measured-FK
motion remains much smaller than the LiDAR-observed motion.

This suggests that at least one of the following must be resolved before foot
velocity can receive more trust:

- `get_motor_angles` may not expose sufficiently fresh physical joint
  position;
- one-degree joint quantization may be too coarse at the current motion per
  sample;
- the motor snapshot and gait phase may not be synchronized tightly enough;
- commanded stance is not measured ground contact, and slip remains unknown.

## EKF Effect

Compared with LiDAR plus IMU, enabling the current low-trust foot velocity
changed the fused result by:

- `0.000383 m` position RMS, with a `0.008656 m` maximum;
- `0.027677°` yaw RMS, with a `1.505672°` brief maximum;
- zero final difference after the robot returned to standby.

The large foot covariance is doing its job: the invalid scale does not pull
the LiDAR-owned pose significantly. This test provides no justification for
lowering the foot variances or fusing foot yaw.

## Applied Configuration Decisions

1. Production motor polling remains `2.0 Hz`.
2. A foot node or bag recorder request above 2 Hz now requires
   `allow_experimental_high_rate_motor_polling:=true`.
3. Replay opts in automatically because its motor service is virtual and does
   not block the robot controller.
4. Foot odometry remains a low-trust planar-velocity-only EKF input.
5. The RF2O `measured` covariance profile remains the default. Its pose
   diagonal is `2.5e-4 m²` for X/Y and `1.0e-4 rad²` for yaw. The profile is
   applied by the parent wrapper, not inside the RF2O submodule.
6. RF2O translation and yaw deadbands now default to zero; the `0.03 m`
   translation and `5°` yaw jump guards remain enabled.
7. A 20 Hz hardware test must not be attempted with the current blocking
   serial service.

## Next Test Requirements

Before repeating the high-rate hardware test:

- obtain controller-streamed or otherwise non-blocking joint feedback;
- confirm whether joint readings are physical present-position values;
- retain the 50 Hz gait and IMU rates while feedback is active;
- record short numeric event payloads with measured X, Y, wrapped yaw, and
  accumulated yaw;
- use at least one measured straight distance and both turn directions;
- replay at 1× for the final acceptance run.
