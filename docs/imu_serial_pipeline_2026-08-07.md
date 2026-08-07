# Muto IMU and serial-pipeline audit — 2026-08-07

## Conclusion

The ICM-20948 is not intrinsically a 1 Hz sensor. A live exclusive-serial test
on `new-spider` confirmed that the approximately 1.033 Hz raw-value cadence is
created on the Muto baseboard, before the ROS timer receives the packet.
Increasing the ROS polling rate or the host UART baud cannot make that cached
packet newer.

The stock data path is:

```text
ICM-20948
  -> baseboard STM32 over I2C1
  -> cached raw-IMU snapshot at protocol address 0x61
  -> separately fused Euler attitude at protocol address 0x60
  -> STM32 USART1 at 115200 baud
  -> synchronous Muto Python read on the shared serial lock
  -> /imu/data_raw, /imu/mag_raw, /imu/data_processed
  -> /imu/controller_attitude (recorded raw source)
  -> stationary stability gate
  -> /imu/controller_attitude_imu (default sparse stop-only EKF input)
```

Yahboom's published protocol defines `0x02` as a read request and `0x12` as a
data-return response. Therefore a raw-IMU response type of `0x12` is normal.
The protocol exposes raw IMU address `0x61`, but no ICM register, output-rate,
filter, FIFO, sensor timestamp, sequence-number, or UART-baud command.

## Why the values change at approximately 1.033 Hz

The 2026-08-07 bag showed about 44 successful host reads per second, but the
accel/gyro values changed only about `1.03316` times per second. Thus the UART
reply is responsive while its payload is cached.

The strongest register-level hypothesis comes from the supplied ICM-20948
datasheet:

- gyroscope ODR is `1125 / (1 + GYRO_SMPLRT_DIV)` with an 8-bit divider, so
  its minimum divider-controlled rate is about `4.3945 Hz`, not 1 Hz;
- accelerometer ODR uses a 12-bit divider; divider `1088` produces
  `1125 / 1089 = 1.033058 Hz`, within about 0.01% of the bag's observed
  transition rate; and
- the controller may therefore be refreshing its combined `0x61` cache only
  on this slow accelerometer data-ready event. Before the live cadence test, a
  separate approximately one-second STM32 refresh task was the other plausible
  explanation.

The later live cadence measurement made the divider hypothesis substantially
stronger: 31 raw transitions over 30 seconds had a mean interval of
`967.979 ms`, compared with the divider prediction of exactly `968.000 ms`.
It is still not a register readback, so the firmware configuration remains
unconfirmed. The previously plausible one-second STM32 timer is now a much
poorer fit. Filter bandwidth alone does not impose this update rate.

## Live raw-versus-fused endpoint test

The controller was tested on 2026-08-07 with the robot stationary, no
locomotion commands, exclusive ownership of `/dev/myserial`, and independent
trials for raw address `0x61` and fused-attitude address `0x60`.

| Requested rate | Raw transitions/s | Fused-orientation transitions/s |
|---:|---:|---:|
| 5 Hz | 1.000 | 3.667 |
| 10 Hz | 1.000 | 4.067 |
| 20 Hz | 1.000 | 4.200 |
| 50 Hz | 1.000 | 4.267 |

Every request in the 15-second sweep received a valid `0x12` response: 75,
150, 300, and 750 successful replies per endpoint, with no timeout or invalid
frame. At a requested 50 Hz, raw response latency was `5.815 ms` mean and
`6.721 ms` p95; fused response latency was `4.825 ms` mean and `5.650 ms` p95.

A second 30-second, 50 Hz cadence trial produced:

- raw `0x61`: 31 transitions, `1.03333 Hz`, and `967.979 ms` mean interval;
- fused `0x60`: 121 observable transitions, `4.03333 Hz`, and `199.991 ms`
  median interval; and
- 1,500 valid replies out of 1,500 requests for each endpoint.

The fused transition count is a stationary lower bound because its 0.01-degree
quantization can legitimately repeat. Its approximately 200 ms transition
cadence nevertheless shows a separate roughly 5 Hz controller update loop.
Thus `0x60` bypasses the 1.033 Hz raw cache, but it is not a hidden 50 Hz IMU
stream. It is used only as a guarded stop-time heading check, not as a
high-rate yaw-rate source. Measured-angle, power-cycle, magnetic-disturbance,
and systematic covariance tests remain required.

## First marked attitude bag and scheduling fault

`muto_odometry_attitude_001` recorded 387.79 seconds, 2,491 controller-attitude
messages, and six manual field markers. At rest, 2,489 samples arrived over
250.46 seconds: 9.94 Hz host polling with a 0.200 s median interval between
distinct Euler values. This confirms the roughly 5 Hz producer under a 10 Hz
poller. Positive yaw matched counter-clockwise ROS rotation, negative yaw
matched clockwise rotation, and the final turn crossed the expected +/-180
degree wrap. Sequential turn changes from RF2O and `0x60` agreed within about
0.4 to 3.6 degrees, but the markers were approximate and are not sufficient
for accuracy or covariance estimation.

The same bag exposed a host scheduling fault: only two `0x60` samples occurred
during 136.89 seconds of active gait, with zero samples during stable turns and
a maximum 26.0 second gap. The raw and attitude timers had identical 10 Hz
periods. Raw was registered first and normally consumed the only serial read
that fit after a moving gait phase; the attitude deadline guard then skipped
its aligned callback. This was not rosbag loss. Gait still passed acceptance at
50.00 Hz, with 20.64 ms active p95 and 22.38 ms active p99 intervals and no
missing gait sequence.

The workspace correction replaces both sensor timers with one gait-slotted
scheduler. Every 20 ms control slot sends the gait phase first and services at
most one due telemetry endpoint. Runtime raw polling remains 10 Hz for rollback
diagnostics with its prior timestamp-observation window, while attitude polling
also runs at 10 Hz, and the first raw deadline is
phased 50 ms after attitude. A controller read attempt advances its deadline
without catch-up bursts; a pre-I/O gait-deadline skip retries next slot. The
deadline guard remains intact. `/muto/imu_telemetry_status` publishes cumulative
selection, deferral, attempt, success, failure, duplicate, and skip counters
and is included in schema-4 odometry bags.

## Second marked attitude bag

`muto_odometry_attitude_002` recorded 331.03 seconds after the scheduler fix.
It contained 3,285 controller-attitude messages and 329 scheduler-status
messages. During 110.88 seconds of active gait, 1,110 attitude replies arrived
at 10.01 Hz and contained 575 changed snapshots. The longest active poll gap
was 0.104 seconds. Gait sequence 961 through 17387 was continuous at 49.99 Hz;
active p95 and p99 intervals were 20.70 and 22.24 ms. The scheduler therefore
removed the moving-gait starvation without degrading active gait cadence.

Changed Euler snapshots arrived at about 5.04 Hz overall. Positive yaw matched
counter-clockwise ROS rotation, negative yaw matched clockwise rotation, and
the -180/+180 wrap was continuous. A 2x offline replay retained 99.73% of RF2O
scans. After removing the arbitrary startup offset, controller yaw and RF2O
differed by 1.14 degrees RMS and had 0.988 turn-rate correlation. During the
initial standby interval, RF2O drifted -3.18 degrees while controller yaw
changed only +0.08 degrees. This supports testing `0x60` as a slow relative
heading correction, but the approximate field labels do not prove it more
accurate than RF2O.

The default path now suppresses cached packets and evaluates all changed 5 Hz
samples, but publishes only a startup anchor and one stable circular-mean
correction per stop. A correction needs fresh selected-and-active standby, a
two-second dwell, at least three distinct samples in a one-second window, and
at most one degree of circular yaw span. `robot_localization` receives yaw-only
`/imu/controller_attitude_imu` with `imu0_relative: true`,
`imu0_differential: false`, `(4 deg)^2` variance, and a `1.0` innovation guard.
No controller-yaw message is published while moving, and Euler angles are
never differentiated into angular velocity.

The normalized `_002` replay admitted ten messages total and rejected all 575
changed moving snapshots. Repeating corrections every one or two seconds did
not improve the initial RF2O stationary drift and added more transients, so the
default remains one correction per stationary episode. The field markers were
approximate and were not treated as accuracy ground truth.

Normal on-robot launch:

```bash
ros2 launch muto_slam_mapping muto_nav2_pipeline_launch.py \
  fuse_controller_attitude_yaw:=true
```

Do not expect `/imu/controller_attitude_imu` at 5 Hz: it is intentionally
sparse. Inspect `/muto/controller_attitude_yaw_status` for the full-rate input
and gate counters, then record the schema-4 odometry bag. Roll back by setting
`fuse_controller_attitude_yaw:=false`.

The exact reports and tested source were preserved on `new-spider` at:

```text
/opt/muto_rs_ws/bags/imu_endpoint_probe_20260807
/home/dase-iot-orin/Downloads/imu_endpoint_probe_20260807
```

## Measured host-side congestion

The same 326.4 s bag isolated a second, independent problem:

| Operation | Measured behavior |
|---|---:|
| All gait phases | 46.49 Hz mean; 40.53 ms p95 interval |
| Active-motion gait phases | 44.26 Hz mean; 49.78 ms p95 interval |
| IMU ROS publications | 44.36 Hz, despite only 1.033 value changes/s |
| 18-motor read at 2 Hz | 25.94 ms median; 26.52 ms p95 response |

There were 652 gait intervals overlapping a motor transaction. Of those, 651
exceeded 30 ms, with a 40.62 ms median. The driver, gait, IMU, and motor service
used one executor thread and one serial lock, so a blocking feedback read
prevented the 20 ms gait timer from running.

This also invalidates final `cmd_vel` calibration from that bag. Restoring the
active phase rate from 44.26 to 50 Hz can increase gait speed by roughly 13%
without changing a gait level.

## Changes now in the workspace

- The protocol parser expects the documented `0x12` data-return instruction.
- The custom library can read the controller's fused roll, pitch, yaw, and
  temperature endpoint at address `0x60` without the vendor's fixed 50 ms
  post-request sleep.
- The normal runtime raw-IMU host poll rate remains 10 Hz for rollback and
  diagnostics.
  This avoids worsening the host-observed transition timestamp of its measured
  1.033 Hz cache and biasing the comparison toward `0x60`. Startup calibration
  keeps its independent 10 Hz attempt loop. Neither parameter configures ODR.
- The driver requests fused `0x60` attitude at 10 Hz and publishes every valid
  response, including duplicates, on `/imu/controller_attitude`. Ten requests
  per second avoid the cadence alias observed when polling the roughly 5 Hz
  producer at exactly 5 Hz.
- One 50 Hz gait-slotted scheduler owns both runtime sensor endpoints. It sends
  gait first and permits at most one raw or attitude serial request per slot.
- `/muto/imu_telemetry_status` exposes scheduler and controller-read counters
  and is recorded and optionally replayed by `muto_odometry_bag`.
- `ControllerAttitude` preserves the controller's degree-valued roll, pitch,
  yaw, and raw temperature byte with host receive time. Its diagnostic frame
  remains intentionally empty. A separate stationary-gated adapter maps only
  stable yaw into `imu_link` and publishes gate counters at 1 Hz.
- Runtime IMU reads have an 8 ms response budget and are skipped when they
  cannot fit before the next gait deadline.
- Consecutive identical accel/gyro snapshots are suppressed by default, so a
  cached packet is not assigned a succession of new ROS timestamps.
- Startup calibration now targets ten changed accel/gyro snapshots, is bounded
  to 15 s and 150 attempts, and physically commands standby before sampling.
- Foot odometry is disabled by default. Its package and launch option remain
  available for explicit diagnostic trials.
- A gait phase is emitted as one contiguous batch of six unchanged vendor leg
  frames with one pacing delay, rather than six Python writes and six sleeps.
  The host output queue is drained before a timed sensor request begins, so
  gait bytes do not consume the IMU response budget invisibly.
- Gait-state publication now runs after the serial transaction releases its
  lock; ROS publication time no longer extends bus ownership.

Exact-value suppression is deliberately described as a heuristic. With no
controller sequence or acquisition timestamp, an identical stationary sample
cannot be distinguished perfectly from a cached sample. Set
`imu_suppress_identical_snapshots:=false` only when the duplicate replies
themselves are being studied.

If the baseboard parser does not accept contiguous frames during the first
hardware test, restore the old inter-frame pacing with
`batch_gait_phase_writes:=false` without reverting the other changes.

## Standalone controller-rate probe

Stop the ROS hardware driver first; only one process may own `/dev/myserial`.
Place the robot in standby and support it safely, then run:

```bash
source ~/init_ros_env.bash
source /opt/muto_rs_ws/install/setup.bash

fuser /dev/myserial

ros2 run muto_hexapod_lib_custom imu_serial_probe \
  --endpoints raw,attitude \
  --rates 2,5,10,20,50 \
  --duration 12 \
  --output /opt/muto_rs_ws/bags/imu_endpoint_rate_sweep.json
```

The tool reports response success, mean/p95 response latency, exact raw or
fused-orientation transitions, and inter-transition timing for each host poll
rate. The transition rate is a lower bound, not an authoritative sensor ODR.

## Controller-firmware fix

The proper fix is below ROS. Inspect or ask Yahboom to read back:

- `PWR_MGMT_1`, `PWR_MGMT_2`, `LP_CONFIG`, and `USER_CTRL`;
- `GYRO_SMPLRT_DIV`, `ACCEL_SMPLRT_DIV_1/2`, gyro/accel configuration, and
  `ODR_ALIGN_EN`;
- raw-data-ready status/interrupts and FIFO configuration; and
- auxiliary-I2C/magnetometer ODR and delay control.

For approximately 50 Hz accel and gyro sampling, divider `22` gives about
`48.913 Hz`; divider `21` gives about `51.136 Hz`. The STM32 should refresh the
exported sample from data-ready or FIFO and add at least a controller sequence
number and acquisition timestamp. A streaming joint-feedback packet is also
needed before measured-joint foot odometry can run continuously without
stealing gait deadlines.

At 115200 baud, the current 9-byte request plus 26-byte response takes about
3.04 ms on the wire. That is sufficient for a 50 Hz IMU stream; a 1 Mbaud host
setting is neither needed for this fault nor exposed by the published Muto
protocol.

## Hardware acceptance test

After rebuilding and deploying, record another marked-field bag and require:

- gait mean between 49 and 51 Hz;
- gait p95 interval at or below 22 ms and p99 at or below 25 ms;
- no regular 40–52 ms gaps associated with motor reads;
- nonzero `/imu/controller_attitude`, `/muto/imu_telemetry_status`, and
  `/muto/controller_attitude_yaw_status` bag counts;
- close to 10 Hz successful attitude polls during both standby and active gait,
  with distinct-value transitions reported against the prior roughly 4-5 Hz
  stationary lower bound rather than treated as a hard 5 Hz requirement;
- scheduler status showing no prolonged attitude starvation, bounded
  deadline-skip growth, and raw polling close to its configured 10 Hz;
- marked stationary, positive/negative 90-degree, and full-turn segments for
  sign, wrap, lag, reset, and magnetic-disturbance analysis before any fusion;
- measured controller response and value-transition rates from the standalone
  probe; and
- no final locomotion velocity recalibration until the phase-rate criteria
  pass.

The batching and short runtime timeout are code-tested but still require this
on-robot acceptance test. RF2O code and configuration were not changed by this
work.

## Primary references

- Local supplied datasheet:
  `/home/kailoskeuzhao/Downloads/DS-000189-ICM-20948-v1.5.pdf`
- TDK ICM-20948 product and current documentation page:
  <https://www.invensense.tdk.com/en-us/products/9-axis/icm-20948>
- Yahboom Muto-RS study/download page:
  <https://www.yahboom.net/study/Muto-RS>
- Yahboom baseboard communication protocol:
  <https://drive.google.com/file/d/1Y7h9gGguj3GpUJw788jSx2jZVVWdPeyL/view>
- Yahboom expansion-board introduction:
  <https://drive.google.com/file/d/1GI68g1Z_zTrZURzI3aC2-FfeZTDrviCl/view>
