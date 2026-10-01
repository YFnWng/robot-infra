# Adaptive-MPPI hardware trajectory audit: 20260912_210822

## Scope and evidence

This is a passive audit of the real-hardware session at:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260912_210822_mppi_demo`

The 138.50 s ROS bag contains the controller diagnostics, all 41 target
transitions, action feedback/status, marker measurements, planned and relayed
commands, raw encoder and position feedback, manager safety status, and MPPI
response traces. The controller was configured with the UKF estimator,
hardware command output enabled, and proximal Jacobian adaptation enabled.

## Executive result

The control and safety plumbing remained operational, but trajectory tracking
was poor and the advertised adaptive Jacobian did not adapt.

- The action processed all 41 waypoints and ended normally at 111.07 s.
- Only 8 waypoints met the 1.8 mm tolerance for the required settle time; 33
  timed out. Final error was 4.824 mm.
- Controller feedback error was 3.84 mm median, 6.36 mm p95, and 8.90 mm max.
- The controller remained `ACTIVE` for 75.60 s and then disarmed normally.
  There were no MPPI faults, manager inhibits, deadline faults, marker
  rejection streaks, or limit faults.
- `MANAGER_READY` was present in all 693 recorded safety-status samples.
- The proximal Jacobian had exactly one unique value throughout all 1,045
  response traces. RLS covariance, weight, update norm, and condition number
  were also constant. Thus this session behaved as fixed-J MPPI despite
  `model_adaptation_enabled=True`.

## Tracking behavior

The controller started from a measured tip of approximately
`[21.869, 16.990, 71.363] mm`, close to the fixed YAML reference. The first
five approach points all timed out. Tracking improved temporarily over the
lower portion of the circle: the only settled successes were concentrated in
the neighborhood of waypoints 24--36. Performance then degraded sharply;
waypoint 37 rose from about 2.53 mm to 8.74 mm error and the last waypoint
ended at about 4.84 mm.

The response trace shows a large model-to-hardware gap:

- Predicted displacement magnitude: 2.20 mm median, 3.22 mm p95.
- Measured displacement magnitude over the corresponding causal horizon:
  0.133 mm median, 0.816 mm p95.
- The median predicted/measured magnitude ratio was 13.1. The ratio of the
  mean magnitudes was about 8.0.
- Direction cosine was only 0.098 median; 45.6% of responses pointed more than
  90 degrees away from the prediction, and only 29.8% had cosine above 0.5.
- Endpoint prediction error was 2.19 mm median and 3.28 mm p95.

These measurements support substantial backlash/torsional-windup and local
proximal-model error. They do not support treating the current fixed Jacobian
as an accurate local control map.

## Why adaptation never updated

The robust motion accumulation and reversal guards were active, but every
candidate was rejected before RLS application:

- Active-state RLS reasons: 456 `rls_response_below_floor`, 144
  `rls_accumulating_frames`, 95 `rls_reversal_holdoff`, 41
  `rls_reversal_released`, and 19 `rls_accumulating_motion`.
- No `rls_awaiting_confirmation`, accepted update, mixed-excitation rejection,
  or update-clamp state was reached.
- Candidate action excitation was ample: normalized action norm was 0.758
  median and directional purity was 0.938 median.
- Candidate interface response translation was 0.136 mm median and 1.16 mm
  p95. Rotation was 1.10 degrees median and 4.35 degrees p95.
- The UKF-derived response SNR was the blocking gate. Rotation SNR never
  exceeded 1.36 and translation SNR never exceeded 2.69, while the configured
  minimum was 3.0. Even the largest accepted-motion window therefore could not
  become an RLS candidate.

The lower response threshold is doing the requested job of excluding static
and backlash-dominated windows, but the additional SNR gate makes adaptation
unreachable with the current UKF covariance calibration. This is the first
issue to correct before another adaptive hardware trial. Do not simply remove
all response gating: the trace contains many tiny, directionally inconsistent
responses that would corrupt the Jacobian.

## Timing and ROS execution health

No scheduling failure occurred, but estimator cost remains substantial:

- MPPI plan time: 23.09 ms median, 37.36 ms p95, 46.88 ms max.
- Marker correction: 10.44 ms median, 18.40 ms p95.
- Full rewind/correct/replay path: 18.81 ms median, 31.38 ms p95, 38.61 ms max.
- Estimator callback duration window maximum: 34.34 ms median and 47.59 ms
  p95; estimator timer lateness maximum: 16.23 ms median and 28.56 ms p95.
- Planner timer lateness remained small: 0.415 ms median and 0.689 ms p95.
- Feedback was fresh enough for this run: marker age 21.18 ms median and
  26.13 ms p95; position age 5.86 ms median and 11.01 ms p95; encoder age
  23.89 ms median and 38.39 ms p95.

The split execution structure successfully protected planning and heartbeat
from the expensive estimator path. UKF correction is still the dominant CPU
cost and should be optimized later, but it was not the reason this trajectory
failed.

## Hardware and command path

- Planned commands were produced throughout the active interval and relayed
  to `/teleop/control`, `/manager/control`, and `/device/command_tx`.
- The relay/manager/device path ran near 100 Hz while active; all device command
  messages used predicate 86.
- Position feedback stayed inside configured limits. Axis 0 ranged from 8.935
  to 37.754 mm, leaving only 2.246 mm margin to its 40 mm upper boundary.
  Rotation ranged from -73.28 to +11.75 degrees. No safety latch occurred.
- Marker updates were healthy: 1,371 accepted status samples and one startup
  sample before the rewind buffer. Marker diagnostics remained `TRACKING`.

## Trajectory-definition caveat

The 10 mm circle uses 36 samples including the repeated endpoint, so adjacent
circle points are about 1.793 mm apart—nearly identical to the 1.8 mm waypoint
tolerance. Once the catheter is near the path, a newly selected waypoint can
already satisfy the tolerance without demonstrating meaningful motion. The
settle requirement reduces but does not eliminate this ambiguity. This did not
cause the overall failure (most errors were much larger), but future tracking
metrics should use either tighter evaluation tolerance or arc-length/time-based
tracking metrics independent of the action's transition tolerance.

## Recommended next steps

1. Recalibrate the adaptation confidence gate. Log and compare the UKF
   interface-pose covariance against stationary and deliberately excited
   hardware windows, then set the SNR threshold from empirical distributions.
   The present threshold of 3.0 admitted zero candidates.
2. Keep the existing absolute response floor, accumulated multi-frame motion,
   reversal holdoff, directional-purity check, two-window confirmation, and
   Jacobian gain/direction clamps.
3. Before another circle, run a short, interior-workspace, single-axis
   excitation sequence with long enough dwell to produce clean translation or
   rotation. Confirm at least one bounded RLS update and validate its predicted
   direction on a held-out reversal-free segment.
4. Only after that validation, repeat a trajectory test. Report continuous
   target-to-tip RMS/p95 error and phase lag, not only waypoint success.
5. Keep axis 0 farther from 40 mm during follow-up tests; this run approached
   within 2.25 mm of the hard limit.

## Classification

- **Primary:** estimator/adaptation gate calibration defect. Adaptation was
  enabled but unreachable under the observed covariance/SNR.
- **Secondary:** severe proximal response mismatch consistent with backlash,
  windup, and a nonlocal fixed Jacobian.
- **Not implicated in this run:** ROS command continuity, manager safety state,
  marker acceptance, planner scheduling, or hard-limit handling.
