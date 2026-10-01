# Adaptive MPPI audit: 20260912_192354_mppi_demo

## Scope

Passive reconstruction of the hardware bag at
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260912_192354_mppi_demo`.
The manifest records UKF estimation, adaptive Jacobian enabled, 32 MPPI
samples, four 40 ms rollout steps, 15 Hz planning, 100 Hz command heartbeat,
and hardware output enabled. No hardware actions were performed during this
audit.

## Result

The final fault was a real upper insertion-limit crossing, but it did not
start from the 20 mm home configuration. Three earlier target episodes had
already moved insertion to 33.46 mm. The final +10 mm base-z tip target then
caused MPPI to request approximately +6.96 mm of logical insertion, reaching
40.000225 mm. Controller intent, manager output, and serial command trace are
identical. This excludes command relay substitution.

The adaptive Jacobian did not adapt at all. Its 6x3 value remained exactly
equal to the v174 initialization for the entire run; RLS weight and update norm
were always zero. The dominant diagnostic was `rls_accumulating_motion` even
while encoder motion accumulated over many accepted camera frames. This
revealed a per-frame threshold defect in the robust adaptation gate.

## Target and state sequence

| Episode | Target offset inferred from target log | Start insertion | End insertion |
| --- | --- | ---: | ---: |
| 1 | +3 mm x | 19.999 mm | 26.020 mm |
| 2 | -5 mm x, +7 mm y | 26.020 mm | 25.866 mm |
| 3 | -5 mm x, +5 mm y | 25.866 mm | 33.742 mm |
| 3 re-arm | unchanged target | 33.742 mm | 33.459 mm |
| 4 | +10 mm z | 33.459 mm | 40.000 mm / fault |

The final active interval ran from ROS time 1789255712.453 to
1789255714.923. Integration of the transmitted logical commands over that
interval is approximately `[+6.96 mm, -33.18 deg, -0.79 mm]`. The first
invalid POS sample was `[40.000225, 12.578627, 3.470374, 0, 0, 0]`, preceded
by `[39.996639, 12.588751, 3.495824, 0, 0, 0]`.

## Jacobian and response evidence

- RLS reasons: 2,220 `rls_accumulating_motion`, 484
  `rls_accumulating_frames`, 398 `rls_response_below_floor`, 26 reversal
  holdoff, and 18 reversal releases. There were no confirmation or update
  states.
- Interface response translation was 0.037 mm median, 0.135 mm p95, and
  1.809 mm maximum. Rotation was 0.094 deg median, 1.573 deg p95, and
  15.030 deg maximum.
- The causal 40 ms tip forecasts overpredicted response: predicted response
  norm was 1.540/2.800/4.190 mm at p50/p95/max, versus measured
  0.101/0.470/3.318 mm.
- Median response direction cosine was 0.062; its p05 was -0.674. This
  confirms a large local model-response gap but does not by itself identify
  catheter mechanics versus interface-state estimation as the source.

## Corrections

1. **Accumulated-motion RLS gate.** Sub-threshold motor increments now
   accumulate across accepted camera frames before counting a motion interval.
   Static frames do not accumulate, the response magnitude/SNR floors still
   reject backlash without visible interface response, and reversal holdoff is
   evaluated on the accumulated direction. Existing confirmation, column
   bounds, and safety gates remain in force.
2. **Autonomous insertion reserve.** The `imricor_test` controller contract now
   reserves 1.0 mm at each insertion boundary, giving an autonomous operating
   range of `[1, 39]` mm while retaining the manager's hard `[0, 40]` validity
   range. This is not a widened tolerance.
3. **Heartbeat reprojection.** Every 100 Hz MPPI command heartbeat is
   re-projected against the newest POS feedback and the operational reserve.
   A cached 15 Hz plan can therefore no longer continue driving outward while
   the latest position approaches the reserve.
4. **Additional RLS evidence.** Diagnostics now include accumulated motion
   interval count, normalized action norm, directional purity, and rotation
   and translation SNR so the next run can show exactly which adaptation gate
   passed or failed.

## Verification boundary

Unit tests cover scalar/batched operational projection, planner rollout-step
projection, and slow multi-frame RLS accumulation. A rebuilt and restarted
stack is required before another hardware trial. The next trial should begin
near 20 mm and use one target episode per bag; confirm axis-0 commands taper to
zero before 39 mm and inspect `model_rls_reason` before interpreting the
adapted Jacobian.
