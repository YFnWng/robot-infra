# Coordinate-split hardware and home-interruption audit

Date: 2026-09-19

## Scope

Passive analysis of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260919_154754_mppi_demo/20260919_154754_mppi_demo_0.db3`

This was the first rotation-disabled sparse-point hardware run after separating
the raw calibrated v171 motor coordinate from the proximal Jacobian's virtual
transmitted coordinate. The run ended while returning from point 3 because the
point-4 home transaction timed out.

## Executive result

The coordinate split was active, but it did not restore closed-loop tracking.
All three attempted targets timed out and ended farther from their targets than
they began. The estimator remained healthy; the dominant remaining issue is a
large short-horizon model/plant response mismatch, not marker rejection or a
planner fault.

The final home failure was independent of MPPI. A tendon lower-limit state
transition stopped every motor during the atomic home transaction. Firmware
left the position tracker running but did not schedule the still-needed
insertion residual. Its generic no-progress detector then timed out 500 ms
later because insertion had completed 4.665 mm, narrowly below the required
4.887 mm (25% of the original error).

## Runtime identity

The bag's `/parameter_events` prove the running controller used:

- UKF marker estimator;
- CUDA with 1,024 MPPI samples;
- grouped mode sampling;
- backlash compensation and take-up transactions enabled;
- controller velocity limits `[10, 0, 4.5, 4, 25, 25]`, so catheter rotation
  was disabled;
- hardware command output enabled.

The sibling manifest's `controller_parameters` section contains base launch
defaults rather than the final values overlaid by
`causal_v2_fixed_hardware.yaml`. Its artifact path and hash identify the
profile, but its resolved-parameter map is not authoritative for this run.

The new `model_raw_motor_angle_rad` diagnostic was present. Raw-minus-effective
absolute offsets were substantial, confirming that the two coordinates were
actually distinct rather than accidentally identical:

| Shaft | P50 | P95 | Max |
| --- | ---: | ---: | ---: |
| insertion 0 | 7.678 rad | 8.371 rad | 17.732 rad |
| rotation 1 | approximately 0 | approximately 0 | approximately 0 |
| tendon 2 | approximately 0 | 1.139 rad | 2.881 rad |

## Estimator and scheduling health

- Estimator health was `TRACKING` for 10,890/10,897 traces; the remaining
  seven were startup initialization.
- Marker RMS after correction was 0.298 mm P50, 0.394 mm P95, and 0.554 mm
  P99.
- Total estimator work was 30.94 ms P50, 40.31 ms P95, 48.71 ms P99, with a
  125.72 ms maximum. This exceeds the nominal 20 ms estimator period often,
  but latest-sample replacement prevented a stale-data or estimator-degraded
  fault in this capture.
- Two isolated MPPI deadline misses occurred; neither repeated and neither
  faulted the controller.
- There were no controller fault logs during the three target actions.

Thus scheduling jitter exists, but it does not explain the consistent response
direction error or the failed targets.

## Target outcomes

The circle was frozen from one initial home tip. Rotation was disabled, so
some target error was necessarily outside the reachable bending plane. MPPI
should nevertheless have reduced the reachable component. Instead, the
measured error barely improved initially and then increased:

| Point | Start error | Minimum error | End error | Result |
| ---: | ---: | ---: | ---: | --- |
| 1 | 15.276 mm | 15.216 mm | 23.347 mm | timed out |
| 2 | 17.605 mm | 17.431 mm | 22.906 mm | timed out |
| 3 | 23.784 mm | 23.251 mm | 25.887 mm | timed out |

All three trajectories drove substantial positive insertion and tip-z motion.
Point 1 reached insertion 48.97 mm and tendon 12.00 mm; point 2 reached
49.03/11.57 mm; point 3 reached 43.36/1.45 mm. The measured tip moved mostly
in positive z, while the later error vector required negative z correction.

## Short-horizon prediction audit

The existing response trace selects a later accepted observation rather than
an observation interpolated at the exact nominal 40 ms due time. For this
audit, all 258 forecast endpoints were recomputed from the marker stream by
linear interpolation at forecast root and due timestamps.

- endpoint error: 0.588 mm P50, 1.973 mm P95, 2.872 mm P99, 3.417 mm max;
- predicted displacement norm: 0.536 mm P50, 1.622 mm P95;
- measured displacement norm: 0.173 mm P50, 0.866 mm P95;
- median direction cosine: -0.072;
- 51.6% of valid responses opposed the predicted direction;
- the response observation selected by the online trace was 35.8 ms late P50
  and 64.4 ms late P95 relative to forecast due time.

A full-rank two-axis least-squares diagnostic over 154 windows with meaningful
encoder motion gave these base-XYZ response rows in mm per shaft radian:

```text
measured plant (axis 0, axis 2):
[[-0.018, +0.012, +0.526],
 [-0.141, -0.122, +0.086]]

model forecast (axis 0, axis 2):
[[+0.556, -0.568, -0.036],
 [-0.463, -0.170, +0.276]]
```

The excitation matrix rank was two with condition number 1.33. The tendon
direction improved substantially relative to the previous 20260916 audit:
cosine 0.946 here versus approximately 0.72 previously, and forecast/plant
gain improved from 3.96 to 2.75. The insertion direction improved from cosine
approximately -0.66 to -0.086, chiefly because the erroneous predicted
negative-z response disappeared. It remains almost orthogonal to the measured
response: the plant moved predominantly +z, while the model predicted a large
+x/-y response with almost no z. Its forecast/plant gain was 1.51.

Therefore the raw/effective coordinate split corrected part of the structural
error, especially tendon direction and insertion z sign, but the fixed
proximal response model still gives the wrong insertion response direction and
overpredicts response magnitude. The correction is retained because it
enforces the v171 artifact contract; reverting it would reintroduce the known
absolute-coordinate violation.

## Home interruption timeline

At point-3 completion, feedback was approximately
`[39.5494, 0.0203, 1.4477]`. The experiment requested
`[20, 0, 0]` at speeds `[4, 25, 2]`. Identical commands were observed on
`/teleop/control`, `/manager/control`, and `/device/command_tx`.

The firmware moved insertion and tendon to approximately
`[34.8846, 0.0203, -0.0013]`. At `1789847765.182`, tendon crossing its lower
software boundary emitted a limit-state transition. The current firmware
stopped every motor at that transition. At `1789847765.682`, its position
tracker emitted `POSITION_TIMED_OUT`, mask `0x05`, with errors
`[-14.8860, -0.0203, +0.0013, 0, 0, 0]`.

This is an observed firmware state-machine defect, not ROS command loss:

1. the home command was received and executed;
2. the boundary transition intentionally stopped every axis;
3. the position transaction retained axes 0 and 2 as active;
4. no immediate residual move was scheduled;
5. the endpoint no-progress rule evaluated the externally interrupted segment
   as if its driver had independently stopped.

## Implemented remediation

`PositionMoveTracker::resumeMaskAfterExternalStop()` now rebases its local
progress detector when an explicitly known external all-axis stop occurs. It
returns only active axes still outside encoder tolerance. Axes already at the
target remain stopped and finish their normal settling interval.

`checkLimitSwitches()` schedules that returned mask through the existing
position-correction path immediately after stopping all motors. That path still
rechecks finite residuals, motor range, current limit direction, nonzero speed,
and latched firmware faults before coordinated re-enable. Any invalid residual
rejects the transaction fail-closed. The original transaction start time and
45-second hard timeout are preserved; a limit interruption neither resets the
hard timeout nor consumes a normal endpoint retry. Velocity-mode behavior is
unchanged: a limit transition remains a global stop until a fresh command.

The native firmware regression suite passes with C++17, `-Wall -Wextra
-Werror`. A full Teensy sketch compile was unavailable in this environment
because no Arduino CLI is installed; compile and flash verification remains
required on the workstation's normal Teensy toolchain.

## Classification and next test

- Position command delivery and boundary-triggered interruption: observed.
- Firmware interruption mechanism: source-confirmed.
- Coordinate split active: observed.
- Healthy marker estimator: observed.
- Split improved some local response directions: observed in cross-session
  diagnostic fits.
- Fixed insertion response remains wrong: observed.
- Target reachability with rotation disabled: partly constrained and unknown
  per point, but does not explain movement away from the closest reachable
  point.

After reflashing, first test one guarded `[20,0,0]` home from an interior
nonzero insertion/tendon state and confirm that the tendon boundary event is
followed by continued insertion and `POSITION_COMPLETE`. Only then repeat the
sparse-point controller experiment. Because all three target trials diverged,
do not interpret successful homing as evidence that the controller is ready
for broader trajectories.
