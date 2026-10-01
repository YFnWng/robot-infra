# MPPI hardware circle audit: 20260913_203503_mppi_demo

## Scope and evidence

This is a passive audit of the real-hardware session at:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260913_203503_mppi_demo`

The 83.01 s ROS bag contains 53,533 messages, including all 41 target
transitions, trajectory-action feedback/status, controller and estimator
diagnostics, marker observations, planned and relayed commands, raw encoder and
position feedback, manager safety state, and 497 causal response forecasts.
No hardware action was performed during this audit.

## Executive result

The ROS, sensor, command, and safety paths remained operational and the action
processed all 41 waypoints without a controller fault. Tracking of the full
circle was nevertheless unsuccessful.

- 17 of 41 waypoints entered the 1.8 mm tolerance; 24 used their full time
  budget. The final waypoint ended at 5.35 mm error.
- Action-feedback error was 3.56 mm median, 3.94 mm RMS, 6.61 mm p95, and
  8.27 mm maximum.
- The action protocol reached terminal status `SUCCEEDED`, meaning the server
  completed the requested schedule. It does not mean that all Cartesian
  waypoints were reached.
- The controller remained `ACTIVE` through the action and disarmed normally at
  71.35 s. No controller fault, manager inhibit, marker rejection streak, or
  hard-limit fault occurred.
- Proximal adaptation was disabled for the entire bag
  (`model_adaptation_enabled=False`). The Jacobian had one unique snapshot, so
  this was a fixed-J trial, not adaptive MPPI.

## Tracking behavior

The controller reached three of the five approach waypoints. Circle tracking
then had strong configuration dependence:

- Waypoints 5--15 all timed out.
- Waypoints 16--21 contained five successes.
- Waypoints 22--27 contained only one success.
- Waypoints 28--35 all succeeded.
- Waypoints 36--40 all timed out, and the last point ended at 5.35 mm error.

The measured tip at the start of the bag was approximately
`[24.257, 18.509, 76.186] mm`. The trajectory retained the previously recorded
absolute plane at `z = 71.381 mm`; the first commanded approach point was
already 5.42 mm from the observed tip. This difference exists despite a near
nominal initial logical configuration and is itself evidence of state/history
dependence. It also means this run should not be interpreted as starting from
the exact tip pose used to construct the YAML path.

## Model-response evidence

The dominant failure is a proximal model-to-hardware mismatch rather than a
ROS scheduling or routing failure.

- Predicted causal displacement magnitude: 2.19 mm median and 3.18 mm p95.
- Measured causal displacement magnitude: 0.183 mm median and 1.47 mm p95.
- Predicted/measured magnitude ratio: 10.27 median, 50.15 p95.
- Forecast endpoint error: 2.15 mm median and 3.29 mm p95.
- Response direction cosine: 0.177 median. 40.2% of responses pointed more
  than 90 degrees away from the forecast; only 37.2% had cosine above 0.5.

Larger realized responses were more informative: among windows exceeding
1 mm measured displacement, median direction cosine was about 0.70. Tiny
responses were much less directionally reliable. This supports retaining the
accumulated-response floor for any online Jacobian update.

## Backlash estimator and compensator

Backlash compensation was enabled and its inferred state evolved throughout
the run; it was not permanently stuck in take-up.

| Axis | `TAKEUP` status samples | `ENGAGED` samples | Phase transitions | Maximum remaining take-up |
| --- | ---: | ---: | ---: | ---: |
| insertion | 204 | 554 | 69 | 8.34 model rad |
| rotation | 135 | 623 | 66 | 7.07 model rad |
| bending | 221 | 515 | 7 | 34.83 model rad |

The remaining take-up was normally zero (median zero on all axes), but axis 0
and axis 1 reversed frequently. The compensation materially changed the
command during take-up; the p95 absolute planned/effective difference was
5.70, 28.46, and 2.32 in the respective logical command units. Compensation
therefore executed, but did not make the fixed global Jacobian locally
accurate across the circle.

The bag also contains many candidate windows labeled
`rls_response_below_floor` and reversal holdoff/release diagnostics. These are
informational because adaptation was disabled; no online Jacobian fit occurred.

## Timing and ROS execution health

- Plan time was 26.18 ms median, 40.18 ms p95, and 72.51 ms maximum against a
  60 ms deadline. Three isolated deadline warnings commanded zero; none formed
  a consecutive fault streak.
- Planner timer lateness was 0.41 ms median and 0.63 ms p95. Snapshot lock wait
  and hold times were negligible.
- Full marker rewind/correct/replay cost was 16.48 ms median, 32.14 ms p95, and
  55.88 ms maximum. This remains a sizable CPU load but did not starve the
  heartbeat or fault the run.
- Position age was 5.22 ms median and 10.68 ms p95; encoder age was 24.66 ms
  median and 47.49 ms p95; feedback-pair skew was 21.49 ms median and
  43.90 ms p95; marker age was 22.97 ms median and 31.87 ms p95.
- All 819 post-startup marker corrections were accepted and all marker
  diagnostics reported `TRACKING`.
- The command path was continuous through planned control, `/teleop/control`,
  manager output, and serial transmit. All 6,443 manager/device commands used
  velocity predicate 86. All 416 recorded manager safety states were
  `MANAGER_READY`.

Two short firmware `MOTION_SUSPECTED:STALL` warnings recovered within about
66 ms and 11 ms respectively. Neither became a confirmed motion fault.

## Joint motion and safety margin

The logical position moved from approximately
`[20.007 mm, 0.027 deg, 0.0004 mm]` to
`[27.074 mm, -34.884 deg, 8.039 mm]`. During the run:

- insertion ranged from 8.125 to 38.606 mm;
- rotation ranged from -88.077 to 10.139 degrees;
- bending ranged from 0.0004 to 8.039 mm.

No hard limit was crossed. However, insertion approached within only 0.394 mm
of the controller's 39 mm autonomous upper reserve. Future trajectories should
retain more interior margin even if their Cartesian targets appear feasible.

## Conclusion and next experiment

This run validates command continuity, manager gating, marker acceptance, and
clean action completion. It does not validate the circular tracking objective.
The fixed causal Jacobian plus backlash compensation is strongly inaccurate in
both response magnitude and direction over significant portions of the path.

The next useful experiment is not an immediate repeat of the same circle.
First perform an interior-workspace adaptive trial with deliberately
response-producing, reversal-separated excitation and confirm bounded
Jacobian updates in the diagnostics. Then rebuild the circle relative to the
tip measured at that run's start (or resolve relative waypoints on goal
acceptance), and compare fixed-J and adaptive-J runs using continuous RMS/p95
tracking error and response direction—not only waypoint completion.

## Classification

- **Primary:** severe configuration/history-dependent proximal response error
  under the fixed causal Jacobian.
- **Secondary:** absolute trajectory start did not match the current measured
  hardware tip; frequent backlash take-up and reversals consume useful motion.
- **Watch item:** isolated planning overruns and high UKF correction cost, but
  neither caused this trial's tracking failure.
- **Not implicated:** ROS command routing, manager readiness, marker validity,
  repeated freshness gates, or hard-limit enforcement.
