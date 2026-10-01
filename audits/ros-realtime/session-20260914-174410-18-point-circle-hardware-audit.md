# MPPI 18-point hardware circle audit: 20260914_174410_mppi_demo

## Scope and evidence

This is a passive audit of the real-hardware session at:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260914_174410_mppi_demo`

The 117.69 s ROS bag contains the 23-point trajectory action, controller and
estimator diagnostics, marker observations, POS/ENC feedback, all command-path
topics, and 264 causal response forecasts. The active trajectory interval was
approximately 41.39 s. No hardware action was performed during this audit.

The comparison baseline is the compensated 36-circle-point session
`20260913_203503_mppi_demo`. Both trials used the same 10 mm circle geometry,
UKF, fixed causal Jacobian, and backlash compensation. The current manifest
confirms `adaptation_enabled=false` and `backlash_compensation_enabled=true`.

## Executive result

Reducing the circle from 36 to 18 points made the trial shorter and improved
some median/model-direction metrics, but it did not solve path tracking. Large
errors became worse.

- The action processed all 23 targets and ended normally. Five targets
  advanced before their 2 s budget expired; 18 consumed the full budget.
- Target-relative action error was 3.50 mm median, 3.96 mm mean, 7.40 mm p95,
  and 11.51 mm maximum. The final target error was 4.63 mm.
- Closest-circle geometric error during the circle was 2.83 mm median,
  3.75 mm RMS, 7.60 mm p95, and 11.48 mm maximum. The tip was within 1.8 mm of
  the continuous circle for 33.0% of circle samples.
- The controller remained active throughout the action, then disarmed
  normally. There were no manager inhibits, deadline warnings, marker
  rejection streaks, or limit faults.
- The Jacobian had one unique snapshot. No online fit occurred.

## Before/after comparison

| Metric | 36 circle points | 18 circle points | Interpretation |
| --- | ---: | ---: | --- |
| Total action targets | 41 | 23 | five approach points in both |
| Circle chord length | 1.79 mm | 3.67 mm | larger commanded segments |
| Active action duration | 64.56 s | 41.39 s | 35.9% shorter |
| Early target completions | 16/41 | 5/23 | fewer points did not improve settle success |
| Closest-circle median | 3.03 mm | 2.83 mm | 6.7% better |
| Closest-circle RMS | 3.32 mm | 3.75 mm | 12.9% worse |
| Closest-circle p95 | 5.25 mm | 7.60 mm | 44.7% worse |
| Closest-circle maximum | 7.09 mm | 11.48 mm | 61.9% worse |
| Samples within 1.8 mm of circle | 21.2% | 33.0% | more time locally near path |
| Predicted/measured response ratio, median | 10.27 | 7.80 | larger moves transmit better |
| Response direction cosine, median | 0.177 | 0.438 | model direction improved |
| Opposite-direction responses | 40.2% | 31.4% | still frequent |

The new trial began closer to the fixed absolute trajectory: its first target
error was 2.76 mm versus 5.42 mm previously. This makes the comparison
favorable to the new run and prevents attributing every difference solely to
waypoint count.

## Tracking interpretation

The larger 3.67 mm point spacing produced clearer motion than the old 1.79 mm
spacing. Forecast direction and scale improved, consistent with more commands
escaping backlash/stiction. However, the controller still held each point
until settled or timed out. It therefore retained the stop--correct--reverse
behavior that excites stick--slip. Larger target jumps also produced larger
overshoot and path departures, explaining the improved median but much worse
tail.

Only two of five approach points and three of eighteen circle points advanced
early. The long failure region over global target indices 5--14 includes
target errors up to 11.51 mm. Global targets 15, 17, and 18 completed early,
after which the remaining targets again timed out. The behavior remains
strongly configuration/history dependent.

## Backlash state

The compensator was active and did not remain stuck in take-up.

| Axis | `TAKEUP` samples | `ENGAGED` samples | Phase transitions | Previous transitions |
| --- | ---: | ---: | ---: | ---: |
| insertion | 139 | 954 | 49 | 69 |
| rotation | 154 | 922 | 32 | 66 |
| bending | 127 | 923 | 7 | 7 |

Absolute transition counts fell because the action was much shorter. After
normalizing by active duration, insertion transition frequency did not improve
materially. Thus point reduction alone did not remove fine reversing behavior.

## Model-response evidence

- Predicted displacement: 2.03 mm median and 3.13 mm p95.
- Measured displacement: 0.190 mm median and 1.96 mm p95.
- Predicted/measured ratio: 7.80 median and 49.87 p95.
- Forecast endpoint error: 1.99 mm median and 3.13 mm p95.
- Direction cosine: 0.438 median; 31.4% were negative and 44.7% exceeded 0.5.

For measured responses above 1 mm, median direction cosine rose to 0.782.
This reinforces the existing policy that Jacobian learning should use
accumulated, clearly transmitted motion rather than fine-adjustment windows.

## Timing, sensing, and safety

- Planning time was 24.39 ms median, 36.77 ms p95, and 48.33 ms maximum,
  entirely below the 60 ms deadline.
- Full marker rewind/correct/replay time was 15.71 ms median, 28.24 ms p95,
  and 32.76 ms maximum.
- Position age was 5.54 ms median and 10.45 ms p95; encoder age was 23.77 ms
  median and 31.72 ms p95; feedback-pair skew was 20.90 ms median and
  22.33 ms p95; marker age was 23.67 ms median and 27.54 ms p95.
- Marker diagnostics reported `TRACKING`. There were 1,154 accepted updates,
  11 accepted no-ops, and one expected startup sample before the rewind buffer.
- All 590 manager safety samples were `MANAGER_READY`. Planned, relayed,
  manager, and device velocity-command paths remained continuous.
- Insertion ranged from 7.52 to 37.98 mm, leaving 1.02 mm to the 39 mm
  autonomous reserve. No limit fault occurred, but margin remains small.

## Conclusion

The 18-point trajectory is a useful diagnostic improvement because it elicits
more informative transmitted motion and completes faster. It is not a robust
tracking solution: geometric p95 and maximum errors worsened sharply, and the
stop-and-settle sequencer continues to excite transmission history.

Do not reduce the waypoint count further as the next remedy. The next
controller-side change should be a continuously advancing, time-parameterized
circle reference with an explicit future-reference horizon, reversal and
command-rate penalties, and closest-path error. That preserves sustained
directional motion without turning the circle into still larger point-to-point
jumps. A later adaptive-J trial should only learn from the high-response,
directionally persistent portions.

## Classification

- **Observed:** safe action completion, healthy ROS command/sensor path, no
  scheduling faults, and fixed-J operation.
- **Observed:** slightly lower median circle error but substantially worse RMS,
  p95, and maximum error with 18 circle points.
- **Inferred-high-confidence:** larger segments escape the transmission
  deadzone more often, while waypoint settling still excites stick--slip and
  causes overshoot.
- **Not implicated:** manager readiness, marker acceptance, freshness gates,
  planner deadline, or command routing.
