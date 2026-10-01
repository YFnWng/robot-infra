# 2026-09-20 v175 grouped-MPPI no-rotation axial audit

Session: `20260920_165501_mppi_demo`

## Verdict

**PASS for the axial gate.** Both independent targets reached the 1.8 mm
tolerance, logical and manager-forwarded rotation remained exactly zero, the
physical rotation position and encoder count remained zero, and there were no
controller faults, manager interventions during either target, marker
rejections, or planner deadline misses.

This result authorizes construction and simulation verification of the gated
two-axis sparse target block. It does not yet authorize a continuous hardware
path.

## Runtime identity

Controller diagnostics observed:

- `command_output_enabled=True`;
- `marker_estimator=ukf`;
- `model_adaptation_enabled=False`;
- `controller_velocity_max=[10,0,4.5,4,25,25]`;
- `compute_device=cuda:0`;
- estimator health `TRACKING`, with valid POS and ENC feedback.

The profile therefore passed the experiment's new fail-closed identity check.

## Closed-loop outcomes

| Target | Duration | Action result | Measured tip displacement | Device POS change, axes 0..2 |
| --- | ---: | ---: | --- | --- |
| frozen base-Z `+5 mm` | 1.405 s | reached, 0.671 mm | `[+0.338,+0.068,+5.275] mm` | `[+8.139,0,+0.070]` |
| frozen base-Z `-5 mm` | 1.304 s | reached, 0.637 mm | `[-0.610,-0.657,-9.536] mm` | `[-7.825,0,+3.841]` |

The second target began 9.31 mm from its frozen absolute target, rather than
5 mm, because returning the encoders to `[20,0,0]` did not reset the physical
tendon/distal history. Grouped MPPI nevertheless converged by combining
retraction and positive tendon actuation. This is the intended
history-preserving hardware behavior.

## Rotation isolation and limits

- Maximum absolute planned rotation velocity: exactly `0`.
- Maximum absolute manager-forwarded rotation velocity: exactly `0`.
- Rotation POS range: exactly `[0,0]`.
- Rotation ENC range: exactly `[0,0]`.
- No rotation-guard trip occurred.
- Axis-0 POS stayed in `[11.5545,28.3862] mm`, leaving more than 11.5 mm
  margin to either `[0,40] mm` hard boundary.
- Tendon-axis POS stayed in `[0.0004,3.8421]`; no limit event was recorded.

There was one late insertion-direction reversal in each target. The second
target also contained one late planned tendon reversal, but no manager-forwarded
tendon reversal before completion.

## Model response

Nineteen causal forecast/observation comparisons were recorded:

- endpoint prediction error: P50 `0.668 mm`, P95 `1.028 mm`, maximum
  `1.040 mm`;
- response direction cosine: P50 `0.914`, P95 `0.990`;
- one initial second-target comparison had cosine `-0.638` while take-up was
  unresolved and measured motion was effectively zero;
- after engagement, the second target's response cosines were predominantly
  `0.90` to `0.99`.

For the first target, measured per-forecast Z response exceeded prediction on
average by `0.391 mm`. For the second, mean Z bias was only `-0.064 mm`, while
the largest systematic mismatch was lateral Y (`-0.360 mm` mean
measured-minus-predicted). Feedback corrected these local errors without
rotation.

## Estimator, manager, and timing

- Estimator traces: 872 total; 865 `TRACKING`, seven startup `INITIALIZING`.
- Maximum consecutive marker rejections: `0`.
- Marker diagnostics during the two actions: 179/179 `TRACKING`.
- Camera-pair skew during ACTIVE: P50 `0.085 ms`, P95 `0.188 ms`, maximum
  `0.196 ms`.
- Planner elapsed time: P50 `48.365 ms`, P95 `51.759 ms`, P99 `56.096 ms`,
  maximum `57.311 ms` against the 60 ms deadline.
- Maximum consecutive planner deadline misses: `0`.
- Manager reported only `MANAGER_READY` during the target interval. The sole
  recorded `FEEDBACK_NOT_QUALIFIED` inhibition occurred during startup, before
  the experiment.

The planner passed but retained only 2.69 ms worst-case deadline margin in this
short run. Preserve the existing miss/fault gates and continue recording timing
under the next, longer two-axis block.

## Comparison with the earlier v174 axial trial

The earlier `20260919_171058_mppi_demo` run also reached both targets with zero
rotation. The new v175 run produced lower reported final errors (`0.671/0.637`
versus `0.723/0.785 mm`) and shorter action durations (`1.405/1.304` versus
`1.774/1.433 s`). These are encouraging but not a controlled performance A/B:
the second-target run-start errors and hidden physical history differed.

## Next gate

Generate targets from the frozen forward model at the accepted hardware home
state, with logical rotation fixed to zero and reviewed joint reserve:

1. one insertion-dominant target;
2. one tendon-dominant target;
3. two diagonal insertion/tendon targets.

Run them as independent sparse targets with guarded `[20,0,0]` returns and the
same runtime/rotation guards. Stop at the first failure. Audit in-plane and
out-of-plane residual, response prediction, reversal count, take-up occupancy,
limits, estimator health, and deadline margin before designing a planar
continuous path.
