# Session 20260920_161502 v175 grouped-path stall audit

## Scope and result

This is an observational audit of
`20260920_161502_mppi_sim`. The run loaded the corrected split model:

- `raw_response_during_interface_takeup=[false,false,true]`;
- `takeup_response_free_mask=[true,true,false]`;
- valid plans report `plan_transmission_prediction_applied=True`.

The earlier bug is fixed: axis-2 reversal is no longer rejected merely because
the v175 interface-knob play would cross the bend-axis limit. The new stop is a
different finite-horizon planning deadlock inside the unchanged v171 tendon
history.

## Observed stop

- The path reached 29.700 mm of 80.811 mm, or 36.753%.
- The governor remained `SLOWED`, not faulted, and its speed scale decayed to
  0.0048 as reference error approached 4.99 mm.
- The final closest-path error was 4.648 mm.
- The final simulated joint position was approximately
  `[27.647 mm, -78.563 deg, 3.606 mm]`.
- The last nonzero planned command occurred 46.60 s after the first planned
  command. The following 15.80 s contained only zero plans.
- The estimator remained `TRACKING`; all 1,384 recorded marker updates were
  `accepted` or `accepted_noop`.
- The run ended by user disarm, not by a controller or estimator fault.

At the stop, the target-minus-observed tip error was approximately
`[-4.65, -1.66, +0.73] mm`. The established physical-shaft direction lease was
`[+1, -1, -1]`. Continuing those directions could not reduce the error; the
lowest-cost alternative was an axis-2 reversal.

## Grouped-mode evidence

Mode bit 2 is axis-2 reversal. In the previous run it was infeasible. In this
run it became finite in every valid plan after 20 s, proving that the v175
joint-limit reservation correction took effect.

For valid plans from 30--63 s:

| reversal mask | finite plans | median cost relative to mode 0 |
| --- | ---: | ---: |
| 0, continue/hold | 100% | 0.000 |
| 1, axis 0 | 100% | +5.164 |
| 2, axis 1 | 100% | +13.145 |
| 4, axis 2 | 100% | +1.071 |

The configured first-step switch cost is 1.0. Thus the best axis-2 reversal
had essentially the same tracking cost as hold over the 160 ms rollout and
then lost by the switch penalty. Typical terminal diagnostics were:

- zero terminal error: 4.994 mm;
- selected zero-plan terminal error: 4.994 mm;
- mode-0 total cost: 224.456;
- mode-4 total cost: 225.539.

The optimizer was not vetoed after selection. Both the unrestricted and final
selected candidate were deterministic candidate 0, the exact zero plan.

## Why raw v171 propagation did not resolve the stop

The corrected rollout does send raw motor 2 through the frozen v171 tendon
history. That history itself contains a learned gross motor-play operator.
Replaying the recorded encoder trajectory through that frozen operator gives,
at the stop, approximately 9.9 rad of additional raw motor-2 travel before a
reversal changes the v171 effective tendon coordinate.

The MPPI horizon is only `4 * 0.04 = 0.16 s`. Even at the bounded 4.5 rad/s
axis-2 rate, one horizon spans at most about 0.72 rad. Consequently every
axis-2 reversing candidate ends before the v171 distal branch responds. The
v175 interface coordinate is also correctly held by its independent play
operator. The candidate therefore predicts no useful tip motion, incurs the
switch cost, and loses to hold. Replanning repeats the same comparison without
ever issuing the travel needed to expose the future response.

This is a mode-value/horizon problem, not the former model-boundary or limit
bug.

## Model and timing checks

Prediction agreement for the commands that were executed was good:

- response endpoint error: 0.027 mm median, 0.164 mm p95, 0.563 mm maximum;
- estimated-tip versus observed-tip discrepancy: 0.027 mm median,
  0.035 mm p95, 0.243 mm maximum.

Planner timing remains tight but was not causal in the persistent hold:

- valid-plan elapsed time: 51.1 ms median, 58.3 ms p95, 59.6 ms p99,
  59.9 ms maximum;
- 55 isolated deadline-miss diagnostic intervals;
- maximum consecutive misses: 2, below the simulation fault threshold.

Valid on-time plans independently selected zero for many seconds.

## Required next correction

Do not remove the v171 tendon play or reinterpret v175 knob play as tendon
take-up. Instead, make the grouped reversal mode evaluation event-aware:

1. For each candidate direction mode, compute the remaining v171 tendon-play
   travel from the cloned history state.
2. Advance the raw candidate through that travel as a bounded macro interval,
   while continuing to propagate the independent v175 interface play and all
   physical joint limits.
3. Score the useful short MPPI horizon after the event, plus explicit elapsed
   time, effort, limit, and switch costs for the macro interval.
4. If the reversal mode wins, execute one raw motor command in that direction.
   Replan every cycle from updated history; do not post-process or modify the
   selected plan.
5. Record, per mode, remaining v171 play, macro duration/travel, post-event
   terminal error, and cost decomposition.

This keeps MPPI active during take-up in the meaningful sense: it continues to
choose the direction and rate from the full model, but it evaluates value past
the next modeled response event rather than requiring that event to fit inside
160 ms.

## Path-reference deadlock addendum

The trajectory geometry is also causal. The controller is not presently
optimizing geometric circle progress. MPPI receives the governor's time/phase
reference preview and minimizes Euclidean error to those points. As the error
grows, the governor reduces its speed scale; near the 5 mm pause threshold the
entire preview collapses onto essentially one fixed, likely unreachable point.

Late same-direction rotation measurements confirm why MPPI did not simply
keep rotating. With target error dominated by negative x, continued negative
rotation was predicted to move the tip in positive x while reducing y error.
Representative late predicted displacements were approximately:

| target error (mm) | predicted rotation response (mm) | predicted norm reduction |
| --- | --- | ---: |
| `[-3.51,-1.74,+0.50]` | `[+0.065,-0.123,+0.011]` | -0.004 mm |
| `[-4.02,-1.74,+0.64]` | `[+0.070,-0.151,+0.003]` | -0.007 mm |

Thus continued rotation was locally neutral or harmful for the frozen
reference, even though it could advance the catheter toward later points on
the circle. The optimizer correctly chose hold for its current objective; it
was never offered a reward for leaving the unreachable reference behind.

Continuous tracking therefore needs a geometric path objective and progress
policy in addition to event-aware tendon reversal:

- penalize distance to a forward arc window or closest admissible circle
  point rather than only the phase-locked point;
- reward monotonic tangential progress explicitly;
- permit bounded reference progression when cross-track error reaches an
  irreducible plateau, while retaining the hard-error safety abort;
- keep endpoint capture as a point objective only at the final arc length.

Merely increasing tendon relaxation or the point-tracking horizon cannot make
an unreachable phase reference reachable, and merely loosening the governor
without a path-progress cost can skip the curve without controlling direction.

## Confidence

- Corrected runtime identity and axis-2 mode feasibility: **observed**.
- Persistent zero-plan selection and governor slowdown: **observed**.
- Frozen v171 motor-play remaining travel exceeding the rollout horizon:
  **inferred-high-confidence**, reconstructed from recorded encoders and the
  deployed checkpoint.
- Event-aware grouped mode evaluation as the next correction:
  **design recommendation**; it requires simulation regression before any
  hardware use.

## Target correction implemented 2026-09-20

Finding addressed: the phase-locked point-reference deadlock above. Motor
play, tendon history, take-up estimation, and grouped proposal generation were
not changed.

- Continuous-path MPPI scoring now treats each sampled reference as a local
  forward corridor: it penalizes cross-track displacement and lag behind the
  reference plane, but does not penalize forward overshoot. Final endpoint
  capture still uses the original Euclidean point objective.
- The path governor now advances at a bounded recovery speed after crossing
  the former pause threshold. The recovery speed decays continuously to zero
  at the hard-error boundary; the existing hard-path-error abort remains the
  final capture-tube authority.
- During recovery, the governed phase remains slow but the MPPI preview keeps
  a nominal-speed forward arc window. Only transmission holds and endpoint
  holds collapse the preview to one point.
- Bounded phase catch-up is permitted throughout the hard capture tube, so a
  tip progressing along a path with irreducible residual error is not sent
  back to an obsolete target.
- The reference horizon's already-published tangent vectors are now retained,
  timestamp-resampled, normalized, and passed to MPPI. No ROS interface change
  was required.

Focused verification: `test_path_tracking.py` and `test_mppi.py` pass (46
tests). The full `catheter_control` unit set passes 254 tests with one unrelated
pre-existing estimator-trace fixture failure (`fake.runtime` absent).
