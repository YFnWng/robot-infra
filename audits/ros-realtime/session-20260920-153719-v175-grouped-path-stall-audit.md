# Session 20260920_153719 grouped-MPPI path-stall audit

## Scope

Read-only audit of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260920_153719_mppi_sim/20260920_153719_mppi_sim_0.db3`

The bag has no `metadata.yaml` or runtime manifest, but the SQLite topic and
message tables are intact. Runtime identity below is therefore inferred from
recorded diagnostics, not a launch manifest.

## Outcome

The controller did not fault at the observed stop. It remained `ACTIVE`, and
the path action was later canceled. Path progress reached 33.451 mm of
80.811 mm (41.39%). The apparent stop began near 30% when the path governor
entered `SLOWED` and grouped MPPI increasingly chose candidate zero.

This is a constrained-optimization deadlock, not a perception failure:

1. The established shaft-direction lease was `[+1, -1, -1]`.
2. Following the upper arc required a direction change. The implementation
   marked all modes reversing motor axis 2 infeasible because it treated the
   v175 *interface-knob* play as a tendon take-up transaction and reserved its
   full reversal travel against the bend-axis lower joint limit. This is a
   model-boundary error: v175 interface play is a parallel motor-to-interface
   branch, while raw motor 2 must continue to drive the frozen v171 tendon
   history during that travel.
3. The remaining axis-0/axis-1 reversal modes were feasible, but their switch
   and take-up costs exceeded their predicted 160 ms tracking benefit.
4. Candidate zero therefore won. The path governor slowed the reference as
   cross-track error approached 5 mm, preventing the error from growing enough
   to overcome the fixed reversal cost. This closed the deadlock loop.

## Evidence

### Path behavior

| Elapsed | Progress | Cross-track error | Speed scale | Governor |
|---:|---:|---:|---:|---|
| 10.0 s | 17.75% | 0.131 mm | 1.000 | RUNNING |
| 18.2 s | 30.55% | 1.913 mm | 1.000 | SLOWED transition |
| 25.0 s | 36.23% | 3.521 mm | 0.422 | SLOWED |
| 40.0 s | 40.06% | 4.577 mm | 0.090 | SLOWED |
| 60.0 s | 41.17% | 4.819 mm | 0.020 | SLOWED |
| 74.2 s | 41.39% | 4.861 mm | 0.0077 | SLOWED |

At the final trace, measured tip was
`[40.222, 1.233, 64.571] mm`; the closest path point was
`[35.399, 1.316, 65.169] mm`.

### MPPI selection

At about 25 s the selected four-step logical plan was
`[hold, +insertion, hold, hold]`. Since only the first step executes in
receding-horizon control, this already deferred useful motion. By about 30 s,
the selected plan was hold for all four steps. It remained so through the end.

At 74 s the best total cost by reversal mask was:

| Reversal mask | Best total cost |
|---:|---:|
| 0 (continue/hold) | 222.604 |
| 1 (axis 0) | 227.526 |
| 2 (axis 1) | 240.973 |
| 3 (axes 0+1) | 251.344 |
| 4--7 (any axis-2 reversal) | infeasible |

The selected candidate was deterministic candidate 0, with zero command and
zero reversal mask. The controller's zero-command terminal error was
4.973 mm; the recorded selected terminal error was effectively identical.

### Why the implementation rejected axis-2 reversal

The final logical joint position was approximately
`[27.912, -86.630, 3.254]`. Axis 2 has limits `[0, 15]` mm. The v175 artifact
reported a full reversal window of 28.868 rad, equivalent to 5.470 mm of knob
travel. Reversing the established negative tendon direction reserves a
-5.470 mm bend-axis offset before useful post-take-up motion. That predicts
axis 2 at about `3.254 - 5.470 = -2.216` mm, below its lower limit. The planner
therefore rejected every mode containing axis-2 reversal.

That arithmetic describes the implemented gate, but not the intended model.
The 5.470 mm width comes from the v175 interface-transmission checkpoint, not
from the v171 tendon model. The ROS node currently copies all three v175
interface widths into the generic backlash estimator, feedforward compensator,
take-up arbiter, and MPPI limit reservation. For axis 2 this incorrectly
assumes that the entire knob-interface play produces no useful catheter
response. In the intended parallel model, raw motor-2 motion continues through
the unchanged v171 tendon-history branch even while the v175 transmitted knob
coordinate is stationary. Axis-2 reversal must therefore be rolled out with
two simultaneous coordinates rather than rejected as an all-or-nothing
tendon dead zone.

### Estimator and model health

- Estimator health stayed `TRACKING` throughout the action.
- No marker update was rejected; reasons were `accepted` or `accepted_noop`.
- Estimated-tip versus observed-tip discrepancy was 0.027 mm median,
  0.042 mm p95, and 0.191 mm maximum.
- Backlash state ended `ENGAGED` on all three axes, with zero remaining
  estimated take-up and no active transaction.
- Model adaptation was disabled, as intended for this fixed-model test.

These observations rule out a UKF loss of lock or an active take-up gate as
the immediate stop mechanism.

### Timing

- Planner elapsed time: 50.26 ms median, 57.63 ms p95, 67.07 ms maximum,
  against a 60 ms deadline.
- There were 22 isolated `planner_deadline_miss_zero` diagnostic samples and
  at most two consecutive misses. No repeated-deadline fault occurred.
- Estimator update time: 36.02 ms median, 57.93 ms p95, 76.73 ms maximum.

Timing margin is thin and should remain a separate optimization target, but it
does not explain the persistent zero-command selection: the recorded valid
plans themselves select candidate zero.

## Ranked next actions

1. **Correct the axis-2 model boundary:** retain the v175 knob play for the
   interface-pose branch, but do not reuse it as a v171 tendon dead zone or an
   all-response take-up gate. Candidate rollout must advance raw motor 2
   through v171 tendon history while independently advancing the transmitted
   v175 interface coordinate through its play operator.
2. **Re-evaluate feasibility after that correction:** only the raw joint
   trajectory should be checked against the physical limit. A predicted
   interface take-up offset is not a response-free tendon offset.
3. **Make mode comparison horizon-consistent:** compare reversal modes using
   tracking benefit after their take-up transaction over a horizon long enough
   to expose useful motion, or use a separate mode-level value/terminal cost.
   A fixed multi-second take-up penalty compared against only 160 ms of useful
   rollout structurally favors hold.
4. **Break the governor/optimizer deadlock:** the governor should not freeze
   the reference at an error just below the level needed to justify a mode
   transition. A mode-switch request or persistent hold-with-error condition
   should explicitly trigger recovery or report path infeasibility.
5. **Improve instrumentation:** record per-mode terminal error, per-mode cost
   decomposition, predicted take-up joint offset, and the exact infeasibility
   reason/axis. The current bag exposes total per-mode cost but not those
   components.
6. **Recover timing headroom separately:** the 60 ms planner budget is missed
   occasionally, although it was not causal in this stop.

## Implemented correction (2026-09-20)

The first two actions above are now implemented without changing the frozen
v171 distal/tendon model:

- MPPI carries separate candidate-local raw-shaft and interface-rate
  sequences. Raw motor 2 always advances the v171 tendon history; v175 play
  filters only motor 2's interface-Jacobian input.
- The v175 motor-2 play width is no longer reserved as response-free travel
  against the physical joint limit and contributes no synthetic take-up delay.
- The response-free take-up transaction arbiter covers axes 0 and 1 only when
  a v175 interface checkpoint is loaded. A motor-2-only plan executes directly
  as the single raw motor command selected by MPPI.
- Distal bending evidence no longer confirms engagement of the distinct v175
  interface-knob play. That play is confirmed from interface-pose response.
- Diagnostics publish the raw-response and response-free axis masks so the
  deployed semantics can be verified from a bag.

Focused regressions verify the raw/interface split, physical-limit treatment,
axis-specific arbitration, and v171 raw-history propagation. The ROS package
build also completes successfully.
