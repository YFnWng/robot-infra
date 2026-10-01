# Hardware two-axis sparse audit: 20260928_150940_mppi_demo

## Outcome

Points 1 and 4 reached tolerance. Points 2 and 3 timed out because MPPI issued
no tendon command at all. Both failed actions briefly moved insertion and then
selected deterministic zero for nearly the entire 15 s action. The failure is
not attributable to marker loss, estimator degradation, take-up risk cost, or
reversal blocking.

The strongest explanation is a horizon mismatch. These Cartesian targets were
generated from 25 model steps at 40 ms (1.0 s), whereas the deployed MPPI used
4 steps at 40 ms (0.16 s). For points 2 and 3, the useful tendon endpoint is a
longer-transient result; over 0.16 s, the scored tracking objective preferred
hold. Gain-belief learning also halved tendon proposal velocity while the
direction remained `LEARNING`, further shortening the effective look-ahead.

## Evidence

Session bag:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260928_150940_mppi_demo/20260928_150940_mppi_demo_0.db3`

The bag is 78.49 s and contains 42,111 messages, including 2,258 marker
observations, 1,240 estimator traces, 470 plans, and 306 causal response
comparisons.

| Point | Result | Minimum / final tip error | Planned nonzero insertion | Planned nonzero tendon |
|---|---|---:|---:|---:|
| 1 | reached | 0.465 / 0.530 mm | 10/11 | 0/11 |
| 2 | timeout | 4.729 / 5.502 mm | 10/223 | **0/223** |
| 3 | timeout | 5.145 / 5.515 mm | 21/199 | **0/199** |
| 4 | reached | 0.431 / 0.431 mm | 36/37 | 35/37 |

### Target-generation contract

Recorded model-preview results were:

- point 1: realized logical delta `[4.9929, 0, 0]`;
- point 2: `[0.0231, 0, 2.9964]`;
- point 3: `[6.9795, 0, 2.9964]`;
- point 4: `[-4.0161, 0, 2.9964]`.

Thus points 2 and 3 were explicitly tendon-dependent according to the same
runtime model used to create them. Point 4 demonstrates that a tendon-dependent
target and the take-up/command path can succeed in this run.

Recorded parameters establish the horizon mismatch:

- target preview: `rollout_steps=25`, `dt=0.04 s`, total 1.0 s;
- MPPI: `horizon_steps=4`, `rollout_step_s=0.04`, total 0.16 s;
- uncertain tendon gain proposals were additionally scaled by
  `mppi_engaged_gain_learning_velocity_scale=0.5`.

### Point 2

MPPI initially retracted insertion for about 0.7 s. At 0.95 s it selected
zero, with `plan_selected_total_cost == plan_zero_total_cost == 367.127`.
Thereafter the two costs remained equal while the tip error stayed near
5.2--5.4 mm. The tendon direction was already `ENGAGED`, take-up risk was zero,
the selected reversal mask was zero, and `planner_blocked_motor_direction` was
`[0,0,0]`. Therefore zero was selected by the nominal tracking objective; it
was not imposed by the compensator or scheduler.

### Point 3

Point 3 performed an insertion-direction take-up/replan and several brief
insertion commands, then also settled on zero. It never issued tendon motion.
The controller reported no direction block and only two isolated deadline-miss
zero cycles. Those misses cannot explain the roughly 12 s hold interval.

### Tracking and estimator health

For both failed targets:

- all 453 recorded marker diagnostics were `TRACKING`;
- all estimator traces were `TRACKING` (187 for point 2, 194 for point 3);
- accepted/no-op visual corrections continued throughout;
- no `marker_feedback_degraded`, controller fault, or stale-feedback gate
  occurred.

The cold-start multi-rig correction therefore resolved the prior vision
failure for this run.

## Findings

### F-20260928-01: Planning horizon is inconsistent with target construction

Severity: high
Confidence: inferred-high-confidence

The same model declares points 2 and 3 reachable after a 1.0 s tendon rollout,
but the controller scores only the next 0.16 s. The zero candidate wins once
the immediate insertion improvement is exhausted. Point 4 succeeds because
its large negative-z error makes tendon motion beneficial within the short
horizon.

### F-20260928-02: Gain-learning velocity scaling amplifies horizon myopia

Severity: medium
Confidence: inferred

The positive tendon gain remained `LEARNING`; its mean varied substantially
(roughly 0.3--0.8) and proposals were scaled to 50%. Robust scenarios are
appropriate, but within a four-step horizon this leaves only about 0.08 s of
nominal tendon progress. This is secondary to, not a substitute explanation
for, the horizon mismatch.

### F-20260928-03: Tendon gain belief updates without executed tendon motion

Severity: medium
Confidence: observed

During points 2 and 3 the tendon gain repeatedly reported `updated` despite
zero executed tendon commands. Source inspection confirms that
`observe_response()` invokes the gain observer whenever the tendon phase is
`ENGAGED`; `EngagedGainEstimator.observe()` requires a nominal lambda
increment but does not require a contemporaneous tendon motor increment.
Consequently, delayed distal relaxation or insertion-correlated model
evolution can tighten the directional tendon posterior. That is not a clean
causal measurement of engaged tendon gain and can contaminate the belief used
by MPPI.

## Recommended next correction

Do not weaken take-up or reversal safety gates. First make the point-target
planner capable of valuing delayed but useful tendon response:

1. add a longer, coarsened terminal tail for point targets (for example,
   retain four 40 ms control moves, then propagate the terminal command/state
   through a low-cost model-only tail approaching the 1.0 s preview horizon);
2. score that tail in every engaged-gain scenario, preserving the robust risk
   aggregation and joint-limit projection;
3. keep only the first 40 ms command executable, so command frequency and
   take-up arbitration remain unchanged;
4. add a deterministic positive-tendon probe to every applicable U/C group so
   the delayed-response branch cannot disappear through sampling;
5. gate engaged-gain updates on causal tendon excitation or explicitly modeled
   distal relaxation.

This should be replay-tested on this bag before another hardware run. The
acceptance criterion is that points 2 and 3 select a nonzero positive tendon
plan from their home states without changing the successful decisions for
points 1 and 4.
