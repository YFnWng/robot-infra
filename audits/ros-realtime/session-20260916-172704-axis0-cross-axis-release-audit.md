# Hardware path audit: 20260916_172704_mppi_demo

## Outcome

The terminal `backlash_takeup_unconfirmed:axis_0` was a false engagement
failure. The insertion transaction produced a large, causally observable
catheter response, but the response was not sufficiently aligned with the
nominal axis-0 interface Jacobian column. The current observer uses that model
alignment both to validate the model and to decide whether mechanical take-up
has engaged. Those are different questions.

The user's torsional-release explanation is consistent with the record and is
classified as inferred-high-confidence. What is directly observed is that
dominant insertion-shaft motion changed interface orientation substantially
while the rotation encoder moved very little.

## Recording boundary

The original SQLite bag was still being written after the controller fault.
Analysis used a SQLite online-backup snapshot taken into `/tmp`; the recorder
and hardware stack were not stopped or modified. The snapshot includes the
fault and subsequent stationary diagnostics.

## Terminal sequence

- Take-up generation 43 commanded negative rotation at the reduced hardware
  rate. Across the sampled transaction, encoder-count change was approximately
  `[+3858, -10640, +3702]`; the interface body rotation included about
  `+0.0491 rad` around local z.
- Generation 44 was a short tendon transaction.
- Generation 45 then requested negative insertion only from the arbiter. Its
  sampled encoder-count change was `[-12720, -511, -62]`, so axis 0 accounted
  for about 95.7% of absolute encoder-count travel.
- During generation 45, the interface body rotation was approximately
  `[+0.0120, -0.0669, -0.0566] rad`. The local-z change was about -3.24
  degrees, opposite and comparable in magnitude to the preceding rotation
  transaction's +2.81 degrees despite only 511 counts of rotation-shaft
  motion.
- The measured tip moved `[+2.345, -6.872, -0.857] mm`, or 7.312 mm in norm.
  Closest-path error improved from 6.230 to 1.420 mm while path progress was
  intentionally held at 17.500 mm.
- Several estimator frames assigned axis-0 inferred increments with response
  evidence near 1.0, but the response remained `INCONCLUSIVE` because its
  direction did not satisfy the Jacobian-column cosine condition.
- Axis-0 raw travel reached approximately 10.65 rad, exceeding the unchanged
  fail-closed maximum of `1.5 * 6.856 = 10.284 rad`. Since no
  model-direction-confirmed response had occurred, the observer set axis 0 to
  `FAILED`, producing the reported controller fault.

The controller completed 45 take-up generations in roughly 63 seconds and
reached 17.500/72.913 mm (24.0%) path progress before this fault. Lowering the
rotation take-up rate was active (`19.8` after command quantization), but it
does not address this separate cross-axis response classification problem.

## Source mechanism

`BacklashStateEstimator.observe_response()` requires inferred transmitted
motion, response evidence, and directional cosine against the corresponding
local Jacobian column before declaring interface response confirmed. The
pre-confirmation maximum-travel guard then faults if this condition is never
met. This is appropriate for accepting a sample for Jacobian or width
learning, but too strict for the binary question of whether a visibly moving
mechanism has exhausted take-up.

The UKF already corrects the current pose after the cross-axis response. What
is missing is separation between:

1. causal mechanical engagement;
2. nominal-model-consistent response suitable for parameter learning; and
3. a history-dependent cross-axis release residual, such as torsional windup
   released by insertion.

## Recommended correction

1. Add a `CAUSAL_RESPONSE` engagement path for an isolated pending shaft.
   Require dominant raw motion on that shaft plus accepted marker-derived
   interface/tip response clearly above measured noise. Do not require the
   response to align with its nominal Jacobian column.
2. On `CAUSAL_RESPONSE`, stop the fixed-rate take-up immediately, mark the
   requested direction at least provisional, issue the existing zero/replan
   barrier, and plan again from the UKF-corrected state.
3. Keep the stricter directional/Jacobian test for backlash-width learning and
   online Jacobian adaptation. An off-model causal response must not train the
   nominal local Jacobian or directional width.
4. Record an explicit off-model response residual and its dominant axis. This
   provides the data needed to decide whether a small history-dependent
   torsional-release state is necessary. Do not fold this transient into a
   fixed global Jacobian.
5. Preserve the maximum-travel fault for truly response-free transactions and
   retain manager, projection, freshness, and firmware safety gates.

## Torsion-management design directions

The response-classification correction above prevents a false fault, but it
does not make the history-dependent torsional release predictable. Two
controller-level directions should be evaluated separately.

### A. Prevent windup accumulation

- Treat stored rotation travel without proportional UKF material-roll motion
  as an estimated windup budget, distinct from backlash travel and material
  roll.
- Terminate rotation take-up on the first credible causal response rather than
  continuing until the response agrees with the nominal Jacobian.
- Penalize additional same-direction rotation as the estimated windup budget
  grows, and prefer plans that achieve the target without accumulating more
  stored twist.
- Order coupled actions so planned insertion/retraction occurs before final
  precision rotation where feasible. Rotation followed immediately by
  insertion is the sequence most exposed to release of newly stored twist.
- Do not train the nominal Jacobian with windup accumulation or release
  transients.

This direction preserves continuous operation but requires an observable
windup state or at least a conservative bound. Encoder angle alone is not that
state because the material roll can lag it.

### B. Unwind before insertion

- Before admitting a significant insertion-direction change, check whether
  encoder rotation and UKF material roll imply stored twist above a threshold.
- If so, hold path progress and run an explicit, bounded unwind transaction.
  The unwind must be response-terminated, manager-projected, and followed by a
  zero/replan barrier.
- With no torque sensor, neutral torsion cannot be inferred from encoder home
  alone. An unwind completion criterion must use material-roll response and a
  calibrated hysteresis history; simply commanding rotation encoder zero is
  not a valid reset.
- If neutral state cannot be established within a bounded travel/time budget,
  fail closed rather than allowing insertion to release an unknown amount of
  twist.

This direction is slower and can disturb tip position before insertion, but it
creates a clearer causal boundary. It should first be tested in isolated
rotation-then-insertion experiments, not introduced during a path run.

The two approaches can eventually be combined: prevention during normal MPPI
operation, with explicit unwind only when the estimated windup budget exceeds
a reviewed threshold. Neither changes the physical encoder-zero reference.

## Classification

- False engagement failure and code mechanism: observed/source-confirmed.
- Insertion-induced off-model interface rotation: observed.
- Torsional windup release as the physical cause: inferred-high-confidence.
- Exact predictive windup model and its repeatability: unknown; requires
  repeated isolated rotation-then-insertion experiments or causal replay.
