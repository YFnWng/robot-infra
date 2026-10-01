# Transmission-aware simulation stall audit (2026-09-15)

## Scope

Passive analysis of `20260915_121215_mppi_sim` after enabling candidate-local
backlash propagation, measured-response confirmation, and the rotation
direction latch. No live ROS or hardware commands were issued.

## Outcome

The run did not enter the literal `PAUSED` state. It entered `SLOWED` after
1.037 s, then asymptotically approached the 5 mm pause threshold. Governed
speed fell to `1.31e-10`, so the observed behavior was functionally paused.
Maximum path progress was 4.438%.

This is a deterministic planning deadlock introduced by putting the full
geometric take-up delay inside a 160 ms rollout. It is not a CUDA deadline,
ROS scheduling, marker-estimation, or simulated rotation-stick-slip failure.

## Evidence

- The 50.05 s action trace contains 31 `RUNNING` and 1,467 `SLOWED` samples,
  with no `PAUSED` sample.
- At the end, reference error was exactly 5.000 mm, closest-path error was
  1.735 mm, and progress was 4.438%.
- Planned rotation was zero for the entire run. The immediate stall therefore
  was not caused by the rotation latch.
- During the first 5 s, 68.1% of plans were nonzero and used insertion/bend.
  From 5 s onward, every published planned command was zero.
- Status-sampled plan time P50/P95/P99/max was
  48.819/58.901/62.952/92.393 ms. Isolated deadline-zero cycles occurred and
  the maximum consecutive count was three, below the simulation limit of
  five; ordinary valid plans still remained zero after 5 s.
- The final transmission state was axis 0 `TAKEUP`, with direction `-1` and
  6.242 rad remaining. Axis 2 was `ENGAGED`; axis 1 remained `UNKNOWN`.
- The full candidate rollout correctly marked transmission prediction active,
  but every selected four-step logical sequence was zero.
- The simulator received nonzero projected input during 40.9% of the first
  8 s, while only 19.4% produced downstream realized motion because its
  actuator dead zone consumed upstream travel internally.

## Mechanism

The controller horizon is four 40 ms steps. Candidate-local take-up uses the
bounded physical feedforward rate, but an active gap of 6.242 rad cannot be
cleared within 160 ms. All useful candidates consequently predict no interface
response inside the optimized horizon. Effort and slew terms then make the
zero candidate cheapest. Once zero is selected, no shaft motion is commanded,
the estimated gap never shrinks, and every subsequent solve sees the same
state.

This is the exact circular dependency that a backlash macro action must avoid:

```text
no predicted in-horizon response -> choose zero command
choose zero command              -> no take-up progress
no take-up progress              -> no predicted in-horizon response
```

The simulation also exposes a fidelity limitation. Its actuator perturbation
applies backlash before updating the single encoder/model angle, so simulated
ENC reports downstream transmitted motion. Hardware ENC is the upstream motor
shaft signal used to measure take-up. That mismatch delays the simulated
online estimator's recognition of gap consumption and makes this run more
conservative than hardware, although it is not the reason MPPI selected zero.

## Secondary governor issue

The governor tests the pause threshold before advancing phase, then recomputes
the post-advance error. Its proportional slowdown can approach 5 mm strictly
from below forever, producing `SLOWED` with a vanishing scale rather than
literal `PAUSED`. This affects state reporting and completion semantics, but
changing it would not restore control motion.

## Required correction

Keep MPPI's decision variable as desired **post-engagement** input and let the
learned distal rollout evaluate that response directly. Represent take-up as
a bounded executable macro action outside the expensive short horizon:

1. Charge each candidate a direction-specific take-up time/travel cost.
2. If the selected first input reverses or its axis is already in `TAKEUP`,
   commit the compensated direction until measured interface response confirms
   engagement or the calibrated maximum travel faults closed.
3. Do not resample a zero/reversing first action while that macro action is
   active; only its magnitude may be safety-projected.
4. After confirmation, resume ordinary post-engagement MPPI every cycle.
5. Split simulated upstream encoder angle from transmitted interface/model
   angle before using simulation to qualify estimator timing.

The rotation-specific latch remains useful, but the macro-action semantics
must cover any axis with a long gap; this run deadlocked on axis 0 before
rotation was ever requested.
