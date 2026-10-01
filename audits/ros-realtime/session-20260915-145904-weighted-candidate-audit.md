# Weighted-candidate simulation audit (2026-09-15)

## Scope

Passive analysis of `20260915_145904_mppi_sim`, the first deterministic run
after multi-frame response accumulation and proactive take-up coordination.
No hardware command was issued.

## Outcome

The response-estimation correction worked, but the path still diverged and
paused. The remaining failure is downstream of estimator initialization and
take-up confirmation: the MPPI weighted control can be a poor command even
when it was formed from individually feasible, scored candidates.

## Evidence

- The bag spans 15.979 s and contains 479 path-tracking samples.
- Progress reached 1.965 of 76.918 mm.
- Final/maximum reference error was 6.077/6.081 mm; final/maximum
  closest-path error was 6.046/6.050 mm.
- Governor counts were 47 `RUNNING`, 27 `SLOWED`, and 405 `PAUSED`.
- All 158 sampled controller states were `ACTIVE`; there was no controller
  fault in the captured interval.
- Rotation entered `TAKEUP` at 0.384 s, all axes were in `TAKEUP` at 0.484 s,
  bending engaged at 0.784 s, insertion at 0.984 s, and all three axes were
  engaged by 1.184 s.
- At 1.48, 1.68, 1.88, and 2.08 s, current/weighted-terminal reference errors
  were respectively 1.402/1.856, 2.057/2.650, 3.300/3.735, and
  4.163/4.600 mm. The command prediction was worsening the tracked objective
  before the controller fell into zero and reversal/take-up cycles.

## Mechanism

The planner projects and scores every sampled control sequence, but normally
executes their softmax-weighted mean. The weighted tip sequence is likewise an
average of candidate predictions, not a fresh rollout of the averaged command.
For a nonlinear learned model, neither average is guaranteed to be a scored
or improving trajectory. The run shows that this approximation was worse than
the current state over several successive plans.

Directional backlash makes the *physical shaft-command-to-response map*
hybrid and non-convex, but the current transmission-aware planner deliberately
does not put that map inside the learned rollout. It passes desired
post-engagement motor rates to v171. Backlash still influences candidate
weights through take-up-delay and first-step reversal penalties, constrains
rotation samples during a take-up latch, and changes the executed first action
through the coordinated compensator. It was therefore inaccurate to describe
the learned candidate response set itself as backlash-filtered or non-convex.

The old trace did not record the zero and best-candidate tracking costs, so it
cannot prove from bag data alone which sampled candidate should have been
executed. The source mechanism and the non-improving weighted prediction are
sufficient to add a guarded, instrumented test rather than another unobserved
heuristic.

## Correction

The opt-in `mppi_best_candidate_guard` retains the weighted MPPI update unless
both of these tests hold:

1. the weighted predicted trajectory has greater tip-tracking cost than the
   deterministic zero candidate; and
2. the already-scored minimum-total-cost candidate has lower tip-tracking cost
   than zero.

When both hold, the planner executes that feasible candidate and reports
`plan_best_candidate_selected=true`. It also records weighted, zero, and
selected-best tracking costs. This requires no additional learned-model
rollout and leaves limit projection and the take-up execution barrier intact.

## Post-correction result

The follow-up `20260915_150941_mppi_sim` reproduced the failure: 1.997 mm
progress, 6.030 mm maximum closest-path error, and 146 `PAUSED` updates out of
222. The guard selected no candidate in all 71 recorded active status samples.
At one source-matched response sample, the target-minus-start vector was
`[-1.981,-1.089,+1.634]` mm, while model and plant increments were respectively
`[+0.277,+0.030,-0.185]` and `[+0.320,+0.029,-0.240]` mm (direction cosine
0.998). The learned prediction was accurate, but the selected first action
moved away from the reference.

For the corresponding plan, weighted/zero/best tracking costs were
72.633/52.246/52.246. The implemented guard requires a nonzero best candidate
to be strictly better than zero, so it does nothing when zero itself is the
best action. A later recorded plan had 188.227/181.132/165.841 but still
reported the guard disabled, showing that the launch did not activate the
option in this run. The implemented remediation is therefore insufficient and
must not be treated as validated.

## Verification

- focused `catheter_control` suite: 193 passed;
- cross-package suite: 351 passed, excluding the environment-missing
  `ros2_igtl_bridge` test;
- `catheter_control` ROS build: passed;
- deterministic simulation: pending.

## Second remediation implemented

The planner/physical-transmission boundary is now explicit:

- MPPI ignores measured gap, phase, and direction state and no longer applies
  take-up delay or transmission-derived reversal penalties;
- guarded execution always chooses the minimum-total-cost sampled candidate,
  with deterministic zero included in the same batch, while the weighted mean
  is retained only as the next sampling nominal;
- a stateful downstream arbiter latches the physical active mask and requested
  shaft directions, commands only response-unconfirmed shafts at bounded
  take-up rates, and holds already-ready participating shafts at zero;
- engagement emits a zero barrier and invalidates the old plan before a fresh
  post-take-up rollout; and
- continuous-path progress reports `TRANSMISSION_HOLD` and does not advance
  during take-up or the replan boundary.

The profile uses the explicit `takeup_transaction_enabled` switch. Legacy
`mppi_transmission_aware_rollout` remains only as a launch compatibility alias
and is disabled in the profile. Focused catheter-control regressions pass
194/194, both ROS packages build, both launch files expose the new argument,
and a non-actuating simulation launch smoke test started every node. A full
closed-loop deterministic simulation remains required before hardware use.
