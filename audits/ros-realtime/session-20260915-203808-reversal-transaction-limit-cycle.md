# Sparse target reversal transaction audit — 20260915_203808

## Scope and evidence

This is a passive analysis of the completed simulation bag:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260915_203808_mppi_sim/20260915_203808_mppi_sim_0.db3
```

No ROS process, simulator state, controller parameter, or hardware state was
changed. Evidence came from controller status, trajectory-action feedback,
target, planned-control, response-trace, ground-truth-tip, upstream feedback,
and transmitted-state topics.

## Result

Point 1 reached tolerance. Point 2 did not: it timed out about 6.43 mm from
the fixed target. Point 3 then faulted with
`backlash_takeup_unconfirmed:axis_2`.

The point-2 failure is a reversal-transaction limit cycle, not a continuous
reference artifact. Rotation opened transaction generations 3 through 11.
After the initial negative-direction transaction, generations 4--11
alternated positive/negative on every fresh plan: eight sign reversals in
about 6.8 seconds. Each take-up occupied roughly 0.8--0.9 seconds, while most
post-take-up commands had zero or one 100 ms status interval before the next
reversal. Because the transaction barrier held the other requested shafts,
rotation fine-tuning starved the insertion/bending motion needed for the
remaining approximately `[-4, *, +5]` mm Cartesian error.

The rotation-sensitive target-error component crossed zero during each
full-rate take-up. Representative target-minus-tip values were:

```text
before positive take-up: [-4.30, +0.57, +5.77] mm
after positive take-up:  [-4.07, -0.79, +5.80] mm
after negative take-up:  [-4.34, +0.31, +5.22] mm
after positive take-up:  [-4.21, -0.47, +5.24] mm
```

Thus each correction overshot only the small rotation-sensitive component and
caused the next post-take-up solve to request the opposite direction, while
the dominant Cartesian error did not reverse.

## Source mechanism

MPPI correctly optimizes post-take-up motion. Its first-step reversal term is
only a soft cost. In this run, reversal plans commonly improved total sampled
cost over deterministic hold by about 9--14%, while the configured additive
reversal cost was negligible relative to total costs in the 330--430 range.
The planner does not charge the real 0.8--0.9 second transaction delay.

`TakeupTransactionArbiter.begin` immediately accepts the sign of every fresh
MPPI command. If that sign differs from measured engagement, it opens a new
transaction without persistence, improvement-margin, productive-motion, or
cooldown checks. The earlier plan document specified reversal arbitration,
but no corresponding implementation exists.

Point 3 reproduced the same structural issue on bending: generations 13--27
repeatedly changed shaft-2 direction. The F-042 correction correctly honored
the provisional rejection window, but repeated new reversals continually
reloaded take-up state. Generation 27 eventually exceeded the bounded
unconfirmed travel and failed closed. The fault is therefore a downstream
consequence of unscheduled reversal churn, not evidence that the F-042 change
was ineffective.

## Required correction: per-axis direction lease

Insert a response-clocked direction scheduler between MPPI candidate scoring
and `TakeupTransactionArbiter.begin`:

1. A first credible response grants that shaft a direction lease. Same-sign
   motion and zero remain admissible.
2. Opposite-sign candidates are not allowed to start a transaction
   immediately. MPPI must score a constrained alternative in which that
   physical shaft is held at zero, while the other shafts remain free.
3. A reversal becomes eligible only after all of these hold:
   - the reverse direction wins by a configured fractional and absolute cost
     margin over the best constrained alternative;
   - the same reverse intent persists for multiple fresh plans;
   - a minimum number of accepted camera observations or a minimum productive
     response has occurred since engagement;
   - a cooldown since the previous response/reversal has expired.
4. While intent is pending, execute the constrained plan rather than opening
   take-up or holding the complete coupled vector. This lets non-reversing
   shafts reduce the dominant error.
5. On acceptance, open exactly one bounded take-up transaction, invalidate the
   former plan, and grant the new lease only after credible response.
6. Reset pending intent on target identity change, disarm, fault, or a
   conflicting plan. Preserve the estimator's physical engagement state.

Candidate constraints must be applied before rollout so the modified action
is actually scored. Post-hoc removal of one motor component would execute an
unscored coupled command. Expose lease direction, pending direction/count,
allowed/reverse cost margins, productive-response age, and the reversal
decision reason in diagnostics.

## Verification

Add deterministic regressions for alternating `+/-` plans with a fixed target:
no transaction may open before the persistence and margin gates pass, zero is
always permitted, and other axes must remain plannable. Then repeat this exact
eight-point sparse simulation. Require all points to complete or time out
without a controller fault, no axis to alternate take-up direction every
transaction, and no weakening of the existing maximum-width, saturation,
freshness, or manager safety gates.

## Remediation status — 2026-09-15

Implemented the scheduler described above in
`catheter_control/reversal_scheduler.py` and integrated it before MPPI action
selection and take-up transaction admission. Unrestricted and constrained
candidates are evaluated in one batch; pending reversals execute the best
scored no-reversal alternative rather than a post-processed command. The
default admission gates are three persistent plans, absolute improvement 5.0,
fractional improvement 0.15, three accepted observations, and a 1.0 s
cooldown. Diagnostics expose leases, pending/approved directions, cost
improvements, candidate indices, and the decision reason.

Local verification passed: 26 focused scheduler/MPPI tests, all 216 source
package tests, `ament_flake8` on touched Python files, package build, and launch
argument inspection. The historical workspace result index still contains
unrelated `control_interface` lint failures and is not claimed clean. Repeat
the same sparse simulation before closing F-043.
