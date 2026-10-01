# Session 20260915_192609 MPPI simulation: reversal-intent audit

## Scope

Passive analysis of:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260915_192609_mppi_sim/20260915_192609_mppi_sim_0.db3
```

No ROS process, controller parameter, simulator, or hardware state was
changed.

## Result

F-040 improved progress and reduced insertion reversal frequency, but a soft
single-plan cost does not provide the temporal commitment required before an
expensive transmission transaction.

Compared with `20260915_191324_mppi_sim`:

| Metric | Previous | Current |
| --- | ---: | ---: |
| Maximum progress | 5.739 mm | 10.565 mm |
| Path completion | 7.46% | 13.74% |
| Physical insertion reversals | 56 | 33 |
| Rotation reversals | 6 | 4 |
| Transmission-hold samples | 80.4% | 75.0% |
| Maximum closest-path error | 0.455 mm | 0.608 mm |

Thus planner-memory preservation was beneficial, but it did not meet the
first-10-mm reversal or hold-occupancy gates.

## Terminal fault

The controller faulted with:

```text
backlash_takeup_unconfirmed:axis_2
```

Feedback and scheduling were healthy at the fault: estimator health was
`TRACKING`, marker updates were accepted, feedback ages were bounded, there
were no consecutive deadline misses, and the last plan completed in 28.7 ms.
The manager remained ready after the controller disarmed.

Bend was absent for the first 70 plans. It then appeared in only 12 of 123
plans, but its physical shaft direction reversed nine times. Five consecutive
bend sign changes occurred in plans 73--77. Later reversals occurred at plans
92, 94, 121, and 122. The final two plans requested opposite bend directions
approximately 0.33 s apart. The estimator had repeatedly produced bend
response evidence near 1.0 and provisional engagement before those reversals;
the fault is therefore the end result of repeated incomplete/reversed
transactions, not a general loss of visual feedback.

## Mechanism

The active controller executes the single minimum-cost sampled candidate.
With 1,024 samples, a weakly useful axis can appear at its hardware minimum
nonzero speed for one solve and at the opposite minimum in the next. For the
configured catheter, bend commands below 2 mm/s are quantized either to zero
or to the 2 mm/s reliable floor. A one-cycle sign choice is therefore not a
small perturbation.

The first-step physical reversal cost is evaluated independently in each
solve. It reduced chatter, but it neither requires a direction to persist
across fresh observations nor preserves the last nonzero physical direction
through zero plans. Thirty-three physical insertion reversals remained; 29
were direct plan-to-plan sign changes and four followed an immediately zero
plan. Every accepted reversal can open a response-terminated take-up macro,
so a sampled sign is being promoted into a stateful transmission action too
early.

The path action also recognizes the old `replanning_after_takeup` reason but
not the newer `replanning_after_takeup_response` and
`replanning_after_takeup_saturation` reasons. Reference progress can therefore
advance during the nominal zero/replan boundary. This is a secondary hold
accounting defect, not the source of the alternating sampled commands.

Confidence classifications:

- progress, path error, fault reason, and reversal counts: **observed**;
- healthy feedback/scheduling at the terminal fault: **observed**;
- minimum-speed quantization and per-solve candidate selection:
  **observed in source and trace**;
- premature promotion of one sampled sign to a transaction as the chatter
  mechanism: **inferred-high-confidence**.

## Required correction

1. Add a physical-shaft reversal-intent arbiter between MPPI selection and the
   take-up transaction. A new or opposite shaft direction must remain
   consistent for at least two fresh plans and beat deterministic hold by a
   configured cost margin before it can open a transaction.
2. While intent is pending, command zero and freeze path reference progress.
   Do not execute the previous direction and do not consume a take-up budget.
3. Preserve a last-nonzero physical direction independently of the prior
   command magnitude so an intervening zero plan cannot erase transmission
   direction memory.
4. Scope `backlash_takeup_unconfirmed` failure to the currently active
   transaction and its requested direction. A stale provisional direction
   must not fault after arbitration has withheld or changed that intent.
5. Treat both new post-take-up replan reason strings as transmission holds in
   the path action.

This correction should be verified with the same seed, path, plant, and 1 mm/s
reference. The first gate remains fewer than five insertion reversals in the
first 10 mm, transmission hold below 30%, monotonic progress beyond 10 mm, and
maximum closest-path error below 2 mm. Also require zero bend transactions
that fail without first passing the reversal-intent gate.
