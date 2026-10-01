# Session 20260929-190553 plain-MPPI circle timeout audit

## Scope

Read-only audit of
`mppi_circle_plain_current/20260929_190553_mppi_sim`, the first complete
continuous-path simulation using 512 samples, a four-step 0.8 s horizon, and
the exact coarse backend rollout route.

## Outcome

The action ran for 89.94 s and timed out after advancing 22.411 mm of the
80.811 mm path (27.73%). The simulated plant moved primarily in rotation and
then became effectively stationary. The path governor continued advancing the
reference in `RECOVERY_ADVANCE`, so reference error grew after plant motion
ceased.

## F-001: Plain weighted MPPI discarded useful sampled controls

Severity: high

Confidence: observed

Across 836 comparable valid plans:

- the best sampled tracking cost beat zero control in 836/836 plans;
- the weighted command beat zero in only 214/836 plans;
- the weighted command was worse than zero in 622/836 plans;
- median tracking costs were 201.334 (best), 311.937 (weighted), and
  296.581 (zero);
- median weighted-minus-best cost was 142.388;
- median effective sample count was 42.23 of 512; and
- median blocked-candidate count was 461 of 512.

The executed command was the `weighted_feasible_candidate_mean`; scored-best
candidate selection and grouped sampling were both disabled. Command
projection then mapped the small averaged insertion and tendon components to
zero. Of 1,344 planned commands during the action, insertion was always zero,
tendon was zero in 99.85%, and rotation was zero in 93.45%.

This is the primary cause of the timeout. The optimizer found useful actions,
but plain MPPI did not execute them.

## F-002: The plant stopped while the reference continued

Severity: medium

Confidence: observed

Only 88 of 1,344 planned commands were nonzero. The last nonzero command was at
71.786 s; all meaningful commands were rotation except a brief tendon command
near startup. Rotation position accumulated 61.77 degrees of one-direction
motion. The closest-path error eventually plateaued at 4.266 mm.

The governor spent 2,074/2,691 samples in `RECOVERY_ADVANCE`, 558 in `SLOWED`,
and only 59 in `RUNNING`. Therefore, this was not a governor pause. The
reference deliberately continued at reduced speed while the zero-command
controller did not reduce cross-track error, growing final reference error to
more than 8 mm.

## F-003: Coarse rollout removed the former compute explosion

Severity: informational

Confidence: observed

The exact four-step path rollout was active (`mppi_path_rollout_coarse_steps =
true`, horizon 0.8 s). Planner timing was:

- elapsed: P50 51.14 ms, P95 62.43 ms, maximum 123.85 ms;
- rollout: P50 36.80 ms, P95 46.59 ms;
- valid plans: 1,221/1,346 timing records;
- deadline misses: 125/1,346 timing records.

This is materially lower than the prior approximately 191 ms internally
subdivided rollout. Deadline misses caused intermittent skipped plans, but do
not explain the persistent zeros: the weighted-mean failure is present in 836
valid, fully scored plans.

## Conclusion

The modern long-horizon plain-MPPI circle trial remains a valid negative
baseline. Longer horizon and 512 samples do not fix convex averaging across
incompatible control modes. The next controlled comparison should keep the
same plant, reference, 512 samples, four 0.2 s coarse steps, and governor, and
change only selection from plain weighted mean to grouped/scored selection.
