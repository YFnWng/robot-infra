# Axis-2 hold sign regression — 2026-09-16 10:05

## Scope

Passive audit of sparse simulation session `20260916_100502_mppi_sim` after
the fixed-size F-044 candidate-bank correction. No hardware was commanded.

## Result

Sparse points 1 and 2 completed. Point 3 faulted on
`backlash_takeup_unconfirmed:axis_2`. Planner timing was healthy: 190 reported
plans were 37.909 ms P50, 46.301 ms P95, 51.180 ms P99, and 53.904 ms maximum,
with no plan over the 60 ms deadline.

The point-3 failure was transaction churn. Physical axis 2 alternated take-up
direction across generations 11--24, commonly changing sign every 0.2--0.4 s.
At 1789567542.364, for example, diagnostics reported lease `-1`, proposed
reversal `+1`, no approval, and `plan_hold_branch_applied=True`, yet generation
16 began a physical `+1` axis-2 take-up transaction. The final credible
negative response at 1789567544.965 was followed by another sign change; the
bounded unconfirmed-travel guard then faulted closed.

## Mechanism

The fixed-size candidate bank converted logical commands into firmware
motor-axis units and compared their signs directly with scheduler leases. The
leases and transaction directions are physical shaft radians/s. Axis 2 uses a
negative motor-axis-units-per-RPM constant, so its firmware-unit sign is the
opposite of physical shaft direction. The bank therefore failed to neutralize
the exact axis-2 samples that the scheduler classified as reversals.

## Correction

Candidate construction now divides motor-axis units by the signed firmware
conversion constant before reversal classification. Hold and per-axis
ablation variants remain represented in motor-axis units for reconstruction,
but their direction semantics are physical shaft radians/s. The execution
guard additionally requires the projected physical first rate to be exactly
zero on every unapproved reversing axis; otherwise deterministic zero is used.
Per-axis scheduler evidence is accepted only from an actually neutral
projected ablation.

An axis-2 regression exercises the negative conversion explicitly and verifies
that an unapproved positive physical reversal under a negative lease yields a
zero projected physical axis-2 command. All 218 package tests, touched-file
lint, and the package build pass.

## Status

Source remediation is complete. Repeat the same sparse GPU simulation. F-044
remains open until the run completes without reversal churn, take-up fault, or
deadline regression.
