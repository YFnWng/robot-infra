# Sparse-target direction-lease regression — 2026-09-15 22:37

## Scope

Passive audit of simulation session `20260915_223716_mppi_sim` after the
F-043 direction-lease scheduler was enabled. The run used eight independent
sparse targets with a return to the same joint home between actions. No live
system was commanded during this audit.

## Outcome

The session finished without a controller fault, but only point 1 entered the
1.8 mm tolerance. Points 2--8 timed out. Their minimum/final action errors were:

| Point | Minimum error (mm) | Final error (mm) |
| ---: | ---: | ---: |
| 1 | 0.633 | 1.463 |
| 2 | 8.841 | 35.877 |
| 3 | 25.217 | 36.104 |
| 4 | 24.829 | 33.051 |
| 5 | 32.906 | 45.943 |
| 6 | 12.735 | 22.203 |
| 7 | 23.776 | 37.202 |
| 8 | 10.926 | 23.256 |

This is a controller regression, not merely unreachable geometry: for points
3--8, 94--98% of active plans reported `plan_direction_lease_applied=True`.
Point 3, for example, retained lease `[+1,+1,+1]`, repeatedly proposed a
negative shaft-1 reversal, but executed positive shaft-1 commands. The target
error grew from approximately `[+0.59,-22.90,+21.58]` mm to
`[+13.65,-32.32,+7.95]` mm during representative status samples.

## Source-confirmed mechanisms

### 1. The constrained candidate does not hold a reversing shaft at zero

`mppi.py:543-568` defines a constrained candidate as any candidate whose first
motor command is not opposite the lease. This admits both zero **and continued
motion in the old leased direction**. The test at `test_mppi.py:183-222`
explicitly expects the constrained command to remain positive when the
unrestricted optimum is negative.

That is not the F-043 contract. Pending reversal was supposed to hold the
affected physical shaft at zero while allowing other shafts to progress. The
implemented admissible set instead lets the optimizer move farther in the
known-wrong direction because that candidate may have lower finite-horizon
cost than other sampled, sign-compatible candidates.

The failure is visible even inside the model prediction: the selected
candidate's predicted terminal error exceeded the current error in 100% of
usable point-3 plans, 99% of point-4 plans, 78% of point-6 plans, and 91% of
point-8 plans.

### 2. Fractional reversal admission is normalized by irreducible total cost

`reversal_scheduler.py:138-145` computes

```text
(constrained_total_cost - unrestricted_total_cost) / constrained_total_cost
```

and requires 0.15. The total includes the large residual tracking cost across
the complete short horizon. In this run, useful unrestricted reversals commonly
improved cost by hundreds of absolute units but only 0.02--0.07 of total cost.
For point 3, representative improvements were 192--484 cost units but only
2.6--4.0%. The pending count reached 145 while the reason remained
`reversal_cost_margin`; approval was therefore impossible despite stable
intent and ample observations.

### 3. The comparison is global, not per-axis causal evidence

One unrestricted candidate can propose several reversing axes. The scheduler
uses one global cost difference to approve every proposed axis. It cannot say
which physical reversal caused the improvement. Point 2 consequently still
started ten approved reversal transactions and alternated the effective axis-0
sign 18 times, while later points were indefinitely locked. The scheduler has
therefore produced both failure modes it was meant to exclude: intermittent
reversal churn and permanent wrong-direction locking.

## Corrective design

1. Score an explicit physical-shaft hold branch. For each proposed reversal,
   set the affected physical motor rate to exactly zero, reconstruct the
   coupled logical command, and roll it out before selection. Do not admit
   continued old-direction motion as the reversal alternative.
2. Compare reversal benefit using per-axis ablations: score the unrestricted
   candidate with one proposed reversing shaft neutralized at a time. This
   gives a marginal, causally attributable benefit for each reversal.
3. Base admission on predicted geometric descent above the estimator/noise
   floor plus persistence and cooldown. Do not normalize a short-horizon
   marginal action benefit by the entire irreducible tracking cost.
4. While approval is pending, execute the fully scored command with all
   unapproved reversing shafts neutralized; compare it with deterministic zero
   and never execute a branch predicted to increase terminal tracking error.
5. Retain all bounded take-up, saturation, freshness, manager, and model-valid
   gates. Keep approval single-use and replan after engagement.

## Status

F-043 is not remediated. The scheduler correctly prevented the former terminal
fault, but failed its functional acceptance test. No source correction was made
as part of this passive audit.

## Remediation status — 2026-09-16

Implemented the F-044 correction. The planner now starts from the unrestricted
optimum, neutralizes proposed unapproved reversals in physical motor
coordinates, reconstructs the coupled logical action, runs hardware projection,
and scores the resulting learned-model rollout. It scores both the combined
hold and one single-axis ablation per proposed reversal. The scheduler uses the
per-axis ablation benefit rather than assigning a global cost difference to
every axis.

Pending execution selects the combined hold only if its horizon tracking cost
and terminal error are no worse than deterministic zero; otherwise it commands
zero. Admission requires at least 5.0 tracking-cost units and 0.25 mm predicted
terminal-error benefit for that axis, plus three persistent plans, three
accepted observations, and the one-second cooldown. The old fractional total
cost threshold defaults to zero and no longer participates in admission.

Local verification passes 27 focused tests, all 217 `catheter_control` tests,
lint for touched files, package build, and launch-argument inspection. Repeat
the eight-point sparse GPU simulation to measure added rollout time and close
or refine F-044.
