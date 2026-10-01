# Plain MPPI hardware baselines versus grouped MPPI — 2026-09-29

## Scope

This audit compares three rotation-disabled hardware sessions executing the
same four absolute Cartesian targets:

- grouped MPPI plus response-terminated take-up compensation:
  `20260929_104353_mppi_demo`;
- plain MPPI plus the same take-up/engaged-gain layer:
  `20260929_114133_mppi_demo`;
- plain MPPI with no compensation:
  `20260929_114545_mppi_demo`.

Evidence was read passively from the rosbags. No live control or hardware
operation was performed.

## Runtime identity and controlled differences

All three sessions report the same frozen model artifacts:

- distal SHA-256 `adfbe11f58d409c9a24fdbdef23ef38b32c83fcc692ef77fa68c52508254ac1a`;
- Jacobian SHA-256 `6d3f8573d10511dcbea3c27c97141f132b95cc42c450d3aec3702db33430ad2c`;
- 48 samples, four 0.2 s rollout steps, zero rotation velocity, fixed v174
  Jacobian (`adaptation_enabled=false`).

| Feature | Grouped | Plain + compensation | Plain only |
| --- | ---: | ---: | ---: |
| v175 transmission/take-up | yes | yes | no |
| engaged-gain scenarios | yes | yes | no |
| grouped U/C proposals | yes | no | no |
| scored-candidate execution guard | yes | no | no |
| take-up risk weight | 4 | 0 | 0 |
| reversal scheduler | yes | no | no |
| executed prediction | scored feasible candidate | weighted feasible mean | weighted feasible mean |

The comparison is not a perfectly reset physical crossover. Encoder home was
the same, but the target-1 starting error was 8.11 mm in the grouped run versus
14.36 and 15.50 mm in the later baselines. Points 2--4 began much more
comparably (within roughly 1.5 mm in error norm), and are the stronger A/B
evidence.

## Outcomes

| Target | Grouped result (time, final/min error) | Plain + compensation | Plain only |
| --- | --- | --- | --- |
| 1 | reached, 5.42 s, 1.085/1.085 mm | reached, 6.87 s, 1.687/1.684 mm | timed out, 30.11 s, 1.768/1.708 mm |
| 2 | reached, 6.76 s, 1.017/1.017 mm | reached, 26.43 s, 1.568/1.513 mm | timed out, 30.11 s, 2.093/2.037 mm |
| 3 | reached, 12.31 s, 0.941/0.342 mm | timed out, 30.08 s, 2.180/1.912 mm | timed out, 30.09 s, 2.784/2.353 mm |
| 4 | reached, 8.38 s, 0.440/0.344 mm | timed out, 30.04 s, 3.022/2.877 mm | timed out, 30.10 s, 2.513/2.375 mm |

The settle gate required about six consecutive 20 Hz feedback samples inside
1.8 mm. Plain-only target 1 accumulated 34 in-tolerance samples but its longest
continuous streak was only four, so the apparent 1.768 mm terminal value was
not a stable reach.

## F-001: Plain weighted execution collapses into the motor deadband

Severity: high
Confidence: observed

The plain-only controller remained armed and reported `reason=active`; this was
not a lifecycle pause. Nevertheless, the planned two-axis command was exactly
zero for 84.8%, 80.2%, 87.8%, and 82.9% of evaluated target cycles.

This is caused by the implemented plain-MPPI execution rule. The best sampled
candidate is evaluated and reported, but with the scored-candidate guard off
the controller executes the convex weighted mean of feasible candidates. It
then independently zeros each component below the physical minimum speed. The
feasible actuator set is nonconvex around zero, so averaging useful candidates
from different modes can synthesize a small command that was never itself a
useful sampled plan and is subsequently deadbanded.

The bag directly shows that useful candidates existed:

| Target | Median(best cost − zero cost) | Median(executed weighted cost − best cost) | Weighted terminal prediction worse than zero |
| --- | ---: | ---: | ---: |
| 1 | −21.8 | +22.1 | 0.3% |
| 2 | −55.2 | +61.3 | 12.4% |
| 3 | −29.3 | +54.8 | 62.2% |
| 4 | −52.7 | +65.5 | 39.5% |

Thus the optimizer found a lower-cost nonzero candidate while the execution
map discarded much of that benefit. For targets 3 and 4, the weighted command's
predicted terminal point was frequently worse than simply holding.

The uncompensated response also exposed the expected transmission mismatch:
median predicted response magnitudes were 0.32--0.61 mm while measured
magnitudes were only 0.15--0.18 mm, and median predicted/measured direction
cosines were only 0.23--0.36. Raw shaft travel was therefore a poor proxy for
immediate effective motion.

## F-002: Compensation is necessary but plain selection repeatedly re-enters take-up

Severity: high
Confidence: observed

Adding the response-terminated take-up layer materially improved performance:
plain + compensation reached targets 1 and 2 and produced response-direction
cosines of 0.64--0.81 rather than 0.23--0.36. It was nevertheless much slower
than grouped MPPI and timed out on targets 3 and 4.

For targets 2--4, plain + compensation opened 9, 11, and 10 distinct take-up
generations. `TAKEUP_ACTIVE` alone occupied approximately 15.3, 15.8, and
20.5 seconds; including confirmation/replan states, transmission arbitration
consumed about 17.3, 18.2, and 22.1 seconds of each 30-second target. Active
masks alternated between insertion-only `[1,0,0]` and coupled insertion+tendon
`[1,0,1]`. Executable MPPI insertion commands reversed 4, 6, and 3 times.

The plain compensated controller still executed a weighted mean rather than an
independently scored plan. Its median executed-cost penalty relative to the
best available sampled candidate was +329, +158, and +123 cost units on targets
2--4. With no grouped modes, scored-candidate guard, reversal scheduler, or
take-up risk cost, those direction changes repeatedly handed control back to
the slow take-up arbiter.

The terminal failures are slow-convergence failures, not evidence that the
points were unreachable. Target 3 reached 1.912 mm only 0.31 s before timeout;
target 4 reached its 2.877 mm minimum around 28.0 s. The same absolute targets
were reached by grouped MPPI.

## F-003: Grouped/scored execution preserves complete plans

Severity: informational
Confidence: observed for behavior; inferred-high-confidence for causal ranking

Grouped MPPI executed an already-scored feasible candidate, so its
`selected_total_cost - best_cost` was identically zero in the recorded plans.
Its command was zero in only 7.6--11.8% of target cycles. For targets 2--4 it
usually evaluated four U/C proposal groups and retained explicit continuation
alternatives instead of averaging incompatible direction modes.

The grouped run used only 3, 5, and 3 take-up generations for targets 2--4 and
finished them in 6.76, 12.31, and 8.38 seconds. The reversal observation
holdoff was active where needed, and the take-up risk term charged physical
delay before selecting a reversal.

This three-way experiment changes grouped sampling, scored-candidate execution,
take-up risk, and reversal scheduling together. The bags therefore establish
the combined mechanism, but do not identify the individual contribution of
each grouped feature. The strongest directly observed distinction is
scored-plan execution versus an unscored convex mean followed by a nonconvex
minimum-speed deadband.

## F-004: Estimator and deadline faults were not the failure mechanism

Severity: informational
Confidence: observed

All sessions completed without controller fault. The maximum consecutive
planner deadline-miss count was one in every run. The compensated and grouped
runs had zero consecutive marker rejections; plain-only had one transient
rejection and otherwise remained `TRACKING`. Deadline misses commanded zero for
one cycle but never reached the repeated-miss fault threshold.

## Conclusion

The experiment supports two separate conclusions:

1. Removing compensation makes the raw-shaft v171/v174 prediction too optimistic
   about immediate response, but the more immediate stopping mechanism was the
   weighted-mean command collapsing into the motor deadband.
2. Compensation alone is insufficient. Plain MPPI repeatedly changes the
   requested insertion/coupled direction, forcing slow take-up transactions
   that consume most of the target timeout. Grouped/scored selection keeps a
   complete feasible plan intact and sharply reduces this churn.

The next clean attribution experiment would hold compensation fixed and vary
only (a) weighted mean versus scored-best execution, followed by (b) grouped
U/C proposals and take-up-risk/reversal scheduling. That separates the
nonconvex execution-map problem from proposal coverage and reversal policy.

## Source anchors

- `src/catheter_control/config/v175_grouped_hardware_no_rotation.yaml`
- `src/catheter_control/config/v175_plain_takeup_hardware_no_rotation.yaml`
- `src/catheter_control/config/v171_plain_hardware_no_rotation.yaml`
- `src/catheter_control/catheter_control/mppi.py:1225-1228`
- `src/catheter_control/catheter_control/mppi.py:1551-1600`
- `src/catheter_control/catheter_control/mppi.py:1619-1652`
- `src/catheter_control/catheter_control/mppi.py:1694-1700`
