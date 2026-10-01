# Plain MPPI: failed circular-path simulation versus successful two-axis hardware run

## Scope

Passive comparison of:

- `mppi_ab_plain/20260917_191047_mppi_sim`, the earlier plain-MPPI
  continuous circular-path simulation; and
- `20260929_180313_mppi_demo`, the later uncompensated plain-MPPI
  rotation-disabled four-point hardware run.

The two sessions are not a controlled plant A/B test. They differ in target
semantics, enabled axes, transmission arbitration, rollout horizon, and
physical versus simulated plant. The evidence is nevertheless sufficient to
identify why the old controller stalled and why the new task was easier.

## Recorded outcomes

The old path contained 240 knots and 80.811 mm of arc length. It advanced only
2.297 mm (2.843%). It therefore failed on the approach segment before tracing
the circular portion. After 5.6 s the path governor entered `PAUSED`; the
reference then remained fixed while the cross-track error stabilized near
8 mm.

The new hardware run reached all four static targets. Their initial errors
were 14.56, 17.97, 23.21, and 21.19 mm. Active convergence took approximately
3.3, 8.8, 11.3, and 4.9 s, and terminal status errors were 1.68, 1.67, 1.46,
and 1.47 mm.

## Finding 1: the old weighted MPPI execution was a bad action

Confidence: observed.

The old controller used one ungrouped population, 1,024 samples, and the
weighted feasible-candidate mean. It was not sample-starved in nominal batch
size. However, a median 908/1,024 candidates were blocked and the median
effective sample count was only 24.3.

For 421 of 430 comparable active plans, the weighted command's predicted
terminal error was worse than zero control. Median tracking costs were:

- best sampled candidate: 583.995;
- zero control: 586.218;
- executed weighted mean: 608.321.

Thus useful candidates existed, but their advantage over hold was small and
the convex average of incompatible modes destroyed it. The implementation at
that time had neither grouped U/C proposals nor scored-best-candidate
execution.

The new run still used the plain weighted mean, but the static targets gave a
much stronger directional signal. Its median best, zero, and weighted costs
were 83.889, 192.177, and 168.341. The weighted terminal prediction beat hold
in 151/269 comparable plans rather than 9/430. Its median effective sample
count was 106.7, despite using only 512 nominal samples.

## Finding 2: rotation take-up arbitration turned planner indecision into churn

Confidence: observed.

The old run had backlash compensation and atomic take-up transactions enabled
on all three logical axes. During 434 active status samples:

- 218 reported `takeup_active`;
- the take-up generation reached 28;
- 676/1,314 path updates were in `TRANSMISSION_HOLD`;
- the rotation command changed nonzero sign 25 times; and
- simulated rotation position accumulated 870.35 degrees of total variation
  for only -42.32 degrees of net displacement.

After insertion and tendon stopped changing materially, rotation continued to
alternate. Because path progress is intentionally frozen during transmission
take-up, each reversal delayed fresh task-space control. Once the Cartesian
error crossed the pause threshold, the fixed reference did not resolve the
underlying weighted-mean ambiguity, so the cycle repeated.

The new run explicitly disabled rotation, backlash compensation, and take-up
transactions. MPPI commands went directly through the normal safety and motor
limits to insertion and tendon. There was therefore no transaction state that
could hold target progress and no high-play rotation axis that could consume
the trial through repeated reversals.

## Finding 3: the tasks supplied very different optimization signals

Confidence: observed for target geometry; inferred-high-confidence for its
effect on proposal weighting.

The old reference moved at 1 mm/s and began at the measured tip. Its recorded
plan contained four control stages on the fine continuous-path grid. Small
incremental reference displacements were comparable to transmission delay and
model error, so hold and opposing rotation branches had similar cost.

The new task used fixed points 14.6--23.2 mm from the measured home tip and an
explicit four-step, 0.8 s point horizon. Each point began after an encoder home
with rotation locked. These large, static, primarily one-direction errors made
the cost gradient persistent across replans. Even an imperfect weighted mean
usually produced useful motion, and feedback could correct it without chasing
a moving phase variable.

## Finding 4: timing was not the primary distinction

Confidence: observed.

The old planner elapsed time had a 41.6 ms median and 52.8 ms p95, with only
four deadline-miss-zero status cycles. The new planner was not faster: 46.3 ms
median and 61.2 ms p95. The old stall was therefore a control-selection and
transmission-arbitration failure, not a compute deadline failure.

## Conclusion

The new run does not show that plain MPPI is generally robust to backlash or
continuous tracking. It shows that plain MPPI can solve large, static,
rotation-disabled targets with a long coarse horizon and no take-up arbiter.
The old run exercised the opposite corner: a slowly moving local reference,
all three axes, large rotation play, and transaction holds. In that setting,
the plain weighted mean repeatedly discarded the best sampled plan and drove
take-up direction churn.

For a fair modern circular-path retest, hold the model, 512 samples, and 0.8 s
four-step horizon fixed, then compare plain versus grouped/scored selection
with identical compensation and path-governor settings. The successful
two-axis point run alone should not be used to retire grouped selection.
