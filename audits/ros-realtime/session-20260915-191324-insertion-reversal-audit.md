# Session 20260915_191324 MPPI simulation: insertion reversal audit

## Scope

Passive analysis of the recorded simulation bag at:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260915_191324_mppi_sim/20260915_191324_mppi_sim_0.db3
```

No ROS process or hardware state was changed.

## Result

The provisional response handoff removed the previous rotation-dominated
limit cycle, but exposed a separate insertion sampling-memory defect. The
insertion reversals are not explained by the path reference crossing the tip,
by bend/insertion coupling, or by estimator noise.

### Observed path behavior

- Recorded path interval: 43.628 s.
- Maximum progress: 5.739 mm of 76.918 mm (7.46%).
- Governor samples: 1,186 `TRANSMISSION_HOLD`, 120 `RUNNING`.
- Closest-path error: median 0.325 mm, P95 0.433 mm, maximum 0.455 mm.
- Point-reference error: median 0.433 mm, P95 1.029 mm, maximum 1.477 mm.

The new path-tube behavior is therefore functioning: geometric error stayed
small and the governor did not pause for path error. Progress was instead
suppressed by repeated transmission transactions.

### Observed command behavior

- 108 selected plans were recorded.
- Logical insertion reversed 56 times; physical insertion-shaft intent also
  reversed 56 times.
- Rotation reversed six times and remained predominantly negative.
- Bend was identically zero, so firmware bend/insertion coupling cannot
  explain the insertion reversals in this session.
- The insertion joint moved over 0 to 29.291 logical units and had the same 56
  direction changes.
- `TAKEUP_ACTIVE` occupied 349 of 434 status samples (80.4%).

For every nonzero consecutive insertion command, the target-minus-tip vectors
were compared at direction reversals. Unlike the prior rotation failure, the
Cartesian error generally did **not** cross the target: 54 of 56 error-vector
dot products were positive and the median cosine was 0.860. Representative
cosines were 0.92, 0.94, 0.99, and 1.00 while reference errors remained
approximately 0.4--0.6 mm.

### Transmission behavior

The response detector was decisive rather than noisy. During the action, 182
insertion observations simultaneously exceeded the 0.5 attribution-evidence
and 0.1 rad inferred-motion thresholds. Nevertheless, each newly selected
opposite insertion command immediately opened another approximately 6.9--8.2
rad take-up transaction at 8 rad/s. The resulting roughly 0.8--1.0 s hold
dominates a path advancing at only 1 mm/s.

## Mechanism

At ordinary successful `REPLAN_REQUIRED`, the ROS node currently calls
`planner.reset()`, clears `last_effective_command`, and then requests the fresh
post-take-up solve. This correctly prevents execution of the stale action, but
also erases the MPPI sampling distribution and the previous-command input to
the slew cost.

The next solve is therefore centered at zero with insertion noise standard
deviation 4.0. Rotation remains directionally determined by the target, while
insertion is weakly constrained inside the sub-millimetre path tube. The
minimum-cost sampled candidate can then choose either insertion sign. That
sign opens another expensive insertion take-up transaction. The existing
`mppi_first_step_reversal_weight` parameter is declared and validated but is
not applied to the cost function.

Confidence classifications:

- repeated insertion reversal and transaction occupancy: **observed**;
- absence of bend coupling in this bag: **observed**;
- Cartesian error remaining aligned across insertion reversal:
  **observed**;
- planner reset removing sampling and slew memory: **observed in source**;
- weakly constrained zero-centered sampling selecting alternating insertion:
  **inferred-high-confidence**.

## Required correction

1. On ordinary response-completed `REPLAN_REQUIRED`, discard the executable
   pre-take-up action but preserve MPPI's nominal sampling sequence and the
   prior desired post-engagement command used by the slew cost.
2. Continue to reset planner memory on `SATURATED_REPLAN`, target/path change,
   disarm, fault, or model reset.
3. Implement the already exposed first-step reversal cost in physical shaft
   coordinates. It should penalize a sign reversal relative to the previous
   nonzero physical desired rate, not forbid it. A real Cartesian target
   crossing must still be able to overcome the penalty.
4. Keep the current provisional response thresholds and path-tube behavior;
   this bag does not implicate either one.

## Verification

1. Unit-test that ordinary take-up completion preserves nominal and previous
   command memory, while saturation/fault reset them.
2. Unit-test the first-step physical reversal term and prove that a sufficiently
   better reversing trajectory still wins.
3. Repeat the same seeded 1 mm/s circle. Require fewer than five insertion
   reversals in the first 10 mm, `TRANSMISSION_HOLD` below 30% of path samples,
   monotonic progress beyond 10 mm, and closest-path maximum below 2 mm.
4. Repeat with controller/truth take-up width mismatch before any hardware
   test.
