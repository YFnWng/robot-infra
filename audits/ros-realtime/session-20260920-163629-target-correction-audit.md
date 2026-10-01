# 2026-09-20 continuous-path target-correction simulation audit

Session: `20260920_163629_mppi_sim`

Comparison baseline: `20260920_161502_mppi_sim`

## Scope

This audit evaluates only the continuous-path target correction. The v171
distal tendon history, v175 interface transmission/play model, take-up
transactions, grouped MPPI proposal modes, manager, and simulator plant were
not changed by that correction.

Evidence was read from the finalized SQLite3 bag. The corrected runtime is
directly identified by `RECOVERY_ADVANCE` path states and noncollapsed preview
horizons during those states.

## Outcome

| Metric | Before correction | Corrected run |
| --- | ---: | ---: |
| completed arc | 29.700 / 80.811 mm | **80.811 / 80.811 mm** |
| completed fraction | 36.75% | **100%** |
| monotonic progress | yes, then asymptotic stop | **yes, through endpoint** |
| action terminal status | canceled/interrupted | **SUCCEEDED** |
| final reference error | not at endpoint | **1.464 mm** |
| final closest-path error | not at endpoint | **1.388 mm** |
| maximum closest-path error | 4.656 mm | **6.587 mm** |
| hard-error samples | 0 | **0** |

The target deadlock is fixed. The action no longer holds an unreachable local
point indefinitely; it exposes later path directions and completes the circle.

This is not an accuracy improvement over the unreachable segment. Across all
2,331 path updates, closest-path error was 5.033 mm median and 6.542 mm P95.
The governor spent 1,282 updates (55.0%) in `RECOVERY_ADVANCE`; during those
updates closest-path error was 5.994 mm median and 6.558 mm P95. This is the
expected consequence of traversing a path segment outside the simulated
workspace rather than declaring that segment reachable.

## Reference and control behavior

- Preview span increased from 0.052 mm median in the stalled baseline to
  **0.800 mm median**, demonstrating that the forward window no longer
  collapses near the old pause boundary.
- Path progress remained monotonic.
- The simulated rotation coordinate ranged from `-116.49 deg` to `0 deg` and
  returned to `-2.89 deg` at session end. The controller therefore continued
  rotating around the circle instead of holding at approximately 30 mm arc.
- Planned-command nonzero-sequence reversals were 6 insertion, 3 rotation,
  and 3 tendon. The correction did not introduce a high-frequency reversal
  limit cycle in simulation.
- Model-response endpoint error remained small: 0.030 mm median, 0.198 mm
  P95, 0.782 mm maximum across 179 causal response traces.

## Estimator and timing

Estimator health was stable: 2,265 `TRACKING` traces after 7 initializing
traces, zero rejected marker updates, and maximum rejection streak zero.
Post-correction marker RMS was 0.019 mm median and 0.029 mm P95.

Planner elapsed time was 51.05 ms median, 61.24 ms P95, 66.00 ms P99, and
76.17 ms maximum against the configured 60 ms deadline. Sixty-four diagnostic
status publications reported `planner_deadline_miss_zero`; the action still
completed without a repeated-deadline fault. These status samples are not
independent deadline events, so the bag does not support interpreting 64 as an
exact miss count. Timing margin remains limited and must be checked in every
hardware session.

## Findings

### F-163629-1: Target deadlock removed

Severity: resolved high

Confidence: observed

The corrected forward-corridor objective and recovery progression allowed the
reference to traverse the full path and terminate successfully.

### F-163629-2: Unreachable geometry is traversed, not accurately tracked

Severity: medium

Confidence: observed

The corrected controller makes the requested semantic choice—continue along
the path while remaining inside the hard capture tube—but carries about 6 mm
cross-track residual through the unreachable arc. A completed action must not
be reported as evidence that all path geometry was reachable.

### F-163629-3: Planner deadline margin remains small

Severity: medium

Confidence: observed

P95 planner time exceeded the configured 60 ms deadline. No fault occurred in
this run, but hardware qualification must retain deadline monitoring and abort
on a repeated-miss fault.

## Hardware implication

The correction is suitable for a bounded hardware isolation test because it
preserved the hard capture tube, estimator gates, manager authority, and
endpoint capture. It is not sufficient justification for running the same
circle on hardware with rotation disabled. That mechanism is restricted to a
two-DOF reachable surface, while this circle required substantial simulated
rotation and included a known unreachable segment.

The next hardware test should therefore use independent, model-generated
sparse targets in the insertion/tendon reachable surface, returning to
`[20,0,0]` before every target. Continuous path tracking should follow only
after those sparse two-DOF targets pass.
