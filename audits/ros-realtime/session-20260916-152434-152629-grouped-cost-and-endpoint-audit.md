# Grouped-cost and closed-path endpoint audit

Sessions:

- `20260916_152434_mppi_sim` at 1 mm/s
- `20260916_152629_mppi_sim` at 3 mm/s

## Outcome

The grouped-MPPI transaction costs materially reduced reversal chatter. Both
runs used 1,024 samples with switch weight 1 and take-up-delay weight 4. The
1 mm/s action completed. The 3 mm/s action reached 80.599/80.811 mm (99.74%)
before entering `PAUSED`; the pause was a path-governor branch-selection bug,
not a transmission hold or controller fault.

| Metric | 1 mm/s | 3 mm/s |
| --- | ---: | ---: |
| Active path duration | 67.17 s | 27.68 s before pause |
| Final/max progress | 80.811/80.811 mm | 80.599/80.599 mm |
| Path-error RMS before pause | 1.176 mm | 1.275 mm |
| Path-error P95 before pause | 2.552 mm | 2.643 mm |
| Path-error maximum | 2.608 mm | 2.732 mm |
| Take-up transactions | 7 | 6 |
| Transmission-hold time | 5.383 s | 4.313 s |
| Nonzero rotation sign changes | 3 | 3 |
| Plan time P50/P95/max | 42.52/52.89/63.04 ms | 42.84/53.18/78.90 ms |
| Plans over 60 ms | 4/658 | 5/814 |

The preceding `20260916_150511_mppi_sim` contained 50 take-up holds totaling
44.85 s and 35 nonzero rotation sign changes. The new transaction costs
therefore removed the dominant mode chatter without preventing the required
large reversals.

## End-pause mechanism

At the first 3 mm/s `PAUSED` sample:

- governed progress was 80.599 mm;
- remaining path length was only 0.212 mm;
- reference error was 0.522 mm;
- closest geometric distance was 0.505 mm;
- the dense path sample nearest the tip was on the coincident circle-start
  branch at arc 17.842 mm rather than the endpoint branch near 80.51 mm.

`PathProgressGovernor._tracking_error_mm()` interprets a closest point behind
the governed phase as real reference lag. The branch jump therefore created
approximately 62.76 mm of apparent lag and tripped the 5 mm pause threshold.
The existing closed-path ambiguity guard handles only a closest point *ahead*
of progress; it does not handle a coincident earlier branch near closure.

Once paused, progress could not advance the final 0.212 mm. The final physical
tip remained close to the path (0.472 mm) and reference (0.633 mm), but the
artificial phase-lag metric could not fall below the 1.5 mm resume threshold.
The planner eventually selected deterministic zero with equal selected/zero
cost, which was locally reasonable for the frozen sub-millimetre reference.

## Recommended correction

Use a progress-local projection for governor phase error. Candidate path
projections should be restricted to a bounded arc neighborhood of the current
monotonic progress, or multiple geometric minima should be disambiguated by
arc continuity. Retain the unrestricted global closest point only for pure
geometric path-error reporting. A special endpoint snap would fix this one
trace but would not fix the same ambiguity at any closed-path intersection.

The isolated planner deadline misses recovered and did not cause the pause.
They remain timing-tail evidence but are not the proximate failure here.

## Remediation

Implemented in `catheter_control/path_tracking.py`. The path now exposes a
projection constrained to an arc interval around the current governed phase.
The governor uses this branch-continuous projection for catch-up, lag,
pause, and resume decisions, while retaining the unrestricted global closest
point for geometric error reporting and the hard-error safety check. A closed
path regression proves that a coincident earlier branch remains visible in
diagnostics without pulling the monotonic phase backward or latching pause.
All 229 catheter-control tests pass and the ROS package rebuilds cleanly. A
3 mm/s simulation rerun remains the runtime verification gate.
