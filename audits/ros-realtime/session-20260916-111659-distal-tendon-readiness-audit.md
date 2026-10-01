# Distal-tendon/readiness rerun audit — 20260916_111659

## Scope

Passive analysis of:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260916_111659_mppi_sim/20260916_111659_mppi_sim_0.db3
```

The run used the per-axis response-readiness correction and the new
marker-corrected distal-bending tendon evidence. No live ROS or hardware state
was changed during this audit.

## Outcome

The run completed without controller fault. Point 1 reached the 1.8 mm
tolerance; points 2--8 exhausted their 10 s budgets.

| Point | Target (mm) | Initial error (mm) | Minimum error (mm) | Final error (mm) | Result |
|---:|---|---:|---:|---:|---|
| 1 | `[33.601, 10.000, 60.976]` | 12.690 | 0.878 | 1.121 | reached |
| 2 | `[33.601, 7.071, 68.047]` | 18.037 | 5.117 | 5.173 | timed out |
| 3 | `[33.601, 0.000, 70.976]` | 26.156 | 7.616 | 7.673 | timed out |
| 4 | `[33.601, -7.071, 68.047]` | 31.915 | 4.943 | 5.155 | timed out |
| 5 | `[33.601, -10.000, 60.976]` | 33.518 | 3.057 | 3.273 | timed out |
| 6 | `[33.601, -7.071, 53.905]` | 30.594 | 3.331 | 3.609 | timed out |
| 7 | `[33.601, 0.000, 50.976]` | 24.789 | 3.710 | 3.950 | timed out |
| 8 | `[33.601, 7.071, 53.905]` | 17.930 | 2.957 | 3.416 | timed out |

## F-046 correction result

Point 1 is a successful before/after discriminator for the response-readiness
failure:

- it reached tolerance in 2.55 s;
- target error never exceeded its 12.690 mm starting value;
- physical tendon motion was monotonic, with zero sign reversals;
- raw tendon position moved from 0 to -29,992 counts and transmitted tendon
  state moved from 0 to -25,424 counts;
- shaft 2 entered `TAKEUP` at approximately 1.20 s, `PROVISIONAL` at 1.40 s,
  and `ENGAGED` at 1.60 s.

In the preceding `20260916_101403` run, the same point spent the full 10 s,
peaked at 31.284 mm error, moved tendon raw position as far as -99,574 counts,
and reversed once because a stopped subthreshold shaft blocked the shared
response gate. That failure mode did not recur.

The simulator's interface response crossed its attribution gate before the
distal projected increment crossed 0.05, so this run confirmed shaft 2 through
the interface channel. The distal channel was active and recorded projected
changes, but its instantaneous confirmation flag was not the winning channel
for point 1. This does not invalidate it: on hardware, where tendon bending may
occur with little interface-body motion, the same accepted marker posterior
can independently produce the provisional handoff.

## Later points and reachability

Points 2--8 commanded exactly zero tendon velocity and showed zero raw or
transmitted tendon-state change. Encoder-only homing again left target 1's
transmitted tendon remanence at -25,424 counts, so later points started from a
different distal shape than point 1. Their terminal tips generally passed the
requested `x=33.601 mm` plane and ended around `x=36--39 mm` while approaching
the requested Y/Z coordinate. Their failures are therefore consistent with a
target plane too close to the base-frame z axis for the remanent precurved
shape, not renewed tendon chatter.

As a diagnostic only, the recorded response curves were compared against
hypothetical parallel target planes. Shifting the plane from +10 mm to
+12.5 mm relative to the initially frozen tip reduced mean closest-point error
from 3.951 to 2.308 mm, and five of eight recorded curves intersected the
1.8 mm tolerance instead of one. Larger constant shifts degraded point 1 and
did not improve the worst point enough to justify them. This counterfactual
does not predict the exact closed-loop trajectory under the new target; it is
used only to select the next conservative test plane.

Both sparse simulation and hardware configurations now use
`center_x_offset_mm: 12.5`.

## Scheduling health

The estimator reported `TRACKING` on 1,440 of 1,445 status samples. There were
no faults. Completed planning time over 702 active status samples was
38.077/48.543/54.032/68.174 ms at P50/P95/P99/max. One plan exceeded the
60 ms planning deadline and produced one zero-command deadline cycle; the
consecutive-miss count never exceeded one.

Feedback ages remained bounded:

| Quantity | P50 | P95 | P99 | Max |
|---|---:|---:|---:|---:|
| POS age (ms) | 7.765 | 8.532 | 8.811 | 9.170 |
| ENC age (ms) | 17.313 | 47.619 | 48.368 | 87.252 |
| marker age (ms) | 13.580 | 51.223 | 73.180 | 83.884 |

## Conclusion

The per-axis readiness remediation passed its representative simulation gate:
the previous unconfirmed tendon excursion and reversal disappeared. The next
sparse run should evaluate the +12.5 mm target plane. It still will not be a
strict independent-point plant test unless simulator transmitted tendon state
is reset or each point's target is generated from its actual post-home distal
shape; encoder homing alone cannot erase remanence.
