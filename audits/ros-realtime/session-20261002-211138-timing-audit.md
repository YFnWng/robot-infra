# Recorded timing audit: grouped two-axis reaching

Session: `attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z` under the external catheter session root.

Offline, read-only bag analysis; no hardware operations or controller changes.
Full distributions: [timing JSON](session-20261002-211138-timing.json).

## Observed outcome

The controller fault was `planner:repeated_deadline_miss`, not marker rejection
or homing failure. Effective recorded configuration: CUDA, 512 samples, four
0.2 s point-rollout steps (0.8 s horizon), no prediction tail, 15 Hz planning,
60 ms deadline, three consecutive misses to fault. Command publication is
configured at 100 Hz; estimator timer at 50 Hz; marker updates at 20 Hz.
These are configured rates, not proof of achieved processing rates.

| Target | Duration to result/fault (s) | Recorded solves | Deadline-miss rows | Solve P50 / P95 / max (ms) |
|---|---:|---:|---:|---|
| 1, reached | 4.119 | 16 | 2 | 55.77 / 66.27 / 81.87 |
| 2, reached | 8.630 | 66 | 7 | 54.76 / 62.61 / 64.94 |
| 3, faulted | 4.356 | 12 | 4 | 56.29 / 63.66 / 64.93 |

There were 13 deadline-miss rows in 94 solves (13.8%). Rows include discarded
warm-up solves; this is not an executable-command miss rate. Target 3's last
three solves took 60.876, 60.472, and 62.619 ms, consecutively exceeding 60 ms.
Timing rows are published before updating the miss counter, hence the last
three rows carry counters 0, 1, 2; the fault snapshot reports 3.

## Recorded distributions

Planner values represent individual completed solves (n=94). Estimator/marker
values are the latest measurements sampled at solve completion (n=94), and may
repeat; they are not a complete per-callback population.

| Stage / age | P50 (ms) | P95 (ms) | P99 (ms) | Maximum (ms) |
|---|---:|---:|---:|---:|
| Planner elapsed | 55.04 | 63.13 | 66.13 | 81.87 |
| Planner callback elapsed | 55.07 | 63.65 | 66.30 | 82.07 |
| Sample projection | 9.75 | 17.87 | 18.87 | 19.21 |
| Rollout | 37.74 | 47.37 | 51.11 | 52.90 |
| Cost weighting | 3.01 | 7.63 | 10.20 | 20.37 |
| Update projection | 0.70 | 0.97 | 1.71 | 1.82 |
| Latest estimator callback | 91.80 | 107.15 | 119.61 | 119.61 |
| Latest estimator timer lateness | 80.63 | 100.03 | 114.23 | 114.23 |
| Latest marker pending age | 16.93 | 29.69 | 31.97 | 31.99 |
| Latest marker correction | 29.19 | 38.72 | 41.94 | 45.53 |
| Latest marker rewind | 7.91 | 15.76 | 16.47 | 19.29 |
| Latest marker replay | 18.73 | 30.46 | 33.73 | 40.99 |
| Latest marker total | 56.04 | 68.40 | 71.16 | 72.70 |
| Marker source age at solve completion | 203.11 | 243.15 | 246.89 | 247.99 |
| Accepted-marker commit age | 57.52 | 94.74 | 100.38 | 104.23 |
| Encoder commit / planner snapshot age | 56.58 | 97.13 | 105.94 | 112.08 |
| Position commit age | 4.77 | 9.98 | 10.52 | 10.55 |

Vision diagnostics (n=1893) report processing P50/P95/P99/max of
4.066/5.474/6.537/78.167 ms and inter-rig timestamp skew of
8.782/9.030/9.166/11.270 ms. Marker header-to-bag receipt age is
65.342/67.961/69.060/141.452 ms. This latter measurement includes capture/source
timestamp semantics, synchronization, processing, transport, and recorder
scheduling; it is not pure DDS latency. Status counts: 1649 TRACKING, 244
PARTIAL_MULTI_RIG_TRACKING, one INITIALIZED; no invalid-rig status recorded.

Value-matched command transport (bag receive timestamps): controller output
topic `/teleop/control` to manager P50/P95 = 0.464/0.847 ms (1657 matches,
seven unmatched); manager to serial TX = 0.659/1.245 ms (1664 matches, zero
unmatched). Repeated equal commands can make correspondence ambiguous; these
are approximate transport measurements, not uniquely identified transactions.
TX to next device feedback P50/P95 = 5.379/10.137 ms; this does not measure
mechanical response or prove command acknowledgment.

## Interpretation and limits

1. Observed: planning has approximately 5 ms median margin, while P95 already
   exceeds the 60 ms deadline. Target 3 failed because small overruns clustered,
   not because it encountered the session's largest single overrun.
2. Observed: expensive estimator callbacks substantially exceed the 20 ms
   timer period. `_estimator_tick` processes an encoder sample, a causal marker
   correction (UKF rewind/correction/replay), then another encoder sample.
   It does not detect markers in images. Recorded lateness and stale snapshots
   show that a configured 50 Hz timer does not yield 50 Hz full corrections.
3. Inferred: estimator and planner work under concurrent full-stack load leave
   insufficient scheduling/computation margin. Stage durations are wall time,
   including possible preemption/contention; CPU/GPU/scheduler causes cannot
   be separated from this bag alone. Do not add their overlapping durations
   to manufacture an end-to-end latency.
4. Observed: source-marker age is materially larger than commit age. Freshly
   committed UKF measurements still describe older camera observations.
   The snapshot-age timing field here is sampled at solve completion, whereas
   the fault snapshot's age can refer to the solve-input age; do not compare
   them as identical measurement points.
5. Solve completion intervals have medians 67.86/68.16/68.88 ms by target,
   near the configured 66.67 ms planning period. Multi-second gaps include
   warm-up/compensation/gating intervals, so solve intervals are not an
   unconditional control-loop frequency measurement.
6. No evidence establishes video recording as the cause. A matched recording
   on/off run or scheduler trace is needed to attribute contention. Increasing
   only the planner deadline would not fix estimator lateness or observation
   age; any scheduling change needs full-stack timing qualification.

## Reproduction

Source ROS Humble and the workspace overlay, then run the existing
`audits/ros-realtime/tools/qualify_control_cycle_timing.py` with this session
path and `--output` pointing to the linked JSON. Trial windows come from
`trials.jsonl`; fault evidence comes from `controller.log`; effective parameters
come from `controller_manifest.json`. Relevant code:
`src/catheter_control/catheter_control/node.py`, `_estimator_tick`,
`_plan_tick` and `_publish_control_cycle_timing_locked`.
