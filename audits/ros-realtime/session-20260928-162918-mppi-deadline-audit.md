# MPPI deadline audit: `20260928_162918_mppi_sim`

## Outcome

The failure is a deterministic compute-budget mismatch, not an isolated
scheduling spike.  All five executable planning attempts exceeded the 60 ms
deadline and the controller correctly faulted after five consecutive misses.

The planning cycle must not be slowed in isolation.  The ROS node currently
commits only `plan.command_logical_velocity` (the first element of the MPPI
sequence) and the heartbeat repeats that command until the next plan.  The
model's first control interval is 40 ms.  Reducing the planner from 15 Hz to
3--4 Hz without changing command scheduling would therefore apply a command
designed for 40 ms for 250--333 ms.

## Evidence

- Session: `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260928_162918_mppi_sim`
- Compute device: NVIDIA GeForce RTX 4090 (`cuda:0`)
- Samples: 144
- Optimized horizon: 4 x 40 ms
- Point prediction tail: 7 x 120 ms
- Configured planning rate/deadline: 15 Hz / 60 ms
- Deadline misses at fault: 5 consecutive

| Measurement | Count | P50 | P95 | P99 | Maximum |
|---|---:|---:|---:|---:|---:|
| Total planner latency | 5 | 206.218 ms | 218.670 ms | 220.999 ms | 221.581 ms |
| Model rollout | 5 | 194.846 ms | 205.995 ms | 208.150 ms | 208.689 ms |
| Sample projection | 5 | 7.459 ms | 9.321 ms | 9.547 ms | 9.603 ms |
| Cost weighting | 5 | 2.082 ms | 2.571 ms | 2.591 ms | 2.596 ms |
| Update projection | 5 | 0.702 ms | 0.749 ms | 0.751 ms | 0.752 ms |

The rollout consumes about 94% of total planning time, so executor tuning or a
small deadline increase cannot recover the current 15 Hz rate.

## Recommendation

1. A temporary simulation-only timing diagnostic may use 3 Hz with a 280 ms
   deadline.  This should eliminate the deadline fault, but its tracking result
   is not control-valid because the first action is held too long.
2. Before using the slower cadence to judge tracking or deploying it on
   hardware, add a time-consistent command policy.  The smallest safe policy is
   to execute the first MPPI action for its modeled 40 ms interval and then
   command zero until a new plan is available.  A higher-performance policy
   would execute later sequence elements only through the same take-up and
   engagement arbitration used for the first element.
3. Retain the independent 100 Hz heartbeat, estimator rate, freshness gates,
   and manager watchdog.  Only the expensive planner cadence/deadline should
   change.
4. Re-measure the full-stack planner latency after the scheduling change; use a
   deadline above observed P99 with explicit margin and below the plan period.

## Interpretation limits

Only five distinct executable plans were recorded because the fail-closed
threshold stopped the run.  The samples are internally consistent, but a
longer successful run is required to characterize thermal and background-load
tails.
