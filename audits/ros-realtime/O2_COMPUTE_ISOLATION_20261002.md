# O2 compute isolation: implementation and qualification boundary

## Decision

The opt-in implementation is available. **O2 is not yet qualified for default
deployment.** CPU estimation reduces estimator latency in recorded concurrent
workload replay, but the planner tail does not improve and complete CPU/CUDA
UKF state equivalence exceeds the deliberately strict declared tolerances.
No controller was armed, hardware link opened, safety gate changed, or installed
model wheel replaced. The ROS real-time audit skill guided the ownership,
timing and qualification boundaries.

## Implementation

- `device` still selects the planner device. New `estimator_device` is an
  optional device string; its empty default inherits `device`, preserving the
  single-runtime path. Both control and simulation bringup expose it.
- Split runtimes load the same manifest, artifact hashes, dtype and backend.
  Mismatches fail startup. Old model packages without `RuntimeState.clone_to`
  also fail startup for split configurations, rather than failing during motion.
- The estimator remains the only owner of propagation, UKF correction,
  rewind/replay and mutable history. The planner runtime consumes explicit
  snapshots; it does not maintain a second evolving estimator.
- Snapshot transfer packs tensors by source device/dtype, transfers each group
  synchronously, and restores all fields without casting or aliasing the source.
  Non-tensor J/RLS objects and NumPy arrays are independently copied. Timestamps,
  pending adaptation evidence, covariance, accepted anchors, tendon history and
  reconciliation fields are retained, not initialized from the planner model.
- Transfer occurs outside estimator and snapshot-exchange locks, once per solve
  (or target preview), and is measured as `plan_snapshot_transfer`. Its duration
  is included in the existing planner deadline. ROS transmission/gain beliefs
  continue through the existing snapshot path unchanged.
- Callback groups, owner cadence, freshness thresholds, deadline, command
  authority and watchdog behavior are unchanged. No C++ implementation was
  introduced: the present bottleneck comparison does not justify it yet.

## Recorded comparison

Input: `attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z`.
Selected receipt window: 39–41 s, with the entire preceding history replayed;
90 propagation calls and 36 delayed marker updates per case. The O0 causal
deferral and thinning are reused without resetting their counters at the window.
The concurrent worker repeatedly plans from an identical owned frozen fixture,
seed 17, 512 samples, four 0.2 s coarse steps, 60 ms deadline. Adaptation updates
are disabled. No transmission/gain-belief scenario fixture is supplied to the
planner; this measures the nominal rollout workload, not the full adaptive
three-scenario workload. This is not full online gain-belief reconstruction or live ROS
callback scheduling. Two Torch intra-op threads and one inter-op thread were
used on RTX 4090 / Torch 2.5.1+cu121, float32.

Final report:
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o2_final_20261002/comparison.json`.
An earlier independent run is retained under `compute_optimization_o2_20261002`.

| Workload | CPU estimator P95 / P99 (ms) | CUDA estimator P95 / P99 (ms) |
|---|---:|---:|
| Isolated propagation | 1.22 / 1.25 | 2.79 / 3.18 |
| Isolated marker correction + rewind/replay | 9.59 / 10.17 | 20.85 / 21.98 |
| Concurrent propagation | 3.06 / 3.53 | 5.36 / 5.72 |
| Concurrent marker correction + rewind/replay | 17.90 / 18.72 | 34.72 / 35.99 |

Concurrent planner P95 / P99 was **27.57 / 28.82 ms** with CPU estimation and
**23.35 / 23.99 ms** with CUDA estimation. CPU snapshot-transfer P95 / P99 was
**0.61 / 2.56 ms**, included in those solve times. No solve exceeded 60 ms in
either case (32 split solves, 72 shared solves). Counts differ because replay is
unpaced and CPU replay finishes sooner; this is not a throughput or control-rate
comparison. The planner worker saturates its own loop rather than matching a
live timer cadence. These short-window percentiles are descriptive, not robust
full-stack P99 qualification.

## Conformance and remaining gate

Follow-up: [45-second numerical qualification](O2_ESTIMATOR_NUMERICS_20261002.md)
localizes float32 differences to UKF observable-rank/covariance arithmetic,
including identical-prior probes. Float64 passes the original state tolerances.
No default or estimator equation has changed; numerical stabilization and
representative timing qualification remain open.

Exact complete snapshot CPU→CUDA→CPU copy and coarse-rollout equivalence pass
in source tests. For recorded replay, concurrency produces no final-state
differences relative to the same device's isolated run. All cases have identical
marker acceptance/rejection decisions and discrete final-state values.

However, CPU vs CUDA UKF replay fails `rtol=3e-5, atol=2e-6` for several internal
fields: maximum pose-matrix element difference 0.000569, strain difference
0.0231, lambda difference 0.00882, covariance difference 0.289 and observable
projection difference 0.0794. These are differences from executing the same
float32 estimator on different backends, not from snapshot transfer. Pose-matrix
entries mix rotation and translation; do not interpret that matrix maximum as
a Cartesian distance. The maximum final reconstructed marker-position
difference is only **0.00349 mm**, but this alone cannot qualify future
covariance, observability or adaptation decisions. Numerical tolerances were
not loosened to obtain a pass.

Before promotion:

1. Diagnose backend sensitivity through UKF factorization/observability and
   longer replay, including enabled J/RLS and online gain-belief traces. Keep
   rejection decisions and timestamps exact; review physical tolerances for
   continuous fields explicitly.
2. Compare paced concurrent work and representative recording/tracking load,
   including ingress age, estimator owner backlog, transfer, planner and
   heartbeat tails. The current split improves estimator tails, not every tail.
3. Rebuild/verify the deployed model package and runtime import identity, then
   perform explicitly authorized full-stack shadow qualification. No live
   timing or hardware qualification is inferred from this offline replay.

## Reproduction

Use source imports until the model package is deliberately promoted; otherwise
an older installed wheel can shadow the edited runtime. Output must be a new
directory. This command reads a bag and runs math only, not a ROS controller.

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/catheter_control:$PWD/src/runtime_supervision:/home/chen-lab/Yifan:/home/chen-lab/Yifan/cr-common:/home/chen-lab/Yifan/control:$PYTHONPATH"
/home/chen-lab/Yifan/cr-venv/bin/python -m runtime_supervision.benchmark_compute_isolation \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z \
  --model-manifest /home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json \
  --limits-file src/control_interface/config/catheter_limits.yaml \
  --samples 512 --start-offset-s 39 --duration-s 2 \
  --output /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o2_repeat
```

Ensure other GPU workloads and live controllers are idle. This CLI reuses O0's
local controller-process refusal guard. It requires CUDA and does not substitute
CPU results when GPU access is unavailable.

Source verification: 47 CPU/CUDA model tests pass; 274 ROS/runtime/bringup tests
pass together, and seven phase-5 tests pass separately (their ROS message stubs
conflict when collected with the wider suite). The three affected ROS packages build
successfully. No full-stack timing run was performed.
