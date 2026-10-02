# Control computation audit and optimization plan

Date: 2026-10-02. Status: source audit and proposed gates; no runtime changes.

## 1. Objective and constraints

Reduce estimator latency and planner tail latency without changing mechanics,
engagement belief, take-up behavior, candidate selection, costs, integration
grid, sample budget, or safety policy. Qualify the 512-sample, four-step,
0.8 s point horizon first; then measure continuous-path and 1024-sample workloads
separately. A language change is not itself a real-time qualification.

This plan supplements, rather than replaces, the existing
[C++ shell plan](../../docs/architecture/PHASE4_CPP_ROS_SHELL_PLAN.md).
C++ command authority remains deferred and requires separate approval.
No hardware operations, installs, scheduling changes, or benchmarks competing
with a running controller were performed in this audit.

## 2. Baseline evidence

[Recorded timing audit](session-20261002-211138-timing-audit.md): 94 completed
solves, 13 deadline-miss rows including warm-up. Final fault followed three
consecutive solves of 60.876, 60.472, and 62.619 ms against a 60 ms budget.

| Work | P50 / P95 / P99 / max (ms) | Evidence population |
|---|---|---|
| Planner | 55.04 / 63.13 / 66.13 / 81.87 | 94 solves |
| Projection/preparation | 9.75 / 17.87 / 18.87 / 19.21 | 94 solves |
| Rollout | 37.74 / 47.37 / 51.11 / 52.90 | 94 solves |
| Cost/selection | 3.01 / 7.63 / 10.20 / 20.37 | 94 solves |
| Estimator owner | 91.80 / 107.15 / 119.61 / 119.61 | latest callback sampled at solves |
| Marker correction total | 56.04 / 68.40 / 71.16 / 72.70 | latest correction sampled at solves |
| Marker detection | 4.07 / 5.47 / 6.54 / 78.17 | 1893 diagnostics |

The estimator owner nominal period is 20 ms; planning period is 66.67 ms.
Detection is not the dominant sustained cost. Serial command forwarding is
also not the first optimization target. Do not sum asynchronous stage maxima
or interpret sampled latest metrics as independent callback distributions.

## 3. Current execution boundary

```text
ROS device/marker ingress -> newest pending samples
  -> one estimator owner: encoder propagation -> marker rewind/UKF/replay
     -> encoder propagation -> immutable planner snapshot
  -> planner: CPU proposals/projection -> GPU rollout -> costs/selection
     -> CPU result -> ROS heartbeat -> manager -> serial/firmware
```

Observed in source: `node.py` constructs one runtime using the selected compute
device (lines 274-277) and passes it to the planner. For this session that device
is CUDA. Separate mutually-exclusive callback groups allow estimator and
planner work to overlap; they do not isolate Python execution or GPU work.
One owner of mutable estimator state is an important invariant to retain.
Snapshot exchange locks were short in the fault snapshot, so wholesale lock
rewriting is not the first intervention.

## 4. Findings and proposed remedies

Paths below are relative to `/home/chen-lab/Yifan`.

### F1: Tiny estimator kernels share CUDA with the large planner batch

Confidence: source-observed; contribution to measured jitter is a hypothesis.

- `robot-infra/src/catheter_control/catheter_control/node.py:274` selects a
  single runtime device for propagation, correction, and rollout.
- `.../planning/mppi.py:31` calls `torch.cuda.synchronize(device)` after rollout
  and selection. These barriers are device-wide, not solve-specific.
- `cr_meta_lnn/deployment/v171_streaming_runtime.py:888` already batches UKF
  sigma points. It nevertheless uses small Cholesky/solve/SVD operations and
  scalar conversions, which can make accelerator launch/synchronization cost
  important relative to arithmetic.

First compare CPU versus CUDA estimator replay with identical observations.
Likely target: CPU estimator, GPU planner, explicit complete-state snapshot
transfer once per solve. Keep model parameters immutable and shared by artifact
identity, not mutable state. Initially use the same model implementation on
both devices; do not create independently evolving equations.

If CUDA estimation is retained, benchmark explicit streams and event-based
dependency/measurement boundaries. Never remove synchronization just to make
timing look shorter: deadline completion must include completed GPU work.
Device-wide waiting can include unrelated work; the bag does not quantify how
much. An event measures a stream dependency, not all wall-clock delay.

### F2: Tip-only rollout still evaluates unnecessary geometry

Confidence: source-observed; speedup unmeasured.

- `_predict_sequence` (`v171_streaming_runtime.py:1549`) calls
  `distal_model.points` at every horizon step, then retains only its endpoint
  when markers and centerline are disabled.
- The active loader builds `FlexiblePCSRigidTipKinematics` (eight flexible
  sections plus rigid tip), not the dense material-grid observation class.
  Its output contains the interface, eight section endpoints, and the tip.
  `networks/hybrid/kinematics.py:162` supplies dense material-grid sampling for
  training observations, not the active planner. Endpoint-only evaluation can
  avoid intermediate output storage, but must still compose all eight section
  transforms; no savings from eliminating 64 dense samples should be claimed
  for the current production rollout.
- `_marker_points` (`v171_streaming_runtime.py:1713`) recomputes static marker
  section indices/local lengths on each call.

Add an exact endpoint-only kinematic method: compose all flexible-section
transforms and rigid-tip translation without dense sample evaluation or output
stacking. The same PCS equations remain the owner. Precompute marker section
indices/local distances; batch their local exponentials. Keep dense geometry
for visualization and diagnostic callers. Verify endpoint and marker equality
across curved, straight, twisted, and near-singular configurations.

### F3: Repeated constant operators and full state construction

Confidence: source-observed; relative cost unmeasured.

- `_implicit_step` (`v171_streaming_runtime.py:491`) rebuilds stiffness,
  damping, identity, and `D+hK+floor*I`, then solves the same operator inside
  every Picard iteration. Frozen mechanics and shared dt mean the operator is
  identical across candidate RHSs for that step.
- `_advance_motor` (`:553`) constructs a complete streaming state, copies the
  adaptive Jacobian, and clones adaptation bookkeeping per step, including
  hypothetical candidate propagation where adaptation is frozen.
- `_expand_state` (`:1532`) deep-clones first and then expands/clones tensor
  fields. Existing code does batch candidates; there are not 512 Python
  runtime instances.

Cache frozen mechanics buffers by artifact/device/dtype. Factor the operator
once for a shared dt and reuse the factorization for all batch RHSs and Picard
iterations. Use LU initially unless SPD assumptions are verified for Cholesky;
do not substitute an explicit inverse. Support measured nonuniform estimator
dt without an unbounded floating-point-key cache.

Introduce a narrow tensor propagation payload with all physically evolving
history fields, while sharing immutable model/J metadata. Adaptation and health
bookkeeping stay in the streaming owner. Preserve the complete public snapshot
contract and independent candidate memory; no aliasing of mutable history.

### F4: Repeated CPU/GPU crossings and candidate-wise Python work

Confidence: source-observed.

- `planning/mppi.py:991` and `:1019` loop over candidates for learning scales
  and three gain scenarios; gain depends primarily on direction, so a small
  direction table can be indexed over the full batch.
- `:1499` performs mode-wise reductions and transfers winners/costs to CPU
  individually. Later capture/selection and diagnostics repeatedly convert
  scalar tensors with `.cpu()`, `int`, and `float`.
- Projection is already NumPy-vectorized across candidates with a horizon
  loop (`:596`); optimize allocations and repeated validation before assuming
  a C++ port will deliver a large gain.

Keep projection on CPU initially to preserve its safety-sensitive semantics.
Vectorize direction lookups, reuse scratch buffers, and upload one compact
proposal batch. Batch mode reductions on GPU and download one compact decision
record. Where CPU capture policy needs intermediate values, use a deliberately
bounded transfer boundary, then one final result boundary; do not force a
misleading promise of exactly one transfer for every policy.

Preserve seeded candidate arrays for comparison. Moving random generation to
GPU changes the sample stream and is a separate experimental change.

### F5: Estimator correction, covariance work, replay, and diagnostics

Confidence: source-observed; individual subcosts still need profiling.

- `_positive_covariance` (`v171_streaming_runtime.py:728`) eigen-decomposes the
  reduced covariance; `_marker_ukf` uses multiple solves plus observable-space
  SVD, and `_gauge_fixed_covariance` repairs covariance again.
- `_advance_motor` repairs covariance on scalar propagation steps. This must
  not be removed without proving PSD/gauge preservation.
- `observe_markers` (`:1417` vicinity) scans/slices a rewind list, clones the
  selected state, then rebuilds/clones every replayed state. This is necessary
  delayed-observation semantics implemented with potentially avoidable
  allocation, not evidence that rewind itself should be disabled.
- `diagnostics` (`:1739`) computes NumPy eigenvalues/SVD and Torch eigenvalues,
  then serializes many fields. `node.py:878` and `:1022` call it after encoder
  and marker processing, not only at low-rate diagnostics publication.

Separate essential validity calculations from rich diagnostic serialization.
Cache Jacobian validity only while J/covariance are unchanged; covariance
health checks required by gates must remain timely. Serialize rich telemetry
from immutable snapshots outside the owner critical path at its existing
diagnostic cadence. Instrument its actual cost first.

Reuse one innovation factorization for NIS and gain RHSs. Cache static UKF
weights/scales/identities and marker maps. Preserve observable/gauge correction.
Later evaluate fewer covariance repairs only as a separately reviewed numerical
algorithm change with adversarial covariance tests.

Use a bounded timestamp-indexed history with reusable storage if allocation
profiling warrants it; retain interpolation, raw/effective motor streams,
corrected historical snapshots, and chronological replay. Cap memory, not
physical history semantics. Replay counts and substeps must be recorded.

## 5. Ordered implementation gates

| Gate | Deliverable | Acceptance |
|---|---|---|
| O0: profile | Reusable non-actuating replay/profiler CLI using recorded inputs and frozen proposal fixtures | Every owner callback measured individually; CPU time, wall time, CUDA kernels/transfers/barriers, allocations, replay count, and diagnostic cost distinguished |
| O1: remove redundant work | Endpoint-only FK, static maps/buffers, cached shared operator factorization, batched direction/mode lookups, compact telemetry | Existing conformance fixtures plus edge cases pass; same sample/cost/decision behavior within declared tolerances |
| O2: compute isolation | CPU estimator vs CUDA comparison; then immutable CPU-to-GPU snapshot boundary if beneficial | Delayed replay, all history, J/RLS, gain belief, timestamp and rejection decisions preserved; total concurrent-load tails improve including copy overhead |
| O3: accelerator execution | Fixed-shape tensor kernel for four-step rollout and selection; benchmark eager vs compiled vs graph replay | No on-demand compilation during control; warm-up complete before readiness; guard failures remain fail-closed; improve P95/P99, not just mean |
| O4: native core, conditional | C++ estimator/FK or projection kernel only where O0-O3 identify residual bottlenecks | Cross-language fixtures and shadow replay pass; bounded allocations and runtime costs demonstrated |
| O5: production scheduling | Qualify existing C++ shell and worker protocol under representative recording load | No safety authority migration until prior shell conformance/timing gates and explicit approval |

Recommended first implementation: O0, then O1. Do not simultaneously rewrite
the estimator, projection, ROS wrapper, and model.

O0 implementation (2026-10-02): the bounded recorded-input CLI and its tests
are available; see [profiling commands and scope](tools/COMPUTE_PROFILE.md).
It reuses the deployed-artifact benchmark and recorded timing tools, adds
per-call wall/thread-CPU measurements, optional cProfile/Torch allocation and
kernel traces, frozen seeded proposals, and replay workload counts. Full-stack
executor/GIL/scheduler attribution remains a qualification task; serial offline
workload replay is explicitly not exact online belief or callback replay.

O1 implementation (2026-10-02): endpoint-only FK, static marker/mechanics
buffers, shared per-update LU factorization, batched gain/mode lookups and
compact tensor telemetry are implemented. CPU/CUDA edge-case equivalence,
deployment packaging and frozen planner conformance pass. Isolated FK P50 at
512 float32 samples improved 1.38x on CPU and 1.75x on RTX 4090; see the
[O1 scope and benchmark report](O1_FK_BENCHMARK_20261002.md). Projection scratch
reuse and larger propagation-payload redesign remain follow-ups. Full-stack
timing and installed-model promotion are not qualified by this microbenchmark.

## 6. Where C++ helps, and where it does not

Preferred eventual boundary:

```text
C++ ROS shell: ingress, timestamps, lifecycle, watchdog, bounded snapshots
  -> compact CPU estimator core (C++ only if profiling justifies it)
  -> batched GPU planner kernel (Python orchestration can remain initially)
  -> complete validated decision -> existing manager -> existing firmware
Python: training, model exploration, task sessions, plots, offline analysis
```

- CPU UKF/PCS/history core: promising native target because dimensions are small
  and work includes Python dispatch, allocation, and many small tensor ops.
  Prototype CPU Torch first; port stable math only after measuring residual cost.
- Projection/belief transitions: compact deterministic native candidates,
  with one native implementation and Python bindings if promoted. Keep the
  current Python reference for conformance, not a second evolving production
  implementation. Reuse adaptive-Jacobian algebra through its owning API.
- GPU rollout: C++/LibTorch alone still launches essentially the same kernels;
  it does not automatically fuse them, remove dense geometry, or eliminate
  synchronization. Prefer tensor-kernel simplification/compilation first.
- ROS shell: useful for isolating heartbeat/lifecycle from Python worker
  stalls, but cannot make a 100 ms estimator fast by changing its caller.
- Teensy: future low-level regulation/telemetry only. Do not move learned
  estimation or large MPPI batches there, change calibration, or bypass final
  safety authorities in this optimization project.

Compilation/graphs are conditional on the installed Torch/CUDA version.
Current tensor-to-Python branches, dynamic visibility, and dataclass mutation
make compiling the entire runtime a poor first attempt. Compile a pure tensor
kernel with fixed horizon/batch/scenario shapes; use explicitly warmed variants
for supported masks/shapes. Preserve validated error handling outside it.
See [PyTorch graph-break guidance](https://docs.pytorch.org/docs/main/user_guide/torch_compiler/compile/programming_model.graph_breaks_index.html)
and [CUDA graph guidance](https://pytorch.org/blog/accelerating-pytorch-with-cuda-graphs/).
Bounded native execution also requires controlling allocation/page faults and
blocking, not just selecting C++; see [ROS real-time guidance](https://github.com/ros2/ros2_documentation/blob/rolling/source/Capabilities/Motion-planning/Real-Time-Programming.rst).

## 7. Measurement and promotion criteria

Proposed engineering targets, not existing qualified guarantees:

- Planner: P95 <= 45 ms, P99 <= 50 ms at 512 samples; every measured warmed
  production solve remains below the unchanged 60 ms deadline in qualification.
- Estimator owner: aim for P99 <= 15 ms against its 20 ms period. If unreachable,
  report the deficit; a rate/scheduling redesign is a separate reviewed gate,
  not a hidden optimization or a deadline relaxation.
- Heartbeat: measure entry lateness, execution, command age, and output intervals
  against its 10 ms period; require no freshness/deadman failures.
- Compare marker source age, accepted commit age, snapshot age at solve entry
  and completion, and source-to-command age. Do not optimize only solve time.
- Evaluate 512 and 1024 samples, 1/3 gain scenarios, two/three active axes, point
  and path references, static/motion/reversal, partial marker visibility,
  initialization versus steady tracking, and worst allowed delayed replay.
- Use camera/recording off/on representative-load tests, only when hardware
  access is authorized. Replay speed alone is not a live qualification.
- Report n, P50/P95/P99/max, misses, consecutive-miss streaks, skipped/replaced
  observations, CPU/GPU utilization, replay depth, numerical and discrete
  decision differences. Keep cold starts separate. Profiler runs and ordinary
  timing runs must be separate because instrumentation perturbs execution.
- No reduced sample count, shorter horizon, removed uncertainty scenarios,
  relaxed gates, or new model checkpoint may be credited as an equivalent
  computational optimization. These are separate research/configuration tests.

Unresolved: exact kernel launch/compute ratio; GPU cross-workload barrier cost;
time in diagnostics/clones versus matrix algebra; OS/GIL/native-thread wait;
recording resource contention. O0 should resolve these before selecting a port.

## 8. Ownership and verification

`cr_meta_lnn` owns shared mechanics, geometry, UKF, history/replay, and rollout
kernel. `robot-infra` owns proposal/cost policy, ROS scheduling, conversion,
telemetry, worker contracts, and safety integration. Tests must include complete
snapshot clone/restore, candidate independence, nonuniform dt, coarse-step
equivalence, delayed correction/replay, out-of-order/rejected markers, PSD and
gauge behavior, joint projection/limits/coupling, all controller variants, and
discrete reasons at decision boundaries. Near ties need declared policy and
explicit tolerance review rather than silently accepting changed winners.

Use existing Phase 5 deterministic conformance fixtures and Phase 4 shadow
contracts; add focused tests instead of creating another generic framework.
Preserve manager/firmware authority, source stamping, watchdogs, command-output
interlock, and the unconditional rejection of encoder-zero modification.
