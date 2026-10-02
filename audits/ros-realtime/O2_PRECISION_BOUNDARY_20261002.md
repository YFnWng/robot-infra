# O2 explicit estimator/planner precision boundary

## Implementation

The candidate uses the existing complete **float64 CPU estimator**, not a
second UKF implementation. MPPI uses the existing float32 CUDA runtime.
The [numerical diagnostic](O2_ESTIMATOR_NUMERICS_20261002.md) established
complete CPU/CUDA float64 conformance over 45 seconds of recorded input;
float32 had weak-observability rank sensitivity. No covariance floor, SVD
cutoff, rejection gate, safety threshold, model equation or artifact changed.

`estimator_device:=cpu estimator_dtype:=float64` selects this experimental
boundary with a CUDA planner selected through the existing `device` argument.
An empty estimator dtype keeps float32. Both launch entry points expose the
parameter; default device and precision behavior is unchanged.

`RuntimeState.clone_to(device, dtype=torch.float32)` copies every floating
tensor to owned float32 storage. Integer and Boolean tensors, timestamps,
NumPy arrays, J/RLS metadata and optional history retain their types and values.
The cast never writes back to the estimator. It completes once outside owner
locks before a solve or preview; its duration is included in the planner
deadline and reported as `plan_snapshot_transfer`. Same-device precision
changes also copy. Runtime-pair startup verifies matching manifests/artifacts,
requested precision, and a dtype-aware snapshot API; obsolete packages fail
closed. Diagnostics report both compute devices and dtypes.

The artifact manifest already permits float32 and float64. Its historical
qualification and hashes remain unchanged: this precision combination is **not
a newly qualified production baseline**, and the installed model wheel has
not been replaced.

## Verification and remaining gate

CPU/CUDA tests cover complete snapshot casting and ownership, discrete types,
float64-reference coarse rollout, old API rejection, configuration defaults,
and same-device precision conversion. Snapshot rollout error uses the existing
`rtol=2e-5, atol=2e-7 m` tolerance. The full float64 estimator mathematics is
unchanged from the numerical-reference pass.

The existing O2 benchmark accepts `--estimator-dtype float64`. CUDA reference
cases still use float32; cross-precision estimator state differences are
diagnostics, not an expectation of equality. Same-device isolated/concurrent
checks remain the reproducibility test. Its fixed nominal planner fixture is
shared across cases; CPU float64 transfers cast that fixture to float32.
This excludes three adaptive-gain scenarios and ROS/camera/recording load,
and runs an unpaced planner worker. It cannot certify live callback latency.

Before deployment, run paced evolving-snapshot replay with the actual gain
belief scenarios, check marker acceptance/rank against float64 references,
then obtain authorization for full-stack non-actuating shadow qualification.
Report sensor-to-estimator and sensor-to-command age, callback tails, transfer
cost, planner tails, deadline misses and freshness failures together. Do not
relax those gates to obtain a pass.

## Recorded benchmark

Input: `attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z`.
Window: 39–41 seconds, complete causal prefix; 512 samples, four 0.2-second
rollout steps, 60 ms deadline; manifest-selected RLS disabled.
Output: external session directory
`compute_optimization_o2_precision_boundary_20261002/comparison.json`.

Concurrent nominal-workload wall timings (milliseconds):

| Stage | CPU float64 estimator / CUDA float32 planner P50 / P95 / P99 | Shared CUDA float32 P50 / P95 / P99 |
| --- | --- | --- |
| Encoder propagation | 1.89 / 2.91 / 3.29 | 4.05 / 5.40 / 5.77 |
| Marker rewind/correction/replay | 16.14 / 19.85 / 21.23 | 32.30 / 36.34 / 39.24 |
| Planner including handoff | 26.01 / 27.15 / 29.44 | 22.70 / 24.79 / 26.61 |

Float64-to-float32 handoff P50/P95/P99: 0.431/0.535/0.557 ms. Neither
concurrent case missed 60 ms (32 split and 73 shared plans); worker counts
reflect differing unpaced replay wall durations, not control throughput.
CPU float64 marker-update P50/P95 in isolation was 8.26/9.31 ms.
Compared with the earlier CPU float32 concurrent pass (15.41/17.90 ms), the
float64 marker tail is modestly higher; these are separate runs, not a paired
statistical estimate. It remains substantially below this run's shared GPU
marker tail. Planner tails are not improved by splitting computation.

All isolated/concurrent states agree within their own precision, and all 36
window marker decisions match. The final marker difference between CPU float64
and CUDA float32 is 0.00482 mm; latent/covariance differences remain, as expected
from the previously established float32 sensitivity. This is not a claim of
cross-precision state conformance or improved physical estimation accuracy.

Verification: 52 CPU/CUDA model tests and 278 ROS/runtime-supervision/bringup
tests passed; the three affected ROS packages rebuilt. Separately, six of seven
phase-5 tests passed; the trace fixture fails with a `Time` object missing
`sec`, including after preloading generated ROS message modules. This test
environment issue remains unresolved and is not a full-suite pass.

Resolved in the subsequent [paced gate](O2_PACED_QUALIFICATION_20261002.md):
the failing `Time` was ROS 1 `genpy.rostime.Time`, supplied by a shadowing ROS 1
`std_msgs.Header`, not a test stub. Put venv NumPy ahead of system NumPy, and
Humble message paths ahead of `/usr/lib/python3/dist-packages`. The prior
test-stub hypothesis was incorrect. The corrected environment passes phase 5
and the combined package suite without preloading or changing production code.

Reproduce with the source model import root ahead of the installed wheel:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="/home/chen-lab/Yifan:/home/chen-lab/Yifan/cr-common:/home/chen-lab/Yifan/control:$PYTHONPATH"
/home/chen-lab/Yifan/cr-venv/bin/python -m runtime_supervision.benchmark_compute_isolation \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z \
  --model-manifest /home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json \
  --limits-file src/control_interface/config/catheter_limits.yaml \
  --estimator-dtype float64 \
  --output /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o2_precision_boundary_repeat
```

The CLI refuses detected local controllers and overwriting output directories;
it reads the bag only and creates no device connection or command publisher.
