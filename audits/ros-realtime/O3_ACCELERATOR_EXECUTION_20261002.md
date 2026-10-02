# O3: fixed-shape accelerator execution

Status: implemented as an explicit offline/startup-prepared API. CUDA graph
numerical and isolated timing gates pass. **ROS readiness integration and
hardware promotion remain pending.** No physics artifacts, sample budget,
horizon, deadlines, freshness gates, manager or firmware policy changed.

## Ownership and execution

`cr_meta_lnn/deployment/control_accelerator.py` owns the four-step tensor
recurrence and owned eager/Inductor/CUDA-graph buffers. It reuses the existing
motor transmission, tendon-history step, implicit mechanics, SE(3) exponential
and authoritative endpoint FK rather than copying their equations.

`CatheterRuntime.prepare_control_accelerator(...)` is explicit preparation;
only the coarse four-step, tip-only predictor uses its result. One unbatched
posterior supplies pose, strain, both motor streams, all five history fields,
equilibrium correction, lambda anchor and frozen J. Rates, shared per-step dt
and gain remain dynamic tensors. Complete final-state metadata, covariance,
anchors and RLS are cloned unchanged; all evolved fields are returned.

Preparation must finish before readiness. Signature mismatch raises an error,
never lazy compilation, fallback, padding or probe removal. Compiled execution
is full-graph/fixed-shape with error-on-recompile. Output cloning prevents later
replays from overwriting prior decisions. Buffers are single-consumer objects.

Graph capture cannot run LU's host error check. The authoritative implicit-step
owner exposes an optional `lu_factor_ex` status route; the public accelerated
predictor checks status/finite strain before returning. Errors reach MPPI's
existing fail-closed rollout handling. Default estimator/eager LU is unchanged.

`planning/tensor_selection.py` extracts existing weighting. Production retains
the original eligible-population std reduction. The benchmarked fixed-shape
variant uses equivalent masked moments, within declared tolerance. Capture,
reversal policy, cost construction, projection and command authority remain
outside it. Full planner measurements accelerate **rollout only**; selection
graphs are separately benchmarked, not integrated into live selection.

`runtime_supervision/benchmark_accelerator.py` is installed as
`ros2 run runtime_supervision benchmark_accelerator`. It refuses a detected
local controller and never stops processes, opens devices or creates commands.
Preparation is separate from warmed timing; public-rollout times include input
validation, copies, owned outputs and final-state reconstruction.

Grouped sampling appends probes: the tested two-axis lease uses **518 actual
candidates for 512 stochastic samples**, or 1030 for 1024 samples. Three gain
scenarios flatten these to 1554/3090 items. Future ROS startup must enumerate
and warm every supported lease/candidate/scenario signature; the present API
deliberately supports just one explicitly prepared signature per runtime.

## Observed isolated performance

RTX 4090, Torch 2.5.1+cu121, float32, two Torch CPU threads, four coarse 0.2 s
steps, 100 warmed repetitions. P50/P95/P99/max in ms. References use identical
physics/proposals in the same process. These are synthetic frozen workloads,
**not O2 paced replay or live ROS timing**.

| Workload | Reference P50/P95/P99/max | CUDA graph P50/P95/P99/max |
| --- | --- | --- |
| Public rollout, 512 × 3 scenarios | 10.11 / 10.72 / 11.29 / 14.97 | 2.78 / 3.22 / 3.33 / 3.35 |
| Grouped planner, 512 × 3 scenarios | 15.16 / 16.31 / 16.49 / 17.35 | 7.55 / 8.16 / 8.36 / 8.62 |
| Grouped planner, 512 × 1 scenario | 14.51 / 15.17 / 15.51 / 15.56 | 7.30 / 7.94 / 8.10 / 8.21 |
| Grouped planner, 1024 × 3 scenarios | 16.58 / 17.49 / 17.66 / 18.03 | 9.28 / 9.98 / 10.12 / 10.63 |

All three planner fixtures preserve validity/reason, selected index, command
and best cost. Each case has 100 valid plans per variant per execution mode
and zero 60 ms deadline misses. "Plain compensation" is a planner-policy
fixture, not a replay of the take-up arbiter or hardware baseline.
Standalone selection at 512 × 3 has P95/P99 0.21/0.31 ms in tensor eager and
0.08/0.09 ms in graph replay. Graph preparation takes approximately 80–109 ms
in these fixtures; it must not enter a control deadline.

Inductor rollout compilation fails on this Torch build with
`BackendCompilerFailed: TypeError: Expected a number but got Identity`.
Failure is reported explicitly, not disguised as eager execution. The simple
compiled-kernel guard test passes; full mechanics Inductor remains unapproved.
TF32 precision was not enabled to improve timing.

Evidence directory:
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o3_20261002`:

- `full_planner_512_final.json`
- `full_planner_512_one_scenario.json`
- `full_planner_1024.json`
- `compiled_final.json`

Earlier diagnostic reports are retained, not overwritten. Manifest SHA-256:
`2e9f1ccdade740fcbaae4c941eb992e15e9561f3b8b059f247182fccde9b1dd5`.

## Verification and remaining gates

CPU/CUDA tests cover nonuniform dt, distinct raw/effective rates, evolving
history/correction/J, gain scenarios, evolved tensor state, output/snapshot
independence, finite/dt/signature guards, preparation failure, LU failure,
unprepared use and weighting/ties. Relevant ROS tests/builds and installed
benchmark help pass: 311 ROS-package tests, 39 model/CPU/CUDA tests with one
CPU-only graph case skipped, and successful `catheter_control` /
`runtime_supervision` builds. No installed model wheel or historical fixture changed.

The full historical Phase-5 capture also reports five small numeric differences
in estimator residuals and final strain (maximum strain delta 1.91e-5 versus
its 2e-6 tolerance), with no changed discrete reasons or planner decisions.
This is **not a passing historical conformance gate**. Its fixture was not
loosened or regenerated. Resolve that estimator/source-identity/numerical
comparison before claiming whole-stack baseline equivalence; it is separate
from passing O3 accelerated-versus-current-reference comparisons.

Before promotion:

1. Resolve the historical conformance discrepancy.
2. Integrate a bounded enumerated signature set into startup/readiness, with
   no preparation or recompilation in planning callbacks.
3. Repeat O2 paced replay with evolving CPU64→GPU32 snapshots, path references
   and three active axes, not just these frozen fixtures.
4. Run separately authorized full-stack non-actuating shadow tests with
   cameras/recording on/off, measuring callback timing, input/command ages,
   deadline streaks, replaced observations and heartbeat intervals.
5. Only then promote a ROS profile and installed model artifact. This result
   does not authorize a new hardware default or 1024-sample experiment.

## Reproduce (controller stopped; no motor commands)

```bash
source /opt/ros/humble/setup.bash
cd /home/chen-lab/Yifan/robot-infra
source install/setup.bash
export PYTHONPATH=/home/chen-lab/Yifan:$PYTHONPATH

ros2 run runtime_supervision benchmark_accelerator \
  --model-manifest /home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json \
  --limits-file /home/chen-lab/Yifan/robot-infra/src/control_interface/config/catheter_limits.yaml \
  --device cuda:0 --samples 512 --scenarios 3 \
  --warmup 5 --repeats 100 --modes eager cuda_graph \
  --output /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o3_repeat/report.json
```

Use a new output path. Add `compiled` to record that backend's compatibility
result. Cold compilation/capture and warmed performance are reported separately.
