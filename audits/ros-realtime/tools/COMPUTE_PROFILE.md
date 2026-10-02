# O0 recorded-input computation profiling

The reusable offline CLI lives in `runtime_supervision.compute_profile`.

The complementary isolated FK benchmark lives in
`runtime_supervision.benchmark_fk`; see
[O1 results and reproduction](../O1_FK_BENCHMARK_20261002.md).
It extends the deployed-artifact benchmark pattern in `safety.validation`
(canonical runtime/planner, phase timers, percentile summaries), uses the same
ROS storage/CDR reader pattern as `qualify_control_cycle_timing.py`, and reuses
the production marker conversion. No node, publisher or hardware command is
created. It refuses to run when a local `catheter_mppi` process is detected;
also ensure other GPU workloads are idle before comparing performance.

## Build and run

Use a ROS terminal. The console entry point selects `cr-venv` automatically.

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select catheter_control
source install/setup.bash

PROFILE_SESSION=/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/attribution_twoaxis_points_grouped_n512_mount01_repeat01_20261002T211138237441Z
MODEL_MANIFEST=/home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json
LIMITS_FILE=/home/chen-lab/Yifan/robot-infra/src/control_interface/config/catheter_limits.yaml

ros2 run runtime_supervision compute_profile "$PROFILE_SESSION" \
  --model-manifest "$MODEL_MANIFEST" --limits-file "$LIMITS_FILE" \
  --device cpu --samples 512 --active-axes 2 --gain-scenarios 3 \
  --start-offset-s 39 --duration-s 5 \
  --output "$PROFILE_SESSION/compute_profile_cpu_timing"
```

Repeat with `--device cuda` and a new output directory to compare devices.
Run intrusive profiling separately: same inputs/start, use `--duration-s 1`
and add `--profiler`, with e.g. `compute_profile_cuda_trace` as output.
Intrusive captures are capped at 2 s because operator/allocation traces can
reach hundreds of MB even in a short window. Do not compare their deadline-miss rate to
an ordinary timing run: shape tracking, allocation tracking, cProfile, and
Torch tracing all perturb execution. Start offset is relative to the first
selected bag receipt; 39 s covers the first target in this specific session.
Use `--prefix /sim` for a simulation bag. Existing output directories are
rejected to preserve previous measurements.

## Outputs

- `report.json`: artifact identity, exact workload configuration, source window,
  per-stage n/P50/P95/P99/max wall and calling-thread CPU time, every solve's
  canonical phase timing, correction acceptance/rejection and replay workload.
- `timing_rows.csv`: individual propagation, correction, diagnostics, cloning
  and planner calls, rather than sampled latest callback values.
- `proposal_fixture.npz`: canonical seeded U/C proposals and group IDs for the
  fixed direction fixture. Array-content SHA-256 in the report allows checking
  CPU/CUDA input parity (ZIP-file hashes can differ with ZIP timestamps).
- With `--profiler`: `python_calls.prof`, readable `python_calls.txt`, structured
  `function_costs.json`, and `torch_trace.json` for CPU/native operations, CUDA
  kernels, copies, synchronization, and tensor-memory events. Open the Chrome
  trace in a compatible trace viewer. Python peak allocation is separately
  reported and does not include native/tensor memory.

## Interpretation boundary

This is a **serial algorithm workload replay**, not exact closed-loop replay.
Recorded solve-completion receipts trigger solves; the profiler does not know
the original callback-entry sequence or scheduling wait. Encoders are source-
time thinned to 50 Hz and markers are latest-pending, causally deferred, and
rate-limited to 20 Hz. The prefix is replayed before the measured window without
resetting history or instrumenting it. Trials do not trigger hidden-state reset.

The workload uses UKF, disabled Jacobian adaptation, four exact 0.2 s coarse
point steps, explicit sample/axis/scenario budgets, raw encoder inputs and a
fixed engaged transmission fixture with zero gap and prior gain uncertainty.
It does **not** reconstruct the online effective motor, transmission/gain
posterior, warm start, point-capture history, arm lifecycle or command execution.
Every solve starts from a fresh seeded zero nominal so CPU/device comparisons
are not confounded by timing-dependent previous solves. Actual online decisions
must not be attributed using these replay decisions.

Per-call wall time is not pure GPU kernel duration, and calling-thread CPU time
does not include PyTorch worker threads. No additional CUDA barrier is inserted
around each estimator method; use the Torch trace to distinguish asynchronous
launches and completed work. Nested cProfile cumulative times must not be added.
Replay entry/substep counts inspect the existing retained runtime history
read-only; rejected corrections report zero replay work.

Use the existing `qualify_control_cycle_timing.py` separately for recorded
full-stack camera/transport/callback metrics. O0's algorithm profiling tool is
implemented; executor-ready wait, GIL contention, kernel scheduler attribution,
and every real ROS callback event remain full-stack tracing qualification work.
Neither offline timing nor profiling passes hardware real-time qualification.

Implementation verification: five focused profiling/conformance tests passed;
the `catheter_control` package rebuilt and its installed CLI help ran. A bounded
CPU recording replay exercised 90 propagation calls, 36 marker corrections,
126 diagnostics/clones and two solves in its selected two-second window;
cProfile, structured function costs, allocation summaries and Torch CPU trace
were exported. Smoke used eight samples/one gain scenario, not a performance
qualification. CUDA execution was not qualified in this environment.
