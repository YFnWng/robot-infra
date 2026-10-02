# O1 implementation and FK microbenchmark

Date: 2026-10-02. Non-actuating source-level qualification; no hardware run.

## Implemented scope

- The authoritative PCS function in `cr-common` accepts `return_endpoint=True`.
  It uses the same section exponentials and composition, without collecting or
  stacking intermediate transforms. `FlexiblePCSRigidTipKinematics.endpoint`
  adds the rigid tip along the final material tangent, avoiding a separate
  zero-rotation SE(3) exponential. Full geometry callers remain unchanged.
- Control-only rollouts use endpoint FK. Diagnostic rollouts retain centerlines.
- Marker section indices and local material distances are computed once per
  runtime; four local marker transforms are evaluated in one batch.
- Frozen stiffness, damping and identity buffers are runtime-local. Each
  implicit update factors its operator once using LU and reuses it across
  Picard iterations. Shared scalar/horizon dt remains shared across candidate
  RHSs; explicitly nonuniform candidate dt retains batched operators. No
  unbounded dt-key cache, explicit inverse or new SPD assumption is introduced.
- MPPI gain learning/scenario lookups use small direction tables, and the eight
  reversal-mode winners/costs cross the device boundary together. Sampling,
  costs, projection, tie order and selection policy are unchanged.
- Rich diagnostics retain their fields, calculations and cadence, but transfer
  the tensor telemetry record to the CPU together. Covariance health checks
  have not been removed or deferred.

This does not implement O2 device/executor separation or the larger propagation
payload redesign. Projection scratch-buffer reuse and further transfer reduction
remain profiling-driven follow-ups, not measured benefits of this change.

## FK measurement

Report:
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o1_20261002/fk_benchmark.json`

The reusable benchmark is `runtime_supervision.benchmark_fk`. It loads the
selected manifest geometry: eight 5.3125 mm flexible sections and a 14.5 mm
rigid tip. Seeded synthetic curved/twisted strains are identical for both
routes. Both float32 and float64 were measured, with 20 warmups and 100 timed
calls, alternating route order. CPU uses two Torch threads. CUDA measurements
synchronize before/after each call and therefore measure completed wall time,
not just enqueue time. GPU: RTX 4090; Torch: 2.5.1+cu121.

Float32 results, milliseconds:

| Device | Batch | Full FK P50 | Endpoint P50 | Speedup | Full FK P95 | Endpoint P95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| CPU | 1 | 0.276 | 0.159 | 1.74x | 0.296 | 0.177 |
| CPU | 21 | 0.331 | 0.196 | 1.69x | 0.350 | 0.202 |
| CPU | 512 | 0.875 | 0.636 | 1.38x | 0.944 | 0.672 |
| CPU | 1536 | 1.440 | 1.087 | 1.32x | 1.570 | 1.145 |
| CUDA | 1 | 0.699 | 0.400 | 1.75x | 0.783 | 0.457 |
| CUDA | 21 | 0.719 | 0.409 | 1.76x | 0.860 | 0.502 |
| CUDA | 512 | 0.722 | 0.412 | 1.75x | 0.823 | 0.500 |
| CUDA | 1536 | 0.733 | 0.416 | 1.76x | 1.252 | 0.716 |

Maximum coordinate difference in all benchmark rows: zero. Dedicated tests
also cover scalar/multidimensional batches, straight and near-zero strain,
curvature/twist, gradients, batched markers, implicit-step reference equations,
and shared versus explicitly replicated/nonuniform dt.

These are isolated per-call measurements, not complete planner or estimator
speedups. CUDA host/kernel-launch overhead is included; small batches need not
be faster than CPU. Do not extrapolate a ROS deadline guarantee from this table.
No sample count, horizon, uncertainty scenario, safety gate, model checkpoint,
or command authority changed.

Verification: 64 model/optimization/import/bundle/packaging tests passed with
CPU and CUDA available; 53 MPPI and frozen cross-repository conformance tests
passed. The `catheter_control` and `runtime_supervision` ROS packages rebuilt
successfully. No live timing or actuator test was performed.

## Reproduction

Run in the model environment with the source repositories on the import path:

```bash
cd /home/chen-lab/Yifan
export PYTHONPATH="$PWD/robot-infra/src/runtime_supervision:$PWD/robot-infra/src/catheter_control:$PWD:$PWD/cr-common:$PWD/control${PYTHONPATH:+:$PYTHONPATH}"
/home/chen-lab/Yifan/cr-venv/bin/python -m runtime_supervision.benchmark_fk \
  --model-manifest "$PWD/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json" \
  --output /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/compute_optimization_o1_repeat/fk_benchmark.json
```

The benchmark refuses to overwrite the output file. Tests and measurements
explicitly used source model packages; the pre-existing installed model wheel
is not silently replaced. Rebuild/reinstall the model packages before any
deployment qualification, verify imported file identity, then repeat O0 under
representative full-stack load. Historical manifest qualification remains
limited to its original recorded scope.
