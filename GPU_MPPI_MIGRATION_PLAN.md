# GPU MPPI Migration Plan

## Goal

Raise MPPI sampling from the current CPU baseline of 32 samples to an initial
CUDA target of 1,024 samples while preserving the 4 x 40 ms rollout horizon,
60 ms command-commit deadline, exact joint projection, controller lifecycle,
manager/firmware safety authority, and CPU fallback.

This migration is non-actuating until the CUDA configuration passes offline,
simulation, and full-stack shadow timing gates. It does not modify encoder
calibration or introduce any `SET_ZERO` route.

## Implementation Status (2026-09-14)

Implemented through Phase G3 in source:

- fail-closed CPU/CUDA device resolution and diagnostics;
- v171 tip-only control rollout with full-rollout compatibility retained;
- accelerator-synchronized rollout and cost timing;
- compact post-cost device-to-host result transfer;
- configurable sample/horizon/step preflight with per-phase distributions;
- composable controller/performance parameter overlays;
- non-actuating 1,024-sample CUDA shadow profile;
- separate CPU-by-default simulation truth-model device, preventing synthetic
  perception from contending with the controller GPU;
- source-stamp POS/ENC pairing independent of processed-feedback freshness;
- CPU and runtime regression tests.

CUDA execution remains unverified on this host because `nvidia-smi` cannot
communicate with the driver and `cr-venv` reports
`torch.cuda.is_available() == False`. This is a runtime/environment gate, not
a reason to weaken startup validation.

As an informative isolated result, the optimized tip-only path completed 100
CPU plans at 1,024 samples x 4 steps with 29.51 ms P50, 33.87 ms P95,
37.09 ms P99, and 37.37 ms maximum. This is not representative full-stack
evidence; simultaneous cameras, UKF, ROS scheduling, and recording still need
shadow measurement. It does show that removing unused rollout work is at least
as important as selecting the accelerator.

## Current Boundary

- The v171 runtime already places models and state on its configured PyTorch
  device.
- MPPI sampling and safety projection are NumPy/CPU operations.
- Learned rollout and costs use PyTorch.
- The planner currently requests full centerlines and marker predictions even
  though tip-only MPPI does not consume them.
- CUDA operations are asynchronous, so CPU-style phase timers are not valid
  unless the device is synchronized before a timing boundary.
- The controller `device` parameter selects the complete estimator/rollout
  runtime. The first migration therefore moves both to CUDA; planner/estimator
  device separation is a later option if measured contention requires it.

## Phase G0: Device Preflight and Fail-Closed Startup

1. Resolve the requested PyTorch device before loading artifacts.
2. Reject `cuda` with an actionable startup error when CUDA is unavailable.
3. Report resolved device, CUDA name, memory allocation, and peak allocation
   in controller diagnostics.
4. Keep `cpu` as the default launch value.

## Phase G1: Control-Only Rollout

Add a v171 `predict_control_sequence()` entry point that shares the existing
state propagation but omits marker and stacked-centerline outputs. It still
computes the distal curve needed to extract the tip, but avoids a second
marker-kinematics pass and avoids retaining large unused tensors.

`CatheterMppi` uses this optimized method when present and falls back to the
existing `predict_sequence()` protocol for tests and other backends. Marker
prediction remains available through the old method and is not removed.

## Phase G2: CUDA-Correct Planner Timing

1. Synchronize CUDA after learned rollout before recording `rollout_ms`.
2. Compute costs, weighted command prediction, selected trajectory, cost, and
   effective sample size on-device.
3. Synchronize once at the end of cost weighting, then copy only the compact
   planner result to CPU.
4. Preserve callback-entry-to-command-commit deadline enforcement. A CUDA
   error or deadline miss returns an explicit zero result.
5. Keep CPU sample generation and exact projection for the first migration;
   at 1,024 x 4 these arrays are small, and this retains the already-tested
   hardware projection implementation.

## Phase G3: Benchmark Harness and Profiles

Extend the non-actuating preflight to accept sample count, horizon, and rollout
step so CUDA can be evaluated with the deployed artifacts and exact planner.
Record P50/P95/P99/max total plan time plus synchronized phase timings.

Add reviewed parameter profiles:

- CPU reference: 32 samples, 4 steps, 40 ms, `device: cpu`.
- CUDA candidate: 1,024 samples, 4 steps, 40 ms, `device: cuda`.

The CUDA profile must keep hardware output disabled. Enabling output remains a
separate explicit launch override after promotion.

## Promotion Gates

### Offline

- CPU and CUDA predictions agree within configured float32 tolerance for the
  same state and controls.
- All commands remain finite and exactly projected by the hardware contract.
- Warm-up completes before timing samples are collected.
- At least 100 timed plans: P95 <= 45 ms, P99 <= 55 ms, maximum <= 60 ms.
- No CUDA out-of-memory, asynchronous execution, or invalid-rollout failures.

### Simulation

- Run the existing trajectory with identical seeds and plant perturbations.
- No planner deadline, stale-command, manager, estimator, or marker fault.
- Effective sample size and tracking error improve or remain acceptable.
- Record complete phase timing and CUDA memory diagnostics.

### Full-stack shadow

- Real cameras, UKF, manager feedback, recording, and 1,024-sample MPPI run
  together with `command_output_enabled: false`.
- Report plan and estimator timing distributions, deadline misses, GPU memory,
  feedback ages, and scheduler tails.
- If UKF/MPPI GPU contention violates the gate, create a dedicated CUDA rollout
  runtime and keep estimation on CPU rather than weakening deadlines.

### Hardware

- Promote only the exact profile/artifact hashes that passed shadow testing.
- Begin with single interior targets, then the continuous path trial.
- Retain the 60 ms deadline and all existing fail-closed behavior.

## Deferred Optimizations

Move sampling, correlated-noise generation, projection, and warm-start update
to Torch/CUDA only if G1--G3 show CPU preparation dominates at larger batches.
Any CUDA projection rewrite must pass scalar/batched parity tests against
`HardwareContract`; performance alone is not sufficient. CUDA graphs,
`torch.compile`, separate GPU streams, mixed precision, and estimator/planner
device separation are also deferred until measurements justify their added
complexity.
