# v171 adaptive-Jacobian ROS control migration plan

Date: 2026-09-10

Status: planned; command output must remain disabled until the validation gates
in this document pass.

This plan supersedes the v150 model portions of
`MPPI_CLOSED_LOOP_CONTROL_PLAN.md`. The hardware contract, manager interface,
explicit arm/disarm lifecycle, command watchdogs, and the rule to never send
`SET_ZERO` remain in force.

## 1. Selected controller model

The active model will be the composition specified by
`cr_meta_lnn/REAL_HARDWARE_CONTROL_HANDOFF.md`:

1. a causal four-marker estimator of interface pose `g0` and distal strain
   `q`;
2. a locally adaptive forward interface Jacobian
   `delta_xi = J delta_a`;
3. the frozen standalone v171 transmission, scalar tendon history, and
   mass-free first-order distal mechanics;
4. eight-section PCS forward kinematics plus the known 14.5 mm rigid tip;
5. short-horizon, limit-projected MPPI.

Required artifacts are:

- `cr_meta_lnn/checkpoints/real_distal_first_order_v171_multistep_map_em.pt`
- `cr_meta_lnn/evaluation/real_adapj_v172_v171_posterior_states.npz`
  (offline validation and Jacobian-identification provenance only)
- `cr_meta_lnn/evaluation/real_joint_local_distal_v174.json`
  (`jacobian_fits.shaft` is the runtime Jacobian initialization)

The v150 chart is not part of the new prediction path. It must not silently
serve as a fallback if v171 loading or initialization fails. The old runtime
may remain available only through an explicitly named legacy launch for
recorded A/B comparison.

## 2. Invariants that do not change

- Encoder zero is identical to the training zero and is read-only. Never send
  `SET_ZERO`, including startup, shutdown, recovery, or testing.
- Use unwrapped motor-output shaft radians in insertion, rotation, bending
  order. Preserve multi-turn rotation.
- Do not use nominal backlash-free joint positions as model state.
- Continue projecting every MPPI sample through manager position/velocity
  limits, motor coupling, minimum reliable speed, and firmware RPM
  quantization before rollout.
- The manager and firmware remain the final safety authority.
- Hardware command output defaults to false and is never enabled by loading a
  model, restoring a state, or passing an offline test.
- A late MPPI result is never executed. Zero/hold is used for that cycle;
  repeated deadline misses latch a fault.
- Candidate rollouts never mutate the live estimator, tendon memory, or RLS
  state.

## 3. Target software boundaries

```text
/device/state ENC
      |
      v
v171 encoder transition --------------------------+
  transmission + tendon history + distal q prior  |
                                                   v
/shape_tracking/markers --> delayed visual update (g0,q)
                                |
                                +--> causal achieved delta(g0)
                                         + achieved delta(a)
                                         v
                                AdaptiveForwardJacobian RLS
                                         |
                                         | freeze J per solve
                                         v
                         immutable composed root snapshot
                                         |
                                         v
                               limit-projected MPPI
                                         |
                                         v
                       /teleop/control --> manager --> firmware
```

The model implementation belongs in `cr_meta_lnn/deployment`; the ROS node
owns scheduling, lifecycle, topic conversion, and safety. The ROS node must not
reimplement v171 physics or the RLS equations.

### Proposed interfaces

Create a v171 deployment runtime with the same high-level shape used by the
current controller:

```python
initialize(timestamp_ns, encoder_counts, initial_markers=None)
advance_encoder(timestamp_ns, encoder_counts)
observe_markers(timestamp_ns, points_base_m, quality)
clone_state()
predict_sequence(state, motor_velocity_sequence, dt_sequence)
current_markers()
diagnostics()
```

The root state must deep-copy:

- timestamp and previous three-axis unwrapped shaft vector `a`;
- material interface transform `g0`;
- 24-dimensional distal strain `q`;
- v171 downstream transmission state;
- all scalar tendon-history fields, including play, relaxation,
  previous-input, and filtered equilibrium;
- `AdaptiveForwardJacobian`, including normalized `J`, covariance `P`, fixed
  normalization scales, and hyperparameters;
- the last two accepted visual pose/action anchors needed for prequential RLS;
- estimator covariance/validity, observation metadata, and rewind history.

MPPI receives an immutable snapshot. Its rollout freezes the snapshot's `J`
for the entire solve while independently cloning the transmission, tendon,
pose, and strain state across candidates.

## 4. Implementation work packages

### Work package A — artifact loaders and exact Jacobian restoration

Add a deployment-safe v171 loader. Extract/reuse the authoritative
reconstruction logic currently reached through
`evaluate_real_adapj_v171_posterior._load_frozen_physics`, but remove all
runtime HDF5/dataset dependencies. It must:

- verify checkpoint experiment/schema identity;
- reconstruct and freeze the v171 distal model, calibrated transmission,
  gauge-fixed bending port, and complete history operator;
- validate eight PCS sections, 24 strains, natural torsion zero, and 14.5 mm
  rigid tip;
- expose artifact paths, hashes, schema version, dtype, and device in runtime
  diagnostics;
- fail startup on missing or incompatible fields instead of applying defaults.

Extend `control.adapj.AdaptiveForwardJacobian` with explicit
`state_dict()`/`from_state_dict()` (or equivalent constructor) and validation.
Load all values from `jacobian_fits.shaft` in the v174 JSON:

- `initial_normalized_jacobian`;
- `initial_rls_covariance`;
- `action_scale` and `state_scale`;
- ridge, forgetting, adaptation rate, and maximum normalized update norm.

Do not reconstruct the normalized state from only the printed physical `J`.
Reject wrong shapes, non-finite values, nonpositive scales, nonsymmetric or
non-positive covariance, and a physical-J consistency mismatch.

Deliverables:

- `cr_meta_lnn/deployment/v171_loader.py`
- serialization support and tests in `control/adapj.py` and
  `control/test_adapj.py`
- an artifact-only smoke test that requires no recorded dataset

### Work package B — clone-safe composed v171 transition

Add `cr_meta_lnn/deployment/v171_streaming_runtime.py`. Reuse state-cloning,
batch-leading-dimension, positive-variable-`dt`, and subdivision patterns from
the old runtime, but do not reuse its global chart transition.

For every live encoder step and rollout substep:

1. convert counts through the hardware calibration and v171 preprocessing;
2. obtain the three unwrapped shaft angles `a_next`;
3. compute `delta_a = a_next - a`;
4. update `g0 <- g0 Exp(J delta_a)` using the same body-twist ordering as
   v174 (`se3_log`/`se3_exp`, right multiplication);
5. advance the v171 transmission and all tendon-history states;
6. compute the causal scalar `lambda` and force `K B lambda`;
7. apply the overdamped implicit v171 strain step with measured `dt`;
8. compute marker, centerline, and tip points using PCS plus rigid tip.

Large positive intervals are subdivided; state is not reset across ordinary
camera gaps. Nonpositive/nonmonotonic timestamps fail explicitly. Rollouts use
the achieved, quantized motor rates returned by the ROS hardware contract.

Acceptance gates:

- one-step transmission, `lambda`, strain, pose, marker, and tip parity with
  the authoritative v174 functions;
- batched/scalar and variable-step/subdivision equivalence;
- no mutation of the live state or frozen root snapshot;
- reversal and multi-turn rotation tests;
- no references to the v150 chart checkpoint in the v171 runtime.

### Work package C — causal visual state estimator

Replace the v150-specific pose/strain correction with a v171 observation
update around the predicted `(g0,q)` state.

The initial implementation should be bounded, quality-weighted Gauss-Newton
or an error-state EKF. It must:

- compare four markers in Cartesian space using the v171 PCS-plus-tip model;
- update `g0` on SE(3), preserving material roll;
- correct only observable smooth strain modes, not all 24 strains
  independently;
- keep natural torsion and frozen v171 parameters fixed;
- use confidence, reprojection error, source-rig count, and cross-rig status;
- enforce gross residual, NIS, rank/conditioning, step-size, and improvement
  gates;
- expose covariance/uncertainty and `INITIALIZING`, `TRACKING`, `DEGRADED`,
  and `STALE` health;
- preserve transmission and tendon memory on rejection.

Initialization requires a stationary marker interval while encoder/history
propagation is already running. Motion and RLS adaptation remain inhibited
until at least the configured number of accepted, well-conditioned updates
stabilize the pose and strain estimate. The offline v172 posterior must never
be replayed as a hardware measurement or online prior trajectory.

Implement timestamp-correct delayed observations before command output is
eligible: keep a short ring buffer of encoder inputs and pre-update model
snapshots, correct at the image timestamp, then replay achieved encoder inputs
to the present. The current direct-to-latest observation approximation is not
adequate for the v171 production baseline.

### Work package D — causal online Jacobian adaptation

Use `control.adapj.AdaptiveForwardJacobian` directly. RLS is updated only when
two accepted visual interface poses bracket achieved shaft motion:

```text
anchor k:     accepted (g0_k, a_k, timestamp_k)
executed:     actual encoder-derived delta_a
anchor k+1:   accepted (g0_k+1, a_k+1, timestamp_k+1)
measurement:  delta_xi = Log(inv(g0_k) g0_k+1)
score with J_k, then update J for future solves
```

Use encoder-derived achieved `delta_a`, not requested MPPI velocity. The same
accepted visual posterior must update both the live state and the RLS anchor.
Never update from predicted poses or inside hypothetical candidates.

Adaptation weight is zero for rejected/stale/ill-conditioned corrections,
camera degradation, insufficient excitation, excessive estimator correction,
or pose motion below a jitter threshold. Otherwise derive a bounded weight
from visual covariance/quality. Keep the v174 update clipping exactly.

Add safety monitors for:

- non-finite or ill-conditioned `J`/`P`;
- normalized update norm and covariance trace;
- physically excessive predicted twist for the observed action increment;
- prolonged lack of excitation (diagnostic only, not forced motion);
- large prequential innovation (freeze adaptation and degrade/hold rather than
  learning through an outlier).

Reset policy must be explicit: ordinary disarm/rearm keeps the model's causal
physical state but may restore `J0` only via a separate, non-active service or
process restart. No reset occurs at MPPI windows or camera gaps. Persisted
adapted Jacobians are experimental artifacts and must never auto-load as the
new baseline without qualification.

### Work package E — MPPI backend migration

Retain the catheter-specific ROS MPPI sampler because it already applies the
production hardware projection at every sample and averages feasible controls.
Replace only its rollout backend with the v171 composed runtime. Use
`control/mppi.py` as a reference where useful, but do not lose the existing
hardware-limit transform.

Initial settings remain conservative:

- 0.10–0.25 s effective horizon; start with the current 4 x 40 ms = 0.16 s;
- 32 samples until the new model is benchmarked under simultaneous cameras,
  estimator, ROS bagging, and manager traffic;
- small perturbations inside the recorded envelope;
- zero and warm-start candidates on every solve;
- penalties for tip error, intermediate-marker shape, effort, slew,
  reversal, projection, and joint-boundary proximity.

Add estimator-uncertainty and model-validity penalties once their scales are
validated. Freeze `J` once per solve. Never adapt from sampled trajectories.
Keep the non-actuating warm-up, 60 ms executable-plan deadline, zero on an
isolated miss, and latched fault on repeated misses.

### Work package F — ROS configuration, diagnostics, and logging

Update `robot-infra/src/catheter_control` as follows:

- `node.py`: load the v171 runtime, preserve latest-sample encoder/marker
  scheduling, and make estimator/RLS updates atomic before cloning a solve
  root;
- `control.launch.py`: remove active `chart_checkpoint`; add
  `v171_distal_checkpoint` and `jacobian_initialization_json` parameters;
- `validation.py`: replace every v150 gate and configuration field with v171
  loader, transition, estimator, RLS, and rollout checks;
- `README.md`: document the new state, artifacts, lifecycle, and commands;
- rosbag topic list: add structured model-estimator diagnostics.

Keep the public target, arm, emergency-stop, planned-control, and predicted-tip
interfaces stable unless a versioned message is required. Add a structured
diagnostic stream (a small new message is preferred over unbounded JSON once
the schema stabilizes) containing at least:

- raw counts, unwrapped shaft radians, and actual `dt`;
- `g0`, 24-vector `q`, `lambda`, estimator covariance/validity;
- accepted marker positions and residual/NIS/rank;
- `J`, covariance trace/eigenvalue bounds, RLS innovation, weight, and update
  norm;
- predicted versus observed interface/marker displacement;
- pre/post-projection command, RPM realization, plan latency/ESS, and all
  saturation/fault events.

Lifecycle readiness gains explicit gates for compatible artifact schemas,
initialized visual state, valid Jacobian/covariance, fresh replayed estimator
state, and acceptable model uncertainty. Loading failure, invalid Jacobian,
NaN, estimator loss, or model-domain violation prevents mode claim.

## 5. Validation ladder and acceptance criteria

### Gate 1 — deterministic unit and artifact tests

- Existing hardware-contract, manager, lifecycle, and never-`SET_ZERO` tests
  continue to pass.
- Exact v174 Jacobian restoration and copy/serialization round-trip.
- v171 loader reconstructs without HDF5 and rejects schema drift.
- State cloning includes every history, estimator, and RLS field.
- Injected NaN, timestamp, covariance, rank, and deadline failures go to zero
  or inhibit arming as specified.

### Gate 2 — numerical equivalence to v174

Run the v171 posterior/recorded motor stream through both the authoritative
evaluation code and the deployment runtime. Compare at each native step:

- downstream coordinates and tendon `lambda`;
- interface pose under frozen `J`;
- implicit distal strain;
- markers, centerline, and rigid-tip point.

The test must cover holds, insertion, rotation, bending, reversals, dropped
interval subdivision, and multi-turn rotation. Tolerances must be stated per
field and tight enough to detect convention or state-reset errors.

### Gate 3 — causal estimator and RLS replay

- Replay only information available at each timestamp; assert that no future
  posterior sample is read.
- Inject real camera latency and dropped/rejected frames, rewind to image time,
  and replay encoders.
- Report marker/tip RMSE, pose/strain innovation, rejection rate, covariance,
  RLS update norms, and prequential interface response error.
- Verify rejected observations never change physical memory or `J`.
- Verify adapted `J` is frozen inside every MPC window.

The v174 offline metrics are reference evidence, not automatic pass thresholds
for the causal filter. Any proposed hardware threshold must be derived from
this causal replay, documented, and fail closed.

### Gate 4 — MPPI and controller-in-the-loop replay

- Batched/scalar rollout parity and clone isolation.
- Every candidate respects projected position/velocity/RPM limits at every
  substep.
- Benchmark p95/p99/max solve time under the complete live workload.
- Confirm the 0.16 s horizon stays within the validated local-model range.
- Inject estimator degradation, camera loss, model OOD, planner overruns,
  process death, and manager loss; verify zero/hold and latched-fault behavior.

### Gate 5 — ROS non-actuating hardware dry run

Run both cameras, marker tracking, manager, v171 estimator/RLS, MPPI, and bag
recording with `command_output_enabled:=false`. Require sustained readiness and
planning with no queue growth, timestamp faults, state resets, or unexplained
Jacobian drift. Review the saved bag and machine-readable qualification report.

### Gate 6 — powered progression

Only after Gates 1–5 pass and the physical emergency-stop procedure is ready:

1. powered static hold;
2. tiny insertion;
3. tiny rotation;
4. tiny bending;
5. conservative single-axis reversals;
6. small Cartesian point regulation;
7. slow short Cartesian trajectories.

At each stage compare observed and predicted marker/tip displacement,
direction cosine, signed gain, overshoot, steady-state error, Jacobian
innovation, and every safety event. Stop on wrong response direction,
unbounded adaptation, estimator loss, unexpected coupling, or timing failure.

## 6. Recommended implementation order

1. Implement and test exact artifact loaders and Jacobian restoration.
2. Implement the frozen-J v171 composed transition and prove v174 parity.
3. Add the causal visual filter with delayed rewind/replay; keep RLS frozen.
4. Add logged prequential RLS in shadow mode (`weight=0`), then enable bounded
   adaptation only after replay review.
5. Connect the new immutable root state to catheter MPPI and benchmark it.
6. Migrate ROS launch, diagnostics, rosbag recording, and Phase-5 preflight.
7. Complete the non-actuating live dry run before requesting any powered test.

This ordering isolates physics, estimation, adaptation, optimization, and ROS
scheduling failures. It also keeps the current output interlock closed until
the new model—not the retired v150 runtime—has passed its own evidence ladder.

## 7. Explicit non-goals for the first migration

- No v150 chart in the active prediction path.
- No mass or second-order proximal model.
- No neural proximal residual `phi`.
- No use of learned tendon `lambda` as the interface-J bending coordinate.
- No adaptation inside MPPI candidates.
- No replay of the offline v172 posterior as online truth.
- No long open-loop rollout or horizon beyond the validated local range.
- No automatic loading of an unqualified previously adapted Jacobian.
- No automatic motor qualification, encoder zeroing, or command-output enable.
