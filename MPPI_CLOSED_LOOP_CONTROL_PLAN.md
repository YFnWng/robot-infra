# MPPI closed-loop catheter control plan

Date: 2026-09-08

> **Model baseline superseded:** the v150 model portions of this document were
> superseded on 2026-09-10 by
> `V171_ADAPTIVE_JACOBIAN_ROS_MIGRATION_PLAN.md`. Hardware, manager, watchdog,
> limit-projection, command-output-interlock, and never-`SET_ZERO` requirements
> remain applicable.

## Scope and fixed decisions

The controller will close the loop from four ZED2-triangulated catheter
markers to the first three catheter actuators using the selected v150
chart-only compatibility model. The existing ROS manager and firmware remain
the safety authority. MPPI publishes six-axis `JOINT_VEL` intent through
`/teleop/control`; the three sheath commands remain zero during initial work.

The encoder zero currently installed on the robot is the same zero used by the
training data. It is a fixed calibration invariant:

- never send `SET_ZERO` during controller startup, shutdown, recovery, or use;
- do not estimate or silently replace the encoder zero online;
- refuse controller arming if the encoder/model coordinate convention cannot
  be verified;
- use raw `ENC` counts, converted with `0.045 deg/count`, as model inputs;
- never substitute nominal `/manager/state.joint_pos` for raw motor angles.

The production manager and low-level ROS device service both reject `SET_ZERO`
unconditionally. The firmware parser retains its legacy predicate for protocol
compatibility, but this ROS stack has no supported path that transmits it.

Selected checkpoints:

- `cr_meta_lnn/checkpoints/real_joint_chart_distal_v150_chart_chart.pt`
- `cr_meta_lnn/checkpoints/real_joint_chart_distal_v150_chart_distal.pt`

The model is a short-horizon predictive prior. Initial MPPI horizons must be
0.10--0.25 s, with visual correction at every accepted camera update.

## Target architecture

```text
ZED2 cameras
    |
    v
/shape_tracking/markers --> timestamp-aware marker estimator
                                  |
/device/state (ENC/POS) ------> streaming learned model
                                  |
                                  v
                          catheter-specific MPPI
                                  |
                                  v
/teleop/control --> ControlManager --> Teensy --> motor drivers
```

ROS callbacks and command/watchdog publication must not be blocked by GPU
optimization. A missed planning deadline produces a zero/hold command, never
a late command.

## Phase 0: hardware/model contract

Status: implemented in
`src/catheter_control/catheter_control/safety/hardware_contract.py`.

One shared implementation owns:

1. raw encoder count to motor-shaft radians;
2. the manager's position, velocity, and reliable-speed constraints;
3. logical-joint to physical motor-axis coupling;
4. firmware RPM quantization and the resulting motor-shaft velocity;
5. integration of candidate motor angles over an explicit positive `dt`.

The MPPI sampler must evaluate the projected/quantized result returned by this
module, and the ROS node must publish the corresponding projected logical
command. Tests mirror the manager and firmware behavior.

Before Phase 1 hardware use, verify that `robot_base` in marker messages is the
same Cartesian frame used by the learned model. A mismatch is a startup
failure, not a warning.

The online camera prerequisite is implemented in
`../catheter-shape-tracking/HD720_REGISTRATION_AND_QUALITY.md`. It uses the
existing dual-ZED ChArUco registration pipeline with the dedicated
`camera_config_hd720.yaml` profile, then runs the exact online marker path as a
finite, machine-readable quality gate. Closed-loop arming requires a passing
HD720 report from the current physical camera/board setup.

## Phase 1: streaming learned-model runtime

Status: minimal demo runtime implemented in
`../cr_meta_lnn/deployment/streaming_runtime.py`.

For the first closed-loop demo, Phase 1 is intentionally limited to the frozen
v150 model's causal state, raw-encoder advancement, clone-isolated batched
rollouts, four-marker prediction, variable time steps, and automatic
subdivision above 40 ms. It does not load an HDF5 dataset or replay `_load_chart`
at runtime. Marker observations are retained but cannot yet overwrite physical
memory. The Phase 2 estimator remains the sole owner of visual correction.

On this machine, a CPU smoke benchmark for a six-step rollout measured about
22 ms for 64 samples, 30 ms for 256 samples, and 56 ms for 1024 samples. The
quick demo should therefore begin with 128--256 samples and six 25 ms steps;
GPU benchmarking can wait until the ROS process environment is finalized.

Build a deployment module in `cr_meta_lnn` exposing:

```python
initialize(timestamp, encoder_counts, markers=None)
advance_encoder(timestamp, encoder_counts)
observe_markers(timestamp, points, quality)
clone_state()
predict_sequence(state, motor_velocity_sequence, dt_sequence)
```

Its explicit state owns insertion play, handle/post-backlash rotation,
transmitted rotation and insertion activity, chart-relative pose correction,
distal PCS strain, tendon-base compatibility state, the complete v74 history
state, timestamps, and estimator covariance/health.

Requirements:

- remove the real-time dependency on offline `_load_chart` replay;
- preserve chart transport and every recurrent transition exactly;
- accept batch-leading dimensions and explicit variable time steps;
- subdivide unusually large intervals without resetting recurrence;
- keep sampled rollouts isolated from the live state;
- verify streaming-versus-batch equivalence on both recorded sessions,
  including holds, dropped frames, and reversals.

## Phase 2: causal four-marker estimator

Status: minimal quick-demo estimator implemented inside the deployment runtime.
It corrects chart-relative pose and four smooth bending coefficients using
quality-weighted, bounded, damped Gauss--Newton updates. It includes gross
outlier, normalized-innovation, rank, improvement, and timestamp gates plus
`INITIALIZING`, `TRACKING`, `DEGRADED`, and `STALE` status. Five accepted
observations are required for `TRACKING`.

To keep the first demonstration small, marker observations more than 60 ms
behind the current encoder state are rejected. The ring-buffer rewind/replay
design below remains required before broader or faster operation, but is not
needed for a conservative 30 Hz visual-feedback demo when encoder processing
is synchronized closely to each marker callback.

Convert the existing marker Gauss--Newton diagnostic into a bounded iterated
EKF or equivalent error-state filter. Estimate the chart-relative pose and a
low-dimensional observable distal bending subspace. Do not independently
estimate all 24 strains from 12 Cartesian measurements, and never overwrite
backlash or v74 physical memory from a camera frame.

Use marker confidence, reprojection error, and source-rig count in measurement
covariance. Add innovation/NIS gating, bounded corrections, covariance
inflation after rejection/reversal, and explicit `INITIALIZING`, `TRACKING`,
`DEGRADED`, and `STALE` states.

Camera observations must be applied at image-acquisition time. Keep a short
ring buffer of encoder inputs and model snapshots, update the historical state,
then replay encoders to the present. Never apply a delayed observation
directly to the newest state.

Initialization uses a stationary 1--2 s marker support interval while motor
history propagation is already running. Motion remains inhibited until
innovations and covariance are stable.

## Phase 3: catheter-specific MPPI

**Minimal implementation status (2026-09-08): implemented offline.** The
`catheter_control.planning.mppi.CatheterMppi` core uses six correlated 40 ms knots,
includes zero and warm-start candidates, evaluates the Phase 0 projection and
firmware RPM quantization at every rollout step, and returns zero on rollout,
finite-value, or 60 ms deadline failure. It includes tip, optional intermediate
marker, effort, slew, reversal, projection, and boundary costs. ROS command
publication and lifecycle gating remain Phase 4 work. Estimator covariance and
learned out-of-distribution scoring remain deferred until those signals exist;
they must be added before expanding beyond the guarded demonstration.
The information-theoretic update averages the projected feasible samples (the
catheter equivalent of the existing controller's `g(v)` output), not the raw
Gaussian requests.

Retain the existing information-theoretic sampling and weighting ideas, but
replace its stateless `WorldModel.predict_batch(x, u)` assumption with a
structured rollout backend:

```python
rollout(root_state, velocity_samples, dt_sequence) -> {
    "markers": ..., "tip": ..., "motor_angles": ..., "trust": ...
}
```

Optimize piecewise-constant physical joint velocities. Project every sample
through the Phase 0 contract before integrating candidate motor angles. Use
temporally correlated noise or short control knots and always include the zero
command and the warm-start nominal sequence.

The initial cost contains distal target tracking, intermediate-marker shape
regularization, effort, slew, reversal, boundary, estimator-uncertainty, and
out-of-distribution penalties. Disable integral reference correction until
the visual estimator is validated.

Start with 4--8 steps over 0.10--0.25 s, replan near the 30 Hz camera rate,
publish the command heartbeat at 100 Hz, and execute only the first action.

## Phase 4: ROS integration and safety

**Minimal implementation status (2026-09-08): implemented, not hardware
validated.** `catheter_control.node` provides explicit arm/disarm and emergency
stop services, fresh-input lifecycle gating, a 15 Hz planning worker, a 100 Hz
command heartbeat, a dedicated `catheter_mppi` priority source, target and
prediction topics, diagnostics, and optional rosbag recording to the external
session drive. It refuses coexistence with the collection node. Faults publish
zero, release manager mode, latch `FAULTED`, and require explicit re-arming.
The manager now emits a 5 Hz readiness heartbeat so freshness can be enforced.
This minimal phase does not automatically qualify/start motors, estimate OOD,
or claim hardware validation.

Add a `catheter_control` ROS node with subscriptions to raw `/device/state`,
markers and marker diagnostics, `/manager/safety_status`, and manager events.
Publish six-axis intent to `/teleop/control`, mode/stop events to
`/teleop/event`, and controller/estimator diagnostics.

Use a dedicated priority source such as `catheter_mppi`; do not share the data
collection node's source identity. Collection and MPPI launch files must be
mutually exclusive.

Lifecycle:

```text
DISARMED -> WAITING_FOR_MANAGER -> INITIALIZING_ESTIMATOR
         -> READY -> ACTIVE -> DEGRADED/HOLD -> FAULTED
```

`ACTIVE` requires `MANAGER_READY`, fresh POS and ENC, fresh accepted markers,
completed initialization, verified frame/calibration invariants, and a passed
inference deadline check. Any fault, stale input, invalid timestamp, repeated
innovation rejection, NaN, out-of-distribution state, or compute overrun
commands zero and releases control. Recovery requires explicit re-arming.

Extend rosbag recording with markers/status, pre- and post-projection commands,
estimator diagnostics, predicted trajectories, recurrence summaries, timing,
and MPPI statistics.

## Phase 5: validation ladder

Minimal implementation status: an offline, non-actuating preflight now covers
streaming/batched equivalence, variable time steps, reversal and clone checks,
synthetic causal estimator correction, scalar/batched hardware-projection
parity, real-checkpoint MPPI timing/ESS, and injected failure-to-zero behavior.
It writes a machine-readable report outside the repository. The ROS launch has
an explicit `command_output_enabled` interlock that defaults to false. These
checks do **not** set `hardware_qualified`; recorded replay, power-off watchdog
testing, and powered gates below remain explicit operator-run steps.

1. Streaming/batch, batched/scalar, variable-`dt`, reversal, and clone tests.
2. Causal recorded replay with no future observations.
3. Controller-in-the-loop replay with command projection and injected model
   residuals.
4. ROS dry run with motor-driver power off and process-kill watchdog tests.
5. Powered static hold.
6. Tiny single-axis insertion, rotation, and bending tests.
7. Conservative single-axis reversals.
8. Small Cartesian point regulation.
9. Slow short Cartesian trajectories.

At every gate report marker/tip RMSE, maximum error, response-direction cosine,
signed gain, rejected-frame fraction, saturation/reversal counts, MPPI
effective sample size, estimator NIS, and p95/p99 compute latency.

The baseline is ready for broader experiments only after streaming equivalence,
causal estimator improvement, planning deadlines, command-projection parity,
failure-to-zero tests, and consistent small-step response direction all pass.
The online GP residual is deferred until these baseline gates succeed.
