# Catheter control

This ROS package contains the hardware contract (Phase 0), offline controller
core (minimal Phase 3), guarded ROS integration (minimal Phase 4), and a
non-actuating validation preflight (minimal Phase 5).

Both installed executables automatically re-exec in the shared Python 3.10
environment at `/home/chen-lab/Yifan/cr-venv` before importing NumPy or
PyTorch. This avoids mixing the ROS build interpreter with the learned-model
environment. Set `CR_VENV=/another/python310/venv` before launch to override
the location. Manual activation and `PYTHONPATH` modification are not required
for `catheter_mppi` or `phase5_preflight`.

`CatheterMppi` accepts a cloned `V171StreamingCatheterRuntime` state, the current
six-axis logical joint position, and a target tip position in the calibrated
base frame. It samples four correlated 40 ms velocity knots, inserts explicit
zero and warm-start candidates, and projects every step through manager limits,
firmware coupling, and integer-RPM quantization before the model rollout.
When a response-confirmed direction lease exists, the batch is stratified over
the Cartesian product of two proposal modes per shaft: unconstrained (U) or
continue/zero (C). Mode winners are complete scored trajectories; the planner
never zeros or splices a chosen plan after rollout. The qualified grouped hardware stack uses one 512-sample CUDA batch over four
controller steps; point-reaching rollouts use the reviewed 0.20 s coarse
step. The executable-plan budget remains 60 ms, below the 66.7 ms planner period. Only
the first six-axis logical velocity in a valid result is intended for eventual
execution; axes 4--6 are always zero in this minimal controller.

```python
from catheter_control import CatheterMppi, load_hardware_contract

contract = load_hardware_contract(
    "robot-infra/src/automation/config/catheter_limits.yaml", "imricor_test")
controller = CatheterMppi(runtime, contract)
plan = controller.plan(
    runtime.clone_state(), joint_position,
    target_tip_base_m=[x, y, z],
    target_markers_base_m=optional_four_marker_reference)
if plan.valid:
    first_velocity = plan.command_logical_velocity
```

The planner fails closed to a six-axis zero command on rollout errors,
non-finite predictions/costs, or a 60 ms planning deadline miss. The ROS
wrapper discards an isolated overdue plan, publishes zero for that cycle, and
allows the next plan to recover; three consecutive deadline misses latch a
fault. Other planner failures latch immediately. The planner core has no
serial or ROS output. The ROS wrapper has no encoder-zero event or service.
The runtime reports estimator covariance, the physical Jacobian, RLS
covariance bounds, adaptation weight/update size, tendon lambda, and rewind
depth through `/catheter_mppi/status`.

Marker correction is selectable at process startup with
`marker_estimator:=gauss_newton`, `marker_estimator:=ekf`, or
`marker_estimator:=ukf`. The established Gauss--Newton implementation remains
the default. The selected backend is forwarded to the learned runtime and
reported as `marker_estimator` in `/catheter_mppi/status`; changing it requires
restarting the node. Compare EKF and UKF in non-actuating shadow runs before
considering either for command-producing operation.
Both filters keep covariance in the normalized 10-dimensional tangent state
(six interface-pose errors and four smooth bending-mode errors). Their initial
covariance and approximate random-walk process noise are exposed as
`estimator_filter_initial_covariance` and
`estimator_filter_process_std_sqrt_s`. These parameters require recorded and
live shadow calibration; they do not affect the Gauss--Newton path. EKF and
UKF updates now identify the local observable subspace, pin the instantaneous
unobservable gauge at the covariance floor, and inject process noise only in
the last accepted observable subspace. This prevents the known rank-nine gauge
from accumulating isotropic random-walk variance, but it is not a substitute
for live covariance calibration.

Accepted UKF strain corrections are also reconciled with the hidden learned
tendon history. The runtime projects the strain correction onto the scalar
tendon equilibrium coordinate and carries only that correction-induced shift
through rewind/replay and future rollouts. This prevents a corrected distal
shape from immediately relaxing toward stale history on the next zero-command
prediction. Diagnostics expose `model_history_equilibrium_correction` and
`model_history_reconciliation_last_delta`.

Reviewed semantic configurations are selected with
`stack_config:=hardware_grouped_no_rotation_farther_tendon_12`. The stack
resolves controller, platform, and performance layers in a fixed order, rejects
unknown or multiply-owned parameters, and records the complete result in the
session manifest. See [`config/README.md`](config/README.md). Legacy replay
profiles can still be applied with
`controller_config:=/absolute/path/to/profile.yaml`. The causal-v2 calibration
is installed as `config/causal_v2_shadow.yaml`; it deliberately keeps both
hardware command output and adaptation disabled. Reversal holdoff can be set
independently for physical shafts 0, 1, and 2. A negative per-shaft value uses
the legacy scalar `adaptation_reversal_holdoff_normalized_action` fallback.

The first UKF marker update does not assume that the interface material frame
is aligned with `robot_base`, nor does it load a recorded initial transform.
Marker 0 and the proximal centerline tangent establish the interface origin and
z axis. The runtime then evaluates `estimator_initial_roll_hypotheses`
material-roll hypotheses with the loaded distal model and selects the UKF
posterior with the lowest innovation NIS. Normal single-posterior UKF tracking
continues afterward. The selected roll hypothesis and its innovation NIS are
reported in controller diagnostics.

`marker_nis` is retained for compatibility and is the post-correction mean
squared residual normalized by marker sigma. The conventional pre-update
innovation statistic is reported separately as `marker_innovation_nis`, with
`marker_innovation_nis_per_dof` and `marker_innovation_dof`. Covariance
eigenvalue bounds and the current observable rank are also in status.

The limit projection is MPPI's sampling transform, analogous to `g(v)` in the
existing controller: every raw sample is projected before rollout, costs are
computed on projected controls, and the weighted nominal update averages those
projected controls rather than the raw Gaussian requests. The resulting first
action is projected once more because the minimum-speed deadband is non-convex.
MPPI commands remain post-take-up commands. For each proposal, the estimator's
directional take-up width (or active remaining travel) is converted from shaft
radians into coupled logical joint displacement and reserved before horizon
limit projection. Thus a useful-motion plan cannot appear feasible by ignoring
the encoder travel consumed before engagement. The downstream transaction
arbiter still performs take-up, discards the pre-engagement plan, and replans
after response-confirmed engagement.

## Minimal Phase 4 ROS node

`catheter_mppi` subscribes to raw `/device/state`, calibrated four-marker
feedback and diagnostics, `/manager/safety_status`, manager events, and a
`geometry_msgs/PointStamped` target on `/catheter_mppi/target_tip`. Its target
frame must exactly equal `robot_base` unless `frame_id` is reconfigured.

Use `catheter_target_offset` to construct a target directly from marker ID 3
in the latest registered measurement:

```bash
ros2 run catheter_control catheter_target_offset \
  --dx-mm 5 --dy-mm 0 --dz-mm 0
```

The helper requires fresh marker and controller status, discovers the target
subscriber, and limits the offset norm to 10 mm by default. It refuses to
change the target while the controller is armed. For an intentional live
retarget, add `--allow-armed-retarget`; this can immediately cause hardware
motion. Use `--maximum-offset-mm N` only when a reviewed experiment requires a
different bound.

It starts disarmed. Arming is explicit:

```bash
ros2 service call /catheter_mppi/set_armed std_srvs/srv/SetBool "{data: true}"
```

Disarm with the same service and `data: false`. An emergency service also
publishes the manager's guarded motor-stop event:

```bash
ros2 service call /catheter_mppi/emergency_stop std_srvs/srv/Trigger "{}"
```

Active control requires a fresh `MANAGER_READY` heartbeat, synchronized fresh
POS and ENC feedback, at least eight accepted estimator observations, two
consecutive corrections whose maximum marker residual is inside the normal
tracking gate, fresh marker diagnostics, a valid target, and no `collection`
node or collection event. A
failure publishes zero, releases `JOINT_VEL` mode, latches `FAULTED`, and
requires explicit re-arming. The manager remains the final safety authority.
The 30 Hz marker topic and device feedback are latest-sample cached. One 50 Hz
estimator timer is the sole owner of mutable learned-runtime state: it drains
the newest encoder sample, applies at most one rate-limited marker correction
only after estimator time has reached the image timestamp, then drains the
newest encoder again. A short state/encoder ring buffer corrects at the image
timestamp and replays achieved encoder inputs to the present. The model accepts
up to 150 ms of image-timestamp lag.
The 20 Hz correction limiter uses an absolute release phase. With the 50 Hz
owner it alternates across timer ticks to average 20 Hz; late releases skip
missed phases and never create a correction backlog.
Independently, an applied correction must be
newer than 150 ms; its freshness clock starts only after the runtime validates
and commits it, so queue and optimization latency are not counted twice.
POS/ENC source messages are hardware-paired to sub-millisecond precision. The
controller permits up to the same 150 ms processing skew as its encoder
freshness bound; an older recurrence fails the existing `encoder_stale` gate.
The minimum accepted count and convergence streak are launch arguments
`initialization_observations` and `initialization_consecutive_inliers`.

The wrapper plans at 15 Hz and publishes the most recent valid first action at
100 Hz. Status, the projected planned command, and its already evaluated
candidate trajectory are published on
`/catheter_mppi/status`,
`/catheter_mppi/planned_control`, and `/catheter_mppi/predicted_tip`.
Status also reports the accepted camera tip and target-minus-observed tracking
error as `observed_tip_m`, `tip_error_xyz_mm`, and `tip_error_norm_mm`. While
ACTIVE, the same error is logged at `tip_error_log_rate_hz` (1 Hz by default;
set it to zero to disable terminal logging).
Planning reads a replace-only cloned state snapshot under a small exchange
lock, so it never waits for live marker rewind/replay. PyTorch is capped at
two intra-op and one inter-op thread by default. A concurrent planner/UKF
benchmark found no isolated-rollout benefit from four threads and substantially
larger shared-pool tails. Status also contains bounded
audit instrumentation. Keys beginning with `timing_` report the count, mean,
and maximum measured within the preceding diagnostic window for timer-start
lateness, complete timer callback duration, subscription header age,
pending-sample age, causal marker deferral, estimator-owner work, and snapshot
exchange wait/hold time.
The current marker update also reports `model_marker_timing_rewind_ms`,
`model_marker_timing_correction_ms`, `model_marker_timing_replay_ms`, and
`model_marker_timing_total_ms`. Each plan reports
`plan_sample_projection_ms`, `plan_rollout_ms`, `plan_cost_weighting_ms`, and
`plan_update_projection_ms`, allowing rare aggregate tails to be assigned to a
specific phase. With `mppi_best_candidate_guard`,
`plan_command_prediction_kind` is `scored_feasible_candidate`: execution uses
the minimum-cost member of the sampled batch while the weighted mean updates
only the next sampling nominal. This deliberately reuses the main rollout
batch; a second exact model call caused unacceptable real-time deadline
overhead. Only the first action is executable; later actions are proposals
that the next MPPI update will replace. Consequently, response
instrumentation uses the first 40 ms prediction, not the non-causal 160 ms
terminal prediction. Each forecast is matched to the first accepted camera
sample at or after that timestamp. The
`response_*` status keys then report predicted and measured tip displacement,
observed-minus-predicted endpoint error, direction cosine, timestamp alignment,
and pending forecasts. These values are diagnostic only and do not affect
estimator gates or control output.
When at least eight samples are configured, MPPI also reserves six candidates
for constant positive and negative insertion, rotation, and bending probes.
This guarantees independent actuator directions are evaluated in every plan;
the remaining candidates retain the correlated random exploration.
The windows reset after each status publication and do not participate in
safety decisions.
Before claiming the control mode, it performs one non-actuating model rollout
to initialize PyTorch's lazy CPU path, discards that result, and resets the
MPPI nominal sequence. Only subsequent plans are eligible for execution and
must meet the 60 ms deadline. Deadline accounting begins at planner callback
entry and is checked again immediately before command commit, so snapshot and
lifecycle-lock waits are included.
The sample count, horizon, rollout step, and hard planning deadline are exposed
as `samples`, `horizon_steps`, `rollout_step_s`, and
`planning_deadline_s` launch arguments. `torch_intraop_threads` and
`torch_interop_threads` expose the native compute-pool bounds.

### CUDA MPPI candidate

The deployed runtime accepts `device:=cuda` or `device:=cuda:<index>`. CUDA is
validated before model loading; an unavailable driver/device terminates startup
instead of deferring the failure to an armed plan. Controller diagnostics
report the requested/resolved device, CUDA device name, and allocated,
reserved, and peak memory. CUDA phase timing is explicitly synchronized, so
`plan_rollout_ms`, `plan_cost_weighting_ms`, and the 60 ms deadline measure
completed accelerator work rather than kernel-submission time.

MPPI uses the v171 runtime's tip-only control rollout. It preserves identical
state propagation and tip prediction while skipping marker kinematics and
stacked centerline outputs that the tip objective does not consume. The full
`predict_sequence()` contract remains available to validation and
visualization callers. Sampling and exact hardware projection remain on CPU in
this first migration.

Run the non-actuating deployed-artifact benchmark before ROS testing:

```bash
ros2 run catheter_control phase5_preflight \
  --device cuda --samples 1024 --horizon-steps 4 \
  --rollout-step-s 0.04 --timing-trials 100 --deadline-s 0.06
```

The report is written under the configured catheter session root and includes
P50/P95/P99/max total and per-phase timing. It does not qualify or command
hardware. For a full-stack hardware shadow launch, combine the selected model
profile with the non-actuating performance overlay:

```bash
ros2 launch catheter_control control.launch.py \
  controller_config:=/home/chen-lab/Yifan/robot-infra/src/catheter_control/config/causal_v2_fixed_hardware.yaml \
  performance_config:=/home/chen-lab/Yifan/robot-infra/src/catheter_control/config/gpu_mppi_1024_shadow.yaml \
  record:=true
```

Performance and controller profiles are forbidden from declaring
`command_output_enabled`; launch fails if one does. Performance overlays also
cannot be combined with enabled output. Do not enable hardware output until
the offline, simulation, and full-camera/UKF shadow timing gates in
`GPU_MPPI_MIGRATION_PLAN.md` pass. The default CPU launch and 32-sample
behavior remain unchanged.

`command_output_enabled` is reapplied as the final direct ROS parameter. This
makes the launch argument the sole output interlock: a profile can neither
silently enable commands nor mask an explicit
`command_output_enabled:=true`. Supplying any
`performance_config` together with `command_output_enabled:=true` is rejected;
performance overlays are shadow-only.

The active artifact arguments are `v171_distal_checkpoint` and
`jacobian_initialization_json`; there is no chart checkpoint in this path.
`adaptation_enabled` defaults to `false`, which retains causal accepted-pose
anchors and diagnostics but leaves the qualified v174 `J0` unchanged. Enable
bounded RLS only after reviewing a causal shadow-mode bag.
When enabled, RLS accumulates multiple accepted camera frames, requires two
actual motion intervals and an interface response above the configured
absolute/SNR floors, excludes reversal/backlash travel, rejects mixed-axis or
inconsistent responses, and constrains every update around `J0`. The relevant
`model_rls_reason`, accumulated response, confirmation, and reversal-holdoff
fields are published in status. The conservative defaults are 0.10 degree or
0.30 mm response, three-sigma EKF/UKF evidence, two consistent windows, and
physical column bounds of 0.25--4 times `J0` within 60 degrees.

Motor increments below the per-interval excitation threshold are accumulated
across accepted camera frames. This allows slow continuous motion to form the
required motion intervals without admitting static data; a window still needs
sufficient total action and interface response magnitude/SNR. The diagnostics
include `model_rls_motion_intervals`, `model_rls_normalized_action_norm`,
`model_rls_directional_purity`, `model_rls_rotation_snr`, and
`model_rls_translation_snr`.

For `imricor_test`, autonomous velocity projection retains a 1 mm insertion
reserve inside each hard endpoint. The controller re-projects every 100 Hz
heartbeat against the newest position, so a cached plan cannot continue
outward after entering that reserve. This does not alter the manager's hard
`[0, 40]` mm validity limits or fault behavior.

## Time-budgeted tip trajectories

The hardware and simulation launch files also start
`catheter_tip_trajectory`, a ROS 2 action server that sequences absolute tip
targets in `robot_base`. Each waypoint has its own wall-clock time budget. A
waypoint advances as soon as the measured marker-3 tip remains within the
requested tolerance for the settle time, or when its time budget expires.
Consequently, a difficult point cannot stall the remaining trajectory.

The server accepts one goal at a time, monitors fresh marker and controller
status throughout, and uses only the controller's guarded `set_armed` service.
With `auto_arm: true`, it arms after publishing the first target and always
disarms after success, timeout completion, cancellation, or abort. It never
bypasses manager readiness, estimator, limit, or fault gates. A completed
schedule has a ROS terminal state of `SUCCEEDED` and `result.success=true`,
including when a configured time budget advances a waypoint. Use
`reached_waypoints` and `timed_out_waypoints` to evaluate tracking quality.
Aborted or canceled schedules return `result.success=false`.

After launching the simulation, this example holds two absolute targets for up
to eight seconds each (waypoint indices in feedback are zero-based):

```bash
ros2 action send_goal --feedback \
  /sim/catheter_mppi/track_tip_trajectory \
  control_interface/action/TrackTipTrajectory \
  "{header: {frame_id: robot_base}, waypoints: [
    {x: 0.025, y: 0.015, z: 0.075},
    {x: 0.020, y: 0.020, z: 0.080}],
    waypoint_timeouts_s: [8.0, 8.0], tolerance_mm: 0.5,
    settle_time_s: 0.5, auto_arm: true}"
```

Use `/catheter_mppi/track_tip_trajectory` for the hardware launch. Confirm the
waypoints are feasible and comfortably inside joint/workspace limits before
sending a hardware goal. Setting `auto_arm: false` requires MPPI to already be
armed, but the action still disarms when it exits. Cancel an active goal with
the standard `ros2 action` client or Ctrl-C in the foreground client.

`catheter_tip_trajectory_file` loads the same goal from YAML. Generator YAML
can depend on the latest measured tip without placing filesystem access in the
action server. `circle_trajectory_sim.yaml` dynamically samples 36 points on a
10 mm-radius circle parallel to the base YZ plane and centered at
`(x0+10 mm,0,z0)`. Run it in the simulation domain with:

```bash
ros2 run catheter_control catheter_tip_trajectory_file \
  /home/chen-lab/Yifan/robot-infra/src/catheter_control/config/circle_trajectory_sim.yaml
```

The hardware-specific `circle_trajectory.yaml` fixes the measured 20 mm
insertion reference tip at
`(0.0219061542,0.0170453805,0.0713809729) m`. Its circle center is therefore
`(0.0319061542,0,0.0713809729) m`. It uses the hardware endpoints and a
1.8 mm positional tolerance. Five intermediate approach waypoints divide the
initial 12.23 mm displacement into approximately 2.04 mm target increments.
The YAML controls circle and approach waypoint counts, per-waypoint timeout,
direction, settling time, and auto-arm behavior.

### Independent sparse-circle sanity test

`catheter_sparse_point_experiment` tests eight nonduplicated points on the
same 10 mm-radius base-YZ circle without commanding motion along the circle.
It first moves all six logical joints to
`[20, 0, 0, 0, 0, 0]` through the manager's guarded position mode. The first
settled home tip freezes the absolute circle. Before every point it returns to
the same joint home, waits for position and marker settling, and submits a
single-point trajectory action. The action disarms before the next homing
transaction. A timed-out point is recorded and the experiment proceeds; a
controller fault or manager inhibition aborts the experiment.

The simulation YAML selects `plant_reset.mode: full_simulation`. After each
joint home it calls three `/sim`-only reset services: the device re-registers
its hidden transmitted coordinates to the current shaft encoders, the truth
model discards its distal recurrence, and the controller discards estimator,
backlash, scheduler, and MPPI memory. Fresh marker observations must return
the controller estimator to `TRACKING` before the target is issued. This mode
isolates controller behavior by making every point start from the same full
plant state. Set the mode to `history_preserving` to keep simulated tendon
remanence between targets.

The hardware YAML is permanently `history_preserving`: moving encoders back
to `[20,0,0]` cannot erase physical tendon remanence. No hardware reset service
is created or called, and this experiment never changes encoder zero.

For an A/B simulation using the same targets but hardware-faithful remanence,
run `sparse_circle_points_sim_history_preserving.yaml` instead of
`sparse_circle_points_sim.yaml`.

Simulation configuration:

```bash
ros2 run catheter_control catheter_sparse_point_experiment \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/sparse_circle_points_sim.yaml"
```

The hardware configuration is `sparse_circle_points_hardware.yaml`. It uses
the same geometry and home but hardware endpoints. Run it only after the
normal serial, driver-power, marker, estimator, and manager-ready preflight.
The utility never changes encoder zero and does not bypass manager limits,
freshness checks, fault latching, or firmware safeguards.

For the fixed-J axial A/B test, launch the controller with
`v174_fixed_hardware_no_rotation.yaml` and run
`axial_points_v174_hardware.yaml`.  The latter homes to `[20,0,0]` before each
independent target and freezes two targets at `+5 mm` and `-5 mm` base Z from
the first measured home tip.  This test preserves physical tendon history and
does not reset or redefine encoder zero.

For the v175 grouped-MPPI hardware isolation, use
`v175_grouped_hardware_no_rotation.yaml` with
`axial_points_v175_hardware_no_rotation.yaml`. This pair preserves the v171
distal model, v174 Jacobian initialization, fitted v175 interface transmission,
and grouped GPU planner while setting logical rotation velocity exactly to
zero. Before arming, the experiment requires controller diagnostics to report
hardware output enabled, UKF estimation, adaptation disabled, and the exact
six-axis velocity limit. While the target action is active it independently
observes both `/catheter_mppi/planned_control` and manager-forwarded velocity
commands; any non-finite or nonzero rotation command cancels the action and
releases manager mode. The guard does not apply to the preceding guarded
position return, which is allowed to restore the rotation encoder to zero. Run
the two-axis target block only after this axial regression is audited.

The follow-on two-axis block does not use hand-authored Cartesian offsets.
After each candidate's own guarded home and while disarmed,
`/catheter_mppi/generate_sparse_targets` clones the current accepted
estimator/model state and predicts that candidate's target for a fixed guarded
insertion/tendon displacement. It accounts for fitted response-free take-up
travel in the endpoint limit check, subtracts the zero-control model drift,
rejects rotation, non-finite, too-small, too-large, or reserve-violating
targets, and does not mutate estimator or MPPI warm-start state. Qualify it
first with
`two_axis_points_v175_sim_no_rotation.yaml`; hardware uses
`two_axis_points_v175_hardware_no_rotation.yaml` and stops at the first failed
point. The simulation launch must receive
`controller_velocity_max:=[10,0,4.5,4,25,25]` so its runtime identity matches
the isolation contract.

The farther follow-on profiles are
`far_two_axis_points_v175_{sim,hardware}_no_rotation.yaml`. They use logical
displacements `[8,0,0]`, `[0,0,4.5]`, `[-8,0,3]`, and `[-6,0,4.5]`, a
40-step preview, and a 3--20 mm predicted-tip information envelope. The model
preview remains the final authority on live joint reserve, so an unsafe or
uninformative candidate is rejected before the action is armed.

After that block completes on hardware, the hardware-only extended-distance
profile is `farther_two_axis_points_v175_hardware_no_rotation.yaml`. It uses
logical displacements `[10,0,0]`, `[0,0,6]`, `[-10,0,4.5]`, and
`[-8,0,6]`, retains the same 40-step model preview and endpoint reserves, and
allows 30 seconds per target. Its 4--25 mm predicted-tip envelope is only a
validation bound: the live preview still rejects any candidate that violates
the joint limits, estimated take-up allowance, or configured reserve before
the controller is armed.

The next hardware-only tendon-range extension is
`farther_tendon_12_two_axis_points_v175_hardware_no_rotation.yaml`. It keeps
the qualified insertion components at `+10`, `-10`, and `-8` mm while using
tendon components of `0`, `12`, `9`, and `12` mm. The increased 35 mm
predicted-tip validation ceiling accommodates the larger requested motion.
This exploratory profile removes the extra endpoint-reserve margin so a target
near a boundary can run; hard planner projection, manager limits, and firmware
limits remain active.

For the hardware A/B comparison against the successful grouped session
`20260929_104353_mppi_demo`, the conventional plain-MPPI implementation is
materialized in two reviewed rotation-disabled profiles:

- `v175_plain_takeup_hardware_no_rotation.yaml` disables grouped U/C
  proposals, scored-candidate execution, take-up-delay cost, and the reversal
  scheduler while retaining the v175 transmission belief and the same slow,
  response-terminated take-up transaction.
- `v171_plain_hardware_no_rotation.yaml` additionally removes the v175
  transmission artifact, backlash belief, engaged-gain scenarios, and take-up
  compensation. The v171/v174 runtime consumes raw shaft encoders and MPPI
  commands are passed directly through normal limit projection.

Their matching `farther_tendon_12_...plain...yaml` experiment profiles use
the four exact Cartesian targets recorded in the grouped session, rather than
asking each baseline's different model to generate a different task. Runtime
diagnostics must match the requested planner and compensation boundary before
the experiment can arm. Manager and firmware limits remain authoritative.

## Continuous tip paths

`catheter_tip_path` removes waypoint stop-and-settle transitions. It publishes
a timestamped Cartesian preview at 30 Hz, and MPPI resamples that preview at
the estimator snapshot time for all four rollout steps. A monotonic progress
governor slows and pauses when reference error grows, resumes with hysteresis,
and only applies a tolerance/settle condition at the final path endpoint.

The point and path inputs are mutually exclusive while armed. A stale, short,
wrong-frame, conflicting, or changed-ID path reference faults closed through
the normal controller release route. All existing manager, feedback, joint
projection, command-freshness, and firmware watchdog gates remain unchanged.
The legacy waypoint action remains available for comparison.

After launching simulation, run the current-tip-relative YZ circle with:

```bash
ros2 run catheter_control catheter_tip_path_file \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/continuous_circle_sim.yaml"
```

For the Phase-C speed comparison, restart the simulation before each run so
all three trials begin from the same plant and backlash state. The path client
accepts runtime speed and action-timeout overrides, so the geometry and all
other YAML settings stay identical:

```bash
# 1 mm/s
ros2 run catheter_control catheter_tip_path_file \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/continuous_circle_sim.yaml" \
  --speed-mm-s 1 --total-timeout-s 180

# 2 mm/s baseline
ros2 run catheter_control catheter_tip_path_file \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/continuous_circle_sim.yaml" \
  --speed-mm-s 2 --total-timeout-s 90

# 3 mm/s
ros2 run catheter_control catheter_tip_path_file \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/continuous_circle_sim.yaml" \
  --speed-mm-s 3 --total-timeout-s 75
```

The selected speed and timeout are encoded in the action goal and the nominal
speed is recorded in every `path_tracking_trace` message. These overrides do
not change the hardware YAML or controller parameters.

The full curve, governed reference point, and MPPI preview appear in RViz.
Rosbags include `reference_horizon`, `reference_path`,
`path_reference_point`, `path_tracking_trace`, and action feedback/status in
addition to the existing estimator, encoder, marker, command, and MPPI traces.
The hardware YAML is `continuous_circle_hardware.yaml`; do not use it until a
recorded simulation comparison and the normal hardware preflight pass.

### v175 interface-transmission simulation

The optional v175 artifact keeps the v171 tendon/distal branch unchanged and
replaces only the motor-to-interface take-up widths and forward Jacobian. The
controller estimates take-up from raw encoder/marker observations; simulation
truth advances a separate exact copy of the fitted play operator. Do not also
configure proximal `actuator_reversal_backlash_*` values, because that would
apply the same deadzone twice and launch rejects it.

```bash
export ROS_DOMAIN_ID=43
ros2 launch catheter_control simulation.launch.py \
  interface_transmission_checkpoint:=/home/chen-lab/Yifan/cr_meta_lnn/artifacts/deployed/20260929_175554_grouped_no_rotation/real_interface_transmission_v175.pt \
  backlash_compensation_enabled:=true \
  takeup_transaction_enabled:=true \
  mppi_variant:=grouped \
  device:=cuda truth_model_device:=cpu samples:=1024 \
  rviz:=true record:=true
```

The controller startup line must report reversal widths approximately
`[10.031975, 15.689875, 28.868269] rad`. These are full reversal windows, not
the one-sided fitted play values. Submit `continuous_circle_sim.yaml` using
the normal path-file command above.

The simulation launch permits five consecutive planner deadline misses by
default. Every missed cycle still replaces the command with zero; the larger
simulation-only count prevents a short shared-GPU UKF/MPPI contention burst
from terminating an otherwise healthy long path. The hardware launch retains
the stricter three-miss threshold.

```bash
ros2 launch catheter_control control.launch.py
```

For example, run a non-actuating UKF shadow session with:

```bash
ros2 launch catheter_control control.launch.py \
  marker_estimator:=ukf record:=true
```

Hardware output is disabled by default, including mode claims and velocity
heartbeats. This makes the default launch suitable for inspection and dry-run
work. Enabling real commands is intentionally explicit:

```bash
ros2 launch catheter_control control.launch.py \
  controller_config:=/home/chen-lab/Yifan/robot-infra/src/catheter_control/config/causal_v2_fixed_hardware.yaml \
  command_output_enabled:=true record:=true
```

Do not add `performance_config` to that actuating command. The reviewed
hardware profile already selects CUDA and 1,024 samples; performance overlays
are deliberately rejected when output is enabled.

Do not enable that switch until the offline preflight and the separate
power-off ROS/watchdog gate have passed. The controller never sends
`SET_ZERO`; the trained encoder zero is treated as read-only.

The reviewed hardware profile uses take-up velocity `[8.0, 20.0, 4.5]`.
Rotation alone is reduced from the generic/simulation default of 40 after the
`20260916_165943_mppi_demo` hardware run showed multi-millimetre motion before
visual response termination. Reversal-scheduler thresholds are unchanged so a
matched rerun can isolate the rate change.

Recording defaults to
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions`. The launch file
does not start or qualify the motor manager and does not start marker tracking;
those must already be running and healthy before arming.

Hardware control recording is enabled by default. Each bag includes controller
intent, manager-clamped commands, commands confirmed written by the serial
bridge on `/device/command_tx`, POS/ENC feedback, targets, marker observations,
manager/device state, ROS logs, and `/catheter_mppi/response_trace`. Each
response-trace row causally binds the first executable plan action to the first
accepted camera observation after its 40 ms prediction time and includes the
logical command, quantized motor-shaft rate used in rollout, joint/model state,
material interface pose, exact 6x3 Jacobian, target, and predicted/measured tip
response. A sibling `*_manifest.json` records resolved parameters, topic list,
artifact paths, sizes, and SHA-256 hashes. Use `record:=false` only for an
intentional non-recorded run.

## Minimal Phase 5 preflight

Run the real v171 checkpoint, estimator, hardware projection, and MPPI through
the non-actuating preflight:

```bash
ros2 run catheter_control phase5_preflight
```

The JSON report is written by default under
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions`. It includes
batched/scalar and variable-step rollout equivalence, clone isolation,
reversal finiteness, synthetic causal estimator improvement, scalar/batched
command-projection parity, MPPI p95/p99 latency and effective sample size, and
injected non-finite/deadline failure-to-zero checks.

A pass is reported as `offline_preflight_passed: true` while
`hardware_qualified` remains false. Recorded replay, the motor-power-off ROS
and process-kill watchdog check, and all powered response tests are listed as
`not_run`; they cannot be satisfied by model self-simulation.

## Exact-model ROS simulation

The isolated model-in-the-loop stack runs the production manager and MPPI node
against a simulated actuator plus an independent v171/v174 model runtime. It
does not launch the serial bridge or cameras, and every endpoint is remapped
under `/sim`. A non-default ROS domain and one simulation instance per domain
are enforced.

Build and launch it in terminal 1:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select control_interface catheter_control
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch catheter_control simulation.launch.py record:=true
```

The simulated manager qualifies automatically after three seconds. In terminal
2, use the same ROS domain, send an axis-separated target, and arm:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run catheter_control catheter_sim_target --dx-mm 5
ros2 service call /sim/catheter_mppi/set_armed \
  std_srvs/srv/SetBool "{data: true}"
```

Replace `--dx-mm 5` with `--dy-mm 5` or `--dz-mm 5` for independent-axis
tests. Disarm before changing or stopping the experiment:

```bash
ros2 service call /sim/catheter_mppi/set_armed \
  std_srvs/srv/SetBool "{data: false}"
```

RViz starts by default. Set `rviz:=false` for a headless run and
`record:=false` to disable recording. Bags are written below
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions`.

## Simulation robustness tests

The same launch file exposes deterministic sensor, actuator, and truth-model
perturbations. The MPPI node always retains its nominal model: Jacobian gains
modify only the independent plant runtime. Ground truth is published on
`/sim/catheter_sim/ground_truth_markers` and
`/sim/catheter_sim/ground_truth_tip`; the controller sees only the possibly
perturbed `/sim/shape_tracking/markers` stream. Requested,
manager-projected, and achieved commands are available separately as
`/sim/manager/control`, `/sim/catheter_sim/projected_control`, and
`/sim/catheter_sim/realized_control`.

Run one launch per configuration on a fresh, nondefault ROS domain. This
example injects deterministic 0.10 mm independent marker noise, 50 ms sensor
latency, and 2 ms timestamp jitter:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43
ros2 launch catheter_control simulation.launch.py \
  rviz:=false record:=true robustness_seed:=7 \
  marker_noise_std_mm:=0.10 marker_latency_ms:=50 \
  marker_timestamp_jitter_ms:=2
```

In a second terminal on the same domain, execute and score one axis-separated
trial. The command arms and disarms the simulated controller itself, writes a
`scenario_result.json` under the configured session root, returns zero only
when both tracking and safety gates pass, and never accesses hardware:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43
ros2 run catheter_control catheter_sim_scenario \
  --dx-mm 5 --duration-s 20 --label noise_latency_x
```

Exactly one of `--dx-mm`, `--dy-mm`, and `--dz-mm` must be nonzero. Repeat
with both signs and a fresh launch for each configuration. An exit code of 2
means the trial failed a tracking or safety gate; the JSON is still written so
the failure can be analyzed.

Actuator and plant-model examples are:

```bash
# 50% gain on the insertion motor plus 50 ms command delay
ros2 launch catheter_control simulation.launch.py rviz:=false record:=true \
  'actuator_gain:=[0.5,1,1,1,1,1]' \
  actuator_command_delay_s:=0.05

# Hardware-evaluation-centered truth J mismatch; controller J stays nominal
ros2 launch catheter_control simulation.launch.py rviz:=false record:=true \
  'plant_jacobian_angular_column_gain:=[0.55,0.5,1.0]' \
  'plant_jacobian_linear_column_gain:=[0.55,0.0,0.95]'
```

Other launch arguments cover common and marker-specific bias, dropout,
outlier probability/magnitude, per-axis deadband and first-order lag, and
per-axis reversal backlash. All default to the ideal plant. Effective values,
the random seed, estimator results, error integral, command energy, and the
post-disarm zero-command check are serialized in every scenario report. A
severe perturbation may legitimately fail tracking while still passing the
safety gate.

### Causal-v2 backlash trajectory simulation

The simulation defaults to the causal-v2 proximal Jacobian. The preferred
controller samples desired post-engagement motion with the original short
horizon. A persistent estimator tracks engaged direction, remaining take-up,
and confidence from raw encoder increments plus accepted UKF interface-pose
deltas. The same geometric state produces a virtual transmitted motor angle
for the learned **interface Jacobian**: raw shaft travel inside the estimated
gap is recorded but is not allowed to rotate the material frame. The calibrated
raw shaft angle remains the input to the frozen v171 transmission and tendon
history. Keeping these coordinates separate avoids shifting v171's learned
absolute motor reference while still suppressing fictitious proximal motion
during take-up.
A bounded feedforward stage applies `[8, 40, 4.5]` logical units/s
during take-up. Each reversal (and the initially unknown engagement side) gets
at most one estimated-width take-up budget; lack of visual response cannot
produce an unbounded high-speed pulse.

Geometric take-up and confidence are intentionally separate. Full take-up
speed is used only while `backlash_remaining_rad` is positive. Once that
travel is exhausted, the requested post-engagement speed resumes, while the
direction stays latched until repeated marker response confirms `ENGAGED`.
The directional widths are priors, not engagement sensors: sufficiently large,
direction-consistent UKF interface response can confirm engagement before the
nominal width is exhausted. Conversely, exhausting the nominal width does not
by itself prove engagement.

Interface-response attribution is joint across all three shafts. The observer
accumulates source-matched corrected interface pose and raw-encoder motion
across camera frames. Readiness is per axis: a shaft that has stopped below
its own inference floor is excluded instead of blocking a concurrently moving,
observable shaft. Ready axes are solved by bounded least squares using the
current interface Jacobian. A reversal resets the common evidence window.

For the tendon shaft (axis 2), marker-corrected distal bending projected onto
the v171 gauge-fixed bending mode is the primary engagement observation. This
permits engagement when the distal catheter visibly bends but the interface
body barely moves because the proximal segment is mostly retracted. Interface
response remains a fallback and remains the only source used to update the
metric backlash-width prior. The default distal bending threshold is `0.05`;
it is configurable with `backlash_minimum_distal_bending_increment` and was
selected above the stationary per-frame posterior variation in the audited
simulation session.

The trace/status fields
`backlash_inferred_transmitted_increment_rad`,
`backlash_response_evidence`, `backlash_joint_response_residual`,
`backlash_distal_bending_increment`,
`backlash_tendon_distal_response_evidence`, and
`backlash_tendon_distal_response_confirmed` expose the decision.

Physical take-up and adaptive-Jacobian holdoff are different calibrations.
The fast causal-v2 repetitions give median raw-shaft travel to a sustained
0.25 mm RMS marker response of `[8.369, 7.020, 34.858]` rad in positive motor
direction and `[6.856, 7.090, 3.587]` rad in negative motor direction. The
large shaft-2 asymmetry is retained rather than collapsed into a symmetric
dead zone. The normalized RLS holdoffs `[4.5, 6.5, 4.0]` remain adaptation
gates and are never used as feedforward widths.

```bash
export ROS_DOMAIN_ID=43
ros2 launch catheter_control simulation.launch.py \
  record:=true rviz:=true device:=cuda truth_model_device:=cpu \
  horizon_steps:=4 rollout_step_s:=0.04 samples:=32 \
  planning_deadline_s:=0.06 plan_rate_hz:=15.0 \
  backlash_compensation_enabled:=true \
  takeup_transaction_enabled:=true \
  mppi_transmission_aware_rollout:=false \
  mppi_rotation_direction_latch:=false \
  mppi_best_candidate_guard:=true \
  mppi_takeup_risk_weight:=4.0 \
  mppi_takeup_confirmation_time_s:=0.10 \
  backlash_engagement_confirmation_observations:=3 \
  backlash_provisional_rejection_observations:=2 \
  'backlash_width_positive_rad:=[8.369,7.020,34.858]' \
  'backlash_width_negative_rad:=[6.856,7.090,3.587]' \
  'backlash_takeup_velocity:=[8.0,40.0,4.5]' \
  backlash_width_learning_rate:=0.0 \
  'actuator_reversal_backlash_positive_rad:=[8.369,7.020,34.858,0,0,0]' \
  'actuator_reversal_backlash_negative_rad:=[6.856,7.090,3.587,0,0,0]' \
  actuator_initial_backlash_unengaged:=true
```

With repeated confirmation enabled, the first strongly attributed interface
response now enters `PROVISIONAL` and terminates the full-rate take-up
transaction immediately. Further consistent observations commit `ENGAGED`;
`backlash_provisional_rejection_observations` inconsistent moving observations
return the shaft to `TAKEUP`. This separates actuation handoff from width
learning and avoids driving through a nearby target during the confirmation
tail.

`device` selects the controller estimator/MPPI runtime. The independent
simulation truth/perception model uses `truth_model_device`, which defaults to
CPU. Keep the truth model on CPU for CUDA controller benchmarks; otherwise its
30 Hz inference competes for the same CUDA execution resources and makes the
simulated workload unlike the real camera path.

The simulator publishes hardware-equivalent upstream shaft ENC to the
controller and a separate downstream transmitted state to its independent
truth model. This lets the backlash estimator observe take-up travel while the
simulated catheter remains stationary. Use `circle_trajectory_sim.yaml` with
this launch. The earlier
`mppi_reversal_backlash_rad` option remains available as a conservative
long-horizon stress test, but must not be combined with feedforward
compensation.

### Grouped-versus-plain MPPI RViz video comparison

`simulation.launch.py` provides a reproducible `mppi_variant` switch. The
`grouped` profile uses U/C proposal groups, scored-candidate execution, the
configured horizon/first-step reversal costs, take-up-delay cost, and grouped
branch selection. The `plain` profile keeps the plant, estimator, compensator,
sample count, seed, path, and safety projection unchanged, but uses one
ordinary proposal population, the weighted feasible MPPI mean, zero reversal
and take-up-delay costs, and no response-clocked reversal scheduler. RViz
prints the selected profile in the scene.

Build once, then launch the grouped trial. For timing-clean comparison, leave
RViz disabled during control and render the recorded visualization afterward:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select control_interface catheter_control
source install/setup.bash
export ROS_DOMAIN_ID=43

ros2 launch catheter_control simulation.launch.py \
  mppi_variant:=grouped rviz:=false record:=true \
  device:=cuda truth_model_device:=cpu samples:=1024 mppi_seed:=17 \
  horizon_steps:=4 rollout_step_s:=0.04 \
  planning_deadline_s:=0.06 plan_rate_hz:=15.0 \
  backlash_compensation_enabled:=true \
  takeup_transaction_enabled:=true \
  backlash_engagement_confirmation_observations:=3 \
  backlash_provisional_rejection_observations:=2 \
  'backlash_width_positive_rad:=[8.369,7.020,34.858]' \
  'backlash_width_negative_rad:=[6.856,7.090,3.587]' \
  'backlash_takeup_velocity:=[8.0,40.0,4.5]' \
  backlash_width_learning_rate:=0.0 \
  'actuator_reversal_backlash_positive_rad:=[8.369,7.020,34.858,0,0,0]' \
  'actuator_reversal_backlash_negative_rad:=[6.856,7.090,3.587,0,0,0]' \
  actuator_initial_backlash_unengaged:=true
```

Submit the same circle in terminal 2:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43
ros2 run catheter_control catheter_tip_path_file \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/continuous_circle_sim.yaml" \
  --speed-mm-s 1.0
```

After the action returns, stop the launch cleanly to finalize the bag. Repeat
from a fresh launch with the same seeds and all the same arguments, changing
only:

```bash
mppi_variant:=plain
```

Use separate `session_root` directories, or note the two bag paths printed as
`simulation bag -> ...`. To render either bag, start RViz:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43
rviz2 -d \
  "$(ros2 pkg prefix catheter_control)/share/catheter_control/config/mppi_sim.rviz"
```

In another terminal start the RViz-window recorder, then replay the bag in a
third terminal:

```bash
ros2 run catheter_control catheter_rviz_record --label grouped_mppi
ros2 bag play /absolute/path/to/grouped_bag
```

When playback finishes, press `Ctrl-C` in the recorder terminal to finalize
the MP4. Repeat for the plain bag with `--label plain_mppi`. MP4 files are
written to the catheter session root. Keep RViz at the same window size and
camera view for both recordings. For a quick live recording, launch with
`rviz:=true` and run the same recorder during the action, accepting that RViz
and video encoding add observer load. The ROS bags remain the quantitative
comparison; videos should not replace trajectory-error, reversal-count,
take-up-occupancy, and deadline analysis.

### Post-take-up MPPI and take-up transaction

The reviewed hardware profile `causal_v2_fixed_hardware.yaml` separates the
optimizer from physical backlash. MPPI samples and scores desired
post-engagement motion only. Measured gap, take-up phase, prior transmission
direction, and nominal take-up duration do not alter its samples, rollout, or
cost. The old `mppi_transmission_aware_rollout` argument is accepted as a
compatibility alias for the downstream transaction, but new launches should
use `takeup_transaction_enabled`.

Take-up is a measured-response transaction on all three controlled shafts.
The pending set is predicted in motor coordinates from the proposed command,
directional widths and prior direction before first shaft motion; it does not
wait one encoder cycle for the phase label to change. Once a shaft enters
`TAKEUP`, MPPI planning is suspended. Pending shafts receive only the bounded
calibrated take-up rate until three direction-consistent UKF interface-pose
observations confirm engagement. Already engaged participating shafts are held
at zero in motor coordinates, and unrelated shafts do not block the
transaction. This barrier prevents a short-gap shaft
from executing one component of a coupled post-engagement MPPI action while a
long-gap shaft is still blocked. After all pending shafts confirm, the arbiter
commands zero, discards the pre-take-up plan, and requires a fresh MPPI rollout
on the next planning tick. The path governor freezes reference progress during
this take-up/replan barrier. Reaching zero estimated geometric gap alone does
not release the transaction. If take-up exceeds
1.5 times its calibrated directional width without a confirming response, the
estimator enters `FAILED` and an armed controller faults through the normal
manager stop path. Current-position projection remains active on every macro
command.

Take-up is also checked after the final position/coupling projection. If a
pending physical shaft is clipped or the projection moves a shaft that should
be held, the complete command is replaced by zero and the arbiter enters
`SATURATED_REPLAN`. The controller records the current encoder position and
temporarily blocks the saturated physical direction from MPPI candidate
selection. The block is tested against candidate intent before position
clipping, so a clipped bend command cannot masquerade as insertion-only
motion. It clears only after measured joint feedback restores enough margin
for an isolated command in that direction to survive final projection without
coupled leakage. Relevant status keys start with
`takeup_transaction_saturated_`, `takeup_requested_`,
`takeup_realized_`, and `planner_blocked_`.

The transmission-aware launch defaults remain disabled. The hardware profile
enables the take-up transaction and uses one risk-adjusted transaction cost;
interval width and confidence are belief inputs rather than separately tuned
MPPI penalties:

```yaml
backlash_compensation_enabled: true
takeup_transaction_enabled: true
mppi_transmission_aware_rollout: false
mppi_rotation_direction_latch: false
mppi_best_candidate_guard: true
mppi_takeup_risk_weight: 4.0
mppi_takeup_confirmation_time_s: 0.10
reversal_scheduler_enabled: true
reversal_scheduler_required_plans: 3
reversal_scheduler_minimum_absolute_cost_improvement: 5.0
reversal_scheduler_minimum_fractional_cost_improvement: 0.0
reversal_scheduler_minimum_terminal_error_improvement_mm: 0.25
reversal_scheduler_minimum_accepted_observations: 3
reversal_scheduler_cooldown_s: 1.0
backlash_engagement_confirmation_observations: 3
backlash_minimum_response_evidence: 0.5
backlash_minimum_distal_bending_increment: 0.05
```

Do not combine this route with nonzero `mppi_reversal_backlash_rad`; that
legacy approximation would count the same gap twice.

The guarded selector executes the minimum-total-cost member of the evaluated
batch, which always includes deterministic zero. The MPPI weighted feasible
mean updates only the next sampling distribution; it is not sent to the robot.
This prevents an unscored average from becoming an action that no rollout
represented and guarantees the selected sample is no costlier than hold within
the evaluated batch. Status reports the selected index, selected/zero costs,
transaction generation, active/pending masks, and transaction direction.

With grouped U/C sampling enabled, the direction lease defines the
continuation groups but is not a downstream execution gate. MPPI evaluates
complete unconstrained/continue plans, reserves their estimated take-up travel
against physical joint limits, and executes the globally lowest-cost feasible
plan without editing it. A selected reversal then enters the bounded take-up
transaction above; useful MPPI motion waits for credible response. This keeps
the optimization, mode choice, and limit accounting in one decision instead
of allowing a second threshold to replace the chosen plan with hold.
The fixed-size population is partitioned round-robin across active groups;
groups never duplicate a common prefix or discard the exploration tail. With
three active leases, 1,024 total samples provide 128 candidates per group.
Diagnostics report `mppi_samples`, `mppi_active_proposal_groups`, and
`mppi_minimum_samples_per_active_group`.

When grouped sampling is disabled, the response-clocked reversal scheduler
retains its legacy role as a per-physical-shaft direction gate downstream of
the candidate batch. MPPI still evaluates reverse candidates, but only an
approved reversal is executable.

A legacy-mode reversal is admitted per axis only when its ablation shows
sufficient tracking-cost and terminal-error benefit, the intent persists for
the configured number of fresh plans, enough accepted observations have
arrived, and the cooldown has expired. The legacy fractional-total-cost
parameter is retained for launch compatibility but defaults to zero and is not
used for admission. An approved reversal is consumed by one bounded
transaction; only the resulting credible interface response transfers the
lease. Diagnostic keys expose per-axis cost/terminal improvement, hold/zero
terminal errors, and the scheduler decision. Status keys
`reversal_mode_selector` and `reversal_scheduler_gating_active` distinguish
the grouped optimizer from the retained legacy gate.

Direction signs in this interface are physical shaft radians/s, not firmware
motor-axis units. This distinction is material for catheter bending axis 2,
whose units-per-RPM conversion is negative. Candidate-bank reversal tests and
the final hold guard therefore operate on projected physical-rate signs; an
unapproved hold is executable only when every affected projected shaft rate is
exactly zero.
