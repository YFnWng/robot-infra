# Gate 3 filter: current status

Updated 2026-11-02. This is the short navigation/status page, not a new model
specification or a claim that the research filter is deployed.

## Three paths to keep distinct

| Path | Owner / selection | Status |
| --- | --- | --- |
| Deployed control (October hardware pipeline) | [Deployment baseline](../../docs/architecture/ACTIVE_DEPLOYMENT_BASELINE.md); `cr_meta_lnn/deployment` and manifest-owned artifacts | Separate recorded-scope qualification; this research work does not change it |
| Scalar research filter | `cr_meta_lnn/training/hybrid/native_memory_marker_replay.py`, `memory_marker_runtime.py`, `interface_marker_parameter.py` | Reference for current research comparisons; retain unchanged during cleanup |
| Reduced real-time filter | `reduced_law_filter.py`, `reduced_filter_stream.py`; ROS observer `runtime_supervision/reduced_filter_shadow.py`; non-commanding dry-run controller `catheter_control/reduced_filter_dry_run.py` (stack `hardware_reduced_filter_dry_run_no_rotation`) | Research. Dry-run planning verified on recorded replay only (filter bit-identical to the observer); commands nothing; not live-qualified ([audit](REALTIME_DEPLOYMENT_GAP_AUDIT.md)) |

The array-native candidate (`batched_interface_step.py` and its plan) was
superseded by the reduced filter and archived on 2026-11-02. Its shared batched
helpers that the scalar filter imports remain.

## Current research components

- Distal deformation, visit-local profile and tendon engagement:
  `distal_deformation_runtime.py`, `soft_deformation_episode.py`,
  `single_soft_engagement.py`, `spline_profile_gaussian.py`.
- Curve observation/registration: `curve_marker_likelihood.py`,
  `curve_registration.py`; M0 material station is anchored, not raw M0 XYZ.
- Tip/interface response and shared chassis/knob column:
  `tip_body_jacobian.py`, `interface_marker_parameter.py`.
- One positive physical knob scalar: `positive_knob_scale.py`,
  `recursive_knob_scale.py`; explicit log-coordinate comparison remains.
- Chassis scalar-memory distribution: `scalar_chassis_memory.py`,
  `memory_marker_runtime.py`; fixed-width play, not online width learning.
- Refined interface evidence: `interface_readout_transport.py`,
  `interface_pose_refinement.py`; original fitting/derivative owner retained.

Paths above are relative to `cr_meta_lnn/training/hybrid/`. This is an owner
map, NOT a complete dependency list. Older-looking helpers remain protected
when imported by these owners, saved fixtures or regression tests.

## Preserved contracts

One authoritative distal lambda/profile belief. Parameter learning uses aligned
measured actuator motion and observed deformation, not forecast observations.
Learned J is engaged J; takeup must not teach a zero engaged column. Future
rollouts are private branches, never parameter-learning evidence. Full retained
covariances and correlations are part of the behavior-preservation contract.
ROS command authority and safety gates are outside this research change.

## Remaining work

Scalar remains the research reference; no new production/default selection is
authorized. **Grid dependency, 2026-11-02:** the distal owner accepted only
tendon inputs on the recording-specific profile grid. Repair R1 lets spline
profiles take any input inside the support, and it is bitwise identical on the
recorded session ([evidence](GATE3_BATCHED_FORECAST_PLAN.md#9-r1-results-spline-profiles-accept-any-supported-tendon-input-2026-11-02)).
The real-time path now continues the profile linearly 0.5 mm beyond each end of
this recording's range, [0.000893, 10.766078] mm. That covers homing and
upper-limit overshoot without learning there
([evidence](GATE3_BATCHED_FORECAST_PLAN.md#11-support-extension-implemented-2026-11-02)).
Commands beyond 11.27 mm still fault the filter. Full-range data (0–15 mm
joint limits plus margin) will be collected after real-time verification.
Batched MPPI forecast: 518×4 candidates in P99 3.8 ms (float32 CUDA graph),
equal to the exact forecast within 4.3e-14 mm in float64; isolated timing, not
live ([evidence](GATE3_BATCHED_FORECAST_PLAN.md#14-step-3b-gpu-implemented-2026-11-02)).
Planner adapter (3d) works offline: the actual CatheterMppi with this backend
gave 102/102 valid plans on two segments, planner P99 about 22 ms
([evidence](GATE3_BATCHED_FORECAST_PLAN.md#15-steps-3d-and-3e-2026-11-02)).
The October command dispatch is sound: 100 Hz frames, 1 of 8,133 dropped,
motors track dispatched velocity with a 15 ms lag and gain 0.97–0.99. The
earlier 0.71 tendon ratio came from the old planner stopping the tendon
mid-action and from a 47 ms tendon start-up delay, which costs about 0.1 mm
per start ([check](GATE3_BATCHED_FORECAST_PLAN.md#16-dispatch-check-of-the-october-command-path-2026-11-02)).
The hardware characterization ran on 2026-11-02, loaded and unloaded
([results](TENDON_MOTOR_START_TEST_PLAN.md#8-results-2026-11-02-both-hardware-runs)).
The tendon start delay is not caused by load or an acceleration limit; it comes
from the start path. About 19 ms is the firmware's re-initializing start
sequence and about 50–66 ms is the motor-2 driver after enable. The result is
an equivalent dead time of about 42–55 ms per start from rest, nearly
independent of load and history. The firmware also restarts any running motor
when another starts, so a tendon lead makes coupled starts worse. No remedy is
chosen yet.
**Axis prior, 2026-11-02 (finding, not acted on):** the engaged chassis-only response on the Oct 8 session
points 10.1° from the world axial-direction prior (std about 2.9°); all four markers agree
([evidence](REALTIME_DEPLOYMENT_GAP_AUDIT.md#pre-lab-tools-2026-11-02)). Any change is a separate approved experiment.
Other open items: forecast error spikes (below), batched MPPI forecasting integration
for the reduced filter, live non-actuating observation (needs hardware
authorization), and the unqualified
[native live replacement](NATIVE_LIVE_REPLACEMENT_PROBE.md).

## Where to read next

- What is still missing for live deployment, with decisions and an ordered
  offline plan: [deployment gap audit](REALTIME_DEPLOYMENT_GAP_AUDIT.md).

- Current math and information flow: [pipeline reference](GATE3_FILTER_PIPELINE_REFERENCE.md).
- Archived evidence: the
  [array-native plan](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/GATE3_ARRAY_NATIVE_IMAGE_STEP_PLAN.md)
  and the memory-cell grouping measurement with the physical-gain positivity
  repair ([evidence](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/GATE3_CELL_GROUPING_MEASUREMENT.md)).
- Real-time estimator redesign,research shadow in `reduced_law_filter.py`,selected
  by no runtime;isolated replay only,no live or full-session qualification:
  [specification and results](GATE3_REALTIME_FILTER_SPEC.md).
- ROS shadow observer,recorded replay only,no planner or hardware:
  [evidence](GATE3_REDUCED_FILTER_SHADOW.md).
- Batched MPPI forecast, proposal awaiting approval:
  [plan](GATE3_BATCHED_FORECAST_PLAN.md).
- Whole-session forecast spike diagnosis:[evidence](GATE3_FORECAST_SPIKE_DIAGNOSIS.md).
- Cleanup inventory and document disposition:
  [cleanup plan, current research phase](../../docs/architecture/CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md#current-research-cleanup--2026-11-01)
  and the [2026-11-02 retention cleanup](../../docs/architecture/CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md#retention-cleanup--2026-11-02).

Do not append experiment narratives here. Update the affected current facts
and link a separate evidence record instead.
