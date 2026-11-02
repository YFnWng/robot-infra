# Gate 3 filter: current status

Updated 2026-11-02. This is the short navigation/status page, not a new model
specification or a claim that the research filter is deployed.

## Three paths to keep distinct

| Path | Owner / selection | Status |
| --- | --- | --- |
| Deployed control (October hardware pipeline) | [Deployment baseline](../../docs/architecture/ACTIVE_DEPLOYMENT_BASELINE.md); `cr_meta_lnn/deployment` and manifest-owned artifacts | Separate recorded-scope qualification; this research work does not change it |
| Scalar research filter | `cr_meta_lnn/training/hybrid/native_memory_marker_replay.py`, `memory_marker_runtime.py`, `interface_marker_parameter.py` | Reference for current research comparisons; retain unchanged during cleanup |
| Reduced real-time filter | `reduced_law_filter.py`, `reduced_filter_stream.py`; ROS observer `runtime_supervision/reduced_filter_shadow.py` | Research shadow on recorded replay; selected by no runtime and commanding nothing |

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
authorized. Open items: forecast error spikes (below), batched MPPI forecasting
for the reduced filter, live non-actuating observation (needs hardware
authorization), and the unqualified
[native live replacement](NATIVE_LIVE_REPLACEMENT_PROBE.md).

## Where to read next

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
- Whole-session forecast spike diagnosis:[evidence](GATE3_FORECAST_SPIKE_DIAGNOSIS.md).
- Cleanup inventory and document disposition:
  [cleanup plan, current research phase](../../docs/architecture/CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md#current-research-cleanup--2026-11-01)
  and the [2026-11-02 retention cleanup](../../docs/architecture/CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md#retention-cleanup--2026-11-02).

Do not append experiment narratives here. Update the affected current facts
and link a separate evidence record instead.
