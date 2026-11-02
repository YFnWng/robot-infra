# Architecture documents

- [Active deployment baseline](ACTIVE_DEPLOYMENT_BASELINE.md)
- [Component ownership](COMPONENT_OWNERSHIP.md)
- [Maintained and legacy index](MAINTAINED_AND_LEGACY_INDEX.md)
- [Python runtime package layout](PYTHON_RUNTIME_PACKAGE_LAYOUT.md)
- [Codebase cleanup and production migration plan](CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md)
- [Current research-filter status](../../audits/ros-realtime/GATE3_FILTER_STATUS.md)
  (separate from the deployed baseline)

The active deployment baseline is the authoritative runtime entry point.

Architecture documents describe intended ownership and migration gates. They
do not override the runtime safety requirements in the repository root README
or the active model handoff in `cr_meta_lnn/REAL_HARDWARE_CONTROL_HANDOFF.md`.

## What the repositories retain

After the 2026-11-02 cleanup, `robot-infra` and `cr_meta_lnn` keep three code paths:

1. **October hardware pipeline**: the engagement-filter/MPPI controller that
   recorded the October real sessions. It is the ROS packages and firmware in
   `robot-infra`, plus `cr_meta_lnn/deployment` and its imports. See the
   [active baseline](ACTIVE_DEPLOYMENT_BASELINE.md), the
   [parallel engagement deployment plan](../research/PARALLEL_ENGAGEMENT_MODEL_DEPLOYMENT_PLAN.md)
   and `cr_meta_lnn/EXPERIMENTAL_ENGAGEMENT_RUNTIME.md`.
2. **Scalar research estimator**: the reference for estimator comparisons. See the
   [filter status](../../audits/ros-realtime/GATE3_FILTER_STATUS.md) and
   [pipeline reference](../../audits/ros-realtime/GATE3_FILTER_PIPELINE_REFERENCE.md).
3. **Reduced real-time estimator** and its ROS shadow observer. See the
   [specification](../../audits/ros-realtime/GATE3_REALTIME_FILTER_SPEC.md) and
   [shadow evidence](../../audits/ros-realtime/GATE3_REDUCED_FILTER_SHADOW.md).

Open issues: [forecast spikes](../../audits/ros-realtime/GATE3_FORECAST_SPIKE_DIAGNOSIS.md)
and the unqualified [native live replacement](../../audits/ros-realtime/NATIVE_LIVE_REPLACEMENT_PROBE.md).

## Archived material

Legacy research code, superseded plans, experiment records and historical audits
are outside both repositories, with exact bytes and hashes:

- [2026-11-02 archive](../../../repository_archives/cleanup_20261102/README.md):
  1,790 files, including the array-native candidate and its plan.
- [2026-11-01 archive](../../../repository_archives/cleanup_20261101/README.md):
  the pre-extraction pipeline reference, earlier plans and relocated scripts.

Archived documents describe earlier experiments; they do not override the
documents listed above.
