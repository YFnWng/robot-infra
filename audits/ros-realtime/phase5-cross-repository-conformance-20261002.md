# Phase 5 cross-repository conformance — 2026-10-02

## Outcome

P5.6 passed for the selected deployment closure. The shared software baseline
is `catheter-stack-phase5-qualified-20261002`; the frozen numerical reference
remains `catheter-stack-pre-cleanup-20261001`. This is an offline,
non-actuating software qualification and does not expand the historical
hardware qualification scope.

## Covered behavior

The packaged `phase5_conformance_v1.json` fixture checks:

- manifest, bundle, runtime-family, API, and artifact hashes;
- initialized markers, encoder-driven state, tip trace, interface pose, distal
  strain, and downstream state;
- estimator initialization, accepted/no-op updates, gross-outlier rejection,
  health, and exact reason strings;
- delayed marker correction at observation time followed by replay to present;
- grouped, plain-with-take-up, and plain MPPI decisions using one seeded
  workload;
- response-terminated reversal preview for compensated modes and direct
  transmission for uncompensated plain MPPI.

Geometry uses 3 micrometre absolute tolerance, state fields 2e-6, command
fields 1e-6, and planner costs/effective-sample counts 1e-4. Discrete state,
validity, health, policy, and reason fields require exact equality. The harness
has no ROS publisher, serial dependency, hardware output, or encoder-zero
operation.

## Evidence

- Detached `robot-infra` commit `39a3009` and `cr_meta_lnn` commit `ae8752b`
  (the shared pre-cleanup tags) reproduced the fixture with zero failures while
  loading the same verified artifact bytes.
- Current manifest SHA-256:
  `2e9f1ccdade740fcbaae4c941eb992e15e9561f3b8b059f247182fccde9b1dd5`.
- Focused `cr_meta_lnn` deployment, manifest, bundle, packaging, interface,
  runtime, estimator, and rewind/replay tests: 78 passed.
- Focused `robot-infra` conformance, bootstrap, validation, MPPI, backlash, and
  runtime-selection tests: 117 passed.
- ROS build: `control_interface` and `catheter_control` passed.
- `cr-meta-lnn==1.0.0` wheel built and installed in `cr-venv`; wheel SHA-256:
  `608d543bded6bb4e0f2176a2c22deb35bc26d9e61ed5e6d2cf957b64a7caabc3`.
- Rebuilt installed entry point ran from `/tmp` with default installed manifest
  resolution: zero conformance failures.

The first ROS-side pytest invocation was invalid because unrelated system
`launch_testing` plugin auto-loading required `lark` in `cr-venv`. Re-running
with external pytest plugin auto-loading disabled passed all selected tests;
this did not affect runtime imports or the installed entry-point run.

## Deferred boundaries

Representative Phase 4 full-stack timing remains explicitly deferred. The C++
node remains a non-commanding shadow and has no command authority. No powered
hardware gate was run.
