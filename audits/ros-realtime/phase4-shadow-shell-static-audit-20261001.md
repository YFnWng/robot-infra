# Phase 4 shadow-shell static audit — 2026-10-01

## Scope and conclusion

This audit covers the initial, non-actuating Phase 4 implementation in
`control_cpp`, the versioned worker messages in `control_interface`, the Python
reference adapter, and opt-in hardware bringup wiring. It does not qualify C++
command authority and does not include representative dual-camera/GPU/hardware
load.

The implemented graph is fail-silent with respect to the robot: the C++ node
has no manager, teleoperation, or device-command publisher. Python remains the
only possible controller authority, and its existing `command_output_enabled`
interlock is unchanged.

## Observed implementation facts

- `control_shadow` publishes only shadow request, timing, diagnostic,
  parameter-event, and rosout topics.
- `control_shadow_worker` publishes only `ControlShadowDecision`, parameter
  events, and rosout.
- Bringup defaults `start_cpp_shadow` and `start_shadow_worker` to `false`.
- Starting the worker without the C++ shadow is rejected during launch setup.
- The C++ shell assigns a process epoch and monotonically increasing request,
  device, marker, manager, target, and heartbeat sequences.
- A decision is accepted only when schema, epoch, retained request, input
  watermarks, target revision, monotonic computation interval, age,
  dimensionality, finiteness, and worker validity pass.
- The Python adapter retains only the newest request and returns a result only
  after a later Python planning timing/control pair. It therefore cannot label
  a cached pre-request plan with a newer request watermark.
- Heartbeat, input, request/result, and diagnostics each use a separate
  mutually-exclusive callback group and dedicated single-threaded executor.
- A domain-isolated smoke graph showed no `/teleop/control`,
  `/manager/control`, or device-command publisher. With no reference plan, the
  worker failed closed and the shell reported `SHADOW_WAITING`. A paired synthetic timing/plan result was then accepted with matching
  epoch, request, watermarks, and target revision; it expired normally after
  the configured 200 ms shadow age limit.

## Contract verification

A shared CSV fixture is consumed by both Python and C++ tests. It covers the
accepted case and schema mismatch, unknown/stale request, epoch mismatch,
target mismatch, watermark mismatch, invalid worker result, invalid compute
time, future result, stale result, and non-finite velocity. Focused results:

- Python contract, worker, and launch-boundary tests: 14 passed.
- C++ shared-contract tests: 2 passed; colcon reported no failures.
- ROS packages `control_interface`, `control_cpp`, `catheter_control`, and
  `bringup` build successfully with ROS 2 Humble.
- Focused flake8 and pep257 checks pass for every new or modified Python file.
- The aggregate `control_interface` package test still reports 23 flake8 and
  14 pep257 errors in pre-existing manager, serial, launch, and freshness
  sources; none are in the Phase 4 changes. Its CMake and XML lint pass.

## Timing topology

The 100 Hz C++ heartbeat trace is isolated from Python model/planner work. The
request/result executor never waits for the worker: missing or expired results
remain non-fresh and produce diagnostics only. The initial shell is shadow-only,
so it intentionally does not publish zero or nonzero robot commands.

No claim is made yet about P50/P95/P99 or deadline improvement. Those values
must be measured under the same dual-camera, estimator, GPU planner, manager,
serial, recording, and diagnostics load as the frozen Python baseline.

## Remaining qualification gates

1. Freeze behavior-level replay fixtures for controller lifecycle transitions,
   delayed marker rewind/replay, take-up, planner timeout, and faults.
2. Compare Python and C++ lifecycle states and exact reason codes over those
   fixtures.
3. Inject missing, duplicate, reordered, future, stale, and non-finite worker
   traffic in ROS-level tests, beyond the pure contract tests.
4. Run grouped, plain-with-takeup, and plain simulation comparisons.
5. Measure full-stack timing distributions and deadline misses under
   representative load.
6. Run hardware shadow observation only after explicit authorization.
7. Treat any command-authority promotion as a separate reviewed change.

## Evidence classification

The graph surface, source contracts, builds, unit tests, and isolated smoke run
are observed facts. Full-stack timing improvement and behavioral equivalence
are unverified hypotheses until the remaining qualification gates are run.
