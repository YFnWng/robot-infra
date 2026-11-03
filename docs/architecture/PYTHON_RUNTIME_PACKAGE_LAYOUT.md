# Python runtime package layout

Phase 3 separates runtime responsibilities without changing ROS names,
messages, topics, services, actions, parameters, safety behavior, or console
commands. New code must import the canonical modules below. The temporary
root-level compatibility modules were retired after all maintained consumers
migrated to the canonical packages.

## `catheter_control`

```text
catheter_control/
├── node.py                  # thin ROS composition root and callback ownership
├── orchestration/           # timing, causal scheduling, runtime loading,
│                            # marker validation, configuration, diagnostics
├── planning/                # MPPI, path references, trajectories, tip tracking
├── transmission/            # play/backlash, engagement/gain belief, reversals
└── safety/                  # hardware contract, lifecycle gates, validation
```

The non-commanding `orchestration/shadow_worker.py` adapter is an explicit
Phase 4 migration surface: it mirrors only a post-request Python plan into the
versioned shadow contract and owns no manager or device command publisher.

`planning/engagement.py` is the non-publishing experimental tendon-belief
snapshot adapter. The composition root reuses MPPI and the existing takeup arbiter; estimator
mathematics, interface prediction and engaged rollout remain in `cr_meta_lnn`.
It is not selected by the baseline ROS node and does not own a second tendon
estimator, bootstrap coordinator or any command publisher. Experimental feature
validation belongs to `orchestration/configuration.py`, not this adapter.

`planning/adapj.py` owns measured-tip/achieved-joint alignment, initialization
evidence and the plain two-axis AdapJ decision interface. It reuses
`control.adapj.AdapJController` rather than copying its update algebra. The ROS
node retains the same lifecycle and command-publication owner; the existing
`control_tasks` homing client owns explicit calibration motion.
The compensated AdapJ mode submits intent to the existing transmission arbiter
and shares MPPI's transaction advancement and heartbeat. Parameter updates are
suppressed during the transaction, and the AdapJ owner requires a post-handoff
source-time measurement before computing again. It owns neither a separate
engagement estimator nor a command publisher.

The composition root still owns ROS callback groups, immutable planner
snapshots, estimator and planner execution, command publication, and fault
latching. Pure marker-message validation, callback timing instrumentation,
diagnostic value serialization, learned-runtime loading, and the complete ROS
parameter declaration surface have moved behind `orchestration` interfaces.
`orchestration/diagnostics.py` owns deterministic string and JSON formatting;
`node.py` retains ROS diagnostic message construction, publication, locking,
and callback scheduling.
`orchestration/parameters.py` is the sole owner of parameter names, defaults,
and declaration order; launch profiles only assign values. This is the first
seam for the Phase 4 C++ ROS shell; it does not change scheduling.

`orchestration/planner_process.py` owns the opt-in spawned MPPI worker and
private bounded socketpair transport. It reuses `planning/mppi.py` for solve
algebra and the installed model loader for mechanics. The ROS node retains
capture-response policy, takeup, safety gates, command publication and fault
latching. The worker has no ROS node or actuator interface. Planner-only
execution memory crosses the explicit MPPI memory API; no mutable node or
estimator object is shared between processes.

Allowed dependency direction is:

```text
node ────────> orchestration + planning + transmission + safety
```

Canonical implementation modules and maintained consumers import these package
boundaries directly. Retired root module paths are intentionally unsupported.

## `simulation`

The standalone package owns the isolated plant, device, perception, scenario,
target, and RViz simulation runtimes. It depends one-way on
`catheter_control` for shared safety contracts; the controller core does not
import simulation code. Existing executable names and `/sim` endpoints are
preserved.

## `control_interface_py`

`manager.py` and `device_serial_com.py` retain arbitration, freshness, final
ROS-side safety and serial ownership. `streamed_position.py` owns the immutable
encoder-endpoint command, M2 session/lifetime envelope and codec. The C++ codec,
session and pure executor live in firmware headers and are shared with native
simulation through ABI-checked `streamed_position_native.py` (ABI 2). Streamed
endpoints use driver-native position with stop-confirm-replace; the sketch includes them only
behind the default-off compile flag. `encoder_coordinates.py` owns fixed,
read-only calibration/coupling, reused by controller projection and manager
endpoint validation. Controller `safety/position_execution.py` owns plan-root
endpoint construction, not the manager or model runtime. No calibration-reset
surface is added. Ordinary velocity and homing remain unchanged. Details and offline verification
are in `audits/ros-realtime/MPPI_DRIVER_NATIVE_POSITION_INTEGRATION.md`.

`position_stream_probe.py` composes explicit, bounded hardware qualification
through the manager. `native_position_probe.py` owns its driver-native POS
scenario calculations/sequencing, reusing legacy firmware position execution.
It tests cancellation and stop-confirm-replace, not undocumented live driver
retargeting; see `audits/ros-realtime/DRIVER_NATIVE_POSITION_TEST.md`.

## `bringup`

The standalone package owns all launch composition. Runtime nodes, safety rules,
and algorithms remain in their functional packages. Hardware and simulation
stacks are launched through `bringup`.

## `perception`

The standalone ROS package owns online observations and live shape adapters:

```text
perception/
├── marker_tracking.py       # dual-rig online marker publisher and diagnostics
├── marker_udp_receiver.py   # validated remote marker input
├── em_bridge.py             # EM PointArray to canonical PoseArray bridge
├── state_estimator.py       # legacy live coil-shape adapter
└── estimation_protocol.py   # validated live-estimation configuration
```

## `experiments`

The standalone ROS package owns experiment schedules and guarded collection:

```text
experiments/
├── collection.py              # guarded collection runtime
├── causal_experiment.py       # causal isolation schedules
└── identification.py          # continuous identification schedules
```

Research recording additionally uses `recording_session.py` for descriptive
session allocation/finalized evidence and `session_recording.py` for owned
process lifecycle and recording readiness. It owns no motor command surface.
`bringup/research_session.launch.py` defines process composition; reusable
control task clients remain in `control_tasks`.

`reaching_session.py` binds a reviewed frozen target file to an explicitly
enabled, ready recording; it executes the existing guarded `control_tasks`
client rather than owning motion logic. `reaching_analysis.py` owns offline
journal metrics and reports. `control_tasks/trial_records.py` owns optional
durable task event serialization without depending on the research package.
`experiments/reaching_bag.py` owns study-specific bag evidence, reversal metric
definitions and diagnostic plots, reusing ROS deserialization and the canonical
marker validator. It never owns estimator or controller runtime logic.

The package depends only on its declared functional dependencies. Launch
composition belongs to `bringup`; runtime supervision is independently owned.
Controller task clients are owned by the standalone `control_tasks` package.

## `runtime_supervision`

The standalone ROS package owns runtime and session qualification:

```text
runtime_supervision/
├── runtime_identity.py       # parameter and artifact identity capture
├── session_check.py          # finalized-bag completeness validation
├── stationary_analysis.py    # offline stationary-noise qualification
├── compute_profile.py        # non-actuating recorded-input compute profiling
├── compare_compute.py        # isolated historical/promoted package A/B runner
├── compare_spatial.py        # matched Cartesian drift and frozen-root numerics
├── compare_ros.py            # disarmed production callbacks and discarded plans
├── replay_exchange.py        # private latest-only snapshot IPC for offline replay
├── engagement_g4.py          # experimental bundle/replay CLI
├── engagement_qualification.py # independent causal replay/qualification owner
├── qualify_engagement_integration.py # installed-wheel matched offline replay (no ROS graph)
├── qualify_recovery.py        # bounded inert-owner fault suite and artifact identity
└── engagement_shadow.py      # non-commanding ROS observer and timing surface
```

The longer package name avoids collision with the installed third-party Python
package named `supervision`.

Engagement qualification's coarse/fine model diagnostic imports the packaged
`cr_meta_lnn.deployment.experimental.engagement_rollout_check`; no runtime
supervision import may depend on the unshipped model-training package.

Offline compute profiling depends one-way on `catheter_control` and reuses its
model-loading environment. It is not imported by live controller orchestration;
`catheter_control.orchestration.timing` remains the lightweight online probe.
`runtime_supervision.reduced_filter_shadow` is a non-commanding research observer of the
reduced Gate 3 filter, whose mathematics and stream wrapper stay in `cr_meta_lnn`.
`reduced_filter_replay` is its recorded test source, restricted to an isolated domain and
topic prefix. Neither is selected by a launch profile. The observer imports encoder conversion,
the ordered inbox and the encoder input QoS from `catheter_control.orchestration`.

`catheter_control.orchestration.reduced_filter_owner` (ROS-independent) owns reduced-filter
ownership for planning: encoder conversion, the ordered input handoff, health mapping, plan-time
roots under one lock and per-law-count kernel preparation. `catheter_control.reduced_filter_dry_run`
is a separate, non-commanding `catheter_mppi` node class, selected only by
`estimator_backend: reduced_filter_dry_run` through `control.launch.py`. It has no command
publisher, manager client or arm service. `node.py` refuses that backend.

`catheter_control.orchestration.reduced_filter_settings` owns the settings shared by both
reduced-filter controllers:

- the controller-only limits;
- the planner configuration;
- the switches the filter makes illegal;
- the image-stamp `plan_due` cadence;
- the stream and backend loader.

`node.py` also runs a commanding backend, `estimator_backend: reduced_filter` (stack
`hardware_reduced_filter_no_rotation_half_speed`, added 2026-11-03):

- **Estimation:** `ReducedFilterOwner` replaces the v171 runtime pair. A 5 ms owner callback
  (`_reduced_owner_tick`) maps owner health onto the existing readiness gates.
- **Planning:** commit-triggered, in that callback; there is no plan timer.
- **Unchanged:** the existing command heartbeat, arming, mode claim, release, fault latching and task
  GC. Command output stays launch-controlled.
- **v171:** the path is unchanged, as shown by the unchanged existing test suite.

`bringup.research_filter_environment` owns the single-threaded BLAS/OpenMP environment of the
reduced-filter processes.

Run the profiler with `ros2 run runtime_supervision compute_profile`.

`runtime_supervision.qualify_live_compute` owns the bounded, stationary live
compute qualification. It composes production estimator callbacks and the
isolated planner, rejects arming, blocks command/mode publication, and discards
all computed plans. It must replace (not accompany) the normal controller.
It is a research/timing tool, not an alternate actuator-control path.

## `control_tasks`

The standalone package owns task-level action servers, target and path clients,
camera overlays, and recording helpers. It depends one-way on the controller
core; launch composition belongs to `bringup`. Existing executable names are
preserved under the `control_tasks` ROS package.

## Retired package

The historical `automation` package and its compatibility imports, executable
aliases, launch aliases, and configuration symlink were removed after all
maintained consumers migrated. Historical session manifests remain immutable.
The root-level `catheter_control` compatibility modules were removed after
maintained callers migrated to the canonical package paths.
