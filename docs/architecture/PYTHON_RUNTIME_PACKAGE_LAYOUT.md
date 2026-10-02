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

The package depends only on its declared functional dependencies. Launch
composition belongs to `bringup`; runtime supervision is independently owned.
Controller task clients are owned by the standalone `control_tasks` package.

## `runtime_supervision`

The standalone ROS package owns runtime and session qualification:

```text
runtime_supervision/
├── runtime_identity.py       # parameter and artifact identity capture
├── session_check.py          # finalized-bag completeness validation
└── stationary_analysis.py    # offline stationary-noise qualification
```

The longer package name avoids collision with the installed third-party Python
package named `supervision`.

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
