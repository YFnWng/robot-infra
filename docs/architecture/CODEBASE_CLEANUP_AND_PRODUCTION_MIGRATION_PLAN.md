# Catheter Control Codebase Cleanup and Production Migration Plan

Status: maintained plan
Initial inventory date: 2026-09-30
Repositories: `robot-infra`, `cr_meta_lnn`

This document is the maintained cleanup and production-migration plan for the
catheter control stack. It intentionally separates behavior-preserving cleanup
from controller, estimator, model, scheduling, and firmware changes.

The physical encoder zero is a read-only calibration reference. No cleanup,
qualification, migration, or firmware work described here may add or invoke a
`SET_ZERO` path.

## 1. Why cleanup starts with a baseline freeze

The current working trees contain the most mature controller, but much of that
work has not yet been incorporated into the repositories' tracked baselines.
At the initial inspection:

- `robot-infra` was at tracked commit `a179f92`, with the active
  `src/catheter_control` package and many control-interface additions still
  untracked;
- `cr_meta_lnn` was at tracked commit `15c7d3f`, with the active deployment
  runtime, model modules, tests, and many experimental scripts still untracked;
- both repositories contained unrelated modified tracked files that must be
  preserved;
- generated evaluation data, checkpoints, plots, videos, caches, source code,
  and experiment plans were mixed in the same directories.

The first operation is therefore not deletion or module movement. It is to
capture and verify the actual active dependency closure, then commit it in
reviewable units.

## 2. File classification

Every file should eventually have exactly one classification:

1. **Production** -- imported, launched, configured, or tested by the deployed
   control stack.
2. **Supported experiment** -- a reproducible collection, training,
   evaluation, or comparison entry point with a documented configuration and
   output contract.
3. **Historical reproduction** -- retained only to reproduce an important
   result and isolated from production imports.
4. **Generated artifact** -- checkpoints, rosbags, HDF5/NPZ data, plots,
   videos, traces, reports, caches, and build products. These belong outside
   source repositories unless explicitly selected as small test fixtures.
5. **Retired** -- preserved by a Git tag or archival branch and removed from
   the active branch after its replacement is verified.

Version numbers alone are not a retirement criterion. Import consumers, launch
consumers, artifact consumers, tests, and reproducibility value must be
recorded first.

## 3. Repository ownership

### `robot-infra`

Owns ROS 2 nodes, lifecycle, launch, QoS, callback/executor scheduling, message
conversion, command arbitration, freshness and safety gates, controller
integration, experiment applications, device transport, firmware, simulation,
and full-stack qualification.

It must not become the home of learned catheter mechanics or duplicate the
adaptive-Jacobian algebra.

### `cr_meta_lnn`

Owns learned catheter mechanics, model state, deployment artifact loading,
model/estimator propagation, delayed-observation correction, rewind/replay
mathematics, offline training, E-step, evaluation, and visualization libraries.

It must not publish motor commands or own ROS safety policy.

### Existing dependencies

- `control.adapj.AdaptiveForwardJacobian` remains the single implementation of
  adaptive-Jacobian update algebra.
- `cr-common` remains the home of genuinely shared modeling utilities.
- generated sessions and model-run outputs remain under external data roots.

## 4. ROS package restructuring decision

`catheter_control` is conceptually part of robot automation, but the former
`automation` package combined unrelated responsibilities: marker
perception, a legacy estimator bridge, data collection, experiment identity,
session checks, stationary analysis, launch, and configuration aggregation.
Merging `catheter_control` into it would create a larger catch-all.

The generic `automation` package was dissolved into functional packages and
retired after maintained consumers migrated. Runtime topic and diagnostic names
remain unchanged where they are part of recorded or deployed interfaces.

Target package set:

```text
robot-infra/src/
  bringup/                      launch composition and deployment profiles
  perception/                   online marker tracking and diagnostics
  control_py/                   Python reference/development controller
  control_cpp/                  production C++ controller and ROS shell
  experiments/                  collection and identification schedules
  control_tasks/                action servers, target/path clients, overlays
  simulation/                   isolated plant, device, perception, and RViz
  runtime_supervision/          runtime identity, timing, session validation
  control_interface/            manager, serial bridge, messages, safety API
  teleop/                       manual command source and Slicer integration
```

This is a target layout, not a one-shot rename. First separate responsibilities
inside the existing packages. Rename packages only after boundaries are stable
and recorded replay is qualified.

### Internal controller organization first

```text
catheter_control/
  node.py                       thin composition root
  orchestration/
    estimator_owner.py
    planner_worker.py
    actions.py
    diagnostics.py
    parameters.py
  planning/
    mppi.py
    reference.py
    path_tracking.py
  transmission/
    engagement_belief.py
    engaged_gain.py
    takeup.py
    reversal_scheduler.py
  safety/
    hardware_contract.py
    validation.py
```

Learned dynamics remain in `cr_meta_lnn`. The ROS package owns scheduling,
message conversion, lifecycle, safety integration, and controller policy.

## 5. Python/C++ hybrid production stack

Python remains useful for model research, fast controller changes, simulation,
plotting, and experiment development. It has not provided sufficiently
consistent full-stack latency for production operation. The target is a hybrid
stack with a Python reference path and a progressively promoted C++ production
path.

### Python reference/development responsibilities

- learned-model and estimator research;
- rapid MPPI and belief-state iteration;
- simulation, recorded replay, diagnostics, and experiment applications;
- generation of conformance fixtures for stable algorithms.

### C++ production responsibilities

- ROS subscriptions, QoS, timestamp validation, and bounded queues;
- callback groups and executor ownership;
- immutable sensor snapshots and state handoff;
- freshness gates, watchdogs, lifecycle, fault latching, and heartbeat;
- engagement/take-up/gain state machines after they are frozen and qualified;
- command arbitration and publication;
- eventually stable estimator and MPPI kernels when equivalence is proven.

The initial C++ node may call a Python model/planner worker through a bounded,
versioned interface. A late Python result is discarded; it is never awaited in
a safety or command-heartbeat callback.

### Preventing implementation drift

Every component promoted to C++ needs:

1. a language-neutral state and configuration schema;
2. recorded input/state fixtures;
3. seeded Python reference outputs;
4. C++ conformance tests with explicit numerical tolerances;
5. matching transition, saturation, and fault reason codes;
6. A/B shadow execution before C++ may command hardware.

For compact deterministic components, prefer one C++ core with Python bindings
after promotion. For PyTorch/GPU components, retain the Python reference until
a LibTorch/CUDA implementation passes the same rollout and timing gates.

### Promotion order

1. C++ ROS shell: ingestion, timestamps, snapshots, lifecycle, diagnostics,
   watchdog, and command publication.
2. Stable state machines: engagement belief, slow take-up, gain belief,
   joint-limit projection, and reversal policy.
3. Reference/path and deterministic cost preparation.
4. Estimator buffer ownership and rewind/replay scheduling, while learned-model
   calls may remain Python.
5. MPPI sampling, rollout, cost reduction, and selection after conformance and
   representative full-stack timing measurement.

### Deployment modes during migration

- `python_reference`: Python estimates, plans, and commands through the normal
  safety authorities.
- `cpp_shadow`: C++ consumes the same inputs and records decisions only.
- `cpp_shell_python_planner`: C++ owns ROS and safety; Python returns bounded
  model/planner results.
- `cpp_production`: promoted C++ components command, with optional Python
  shadow comparison.

No mode bypasses the manager, serial bridge, or firmware safety authority.

## 6. Future Teensy partitioning

Moving computation onto the Teensy is a future investigation, not part of this
cleanup. Reasonable candidates are timestamped segment execution,
deterministic interpolation, existing motor/encoder inner loops, hard watchdog
and stop barriers, final limit enforcement, and compact timing telemetry.

UKF/marker correction, engagement and gain beliefs, learned-model rollout,
MPPI, and experiment orchestration should remain off the Teensy unless a later
resource and safety review establishes a compelling need.

Any MCU protocol extension must be versioned, fail closed, retain manager and
firmware safety authority, and preserve unconditional `SET_ZERO` rejection.

## 7. Configuration simplification

Controller, platform, performance, task, and visualization YAMLs are currently
mixed and named by development version. The target is composable configuration:

```text
config/
  controller/
    base.yaml
    modes/{grouped,plain_with_takeup,plain}.yaml
  platform/{hardware,simulation}.yaml
  performance/{gpu_512,gpu_1024_shadow}.yaml
  experiments/{sparse_points,continuous_path,causal_identification}/
  rviz/
```

A run resolves an ordered composition such as:

```text
base + grouped + hardware + gpu_512 + sparse_points/farther_two_axis
```

Unknown parameters must be rejected. Each session records fully resolved
parameters, source commits, executable identity, and artifact hashes. Existing
versioned filenames remain temporary replay aliases.

## 8. `cr_meta_lnn` target structure

Reusable code becomes package code; scripts become thin CLIs:

```text
cr_meta_lnn/
  deployment/                   narrow active deployment API
  models/{distal,transmission,kinematics}/
  estimation/
  artifacts/{manifest.py,schema.py}
  datasets/
  metrics/
  visualization/
  training/
  tools/                        thin parameterized CLIs
  experiments/configs/          supported experiment definitions
  tests/
  docs/history/
```

Required changes:

- export only the active runtime from the default deployment namespace;
- require an explicit legacy namespace for v150 and retired comparisons;
- retain a temporary `V171StreamingCatheterRuntime` compatibility alias;
- move functions imported from `scripts` into package modules;
- replace version-specific wrappers with parameterized CLIs and configurations;
- add an installable package definition and remove `PYTHONPATH` assumptions;
- perform any eventual `src/` layout migration separately from behavioral
  refactors.

## 9. Artifact and result policy

Large generated files do not belong in source repositories. Use external roots
similar to:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/
  catheter_sessions/
  model_artifacts/cr_meta_lnn/{production,runs/<run-id>}/
```

Each production bundle has a small trackable manifest containing schema and
runtime API versions, source commits and dirty-state declaration, artifact
paths and SHA-256 hashes, units and dimensions, required runtime features,
training/evaluation run identifiers, and qualification status.

Launch files resolve a bundle from an explicit manifest or configurable
artifact root, never a developer-specific source-tree path. Existing generated
files are copied and checksum-verified before source-tree copies are removed.

## 10. Implementation phases

### Phase 0 -- freeze and inventory

1. Capture commit, branch, dirty state, tracked/untracked counts, file
   classification, and large-file inventory for both repositories.
2. Identify every source, configuration, and artifact loaded by the latest
   successful controller runs.
3. Record resolved parameters, executable paths, environment, artifact hashes,
   and ROS interface versions.
4. Add production source in small logical commits without absorbing unrelated
   modifications.
5. Run unit, build, simulation, and non-actuating replay gates.
6. Tag both repositories with one shared baseline identifier.

Acceptance: a clean clone plus declared artifacts reproduces the same
non-actuating replay outputs and controller decisions.

### Phase 1 -- hygiene and documentation

Tighten cache/generated-file ignores without hiding manifests; compare `.orig`
backups before removal; relocate artifacts after checksum verification; create
one authoritative deployment baseline; add a legacy index; document ownership.

Status: complete. The authoritative baseline, ownership map, legacy index,
recoverable `.orig` archive, canonical deployed-artifact bundle, and
fail-closed compatibility aliases are in place. Maintained runtime consumers
use canonical paths; historical scripts may use manifest-declared aliases.

### Phase 2 -- configuration normalization

Separate controller, platform, performance, experiment, and RViz settings; add
deterministic merge and validation; record resolved configuration; introduce
semantic names and compatibility aliases.

Status: complete for the active controller variants. Semantic stacks compose
controller, platform, and performance layers in a fixed order; task and RViz
references occupy canonical directories. The resolver rejects unknown,
duplicate, escaping, or output-interlock settings. Hardware and simulation
session manifests record the resolved stack, and versioned filenames remain
compatibility aliases.

### Phase 3 -- Python internal modularization

Split the controller composition node, beliefs/state machines, applications,
and simulation. Split `automation` responsibilities behind compatibility entry
points. Do not change behavior.

Status: complete. `catheter_control` has canonical `orchestration`, `planning`,
`transmission`, and `safety` subpackages. Task clients have moved to the
standalone `control_tasks` package, simulation owns the isolated plant and
visualization runtime, and all launch composition has moved to `bringup`. The
temporary root controller compatibility modules have been retired. ROS
parameter declarations and deterministic diagnostic value serialization have
single owners under `orchestration`; the composition node retains only ROS
entities, scheduling, locking, lifecycle transitions, and publication. Further
separation of planner execution is a Phase 4 scheduling change, not
behavior-preserving cleanup. See
[Python runtime package layout](PYTHON_RUNTIME_PACKAGE_LAYOUT.md).

Post-Phase-3 extraction includes five standalone functional packages.
`perception` owns online marker tracking, marker UDP input, the EM bridge,
and the legacy live state-estimation adapter. `experiments` owns guarded
data collection and the causal and identification schedules.
`runtime_supervision` owns runtime identity, finalized-session validation, and
stationary-session qualification. `control_tasks` owns action servers and task
clients. `simulation` owns the isolated device, plant, perception, scenario,
target, and RViz runtime.
Launch composition belongs to `bringup`, and the shared safety limits belong to
`control_interface`. The historical `automation` compatibility package is
retired; historical session manifests remain unchanged.

### Phase 4 -- C++ ROS shell and shadow deployment

Define language-neutral snapshots and results, implement the C++ shell, run
Python/C++ on identical recordings, and qualify timing. Python retains command
authority until C++ shadow conformance passes and a separate authority change is
explicitly reviewed. The implementation sequence and qualification gates are
defined in the
[Phase 4 C++ ROS shell plan](PHASE4_CPP_ROS_SHELL_PLAN.md).

Status: the non-commanding shell, versioned worker contract, Python reference
adapter, opt-in launch surface, shared cross-language fixtures, and static audit
are implemented. Replay/simulation conformance and representative full-stack
timing qualification remain open; no command authority has moved.

Representative timing qualification is deferred by decision on 2026-10-01.
This is not a passed gate: it remains required before any C++ command-authority
promotion, but it does not block behavior-preserving Phase 5 packaging work.

### Phase 5 -- `cr_meta_lnn` deployment stabilization

Narrow and version the deployment API, add artifact manifests and hashes,
package reusable code, and isolate legacy deployment/research pipelines. The
staged implementation and acceptance gates are defined in the
[Phase 5 model deployment stabilization plan](PHASE5_MODEL_DEPLOYMENT_STABILIZATION_PLAN.md).

Status: P5.0-P5.1 complete. The selected closure is machine-readable, and the
semantic deployment API wraps the unchanged concrete v171 runtime. Manifest
schema migration, packaging, consumer migration, and legacy isolation remain.


### Phase 6 -- promote stable algorithms to C++

Promote one bounded component at a time with fixture conformance, replay,
simulation, shadow hardware, and timing qualification.

### Phase 7 -- archival and deletion

After replacements are proven, remove aliases and retired code from the active
branch, preserve important history in tags/branches, and leave a legacy index.

## 11. Characterization and conformance gates

Before structural movement, preserve tests for:

- artifact loading and hashes;
- one-step and multistep model rollout;
- state clone/restore;
- delayed marker rewind/correction/replay;
- engagement, take-up, reversal, and gain-belief transitions;
- seeded MPPI candidates, costs, modes, and commands;
- grouped, plain-with-takeup, and plain controller modes;
- limits and hardware-contract projection;
- launch/profile resolution;
- manager source stamps, freshness, and fault latching;
- unconditional `SET_ZERO` rejection;
- command-output-disabled behavior;
- representative two-axis and continuous-path recorded replays.

Python/C++ comparisons cover numerical values and discrete reason codes. Timing
is measured under representative full-stack load, not inferred from an
isolated microbenchmark.

## 12. Changes excluded from cleanup commits

Do not combine file movement with changes to model equations or artifacts,
estimator covariance, MPPI parameters, engagement/take-up/gain thresholds,
callback scheduling, QoS, ROS interface semantics, or manager/firmware safety.

## 13. Initial implementation checklist

- [x] Save this maintained cross-repository plan.
- [x] Add a read-only repository inventory tool.
- [x] Add a documentation index for architecture and migration material.
- [x] Add safe cache-only ignore rules without deleting files.
- [x] Generate and review the first inventory snapshot.
- [x] Resolve the active production dependency closure.
- [x] Capture artifact hashes and resolved controller parameters.
- [x] Commit untracked production files in logical repository-specific groups.
- [x] Establish and tag the reproducible pre-cleanup baseline as
  `catheter-stack-pre-cleanup-20261001` (see the
  [clean-source gate report](../../audits/ros-realtime/cross-repository-clean-source-gate-20261001.md)).
- [x] Relocate qualified artifacts after checksum and consumer review; retain
  manifest-declared compatibility aliases.
