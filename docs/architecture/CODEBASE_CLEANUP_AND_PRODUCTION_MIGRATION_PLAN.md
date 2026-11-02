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
[Phase 4 C++ ROS shell plan](../../../repository_archives/cleanup_20261102/files/robot-infra/docs/architecture/PHASE4_CPP_ROS_SHELL_PLAN.md).

Status: the non-commanding shell, versioned worker contract, Python reference
adapter, opt-in launch surface, shared cross-language fixtures, and static audit
are implemented. Replay/simulation conformance and representative full-stack
timing qualification remain open; no command authority has moved.

Representative timing qualification is deferred by decision on 2026-10-01.
This is not a passed gate: it remains required before any C++ command-authority
promotion, but it does not block behavior-preserving Phase 5 packaging work.

Further C++ implementation is deferred by user decision on 2026-10-02. The
existing non-commanding shell is retained; deferred gates are not passed gates.

### Phase 5 -- `cr_meta_lnn` deployment stabilization

Narrow and version the deployment API, add artifact manifests and hashes,
package reusable code, and isolate legacy deployment/research pipelines. The
staged implementation and acceptance gates are defined in the
[Phase 5 model deployment stabilization plan](../../../repository_archives/cleanup_20261102/files/robot-infra/docs/architecture/PHASE5_MODEL_DEPLOYMENT_STABILIZATION_PLAN.md).

Status: P5.0-P5.6 complete. API, manifest verification, clean wheel packaging,
consumer migration, legacy isolation, and cross-repository conformance are
closed at shared tag `catheter-stack-phase5-qualified-20261002`. This does not
qualify representative live timing or transfer command authority to C++.


### Phase 6 -- promote stable algorithms to C++

Promote one bounded component at a time with fixture conformance, replay,
simulation, shadow hardware, and timing qualification.

Status: deferred by user decision on 2026-10-02; no C++ authority promotion.

### Phase 7 -- archival and deletion

After replacements are proven, remove aliases and retired code from the active
branch, preserve important history in tags/branches, and leave a legacy index.

Status: verified alias retirement implemented on 2026-10-02. Three root model
loader shims are removed, their sole remaining source caller is migrated, and
tests enforce canonical imports. The Phase 5 tag preserves the retired files;
the research archive branch and unrelated local research remain untouched.
Supported aliases and reproduction paths remain where consumers still exist.
See the [Phase 7 retirement ledger](../../../repository_archives/cleanup_20261102/files/robot-infra/docs/architecture/PHASE7_ARCHIVAL_AND_RETIREMENT_20261002.md)
for the deliberately bounded scope and verification evidence.

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
  [clean-source gate report](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/cross-repository-clean-source-gate-20261001.md)).
- [x] Relocate qualified artifacts after checksum and consumer review; retain
  manifest-declared compatibility aliases.

## Current research cleanup — 2026-11-01

This phase supersedes no deployed baseline or earlier qualification. Start with
the [short research status](../../audits/ros-realtime/GATE3_FILTER_STATUS.md).
No numerical source,configuration,artifact,entry point or test was moved or
deleted in the inventory/document-navigation step.

### Inventory boundary

Existing read-only `generate_repository_inventory.py`,1MiB hash limit:

| Repository | HEAD at inventory | Tracked | Nonignored untracked | Modified tracked |
| --- | --- | ---: | ---: | ---: |
| `robot-infra` | `28ff41a8a0264a4bc005e48bb9d8e131a31c98fb` | 464 | 506 | 86 |
| `cr_meta_lnn` | `99bbbf40cf39d0685f9d8b299e70225e267aa019` | 481 | 1768 | 14 |

These describe dirty working trees,not reproducible clean release commits.
The inventory precedes this new status page and documentation edits.

Static local-import protection roots are deployed-model API/loaders,scalar
native-memory replay/readout,and array candidate/retained-state/benchmark plus
regression entry points. Including package initializers,the conservative
closures contain36/121/162 Python modules respectively,including3/83/124
untracked files. All are protected pending deeper review. Counts overlap.
This scan includes inactive syntax branches;it does NOT establish actual live
execution,dynamic imports,ROS/config/shell consumers or pickle compatibility.
Modules outside these closures are unresolved,NOT retirement-approved.
All3 selected deployment artifacts match manifest hashes and byte sizes.

Recoverable pre-edit baseline saved under the external session root as
`repository_cleanup_baseline_20261101.tar.xz` (6755808 bytes),SHA-256
`ce345908428faf0261fede1caa75dfdf901bfe76dac9466a6508816be62fce5c`.
It contains the complete inventory/summary,protected dependency graph and its
scan script,and an inner `source_snapshot.tar.xz` with3181 nonignored
source/config/document/test/other files. All3181 archive member paths match
the selection;3176 available inventory content hashes verify. Source files
were captured BEFORE the navigation edits in this phase. Git objects,ignored
files,large generated data and deployment artifact bytes are NOT a full-disk
backup;do not delete them on the strength of this archive. Symlinks remain
symlinks. The inventory hash limit is not a cleanup deletion criterion.
CSV and full intermediate analysis remain temporary duplicates;the external
JSON inventory and compact protected graph retain the classification evidence.
External free space after saving:402665472 bytes,above the384MiB reserve but
with only12288 bytes of headroom. Do not launch another artifact-writing run
before separately reviewing disk recovery.

Classification:manifest artifacts and the reviewed closures are protected;
cache files are candidates only;all remaining files await consumer review.
Robot ROS source,firmware,safety,launch/config and qualification tooling remain
protected areas;the model-only import scan cannot classify their retirement.
Generated-artifact classification is lexical:it must not override a selected
artifact manifest,fixture use or explicitly protected external evidence.

### Documentation disposition (no bulk movement yet)

| Existing document | Disposition |
| --- | --- |
| `GATE3_FILTER_STATUS.md` | New short current status/navigation; no experiment diary |
| `GATE3_FILTER_PIPELINE_REFERENCE.md` (4832 lines before cleanup) | Replaced with current-only math/flow/owners;full original archived outside repos |
| `GATE3_KNOB_SCALE_INTERFACE_PLAN.md` (11136 lines) | Full historical evidence archived;old path is a pointer |
| `GATE3_JACOBIAN_REINTEGRATION_PLAN.md` (11913 lines) | Full historical evidence archived;old path is a pointer |
| `GATE3_OBSERVATION_GROUNDED_J_AUDIT.md` (6196 lines) | Full diagnosis archived;old path is a pointer |
| `GATE3_DISTAL_FINAL_SESSION_TEST_PLAN.md` (3021 lines) | Full settled-distal evidence archived;old path is a pointer |
| `GATE3_ARRAY_NATIVE_IMAGE_STEP_PLAN.md` (666 lines) | Active incomplete work; retain latest contract/results, later separate implementation diary |
| Deployment baseline,ownership,package layout,maintained/legacy index | Remain authoritative for their declared deployment/ownership scope; research status does not replace them |

Current-reference extraction is now complete for the architecture/math summary.
Do NOT move a module merely because a historical document mentions it.
Script/test archival and code movement remain separate,reviewed batches.

### External archival and current-reference extraction

User-authorized persistent location (workspace filesystem,not the nearly-full
session disk):`/home/chen-lab/Yifan/repository_archives/cleanup_20261101/`.
All2275 nonignored untracked source/doc/config files selected by the archival
scan are copied into `untracked_source_docs.tar.gz`,6913737 bytes,SHA-256
`43dd50af991a76ba6d4b40225c86a8f4202b59a0b5bb6d27d67e16e5d86aa35d`.
Every archived file/symlink was verified against its source before editing.
There were no skipped nonignored untracked paths in this scan.

The archive also preserves445 neighboring audit Markdown files byte-for-byte,
with individual hashes,original paths and repository HEAD/status in
`manifest.json`. This includes the full pre-extraction reference and six plans:
knob/interface,Jacobian reintegration,observation-grounded audit,distal final
test,value-local gain,and batched conditional shadow. Existing six plan paths
are short external-archive pointers;the reference now contains current state,
math,flow and owners instead of historical experiment narratives. The active
array-native work plan remains in place.

Active untracked numerical code/tests remain at their existing paths:archival
copies do not justify breaking imports,pickle class paths or source identities.
No algorithm,settings,artifact,entry point,ROS source or test changed. This
batch does not promote the array candidate or claim a new numerical result.
Only documentation content was replaced after hash-verified preservation.
Recovery is described in the external archive README;never extract over a
dirty worktree blindly. No numerical replay or build was needed or launched.

Verification:all445 archived document hashes and the untracked archive digest
match;1565 archived untracked Python sources and1976 inventoried Python files
remain unchanged. All24 local links in the current reference/status/pointers
resolve;math-block and code-fence delimiters balance;`git diff --check` passes.
Reference reduced to260 lines;six historical plan pointers are10 lines each.
This is documentation/source-preservation verification,not estimator testing.

### Script archival batch1

Moved8 untracked standalone shell wrappers to the external archive's
`relocated/cr_meta_lnn/scripts/`:convex-material21,memory-ablation23,
diagonal-passive33,scalar-relaxation35,effective-cascade36,
effective-plasticity43,effective-force50 and valley-relaxation-force53.
Selection was based on inspected historical experiment bodies and absent
filename/stem consumers,not version numbers alone. A five-repository search
covered source,docs,tests,configs and build metadata;these wrappers are not
deployment wheel packages/entry points. Dynamic/external callers cannot be
universally excluded. Numerical Python owners,artifacts and tests stay put.

The external `script_archival_batch1.json` records exact paths,SHA-256 and
executable modes. All8 moved files match their previously archived originals
and pass `bash -n` without execution. All1976 previously inventoried Python
sources remain unchanged. Historical execution requires restoring original
relative locations because the wrappers derive repository roots from their
own filename. No compatibility wrappers or alternate algorithms were added.
Focused real-comparison-harness suite:10 tests pass in5.12s,with2 existing
SLSQP clipping warnings. No numerical replay,ROS build or hardware run was
required. `git diff --check` passes in both repositories.

### Script archival batch2

Moved20 additional reviewed nominal-v15 bending/force/memory shell recipes to
the same external archive (28 total). Exact candidates,hashes,executable modes
and five-repository reference-scan scope/result are recorded in
`script_archival_batch2.json`;no literal filename/stem callers were found.
All20 match the original preservation archive and pass `bash -n`.
Numerical Python modules,model artifacts,current tools and tests remain in
place. The original-path restoration requirement and external/dynamic-caller
limitations of batch1 still apply. No replay,training or hardware command ran.
Verification:all28 relocated hashes/modes match,1976 inventoried Python files
are unchanged,and10 comparison-harness tests pass in4.99s (2 existing SLSQP
clipping warnings). Diff whitespace checks pass in both repos.

### Python archival batch3

Moved6 historical CLI modules as a closed consumer group:scalar mechanical-
memory training,scalar play-remanence training/profiling,and remanent/gated-
peak/valley-relaxation force evaluators. The only two remaining literal import
callers were inside this same group;the earlier shell callers are archived.
No external consumers appeared in the searched five-repository source/docs/
configuration/build scope. The archive's `script_archival_batch3.json` records
exact paths,hashes,modes,internal imports and known limitations.

Kept the effective-force evaluator,scalar-GRU/distributed-memory/tendon-GRU
trainers and distributed-memory evaluator because they supply helpers to
retained tools or tests. Do not remove these by treating all scripts as CLIs.
No model/filter/test/artifact movement or helper extraction was bundled in.

All1970 inventoried retained Python files remain byte-identical;the6 relocated
Python files match their original hashes/modes. Total relocated files:34
(28 shell,6 Python). Current comparison plus retained tendon-history and
distributed-evaluator tests:19 pass in5.60s,with2 existing SLSQP clipping
warnings. No training,replay,hardware run or source-fingerprint migration.

### Documentation classification pass

External `documentation_review_map.json` inventories575 Markdown files:
28 current reference/navigation,2 active plan,122 protected technical/safety
evidence,6 existing pointers,18 reviewed historical candidates,and399 unresolved.
It records hashes,classification basis,incoming document mentions,source/policy
mentions and existing archival matches. Candidate bodies/status summaries
identify earlier experiments/proposals rather than current authoritative
filter behavior;all18 match already-preserved evidence. No source/policy
mentions were found for these candidates in the static scan,but16 have inbound
documentation mentions. Reconcile those references before moving them.

This is not exhaustive semantic review:unresolved documents remain untouched,
and tracked technical evidence is conservatively protected rather than declared
current. Existing architecture README now provides one current reading index
and the map link. No document/code moves or numerical tests in this pass.

### Historical document archival batch4

Reconciled26 links/citations in17 referring documents,then moved the18 reviewed
historical documents out of both repos. No source/config/policy consumers
appeared in the repeated five-repository scan. Original document bytes and
modes match the preserved archive;all new links address immutable reading
copies and retain existing section fragments. Pre-edit referring-doc backups
and the exact link patch are retained externally. No historical numerical
claims,model settings or algorithms changed. No new pointer files were added.

The external `document_archival_batch4.json` is the path/hash/recovery ledger;
`relocated/` stores moved originals and `history/` the reading copies. The
architecture README is the single maintained reading index. The original575-
document classification snapshot now corresponds to557 in-repo documents and
18 archived documents. The399 unresolved items were not reclassified or moved.
No numerical replay,training,ROS build or hardware operation was required.
Verification:all18 archived originals/reading copies match hashes,all26 new
references resolve,both retained section anchors match actual headings,and no
Markdown link in the557 remaining docs targets a removed path. All2582
inventoried source/test/configuration hashes are unchanged (including original
bytes for previously archived scripts). Whitespace checks pass in both repos.

## Retention cleanup — 2026-11-02

At the user's direction, the repositories now keep only three code paths: the
October engagement-filter/MPPI hardware pipeline that recorded the October real
sessions, the scalar research estimator as a reference, and the reduced
real-time estimator with its ROS shadow observer. Most documents were archived,
except the main references, unfinished plans and unresolved issues.

The October pipeline exists only as working-tree changes made on 2026-10-04..14,
on top of HEAD `28ff41a`. It is kept as it is and committed as a snapshot. No
ROS source, firmware, launch file, configuration or safety test was moved.

1,790 files moved to
[`repository_archives/cleanup_20261102/`](../../../repository_archives/cleanup_20261102/README.md):

- from `cr_meta_lnn`: 1,270 untracked modules, scripts, tests and configs, and
  53 documents;
- from `robot-infra`: 467 documents, 116 of them previously committed.

The array-native candidate and its plan are included: the reduced filter
superseded them. Code selection uses the static import closure of the three
retained paths, plus every module that `robot-infra/src` imports. The archive
README records the rules, the restored tests and the limits.

Verification:

- every archived hash matched before removal;
- test collection is identical to before, minus the archived tests, for 2,554
  retained `cr_meta_lnn` tests and all `robot-infra` packages;
- focused real-time, scalar, deployment and engagement suites pass: 79, 153
  and 157 tests;
- the one `robot-infra` failure is a recorded-bag `TipReferenceHorizon`
  deserialization mismatch that predates the cleanup;
- all links in the kept documents resolve.

No ROS build was needed, because no ROS source changed.
