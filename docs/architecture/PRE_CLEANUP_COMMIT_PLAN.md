# Pre-cleanup logical commit plan

This plan organizes the dirty working trees without automatically staging or
committing user work. Generated data, caches, and `.orig` backups are excluded.
Files shared by two concerns require `git add -p` and review rather than a
whole-file sweep.

## Progress

| Slice | Status | Verification |
| --- | --- | --- |
| R1 — ROS control interfaces | complete: `f3640b6` | isolated index snapshot built; XML/CMake lint passed; package-wide Python lint still reports pre-existing manager/serial style debt |
| M1 + M2 — active v171 deployment and runtime models | complete: `4b210cd` | exact staged snapshot passed 51 focused tests |

The deployment package is now intentionally v171-only and lazy. The uncommitted
v150 runtime remains untouched in research history rather than being exposed by
the production package.

## Rule zero

Before any commit:

1. preserve the current inventory and runtime-closure manifests;
2. inspect each tracked-file diff for unrelated changes;
3. run the relevant tests for that group;
4. never use `git add -A` at either repository root;
5. never include `build`, `install`, `log`, caches, checkpoints, evaluation
   arrays, videos, or bags.

## `robot-infra`

### R1 — ROS control interfaces (complete)

Scope:

- new `TrackTipPath` and `TrackTipTrajectory` actions;
- new controller trace/reference messages;
- `GenerateSparseTargets.srv`;
- only the matching interface-generation hunks in `CMakeLists.txt` and
  `package.xml`.

Gate: interface generation and dependent Python import tests.

### R2 — catheter controller implementation

Scope:

- `src/catheter_control/catheter_control/*.py`;
- package metadata, resource marker, and bootstrap;
- corresponding unit tests.

Exclude caches and `.orig` files. Keep model/cost/threshold behavior unchanged.
Because `setup.py` exports experiment and simulation entry points, the package
source should be committed as a coherent buildable unit rather than leaving
entry points that target absent modules.

Gate: all `catheter_control` unit tests and package build.

### R3 — launch and reviewed controller profiles

Scope:

- `control.launch.py` and `simulation.launch.py`;
- grouped, plain-with-takeup, and plain comparison profiles;
- reviewed task profiles and RViz configuration;
- the production-baseline closure manifest.

The historical `.orig` files remain out until their unique differences are
reviewed. Versioned profiles are retained during this commit; semantic config
layering is a later behavior-preserving migration.

Gate: launch tests, effective-parameter manifest tests, and command-output
interlock tests.

### R4 — perception and experiment automation

Scope independently reviewable groups:

- four-marker tracking and UDP bridge;
- causal/identification collection;
- runtime identity/session checks;
- launch files and matching tests.

Do not combine marker-tracking behavior changes with experiment generators.
The eventual package split happens only after these responsibilities have
separate tests.

### R5 — manager and serial safety changes

Scope:

- manager, serial bridge, freshness helper, control-interface launch;
- matching manager/transport tests;
- related message fields not already committed in R1.

This group needs a safety-focused review. Preserve fail-closed behavior,
manager authority, watchdogs, command projection, and `SET_ZERO` rejection.

### R6 — firmware safety changes

Scope firmware source, new safety headers, and host regression tests only.
Keep separate from ROS/controller commits.

### R7 — maintained docs and audit summaries

Commit Markdown conclusions and small tables. Relocate NPZ/large plot evidence
to external session directories before removing source-tree copies. Generated
presentation work directories are not source.

## `cr_meta_lnn`

### M1 — active v171 deployment API (complete with M2)

Scope:

- `deployment/v171_streaming_runtime.py`;
- `deployment/v171_loader.py`;
- selected checkpoint loaders;
- deployment tests and artifact hash checks.

The committed deployment surface is v171-only and lazy, with an import-closure
regression test. The legacy runtime remains outside the production package.

### M2 — active runtime model modules (complete with M1)

Scope the model modules required by the closure, including distal first-order,
real motor transmission, scalar tendon chain, shared tendon, kinematics, and
interface transmission, plus their focused tests.

Do not mix training-pipeline experiments into this commit.

### M3 — supported training/reproduction pipelines

Commit only pipelines required to reproduce the selected v171/v174/v175
artifacts and current supported engagement/gain work. Move reusable functions
out of scripts in later commits; first preserve the working state.

### M4 — research history preservation

The remaining untracked source and plans need explicit classification. Preserve
valuable historical pipelines on a dedicated pre-cleanup research-history
branch or tag before deleting them from the active branch. This requires user
review because the current tracked modifications include unrelated model work.

### M5 — artifact manifests and hygiene

Commit the manifest boundary, cleanup documentation, and path-specific ignore
rules. Do not commit checkpoint or evaluation binaries.

## Cross-repository gate before tagging

After R1–R7 and M1–M5 are reviewed:

1. clean-clone build in the supported ROS/Python environments;
2. artifact hash validation;
3. recorded non-actuating replay of the 20260929 qualification;
4. seeded planner and belief-state conformance;
5. full simulation smoke test;
6. verify no hardware output is enabled by any default profile;
7. create one shared baseline identifier across the repositories.
