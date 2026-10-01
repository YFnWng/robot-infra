# Pre-cleanup logical commit plan

This plan organizes the dirty working trees without automatically staging or
committing user work. Generated data, caches, and `.orig` backups are excluded.
Files shared by two concerns require `git add -p` and review rather than a
whole-file sweep.

## Progress

| Slice | Status | Verification |
| --- | --- | --- |
| R1 — ROS control interfaces | complete: `f3640b6` | isolated index snapshot built; XML/CMake lint passed; package-wide Python lint still reports pre-existing manager/serial style debt |
| R2 + R3 — controller package, launch and profiles | complete: `ed7d78c` | isolated index snapshot built both ROS packages; 302 tests passed; 15 installed console entry points resolved |
| R4 — perception and experiment automation | complete: `72335b4`, `438f379` | marker slice: 4 tests passed; experiment slice: 75 tests passed; both exact index snapshots built all three dependent ROS packages |
| R5 — manager and serial safety | complete: `bc242f7` | exact index snapshot built `control_interface`; all 70 manager/transport safety tests passed |
| R6 — firmware safety | complete: `20f2930` | exact index snapshot passed all 8 C++ host regressions and 2 firmware source-contract tests; no flash performed |
| R7 — maintained docs and audit summaries | complete: `29618f0` | 99 Markdown files passed relative-link validation; staged diff contained only text/vector documentation artifacts |
| R8 — reproducible audit tooling | complete: `3e320be` | exact staged snapshot compiled; 6 focused tests and 10 non-actuating CLI checks passed |
| M1 + M2 — active v171 deployment and runtime models | complete: `4b210cd` | exact staged snapshot passed 51 focused tests |
| M3 — supported training/reproduction pipelines | complete: `19bd41b`, `26de1fd`, `5b866d1` | exact snapshots passed 85 v171/v174, 7 v175, and 41 engagement/gain tests; reproduction CLIs loaded non-actuatingly |

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

### R2 — catheter controller implementation (complete with R3)

Scope:

- `src/catheter_control/catheter_control/*.py`;
- package metadata, resource marker, and bootstrap;
- corresponding unit tests.

Exclude caches and `.orig` files. Keep model/cost/threshold behavior unchanged.
Because `setup.py` exports experiment and simulation entry points, the package
source should be committed as a coherent buildable unit rather than leaving
entry points that target absent modules.

Gate: all `catheter_control` unit tests and package build.

### R3 — launch and reviewed controller profiles (complete with R2)

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

R2/R3 were committed together because package tests and installation depend on
the launch files and profiles. The existing shared `catheter_limits.yaml` was
included as a required runtime/test dependency; its values were preserved.
Manager/transport implementations and firmware remain separate review slices.
Only a redundant blank line at the end of one test was removed.

Verification used an exported Git index snapshot under `/tmp`, with ROS Humble
interfaces generated there and the supported `cr-venv` Python interpreter.
Its site-packages must precede system Python packages when adding
`/usr/lib/python3/dist-packages` for ROS launch dependencies: otherwise system
NumPy shadows the supported NumPy 2.x environment. Pytest plugin autoload was
disabled for this unit-suite run. No controller or device was started.

### R4 — perception and experiment automation (complete)

Scope independently reviewable groups:

- four-marker tracking and UDP bridge;
- causal/identification collection;
- runtime identity/session checks;
- launch files and matching tests.

Do not combine marker-tracking behavior changes with experiment generators.
The eventual package split happens only after these responsibilities have
separate tests.

The work was preserved as two commits. `72335b4` contains synchronized
multi-rig marker tracking, partial-view fusion, bounded diagnostic preview and
recording support, the marker-overlay launch, documentation, and its focused
tests. `438f379` contains causal experiment generation and collection,
runtime-identity/session-integrity utilities, stationary analysis, launch, and
matching tests. Shared package metadata was split so the first commit did not
install entry points whose modules were absent.

Both commits were verified from exported Git index snapshots rather than the
dirty working tree. Each snapshot built `control_interface`,
`catheter_control`, and `automation` under ROS Humble. No camera, controller,
manager, or device process was started.

### R5 — manager and serial safety changes (complete)

Scope:

- manager, serial bridge, freshness helper, control-interface launch;
- matching manager/transport tests;
- related message fields not already committed in R1.

This group received a safety-focused review. The preserved implementation adds
end-to-end source timestamp validation, position-feedback plausibility gates,
firmware boot/watchdog status handling, and a narrowly bounded inward-only
axis-0 limit-recovery service. Delayed zero velocity remains an unconditional
safe preemption. Both the manager and serial bridge reject `SET_ZERO` before it
can reach firmware. Manager authority, command projection, freshness
watchdogs, driver qualification, and fault latching remain fail-closed.

Verification built the exact staged `control_interface` snapshot under ROS
Humble and reran all 70 manager and serial framing tests against that snapshot.
No ROS node, device link, motor, or recovery service was started.

### R6 — firmware safety changes (complete)

Scope firmware source, new safety headers, and host regression tests only.
Keep separate from ROS/controller commits.

The preserved firmware adds semantic motion-command watchdog authority,
immediate all-axis stop handling, fixed-baseline encoder-integrity recovery,
bounded position-transaction resumption, and boot/watchdog diagnostics. The
incident-specific historical encoder seed remains compile-time disabled. R6
also closes the legacy raw `ZERO` command: firmware now stops motion and returns
`ERR_ZERO_FORBIDDEN`, so read-only encoder calibration no longer depends only
on the manager and serial bridge.

Verification exported the exact staged snapshot, compiled all eight
Arduino-independent C++ tests with warnings treated as errors, and passed two
source-contract tests covering forbidden encoder zeroing and the disabled
recovery seed. No Teensy toolchain was installed, so the full sketch was not
compiled; no firmware was flashed and no hardware link was opened.

### R7 — maintained docs and audit summaries (complete)

The maintained plans, conclusions, architecture diagrams, and small tabular
summaries are now versioned. Fourteen generated audit artifacts (eight JSON,
four PNG, and two NPZ files; 6.6 MB total) were moved, without deletion, to:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  repository_cleanup_archive_20261001/robot-infra/
```

The archive preserves the repository-relative paths and includes a
`SHA256SUMS` manifest. `audits/README.md` records the recovery procedure and
the ignore policy prevents regenerated JSON, NPZ, PNG, cache, presentation,
and editor-backup output from returning to the source tree. Audit-analysis
Python tools remain uncommitted for a later reproducibility-code review rather
than being folded into this documentation-only slice.

Verification checked whitespace, validated relative links in all 99 staged
Markdown files, and confirmed that the commit contained only Markdown, CSV,
YAML, DOT/SVG, README, and ignore-policy files. No hardware or runtime process
was started.

### R8 — reproducible audit tooling (complete)

The remaining twelve audit-analysis Python files are maintained reproducibility
tools rather than runtime dependencies. Their catalog and safety boundary are
documented in `audits/TOOLING.md`. Developer-specific workspace defaults were
replaced with paths derived from the repository location. The live passive
monitor remains subscription-only and the other tools consume recorded data.

The exact staged snapshot passed source compilation, six focused offline tests,
and ten CLI loading checks in the supported ROS 2 and `cr-venv` environment.
No publisher, service client, device link, or hardware process was opened.

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

### M3 — supported training/reproduction pipelines (complete)

The preserved production reproduction closure contains the v171 standalone EM
trainer, v172 posterior inference, v173 Jacobian diagnostic, and v174 composed
rollout, together with their imported model/training modules and mathematical
handoff. The active v175 motor-to-interface transmission trainer is a separate
commit. Current engagement-label, engaged-distal, and v171 belief-lambda
pipelines are preserved as research reproduction paths and are not deployed
command authority.

Exact Git-index snapshots were checked with declared artifacts copied into a
temporary tree: 85 focused v171/v174 tests, 7 v175 transmission tests, and 41
engagement/gain tests passed. Four production reproduction wrappers, eight
engagement/gain CLIs, and two wrapper syntax checks passed. No checkpoint,
evaluation array, image, video, or hardware action was committed or run.

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
