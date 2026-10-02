# Phase 5 Model Deployment Stabilization Plan

Status: P5.0 and P5.1 implemented; P5.2-P5.6 pending
Primary repository: `cr_meta_lnn`
Consumer repository: `robot-infra`
Baseline: `catheter-stack-pre-cleanup-20261001`
Selected bundle: `20260929_175554_grouped_no_rotation`

Current gate status:

- P5.0: active import/API closure recorded; existing propagation, clone,
  delayed-correction, rewind/replay, and rollout fixtures verified.
- P5.1: lightweight semantic API and lazy v171 compatibility aliases
  implemented and verified.
- P5.2-P5.6: pending. The version-1 manifest and all artifact bytes remain
  unchanged.

Phase 4 representative timing qualification is explicitly deferred. That does
not block this behavior-preserving packaging and API work, but it remains a
hard prerequisite for moving command authority from the Python reference
controller to the C++ shell.

## 1. Objective

Turn the active learned-model runtime into a small, installable, versioned,
manifest-selected deployment library without changing its mechanics,
estimation, rewind/replay, or rollout results.

The end state has:

- one supported deployment entry point selected by an artifact manifest;
- one owner for state propagation, correction, rewind/replay, and rollout;
- explicit state, observation, prediction, units, shapes, and timestamp
  contracts;
- verified artifact hashes and compatibility requirements before model load;
- no production import from training, evaluation, or versioned research
  scripts;
- no dependency on the current working directory, a source checkout being on
  `PYTHONPATH`, or `/home/chen-lab/Yifan/cr_meta_lnn` being a configured root;
- a deliberate legacy namespace for historical runtimes and reproductions;
- characterization fixtures that prove structural changes preserve the
  selected v171/v174/v175 behavior.

This phase does **not** retrain a model, change the selected checkpoint, change
the estimator, tune the controller, alter ROS scheduling, or promote C++
command authority.

## 2. Current boundary and problems to remove

The selected manifest currently identifies:

| Artifact ID | Active role |
| --- | --- |
| `v171_distal_checkpoint` | frozen distal mechanics, nominal motor transmission, tendon history, and force port |
| `v174_jacobian_initialization` | initial local adaptive interface Jacobian state |
| `v175_interface_transmission_checkpoint` | motor-to-interface play and equivalent shaft-Jacobian initialization |

The production runtime is
`cr_meta_lnn.deployment.v171_streaming_runtime.V171StreamingCatheterRuntime`.
Its maintained behavior includes initialization, achieved-encoder propagation,
delayed marker correction, clone/restore semantics, rewind/replay, diagnostic
prediction, and batched control rollout.

The current deployment works, but the boundary is not yet stable:

1. `cr_meta_lnn` has no installable package metadata. Tests and ROS consumers
   rely on source-tree path injection.
2. ROS launch and controller parameters expose a repository root plus separate
   artifact paths. A manifest exists, but it is not the single load authority.
3. `robot-infra/control_tasks/camera_overlay.py` imports
   `cr_meta_lnn.networks.hybrid.distal_first_order` directly instead of using a
   deployment-facing geometry API.
4. The `deployment` directory contains the active v171 runtime beside the old
   generic `streaming_runtime.py` and several research checkpoint loaders.
5. The public class name embeds the research version even though callers need
   a capability contract, not a training-history identifier.
6. Artifact schema version 1 verifies files but does not fully declare runtime
   package compatibility, dependency versions, tensor conventions, optional
   feature contracts, or qualification revocation/supersession.
7. The `cr_meta_lnn` worktree contains substantial uncommitted research. Phase
   5 must not absorb, rewrite, or delete that work while stabilizing the
   production subset.

## 3. Invariants

- The selected model composition and all artifact SHA-256 digests remain
  unchanged during behavior-preserving Phase 5 commits.
- `V171StreamingCatheterRuntime` remains available as a temporary compatibility
  alias until all maintained consumers use the semantic API.
- State history is continuous. No package, loader, or replay boundary may
  reset history at controller windows, camera gaps, or dataset episode labels.
- The public boundary states encoder ordering, SI units, marker ordering,
  timestamp semantics, device/dtype behavior, and tensor shapes explicitly.
- Candidate rollouts clone complete state and cannot mutate the live runtime.
- Learned mechanics and estimator mathematics remain in `cr_meta_lnn`; ROS
  scheduling, lifecycle, messages, and safety remain in `robot-infra`.
- `control.adapj.AdaptiveForwardJacobian` remains the sole adaptive-Jacobian
  implementation.
- A missing, incompatible, or hash-mismatched manifest/artifact fails closed.
  There is no fallback to a “latest” file or older model.
- No step adds or exercises `SET_ZERO`, hardware command output, or any other
  actuation path.
- Structural moves and numerical/model changes use separate commits.

## 4. Target deployment API

Keep `cr_meta_lnn.deployment` as the stable package. Do not rename the whole
repository or perform a `src/`-layout migration in the same change.

The target default surface is intentionally small:

```python
from cr_meta_lnn.deployment import (
    DEPLOYMENT_API_VERSION,
    RuntimeState,
    RuntimeObservation,
    RuntimePrediction,
    MarkerUpdateResult,
    load_runtime_bundle,
)

bundle = load_runtime_bundle(
    manifest_path,
    device="cuda",
    dtype="float32",
    options=runtime_options,
)
runtime = bundle.runtime
```

`load_runtime_bundle` is the only maintained artifact-selection path. It:

1. loads and validates the manifest schema;
2. checks deployment API and package compatibility;
3. resolves artifacts by ID, never by array order;
4. verifies byte size and SHA-256 before deserialization;
5. calls only allow-listed package loaders;
6. returns the runtime plus immutable identity/qualification metadata.

The maintained runtime capability contract contains only:

- `initialize(timestamp_ns, encoder_counts, ...)`;
- `reset()`;
- `advance_encoder(timestamp_ns, encoder_counts, ...)`;
- `observe_markers(timestamp_ns, points, quality)`;
- `clone_state()` and `clone_state_at_or_before(timestamp_ns)`;
- `predict_sequence(state, velocity_sequence, dt_sequence, ...)`;
- `predict_control_sequence(state, velocity_sequence, dt_sequence, ...)`;
- `current_markers()` and `markers_for_state(state)`;
- `estimator_status(now_ns)` and `diagnostics()`.

Experimental methods such as specialized coarse rollout may remain on the
concrete implementation while in use, but they are not added to the stable
contract until a maintained consumer and conformance fixture require them.

The concrete implementation may remain v171 internally. The semantic API is
not permission to hide a model change: runtime family, source identity, and
artifact hashes remain visible in bundle identity and diagnostics.

## 5. Target source organization

Use movement only after characterization tests exist:

```text
cr_meta_lnn/
  deployment/
    __init__.py              narrow stable exports
    api.py                   typed public records and capability protocol
    bundle.py                manifest-selected construction and identity
    runtime.py               canonical active implementation owner
    artifacts/
      manifest.py            schema and file verification
      v171.py                selected artifact deserialization
      interface.py           interface-transmission deserialization
    legacy/
      v150.py                explicit historical import only
  networks/                  reusable learned mechanics, not artifact policy
  training/ or scripts/      research workflows; never imported by deployment
  tests/
```

This layout is a direction, not a mandatory one-shot move. Prefer first adding
`api.py` and `bundle.py` around the existing validated implementation. Rename
or move `v171_streaming_runtime.py` only after imports and serialized
references are characterized. Research checkpoint loaders remain outside the
default export surface; they move to `deployment.experimental` or a research
package only when their callers are known.

## 6. Artifact manifest version 2

Schema version 2 should retain all version-1 fields and add:

- `deployment_api_version` with a supported version range;
- `runtime_family` and concrete runtime factory ID;
- producing and qualified source commits, including dirty-state declaration;
- Python, PyTorch, NumPy, `control`, and `cr-common` compatibility ranges;
- artifact serialization format and loader schema version;
- canonical axis order, marker order, frames, units, dtype, and dimensions;
- required and optional runtime features with explicit defaults;
- qualification scope, evidence links, status, and optional supersession;
- artifact-to-artifact compatibility constraints;
- manifest self-identity recorded by callers as SHA-256 of canonical bytes.

The manifest must remain portable. Artifact paths are relative to an installed
bundle root, not to a developer checkout. Legacy paths are migration metadata,
not a runtime search path.

Version 1 remains readable only through an explicit migration command that
emits a reviewed version-2 manifest. Production launch must not silently
upgrade metadata in memory.

## 7. Work sequence and gates

### P5.0 — Preserve research and freeze the active closure

- Record `cr_meta_lnn` commit, branch, dirty state, active production files,
  maintained experiment files, historical reproduction files, generated
  artifacts, and unclassified files.
- Do not clean or reformat unrelated modified and untracked research files.
- Capture exact `robot-infra` imports, constructor arguments, public calls,
  artifact IDs, and diagnostics consumed by the selected stack.
- Record the selected manifest and all verified hashes.
- Add compact deterministic fixtures for one-step propagation, multi-step
  propagation, state clone isolation, delayed marker correction, rewind/replay,
  diagnostic prediction, and batched MPPI rollout.

Acceptance: the active closure and its numerical/discrete outputs can be
reproduced without importing training or evaluation scripts. Unrelated
research remains byte-for-byte untouched.

### P5.1 — Add the semantic API without moving implementation

- Add typed public records/protocols with documented shapes and units.
- Add a semantic runtime/bundle name while retaining the v171 alias.
- Make default exports lazy and limited to the active API.
- Add import tests proving the legacy runtime and research model families are
  not imported as side effects.
- Reject unknown runtime options rather than accepting generic keyword bags.

Acceptance: existing callers and new semantic callers produce identical state,
predictions, marker results, and diagnostics on the P5.0 fixtures.

### P5.2 — Make the manifest the load authority

- Implement schema version 2 and a deterministic v1-to-v2 migration CLI.
- Add `load_runtime_bundle` with allow-listed artifact IDs and loaders.
- Verify hashes before `torch.load` or JSON parsing.
- Include manifest hash, artifact hashes, deployment API version, concrete
  runtime family, device, and dtype in runtime identity.
- Test missing, modified, duplicated, unknown, escaping, incompatible, and
  superseded artifact records.

Acceptance: the selected bundle loads from a relocated temporary artifact root
and produces fixture-equivalent results; every corrupt or incompatible case
fails before runtime construction.

### P5.3 — Package the deployment library

- Add minimal `pyproject.toml` metadata and declare runtime versus optional
  research dependencies.
- Preserve the existing package name `cr_meta_lnn`.
- Ensure `control` and `cr-common` dependencies are explicit; do not vendor or
  copy their implementations.
- Build a wheel and install it in an isolated environment.
- Verify imports and bundle loading from outside the source checkout with no
  manual `PYTHONPATH` or current-working-directory assumption.
- Keep `/home/chen-lab/Yifan/cr-venv` as the supported deployed interpreter;
  do not install into system Python.

Acceptance: a clean isolated install imports the narrow API and runs artifact
and rollout fixtures from an arbitrary working directory.

### P5.4 — Migrate `robot-infra` consumers

- Replace `cr_meta_lnn_root`, individual checkpoint parameters, and source-path
  injection with one explicit `model_manifest` parameter at the launch/runtime
  boundary.
- Retain old launch arguments as temporary fail-closed compatibility inputs;
  reject ambiguous simultaneous old/new selection.
- Change the controller, simulation perception, runtime identity, and overlay
  tools to use deployment APIs only.
- Add a narrow marker/centerline geometry method to the deployment API so
  `camera_overlay.py` no longer imports `networks.hybrid` directly.
- Record manifest and installed package identity in every session.

Acceptance: maintained `robot-infra` source contains no `sys.path` insertion,
`CR_META_LNN_ROOT` default, direct `networks.hybrid` import, or independently
selected model artifact path. Launch/configuration compatibility tests still
cover the temporary aliases.

### P5.5 — Isolate research and legacy surfaces

- Classify each `deployment/*.py` file as production, supported experiment,
  historical reproduction, compatibility, or retired.
- Move the old generic runtime behind an explicit legacy import.
- Move reusable functions out of scripts only when a maintained package caller
  needs them; keep CLIs thin.
- Do not mass-rename versioned research scripts. Index historically important
  reproduction entry points and leave the rest on the preservation branch
  until their value is decided.
- Delete compatibility aliases only after repository-wide and installed-entry
  point searches show no maintained callers.

Acceptance: production imports form a small acyclic closure and cannot reach
training datasets, evaluation CLIs, plotting stacks, or historical runtimes.

### P5.6 — Cross-repository conformance and baseline update

- Run focused `cr_meta_lnn` deployment tests and complete fixture replay.
- Build affected ROS packages and test installed entry points.
- Run non-actuating simulation/replay for grouped, plain-with-takeup, and plain
  modes using the manifest-selected runtime.
- Compare state, tips, markers, accepted/rejected observations, estimator
  reasons, and controller decisions against the frozen baseline with
  field-specific tolerances.
- Update the handoff, active deployment baseline, component ownership, legacy
  index, and artifact manifest together.
- Create a new shared baseline only after both repositories are clean for the
  selected closure.

Acceptance: clean installs reproduce the baseline behavior and identity from
the manifest alone. Phase 4 timing remains marked deferred and C++ command
authority remains unavailable.

## 8. Required tests

At minimum preserve or add tests for:

- narrow lazy imports and explicit legacy imports;
- manifest schema, relocation, hashes, loader allow-list, and incompatibility;
- artifact-only restoration of v171/v174/v175 state;
- initialization and reset semantics;
- raw versus effective motor coordinates;
- one-step distal/interface/history equivalence;
- nonuniform-`dt` multistep rollout;
- state clone/restore and candidate isolation;
- full versus control-only prediction equivalence;
- batched versus scalar rollout equivalence;
- delayed marker correction and exact rewind/replay ordering;
- estimator rejection reasons and diagnostics;
- CPU/GPU and supported dtype behavior;
- installed-wheel import and arbitrary-working-directory execution;
- `robot-infra` launch resolution and session identity;
- absence of command publication and `SET_ZERO` paths in all deployment tests.

Numerical tolerances are per field. Discrete state transitions, validity, and
reason codes require exact agreement.

## 9. Planned commit sequence

1. `docs: freeze phase 5 model deployment closure`
2. `test: characterize active model runtime behavior`
3. `feat: add semantic deployment API`
4. `feat: add versioned manifest-selected runtime bundle`
5. `build: package cr_meta_lnn deployment runtime`
6. `refactor: migrate robot runtime to model manifest`
7. `refactor: isolate model research and legacy deployment`
8. `test: qualify packaged model runtime across repositories`

Each commit must pass its focused tests. File movement does not share a commit
with equation, estimator, artifact, controller, or scheduling changes.

## 10. Rollback and stop conditions

Rollback is selecting the last qualified manifest and reverting the structural
commit; no artifact is overwritten in place.

Stop and investigate if:

- a fixture changes outside its declared tolerance;
- a discrete estimator/controller reason changes;
- an installed package needs the source checkout to import;
- a loader tries an undeclared or unhashed path;
- the production import closure reaches a training/evaluation module;
- a `robot-infra` consumer needs a concrete v171 internal not covered by the
  declared deployment API;
- unrelated dirty research would need to be modified or committed;
- any change affects ROS scheduling, safety, command authority, or model
  mechanics without a separate review.

## 11. Immediate next implementation slice

The safest first implementation slice is P5.0 plus P5.1:

1. write a machine-readable active import/API closure;
2. add missing delayed-correction and rewind/replay characterization fixtures;
3. introduce semantic typed exports around the existing runtime without moving
   its implementation;
4. retain and test the `V171StreamingCatheterRuntime` alias;
5. make no edits to unrelated research files and no artifact changes.

Only after that slice is reviewed should manifest schema 2 and packaging begin.
