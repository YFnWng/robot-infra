# Phase 7: archival and retirement

## Scope and invariants

C++ implementation stages are deferred. This retirement does not change model
equations, MPPI tuning, scheduling, ROS interfaces, safety gates, artifact hashes,
or the supported deployment API. No hardware operation is required.

## Retired surfaces

These files in `cr_meta_lnn/deployment/` were forwarding-only aliases. Their
implementations remain under `deployment/experimental/` with the same filenames:

- `engaged_distal_checkpoint.py`
- `insertion_tendon_allocation_checkpoint.py`
- `v171_belief_lambda_checkpoint.py`

The sole remaining local source import of the old paths, in the preserved
untracked `scripts/fit_v171_insertion_tendon_allocation.py`, now uses the canonical
namespace. That script remains untracked rather than promoting an entire
historical fitting pipeline into the maintained branch for a one-line migration.
Import tests enforce the absence of the aliases and availability of the owners.
The active closure and maintained/legacy index reflect this classification.

Recovery: the removed files are preserved at
`catheter-stack-phase5-qualified-20261002` in `cr_meta_lnn`. Inspect them with
`git show <tag>:deployment/<filename>`; no worktree reset is needed.

## Deliberately retained

- Versioned public runtime exports: known consumers still use them; removing
  them would break the supported API rather than retire an unused shim.
- Semantic-profile compatibility YAMLs and legacy artifact selector arguments:
  documented callers and fail-closed regression coverage remain.
- Manifest-declared artifact links: reproduction callers still depend on them.
- Explicit `deployment/legacy` and `deployment/experimental` implementations:
  distinct reproduction/research purposes, not duplicate active implementations.
- C++ shadow shell: deferred future work, with no command authority.
- Unrelated dirty/untracked research: user-owned work, also preserved on
  `archive/pre-cleanup-research-20261001`; not a deletion target.
- Historical sessions and audit evidence: immutable provenance, not active code.

Future retirement requires a caller audit, migration, characterization tests,
and an updated ledger. This phase does not authorize wholesale deletion of
research results or currently supported compatibility contracts.

## Verification

- Focused deployment/manifest/runtime/experimental-loader suite: 83 passed.
- Wheel built from clean detached commit `99bbbf4`, excluding local research:
  retired root aliases absent; canonical experimental loaders present.
- Installed wheel import probe from `/tmp`: passed, using `cr-venv` site-packages.
- Installed `ros2 run catheter_control phase5_conformance`: passed, zero failures;
  report at `/tmp/phase7-wheel-SGGm35/conformance.json` (temporary verification
  output, not experimental session data).
- Selected manifest and artifacts unchanged; no ROS source/build change needed.
- `git diff --check`: passed for scoped changes. Unrelated local research remains.

No representative hardware timing test is claimed; scheduling is unchanged.
