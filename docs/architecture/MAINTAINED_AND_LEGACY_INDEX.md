# Maintained and legacy index

Use this index before extending or deleting code. “Legacy” means outside the
active deployment boundary; it does not mean scientifically invalid.

## Maintained production surfaces

### `robot-infra`

- `src/control_interface`: ROS contracts, manager, serial transport, limits,
  freshness, qualification, and safety tests.
- `src/catheter_control/catheter_control/node.py` and the canonical
  `orchestration`, `planning`, `transmission`, `safety`, `simulation`, and
  `applications` subpackages: active controller runtime.
- `src/catheter_control/launch/control.launch.py`: hardware controller launch;
  command output defaults to disabled.
- `src/catheter_control/launch/simulation.launch.py`: isolated `/sim` stack.
- `src/catheter_control/config/v175_grouped_hardware_no_rotation.yaml`: the
  qualified controller profile for the recorded scope.
- `src/automation/automation/perception`: active online marker and legacy
  state-estimation adapters pending a ROS perception-package split.
- `src/automation/automation/experiments` and `supervision`: maintained
  experiment, collection, identity, and qualification infrastructure.
- `tools/maintenance` and `audits/ros-realtime`: reproducible inventory,
  closure, replay, and audit tooling.

### Other repositories

- `cr_meta_lnn/deployment/v171_*.py`,
  `interface_transmission_checkpoint.py`, and
  `artifact_manifest.py`: supported runtime/model loading surface.
- `cr_meta_lnn/artifacts/manifests`: reviewed artifact metadata.
- `control/control/adapj.py`: adaptive forward Jacobian.
- `cr-common/utils.py`: shared deployment utility required by the runtime.
- `catheter-shape-tracking/src/shape_tracking`: Linux capture, online marker
  support, and offline reconstruction used by the active workflow.

## Maintained experiment and reproduction surfaces

These are supported for data collection, diagnosis, or model reproduction but
are not imported by the controller's production path:

- reviewed `robot-infra` experiment YAMLs and task clients;
- `cr_meta_lnn` v171/v174 reproduction scripts committed in M3;
- v175 interface-transmission fitting and validation committed in M3;
- engagement/gain pipeline scripts committed in M3;
- shape-tracking session post-processing and overlay CLIs.

A reproduction script may retain historical filenames. It must not silently
become a runtime default.

## Compatibility-only surfaces

- Legacy checkpoint locations under `cr_meta_lnn/checkpoints` and
  `cr_meta_lnn/evaluation` are compatibility symlinks declared by the selected
  artifact manifest; maintained runtime consumers use canonical bundle paths.
- Existing ROS console entry-point names remain stable while `automation` is
  split.
- Pre-Phase-3 `catheter_control` root modules and `automation.collection`,
  `automation.estimation`, and `automation.marker_tracking` modules are thin
  import/CLI compatibility shims; new code imports canonical subpackages.
- Versioned hardware and experiment YAML names remain addressable as
  compatibility aliases for the semantic stacks introduced in Phase 2.
- Historical absolute paths in audit/session manifests are immutable evidence,
  not templates for new code.

## Research history

The large uncommitted `cr_meta_lnn` research tree is preserved by branch
`archive/pre-cleanup-research-20261001`. Older model families, broad
diagnostic sweeps, generated checkpoints, and evaluation products are not part
of the active runtime merely because they remain on disk.

Before promoting research code:

1. identify its owning package;
2. add a narrow public API and tests;
3. declare every required artifact;
4. pass clean-source replay and simulation gates;
5. update the active deployment baseline explicitly.

## Archived local backups

Four ignored `.orig` files were compared with their maintained counterparts
on 2026-10-01. They contained no unique newer work and were moved, not deleted,
to:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  repository_cleanup_archive_20261001/ignored_backups/
```

`SHA256SUMS` in that directory verifies all four files. Do not recreate
`.orig` files inside active source trees; use Git branches or the external
cleanup archive for preservation.

## Retirement rule

Code may leave the active branch only after its callers are removed or routed
through a qualified compatibility layer, relevant history is preserved, and
the clean-source gate still passes. Generated data must not be committed as a
substitute for a manifest.
