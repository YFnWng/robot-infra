# Active deployment baseline

This is the authoritative entry point for the catheter control deployment
baseline. Historical closure and session documents are evidence, not competing
runtime definitions.

## Identity

- Shared source tag: `catheter-stack-pre-cleanup-20261001`
- Qualified hardware anchor: `20260929_175554_mppi_demo`
- Qualification scope: four sparse two-axis targets, rotation disabled
- Controller profile: `v175_grouped_hardware_no_rotation.yaml`
- Task profile: not recoverable from the historical session manifest; do not guess it
- Marker estimator: UKF
- Model adaptation: disabled
- MPPI: grouped sampling, 512 stochastic samples, four 0.20 s point steps
- Take-up: response-terminated compensation with engaged-gain belief
- Hardware command output: disabled by default and enabled only by the explicit
  `control.launch.py command_output_enabled:=true` interlock

The recorded qualification used hardware output. The tag and this document do
not authorize a hardware run.

## Source components

| Repository | Responsibility | Baseline tag target |
| --- | --- | --- |
| `robot-infra` | ROS interfaces, controller integration, manager, transport, safety, launch, and simulation | `39a30090658e834ce65554f953faf1ab88453da2` |
| `cr_meta_lnn` | v171 runtime, estimator/model state, and artifact loaders | `ae8752b7e140ad913ca2e742b99f5c4da348e01b` |
| `catheter-shape-tracking` | synchronized dual-rig capture and marker observations | `56980d0d6a382a29d97218dd3a5de6f0793e519f` |
| `control` | `AdaptiveForwardJacobian` implementation | `af8b10d2b5bf885e4daf5365a0013e3ca30b51f1` |
| `cr-common` | shared kinematic/model utilities | `db5cdd464c850b07f0a159bc3e984212104ab435` |

## Model artifacts

The portable source of truth is
`cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation.json`.
It declares the v171 distal checkpoint, v174 Jacobian initialization, and v175
interface-transmission checkpoint by relative path, byte size, SHA-256 digest,
loader, and role. Binary files remain external to Git.

Do not select an artifact by “latest” filename or directory order. A deployment
must select a reviewed manifest and verify every file before launch.

Qualified binaries are installed in the manifest-owned
`cr_meta_lnn/artifacts/deployed/20260929_175554_grouped_no_rotation` bundle.
Maintained runtime defaults and profiles use those canonical paths. The
manifest declares reviewed legacy paths, and its fail-closed alias materializer
creates relative symlinks for historical reproduction tools without duplicating
artifact bytes.

## Runtime boundaries

- Learned mechanics, estimator state, rewind/replay, and rollouts belong in
  `cr_meta_lnn`.
- ROS scheduling, lifecycle, messages, command arbitration, and safety
  integration belong in `robot-infra`.
- The adaptive Jacobian algebra remains in `control.adapj`; ROS code must not
  duplicate it.
- Camera acquisition and marker reconstruction remain in
  `catheter-shape-tracking`.
- The manager and firmware remain the final command and limit authorities.
- Physical encoder zero is a read-only calibration reference. `SET_ZERO` is
  forbidden in manager and serial paths.

## Qualification evidence

- [Clean-source cross-repository gate](../../audits/ros-realtime/cross-repository-clean-source-gate-20261001.md)
- [Historical active runtime closure](ACTIVE_RUNTIME_CLOSURE_20260929.md)
- [Machine-readable historical session baseline](production_baselines/20260929_175554_grouped_no_rotation.json)
- `cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation.json`

Any change to model equations, artifacts, controller costs, belief thresholds,
scheduling, ROS interfaces, or safety behavior creates a new candidate
baseline and requires the corresponding conformance gates. Structural moves
may retain this identifier only when replay and simulation remain equivalent.
