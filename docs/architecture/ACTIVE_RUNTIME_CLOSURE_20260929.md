# Active full-stack runtime closure — 2026-09-29 qualification

This closure is anchored to the successful hardware session:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  20260929_175554_mppi_demo
  20260929_175554_mppi_demo_manifest.json
```

The session reached all four sparse two-axis targets with rotation disabled.
It is evidence for the grouped, gain-adaptive, response-terminated take-up
pipeline in that scope. It is not qualification of three-axis or continuous
path control.

The machine-readable closure is:

[`production_baselines/20260929_175554_grouped_no_rotation.json`](production_baselines/20260929_175554_grouped_no_rotation.json)

## Resolved controller identity

- profile: `v175_grouped_hardware_no_rotation.yaml`
- marker estimator: UKF
- model adaptation: disabled
- device: CUDA
- MPPI samples: 512
- horizon: four coarse 0.20 s steps for point targets
- grouped mode sampling: enabled
- engaged-gain scenarios: enabled
- response-terminated take-up: enabled
- command output in the recorded qualification: enabled

The current controller profile hash matches the recorded session hash.

## Qualified artifact identity

| Artifact | Bytes | SHA-256 | Current match |
| --- | ---: | --- | --- |
| v171 distal checkpoint | 4,682,199 | `adfbe11f58d409c9a24fdbdef23ef38b32c83fcc692ef77fa68c52508254ac1a` | yes |
| v174 Jacobian JSON | 97,188 | `6d3f8573d10511dcbea3c27c97141f132b95cc42c450d3aec3702db33430ad2c` | yes |
| v175 interface transmission | 6,422 | `c07960295808ccfe34a79297630a06e6edd53e71181859b2831de0fef700cfc7` | yes |
| insertion-conditioned tendon allocation | — | — | not selected |

No artifact drift was found relative to the recorded session manifest.

## Static full-stack closure

The conservative source closure contains 111 files:

| Repository | Files |
| --- | ---: |
| `robot-infra` | 56 |
| `cr_meta_lnn` | 33 |
| `catheter-shape-tracking` | 10 |
| `control` | 9 |
| `cr-common` | 3 |

Of these files, 84 are tracked and 27 are untracked. There are no unresolved
local Python imports in the static scan.

The closure includes:

- `catheter_mppi`, trajectory/path action nodes, and sparse-point task client;
- online marker tracking;
- manager and serial bridge;
- ROS message, service, and action definitions;
- v171 runtime/loader and the selected v175 interface-transmission loader;
- adaptive Jacobian and shared kinematics utilities;
- the controller profile, hardware limits, package metadata, and launch files.

## Import-time coupling

Commit `4b210cd` made the production deployment package v171-only and lazy.
Importing `V171StreamingCatheterRuntime` no longer imports the legacy v150
runtime; an isolated-snapshot regression test enforces that boundary. This
reduced the conservative closure from 115 to 111 files.

Two broader package-level couplings remain:

1. importing `cr_meta_lnn.networks.hybrid.<module>` executes the broad
   `networks/__init__.py` and `networks/hybrid/__init__.py` registries;
2. importing `control.adapj` executes the broad `control/__init__.py` surface.

The runtime source also contains a conditional loader for the optional
insertion-allocation artifact even though the qualified session did not select
it. That loader remains in the closure intentionally so enabling the declared
runtime option cannot target an absent module.

## Provenance gap

`control.launch.py` records the resolved controller parameters and model
artifacts, but it does not record:

- the exact sparse-point task YAML and command line;
- independently launched marker-tracking parameters;
- independently launched manager/serial parameters;
- source commits or dirty-worktree file hashes for the other processes.

The source executables are conservatively included here, but a future unified
full-stack session manifest must record every launch/profile and repository
identity. The task YAML for this historical run is intentionally not guessed.

## Baseline status

This is a content-hash closure, not yet a clean-clone baseline. All five source
repositories remain dirty, and 27 closure files are still untracked.
The manifest is sufficient to review and stage production files, but not to tag
a reproducible release until those files are committed and replay-tested.
