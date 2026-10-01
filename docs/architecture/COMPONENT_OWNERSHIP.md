# Component ownership

This map defines where maintained catheter-stack behavior belongs. It is a
change-routing contract, not a claim that every current file has already moved
to its final package.

| Concern | Owning repository/package | Notes |
| --- | --- | --- |
| ROS messages, services, and actions | `robot-infra/control_interface` | Language-neutral contracts only |
| Manager arbitration and freshness | `robot-infra/control_interface` | Final ROS-side command authority |
| Serial framing and device telemetry | `robot-infra/control_interface` | Must reject `SET_ZERO`; firmware remains final authority |
| MPPI, belief integration, and estimator scheduling | `robot-infra/catheter_control` | ROS composition around model runtime |
| Simulation device, plant, perception, and RViz | `robot-infra/catheter_control` | Simulation-only endpoints remain under `/sim` |
| Experiment/task clients | future `robot-infra/catheter_experiments` | Currently split between `catheter_control` and `automation` |
| Marker tracking ROS publisher | future `robot-infra/catheter_perception` | Currently under `automation`; consumes tracking outputs |
| Session identity, collection, and qualification | future `robot-infra/catheter_experiments` | Currently under `automation` |
| Manual operator UI | `robot-infra/teleop` | Not a model or autonomous controller |
| Learned mechanics and causal state | `cr_meta_lnn` | Includes v171 distal runtime and play/transmission loaders |
| Estimator rewind/correction/replay implementation | `cr_meta_lnn/deployment` | ROS callback ownership remains in `catheter_control` |
| Model artifact manifests | `cr_meta_lnn/artifacts/manifests` | Binaries remain external to Git |
| Adaptive forward Jacobian algebra | `control/control/adapj.py` | Reuse; do not duplicate in ROS nodes |
| Shared kinematics/model utilities | `cr-common` | Must remain importable without optional GTSAM |
| Camera capture and offline shape reconstruction | `catheter-shape-tracking` | Owns dual-ZED synchronization and observation quality |
| Motor safety and low-level motion | Teensy firmware plus ROS manager | Never bypass manager or firmware safeguards |

## ROS package direction

`catheter_control` is autonomous runtime code and is logically related to
parts of `automation`, but merging the two current packages would preserve
the wrong boundary: `automation` mixes perception, experiments, collection,
and identity tooling. The target is therefore to split `automation` by
responsibility and retain compatibility entry points during migration:

```text
control_interface
├── interface contracts
├── manager / command arbitration
└── serial transport

catheter_control
├── controller composition
├── estimator scheduling
├── MPPI and belief integration
└── simulation

catheter_perception
└── online marker observations and diagnostics

catheter_experiments
├── task clients and experiment schedules
├── session/runtime identity
└── collection and qualification
```

The split is structural only. Topic names, service names, source stamps,
freshness rules, and fail-closed behavior remain unchanged until separately
reviewed.

## Python/C++ boundary

Python remains the reference implementation and development surface. Stable
production components move behind language-neutral snapshots/results and are
mirrored in C++ only after fixture and replay conformance. Recommended order:

1. command freshness and immutable snapshots;
2. backlash/engagement/gain belief transitions;
3. MPPI sampling, projection, costing, and reduction;
4. estimator orchestration after numerical conformance is available;
5. ROS lifecycle and executor shell.

Firmware migration is not part of the current cleanup. Teensy-side computation
is a future option only for bounded safety-critical functions with an explicit
protocol and host fallback.
