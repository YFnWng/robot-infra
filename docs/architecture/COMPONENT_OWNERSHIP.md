# Component ownership

This map defines where maintained catheter-stack behavior belongs. It is a
change-routing contract, not a claim that every current file has already moved
to its final package.

| Concern | Owning repository/package | Notes |
| --- | --- | --- |
| ROS messages, services, and actions | `robot-infra/control_interface` | Language-neutral contracts only |
| Manager arbitration and freshness | `robot-infra/control_interface` | Final ROS-side command authority |
| Serial framing and device telemetry | `robot-infra/control_interface` | Must reject `SET_ZERO`; firmware remains final authority |
| Streamed encoder-position contract | `control_interface_py/streamed_position.py` plus firmware `streamed_position_protocol.h`/`streamed_position_session.h` | M2 opt-in session/deadline envelope and capability-gated ROS/serial dispatch; default disabled |
| Streamed endpoint execution | Firmware `streamed_position_executor.h` and `native_position_command.h` | Native driver owns endpoints; shared stop-confirm/session/lease supervision in sketch and simulation; legacy homing unchanged |
| Bounded encoder/native position hardware probes | `control_interface_py/position_stream_probe.py` and `native_position_probe.py` | Explicit execution only; legacy POS stop-confirm probes plus separately default-off, session-bound one-shot live A-update diagnostic; no MPPI live retargeting |
| Driver command evidence and stopped readback | Firmware `driver_evidence.h`, serial/manager owners, Python `driver_evidence.py` | Fixed UART capture buffer, disabled-only trace drain, guarded read-only DA/DS through existing H transport; no calibration changes |
| Explicit driver-communication fault recovery | `control_interface_py/manager.py` | Guarded operator service; disabled/stationary and UART checks before class-6-only reset, clean requalification afterward; no auto-resume |
| Fixed encoder/shaft/logical coordinate conversions | `control_interface_py/encoder_coordinates.py` | Read-only constants/coupling reused by manager and controller HardwareContract; projection policy remains controller-owned |
| Shared hardware limits | `robot-infra/control_interface/config` | One canonical safety contract |
| Launch composition | `robot-infra/bringup` | Functional packages own runtime implementation |
| MPPI, belief integration, and estimator scheduling | `robot-infra/catheter_control/{planning,transmission,orchestration}` | `node.py` is the Python reference composition root |
| C++ ROS shell and production migration | `robot-infra/control_cpp` | Shadow-only until separate authority review; no command publisher in the initial node |
| Safety projection and lifecycle gates | `robot-infra/catheter_control/safety` | Manager and firmware remain final authorities |
| Simulation device, plant, perception, and RViz | `robot-infra/simulation` | Simulation-only endpoints remain under `/sim` |
| Experiment schedules and guarded collection | `robot-infra/experiments` | Standalone functional package |
| Controller task clients | `robot-infra/control_tasks` | Depends one-way on the controller core |
| Marker tracking and live shape adapters | `robot-infra/perception` | Extracted from the historical `automation` package |
| Session identity and qualification | `robot-infra/runtime_supervision` | Includes offline recorded-input compute profiling and non-actuating recorded-state take-up scoring replay; depends one-way on controller APIs |
| Cross-repository numerical conformance | `robot-infra/catheter_control/safety/conformance.py` | Offline only; owns the frozen fixture and never publishes commands |
| Manual operator UI | `robot-infra/teleop` | Not a model or autonomous controller |
| Learned mechanics and causal state | `cr_meta_lnn` | Includes v171 distal runtime and play/transmission loaders |
| Experimental coarse/fine rollout diagnostic | `cr_meta_lnn/deployment/experimental/engagement_rollout_check.py` | Packaged non-actuating API; historical training import re-exports it |
| Estimator rewind/correction/replay implementation | `cr_meta_lnn/deployment` | ROS callback ownership remains in `catheter_control` |
| Control-route tendon confirmation, widths and handoff | `robot-infra/catheter_control/transmission/backlash.py` | Baseline and experimental control share `observe_at_timestamp`; model filter consumes source-time decisions, not a second detector. Standalone research detector is a separate policy. |
| Model artifact manifests | `cr_meta_lnn/artifacts/manifests` | Selected qualified bytes ship in the deployment wheel; research outputs remain external |
| Adaptive forward Jacobian algebra | `control/control/adapj.py` | Reuse; do not duplicate in ROS nodes |
| Shared kinematics/model utilities | `cr-common` | Must remain importable without optional GTSAM |
| Camera capture and offline shape reconstruction | `catheter-shape-tracking` | Owns dual-ZED synchronization and observation quality |
| Motor safety and low-level motion | Teensy firmware plus ROS manager | Never bypass manager or firmware safeguards |

## ROS package direction

The former `automation` package mixed perception, experiments, collection,
and identity tooling. Those responsibilities now live in separate ROS packages,
and the historical package has been retired:

```text
control_interface
├── interface contracts
├── manager / command arbitration
└── serial transport

bringup
└── launch composition and deployment entry points

catheter_control
├── Python reference controller composition
├── estimator scheduling
└── MPPI and belief integration

control_cpp
├── non-commanding shadow shell
├── versioned result validation
└── isolated heartbeat and timing executors

simulation
└── isolated plant, device, perception, scenarios, and RViz

control_tasks
└── action servers and task clients

perception
└── online marker observations and diagnostics

experiments
├── experiment schedules
└── guarded data collection

runtime_supervision
└── session identity and qualification
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
