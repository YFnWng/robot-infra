# Catheter ROS real-time audit workspace

Audit date: 2026-09-11

Scope: the native-Ubuntu closed-loop path spanning `catheter-shape-tracking`,
`robot-infra`, `cr_meta_lnn`, `control`, and the Teensy firmware. The audit is
observational. It did not arm the controller, publish commands, call services,
open the serial device, or send `SET_ZERO`.

## Safety invariant

The hardware encoder zero is aligned with the learned-model training data.
`SET_ZERO` must never be sent. The host code rejects it, but the firmware still
implements the `ZERO` opcode; see finding F-002.

## Reviewed components

| Component | Runtime | Role |
|---|---|---|
| `automation/marker_tracking` | Python, single ROS executor thread, two camera threads, four processing workers | Capture, synchronize, triangulate, and fuse four markers |
| `catheter_control/catheter_mppi` | Python in `cr-venv`, eight ROS executor threads, PyTorch/NumPy | Estimation, safety gates, MPPI planning, command heartbeat |
| `control_interface/manager.py` | Python, four ROS executor threads | Control-mode arbitration, limits, host watchdogs, qualification |
| `control_interface/device_serial_com.py` | Python, four ROS executor threads plus RX thread | ROS/serial framing and firmware request matching |
| `teensy_tekceleo.ino` | Teensy firmware | Encoder acquisition, motor-driver communication, final watchdog |

## Method and evidence level

The audit traced source-level publishers, subscriptions, timers, callback
groups, locks, queues, timestamps, gating transitions, and watchdogs. A
read-only live graph query found no running application nodes, so current
callback-duration and end-to-end latency distributions were not measured.

Findings use these evidence labels:

- **Observed:** directly present in source/configuration or returned by a command.
- **Inferred:** follows from the observed execution structure but needs a live trace to quantify.
- **Historical:** previously reported runtime behavior, not re-measured in this audit.

Generated artifacts are in this directory. GraphViz sources are the canonical
diagrams; SVG files are rendered copies.
