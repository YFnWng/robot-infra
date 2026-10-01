# v175 no-rotation two-axis sparse audit: 20260920_171702_mppi_sim

## Outcome

The final four-point block passed the simulation gate. All four independent
targets entered tolerance with logical rotation identically zero. The bag also
contains one earlier successful point from the preceding attempt that then
stopped at the stricter home-tolerance check; it is not counted as part of the
final four-point block.

| Final-block target | Type | Initial error (mm) | Final feedback error (mm) | Minimum error (mm) |
| ---: | --- | ---: | ---: | ---: |
| 1 | insertion dominant | 4.042 | 0.293 | 0.053 |
| 2 | tendon dominant | 3.702 | 1.247 | 1.141 |
| 3 | positive insertion/tendon diagonal | 2.409 | 1.267 | 1.188 |
| 4 | negative insertion/tendon diagonal | 6.891 | 1.172 | 0.759 |

Every action accumulated six in-tolerance feedback samples. No controller
fault occurred. Status remained `TRACKING` during active control, adaptation
was disabled, and the recorded controller limit was
`[10,0,4.5,4,25,25]`.

## Rotation and command evidence

- Maximum absolute logical rotation in all 43 recorded planned commands:
  exactly `0.0`.
- Final-block logical command ranges were:
  - target 1: insertion `8.36..9.99`, tendon `0..2.00`;
  - target 2: insertion `-10.0..6.08`, tendon `2.00..4.50`;
  - target 3: insertion `2.02..10.0`, tendon `0..2.02`;
  - target 4: insertion `-9.55..0`, tendon `0..4.50`.
- One recoverable planner-deadline zero was recorded; there was no repeated
  deadline fault.

The returned POS state was measured rather than assumed. The final-block home
starts were close to `[20,0,0]`, with tendon residuals approximately
`0.080..0.100`; this is why the runner's qualified home tolerance is 0.10.

## Model-response evidence

There were 24 causal response traces. Model endpoint error was:

- median: `0.252 mm`;
- P95: `0.482 mm`;
- maximum: `0.495 mm`.

Valid model/measurement direction cosine had median `0.999` and minimum
`0.704`. This supports the internal model-generated target construction in
the idealized simulation plant. It does not validate hardware transmission,
hidden-history reset, or physical reachability.

## Hardware-startup observation collected immediately afterward

This section records a separate live hardware snapshot and is not part of the
simulation result.

- `/device/state` was observed publishing both POS (`predicate 80`) and ENC
  (`predicate 69`) at a combined rate near `181.8 Hz`.
- The sampled POS was
  `[12.1714,0,3.84185,0,0.0135,-0.0135]`; ENC was
  `[19797,0,-25814,0,4,0]`.
- The retained transport status was `SERIAL_OPEN:/dev/ttyACM0`, not
  `SERIAL_READY:/dev/ttyACM0`.
- Manager safety was
  `MANAGER_INHIBITED:MOTION_CONFIRMED:ENCODER_INTEGRITY`.

Therefore the user's earlier “no bidirectional POS/ENC telemetry” response is
not a current absence of POS/ENC bytes. The link currently carries both. The
manager can nevertheless remain transport-not-ready because readiness is set
only from `/device/transport_status`.

## Finding H-001: serial-ready publication race

Severity: high
Confidence: inferred-high-confidence from source plus live state

`device_serial_com.connect()` exposes the newly opened `serial_port` to its RX
thread and only afterward publishes `SERIAL_OPEN`. At high-rate firmware
telemetry, the RX thread can consume a valid POS/ENC frame, set `rx_ready`, and
publish `SERIAL_READY` between those operations. The connect callback then
publishes `SERIAL_OPEN` last. Because `rx_ready` is already set, subsequent
frames do not republish `SERIAL_READY`. The manager subscribes to the retained
status and consequently sees open-but-not-ready even while both feedback
streams are arriving.

Source evidence:

- `src/control_interface/control_interface_py/device_serial_com.py`:
  `connect()`, `_mark_rx_ready()`, and the transient-local status publisher;
- `src/control_interface/control_interface_py/manager.py`:
  `transport_status_callback()` and `qualify_driver_power()`.

The manager's existing fail-closed behavior is correct. Do not fabricate a
`SERIAL_READY` topic message or bypass the transport gate. The correction
should make bridge state publication monotonic (OPEN before exposing the port,
then READY after the first valid stream), and should add a regression test for
the interleaving.

## Finding H-002: independent firmware encoder-integrity latch

Severity: high
Confidence: observed

The manager is separately inhibited by a confirmed firmware encoder-integrity
fault. Even after the transport-status race is repaired, motion must remain
disabled until the existing stationary driver-power qualification validates
the fault class, performs only the supported retained-frame restoration, and
rechecks stable POS/ENC. This finding does not authorize a general fault reset
or any encoder-zero change.
