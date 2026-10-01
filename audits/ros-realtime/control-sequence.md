# Closed-loop execution sequence

## Feedback and planning

1. Each ZED capture thread stores its latest stereo frame with the ZED image
   timestamp.
2. The marker node's 30 Hz ROS timer selects a synchronized rig pair, processes
   rigs sequentially, fuses the four-marker estimates, and publishes the fused
   estimate. Fusion uses the newest contributing rig timestamp.
3. The serial RX thread decodes firmware `POS` and `ENC` frames and stamps each
   at host receipt; the firmware acquisition time is not transported.
4. The controller subscriptions retain only the newest encoder and marker
   observations. One 50 Hz estimator timer owns all mutable runtime state. It
   drains encoder state first and defers any marker newer than estimator time;
   marker correction remains rate-limited to 20 Hz.
5. The estimator owner publishes replace-only cloned states. The 15 Hz planner
   reads one clone through a small snapshot exchange lock and never shares the
   mutable runtime or waits for marker rewind/replay.
6. The planner checks lifecycle gates, runs NumPy sampling/projection plus the
   batched PyTorch rollout, and commits a command only if the complete path from
   callback entry to commit is within 60 ms.
7. A separate 100 Hz timer republishes the most recently committed command.
8. The manager validates source timestamp age/order, source and mode, clamps
   limits, preserves the source timestamp, and republishes through a reliable
   keep-last depth-1 channel.
9. The serial bridge independently validates the preserved timestamp under a
   serialized transmit lock and writes only a fresh command (or any zero). The
   firmware drains queued host frames before one motor cycle, so only the newest
   parsed target reaches motor-driver settings from its nominal 10 ms loop.

## Gate and stop semantics

- Before control: collection must be absent; manager safety status, POS/ENC,
  marker diagnostics, and accepted marker corrections must be fresh; feedback
  streams must be mutually consistent; estimator must report `TRACKING`; a
  target must exist.
- When first ready, the controller sends a mode event and assumes the mode has
  been claimed after a fixed 100 ms. There is no explicit manager acceptance
  message.
- Once active (or internally mode-claimed), losing a gate faults the controller.
  It publishes a zero and requests manager mode `NONE`.
- Independently, the manager sends zero after 0.5 s without an accepted source
  callback and inhibits motion on stale device feedback.
- Independently, firmware broadcasts all-axis disable after 250 ms without a
  semantically valid `VEL` or `POS` frame. Generic, malformed, and unsupported
  traffic does not refresh motion authority; the check also runs inside driver
  acknowledgement waits.

## Important timestamp domains

| Signal | Stamp meaning |
|---|---|
| Marker estimate | ZED image timestamp; fused estimate uses maximum rig timestamp |
| Encoder/POS | ROS host time when the serial RX thread decoded the frame |
| Controller command | ROS time at command publication |
| Manager command | Original controller/source publication time, preserved by manager |
| Watchdogs | `time.monotonic()` callback-arrival age |

Both ROS motion channels retain one sample. Manager and serial bridge reject a
nonzero command older than 100 ms, more than 50 ms in the future, or no newer
than the last accepted command. A delayed zero remains safe to apply.
