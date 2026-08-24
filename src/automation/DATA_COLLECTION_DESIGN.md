# Automated Data Collection — Design

Status: **design only** (no implementation yet).
Scope: drive the catheter robot through continuous excitation trajectories while
recording motor encoders, EM tracking, and ZED stereo video, so the data can be
fused offline into supervised 3-D catheter-shape training data.

Related components:
- `robot-infra` ROS2 workspace (this repo): `teleop`, `control_interface`, `automation`.
- `ros_ws_base/ros2_igtl_bridge` — OpenIGTLink ↔ ROS2 bridge.
- 3D Slicer (Windows): `CardiacCatheterRobot` (GUI), `AuroraTracker` (EM).
- `sofa-cosserat-sim/data_collection/generators` — reused trajectory generators.
- `shape_tracking` (Windows) — ZED recorder + offline stereo reconstruction.

---

## 1. Guiding principles

1. **Record raw, learn the rest.** The authoritative control input we log is the
   **raw motor encoder** stream. Transmission ratios (encoder → joint → catheter
   shape) are *learned by the model*, never injected as known constants.
2. **Velocity control, continuous motion.** Command joint **velocities**, not
   positions. Position setpoints cause stop-and-go dynamics we do not want to
   study. Motion is smooth and persistently exciting.
3. **Asynchronous, maximum-rate acquisition.** Every sensor samples at its own
   native maximum rate with its own timestamps. Nothing is gated on a common
   trigger or down-sampled online. Streams are aligned **offline by
   interpolation** onto a common timeline.
4. **Reuse the simulation generators.** The same `InputGenerator` /
   `SinusoidalGenerator` that drive the SOFA sim drive the real robot, so sim and
   real excitation are identical.

---

## 2. System topology

```
 WINDOWS                                        WSL Ubuntu 22.04 (ROS2 Humble)
 ┌───────────────────────────────┐            ┌───────────────────────────────────────────────┐
 │ NDI Aurora ─► Slicer            │            │ ros2_igtl_bridge (server :18944)              │
 │   AuroraTracker (≤40 Hz)        │  IGTL      │   /IGTL_TRANSFORM_IN  (igtl Transform)         │
 │   vtkMRMLLinearTransformNode ───┼───TRANSFORM─►     │                                         │
 │                                 │            │   [NEW] em_bridge ─► /em_tracker/poses        │
 │ Slicer CardiacCatheterRobot GUI │◄──IGTL────►│   teleop/slicer  (manual teleop, unchanged)   │
 │                                 │            │                                               │
 │ ZED2 (shape_tracking, MANUAL)   │            │   [NEW] automation/collection node            │
 │   SVO2 @ native fps + ts        │            │     import SinusoidalGenerator (SOFA repo)     │
 │                                 │            │     velocity cmd ─► /teleop/control (autonomy) │
 │                                 │            │   control_interface: manager ─► serial ─► MCU  │
 │                                 │            │     /device/state  (POS/VEL/ENC @ serial rate) │
 │                                 │            │   [NEW] rosbag2 (mcap): all topics, async     │
 └───────────────────────────────┘            └───────────────────────────────────────────────┘
        └──────────── OFFLINE: clock-align (motion cross-correlation) + interpolate ───────────┘
```

Robot is **6-DOF**: `[catheter_lin, catheter_rot, catheter_bend, sheath_lin,
sheath_rot, sheath_bend]`. A collection run drives **either** the catheter **or**
the sheath (3-DOF, chosen by CLI param); the other three are commanded zero
velocity (held).

---

## 3. Control approach

### 3.1 Command path (velocity)
The `manager` already supports velocity mode:
`ControlStream.joint_vel` on `/teleop/control` → (mode `JOINT_VEL`) →
`DeviceStream{predicate=VEL, data}` on `/manager/control` → serial → MCU.

The collection node therefore:
1. Sends `ManagerEvent{predicate=MODE, text="3"}` on `/teleop/event` to select
   `JOINT_VEL` (see `ManagerEvent` constants), and asserts the run is enabled
   (deadman off, estop off).
2. Publishes `ControlStream` with `header.frame_id = "autonomy"` (the manager
   arbitrates a single `active_source` by `frame_id`).

### 3.2 Velocity from the position generator
`SinusoidalGenerator.step(t)` returns a **position** `q(t)=[insertion,rot,cable]`.
We command the analytic/finite-difference **velocity**:

```
v_ff(t) = shortest_rotation_delta( step(t+dt), step(t) ) / dt      # base.py helper
```

`shortest_rotation_delta` handles the rotation-joint wrap (index 1). Only the 3
target-joint components are used; the other 3 (the non-driven catheter or sheath
set) are commanded `0.0`.

### 3.3 Drift / limit safety (keep it continuous)
Open-loop velocity integrates in hardware, so the true position may drift.
To avoid limit hits **without** introducing stop-and-go:
- **Per-catheter limits** live in `config/catheter_limits.yaml`: named profiles,
  each with 6-joint `pos_lower`/`pos_upper` (generator sweep range) and `vel_max`
  (per-joint speed cap). Select with the `catheter` param; the node slices the
  driven 3 joints. **Units are mm/deg** (matching the firmware), NOT metres.
  Overrides the `joint_lower/upper` + `joint_max_speeds` params when set.
- The generator is speed-aware (frequencies derived from `vel_max`) and stays
  within `[pos_lower, pos_upper]` with margin.
- MCU-side `LIMIT`/`STALL` events (`DeviceEvent`) hard-stop as a backstop.
- **`expect_enc`** (default true): the node warns at 3 s and at run-end if no raw
  `ENC` frames arrive on `/device/state` — catches an un-flashed MCU before it
  silently records transmission-baked `POS` instead of raw encoders.
- Optional gentle feed-forward + soft correction using the **nominal** `POS`
  stream (if the MCU streams it): `v = v_ff + k*(q_gen - q_pos_nominal)`, `k`
  small. This is for *safety only*; the learned model still uses raw `ENC`.

### 3.4 Generator reuse
Imported directly from the SOFA repo (decision: import, don't vendor):
```
SOFA_SIM_PATH=/home/wangyf/sofa-cosserat-sim   # prepended to PYTHONPATH by launch
from data_collection.generators.sinusoidal import SinusoidalGenerator
```
Joint order matches `[insertion, rotation, bend]`; the generator is unit-agnostic
(it produces whatever units the limits are in). For the real robot we use the
firmware's **mm/deg** (so `joint_vel` is mm/s, deg/s) — see `catheter_limits.yaml`
— NOT the SOFA sim's metres.

---

## 4. Data flow and formats

Listed hop-by-hop, from hardware to disk.

### 4.1 MCU ↔ host serial (control_interface `device_serial_com`)
Framed binary over `/dev/ttyACM0` @ 115200 baud:
```
[ '<' ][ len:uint8 ][ prefix:uint8 ][ payload ][ '>' ]
```
Stream frames (`prefix ∈ {POS='P'(80), VEL='V'(86), ENC='E'(69)}`) carry
`payload = 6 × float32 (little-endian)` = 24 bytes.

> **FIRMWARE (teensy_tekceleo.ino):** at 100 Hz the MCU streams TWO frames on
> `/device/state`:
> - **`POS`** — 6 **logical joint positions** derived from encoder counts.
>   For both identical differential handles, logical bend is
>   `bend_motor - rotation_motor`. These values drive teleop, safety, and
>   logical joint limits.
> - **`ENC`** — 6 **raw encoder counts, uncoupled, int32** (added 2026-07-05).
>   No transmission/coupling applied → **this is the model input** (learns the
>   mapping). Parsed on the ROS side as `<6i` (POS/VEL stay `<6f`), stored exactly
>   as float64. Requires flashing the updated firmware.

Command frames (host→MCU) are `prefix + 6×float32` written by
`on_manager_control`.

### 4.2 control_interface ROS 2 messages

| Topic | Type | Fields (key) | Notes |
|---|---|---|---|
| `/teleop/control` | `control_interface/ControlStream` | `header`, `float64[] joint_vel`, `float64[] joint_pos`, cartesian… | command; automation fills `joint_vel` (6), `frame_id="autonomy"` |
| `/teleop/event` | `control_interface/ManagerEvent` | `header`, `uint8 predicate`, `string text`, `uint8[] state`, `float64[] data` | mode select (`MODE`,`"3"`), triggers |
| `/manager/control` | `control_interface/DeviceStream` | `header`, `uint8 predicate` (VEL/POS/ENC), `float64[] data` | manager → device driver |
| `/device/state` | `control_interface/DeviceStream` | `header`, `uint8 predicate`, `float64[] data[6]` | firmware sends **`POS`** (logical joint pos) AND **`ENC`** (raw counts, model input) per cycle; `header.stamp` = ROS receive time |
| `/manager/state` | `control_interface/ManagerStream` | `header`, `float64[] joint_vel`, `float64[] joint_pos` | manager echo (nominal) |
| `/device/event`, `/manager/event` | `DeviceEvent` / `ManagerEvent` | LIMIT/STALL/… | faults, mode changes |

`ManagerEvent` predicate constants (subset): `MODE=77`, `NONE=48`,
`JOINT_VEL=51("3")`, `JOINT_POS=52("4")`, `START_MOTOR=73`, `STOP_MOTOR=83`,
`SET_ZERO=90`, `LIMIT=76`, `STALL=84`.

> **Timestamp caveat.** `/device/state` is stamped on ROS receipt, after serial
> + USB latency (sub-ms to a few ms, with jitter). If tighter encoder timing is
> needed later, add an MCU-side sample counter/timestamp to the `ENC` payload.

### 4.3 EM tracking: Aurora → Slicer → OpenIGTLink → ROS

| Hop | Format |
|---|---|
| Aurora → Slicer AuroraTracker | NDI 6-DOF per coil, ≤40 Hz; positions held in `AuroraFiducialNode` (`vtkMRMLMarkupsFiducialNode`, 5 control points, mm) |
| Slicer → bridge (OpenIGTLink) | one batched IGTL `POINT` message per frame — all coil positions together (keeps their common sample time) |
| bridge → ROS | `ros2_igtl_bridge/msg/PointArray`: `string name`, `geometry_msgs/Point[] pointdata` (mm) on `/IGTL_POINT_IN` (no header) |
| **[implemented] `em_bridge`** → estimator/recorder | `PointArray` → `geometry_msgs/PoseArray` on **`/em_tracker/poses`** (`header.stamp` = arrival time; `poses[i]` = coil i, position **m**, orientation **identity**) |

`em_bridge` params: `input_topic` (`/IGTL_POINT_IN`), `device_name` (IGTL name
filter; logs seen names when empty), `input_units` (`mm`), `num_coils`,
`frame_id` (`aurora`). Poses are in the **Aurora frame**; Aurora↔robot-base and
camera↔robot-base registrations (ChArUco + EM probe, fixtures in fabrication)
compose the frames offline.

> **Positions vs full pose.** The current AuroraTracker stream is
> positions-only, so orientation is identity. When per-coil 6-DOF is needed (e.g.
> a tip sensor), have Slicer additionally emit `TRANSFORM` messages and merge
> them in `em_bridge`; the `/em_tracker/poses` contract already carries
> orientation. Coil index ↔ order is the `PointArray` order; the estimator’s
> `EMCoilROSAdapter` maps it via `frame_to_node`.

### 4.4 ZED stereo (Windows, shape_tracking)
- **`stereo.svo2`** — native ZED recording: rectified L/R + per-frame timestamps
  (ns) + factory calibration. Record with `TIME_REFERENCE.CURRENT` (system clock)
  so frame times are comparable to ROS/host time.
- **`frame_index.csv`** — sidecar `frame_idx,timestamp_ns` for quick offline
  access without decoding the SVO.
- **`left_intrinsics.npz`** — `K, dist, fx, fy, cx, cy, baseline_m, resolution`.

### 4.5 On-disk recording formats
- **rosbag2, `mcap` storage** (self-describing, good at high rate). Recorded
  topics: `/teleop/control`, `/device/state`, `/manager/state`,
  `/em_tracker/poses`, `/manager/event`, `/device/event`, `/collection/events`.
  Each record keeps both the bag receive time and the in-message `header.stamp`.
- **ZED `SVO2`** as above (Windows filesystem).

### 4.6 `/collection/events` and `manifest.json`
`/collection/events` (proposed `std_msgs/String` JSON, or a small custom msg):
markers for `run_start`, `sync_jog`, segment boundaries, `run_end`, each with the
ROS stamp — used to seed offline alignment.

`manifest.json` (one per session):
```json
{
  "session_id": "20260705_1530_catheter_sin",
  "git": {"robot_infra": "<sha>", "sofa_sim": "<sha>", "shape_tracking": "<sha>"},
  "target": "catheter",                       // or "sheath"
  "generator": {"name": "SinusoidalGenerator", "seed": 42, "duration_s": 120,
                "params": { ... as constructed ... }},
  "joint_limits": {"lower": [...6], "upper": [...6]},
  "joint_max_speeds": [5,30,1,5,30,30],
  "command": {"mode": "JOINT_VEL", "rate_hz": 100},
  "streams": {
    "encoder":  {"topic": "/device/state", "predicate": "ENC", "rate_hz_est": 200},
    "em":       {"topic": "/em_tracker/poses", "rate_hz": 40},
    "zed":      {"file": "zed/stereo.svo2", "resolution": "HD1080", "fps": 30}
  },
  "clock": {"sync": "ptp-phc", "rms_offset_us": 8, "notes": "chrony refclock PHC /dev/ptp_hyperv"},
  "frames": {"em": "aurora", "camera": "zed_left", "robot": "robot_base"},
  "calibration": {"zed_intrinsics": "zed/left_intrinsics.npz"}
}
```

---

## 5. Sampling rates and asynchronous recording

| Stream | Native max | Planned | Limited by |
|---|---|---|---|
| Encoder (`ENC`) | ~serial-bound | as fast as MCU streams | 115200 baud, 26 B/frame → few-hundred Hz; raise baud/USB-CDC for more |
| EM poses | **40 Hz** | 40 Hz | Aurora hardware cap |
| ZED2 video | VGA@100, HD720@60, HD1080@30, HD2K@15 | pick per run | resolution↔fps trade-off |
| Joint velocity command | n/a (input) | ~100 Hz | smoothness, not a sensor |

**Does the plan achieve the highest rate? Yes — by construction:**
- No online sync/trigger. Each source records at its own maximum: rosbag2 stores
  every message as it arrives; ZED writes SVO at its fps. Adding a sensor never
  slows the others.
- Encoders are the fastest, richest input and are logged raw — good, since they
  are the model input.
- EM is hard-capped at 40 Hz; nothing on our side can raise it.
- ZED: **choose resolution vs fps per run.** For dynamic/rate-critical data use
  HD720@60 (or VGA@100); for maximum reconstruction fidelity of the thin catheter
  use HD1080@30 (what `shape_tracking` is tuned for). Record the choice in the
  manifest. (Raising camera rate above EM’s 40 Hz is still useful: video frames
  become the dense timeline everything else interpolates onto.)

**Practical rate boosters (future, optional):** raise the MCU serial baud / move
to USB-CDC for higher encoder rate; add an MCU sample counter for exact encoder
timing.

---

## 6. Time synchronization (offline)

Two clocks (Windows host / WSL2) and three async streams.

**Clock alignment is SOLVED by exact PTP sync** (see `SETUP_TIME_SYNC.md`): WSL's
clock is disciplined to the Windows host clock via chrony + the Hyper-V PHC
(`/dev/ptp_hyperv`) to **~microseconds**. So the ZED (`frame_index.csv`, epoch ns,
host clock) and ROS/rosbag (epoch ns, WSL clock) already share ONE absolute
timeline — no per-session offset/handshake needed. µs error is ~4 orders of
magnitude below a frame interval (16 ms @ 60 fps, 25 ms @ 40 Hz EM).

Remaining alignment steps (now refinements / robustness, not required):
1. ~~Shared clock handshake~~ — **superseded by PTP sync above.**
2. **Sync jog beacon (optional).** A short sharp jog at run start, logged in
   `/collection/events` — a sanity cross-check and a fallback if PTP ever drops.
3. **Motion cross-correlation (fallback / validation).** Cross-correlate the
   video-derived tip motion (`shape_tracking`) against the commanded / EM tip
   motion; the generator's incommensurate sinusoids give a sharp peak. Use to
   validate the PTP alignment or recover sync if the clock lock was lost (e.g. a
   sleep/resume mid-run before chrony re-locked).
4. **Interpolation onto a common timeline.** After alignment, resample every
   stream to query times (e.g. ZED frame times, or a uniform grid):
   - encoder / positions: linear (or zero-order-hold) interpolation,
   - EM rotation: **SLERP**; EM translation: linear,
   - each interpolation carries a validity/staleness flag (gap-aware).

---

## 7. Session directory layout
```
sessions/<session_id>/                 # e.g. 20260705_1530_catheter_sin
  manifest.json
  robot.bag/                           # rosbag2 (mcap): cmd, /device/state(ENC…), EM, events
  zed/
    stereo.svo2
    frame_index.csv
    left_intrinsics.npz
  notes.md                             # optional operator notes
```
Offline, `shape_tracking` reconstructs per ZED frame; encoders/EM interpolate to
those frame times; join on the aligned clock.

---

## 8. New pieces to build (in `automation`, unless noted)

| Piece | Location | Responsibility |
|---|---|---|
| `em_bridge` node | `automation/` (or `control_interface`) | `/IGTL_TRANSFORM_IN` → `/em_tracker/poses` (PoseArray), coil mapping, stamping |
| `collection` node | `automation/collection/` | import generator, target-select, velocity feed-forward, mode/enable, `/collection/events` |
| `collection.launch.py` | `automation/launch/` | compose collection node + em_bridge + `ros2 bag record`; params: `target`, `seed`, `duration`, `zed_*` (metadata only) |
| offline aligner | `shape_tracking` or `automation/offline/` | clock handshake, motion x-corr, interpolation, manifest writer |
| (Windows) ZED sidecar | `shape_tracking` | `frame_index.csv` + system-clock timestamps (manual start) |

`EMCoilROSAdapter` (currently a stub) is completed against the `/em_tracker/poses`
`PoseArray` contract above.

---

## 9. Build order (independently testable)
1. `em_bridge`: verify `/em_tracker/poses` populates from a live Aurora stream.
2. `collection` node driving **one** joint in open-loop velocity; confirm
   `manager` `JOINT_VEL` path + MCU `LIMIT` safety.
3. Swap in `SinusoidalGenerator` (target-select, hold-others=0), add
   `/collection/events` + rosbag2 to the launch.
4. Offline: clock handshake + motion cross-correlation + interpolation + manifest.

---

## 10. Open items to confirm at implementation time
- Exact `ENC` frame **rate** the MCU streams, and whether it also streams `POS`
  (for soft-limit safety).
- Whether encoder values are counts or motor radians (doesn’t change the design —
  recorded raw — but document units per run).
- Aurora transform **names/frames** emitted by Slicer via the bridge, and coil count.
- WSL2↔host clock: solved via PTP sync (`SETUP_TIME_SYNC.md`, ~8 µs RMS); just
  re-check `chronyc sources` after a host sleep/resume before long runs.
- ZED default run profile (HD720@60 vs HD1080@30) as a policy.
```
