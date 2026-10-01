# ROS2 infrastructure for medical robots

This repository contains the basic software and firmware for research medical robots.

The software is built with ROS2 in Docker for reproducibility. For real time robot control, run in bare metal Linux or WSL.

Currently under construction.

Tested on Windows 11 + WSL2, Ubuntu 22.04 + ROS2 Humble

## Hardened catheter teleoperation

The production path is fail-closed:

- the manager requires the `imricor_test` limits file, fresh POS and ENC
  feedback, a bidirectionally qualified serial link, and explicit driver-power
  qualification before accepting motion;
- serial disconnect, stale feedback, a stale command source, or a confirmed
  firmware fault commands zero/stop and returns the manager to mode `NONE`;
- both motion topics are reliable keep-last depth 1; the original command
  timestamp is preserved through the manager and nonzero commands older than
  100 ms or out of order are rejected again at the serial bridge;
- the firmware grants motion authority only to valid velocity/position frames
  and disables all axes after 250 ms without one; unrelated serial traffic does
  not refresh this watchdog;
- reconnecting always revokes driver-power qualification;
- the Slicer key state is a heartbeat. Loss of that heartbeat first publishes
  zero locally, then the independent manager watchdog disables the mode;
- legacy `START_MOTOR`, raw Slicer fault reset, production debug commands, and
  `SET_ZERO` are rejected. The installed encoder zero is the learned-model
  calibration reference and must not be changed through the production stack.

Build and source the workspace normally, then start the control stack:

```bash
source /opt/ros/humble/setup.bash
source ~/robot-infra/install/setup.bash
ros2 launch control_interface launch.py \
  catheter:=imricor_test \
  serial_port:=/dev/ttyACM0
```

### Full closed-loop ROS node structure

The real-hardware closed loop uses the following primary nodes:

| Node | Responsibility |
| --- | --- |
| `/marker_tracking` | Owns the primary and oblique ZED rigs, synchronizes their frames, detects and triangulates the four markers, and publishes marker positions and tracking diagnostics. |
| `/device_serial_com` | Exchanges framed data with the Teensy: it publishes POS/ENC feedback and firmware events, and sends manager-approved commands to the firmware. |
| `/manager` | Arbitrates command sources and enforces mode, freshness, feedback, joint-limit, transport, and firmware safety gates before forwarding a command. |
| `/catheter_mppi` | Runs the estimator/UKF, delayed-marker rewind and replay, engagement and gain beliefs, learned-model rollouts, MPPI planning, take-up transactions, and the command heartbeat. These are callbacks and workers inside one ROS node, not separate estimator and planner nodes. |
| `/catheter_tip_trajectory` | Sends timed waypoint or sparse-point goals to the controller's trajectory action server. It is idle when no trajectory is requested. |
| `/catheter_tip_path` | Sends continuous paths to the controller's path action server. It is idle when no path is requested. |

Experiment and visualization launches may additionally run
`/catheter_sparse_point_experiment`, `/collection`,
`/catheter_camera_overlay`, RViz, and a rosbag recorder. These orchestrate
tests, visualization, or recording; they are not part of the actuator feedback
loop.

The feedback and command paths are:

```text
VISUAL FEEDBACK
ZED rigs -> /marker_tracking -> /shape_tracking/markers
                               -> /catheter_mppi estimator

ENCODER AND SAFETY FEEDBACK
Teensy -> /device_serial_com -> /device/state
                              -> /catheter_mppi estimator
                              -> /manager safety gates

AUTONOMOUS COMMAND
trajectory/path client -> /catheter_mppi action server
                       -> MPPI plan or take-up command
                       -> /teleop/control
                       -> /manager
                       -> /manager/control
                       -> /device_serial_com
                       -> Teensy firmware -> motor drivers
```

Despite its historical name, `/teleop/control` is the manager's generic
high-level motion-intent input, not a keyboard-only topic. Both Slicer/manual
teleoperation and autonomous MPPI publish `ControlStream` messages there.
The message's `header.frame_id` identifies the source, while the manager owns
source arbitration, mode authorization, safety projection, and forwarding to
`/manager/control`. Renaming this deployed topic would require a coordinated
interface migration; new code should treat it as the command-intent bus.

The supported hardware startup order is:

1. Motor-driver power off.
2. Power/connect the Teensy and start the control stack.
3. Open the serial link from Slicer (or call predicate `67`).
4. Power on the motor drivers while the robot is stationary.
5. Press **Qualify Driver Power** in Slicer. This waits for stable POS/ENC,
   stops and verifies disabled motors, clears only encoder-integrity startup
   latches, and rechecks firmware status.
6. Enable a motion mode only after Slicer reports `MANAGER_READY`.

The diagnostic firmware emits a driver-independent boot heartbeat on
`/device/event` every 500 ms before `CONNECT`. The serial bridge exposes this
one-way state as `SERIAL_FIRMWARE_ALIVE_AWAITING_CONNECT:<build-id>` without
claiming bidirectional readiness. This distinguishes firmware/main-loop
liveness from a completed `SERIAL_READY` handshake. After reflashing, verify
the boot heartbeat with motor-driver power off. Before `CONNECT`, the Teensy
built-in LED also blinks every 500 ms; after `CONNECT` it remains steadily on.
The bridge waits for up to one second for this heartbeat before sending
`CONNECT`:

```bash
ros2 topic echo /device/event control_interface/msg/DeviceEvent \
  --filter 'm.predicate == 66'
```

Terminal equivalents for steps 3 and 5 are:

```bash
ros2 service call /device/command control_interface/srv/DeviceCmd \
  "{predicate: 67, cmd: '/dev/ttyACM0', data: []}"

ros2 service call /manager/qualify_driver_power std_srvs/srv/Trigger "{}"
```

The retained safety topic is:

```bash
ros2 topic echo /manager/safety_status control_interface/msg/ManagerEvent \
  --qos-durability transient_local \
  --qos-reliability reliable
```

Only `MANAGER_READY` permits motion. Do not bypass an inhibited state by
calling the low-level device service unless performing deliberate diagnostics.

### Bounded catheter-linear limit recovery

If quantized or coupled physical-axis response leaves only logical
`catheter_lin` (axis 0) slightly outside its configured position range, the
manager exposes one narrow recovery service:

```bash
ros2 service call /manager/recover_catheter_linear_limit \
  std_srvs/srv/Trigger "{}"
```

This is not a general safety bypass. It requires mode `NONE`, no active command
source, fresh POS/ENC, ready transport, disabled motors, clean firmware fault
state (or an exactly confirmed encoder-integrity restoration), responsive
driver UARTs, exactly one violated axis, and an axis-0
excursion no larger than `limit_recovery_max_violation` (0.25 mm by default).
It commands only the configured minimum reliable axis-0 speed in the inward
direction, stops at the configured interior margin, aborts on stale,
unexpected, or farther-outward feedback, and applies a firmware STOP barrier.
It never changes encoder zero. Success deliberately leaves driver-power
qualification false; call `/manager/qualify_driver_power` and require
`MANAGER_READY` before any subsequent home or control command.

The normal Slicer teleoperation launch does not depend on the experimental
shape estimator:

```bash
ros2 launch teleop slicer.launch.py
```

That launch also starts `teleop_bag_recorder`. Pressing **Enable Keyboard
Control** creates
`D:\robot-dev\catheter_sessions\teleop_YYYYMMDD_HHMMSS\rosbag`; pressing
**Disable Keyboard Control** closes the active writer and finalizes rosbag
metadata. Recorder subscriptions stay live while idle, so the enable marker and
first keyboard command are not lost to subprocess startup or topic discovery.
Callbacks serialize and enqueue records immediately; a dedicated writer thread
writes the slower Windows-mounted database. Message header timestamps are used
when available, preserving command/feedback timing even if disk writes lag.
Fault and control records are retained preferentially if the bounded queue ever
fills.
Recording also stops if the Slicer key heartbeat is lost, the manager becomes
inhibited, or the launch shuts down. Each bag contains the Slicer command,
manager-clamped command, POS/ENC feedback, mode events, device events, and
retained manager safety status. When available, it also stores the `/IGTL_TRANSFORM_IN` stream from the
existing robot connector. The recorder keeps only exact `RX<number>` and
`RX<number>_filtered` devices from that shared topic. The live estimator consumes
the translation of the filtered transforms and ignores their identity rotations.
Estimated shape output is intentionally excluded. Configure `output_root`,
`session_prefix`,
`topics`, heartbeat timeout, and queue warning/limit thresholds in
`src/teleop/config/params.yaml`.

## Live catheter and sheath shape estimation

Use a Python 3.10 virtual environment that can also see the ROS 2 Humble
system packages. From the workspace root:

```bash
python3 -m venv --system-site-packages ~/cr-venv
source ~/cr-venv/bin/activate
python -m pip install -r state_estimation/requirements.txt
python -m pip install -e cr-common
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
cd robot-infra
python -m colcon build --packages-select automation teleop --symlink-install
source install/setup.bash
ros2 launch teleop slicer.launch.py enable_state_estimator:=true
```

The launch starts one Slicer bridge on TCP 18944. In Slicer, connect the main
IGTL interface and configure the catheter and sheath separately. Each coil row
contains an exact incoming transform name and its arc-length location. A
per-device **Rigidly fixed coils** toggle selects a direct straight-line fit;
rigid mode always uses one section (two endpoints), which Slicer renders as a
cyan capped cylinder alongside the coil transforms. When the toggle is off,
that device uses the continuum backend. A device may have zero coils;
it is then explicitly inactive, does not gate enabling, and publishes no shape.
One coil is rejected because it cannot define either supported shape. Enabling
is allowed only while every configured transform is actively updating. Shape
outputs are `catheter_shape` and `sheath_shape`. These names intentionally
stay below OpenIGTLink's 20-byte device-name limit. Internal continuum values
use SI units; the OpenIGTLink boundary and rigid fit use RAS mm.

The teleop recorder also stores `/IGTL_STRING_IN`, so each bag contains the
schema-v2 device/coil configuration. It records conventional RX transforms and
any additional transform names explicitly assigned in that configuration.

The estimator launch uses `~/state_estimation`, `~/cr-common`, and
`~/cr-venv` by default. Override these locations with
`STATE_ESTIMATION_PATH`, `CR_COMMON_PATH`, and `CR_VENV`.

## Codebase architecture and cleanup

The maintained cross-repository cleanup, ROS package decomposition, and
Python/C++ production migration plan is in
[`docs/architecture/CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md`](docs/architecture/CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md).
Structural cleanup is behavior preserving and is qualified separately from
controller, estimator, scheduling, model, and firmware changes.

## Serial device

To expose serial device to WSL2, in host powershell(admin), install usbipd:

```
winget install usbipd
```

Then run

```
usbipd list
```

Find the BUSID of the device and attach to WSL

```
usbipd bind --busid X-Y
usbipd attach --wsl --busid X-Y
```

Make sure the port is not occupied before attaching. Verify device detection in WSL

```
ls /dev/ttyACM*
```

Hot replug is currently not supported. Reattach after replugging or MCU flush.

## ROS2 networking

If running ROS nodes in WSL and on other machines at the same time, set WSL to [mirrored networking mode](https://learn.microsoft.com/en-us/windows/wsl/wsl-config#configuration-settings-for-wslconfig). The default NAT mode does not expose WSL to host network stack or support DDS.
