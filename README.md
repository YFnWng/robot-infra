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
- reconnecting always revokes driver-power qualification;
- the Slicer key state is a heartbeat. Loss of that heartbeat first publishes
  zero locally, then the independent manager watchdog disables the mode;
- legacy `START_MOTOR`, raw Slicer fault reset, and production debug commands
  are rejected. `SET_ZERO` is accepted only in mode `NONE` after a stationary
  feedback window.

Build and source the workspace normally, then start the control stack:

```bash
source /opt/ros/humble/setup.bash
source ~/robot-infra/install/setup.bash
ros2 launch control_interface launch.py \
  catheter:=imricor_test \
  serial_port:=/dev/ttyACM0
```

The supported hardware startup order is:

1. Motor-driver power off.
2. Power/connect the Teensy and start the control stack.
3. Open the serial link from Slicer (or call predicate `67`).
4. Power on the motor drivers while the robot is stationary.
5. Press **Qualify Driver Power** in Slicer. This waits for stable POS/ENC,
   stops and verifies disabled motors, clears only encoder-integrity startup
   latches, and rechecks firmware status.
6. Enable a motion mode only after Slicer reports `MANAGER_READY`.

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

## Live catheter shape estimation

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
ros2 launch teleop slicer.launch.py
```

The launch starts the Slicer bridge on TCP 18944 and an isolated tracking
bridge on TCP 18945. In Slicer, connect the main IGTL interface, send the
shape configuration, and start the coil simulator. Internal estimator values
use SI units; the OpenIGTLink boundary uses RAS mm.

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
