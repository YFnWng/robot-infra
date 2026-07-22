# ROS2 infrastructure for medical robots

This repository contains the basic software and firmware for research medical robots.

The software is built with ROS2 in Docker for reproducibility. For real time robot control, run in bare metal Linux or WSL.

Currently under construction.

Tested on Windows 11 + WSL2, Ubuntu 22.04 + ROS2 Humble

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
