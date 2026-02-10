# ROS2 infrastructure for medical robots

This repository contains the basic software and firmware for research medical robots.

The software is built with ROS2 in Docker for reproducibility. For real time robot control, run in bare metal Linux or WSL.

Currently under construction.

Tested on Windows 11 + WSL2, Ubuntu 22.04 + ROS2 Humble

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