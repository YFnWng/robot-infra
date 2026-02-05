# ROS2 infrastructure for medical robots built in Docker

This repository contains the basic software and firmware for research medical robots.

The software is built with ROS2 in Docker for reproducibility.

Currently under construction.

Tested on Windows 11 + WSL2

To expose serial device to docker container, in host powershell(admin), install usbipd:

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