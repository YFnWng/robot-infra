# Online four-marker feedback

The `automation` package owns the ROS 2 node. The external `shape_tracking`
repository provides ZED acquisition, red-component detection, stereo
triangulation, and registration-file loading. Online processing detects only
the four red markers; it does not segment or reconstruct the catheter shape.

The node publishes `/shape_tracking/markers` as
`sensor_msgs/msg/PointCloud` in the configured `robot_base` frame. Its four
points are ordered from the registered catheter base toward the distal tip and
use metres. Channels are `marker_id`, `confidence`,
`reprojection_error_px`, and `source_rig_count`. It publishes no point cloud
unless all four markers pass the stereo quality gates, and never republishes a
stale observation. Health and rejection reasons are on
`/shape_tracking/marker_status` as `diagnostic_msgs/msg/DiagnosticArray`.

Build and run:

```bash
cd ~/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select automation
source install/setup.bash

ros2 run automation marker_tracking --ros-args \
  -p shape_tracking_root:=/path/to/shape_tracking \
  -p camera_config:=/path/to/shape_tracking/camera_config.yaml \
  -p registration_file:=/path/to/session/registration.json \
  -p rig_ids:="['primary','oblique']"
```

For initial timing and robustness qualification, use one physical ZED:

```bash
ros2 run automation marker_tracking --ros-args \
  -p shape_tracking_root:=/path/to/shape_tracking \
  -p camera_config:=/path/to/shape_tracking/camera_config.yaml \
  -p registration_file:=/path/to/session/registration.json \
  -p rig_ids:="['primary']"
```

Inspect the output:

```bash
ros2 topic echo /shape_tracking/markers
ros2 topic hz /shape_tracking/markers
ros2 topic echo /shape_tracking/marker_status
```

A controller consuming this topic must enforce its own feedback timeout and
command zero motion when feedback becomes stale.

## Windows camera to WSL ROS pipeline

Use this pipeline when Windows owns the ZED cameras. Do not run
`marker_tracking` and `marker_udp_receiver` simultaneously because both
publish `/shape_tracking/markers`.

Start the receiver in WSL:

```bash
cd ~/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select automation
source install/setup.bash

ros2 run automation marker_udp_receiver --ros-args \
  -p bind_address:=0.0.0.0 \
  -p udp_port:=50050 \
  -p frame_id:=robot_base
```

Find the WSL address from PowerShell:

```powershell
wsl.exe -d Ubuntu-22.04 hostname -I
```

Then start marker-only camera processing in the `shape_tracking` Windows
environment:

```powershell
cd D:\robot-dev\shape_tracking
.\.venv\Scripts\Activate.ps1

python -m shape_tracking.marker_udp_sender `
  --camera-config .\camera_config.yaml `
  --registration-file D:\robot-dev\catheter_sessions\SESSION\registration.json `
  --rig-id primary `
  --rig-id oblique `
  --udp-host <WSL_IP> `
  --udp-port 50050
```

The receiver validates schema/version, all four IDs, finite values, configured
position and reprojection bounds, frame name, acquisition/send timestamp age,
sender session, and strictly increasing sequence. It drains queued datagrams
and publishes only the newest valid packet. Sequence gaps and superseded
packets are reported in `/shape_tracking/marker_status`.
