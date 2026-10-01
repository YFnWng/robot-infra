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
Because the two cameras are free-running, a bounded timestamp synchronizer
pairs the nearest unused frame from each rig before marker processing. The
`TRACKING` diagnostic reports `rig_timestamp_skew_ms` for the emitted pair and
the cumulative `synchronizer_dropped_frames`; an initial drop while the camera
phases align is expected. Pair formation defaults to 20 ms through
`maximum_rig_pairing_skew_ms`; the separate defensive rejection gate defaults
to 25 ms through `maximum_rig_timestamp_skew_ms`.
Every tracking or stale diagnostic now includes each rig's latest sequence,
monotonic frame age, image timestamp, synchronizer-drop count, and capture
worker error. If no pair is available for `feedback_timeout_s`, the process
timer emits `PAIRING_STALE` with these fields instead of returning silently.
This distinguishes a stopped camera worker from a timestamp-pairing failure;
the independent watchdog continues to emit `FEEDBACK_STALE` until tracking
recovers.
The tracker applies the registered image ROIs and rejects triangulated points
outside the base-frame workspace. Its default 3 mm inward boundary margin
excludes the ChArUco-board plane; override ROS parameter
`workspace_boundary_margin_mm` only if qualification shows the physical marker
workspace requires it.

## Native Ubuntu direct-camera route

Use this route on the robot computer. The ROS node opens both ZED cameras
directly; no UDP sender or receiver participates.

Build and run:

```bash
cd ~/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select automation
source install/setup.bash

ros2 run automation marker_tracking --ros-args \
  -p shape_tracking_root:=/path/to/shape_tracking \
  -p camera_config:=/path/to/shape_tracking/camera_config_hd720.yaml \
  -p registration_file:=/path/to/hd720/session/registration.json \
  -p rig_ids:="['primary','oblique']"
```

The camera profile and registration resolution must match. For closed-loop
operation, first follow
`../../../catheter-shape-tracking/HD720_REGISTRATION_AND_QUALITY.md` and retain
the passing JSON report with the registration session. An HD1080 registration
is intentionally rejected when the cameras open at HD720.

For initial timing and robustness qualification, use one physical ZED:

```bash
ros2 run automation marker_tracking --ros-args \
  -p shape_tracking_root:=/path/to/shape_tracking \
  -p camera_config:=/path/to/shape_tracking/camera_config_hd720.yaml \
  -p registration_file:=/path/to/hd720/session/registration.json \
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

### Partial-view multi-rig fusion

After marker identities have been initialized, the online tracker tolerates one
missing marker in either rig. Each rig contributes the markers it can still
triangulate, and fusion is performed independently for each marker. A fused
frame is published only when all four physical markers are reconstructed
across the two rigs. Jointly observed markers retain the cross-rig
disagreement gate.

Cold start also works when one rig sees all four markers but one eye of the
other rig sees only three. The complete rig's registered 3D estimate is
projected into the incomplete rig to seed marker identity and local search.
Those projections are priors only: the bootstrap frame is rejected with
`MULTI_RIG_BOOTSTRAP_PENDING`, and no marker point cloud is published until a
later frame contains at least three measured stereo markers from the
previously incomplete rig. Thus bootstrap does not manufacture a second
measurement source or weaken `minimum_valid_rigs`.

The marker point cloud's \`source_rig_count\` channel reports stereo-rig support per
marker. Diagnostics report \`PARTIAL_MULTI_RIG_TRACKING\` at WARN level, plus
\`marker_source_count\` and \`partial_markers\`, whenever a marker is supplied
by fewer than all configured rigs. Losing a marker from both rigs, losing more
than one marker within a rig, or violating a geometry gate remains fail-closed.


### Record full-rate SVO while publishing markers

The marker node can record both cameras without opening a second camera
process. Set `recording_enabled` and provide a new, nonexistent session
directory. Marker processing continues at the configured rate while each
camera worker records its native SVO at 30 Hz. On clean shutdown the node
finalizes both frame indices, writes `camera_frame_pairs.csv`, snapshots the
validated registration, and completes `session_metadata.json`.

```bash
ros2 run automation marker_tracking --ros-args \
  -p shape_tracking_root:=/home/chen-lab/Yifan/catheter-shape-tracking \
  -p camera_config:=/home/chen-lab/Yifan/catheter-shape-tracking/camera_config_hd720.yaml \
  -p registration_file:=/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260908_154036/registration.json \
  -p rig_ids:="['primary','oblique']" \
  -p minimum_valid_rigs:=2 \
  -p recording_enabled:=true \
  -p recording_session_dir:=/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/SESSION_NAME
```

Stop with Ctrl-C only after the experiment bag has finalized. Do not run the
standalone `shape-tracking-record` process at the same time: both programs
would compete for exclusive ownership of the cameras.

## Low-rate diagnostic camera overlay

The optional overlay route shows one decimated view from each ZED with colored
measured-marker crosses, the magenta v171 centerline reconstructed from the
timestamp-matched UKF estimator trace, and the yellow continuous target path
when `/catheter_mppi/reference_path` exists. It is disabled in ordinary marker
tracking. The capture node uses the images it already owns; it never opens a
second ZED connection. JPEG work runs in a single-slot worker, and a busy
worker drops the next preview instead of delaying marker processing.

Build both participating packages, stop the existing `marker_tracking` process,
and start the combined diagnostic launch:

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select control_interface catheter_control automation
source install/setup.bash

export CATHETER_REGISTRATION_FILE=/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260908_154036/registration.json
ros2 launch automation marker_overlay.launch.py \
  preview_rate_hz:=5.0 \
  preview_maximum_width:=640 \
  preview_eye:=left \
  marker_crosshair_size:=10 \
  marker_crosshair_line_width:=1 \
  show_window:=true
```

The overlay node expects the controller to be running and publishing
`/catheter_mppi/estimator_trace`. Until the UKF is initialized, the video still
appears but the estimated curve is omitted and labelled `UNAVAILABLE`.
Rendered JPEG topics are also available at:

```text
/catheter_mppi/camera_overlay/primary/compressed
/catheter_mppi/camera_overlay/oblique/compressed
```

Preview and overlay image topics are intentionally absent from the control
rosbag topic list. Keep the default 5 Hz, 640-pixel profile for timing
qualification; do not use raw HD720/30 video during hardware control.

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
  --camera-config .\camera_config_hd720.yaml `
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
