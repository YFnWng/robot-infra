# Bringup

This package owns launch composition and deployment entry points. Runtime
implementation remains in the functional packages it launches.

Canonical launch commands use:

```bash
ros2 launch bringup control.launch.py
ros2 launch bringup simulation.launch.py
ros2 launch bringup <experiment-or-perception>.launch.py
```

## Coordinated research recording

`research_session.launch.py` starts one marker-tracking camera owner, one
sqlite3 bag recorder, and the existing controller/task action servers. It
does **not** start the motor interface, qualify power, arm, home, or submit a
task. Hardware output defaults to false. Stop existing marker/controller/bag
processes first; keep the already-qualified motor interface separate.

```bash
cd /home/chen-lab/Yifan/robot-infra
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch bringup research_session.launch.py \
  session_label:=attribution_twoaxis_points_grouped_n512_mount01_repeat01 \
  stack_config:=hardware_grouped_no_rotation_farther_tendon_12 \
  registration_file:="$CATHETER_REGISTRATION_FILE" \
  video:=true
```

`CATHETER_REGISTRATION_FILE` must name an existing validated registration.
Camera defaults are HD720/30 fps and the `CR_VENV` interpreter (default
`/home/chen-lab/Yifan/cr-venv/bin/python`), which must import installed
`shape_tracking` and `pyzed`. Override `camera_config` or `camera_python`
explicitly if necessary. `video:=false` retains tracking, controller, and bag.

For an explicitly authorized powered test, additionally pass
`command_output_enabled:=true`. This enables output, not automatic arming or
task execution. Start the separately authorized task only after
`recording_ready`; that state confirms recording/telemetry, **not** manager
readiness or permission to move. The actual profile/sample count is captured
in `controller_manifest.json`; a label token is not a parameter override.

The printed content-based session directory contains:

```text
SESSION_ID/
  session_manifest.json
  controller_manifest.json
  robot_bag/
  SESSION_ID_video/          # absent when video=false
    session_metadata.json
    primary_SESSION_ID_video.svo2
    oblique_SESSION_ID_video.svo2
    *_frame_index.csv
    camera_frame_pairs.csv
    registration.json
    robot_bag -> ../robot_bag
  camera.log
  bag.log
  controller.log
```

The camera creates its video subdirectory exclusively; the session allocator
reserves the parent. The bag link lets offline shape tools discover telemetry.
Pass the video subdirectory as `--session` to reconstruction. UTC suffixes
avoid timezone ambiguity; existing directories are never overwritten.

Wait for `recording_ready`, then operate the existing task client separately.
Check `/experiments/session_status` or the manifest for readiness; subprocess
output is saved to the three logs. Readiness requires an owned live recorder,
a created storage file, required recorder subscriptions observed in the ROS
graph, fresh telemetry, and active camera
recording when requested. The owned recorder starts unpaused, without keyboard
input; do not pause it externally during a trial. The supervisor never opens
the active SQLite database: read-only connections can still compete with writes.
Subscriptions/file existence do not prove durable message delivery. SQLite
integrity and message counts are checked only after the recorder has exited
and flushed its cache.

Ctrl-C stops the controller first, finalizes the bag, then finalizes video.
Wait for the final manifest before processing. Unexpected child exit or loss
of recording readiness stops owned processes; there is no automatic recovery.
Shutdown escalation is bounded and recorded. It does not bypass manager or
firmware watchdogs and is not a substitute for an operator emergency stop.

Qualification checks sqlite integrity, required messages, and both camera
reports/index/SVO artifacts. `complete` means finalized recording, not target
success or qualified tracking accuracy. Interrupted startup is `partial`;
failures/escalation are retained as `failed`. Camera/frame-drop and full-stack
timing still require hardware qualification. The synthetic smoke test opens no
devices:

```bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  RUN_RECORDING_ROS_SMOKE=1 /usr/bin/python3 -m pytest \
  src/experiments/test/test_recording_ros_smoke.py -q
```
# Per-session independent camera registration

`research_session.launch.py` now refreshes the camera frame's robot-base pose
before publishing markers or starting SVO recording. Keep the existing
No earlier base or inter-rig transform is used. `registration_file` is retained
only for compatibility with old launch commands and is not required by this
startup route. Expose the robot-base ChArUco fixture to both rigs and keep the frame and
fixture stationary through startup. Optional launch arguments are
`registration_config:=/path/to/registration_config.yaml` and
`registration_frames:=20` (default; minimum 10).

Startup reuses shape_tracking's pose detection, robust averaging and overlays.
It collects distinct valid poses after 15 warm-up grabs, checks translation
spread <=1 mm per coordinate and rotation spread <=0.5 degrees, then refreshes
each rig's camera-to-base pose independently. The SDK stereo calibration
within each ZED remains in use, but the transform between ZED rigs is derived
only from their new base fits. Stability checks are not a claim of
submillimetre absolute accuracy. Missing boards, unstable fits or timeout fail
startup without stale-registration fallback.

Each video/session directory receives `registration.json`, refresh provenance,
fit statistics and four `registration_<rig>_<eye>.png` overlays. The trackers
use that exact file; registration remains frozen throughout motion. Even with
`video:=false`, the directory retains calibration evidence. The old seed and
frozen experiment targets are never rewritten. Independent fit reports and
overlays remain in `registration_fits/` even if a subsequent check fails.
Verify overlays and measured
home after a frame move before deciding whether targets need revision.
