# Linux dual-ZED HD720 recording and full-shape workflow audit

Date: 2026-09-21
Scope: `catheter-shape-tracking` acquisition and offline four-view full-shape
reconstruction on the Ubuntu 22.04 control workstation. No motor path was
opened or exercised.

## Outcome

The dual-camera architecture is suitable for native Linux recording at
1280x720, 30 frames/s after the changes listed below. The two SVO streams are
not hardware-triggered; synchronization is therefore an offline nearest-time,
no-reuse association using ZED image timestamps. The previous verified HD720
session measured 29.96 frames/s on both rigs and 14.80 ms p95 inter-rig skew.

Post-processing is not suitable for concurrent execution with closed-loop
control. It starts two spawned SVO decoder processes, two eye workers per rig,
and the common-spline reconstruction, so it should run only after recording.

## Runtime topology

Acquisition:

1. Main process opens cameras by stable serial number.
2. One `CameraCaptureWorker` thread per ZED calls `grab()` independently.
3. Each successful grab is recorded in the camera's SVO and indexed by the
   SDK image timestamp. Preview retrieval is decimated independently and does
   not reduce the 30 Hz recording rate.
4. The main thread handles preview, color gates, and optional fixed-board
   registration.
5. On stop, full-rate per-rig indices are paired monotonically by nearest
   timestamp without frame reuse. Unpairable reference frames remain explicit.

Offline four-view reconstruction:

1. Two spawned processes independently decode primary and oblique SVOs and
   build reusable per-rig observation HDF5 files.
2. Each rig process uses up to two eye worker threads for chromatic marked
   catheter observations.
3. The parent process fits one common robot-base spline to the available four
   views and applies zero-phase temporal processing.

## Synchronization and timing evidence

- Camera clocks: ZED image timestamps, one monotonic sidecar per SVO.
- Cross-camera association: `camera_frame_pairs.csv`, generated only after
  recording stops; there is no shared hardware trigger.
- Prior HD720 evidence: session `20260908_154036`.
  - primary: 820 frames, 29.9616 fps, 33.531 ms p95 interval;
  - oblique: 821 frames, 29.9619 fps, 33.533 ms p95 interval;
  - paired: 820/820 reference frames;
  - absolute inter-camera skew: 14.803 ms p95, 15.438 ms maximum.
- The new `shape_tracking.session_audit` gate validates both SVOs, monotonic
  timestamps, effective rate, interval p95, calibration, paired fraction, and
  skew before expensive reconstruction.

## Findings and changes

1. **High: new recordings had no safe calibration-reuse path.** A recording
   without a visible registration board could finish without a usable
   `registration.json`, making four-view reconstruction impossible. Added
   `--registration-file`; it validates rig names, live serials, and resolution,
   then writes a session-local calibration snapshot with source provenance and
   current SVO/index names. It fails closed on mismatch.
2. **Medium: Linux inherited a Windows output default.** The recorder could
   create a literal `D:\\robot-dev...` path. The default is now native and
   respects `CATHETER_SESSION_ROOT`, then the mounted session volume, then
   `~/catheter_sessions`.
3. **Medium: there was no post-record acquisition gate.** Added the standalone
   dual-session audit and JSON report.
4. **High: standalone recording competed with required online tracking for
   camera ownership.** Added optional dual-SVO recording to the existing ROS
   `marker_tracking` camera owner. The node now finalizes frame indices,
   timestamp pairs, metadata, and a validated registration snapshot on clean
   shutdown, while the same captured frames continue through marker tracking.
5. **Operational: interpreter split.** `/usr/bin/python3` imports `pyzed` but
   lacks `h5py` and `rosbags`; `cr-venv` imports all required capture and
   post-processing modules. The documented workflow uses only `cr-venv`.
6. **Operational: storage headroom is limited.** The session filesystem showed
   38 GB free and 93% utilization during this audit. Check capacity before each
   recording and retain room for observation caches and reconstructed HDF5.
7. **Unverified live condition:** both camera USB interfaces enumerated at
   5000 Mb/s on separate root controllers, but device nodes were not exposed
   to the audit sandbox and the SDK inventory was empty there. The operator
   preflight must confirm serials from the normal terminal before recording.

## Safety and coexistence

The workflow is camera-only and does not arm or publish robot commands. During
a hardware control run, enable recording on the ROS marker node that already
owns both cameras; never start the standalone recorder beside it. Do not run
reconstruction or overlay rendering until the control run is over. Keep the
two cameras on separate USB 3 root controllers.

## Verification performed

- `cr-venv` imports: `pyzed.sl`, OpenCV ArUco, NumPy, SciPy, scikit-image,
  h5py, PyYAML, and rosbags.
- Targeted tests cover CLI validation, calibration identity/resolution checks,
  dual capture pairing, and the session audit.
- The session audit passes the existing `20260908_154036` HD720 recording.

Representative full-stack recording duration and disk throughput remain a
live measurement item; static inspection cannot prove absence of USB or encode
drops under the exact future ROS/control workload.
