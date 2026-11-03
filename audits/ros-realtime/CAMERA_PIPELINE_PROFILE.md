# Camera and marker-tracking pipeline profile

2026-11-02. Passive measurements only: camera topics observed, nothing
commanded. The 60 fps run used a temporary config copy
(`scratchpad/camera60/camera_config_hd720_60fps.yaml`) with the robot
stationary. The repository config is unchanged: HD720, 30 fps, `exposure: 6`.

## Pipeline and where the time goes

```text
ZED 2 x2 (free-running, USB, DEPTH_MODE.NONE)
  -> grab() + retrieve LEFT/RIGHT       (shape_tracking zed_capture, one worker per rig)
  -> LiveFrameSynchronizer pairing      (shape_tracking multi_camera, 20 ms tolerance)
  -> per-rig stereo marker triangulation, cross-rig fusion (perception/marker_tracking.py)
  -> /shape_tracking/markers (PointCloud, stamped with the fused image timestamp)
```

Source: the node's own `marker_status` stage fields, plus marker receipt minus
header stamp.

| Median (P99) | 30 fps, moving (`20261102_172930`) | 30 fps, stationary (278 s) | **60 fps, stationary, two startups** |
| --- | ---: | ---: | ---: |
| Image timestamp → `grab()` returns | 29.9 (31.4) ms | 30.0 (31.4) ms | **16.1 (17.8) ms** |
| `grab()` → images retrieved | +1.8 ms | — | +1.8 ms |
| Inter-rig timestamp skew | 16.5 ms | 14.6 ms | 10.2 / 9.8 ms |
| Pairing wait of the earlier rig | 16.5 ms (oblique) | — | 8–9 ms (primary) |
| Detection, triangulation and fusion | 4.0 ms | 3.7 ms | 3.8 ms |
| **Image timestamp → marker message received** | **36.1 (39.7) ms** | 35.7 (39.1) ms | **22.2 (26.1) ms** |
| Synchronizer drops | 0 (counter static) | 16 / 8,283 (0.2%) | 23 / 1,737 and 8 / 696 (about 1.2%) |
| Marker noise per frame, std | — | 0.050–0.075 mm | 0.052–0.074 mm |
| Reprojection error | 0.98 px | 0.90 px | 0.91 px |

October's audit measured 64 ms end to end and 3.7 ms skew (archived
`timing-summary.csv`).

**Why the frame rate changes latency, not only sampling.** The image
timestamp marks capture. The sensor readout and USB transfer of each frame
are paced at the frame rate, and `grab()` returns only once the whole frame
has arrived and been decoded. The roughly 30 ms grab latency equals one
frame period at 30 fps and halved to about 16 ms at 60 fps. The pairing wait
is bounded by half a period, so it also halves.

## Offline follow-up (2026-11-02, after the lab session)

Whole recorded sessions (four bags, about 42,000 paired frames) correct the
first impressions from the 30 s live windows.

**Frame accounting** (`scratchpad/camera60/frame_loss.py`):

| Session | Camera/SDK periods with no frame delivered | Synchronizer drops | Uncounted overwrites | Pairs on the larger alignment | Median skew |
| --- | ---: | ---: | ---: | ---: | ---: |
| 60 fps, startup 1 | 206 / 244 (2.6–3.1%) | 64 / 25 | 4 / 4 | 46% | 6.65 ms |
| 60 fps, startup 2 | 353 / 363 (3.8–3.9%) | 56 / 46 | 4 / 4 | 31% | 7.22 ms |
| 30 fps, stationary | 44 / 44 (0.5%) | 8 / 8 | 2 / 1 | 28% | 14.64 ms |
| 30 fps, during runs | 101 / 104 (0.6%) | 24 / 20 | 1 / 2 | 42% | 16.53 ms |

- The node's drop counters count only synchronizer discards. Most frame
  loss is upstream: periods where the SDK timestamps show no frame reached
  the capture thread.
- **88–92% of primary-rig skips coincide with an oblique-rig skip** (within
  1.5 periods), although the rigs are on separate USB buses. This points to
  a process- or host-wide stall that stops both capture threads from calling
  `grab()` in time, not per-camera USB loss. At 30 fps a stall must exceed
  33 ms to cost a frame, at 60 fps only 16.7 ms, consistent with the rates.
- `pyzed` `Camera.grab()` **releases the GIL** while waiting (SVO playback
  test, `scratchpad/camera60/gil_test.py`: a pure-Python thread kept 98% of
  its rate). So the stall is elsewhere in the camera process: long GIL
  holds, garbage collection, or CPU scheduling.
- Measured processing over 20 ms occurs only 1–2 times per session.

**Pairing.** The synchronizer does not lock into one alignment; each camera
skip re-aligns it. Always taking the smaller alignment would lower the
**mean skew by only 0.5–1.7 ms** (individual pairs up to 4.7–6 ms). The
"3–9 ms" estimated from the live windows was unrepresentative. Low
priority.

**Fused timestamp** (`online_markers.fuse_marker_estimates`):
- Fused points are confidence-weighted averages of the per-rig estimates,
  stamped with the **newest** rig timestamp (`max`).
- At equal weights the stamp is late by half the skew: **about 7.8–8.3 ms at
  30 fps, 3.9–4.1 ms at 60 fps**. Single-rig markers carry their own rig's
  time, so one message can mix times.
- A correct stamp (weighted mean time) changes estimator input, so it is a
  behaviour change requiring its own estimator check.

**Effective latency against encoders: inconclusive offline.**
Cross-correlating catheter-marker speed with encoder speed over pulse edges
gave lags from −11 to +60 ms, inconsistent between markers and sessions. The
catheter markers sit behind chassis play (7.05 mm) and tendon engagement, so
edges measure mechanics. A clean measurement needs a rigid fiducial that
moves with an encoder without play.

## Optimizations

Each is a separate proposal; none is implemented.

**No hardware needed (offline):**
1. **Pairing alignment.** Prefer the smaller-skew alignment. Mean gain is
   only 0.5–1.7 ms (follow-up above), so this is low priority.
2. **Fused timestamp semantics.**
   - The fused estimate combines per-rig estimates up to half a period apart.
     If it carries the newer rig's timestamp, as `image_timestamp_ns` matching
     the primary suggests, positions are biased toward older data by up to
     about 8 ms (30 fps).
   - Check `fuse_marker_estimates` and size the bias against motion speed.
3. **Effective camera latency against encoders.** Cross-correlate marker
   motion with encoder-timed pulse onsets in today's sessions. This measures
   what the image timestamp actually represents, which the estimator's time
   alignment depends on.
4. **60 fps frame skips.** Mostly simultaneous on both rigs, so this is a
   process- or host-wide stall (follow-up above). Candidate remedies, each
   needing confirmation by profiling first:
   - run each capture worker in its own process with shared-memory frame
     handoff;
   - reduce long GIL-holding work in the processing or preview path;
   - tune GC in the camera process (thresholds or `gc.freeze()` after
     startup, not disabling it, per `robot-infra/AGENTS.md`).
5. **Using 60 fps data.** Averaging pairs of 60 fps frames lowered stationary
   noise by about 30% (0.036–0.053 mm), or the estimator can take 60 Hz
   markers directly. This is a separate estimator decision.

**Hardware needed (cameras with the lab lighting on):**
- A 60 fps run **with motion**: detection reliability and partial tracking
  under motion, and motion blur at 2 ms exposure.
- A longer 60 fps run (minutes) under the full stack, controller included,
  to find the simultaneous-skip stall:
  - `zed.get_frame_dropped_count()` per rig;
  - per-iteration grab-loop timing in each capture thread;
  - process CPU, and a GIL/thread profile (`py-spy --gil`);
  - GC pause logging (`gc.callbacks`).
- The effective camera latency with a rigid fiducial on the insertion
  carriage, moved in steps, and compared with encoder edges.
- Optional: a CPU sampling profile of the camera process. `py-spy` is not
  installed, and `ptrace_scope=1` requires launching the node as its child.
  Installing it needs approval.

**Little to gain:** retrieval (1.8 ms), detection and fusion (about 4 ms),
publish and transport (under 1 ms).

## Prepared changes (2026-11-02, uncommitted; wheel deployed)

Both are opt-in, and the defaults reproduce current behaviour.

**Fused-timestamp policy.**
- `catheter-shape-tracking` `online_markers.fuse_marker_estimates(...,
  timestamp_policy="newest")`:
  - `"newest"` keeps today's stamp, the latest contributing rig.
  - `"weighted_mean"` stamps the mean over markers of each marker's
    fusion-weighted contributor time.
  - The weighted time is always stored in the new optional field
    `MarkerEstimate.weighted_timestamp_ns`, using exact integer arithmetic on
    relative times.
- `marker_tracking` parameter `fused_timestamp_policy` (default `newest`).
- `marker_status` always reports `fused_weighted_stamp_lag_ms`, so new
  recordings measure the bias under the old policy.

**Stall diagnostics.**
- `catheter-shape-tracking` `CameraCaptureWorker(...,
  collect_loop_statistics=False)` reports per-window `grab_call_max_ms`,
  `between_grabs_max_ms`, `between_grabs_over_10ms` and the SDK's cumulative
  `sdk_dropped_frames`. `ZedCamera.frame_dropped_count()` wraps
  `get_frame_dropped_count()`.
- `perception/stall_diagnostics.py`:
  - `GcPauseMonitor`: `gc.callbacks` pause timing per generation;
  - `SchedulingProbe`: a 2 ms sleeping thread whose late wake-ups expose GIL
    holds or CPU stalls.
- `marker_tracking` parameter `stall_diagnostics_enabled` (default false).
  The fields are added to `marker_status` per published pair.

**Launch.** `research_session.launch.py` adds `camera_stall_diagnostics`
(default false) and `fused_timestamp_policy` (default newest), passed to the
camera node.

**Compatibility.** The camera node imports the *installed* `shape_tracking`
wheel in `cr-venv`. That wheel was built on 2026-10-07 and matches neither
`HEAD` nor the working tree. The node passes the new arguments only when an
option is enabled, so the default path runs unchanged on the old wheel.
Enabling an option on the old wheel fails at startup with a clear message
(`_require_wheel_support`).

**Tests.**
- `catheter-shape-tracking` against the source: full suite 286 passed,
  including fusion-policy and loop-statistics tests.
- `perception`: 41 passed against both the source and the installed wheel,
  including GC, probe (a GIL-holding `list.sort` is detected) and
  wheel-guard tests. `test_estimation_node_runtime.py` needs the unrelated
  `ros2_igtl_bridge`.
- `bringup` research-session launch tests: 14 passed, including the default
  camera command.
- Lint is clean.

**Deployment (done 2026-11-02, user-approved).** The `shape_tracking` wheel
was built from the `catheter-shape-tracking` working tree as is (HEAD
`56980d0` plus uncommitted changes, including the user's `multi_camera.py`,
`online_markers.py`, `register.py`, `zed_capture.py` edits and new untracked
modules) and installed into `cr-venv` with `--no-deps`:
- Built from a clean copy of `src/` and `pyproject.toml`; the repository's
  `build/` was not used.
- Wheel SHA-256 `177d6d94…d95d604`. The installed package is file-identical
  to `src/shape_tracking`.
- The previous installation (Oct 7) and the new wheel are kept in
  `catheter_sessions/shape_tracking_wheel_rollback_20261102/`. To roll back,
  copy `previous_install_20261007/*` back into `cr-venv` site-packages.
- After install: `perception` 41 passed against the new wheel; the startup
  guard accepts both options in the camera interpreter; `pyzed` imports.
- Defaults still select `newest` and diagnostics off. Not yet exercised on
  hardware.

## Next hardware session: stall diagnosis (procedure, declared before running)

Prerequisites:
- the wheel is deployed (done 2026-11-02);
- camera-only and non-actuating unless combined with an authorized motion
  run;
- the ChArUco board is in view at startup.

1. `research_session` with the 60 fps config and
   `camera_stall_diagnostics:=true video:=false`: 3 min stationary. About
   5 min including startup.
2. The same at the normal 30 fps: 3 min.
3. Optional, with motion authorization: a `tendon_start` run at 60 fps with
   diagnostics on.

Analysis per camera skip, where a skip is a period without a delivered
frame, simultaneous on both rigs or not:

| Coinciding evidence (within ±1 period) | Conclusion | Candidate remedy |
| --- | --- | --- |
| `gc_pause_max_ms` > 16 ms (60 fps) or > 33 ms (30 fps) in most simultaneous skips | GC pauses | `gc.freeze()` after startup and/or raised thresholds in the camera process (not disabling GC) |
| `probe_late_max_ms` over one period without a long GC pause | GIL hold or CPU contention elsewhere | Identify the holder (`py-spy --gil`, needs approval to install); capture worker per process |
| `between_grabs_max_ms` over one period on that rig without a probe stall | Worker-local delay (lock or bookkeeping) | Fix the worker path |
| `sdk_dropped_frames` increments without the above | SDK/USB-level | Camera or USB configuration |

Success criterion for a remedy: at 60 fps, simultaneous skips under 0.5% of
periods (the 30 fps level) with unchanged detection quality.

## Stall diagnosis run 1 (2026-11-03, 60 fps): skips are inside the SDK, not GC or GIL

Session `catheter_sessions/camera_stall_60fps_20261103T155107152363Z` (97 MB).

**Setup:**
- `research_session` on domain 42, stack `hardware_grouped_no_rotation_farther_tendon_12`, with the
  60 fps config copy, `camera_stall_diagnostics:=true` and `video:=false`.
- The motor interface was up, driver power unqualified, manager inhibited, and `command_output_enabled`
  false. `/device/command_tx` had 0 messages.
- 181 s of tracking. The bag qualification passed.
- The manifest says `failed` only because the camera process segfaulted at shutdown (below).

**Skips, at 16.67 ms periods:**
- Simultaneous on both rigs: **338 events, 3.17 %** of 10,874 periods. That reproduces the 2.6–3.9 %
  measured on 2026-11-02.
- Single-rig: 46 primary-only and 34 oblique-only events.
- Analysis scripts: `scratchpad/stall/analyze.py`.

**Coinciding evidence for the simultaneous skips**, checked against the decision table above:

| Evidence | Measured | Reading |
| --- | --- | --- |
| GC pause | Maximum 0.089 ms over the whole run; never above one period | Not GC |
| Scheduling probe late | Maximum 6.3 ms, P99 1.6 ms; 0 stalls | No GIL hold or CPU stall visible to Python |
| Between grabs, per rig | Maximum 0.76 ms | Not worker-local |
| SDK dropped-frame counter | Incremented on **both** rigs in 322 of 338 events; `grab()` blocked about 33 ms (two periods) | **SDK/USB level** |

The frames are lost inside `grab()`, on both cameras at the same time. The two cameras sit on
different xHCI controllers: the primary's video on bus 4 (2-port controller `0000:04:00.0`), the
oblique's on bus 2. So a shared USB link is unlikely. Candidates are something shared by both cameras
inside the one camera process: SDK native code (which releases the GIL, so the probe cannot see it)
or GPU contention with the controller. Not yet discriminated.

**Separate hardware fault on the primary camera (S/N 20757336):**
- At 10:52:07 the kernel logged a USB disconnect of the whole primary camera, video (`usb 4-1`) and HID
  (`usb 3-1.2`) together. It is the first ZED disconnect since 2026-11-02 (the other disconnects
  are the Teensy at boot).
- The SDK recovered after a module reset; tracking was out for 11.4 s (21 `PAIRING_STALE` rows).
- At shutdown, 10:54:31, controller `0000:04:00.0` logged an xHCI TRB error. The camera process then
  segfaulted inside `libsl_zed.so`, giving return code −11.
- This points to that camera's cable, connector or add-in controller, not software.

## Stall diagnosis run 2 (2026-11-03, 30 fps): same signature, rate-dependent

Session `catheter_sessions/camera_stall_30fps_20261103T160244388896Z` (86 MB).

**Setup:**
- Identical to run 1, except the default `camera_config_hd720.yaml` (`fps: 30`, `exposure: 6`).
- Run after both camera cables were reseated and the motor interface was restarted.
- Session `complete`; all processes returned 0; qualification passed; `/device/command_tx` 0.
- No USB disconnects or xHCI errors in the kernel log.

| | Run 1, 60 fps | Run 2, 30 fps |
| --- | --- | --- |
| Simultaneous skip events | 338 in 181 s (1.87/s) | 18 in 180 s (0.10/s) |
| Share of periods | 3.17 % | 0.33 % |
| Single-rig skips | 46 + 34 | 0 |
| Events with an SDK drop on both rigs | 322 / 338 | 17 / 18 |
| `grab()` blocked during a skip | about 33 ms (2 periods) | about 66 ms (2 periods) |
| GC maximum; probe-late maximum | 0.089 ms; 6.3 ms | 0.079 ms; 2.5 ms |
| Rig skew P50 / P99 | 2.1 / 18.7 ms | 0.16 / 0.52 ms |

**Conclusion.** Both rates show the same mechanism. A frame is lost inside the SDK on both cameras at
once, and `grab()` then returns the next frame. GC, Python-visible scheduling and the capture worker
are ruled out at both rates. The event rate rises about 19-fold for a 2-fold frame-rate increase, so
it depends on load, not on a fixed periodic disturbance. The two cameras are on separate xHCI
controllers. So the shared cause must sit in the host or in the one camera process: SDK native threads,
or GPU and memory contention with the controller. The next discriminator is the 60 fps capture with
only the camera process running (no controller, recorder or GPU planner), compared with run 1.

## Stall diagnosis run 3 (2026-11-03, 60 fps, camera process only): contention is not the main cause

Session `catheter_sessions/camera_only_60fps_20261103T160922Z` (52 MB).

**Setup:**
- Run 1's exact `perception.marker_tracking` command.
- No controller, session supervisor or full recorder. Only `ros2 bag record` of `marker_status` and
  `markers` ran, plus `nvidia-smi` sampled every 0.5 s.
- No USB faults.
- The camera exited with code 1: an rclpy `publish` after the context became invalid on SIGINT, a
  shutdown race. The data is unaffected.

| | Run 1, 60 fps, full stack | Run 3, 60 fps, camera only |
| --- | --- | --- |
| Simultaneous skips | 338 events, 3.17 % of periods | 229 events, **2.22 %** |
| Single-rig skips | 80 events | 15 events |
| Events with an SDK drop on both rigs | 322 / 338 | 228 / 229 |
| GC maximum; probe-late maximum | 0.089 ms; 6.3 ms | 0.025 ms; 2.8 ms |
| GPU utilization median / P95 / max | not sampled | 11 / 15 / 17 % |

**Conclusion.** Removing the controller, supervisor and full recorder cut simultaneous skips by only
about 30 %. The GPU was mostly idle. The dominant cause is therefore inside the camera process or the
SDK, or at host or USB level, not contention with the control stack.

Remaining discriminators, none run yet:
- one camera alone at 60 fps: is a single camera clean?
- one process per camera: is the coupling in-process?
- the vendor's own capture at 60 fps, as an SDK baseline.

30 fps (0.33 %, single-frame losses) is adequate for the 5 Hz controller, so 60 fps is parked.


### Result

Approved and run as declared below. Evidence, scripts and logs are in
`catheter_sessions/gate3_fused_timestamp_check_20261102/`. The step 0–2 runs
took about 50 s wall time.

**Step 0.** Per-rig replay of 60 frames from image 500 on: both rigs detect
all four markers offline. The live marker 3 matches the oblique point
(median 0.11 mm) and not the primary point (2.68 mm), so it is
oblique-only. Oblique is the newer rig in every shifted image, so
δ ≈ 3/8·skew. As a side check, recorded markers 0–2 match the equal-weight
two-rig mean within 0.09 mm (primary alone: 0.47 mm).

**Step 1.**
- G0 passed: the δ=0 rebuild through the archived owner
  (`repository_archives/cleanup_20261102/.../real_joint_preparation.py`,
  identical to the copy in `sources.zip`) is bit-identical to the archived
  packet; the SHA-256 matches.
- 1198 images shifted, with δ from 6.43 to 6.77 ms (median 6.60). 27 kept
  their stamp: 26 unmatched pairs and 1 other rig pattern.
- Markers and encoder nodes are unchanged, the node count is unchanged, and
  image order is preserved.

| Check | Result |
| --- | --- |
| G1, checkpoint continuation | **Strictly failed, by 7.9e-10 mm.** Not bit-identical, but within the documented 5.8e-8 mm repeatability and far below decision size |
| Stamps | Pass |
| Health | Pass: no holds or faults in either arm. Admission flips at images 815 and 816 (segment A) |
| **Primary: next-image error, moving images** | **FAIL.** Pooled RMS 0.581 → 0.568 mm (n=87), but 42 better against 45 worse; one-sided sign test p=0.67 |
| Negative control, held images ≤ 0.005 mm | **FAIL pooled (0.038 mm).** C 1e-7 mm (pass); A 0.074 and B 0.029 mm, from state divergence carried over from earlier moving images |
| 0.8 s forecasts not worse by > 0.02 mm | Pass: C 0.417 → 0.408, A 2.484 → 2.380, B 0.917 → 0.910 mm |
| Current fit not worse by > 0.005 mm | Pass: C +0.0004, A −0.023, B −0.003 mm |

| Segment | Moving n | Next-image RMS, mm | Better / worse |
| --- | ---: | --- | --- |
| C, chassis only | 24 | 0.3062 → 0.3049 | 9 / 15 |
| A, knee | 32 | 0.871 → 0.850 | 16 / 16 |
| B, engaged tendon | 31 | 0.304 → 0.296 | 17 / 14 |

**Interpretation, limited to these segments.**
- Five images, mostly in A around the 815/816 admission flips (804, 816,
  817, plus 807 and 808 during fast chassis motion), supply 89% of the
  squared-error reduction.
- The cleanest test of the hypothesis is segment C, where only the chassis
  moves. There the shift produced no improvement.
- The roughly 6.6 ms correction is below what this filter and readout can
  resolve on this session.
- The forecast and fit gains in A come through changed history, not a
  per-image alignment effect.

**Decision under the declared rules:**
- No δ retuning and no added mechanism.
- No full-session escalation, since the short segments failed.
- Turning on `weighted_mean` for estimator accuracy is not supported by
  this evidence.
- The fused stamp's bias remains a timing-bookkeeping question (planner root
  age, latency accounting), separate from filter accuracy.

### Formulation (as approved before running)

**Hypothesis.** Each fused image averages marker positions captured at
different rig times. Labeling the image with the newest rig time pairs its
markers with encoder states that are too late by δ_k. Stamping it at the
weighted time aligns encoders with what the markers actually show. That should
lower the filter's next-image prediction error during motion, leave
stationary images unchanged, and not degrade 0.8 s forecasts.

**Only change.** The image stamp used by the estimator, from the recorded
fused stamp t_k (the newest rig) to

$$
t^w_k=\frac14\sum_{i=0}^{3}\frac{\sum_{r\in\mathcal R_i}w_{ri}\,t_{rk}}{\sum_{r\in\mathcal R_i}w_{ri}},
\qquad \delta_k=t_k-t^w_k\ge 0,
$$

which is exactly the `weighted_mean` policy, with R_i the rigs that observed
marker i. Unchanged:
- markers and the full R;
- encoder samples and their interpolation owner;
- artifacts, the initialization and its seed;
- K=2, physical gain, the 0.01 mm profile hold and the 0.5 mm support
  extension;
- admission, update ordering, the forecast horizon and the scoring readout.

**Data available for the replay session (Oct 8 `...20261008T213127347675Z`,
1225 images at 10 Hz):**
- Per-rig image stamps for every pair (`camera_frame_pairs.csv` and
  `marker_status` `rig_*_image_timestamp_ns`). 1199 of 1225 replay images
  match a pair whose newest stamp equals the recorded fused stamp.
- Inter-rig skew: median 17.61 ms, P95 17.89 ms.
- Markers 0–2 are two-rig in all images. Marker 3 is single-rig in all 1225
  images, from an unrecorded rig.

So δ_k ≈ 3/8·skew = 6.6 ms if marker 3 comes from the newer rig, or
5/8·skew = 11.0 ms if from the older one.

**Declared approximations:**
- Per-rig confidences were not recorded, so two-rig markers use equal weights.
  Fused confidences are about 0.89, so the stamp error is under about 1 ms.
- The 26 unmatched images keep their recorded stamp. Their count and indices
  are reported.
- True camera exposure latency is not part of this check. Only the fusion
  bias is corrected.

**Expected size.** Encoder displacement over δ ≈ 9 ms:
- tendon at about 2 mm/s: about 0.02 mm;
- chassis at 4–12 mm/s: 0.03–0.11 mm.

That is far below the 0.885 mm regular-stride forecast RMSE, so the
primary metric is next-image prediction error on moving images, paired by
image across arms.

**Steps:**

0. **Identify marker 3's rig (read-only, under 1 min).** Run
   `shape_tracking.recorded_marker_replay` per rig on a 2 s SVO window. This
   identifies a recorded fact; it is not an estimate.
1. **Build the shifted input with the existing owner.** Use
   `real_joint_preparation.real_observation_packet` with only `source_ns`
   replaced by t^w. Its native grid is encoder times plus image stamps.
   - Gate G0: rebuilding with δ=0 reproduces
     `observations_and_plan.npz` bit-for-bit. Its SHA-256 identity is
     checked.
   - A diagnostic script passes the shifted packet to the existing bootstrap
     and replay owners under its own declared digest. No owner default
     changes.
2. **Checkpoints.**
   - Run the baseline prefix once, without forecasts, and save states at
     images 430, 796 and 956.
   - Each segment starts both arms from the same baseline checkpoint. The
     checkpoint stamp is kept as the root node in the shifted grid, so the
     first shifted interval is t_a → t^w_(a+1).
   - The baseline arm is rerun with the current settings. The archived
     `session_on` predates the 0.5 mm support extension, so it is context,
     not the matched baseline.
   - Gate G1: the δ=0 arm through the diagnostic path (rebuilt packet) is
     bit-identical to the baseline arm through `reduced_filter_session.py`'s
     path (archived packet) on segment C.
3. **Short segments (chosen now, before any run):**

| Segment | Images | Behaviour exercised |
| --- | --- | --- |
| C, chassis only | 430–470 | Tendon stationary, chassis about 2.4 mm/s mean: isolates the b·(Δm+αΔκ) path |
| A, knee | 796–835 | Chassis up to 12 mm/s, a tendon reversal, the 813 knee |
| B, engaged tendon | 956–995 | Tendon about 1.9 mm/s through engagement, the 974/975 roots |

   - Each image in each arm records a one-image (100 ms) frozen-root
     forecast against the next image's readout.
   - Every 6th root, plus 813, 974 and 975, records the standard 0.8 s fan.
   - Each image is scored against its own marker readout (the e6 reference)
     by image index, not by stamp.

**Checks, declared before running:**

| Check | Pass |
| --- | --- |
| G0, G1 equivalence | As above. Otherwise stop and report |
| Stamps | 0 ≤ δ_k ≤ skew_k; image order preserved; δ distribution reported |
| Health | No new rejection, hold or fault; admission flips listed |
| Primary: next-image error on moving images (tendon or chassis travel ≥ 0.05 mm between images) | Paired RMS falls in segments A+B+C combined, with more images better than worse (sign test p < 0.05). Effect size reported per segment |
| Negative control: held images (both under 0.05 mm) | Next-image prediction changes ≤ 0.005 mm RMS |
| 0.8 s forecast at segment roots | Not worse by more than 0.02 mm RMS |
| Current fit | Not worse by more than 0.005 mm RMS |

**Runtime and approval:**
- Step 0: under 1 min.
- Bootstrap plus the baseline prefix to 956: about 40–60 s.
- Segments, 2 arms × 3 × 40 images with forecasts: about 30–60 s.
- Total: about 2–3 min, which needs approval.
- Checkpoints are kept, so any rerun of a segment takes seconds.

**Failure handling.** A failed primary or control check is reported as is: no
δ retuning, no partial-δ arms and no added mechanism. A pass qualifies only
these segments. The full-session arm (shifted only, about 80 s; the baseline
is archived) needs separate approval and would add history and tail
coverage. Turning on `weighted_mean` in live sessions remains a separate
decision. Live sessions would also change the
planner root age by δ.
