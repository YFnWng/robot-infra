# Gate 3 reduced filter: ROS shadow replay

2026-11-02. Research shadow, not deployed. Recorded data only: no hardware, manager, device link or control topic was involved. No output reaches a planner or controller. This is not live timing or live qualification.

## What was added

| Piece | Owner | Role |
|---|---|---|
| Recording-free startup | `cr_meta_lnn/training/hybrid/real_native_memory_bootstrap.py`: `load_native_artifacts`, `initialize_native_root` | Split out of the recorded bootstrap, which now calls them |
| Arrival-order wrapper | `cr_meta_lnn/training/hybrid/reduced_filter_stream.py` | Aligns encoders and images, then calls `ReducedEstimator` |
| Bounded readout window | `BoundedInterfaceReadoutFactory.retain_images` | Keeps long runs from growing the fit cache |
| Observer node | `runtime_supervision/reduced_filter_shadow.py` | Own process, one owner timer, private String status only |
| Recorded source | `runtime_supervision/reduced_filter_replay.py` | Refuses to run outside domain 214 and the `/reduced_filter_replay` prefix |

Estimator owners, settings and update order are unchanged. The path is physical gain, K=2, width 7.05 mm, with the stationary profile hold at 0.01 mm.

An image waits for the first encoder sample at or past its source time and is interpolated inside that bracket by the same owners as the recorded inputs. Later samples cannot reach its update.

## Results

Baseline: the batch replay of the same 1,221 images with the hold on. Tolerance 1e-6 mm, declared before running.

| Check | Result |
|---|---|
| Wrapper in recorded arrival order, no ROS | 1,221 images, position and lambda difference 0.0, no rejections |
| ROS replay at recorded rate, domain 214 | 1,221 images, position and lambda difference 0.0, no rejections |
| Messages | 10,938 of 10,938 encoders and 1,222 of 1,222 images received, in order |
| Graph | Subscribes to the two replay state topics. Publishes only its status plus the standard ROS parameter and log topics. No service or action clients |
| Static test | No control publisher, client or `SET_ZERO` vocabulary |

The whole-session run used a 16-image queue. **With the default queue of 3 the short run failed the comparison:** cold startup (390 ms, then a 108 ms first step) dropped image 5, giving a 0.025 mm difference. So exact equivalence needs a queue that absorbs startup; the filter was not changed. The default stays 3, which bounds staleness and keeps the dropped image's encoder motion in the next interval.

Timing, ROS replay, images after startup catch-up (index above 12), no planner running:

| Measure | P50 | P95 | P99 | Max | Over 50 ms |
|---|---:|---:|---:|---:|---:|
| Estimator step, ms | | | 49.7 | 75.1 | 12 |
| Image arrival to output, ms | 39.1 | 47.4 | 54.1 | 79.7 | 36 |

Including startup, step P50 is 36.2, P95 43.7, P99 50.1, max 105.7 ms, and arrival-to-output P99 is 62.8, max 364 ms. Against the agreed P99 of 50 ms, the step passes only after startup, with almost no margin. No image exceeded the 100 ms spacing after startup, and no backlog formed. Recorded markers arrive about 37 ms after capture (P99 40 ms), so capture-to-output is roughly 76 ms typically. That is an estimate from separate distributions.

## Not covered

Image spacing is 100 ms. The recording's marker quality was all valid, and the node applies only `marker_measurement` validation, not a quality threshold. Rotation motion, gaps and faults were not exercised through ROS. Geometry, marker covariance and priors come from October recordings. The nodes were built into a private overlay, so `robot-infra/install` does not contain them yet.

Evidence is in `gate3_realtime_filter_evidence_20261101/ros_shadow_replay` in the session root.
