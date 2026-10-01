# Causal proximal Phase 0–2 audit: 20260919_190828

## Outcome

**REVIEW — collection succeeded and the Phase 0–2 experiment is usable.**

The initialization, isolation routing, estimator recording, and return-to-start path all operated without a manager or firmware fault. The data establish the correct local response directions for insertion and tendon bending, but also show gain mismatch, history dependence, and imperfect superposition. These are model-identification findings rather than collection failures.

Session:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260919_190828_causal_proximal`

## Runtime identity

- Real hardware; controller command output disabled and controller disarmed.
- Schedule: `phase_0_2`.
- Marker estimator: UKF.
- Model adaptation: disabled, as required for the isolation test.
- Distal artifact SHA-256: `adfbe11f58d409c9a24fdbdef23ef38b32c83fcc692ef77fa68c52508254ac1a`.
- Initial Jacobian SHA-256: `6d3f8573d10511dcbea3c27c97141f132b95cc42c450d3aec3702db33430ad2c`.
- Rotation velocity limit: zero.

## Initialization and completion

The new manager-mediated initialization ran before data collection:

- Initial POS: `[10.0024, 0.0000, 4.5461]`.
- Requested initialization target: `[20, 0, 0]`.
- Initialization duration: 3.291 s.
- Final initialized POS: `[19.9960, 0.0000, -0.0018]`.
- Absolute residual: `[0.0040, 0.0000, 0.0018]`.

The experiment executed all 22 scheduled episodes: two static blocks, six insertion-only episodes, six tendon-motor-only episodes, six compensated-bending episodes, and the bend-bias entry/exit operations. All 22 episode starts have matching episode ends.

The post-run return also succeeded:

- Final returned POS: `[19.9938, 0.0000, 0.0010]`.
- Maximum return error: 0.0023 in the configured joint units.
- Firmware status at completion: no enabled or latched faults.

## Recording integrity

The rosbag contains the required data streams:

| Topic | Messages |
|---|---:|
| `/collection/causal_trace` | 47,118 |
| `/manager/control` | 47,484 |
| `/shape_tracking/markers` | 14,637 |
| `/catheter_mppi/estimator_trace` | 9,588 |
| `/device/state` | 88,540 |
| `/collection/events` | 53 |

The collection checker evaluates the session as complete. However, the launch did not write `completeness.json`; this is a sealing/orchestration defect and not evidence of missing bag data.

The summary field `has_all_fitting_bases=false` is expected for this schedule because Phase 0–2 intentionally omits the rotation/shaft-1 basis.

## Isolation validity

The command and encoder traces confirm the intended experiment routing:

- Insertion-only episodes actuated shaft 0 and produced no shaft-2 encoder excursion.
- Tendon-motor-only episodes actuated shaft 2 while shaft 0 remained fixed; shaft-0 encoder span was exactly zero.
- Compensated-bending episodes actuated shafts 0 and 2 together, capturing the production insertion compensation path.
- Shaft 1 remained disabled throughout.

This means the insertion and tendon columns can be evaluated independently before examining their coupled command.

## Static estimator floor

Across 899 static estimator increments:

- Interface translation increment p99: 0.0625 mm; maximum: 0.0940 mm.
- Interface rotation increment p99: 0.001679 rad (0.0962 deg); maximum: 0.002142 rad.

These are interface-pose increment statistics, not direct distal-tip thresholds.

## Local response fits

Short-window fits used 0.25 s accumulated response windows. Compared with the deployed v174-initialized local Jacobian:

### Shaft 0: insertion

- Direction cosine: **0.984**.
- Measured/model norm ratio: **0.665**.
- Residual/response RMS ratio: **0.455**.

The insertion column therefore has the correct local direction, including its dominant positive/negative axial mapping under the deployed convention, but the deployed column predicts about **1.50 times** the measured short-window response magnitude.

### Shaft 2: tendon bending

- Direction cosine: **0.978**.
- Measured/model norm ratio: **0.730**.
- Residual/response RMS ratio: **0.791**.

The tendon column also has the correct local direction, but the deployed column predicts about **1.37 times** the measured short-window magnitude. Its larger residual indicates materially stronger transient or history dependence than insertion.

The higher-action subsets give consistent conclusions, so these gain differences are not explained solely by static estimator noise.

## Coupled compensated bending

Using the independently fitted shaft-0 and shaft-2 columns to predict the compensated-bending episodes gives:

- Slow, higher-action windows: residual/response RMS 0.771; median direction cosine 0.873.
- Fast, higher-action windows: residual/response RMS 0.742; median direction cosine 0.946.

Typical direction is reasonable, but low-percentile direction cosines are negative. The worst windows cluster around reversals, endpoints, and motion-watchdog retry periods. A single static linear superposition therefore does not explain the whole coupled transient.

This result supports the planned timing/history isolation phase. It does **not** yet justify replacing the deployed Jacobian with one aggregate fit from this session.

## Motion advisories

There were 24 motion-watchdog advisories and no confirmed motion fault. Advisories occurred primarily in tendon-motor-only and compensated-bending episodes, with four retry events. The encoder telemetry remains valid, but fitting should either exclude retry intervals or explicitly label them as a separate regime.

## Endpoint remanence

Comparing the late static windows before the explicit post-run return:

- Interface translation changed by approximately `[0.114, 0.176, 1.189]` mm.
- Interface orientation changed by approximately `[0.221, -0.754, 2.507]` deg as a relative rotation vector.
- Estimated tip changed by approximately `[0.471, 1.086, 0.334]` mm (1.23 mm norm).
- Observed tip changed by approximately `[0.552, 1.183, 0.377]` mm (1.36 mm norm).
- POS changed by approximately `[0.290, 0.000, 0.191]`.

The final static block intentionally applies zero command rather than position-correcting the residual, so this is useful evidence of endpoint/remanent state. The bag does not contain a sufficiently long static observation after the explicit return to determine how much of this distal state subsequently relaxed.

## Conclusions and next test

1. The automatic `[20, 0, 0]` initialization is validated and should remain enabled for subsequent isolation tests.
2. The deployed insertion and tendon Jacobian columns point in the correct directions; the principal discrepancy is local gain plus history-dependent dynamics, not a column sign or axis-routing error.
3. The compensated response is not adequately described by instantaneous linear superposition across reversals and endpoints.
4. Proceed to Phase 3 timing/history isolation before changing the production Jacobian. Estimate command-to-encoder and encoder-to-interface response delays separately, segment by direction and reversal history, and exclude or label watchdog retry intervals.
5. Repair the launch/checker chaining so successful future sessions always produce `completeness.json`.
