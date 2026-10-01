# Causal proximal Phase 3 timing audit: 20260919_194147

## Outcome

**REJECT for Phase 3 causal timing/model identification; PASS for session safety and partial transport-latency evidence.**

The run completed all 113 scheduled episodes and returned to its start pose without a manager or firmware fault. However, the measured physical-shaft command traces do not implement the requested simultaneous and tendon-leading schedules. The minimum-speed lift was applied in coupled logical coordinates before conversion back to raw shafts. This canceled, delayed, or reintroduced raw shaft-0 motion and changed the commanded travel. Consequently, this session must not be used to fit a timing-aware Jacobian or to decide whether insertion-leading, simultaneous, or tendon-leading actuation best explains the coupled response.

Session:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260919_194147_causal_proximal`

## Runtime identity and safety

- Real hardware, UKF marker estimator.
- Controller command output disabled and controller disarmed.
- Model adaptation disabled, as required.
- Rotation velocity limit zero.
- Distal artifact SHA-256: `adfbe11f58d409c9a24fdbdef23ef38b32c83fcc692ef77fa68c52508254ac1a`.
- Initial Jacobian SHA-256: `6d3f8573d10511dcbea3c27c97141f132b95cc42c450d3aec3702db33430ad2c`.
- No manager/device safety fault and no firmware latched fault.
- 34 `MOTION_SUSPECTED:STALL`, 18 `MOTION_RETRYING:STALL`, and 14 `MOTION_RECOVERED:STALL` advisories; no confirmed motion fault.

The experiment ran for 643.49 s versus a planned 643.51 s. Its explicit return succeeded in 0.660 s, ending at `[20.0079, 0, 0.00045]` from a target of `[20.0064, 0, 0.00104]`.

The launch again did not write `completeness.json`, but the bag contains the full scheduled episode sequence and a successful `run_end` event.

## Healthy downstream timing

For commands that were actually present on a raw shaft:

- causal trace to device-transmit onset was typically **1–3 ms**;
- command to credible encoder onset was typically **about 25–40 ms** on both raw shaft 0 and raw shaft 2;
- the shaft-2 encoder polarity is opposite the positive motor command, which the generated evaluator did not account for.

The valid initial onsets in the insertion-leading trials tracked their requested offsets:

| Requested insertion lead | Source onset skew, median | Encoder onset skew, median |
|---:|---:|---:|
| 20 ms | 19.2–19.7 ms | 25.1–25.4 ms |
| 40 ms | 39.3–39.5 ms | 35.9–47.0 ms |
| 80 ms | 79.2–79.4 ms | 79.9–80.3 ms |

Thus there is no evidence here for a fixed 15/30 ms insertion-versus-tendon transport delay. A requested 40–80 ms insertion lead survives the manager, serial bridge, and encoder observation with roughly one telemetry-period resolution. This makes an unexplained transport offset unlikely to be the principal cause of the earlier model-prediction error.

## Command-generation failure

The physical commands were supposed to be constructed directly in raw shaft coordinates:

- insertion only: raw 0 active, raw 2 held;
- tendon only: raw 0 held, raw 2 active;
- simultaneous: both begin together;
- lead/lag trials: one raw shaft begins 20, 40, or 80 ms before the other.

Instead, `_trajectory_velocity()` applied the reliable-speed floor independently to logical insertion and logical bending. `enforce_basis()` then transformed these floored logical commands back into raw coordinates. When both logical components were lifted to the same value, their difference made raw shaft 0 zero even when insertion was requested.

The bag directly shows the resulting failures:

- **Slow simultaneous:** raw shaft 2 began first; raw shaft 0 began about **1.23 s later**, not simultaneously.
- **Fast simultaneous:** raw shaft 0 began about **0.33 s after** raw shaft 2.
- **Slow tendon-leading trials:** requested raw-0 delays of 20/40/80 ms became approximately **1.25/1.25/1.27 s**.
- **Fast tendon-leading trials:** requested delays became approximately **0.33/0.35/0.37 s**.
- **All insertion-leading trials:** their first onset spacing was correct, but raw shaft 0 switched on, off when tendon motion began, then on again. The full waveform was therefore not a continuous insertion-leading excitation.
- **Slow tendon-only trials:** raw shaft 0 was initially held but later became active; median integrated raw-0 command was about **2.7 units** and its encoder span reached thousands of counts. Fast tendon-only trials remained isolated.

The same lift also changed total travel. The nominal single-axis excursions were 6.0 units on raw shaft 0 and 5.5 units on raw shaft 2, whereas integrated requested command was approximately:

| Trial | Slow | Fast |
|---|---:|---:|
| insertion only, raw 0 | 11.2 | 7.1 |
| tendon only, raw 2 | 7.5 | 6.9 |

These are not small timing perturbations. They change the input trajectory, endpoint, mechanical history, and amount of backlash/takeup traversed.

## Generated-summary defects

`causal_experiment_summary.json` should not be interpreted literally for Phase 3 onset metrics:

1. The encoder-onset detector assumes encoder displacement has the same sign as the motor command. Raw shaft 2 has the opposite encoder polarity, so all shaft-2 encoder onsets were reported as missing.
2. The detector searches the whole episode for any nonzero raw command. It therefore treats late, unintended raw-0 motion as a legitimate onset in tendon-only and tendon-leading trials.
3. It does not validate the required command topology (inactive shaft held, one onset per active shaft, requested lead/lag preserved, and expected integrated travel).
4. The preliminary `has_all_fitting_bases=false` value is expected for a Phase 3-only schedule and is not a session failure.

The reported negative encoder-to-estimator delays and empty shaft-2 group statistics are products of these analysis assumptions, not evidence of acausal plant response.

## What remains usable

- Safety, runtime identity, initialization, collection duration, and return-to-start evidence.
- Source-to-manager/device transport latency for commands actually emitted.
- Command-to-encoder latency after applying the correct encoder polarity.
- The first onset of insertion-leading trials, including preservation of 20/40/80 ms ordering.
- Static estimator noise estimates and advisory labels, with the usual caution around retry intervals.

Do not use the coupled response windows, endpoint gains, or tendon-leading/simultaneous groups for parameter identification.

## Required correction before rerun

1. Generate, floor, quantize, and validate Phase 3 commands in **raw shaft coordinates**. Convert to logical coordinates exactly once for publication.
2. Preserve the raw mask and raw onset schedule through position/model-limit projection; reject a trial before motion if projection changes an inactive shaft or the onset ordering.
3. Add online assertions for each trial: expected inactive shafts remain zero, active shafts have one contiguous pulse, source and transmitted onset errors remain within tolerance, and integrated raw travel matches the planned excursion.
4. Make the evaluator polarity-aware and fail the Phase 3 validity gate when command topology is violated.
5. Rerun Phase 3 only after a simulation/unit test proves simultaneous, insertion-leading, tendon-leading, and both single-axis waveforms at slow and fast speeds.

Until that correction is made, the earlier Phase 0–2 conclusion remains the reliable one: the local insertion and tendon directions are correct, but coupled transient prediction is history dependent. This session does not provide a valid causal estimate of the timing term responsible for that mismatch.

## Remediation status

Implemented in experiment protocol `causal_proximal_identification_v5` on
2026-09-19:

- Phase 3 now generates one constant-speed pulse per active physical shaft and
  converts raw shaft velocity to logical command coordinates exactly once.
- The logical per-axis minimum-speed lift is bypassed for Phase 3 because the
  raw pulses are already selected inside the experimentally reliable speed
  band.
- The collection node independently reconstructs the raw command, applies the
  manager's command projection, and aborts before further motion if an
  inactive shaft, pulse direction, or pulse magnitude would be changed.
- The evaluator is encoder-polarity aware and rejects trials with inactive
  shaft motion, fragmented pulses, direction reversals, onset error, or travel
  error.

Verification completed without hardware actuation: the corrected plan has 54
timing episodes and zero projected-command topology violations; 109 relevant
unit/regression tests pass; and the ROS `automation` package builds cleanly.
The old bag is intentionally classified as 0/54 topology-valid trials by the
corrected evaluator.
