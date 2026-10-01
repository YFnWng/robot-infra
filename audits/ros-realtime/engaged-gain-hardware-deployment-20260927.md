# Engaged-gain hardware deployment audit — 2026-09-27

## Outcome

Software preflight passed; powered hardware remains unqualified. The deployed
path preserves one mutable estimator owner, immutable planner snapshots,
response-terminated take-up, zero-barrier replanning, manager projection,
freshness checks, watchdogs, and fault latching. Encoder zero remains read-only.

## Architecture finding

Accepted delayed UKF updates now provide source-time nominal/posterior distal
equilibrium evidence to the rewindable backlash checkpoint. Gain updates occur
only while the tendon shaft is `ENGAGED`. The planner consumes the immutable
snapshot and evaluates mean/lower/upper gain scenarios in one GPU batch. During
`TAKEUP` or `PROVISIONAL`, the transaction arbiter remains the only command
source; MPPI execution is inhibited and the old plan is invalidated.
All three commanded physical shafts now follow this rule. The former tendon-axis
raw-response exception is disabled, while accepted distal bending remains valid
observation evidence for terminating tendon take-up. Confirmation emits a zero
barrier and the next executed command always comes from a fresh MPPI plan.

## Timing

Exact non-actuating CUDA preflight used 384 candidates × 3 gain scenarios × 4
steps on the RTX 4090. Across 100 trials: p50 16.78 ms, p95 17.96 ms, p99
18.58 ms, max 18.69 ms; all plans met the 60 ms deadline. This isolated gate
does not include camera, serial, DDS, or executor contention, so a recorded
full-stack hardware run must confirm the distribution and zero deadline misses.

## Required first hardware gate

Use the no-rotation hardware profile. First run with command output disabled to
verify hashes/parameters and diagnostics. After the normal serial, driver-power,
marker, estimator, and `MANAGER_READY` checks, explicitly restart with output
enabled and run only the axial test, followed by the near model-generated
two-axis test. Stop on any rotation command, take-up/MPPI overlap, estimator
degradation, deadline miss, or fault.

## Evidence

- 35 focused `cr_meta_lnn` runtime/UKF tests passed.
- 276 `catheter_control` tests passed after rebuild.
- Preflight report: `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260927_180602_phase5_preflight.json`.
