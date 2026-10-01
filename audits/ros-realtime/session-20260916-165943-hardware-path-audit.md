# Hardware continuous-path fault and take-up audit

## Scope

This audit uses the recorded hardware session:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260916_165943_mppi_demo`

No hardware was commanded or reconfigured during analysis.

## Outcome

The controller reached 69.815 mm, approximately 95.7% of the requested path,
before faulting. The measured closest-path error was 3.704/8.583/10.935 mm at
P50/P95/maximum and 5.929 mm at the final path sample. The governor spent
3,667 of 4,855 active path samples in `TRANSMISSION_HOLD`, 827 in `PAUSED`,
250 in `SLOWED`, and only 111 in `RUNNING`.

## Terminal fault

The terminal `estimator_degraded` fault bypassed the configured three-frame
marker-rejection tolerance.

- Immediately before the fault, the estimator had accepted 3,263
  observations, was `TRACKING`, and the latest fit improved from 0.490 to
  0.449 mm RMS.
- The next observation was rejected as `marker_outlier`, with 4.514 mm RMS
  before correction. This was rejection 1 of 3.
- `_reject_live()` immediately changed the runtime-wide estimator health to
  `DEGRADED`.
- The heartbeat/readiness path checks estimator health before the consecutive
  rejection threshold and therefore faulted on `estimator_degraded` about
  9 ms after the warning.
- Further observations after disarming remained marker outliers (about
  5.5 mm RMS), so the mismatch was not merely one noisy frame. Nevertheless,
  the configured three-rejection policy was not what governed the stop.

The final marker rejection followed a normal post-take-up MPPI command
`[9.520, 0.0, -4.5, 0, 0, 0]`, not an active full-rate take-up command.
Slowing the take-up macro therefore would not, by itself, prevent this exact
fault transition.

## Take-up and reversal behavior

The active interval opened 162 take-up transactions in approximately 162 s.
Because a transaction is required only after a direction change, the counts
directly expose frequent reversal selection:

| Physical shaft | Transactions | Alternations between transaction signs | Median transaction duration | Median tip motion during transaction | Tip motion >2 mm |
|---|---:|---:|---:|---:|---:|
| insertion (0) | 78 | 77 | 0.501 s | 0.843 mm | 32.1% |
| rotation (1) | 81 | 80 | 0.899 s | 1.516 mm | 44.4% |
| tendon (2) | 20 | 19 | 0.100 s | 0.425 mm | 20.0% |

Rotation take-up was the clearest overshoot risk. It used the configured
40 logical units/s rate, moved the measured tip by as much as 6.128 mm in one
transaction, and worsened closest-path error in 49.4% of sampled transactions.
Insertion take-up moved the tip by as much as 7.344 mm and worsened path error
in 34.6% of sampled transactions. These transaction-level associations are
not pure single-axis causal estimates because some transactions contain more
than one pending shaft, but the rotation result is consistent with the
operator-observed stick-slip and overshoot.

The full-rate transaction ends only after an accepted camera/UKF correction
provides credible response evidence. Consequently, any motion after physical
engagement but before the next usable visual correction is open-loop at the
fixed take-up velocity. Lowering the rate reduces this sampling-delay
overshoot approximately proportionally; it does not reduce the optimizer's
propensity to request a reversal.

## Recommended correction order

1. Repair the estimator gate so transient rejected observations obey one
   explicit policy. A marker rejection below `maximum_marker_rejections`
   should neither stop the controller through the generic health gate nor
   bypass a genuinely fatal estimator-state condition. Preserve fail-closed
   behavior for nonfinite/invalid model state and for the configured rejection
   threshold.
2. Run an isolated rotation take-up-rate comparison before changing all three
   shafts. A conservative first candidate is 20 rather than 40 logical
   units/s for shaft 1, keeping `[8.0, 20.0, 4.5]` otherwise unchanged. The
   response observer accumulates evidence across camera frames, so this should
   reduce post-engagement travel rather than eliminate observability, although
   transaction duration will increase and must be measured.
3. Do not treat the speed reduction as the reversal fix. The controller still
   requested roughly one new transaction per second and froze reference
   progress for 75.5% of path samples. After the rate experiment, raise the
   reversal benefit/persistence requirement or add a path-scale hysteresis
   test using recorded candidate costs; verify that transaction count falls
   without locking a genuinely necessary reversal.
4. Re-run at 1 mm/s and compare transaction count, transaction tip displacement,
   closest-path P95, estimator rejection count, and completed path fraction.

## Evidence classification

- The gate bypass, transaction counts, error distributions, and command/state
  sequence are observed in the bag and source.
- The conclusion that lower take-up speed will reduce the delay-induced part
  of overshoot is inferred with high confidence from the fixed-rate,
  observation-terminated transaction design.
- The magnitude of improvement at 20 units/s is unknown until a matched
  hardware run is collected.

## Remediation implemented

- Lifecycle readiness now recognizes a degraded estimator state as a
  transient marker rejection only when the latest marker update was actually
  rejected and the rejection count remains strictly below the configured
  maximum. Counts 1 and 2 of 3 preserve the last valid control posterior;
  count 3 still fails closed through `marker_feedback_degraded` and the marker
  callback retains its explicit `repeated_marker_rejection:<reason>` fault.
- Accepted updates that leave estimator health degraded, or any degraded state
  without a current rejected marker result, remain immediate
  `estimator_degraded` failures. This prevents the tolerance from masking an
  internal filter/adaptation fault.
- Diagnostics explicitly report `transient_marker_rejection_tolerated`.
- The reviewed hardware profile now uses take-up velocity
  `[8.0, 20.0, 4.5]`, changing rotation only. Reversal-scheduler thresholds
  were deliberately left unchanged so the next hardware run isolates the
  effect of take-up speed.
- Focused tests pass 59/59, all `catheter_control` tests pass 235/235, touched
  lifecycle/test files pass `ament_flake8`, both modified Python modules
  compile, the ROS package rebuild passes, and the installed profile resolves
  the new take-up velocity.

Live hardware verification remains pending.
