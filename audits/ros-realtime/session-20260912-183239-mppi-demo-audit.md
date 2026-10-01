# Hardware MPPI audit: 20260912_183239_mppi_demo

## Scope

Passive audit of the recorded hardware session at
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260912_183239_mppi_demo`.
The run used the UKF estimator, command output enabled, and fixed proximal
Jacobian (`adaptation_enabled=false`). No hardware actions were performed as
part of this audit.

## Executive result

The final `position_feedback_out_of_range` fault was a valid fail-closed
response. Logical catheter insertion (axis 0) crossed its 40 mm upper limit
during a command whose insertion component was zero. The crossing occurred
because catheter bending is implemented by two coupled physical motor axes and
their measured motions did not cancel as the ideal firmware/control model
assumes.

This is distinct from, but compounded by, a model-response mismatch. Offline
replay finds the fixed proximal Jacobian generally has the right angular
directions, but substantially overestimates several interface translation
responses. Full tip motion is also much smaller than predicted for most
250 ms windows.

Do not re-arm from the terminal state: the reported axis-0 position remains
slightly outside the configured hard limit and the existing controller has no
coupled-axis guard that can prevent a repeat near this boundary.

## Limit-crossing evidence

- Configured logical axis-0 bound: `[0, 40]` mm.
- Last valid POS sample before the crossing:
  `[39.997749, 21.016125, 1.137791, 0, 0, 0]`.
- First invalid POS sample:
  `[40.020813, 21.201752, 1.114574, 0, 0, 0]` at bag time
  `1789252833.3040245`.
- Peak recorded axis-0 position: `40.022282` mm.
- Near the crossing, transmitted logical commands were either
  `[0, 0, -2.150674, 0, 0, 0]` or
  `[0, 8.37558, -2.123124, 0, 0, 0]`. Controller intent,
  `/manager/control`, and `/device/command_tx` agree; there is no evidence of
  a relay substitution or a direct positive insertion request.
- Zero commands appear immediately after the invalid sample. The manager and
  controller both latched the unsafe state, so the downstream safety response
  worked as designed.

For the catheter axes, firmware maps logical velocity to physical motor
velocity using `motor[0] = insertion - bending`, while reported logical
insertion is reconstructed as `position[0] = motor_position[0] +
motor_position[2]`. Ideal, simultaneous bend-axis motion therefore cancels
from logical insertion.

Over the final approximately 1.5 s active interval, raw encoder counts changed
by approximately `[+970, +4170, +1176]` on the three model axes. With the
firmware transmission constants this corresponds to about `+0.408 mm` on
physical motor axis 0 and `-0.175 mm` on physical motor axis 2, leaving about
`+0.233 mm` of uncancelled logical insertion. The observed logical POS increase
over nearby endpoints was about `+0.256 mm`. By contrast, integration of the
quantized transmitted commands predicts essentially zero insertion residual
(less than 0.001 mm over that interval). Receive-time integration is only an
approximation, but the discrepancy is far too large to be explained by RPM
rounding.

The evidence therefore points to differential response of the two physical
motor channels used for catheter bending (driver dynamics, load, stiction, or
backlash), not to a ROS command-routing error. Because the recorded encoders
are motor-output-shaft measurements, this particular limit fault is upstream
of distal catheter torsional windup, although both effects may coexist.

## Model and Jacobian evidence

The recorded Jacobian is constant throughout the run and matches the loaded
v174 initialization artifact, as expected with adaptation disabled.

An offline UKF replay used 250 ms equal-duration windows. It produced 316
excited windows with action rank 3 and condition number 2.33, so the aggregate
excitation is adequate for a diagnostic fit. Important limitations are that
the fitted interface pose is UKF-corrected rather than independent ground
truth, and individual pure-axis subsets are smaller.

### Full tip response

- Measured motion norm: median `0.144 mm`, p95 `1.856 mm`.
- Predicted motion norm: median `1.458 mm`, p95 `4.168 mm`.
- For the 83 responses above the 0.25 mm direction threshold, median direction
  cosine was `0.717` but median signed gain (measured/predicted) was only
  `0.141`.
- In stationary encoder windows, apparent tip motion had median `0.109 mm` and
  p95 `0.252 mm` per 250 ms. Small commanded responses are therefore readily
  submerged by estimator/measurement motion.

### Diagnostic Jacobian fit relative to fixed J0

| Motor column | Angular direction / gain | Linear direction / gain |
| --- | ---: | ---: |
| insertion | 0.882 / 0.804 | 0.996 / 0.587 |
| rotation | 0.968 / 0.716 | 0.466 / 0.110 |
| bending | 0.980 / 0.555 | 0.932 / 0.104 |

The angular column directions are broadly consistent but their magnitudes are
lower than J0. The fitted linear rotation column is both weakly aligned and
about one tenth of the J0 projection; the fitted linear bending column is
aligned but also about one tenth of J0 magnitude. The pure bending tip subset
contains only three windows, so a replacement bending column should not be
accepted from this trial alone.

## Findings and recommended order

1. **Critical: coupled position protection is incomplete.** Independent
   logical-axis projection assumes exact cancellation after firmware coupling.
   It cannot protect logical insertion when bend motor channels track
   differently. Add a conservative coupled-axis boundary guard based on actual
   POS margin and measured differential response. Near either insertion bound,
   suppress or taper bend commands unless the coupled motion can be proven
   inward-safe.

   **Bounded recovery implemented; prevention still pending:** The manager now
   exposes `/manager/recover_catheter_linear_limit` for exactly one small
   logical axis-0 excursion. It performs disabled-state firmware and driver
   checks, emits only minimum-speed inward axis-0 commands, monitors live
   feedback, applies a STOP barrier, and leaves ordinary qualification revoked.
   This removes the recovery deadlock but does not prevent coupled drift during
   MPPI; the boundary guard above remains required.
2. **High: do not solve the fault by widening the feedback tolerance.** The
   tolerance only changes when the fault fires; it does not prevent physical
   boundary crossing.
3. **High: qualify coupled bending response away from hard limits.** Record a
   small, supervised bend-only pulse and reversal sequence with sufficient
   insertion margin. Compare both raw motor encoder increments, realized RPM,
   and logical insertion residual. This identifies driver lag/stiction before
   another autonomous run.
4. **High: fixed J0 is optimistic.** After coupled motion is safe, repeat
   sufficiently large, separated insertion/rotation/bending excitation and use
   the robust accumulated-motion adaptation gates. The present trial supports
   adaptation, but does not by itself provide trustworthy replacement columns.
5. **Medium: close-target control needs a noise-aware deadband.** A target or
   predicted response near 0.1--0.25 mm is comparable to stationary apparent
   motion over a camera-scale window. Avoid interpreting such data as plant
   response or Jacobian evidence.

## Interpretation boundary

This audit establishes a command/feedback coupling mismatch and a model
response mismatch. It does not identify whether the unequal physical-channel
response originates in driver acceleration, motor loading, transmission
stiction, or another mechanism. That distinction requires the supervised
open-loop coupled-response qualification proposed above.
