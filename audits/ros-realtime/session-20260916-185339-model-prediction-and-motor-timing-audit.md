# Model-prediction and insertion/tendon timing audit

Date: 2026-09-19

## Scope

This is a passive, non-actuating analysis of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260916_185339_mppi_demo/20260916_185339_mppi_demo_0.db3`

The run disabled catheter rotation and exercised insertion plus tendon bending.
The question was whether relative motor timing between the coupled insertion
and bending shafts explains the poor short-horizon tip prediction.

Raw ENC samples, transmitted commands, planned controls, accepted markers,
estimator traces, controller diagnostics, and response traces were aligned by
their source timestamps. Marker tips and encoder counts were linearly
interpolated at the exact forecast start and due timestamps for the offline
comparisons below.

## Executive finding

Insertion/tendon timing mismatch exists, but it is not the primary explanation
for the poor prediction.

- Cross-correlation gives an approximately 15 ms command-to-encoder lag for
  physical motor 0 and 30 ms for tendon motor 2: a 15 ms relative lag.
- The host receives POS and ENC from the same telemetry cycle. Their median
  header skew is only 0.129 ms (P95 0.280 ms), so this is not a ROS timestamp
  pairing problem.
- Firmware uses a coordinated multi-axis start after direction setup. The
  measured difference is therefore a driver/encoder response delay at roughly
  one to two 100 Hz firmware cycles, not separate ROS commands.
- Only 22/154 response forecasts began within 40 ms of a physical command
  change. The other 132 began after the same command had already run for at
  least 40 ms.
- In those 132 steady-command forecasts, actual shaft travel matched the
  model's requested 40 ms shaft travel with 1.28% median relative error.
  Nevertheless, the interpolated tip response opposed the prediction in
  46.2% of forecasts, with 1.06 mm median endpoint error at the actual due
  timestamp.
- Endpoint error did not rise with shaft-timing error. Its correlation with
  the relative shaft-travel error was -0.40 in this capture.

At the maximum 4.5 mm/s bend compensation rate, a 15 ms relative delay creates
only about 0.068 mm of transient uncompensated insertion. It should be modeled
for high-fidelity short-horizon simulation, but it is too small and too
short-lived to explain the persistent 1 mm-scale error under already-stable
commands.

## Forecast instrumentation caveat

The online response metric does not compare equal time intervals (the already
open F-020 issue):

| Timing quantity | P50 | P95 |
|---|---:|---:|
| accepted marker precedes forecast root | 57.6 ms | 124.6 ms |
| plan publication after forecast root | 46.5 ms | 71.4 ms |
| first command TX after forecast root | 52.8 ms | 77.8 ms |
| first command TX relative to nominal 40 ms due time | +12.8 ms | +37.8 ms |
| selected observation after due time | 36.0 ms | 62.5 ms |

Thus the recorded `measured_delta_mm` and direction cosine are not calibrated
40 ms model metrics. For this audit, the primary endpoint was recomputed using
the marker tip interpolated at the forecast due timestamp. The command-TX delay
also does not necessarily mean no relevant command was executing: in most
forecasts the identical physical command was already active from an earlier
heartbeat.

This instrumentation defect exaggerates some individual response errors, but
does not remove the stable-command mismatch reported above.

## Effective local response mismatch

For the 132 steady-command forecasts, a two-column least-squares diagnostic
was fitted from measured physical shaft increments to (a) interpolated tip
motion and (b) the model's forecast endpoint relative to the measured current
tip. Rows are physical insertion motor 0 and tendon motor 2; columns are base
XYZ in mm/rad:

```text
measured plant:
[[-0.035, -0.049, +0.500],
 [-0.093, -0.119, +0.052]]

effective model forecast:
[[+0.536, -0.582, -0.744],
 [-0.555, -0.049, +0.298]]
```

The excitation matrix is full rank with condition 1.63, so this pooled result
is not caused by insertion/tendon collinearity. The insertion-motor response
directions have cosine -0.66 and the forecast norm is 2.16 times the measured
fit. The tendon response has cosine +0.72 but the forecast norm is 3.96 times
the measured fit. Per-target fits retain a negative insertion-column cosine
(-0.48 to -0.81). The tendon column is less stable and changes direction by
target/history, consistent with hysteretic distal loading.

This fit is a local diagnostic rather than a replacement Jacobian: the model is
nonlinear, the state changes across the trial, and its endpoint also includes
distal dynamics. It nevertheless proves that correct realized shaft travel did
not produce the response direction/gain assumed by the current rollout.

## Estimator and model-coordinate evidence

The camera estimator itself was healthy during the capture:

- 926/933 estimator traces reported `TRACKING` (the other seven were startup
  initialization);
- marker RMS after correction was 0.402 mm median and 0.604 mm P95;
- the fixed interface Jacobian had zero first-to-last change;
- online adaptation was disabled for every estimator trace.

The important structural mismatch is that the deployed runtime no longer sees
the raw calibrated shaft vector defined by the v171 handoff. Backlash
compensation substitutes a virtual transmitted angle before calling both the
interface Jacobian and the v171 transmission/history transition. In this run,
the absolute raw-minus-virtual angle difference was:

| Physical axis | P50 absolute offset | P95 absolute offset |
|---|---:|---:|
| insertion motor 0 | 6.22 rad | 15.94 rad |
| tendon motor 2 | 3.41 rad | 5.05 rad |

Passing those two coordinate systems through the loaded v171 motor
transmission changes its downstream coordinates by median absolute values
`[0.00250, 0, 0.000647]` and P95 values
`[0.00780, 0, 0.000957]`. The first quantity is metre-scale in the learned
transmission convention (about 2.5 mm median and 7.8 mm P95).

This matters because the v171 artifact owns a calibrated absolute motor
reference and was identified from unwrapped raw shaft angles. Its causal
tendon-history operator already learned history from that raw coordinate.
Applying an external take-up coordinate shift to the same input that advances
v171 history changes the model's absolute operating point. The UKF can still
fit the current marker geometry by correcting interface pose and strain, but
the next open-loop process transition can remain wrong. The large accumulated
history reconciliation (`-5.03` at the end, with last correction `-0.169`) is
consistent with the filter repeatedly repairing such process-state mismatch;
it is not by itself proof of the cause.

This is distinct from the planner's protection against double-counting a
configured rollout dead zone. The current code correctly prevents simultaneous
external compensation and MPPI rollout backlash, but it still uses one virtual
motor coordinate for both the proximal interface model and v171's calibrated
distal transmission/history.

## Causal interpretation

Observed with high confidence:

1. motor 2 responds about 15 ms later than motor 0;
2. most poor forecasts occur after that transient has ended;
3. steady measured shaft travel agrees with requested shaft travel;
4. the measured tip response gain/direction disagrees with the rollout;
5. UKF marker correction is healthy while the fixed Jacobian is not adapting;
6. the model is driven by a substantially shifted virtual motor coordinate.

Most likely causes, in priority order:

1. **raw/effective coordinate conflation:** external take-up estimation shifts
   the absolute input used by the calibrated v171 transmission and history;
2. **fixed local response model:** the v174 raw-shaft Jacobian, fitted on 0.25 s
   windows, is being used without adaptation for 40 ms predictions in a
   different hysteretic operating state;
3. **unmodeled load/history dependence:** especially the tendon response,
   whose empirical direction changes across targets;
4. **relative motor delay:** a real but secondary transient contribution.

## Recommended next correction and validation

1. Split runtime motor state into explicit coordinates:
   - `raw_motor_angle_rad`, preserving the v171 calibrated reference and its
     distal transmission/history input;
   - `effective_interface_motor_angle_rad`, used only for the proximal
     interface-pose Jacobian and take-up arbitration.
   Do not silently apply the virtual coordinate to v171 history unless that
   history model is re-identified in the virtual coordinate.
2. Repair response instrumentation per F-020: retain a bounded timestamped
   marker ring and compare interpolated tips at forecast root and due. Record
   the command already active at root separately from the newly selected plan.
3. Add the measured per-axis first-order delay (start with 15 ms motor 0 and
   30 ms motor 2) to simulation/rollout validation. Do not tune it from this
   single run as a permanent hardware constant.
4. Validate the coordinate split first in replay and simulation, then with a
   conservative, interior, single-direction insertion and isolated tendon
   pulse. Compare raw-coordinate v171, virtual-coordinate v171, and split-state
   predictions without online adaptation.
5. Only after selecting the correct coordinate contract, enable shadow
   Jacobian adaptation and require held-out causal improvement before allowing
   it to affect control.

No safety gate or hardware command path was changed by this audit.

## Implemented remediation (2026-09-19)

The first recommended correction is now implemented across the model/runtime
boundary.  Encoder counts remain the authoritative raw calibrated coordinate
for the v171 motor transmission, downstream state, causal tendon history, and
rewind/replay entries.  The backlash estimator's virtual transmitted angle is
passed separately and is used only for the proximal interface Jacobian and its
adaptation state.  MPPI advances both coordinates by the same post-engagement
increment during rollout, without moving the v171 absolute reference.

Controller diagnostics now expose both `model_raw_motor_angle_rad` and the
existing effective `model_motor_angle_rad`; the estimator trace retains raw
encoder counts and documents the effective-angle meaning of `motor_angle_rad`.
The focused v171 estimator tests (30), catheter-control tests (244), and
control-interface tests (70) pass, and the two ROS packages rebuild cleanly.
The next validation is a matched rotation-disabled sparse-point hardware run;
the correction does not weaken manager limits, freshness gates, firmware
watchdogs, or the explicit hardware-output interlock.
