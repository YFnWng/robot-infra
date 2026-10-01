# 2026-09-12 hardware MPPI response and Jacobian audit

## Scope and evidence

This is a passive, non-actuating audit of the hardware run identified as
`2026-09-12-17-43-56-913908-BM2-Desktop-1-4013643`.

The supplied identifier is a ROS launch-log directory, not a rosbag. It contains
the manager and serial bridge log. The six mode-3 intervals in that log were
correlated by ROS timestamps with the controller log
`~/.ros/log/python3_4023434_1789250192162.log`.

The most recent bag found for this setup,
`20260912_165620_mppi_demo`, ends at ROS time `1789249429.996`, before the first
actuated interval at `1789250297.052`. It therefore does not contain the active
trials. In particular, this audit has no time series of the realized actuator
command, encoder motion, current, or the per-status Jacobian/RLS diagnostics
during those trials.

The Jacobian inspected below is the v174 shaft-coordinate artifact in
`cr_meta_lnn/evaluation/real_joint_local_distal_v174.json`. The controller source
defaults to `adaptation_enabled=false`, and the fixed-J hardware command supplied
for this test also selected false. Because the active status topic was not
recorded, fixed-J operation is high-confidence context rather than independently
verified from the active run.

## Executive finding

The controller/model substantially over-predicted short-horizon tip motion.
Across 297 camera-matched, 40 ms forecasts:

- predicted displacement norm: 1.437 mm median, 3.174 mm p95;
- measured displacement norm: 0.094 mm median, 0.295 mm p95;
- measured/predicted norm ratio: 0.0645 median;
- 88.2% of measured displacements were below 0.2 mm;
- 68.4% of measured/predicted ratios were below 0.1;
- measured motion opposed the predicted direction in 45.1% of forecasts;
- endpoint error was 1.435 mm median.

At the same time, four later trials accumulated 5.2--10.9 mm of measured tip
motion over 23--76 seconds. The plant is therefore capable of meaningful
motion, but its response over the controller's 40 ms prediction horizon is
usually near the marker-noise scale and is not represented by the rollout.

This is strong evidence for a missing short-time transmission dynamic such as
windup, backlash, stiction, or hysteresis. It does **not** uniquely prove
torsional windup, because the active run did not record commands and encoders.
A gain/unit error or commanded-versus-realized actuator mismatch could produce
part of the same signature and cannot be excluded from these logs alone.

Scheduling was not the dominant limitation in this run. There were seven
isolated deadline warnings over six trials, only one adjacent pair, no planner
fault, and no marker-rejection storm after the single startup
`observation_before_rewind_buffer` warning.

## Trial summary

The vectors below are target minus observed tip in registered robot-base XYZ.
Net tip motion is inferred from the change of that vector during each fixed
target interval.

| Trial | Initial error XYZ (mm) | Final norm (mm) | Minimum norm (mm) | Net measured tip travel (mm) | Median predicted / measured 40 ms response (mm) | Median response ratio |
|---:|---|---:|---:|---:|---:|---:|
| 1 | `[5.096, 0.031, 0.059]` | 4.781 | 4.547 | 4.042 | 2.957 / 0.066 | 0.024 |
| 2 | `[-0.003, 5.000, 0.023]` | 4.607 | 4.353 | 5.617 | 3.133 / 0.102 | 0.032 |
| 3 | `[-0.089, -0.054, 9.998]` | 1.143 | 0.931 | 10.917 | 0.683 / 0.114 | 0.129 |
| 4 | `[-2.058, 5.133, 0.025]` | 1.569 | 1.524 | 5.188 | 0.856 / 0.109 | 0.105 |
| 5 | `[-2.069, 4.968, -0.058]` | 2.468 | 2.050 | 5.521 | 1.381 / 0.101 | 0.074 |
| 6 | `[-5.000, 5.146, 0.046]` | 1.621 | 1.608 | 7.100 | 1.404 / 0.094 | 0.064 |

Trial 3, predominantly +Z, was the cleanest result. Trials 1 and 2 began as
nominal single-axis Cartesian targets but developed large cross-axis error and
stalled around 4.6--4.8 mm. This supports a direction-dependent model mismatch.
It cannot be assigned to one motor/Jacobian column without the missing realized
command and encoder series.

For tracking errors below 2 mm, the median predicted response fell to 0.750 mm,
but the median measured response remained 0.094 mm. Thus the absolute observed
motion stays close to a noise/dead-zone floor near the target. The mismatch is
not confined to close targets: at 4--6 mm error, the median prediction was
3.025 mm and the median measured motion only 0.085 mm.

## Jacobian audit

The model uses the body-frame relation

`Log(g0[k]^-1 g0[k+1]) = J * delta_motor_shaft_angle`

with columns ordered insertion, rotation, and bending and rows ordered angular
then translational twist. The loaded initial shaft Jacobian is

```text
[[-0.0011517,  0.0028368, -0.0023445],
 [-0.0009672,  0.0139283,  0.0001114],
 [-0.0075678,  0.0589212,  0.0025082],
 [ 0.0000028,  0.0001610, -0.0000051],
 [ 0.0000779,  0.0000039,  0.0000132],
 [ 0.0004332,  0.0000302, -0.0001322]]
```

Its singular values are `[0.061136, 0.002592, 0.001005]`, giving condition
number 60.81. It is full-rank but anisotropic. Per motor-shaft radian, the
angular column norms are `[0.00772, 0.06061, 0.00344]` rad/rad and the direct
interface-translation column norms are `[0.440, 0.164, 0.133]` mm/rad.

The rotation column is dominated by material-frame `omega_z = 0.05892` rad per
shaft radian. In the current rollout implementation, `J * delta_action` is
applied immediately as an SE(3) interface-pose increment. There is no latent
torsional-load, play, or direction-dependent release state in this path. The
only history state is driven by downstream bending/tendon coordinate 2. Thus a
rotation command is predicted to rotate the material interface immediately,
whereas a real shaft can first wind up compliance or traverse backlash. This is
the precise structural mechanism by which torsional windup could create the
observed response gap.

The artifact's offline causal adaptation ended with angular column norms
`[0.00244, 0.06252, 0.00389]`. Most of its total Jacobian change was in the
insertion column: its angular gain fell to 31.6% of the initial value while its
direction remained similar. The rotation-column angular gain changed by only
3.2%. Therefore the calibration artifact does not itself show an unstable
static rotation column. The suspected failure is more plausibly the use of one
memoryless local Jacobian across loading, dead-zone, reversal, and released
states than simply a bad constant rotation-column coefficient.

The condition number also matters operationally: small estimation/model errors
along the weak singular direction require much larger actuator changes, while
MPPI may prefer the apparently strong rotation response. Mechanical attenuation
then invalidates both the predicted magnitude and direction at short horizons.

## What is observed versus inferred

Observed:

- severe 40 ms predicted-versus-measured response mismatch;
- response often at the marker-noise scale;
- meaningful motion accumulates on a seconds-to-minutes time scale;
- direction-dependent tracking outcome;
- only sparse planner deadline misses;
- an anisotropic, condition-60.81 fixed local Jacobian;
- no explicit torsional/play state in the model rollout.

Inferred:

- torsional windup/backlash is a credible primary mechanism;
- the current horizon is shorter than the effective mechanical response time
  during much of the run;
- a single fixed Jacobian cannot represent loading and unloading branches.

Not identifiable from this capture:

- how much commanded shaft motion was actually realized;
- which motor axis was active in each 40 ms forecast;
- dead-zone width, reversal delay, or stored torsional displacement;
- whether the online Jacobian was bit-for-bit unchanged during active control;
- whether motor current/driver saturation contributed.

## Required next diagnostic capture

Before changing the controller or enabling Jacobian adaptation, record one
mechanically conservative identification run containing, on the same ROS clock:

- requested logical velocity and projected/quantized realized command;
- `/device/data` predicates 69 and 80 (encoder counts and position);
- observed and predicted tip pose/response;
- full MPPI status including Jacobian, RLS reason, and adaptation gates;
- manager mode/status and marker diagnostics.

Use isolated positive, hold, negative, and hold pulses for each motor coordinate,
starting with rotation at a safely bounded magnitude. The decisive plots are
shaft-angle change versus material-roll estimate, and shaft-angle change versus
tip displacement, separated by direction and by time since reversal. A loop,
flat initial segment, or delayed release would measure hysteresis, backlash, or
windup directly. Adaptation should remain disabled during this identification
so the plant effect is not confounded with a changing model.

## Recording remediation implemented 2026-09-12

The hardware controller launch now records by default and adds the previously
missing manager state, serial transport state, ROS logs, parameter events, and
post-serial-write command trace. The new
`/catheter_mppi/response_trace` message binds each causal 40 ms response to the
logical command, quantized motor-shaft rate used by the rollout, joint/model
state, material interface pose, exact Jacobian, target, and predicted/measured
tip positions. A sibling JSON manifest records resolved launch parameters and
artifact hashes. This closes the software-side capture gaps above; direct motor
current or independent shaft sensing remains unavailable.
