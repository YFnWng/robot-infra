# Identification excitation and hysteresis review

> **Correction, 2026-09-13:** Findings E2 and E4 below describe the material
> roll missing from the HDF5 `cosserat/interface_pose` itself. The selected
> v171 checkpoint contains `pose_control`, `roll_control`, `reference_pose`,
> and their spline interpolation buffers. `_SplineDistalLatent.values()`
> reconstructs a full posterior material interface pose, so the rotation
> episodes can be used retrospectively to fit the full rotation column. That
> posterior is offline, model-conditioned, noncausal, and uses 0.5 s spline
> knots; it still cannot certify online UKF gates or sub-0.5 s backlash timing.
> The corrected data analysis and controller decision are in
> `HYSTERESIS_AWARE_PROXIMAL_CONTROL_AND_CAUSAL_EXPERIMENT_PLAN.md`.

## Question

Can the labeled single-axis episodes in session `20260829_163456` replace a
new hardware excitation experiment for local proximal-Jacobian adaptation, and
what control approaches are supported by the literature for the observed
backlash, hysteresis, and torsional windup?

## Evidence and scope

Analyzed source:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260829_163456/processed_cosserat_dual_zed_rigid_tip_quotient_map_v18_gamma20/cosserat_states.h5`

The HDF5 contains 17,122 frames and 28 labeled episodes. Local SE(3)
increments were evaluated at approximately 0.25 s and 1.0 s horizons using
the recorded unwrapped motor-shaft coordinates. The episode metadata and
stall events were recovered from the associated robot bag.

The current v174 Jacobian was not fitted from this identification session. Its
artifact states that it was initialized from the clean 0--140 s portion of the
separate sinusoidal session `20260829_173140`, using 0.25 s transitions. This
review therefore evaluates whether `20260829_163456` is useful additional
evidence, not the data that originally produced J0.

## Finding E1: insertion excitation is sufficient for an offline local-column benchmark

Confidence: **observed**.

The dataset has slow, medium, and fast insertion episodes, both motion
directions, 1,819 approximately 0.25 s windows, and 1,759 approximately 1 s
windows. The actual shaft motion is purely axis 0.

At 1 s:

- median normalized action magnitude: 1.93;
- median/p95 interface translation response: 1.13/8.14 mm;
- 86.4% of windows pass the current absolute response floor;
- 63.3% pass an empirical 3x threshold relative to p95 dwell increments;
- positive and negative fitted normalized columns are almost identical:
  cosine 1.000 and relative difference 2.1%.

This is enough to estimate and cross-validate an insertion column across speed
and direction, and to measure the local dead zone around reversals.

## Finding E2: HDF5 quotient pose alone cannot identify the deployed full rotation column

Confidence: **observed**.

There are slow, medium, and fast rotation episodes with 5,267 approximately
1 s windows. At that horizon, 94.8% pass the absolute response floor and
43.4% exceed the empirical dwell-noise threshold. A stall retry occurred in
the medium-speed rotation episode.

The response is strongly direction dependent. Positive and negative
direction-specific normalized columns have cosine 0.30 and a 120% relative
difference. This is direct evidence that a single memoryless rotation column
is inadequate locally.

However, the HDF5 contract explicitly says:

- `cosserat/interface_pose`: material roll is unobserved and represented by a
  deterministic SO(2) quotient gauge;
- `cosserat/interface_body_twist`: axial angular velocity is an unobserved
  gauge value fixed to zero.

Consequently these episodes, when read only through the HDF5 quotient pose,
can characterize rotation-induced position and tangent effects, reversal dead
zones, and direction asymmetry, but cannot supervise the axial-material-roll
component required by the deployed 6-D material-frame Jacobian. The v171
checkpoint's later posterior roll reconstruction supplies that retrospective
signal, subject to its offline/model-conditioned limitations.

## Finding E3: the bending episodes are not single-axis in measured shaft space

Confidence: **observed**.

The commanded experiment labels are bending-only, but insertion floor
tracking moved shaft axis 0 by roughly 29--33 rad while bending shaft axis 2
moved about 78 rad. Therefore they are not clean single-column regressions in
the coordinates used by the deployed Jacobian.

They are nevertheless valuable:

- basic slow/medium/fast bending provides 1,607 approximately 1 s windows;
- 99.8% pass the absolute response floor and 59.3% exceed the empirical
  dwell-noise threshold;
- `bend_hold_unload_relax` and `repeated_bend_loops` directly exercise memory;
- positive/negative basic bending columns have cosine 0.962 but differ by
  28%; the memory episodes increase the direction difference to 42%;
- bending-related and coupled episodes contain recorded stall retries.

Use these episodes to fit a coupled insertion–bending map and a reversal/
take-up state. Do not label their response as an isolated axis-2 Jacobian
column without first accounting for the axis-0 floor-tracking motion.

## Finding E4: the data are not a substitute for one short causal hardware validation

Confidence: **inferred-high-confidence**.

The dataset is sufficient to develop and offline-test a robust adaptation
scheme. It is not sufficient to certify the online UKF/RLS gates because:

- its interface trajectory is an offline smoothed reconstruction over one
  continuous block, rather than the causal online UKF output;
- material roll is missing from the stored quotient pose, while the checkpoint
  posterior reconstructs it noncausally;
- it represents one mounting, transmission history, and acquisition session;
- most excitation amplitudes are much larger and more continuous than the
  small MPPI corrections where backlash dominated the hardware trial.

Static HDF5 dwell increments have p95 magnitudes of about 0.278 degrees and
0.205 mm at 0.25 s, and 0.325 degrees and 0.205 mm at 1 s. The present 0.1
degree absolute rotation threshold would therefore be crossed by offline pose
drift in many static windows; the separate action-motion gate is essential.

## Recommended use of the existing episodes

1. Build direction-conditioned, horizon-matched samples from the recorded
   motor shaft and reconstructed interface pose. Exclude a configurable
   post-reversal take-up interval and every stall-retry interval.
2. Estimate axis-0 insertion columns independently by speed and sign. Use
   their close agreement as an implementation check.
3. Reconstruct observable material roll for rotation episodes before fitting
   axis 1. Fit positive and negative directions separately and estimate a
   reversal deadband/take-up state.
4. Fit insertion and bending jointly for episodes 13--20. Retain direction,
   last reversal, accumulated motion since reversal, and dwell time as state.
5. Populate a small, quality-ranked history stack of finite-excitation windows
   rather than allowing every camera frame to update RLS.
6. Validate offline by leaving out entire episodes or speeds, not random
   frames from the same smooth trajectory.
7. Then run one short, interior-workspace, low-amplitude hardware validation
   using the causal UKF. It need not re-identify the whole model; it only must
   verify the deadband scale, response covariance, and sign-conditioned
   response under current conditions.

## Control architecture suggested by the data and literature

A memoryless adaptive Jacobian alone is structurally mismatched to the
observed system. A practical near-term architecture is:

```text
motor command
  -> per-axis reversal/take-up state (deadband/backlash memory)
  -> effective transmitted shaft increment
  -> direction-conditioned local Jacobian
  -> interface increment
  -> frozen distal model
```

MPPI should clone and propagate the take-up state in every rollout. Online
adaptation should estimate only the residual local Jacobian after this
transmission state, use accumulated motion windows, and update only from a
quality-ranked finite-excitation buffer. Preserve column norms, directions,
condition number, and the frozen-J-per-solve invariant.

This is preferable to lowering the UKF SNR threshold until arbitrary noisy
frames pass: the prior hardware audit showed that nearly half of measured
responses pointed opposite the memoryless prediction.

## Relevant primary literature

- Yip and Camarillo, *Model-Less Feedback Control of Continuum Manipulators in
  Constrained Environments* (IEEE T-RO, 2014), estimates a local Jacobian
  online. The authors explicitly identify the failure where impeded motion can
  drive a Jacobian column toward zero, and recommend conditioning constraints,
  nullspace checks, and empirical reinitialization:
  https://doi.org/10.1109/TRO.2014.2309194
- Zhai et al., *Model-Based Control of a Continuum Manipulator with Online
  Jacobian Error Compensation Using Kalman Filtering* (2025), keeps a model
  Jacobian and estimates a bounded Jacobian-error state from actuation and
  visual pose increments. This is close to the desired nominal-J plus guarded
  residual adaptation structure:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC12329213/
- Do et al., *Motion compensated controller for a tendon-sheath-driven
  flexible endoscopic robot* (2016), models asymmetric tendon-sheath backlash
  and uses real-time feedforward compensation, reporting a 74% tracking-error
  reduction:
  https://pubmed.ncbi.nlm.nih.gov/27045665/
- Do et al., *Nonlinear friction modelling and compensation control of
  hysteresis phenomena for a pair of tendon-sheath actuated surgical robots*
  (2015), develops explicit friction/backlash hysteresis models and inverse
  feedforward compensation:
  https://doi.org/10.1016/j.ymssp.2015.01.001
- Kato et al., *Tendon-driven continuum robot for neuroendoscopy: validation
  of extended kinematic mapping for hysteresis operation* (2016), identifies
  tendon friction as the dominant hysteresis source and extends the kinematic
  map accordingly:
  https://doi.org/10.1007/s11548-015-1310-2
- Zabiri and Samyudia, *A hybrid formulation and design of model predictive
  control for systems under actuator saturation and backlash* (2006), embeds
  backlash as hybrid constraints inside MPC. It is conceptually relevant if
  take-up modes are represented explicitly in MPPI rather than inverted
  outside the planner:
  https://doi.org/10.1016/j.jprocont.2006.01.003
- Parikh, Kamalapurkar, and Dixon, *Integral concurrent learning: Adaptive
  control with parameter convergence using finite excitation* (2019), shows
  how recorded, finite-excitation integral windows can support adaptation
  without continual persistent excitation and with better noise robustness:
  https://doi.org/10.1002/acs.2945
- Deng et al., *Koopman-Operator-Based Data-Driven Hysteresis Compensation for
  Tendon-Sheath-Driven Surgical Continuum Robots* (IEEE T-MRB, 2026), combines
  learned static/dynamic hysteresis compensation with feedback control. It is
  a higher-complexity alternative if the compact take-up model proves
  inadequate:
  https://doi.org/10.1109/TMRB.2025.3644036

## Bottom line

The existing session is enough to avoid immediately collecting a large new
identification dataset. It supports offline identification of insertion,
direction-dependent backlash, bending memory, response floors, and a finite
excitation buffer. It is not enough to directly fit all three columns of the
deployed 6-D material-frame Jacobian or calibrate the causal UKF confidence
gate. A short causal validation remains necessary after the offline model is
constructed.
