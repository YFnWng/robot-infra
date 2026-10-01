# Hysteresis-aware proximal control and causal experiment plan

Date: 2026-09-13

## Decision

Keep the frozen v171 distal mechanics model and the UKF marker correction, but
replace the single memoryless, freely updated proximal Jacobian with a
structured motor-transmission model:

1. exact firmware logical-to-motor coupling and RPM quantization;
2. measured raw encoder shafts as the authoritative realized motor state;
3. a small direction/reversal-dependent take-up state for each physical motor;
4. a body-frame interface Jacobian applied to the effective transmitted motor
   increments;
5. bounded, column-selective residual adaptation around an offline prior.

Do not lower the adaptation gates merely to make RLS update. Motion that is
absorbed by backlash should update the take-up state, not drive a Jacobian
column toward zero.

The first implementation should remain deliberately small. It does not need a
free neural proximal model, a free virtual base, or online adaptation of the
v171 distal mechanics.

## Evidence inspected

The analysis used:

- v171 checkpoint
  `cr_meta_lnn/checkpoints/real_distal_first_order_v171_multistep_map_em.pt`;
- its source trajectory
  `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260829_163456/processed_cosserat_dual_zed_rigid_tip_quotient_map_v18_gamma20/cosserat_states.h5`;
- the posterior reconstruction implemented by
  `_SplineDistalLatent.values()` in
  `scripts/overlay_real_distal_first_order_standalone_em.py`;
- the v174 raw-shaft Jacobian artifact
  `cr_meta_lnn/evaluation/real_joint_local_distal_v174.json`;
- the adaptive hardware audit
  `audits/ros-realtime/session-20260912-210822-adaptive-circle-hardware-audit.md`.

The checkpoint contains 11,801 posterior frames from 0 to 405.6 s. It includes
the insertion, rotation, bend-rate, bend-hold/relaxation, and repeated-bend
episodes, ending immediately before `bend_at_mid_insertion`. The later coupled
interaction and persistent-excitation episodes in the HDF5 are not represented
by this checkpoint posterior.

The posterior is not the quotient pose stored directly in the HDF5. It applies
the optimized `pose_control` and `roll_control` splines to `reference_pose`, so
it reconstructs a full material interface pose, including inferred roll. This
makes the rotation episodes usable for retrospective six-dimensional
interface-response identification.

It remains an offline, model-conditioned MAP trajectory. Pose, strain, and
roll controls use 0.5 s cubic-spline knots and can use evidence from both sides
of a frame. It must not be replayed as a causal online measurement or used to
claim sub-0.5 s backlash timing.

## What the identification data establish

Local increments below use the deployed convention

\[
\Delta\xi_k=\operatorname{Log}(g_{0,k}^{-1}g_{0,k+1}),
\qquad g_{0,k+1}=g_{0,k}\operatorname{Exp}(\Delta\xi_k),
\]

with unwrapped raw motor-output shaft radians
`a = [insertion, rotation, bending]` and approximately 0.25 s windows.

### Pure insertion and rotation are identifiable

The insertion episodes move only raw shaft 0, and the rotation episodes move
only raw shaft 1. Across the three speeds there are 206 nonzero insertion
windows and 592 nonzero rotation windows in the posterior prefix.

A simple pooled scalar-column fit gives:

\[
J_s \approx
\begin{bmatrix}
 2.18\!\times10^{-4}&-5.52\!\times10^{-5}&4.74\!\times10^{-5}&
-1.48\!\times10^{-5}&6.71\!\times10^{-5}&4.07\!\times10^{-4}
\end{bmatrix}^{T},
\]

\[
J_r \approx
\begin{bmatrix}
-5.17\!\times10^{-4}&8.87\!\times10^{-3}&5.22\!\times10^{-2}&
 6.20\!\times10^{-5}&-9.32\!\times10^{-7}&1.88\!\times10^{-6}
\end{bmatrix}^{T}.
\]

The insertion translation gain is stable by direction: approximately
0.409 versus 0.416 mm per shaft radian. The inferred rotation gain is more
rate-dependent: its angular-column norm rises from about 0.033 at the slow
rate to 0.058 rad/rad at the fast rate. This favors an explicit torsional
lag/take-up state over two unrelated static gains.

### A nominal bend-only command is not a raw single-axis experiment

The catheter is attached to the translating push-pull knob. Firmware therefore
commands the insertion motor in opposition to bend motion so that logical
catheter insertion should remain constant. In raw encoder coordinates the bend
episodes excite shafts 0 and 2 together.

For ideal coupling, the recorded hardware constants predict

\[
\Delta a_0/\Delta a_2 \approx 0.3537.
\]

The observed median is approximately 0.377 across the bend windows, with a
strong correlation of 0.989. The two-axis bend design has a Gram condition
number of about 409. An unconstrained fit cannot reliably decide how much of
the response belongs to `J_s` and how much belongs to `J_b`.

The mismatch is physically significant even though it is small per camera
window. Derived logical-insertion residuals have about 0.12--0.17 mm p95
magnitude over 0.25 s, but accumulate to roughly 0.7--2.3 mm in several slow
bend/loop episodes. This is consistent with unequal motor timing,
quantization, and direction-dependent lost motion.

After first fitting `J_s` from insertion-only data, subtracting
`J_s * delta_a0` from the bend response makes a sequential estimate of the
bend shaft column possible:

\[
J_b \approx
\begin{bmatrix}
-3.67\!\times10^{-3}&2.28\!\times10^{-4}&-3.39\!\times10^{-4}&
 1.17\!\times10^{-5}&-1.35\!\times10^{-5}&-1.40\!\times10^{-4}
\end{bmatrix}^{T}.
\]

This estimate is evidence for initialization, not a production replacement
for v174 without held-out validation. Positive and negative bend fits differ
materially, especially in angular response, and the 0.5 s posterior spline
blurs the onset of lost motion after reversal.

### Sufficiency conclusion

The existing data are sufficient to:

- fit and cross-check the insertion and rotation prior columns;
- estimate the bend column sequentially after fixing the insertion column;
- estimate slow direction/rate effects and initial reversal-travel ranges;
- reject an unconstrained simultaneous three-column RLS design.

They are not sufficient to:

- measure sub-0.5 s synchronization and backlash timing;
- separate bend and insertion columns from bend episodes alone;
- calibrate the causal UKF response covariance and SNR gates;
- validate the present mounting/history after power cycles;
- prove performance for the small, reversing commands generated by MPPI.

## Proposed controller state

One timestamped state must own:

- current six-axis logical joint position from `POS`;
- current three raw, unwrapped motor shaft angles from `ENC`;
- filtered realized shaft rates and recent command-to-encoder residuals;
- for each physical motor, last direction, reversal shaft position,
  accumulated take-up travel, effective transmitted coordinate, and lag state;
- interface material pose `g0`, distal strain `q`, UKF covariance, and v171
  tendon-history state;
- fixed prior columns `J_prior[:, i]` and bounded residual columns
  `delta_J[:, i]`;
- one RLS covariance and finite-excitation history stack per column and
  direction, rather than one covariance shared by an unidentifiable mixed
  update;
- timestamps and quality metadata for every state used in an adaptation
  window.

The initial live `g0` and material roll continue to come from the UKF and the
registered marker observation. The offline v171 posterior is for fitting and
validation only.

## Proposed forward transition

### 1. Command and actuator realization

MPPI samples logical catheter velocity

\[
u=[u_{lin}\;u_{rot}\;u_{bend}]^T.
\]

Every sample must pass through the existing manager limit projection, firmware
coupling, and integer-RPM quantization. Candidate rollouts then propagate a
small per-motor rate/lag model. At feedback time, raw encoders replace the
predicted motor state; the prediction error updates only a bounded actuator
bias/uncertainty estimate.

Do not infer realized motor motion from logical `POS`: the raw `ENC` channels
are authoritative for the learned model.

### 2. Backlash and take-up

For each physical motor `i`, propagate a compact play/dead-zone state

\[
z_{i,k+1}=\mathcal P_i(a_{i,k+1},z_{i,k},d_{i,k};\rho_i^+,\rho_i^-),
\]

where `a_i` is measured/predicted shaft angle, `z_i` is effective transmitted
motion, `d_i` is direction, and `rho_i+/-` are bounded direction-dependent
take-up distances. Start with one play element plus one first-order lag for
rotation. Add multiple play elements only if held-out causal residuals require
them.

When the shaft moves after reversal but the interface response remains below
the empirical response floor, advance the take-up state but do not update a
Jacobian column. Once repeatable response begins, update the take-up threshold
slowly from accumulated reversal travel.

### 3. Interface increment

Use the effective increments, not raw increments, in the local body Jacobian:

\[
\Delta\xi = \sum_i (J_{prior,i}+\delta J_i)\Delta z_i,
\qquad
g_0^+=g_0\operatorname{Exp}(\Delta\xi).
\]

The physical insertion caused by moving the bend knob belongs to `J_b`. The
opposing insertion-motor response belongs to `J_s`. Their cancellation is a
result of the two realized motor trajectories, not a hard-coded cancellation
inside either Jacobian column.

### 4. Distal transition

Clone and advance the complete frozen v171 transmission, tendon-memory, and
first-order distal state for every rollout. Use the predicted `g0+` as the PCS
boundary, then evaluate the four marker positions and rigid-tip position.

## Online estimation and adaptation ordering

At a marker timestamp `t_m`:

1. retrieve the most recent encoder sample at or before `t_m`;
2. rewind the model state to `t_m` and perform the UKF marker correction;
3. form a response window only between two accepted historical UKF posteriors;
4. associate the window with encoder increments and commands no later than its
   endpoint;
5. replay the corrected state to the present;
6. evaluate adaptation eligibility outside the command heartbeat callback;
7. atomically publish a new immutable model snapshot for the next MPPI solve.

No future encoder interpolation, centered smoothing, or v171 offline posterior
may enter the causal adaptation path. MPPI freezes all Jacobian, play, and lag
parameters for one solve, while cloning their dynamic states per candidate.

### Column-selective update

For an eligible dominant physical direction `i`, update only that residual
column using the partial residual

\[
r_i=\Delta\xi-\sum_{j\ne i}J_j\Delta z_j.
\]

Do not update all columns from a compensated-bend window. Such a window is
valuable for validating the combined production direction and estimating
motor synchronization error, but is nearly collinear for parameter
identification.

Ordinary mixed MPPI motion should initially be shadow-scored and stored in a
finite-excitation buffer. It may support a batch update only when the stacked
normalized action Gram matrix is well conditioned and held-out prediction
improves.

### Required gates

Retain or add all of the following:

- accepted UKF update and healthy marker diagnostics;
- timestamp, freshness, and rewind-buffer validity;
- minimum accumulated effective action and at least two moving camera
  intervals;
- response above a static-noise-derived absolute floor;
- covariance-normalized response gate calibrated from the causal experiment;
- reversal holdoff until the learned take-up state engages;
- a recognized excitation basis or a well-conditioned finite history stack;
- two consistent windows before parameter commit;
- per-column gain, angular deviation, update norm, and covariance bounds;
- held-out prequential error no worse than the current prior;
- no covariance inflation and no parameter update while static.

Use `J = J_prior + delta_J`; do not modify the sole copy of `J_prior` in place.
If an update fails a gate, continue safe control with the last accepted model
and record the reason.

## Runtime structure

```text
ENC 100 Hz ---> encoder/state history ---> actuator take-up/lag state ----+
                                                                          |
markers 30 Hz -> rewind -> UKF(g0,q,cov) -> replay ----------------------+-->
                                                                          |   immutable root state
logical target/action ----------------------------------------------------+          |
                                                                                     v
planner timer -> MPPI samples -> limits/coupling/RPM -> actuator model -> g0 -> v171 -> cost
      |                                                                              |
      +-------------------------- selected logical velocity <-------------------------+
                                      |
heartbeat publisher 100 Hz ---------> manager ---------> firmware

accepted historical UKF windows -> gated column-wise learner -> next model snapshot
```

The heartbeat publisher must never wait for UKF correction, RLS, logging, or
MPPI. The planner may miss an update and reuse a fresh prior command, but the
existing command-age watchdog remains authoritative.

## Causal hardware experiment

### Safety and starting condition

- Use a mechanically safe interior pose, with insertion near 20 mm and enough
  margin for the planned uncompensated bend test.
- Verify `MANAGER_READY`, healthy markers, valid raw encoder envelope, and
  stable UKF before every block.
- Keep MPPI command output disabled during identification. A dedicated action
  should own the autonomy source and publish a continuous heartbeat.
- Use smooth move-hold-return segments, explicit limits, an abort service, and
  automatic zero-velocity termination. Never send `SET_ZERO`.

### Command bases

Logical command vectors below are ordered
`[catheter_lin_mm_s, catheter_rot_deg_s, catheter_bend_mm_s]`.

| Block | Logical direction | Nominal raw-motor purpose | What it identifies |
|---|---:|---|---|
| Static | `[0, 0, 0]` | no motor motion | UKF drift/noise and false-update rate |
| S | `[v, 0, 0]` | shaft 0 only | insertion take-up, lag, and `J_s` |
| R | `[0, v, 0]` | shaft 1 only | torsional take-up/lag and `J_r` |
| B | `[v, 0, v]` | shaft 2 only after firmware coupling | physical knob translation/bending and `J_b` |
| C | `[0, 0, v]` | shafts 0 and 2, production compensation | synchronization residual and combined response |

Block B deliberately allows logical insertion to change with knob motion so
that the bend motor can be isolated. Validate the complete waypoint against
both insertion and bend limits before enabling it.

### Sequence

1. **Static baseline:** 15 s at the starting pose, followed by 15 s after the
   other blocks. Estimate distributions of UKF pose increments, NIS,
   covariance, and apparent response SNR.
2. **Insertion:** center -> +6 mm -> center -> -6 mm -> center where limits
   permit. Use two reliable speeds and three repeats, with 2 s dwell at each
   endpoint. Reject a resolved amplitude below 5 mm.
3. **Rotation:** center -> +75 deg -> center -> -75 deg -> center. Use two
   reliable speeds and three repeats. Reject a resolved amplitude below 65
   deg and add a 4 s post-stop dwell to expose torsional relaxation.
4. **Isolated bend motor:** establish an interior bend bias, then execute
   approximately +/-5.5 mm knob excursions about a 7.5 mm interior bias with
   `u_lin = u_bend`. Use two reliable speeds and three repeats, reject a
   resolved amplitude below 4.75 mm, and return to the same bias between
   trials.
5. **Compensated bend:** repeat the same bend excursions with `u_lin = 0`.
   This measures how well the two physical motors cancel insertion and how the
   residual changes with direction and speed.
6. **Validation only:** run a short sequence of mixed directions not used for
   fitting. Predict every window before any update from that window.

The exact amplitudes must be reduced automatically if the measured start pose
cannot retain the configured interior margins. A rejected preflight must not
clip the experiment into a different design silently.

### Causal sampling contract

- Record raw encoder and command data at their native rate and marker/UKF data
  at every accepted camera timestamp.
- Build 0.25, 0.5, and 1.0 s accumulated windows using only past samples.
- Exclude windows crossing episode boundaries, stalls, retries, manager mode
  transitions, dropped-marker intervals, or estimator reinitialization.
- Estimate each direction from complete repetitions and validate on held-out
  repetitions/speeds. Never randomly split neighboring frames from one smooth
  move between train and validation.
- Estimate reversal take-up from travel between a commanded/encoder direction
  change and the first repeatable interface response above the static floor.

## Recording requirements

The bag and a compact analysis trace must contain, with source timestamps:

- target waypoint and trajectory phase;
- requested logical velocity, manager-projected logical velocity, coupled
  motor request, integer RPM, and predicted raw shaft rate;
- raw encoder counts, unwrapped shaft radians, logical `POS`, and all freshness
  ages;
- marker positions, marker diagnostics, UKF posterior `g0` and `q`, innovation,
  NIS, covariance diagonal/trace, rewind depth, and correction timestamp;
- actuator direction, reversal position, accumulated take-up, effective
  coordinate, lag state, and command-to-encoder residual for all three motors;
- `J_prior`, every residual column, per-column RLS covariance, selected column,
  action Gram condition number, candidate response, predicted response,
  adaptation weight/update, and rejection reason;
- predicted and measured marker/tip/interface displacement over the same
  causal response window;
- planner phase timings, heartbeat lateness, command age, manager status, and
  firmware faults.

Without the effective-coordinate and selected-column fields, a later audit
cannot distinguish backlash take-up from a failed Jacobian update.

## Acceptance gates

The initial thresholds below are experiment criteria, not hard-coded final
constants:

1. No parameter commits during either static block, and fewer than 1% false
   motion-window candidates.
2. Raw command-to-encoder timing and gain are repeatable by direction; any
   outlying motor cycle is rejected rather than absorbed into `J`.
3. Each isolated column has finite held-out excitation and a stable response
   direction across at least two repetitions. Report direction cosine, gain,
   and full six-dimensional residual by sign and speed.
4. The sign/lag/play model must outperform a memoryless fixed-J baseline on
   held-out reversal windows at all three causal horizons.
5. In compensated bend, p95 0.25 s logical-insertion residual should remain
   below 0.25 mm and full-cycle drift below 0.5 mm, or the compensation model
   must explicitly predict the larger observed residual with uncertainty.
6. In shadow closed loop, median predicted/measured displacement ratio must
   enter `[0.5, 2.0]` and median response direction cosine must exceed 0.7
   before adaptation is allowed to affect MPPI. The previous hardware session
   measured about 13.1 and 0.098 respectively.
7. No manager inhibit, hard-limit event, command-stale event, or planner
   deadline fault is acceptable.

Only after these gates pass should active adaptive MPPI repeat an interior
single target, then a short axis-decomposed target sequence, and finally a
trajectory.

## Implementation work packages

### WP1: reproducible offline posterior analysis

- Add a read-only tool that reconstructs v171 `g0(t)` from the checkpoint.
- Export episode-aligned causal-window candidates and the metrics summarized
  above.
- Correctly distinguish HDF5 quotient pose from checkpoint posterior material
  pose.
- Compare v174, structured pooled, sign-conditioned, and play/lag models with
  leave-one-episode-out validation.

### WP2: actuator-memory state

- Add cloneable per-motor play/lag state to the deployment runtime.
- Correct it from raw encoders and propagate it through batched MPPI rollouts.
- Keep exact existing hardware limit/coupling/RPM projection ahead of it.
- Add unit tests for reversal, static input, state cloning, and compensated
  bend cancellation.

### WP3: structured adaptation

- Represent the model as immutable priors plus residual columns.
- Replace global dominant-axis RLS with column-selective partial-residual
  updates and per-column covariance/history.
- Treat the production compensated-bend direction as validation unless a
  history stack makes the raw design identifiable.
- Add prequential shadow scoring and automatic rollback to the prior.

### WP4: causal identification action and logging

- **Implemented:** a guarded one-shot experiment runner with the five blocks,
  preflight bounds, continuous heartbeat, operator abort, exact raw-shaft-2
  isolation, causal command/UKF traces, hardware/simulation launch routing,
  and a session summary generator. It reuses the proven collection lifecycle
  instead of adding a second motion-owning action-server implementation.
- **Pending execution:** run first against the model-in-loop simulator with
  injected asymmetry, lag, and backlash, then collect the guarded hardware
  session.

### WP5: staged hardware release

- Collect one causal identification session.
- Fit thresholds and priors offline; rerun in shadow mode.
- Enable online residual commits only after the shadow acceptance gates pass.
- Re-test single targets before any long trajectory.

## Explicit non-goals

- Do not change firmware encoder zero or send `SET_ZERO`.
- Do not adapt v171 distal mechanics during this experiment.
- Do not identify all Jacobian columns from compensated bend motion.
- Do not use the offline posterior as live ground truth.
- Do not let estimator correction, adaptation, or logging share the command
  heartbeat execution path.

## Relationship to prior work

The controller choice is consistent with the literature review in
`audits/ros-realtime/20260829-identification-excitation-and-hysteresis-review.md`:
retain a conditioned model prior, represent backlash explicitly, and adapt
from finite-excitation integral windows rather than every camera frame. This
plan specializes those principles to the actual firmware coupling and the
full v171 material-pose posterior.
