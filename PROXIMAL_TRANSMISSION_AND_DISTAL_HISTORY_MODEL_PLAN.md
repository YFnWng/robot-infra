# Proximal transmission and distal-history model modification plan

Date: 2026-09-19

Status: offline transmission boundary and A/B/C replay implemented; production
integration remains gated. No hardware or controller-deployment change is
authorized by this plan.

Implementation update (2026-09-20): Work-order step 1 is implemented by
`audits/model-validation/identify_phase2_transmission.py`. Its first report is
`audits/ros-realtime/session-20260919-190828-phase2-transmission-identification.md`.
The diagnostic confirms a stable total cascaded bend reversal gap but also
confirms that Phase 2 alone cannot allocate that gap uniquely between the
motor-to-knob and handle-clamp play elements. No production artifact has been
changed.

Implementation update (2026-09-20, work-order step 2):

- `cr_meta_lnn/networks/hybrid/proximal_transmission.py` now provides a
  batch-safe causal state for chassis, rotation, knob, handle-body, catheter
  insertion, and effective tendon coordinates.
- `ScalarTendonChain.step_effective()` bypasses only the legacy gross
  motor-play element; the original `step()` path and old checkpoint semantics
  remain unchanged.
- Phase 2 was exported to the ROS-independent
  `phase2_transmission_replay.npz` in the session directory.
- the offline A/B/C evaluator is
  `cr_meta_lnn/scripts/evaluate_phase2_proximal_transmission_variants.py`.

The first replay rejects a width-only deployment: the canonical explicit
front end with frozen v171 downstream parameters increased held-out median
marker RMS from 1.03 mm to 2.18 mm. This is evidence that the downstream
history must be refitted and coherently initialized after changing its input,
not evidence against the physical coordinate decomposition. The ROS/runtime
integration in Sections 5.2 and 6 remains blocked until that refit improves
episode-held-out marker prediction.

Implementation update (2026-09-20, work-order step 3): the v171 standalone-EM
trainer now has an opt-in explicit-transmission path and a new v175 checkpoint
schema. A staged wrapper first refits history with distal mechanics and the
force port frozen, then performs cautious joint EM fine-tuning. It can restore
the v171 posterior spline controls, preserves legacy training behavior by
default, and has passed history-only and joint smoke runs plus strict
checkpoint reconstruction. Full training has not yet been run; online
integration remains gated on the held-out replay criteria above.

## 1. Physical decomposition

Use calibrated, unwrapped raw motor angles

\[
a=[a_s,a_\theta,a_b],
\]

where `s` drives chassis translation, `theta` drives axial rotation, and `b`
drives the bending knob.

The required causal coordinates are distinct:

- `c`: chassis translation relative to ground, after the comparatively large
  motor-to-chassis backlash;
- `k`: knob translation relative to the chassis, after the comparatively small
  motor-to-knob backlash;
- `h`: handle-body displacement relative to the chassis inside the compliant
  clamp;
- `e = k - h`: effective relative knob/body tendon actuation;
- `x = c + k`: catheter/interface insertion relative to ground, with signs and
  gains supplied by the calibrated transmission convention;
- `r`: transmitted axial rotation, retained as a separate state.

The topology is therefore

```text
raw insertion motor a_s -> large drive play -> chassis c ----+
                                                           +--> x=c+k
raw bend motor a_b ------> small drive play -> knob k -------+     |
                                                               interface model
                                      k -> clamp stop h
                                           -> e=k-h
                                           -> v171 distal history/mechanics
```

This structure expresses three facts that the current shared take-up coordinate
cannot express:

1. chassis and knob motion both contribute to catheter insertion;
2. initial knob travel may move the handle body and interface while producing
   little effective tendon actuation;
3. effects after effective tendon engagement remain the responsibility of the
   v171 distal model.

### Directional play operators

Use stateful asymmetric play for the two motor transmissions:

\[
c_k=P_{w_c^+,w_c^-}(G_s(a_{s,k}-a_{s,0}),c_{k-1}),
\]

\[
k_k=P_{w_k^+,w_k^-}(G_b(a_{b,k}-a_{b,0}),k_{k-1}),
\]

with `w_k` expected to be much smaller than `w_c`.

Represent compliant handle-body travel as a bounded stop, equivalently a play
operator on effective tendon actuation:

\[
e_k=\max(k_k-H^+,\min(e_{k-1},k_k+H^-)),\qquad h_k=k_k-e_k.
\]

On a positive reversal, `h` follows `k` until `+H+` and `e` is nearly fixed;
after the bound is reached, further `k` advances `e`. The negative branch is
analogous. A rate-dependent clamp state should only be added if held-out data
show that the rate-independent model is insufficient.

## 2. What Phase 2 can identify

The Phase 2 command bases are suitable for the first offline fit:

| Phase | raw excitation | primary identification role |
| --- | --- | --- |
| 2A shaft 0 | chassis motor only | `G_s`, `w_c+/-`, chassis/interface column |
| 2B shaft 2 | knob motor only | `G_b`, upper bound on `w_k`, knob/interface column, total interface-to-distal play |
| 2C compensated bend | chassis and knob | validation of `x=c+k`, timing, and causal superposition |

The existing audit found an approximately immediate interface response but a
2.6--3.2 mm raw logical-bending transition before the pooled distal response.
This supports the two-stage topology. It does **not** uniquely divide that
travel into handle-clamp play and downstream tendon play.

The current v171 scalar tendon chain has a learned first motor-play width of
approximately 0.94 mm. Because it was fitted using the pre-decomposition raw
tendon coordinate, that value may already include part of the handle-clamp
motion. It must not simply be retained in series with a new 2.6--3.2 mm clamp
width; doing so would double-count play. Nor should 0.94 mm simply be
subtracted from the hinge estimate, because cascaded play and learned
relaxation are not separately identifiable by subtraction.

For initial identification, use marker observations as the measurement loss.
The online UKF interface pose and strain are useful initializations and
diagnostics, but they are posterior states produced by the current model and
must not be treated as independent ground truth.

The rejected `20260919_194147` Phase 3 timing session must not be used for this
fit: its raw command topology did not realize the requested timing conditions.
The corrected Phase 3 generator can be used after a future hardware rerun.

## 3. Canonical factorization to avoid non-identifiability

Use the following first candidate as the canonical model:

1. explicit motor-to-chassis play `P_c`;
2. explicit small motor-to-knob play `P_k`;
3. explicit handle-clamp stop/play `P_h` producing `e`;
4. the v171 static, persistent, relaxation, filtering, lead/rate, distal force
   port, first-order mechanics, and PCS kinematics driven by `e`;
5. disable the v171 chain's first gross `motor_play` element for the first fit.

This does not discard v171 hysteresis. It removes only the first play element
whose physical meaning is now assigned upstream. The persistent play bank,
asymmetric loading/unloading gains, Maxwell relaxation bank, static map,
lead-lag filtering, force mode, and distal mechanics remain.

Run a controlled ablation after the canonical fit:

- A: explicit clamp play, downstream gross motor play fixed to zero;
- B: explicit clamp play plus a strongly shrinkage-regularized residual
  downstream play;
- C: current v171 model without explicit clamp play.

Choose B over A only if episode-held-out marker likelihood improves and the
residual width is stable across direction, speed, and repetitions. Predictive
improvement alone does not make the two widths physically identifiable.

## 4. Interface model basis

The local interface Jacobian must no longer consume one virtual motor vector
whose shaft-2 component is held during tendon take-up. Its action basis should
be the transmitted physical coordinates

\[
u_I=[c,r,k].
\]

Then

\[
g_{0,k+1}=g_{0,k}\operatorname{Exp}
\left(J_I\,[\Delta c,\Delta r,\Delta k]^T\right).
\]

This preserves the observed interface response during handle-body take-up.
The derived `x=c+k` is logged explicitly and may be used for physical priors,
but retaining separate `c` and `k` Jacobian columns lets Phase 2 test whether
their local interface effects are truly identical.

Refit the initial Jacobian from Phase 2A and 2B rather than algebraically
transforming the old raw-shaft `J0`. Use 2C only as held-out coupled
validation initially. If the two translation columns are statistically
indistinguishable, a later constrained model may share their translational
direction while retaining separate transmission states.

Online adaptation must update this transmitted-coordinate Jacobian. Windows
inside `P_c` or `P_k` play have zero transmitted action and therefore cannot
train a column toward zero. Handle-clamp motion `h` must not suppress the knob
coordinate `k` used by the interface Jacobian.

## 5. `cr_meta_lnn` implementation

Learned mechanics and rollout state belong in `cr_meta_lnn`.

### 5.1 New state and transition

Add a module such as
`networks/hybrid/proximal_transmission.py` containing:

- `ProximalTransmissionParameters`;
- `ProximalTransmissionState` with `c`, `r`, `k`, `h`, `e`, directional
  branches, and previous raw motor coordinates;
- a batch-safe, clone-safe `advance(raw_motor, dt, state)` operation;
- named output fields rather than a positional three-vector.

All operations must support the leading batch dimensions used by GPU MPPI.
The transition must be deterministic and free of ROS dependencies.

### 5.2 Runtime state

Revise `deployment/v171_streaming_runtime.py` so one cloned state owns:

- raw calibrated motor angles;
- complete proximal-transmission state;
- interface drive coordinate `[c,r,k]`;
- derived catheter insertion `x` and effective tendon actuation `e`;
- interface pose, distal strain, v171 history, Jacobian/RLS state, and UKF
  covariance;
- rewind entries containing all of the above.

`_advance_motor` should perform exactly one causal sequence per substep:

```text
raw encoders
 -> proximal transmission advance
 -> delta [c,r,k] -> interface Jacobian -> g0
 -> e -> refactored v171 tendon history -> lambda
 -> first-order distal mechanics -> strain/shape
```

Initialization must not silently declare unknown play branches engaged. For
offline replay, burn in from the recorded prehistory. For live operation,
retain explicit branch uncertainty or an unknown state until observation
provides evidence. Visual initialization anchors `g0`; it does not reveal all
hidden transmission states.

### 5.3 v171 artifact migration

Create a new strict checkpoint schema rather than mutating the meaning of the
existing v171 artifact in place. The new artifact must store:

- front-end gains, directional widths, references, and clamp bounds;
- the refactored tendon-history state;
- parameter provenance and training-session identifiers;
- model/configuration hashes and schema version;
- the `[c,r,k]` Jacobian initialization and normalization scales.

Keep the v171 loader read-only and backward compatible. Add a new loader for
the new schema; do not reinterpret old `downstream[2]` as effective tendon
actuation.

### 5.4 Offline fitting

Implement an episode-aware fitting/replay script with these stages:

1. Fit `P_c` and the chassis interface column from Phase 2A.
2. Fit `P_k` and the knob interface column from Phase 2B interface motion.
3. Fit `H+/-` and refit the v171 history/mechanics from Phase 2B marker data.
4. Validate without refitting on Phase 2C.
5. Jointly fine-tune only after the staged solution passes validation, with
   priors preventing gains and widths from trading arbitrarily.

Split by complete episode/repetition, never by adjacent frames. Preserve
direction and speed strata in every validation report.

## 6. `robot-infra` integration

ROS should own scheduling, safety, message conversion, and observation
routing—not a second physical transmission model.

### 6.1 Remove the duplicate virtual-motor path

The current node calls `BacklashStateEstimator.advance_motor()` and supplies
its `effective_motor` to the learned runtime as
`interface_motor_angle_rad`. Replace this with raw encoder delivery to the new
runtime transition.

Do not delete safety behavior. Refactor the ROS backlash component into an
observer/diagnostic and take-up policy consumer of the runtime's named
transmission snapshot. There must be one authoritative transition state for
prediction, rewind, and rollout.

### 6.2 Observation updates

Accepted marker updates provide two different pieces of evidence:

- interface-pose increments inform chassis/knob transmission branches and the
  `[c,r,k]` Jacobian;
- distal bending increments inform handle-clamp engagement and distal-history
  reconciliation.

Do not use distal response to decide whether chassis/knob interface motion was
transmitted, and do not use interface translation alone to declare effective
tendon engagement.

The observation update must be causal, use the encoder sample paired to the
marker timestamp, and participate in rewind/replay. A present-time correction
must not be applied to a historical motor state.

### 6.3 Required trace fields

Record per estimator/control frame:

- raw shaft angles and increments;
- `c`, `r`, `k`, `h`, `e`, and `x`;
- positive/negative branch, remaining play, and confidence for each upstream
  state;
- interface-J predicted and measured body-twist increments;
- distal predicted and measured bending increments;
- v171 static, persistent, transient, rate, lead, and final shift terms;
- marker timestamp, paired encoder timestamp, rewind depth, and correction
  acceptance reason.

These fields are required to determine whether an error originates in drive
play, handle compliance, distal history, timing, or the interface Jacobian.

## 7. Simulator changes

The simulated plant and controller estimator must own separate transmission
states and may use deliberately different parameters.

Add plant parameters for:

- large chassis-drive play;
- small knob-drive play;
- positive/negative handle-clamp travel;
- optional speed-dependent clamp lag, disabled by default;
- the refactored downstream v171 parameters.

Controller-isolation reset may reset all hidden plant states. Hardware-faithful
reset must preserve `h`, `e`, and distal remanence when encoder position is
returned. Both modes must remain explicit.

## 8. Verification and acceptance gates

### Unit tests

- asymmetric play/stop loading, reversal, saturation, and zero-width limits;
- `x=c+k` under isolated and simultaneous motion;
- chassis-only motion changes `c/x` but not `e`;
- knob motion changes `k/x`, while `e` remains fixed until clamp travel is
  exhausted;
- clone, GPU batch, rewind, replay, and substep equivalence;
- zero downstream gross play does not erase persistent/relaxation memory;
- old v171 artifact loading remains unchanged.

### Offline Phase 2 validation

Compare the current and candidate models on held-out complete episodes using:

- marker RMS and maximum error;
- interface-pose translation/rotation error;
- distal-bending onset error after reversal;
- endpoint remanence error;
- direction and gain of isolated shaft responses;
- Phase 2C residual relative to causal superposition of 2A and 2B;
- parameter stability across speed, direction, and repetition.

An onset prediction within one paired-camera period at the median and two
periods at P95 is the desired timing target; report the actual cadence and
confidence intervals. Reject a model that improves aggregate RMS by worsening
one direction or speed regime beyond the static-noise envelope.

### Runtime regression

- estimator correction plus rewind/replay reproduces uninterrupted replay;
- MPPI candidates clone all hidden states and obey raw hardware limits;
- no duplicate take-up subtraction exists between ROS and `cr_meta_lnn`;
- stale-data, manager, command projection, watchdog, and fault-latching tests
  remain unchanged and fail closed;
- full-stack simulation records the new states and meets existing planner
  deadlines before any hardware test.

## 9. Work order

1. **Offline diagnostic now:** reconstruct Phase 2 marker and encoder streams,
   fit the two interface columns, and quantify the identifiable total
   interface-to-distal play by direction and speed.
2. **Prototype in `cr_meta_lnn`:** implement the proximal transmission state
   and refactor the v171 history input; train A/B/C ablations.
3. **Select a new artifact:** use episode-held-out marker validation and write
   a new model handoff. Do not replace the deployed v171 artifact yet.
4. **Runtime migration:** update state cloning, rewind/replay, UKF history
   reconciliation, GPU rollout, and diagnostics.
5. **ROS integration:** remove the duplicate effective-motor feed and route
   raw encoders plus accepted observations to the single authoritative model.
6. **Simulation qualification:** test plant/controller matched and mismatched
   parameters, reset modes, reversals, and grouped MPPI.
7. **Later hardware validation:** rerun corrected Phase 3, then repeat the
   rotation-disabled isolation test before enabling general tracking.

## 10. Decisions that should remain fixed

- Physical encoder zero remains a read-only calibration reference.
- Raw hardware position/velocity limits remain enforced in motor coordinates.
- Visual interface pose is not a direct observation of handle-body position.
- Phase 2 identifies a useful input-output decomposition but does not uniquely
  attribute every millimetre of play without an additional handle-body
  measurement.
- The current v171 checkpoint remains the production rollback artifact until
  the new schema passes offline and simulation gates.

## 11. Implemented v175 integration boundary (2026-09-20)

The fitted v175 artifact is now an optional interface-only override. It does
not replace or retrain v171 tendon/distal mechanics:

- raw motor 2 continues to drive the frozen v171 tendon-history branch;
- v175 physical play is converted to full shaft-space reversal travel;
- the v175 physical Jacobian is converted to an equivalent post-take-up shaft
  Jacobian and atomically replaces the v174 interface initialization;
- the ROS backlash estimator remains the sole controller-side causal state
  owner, so play is not applied twice;
- simulation truth owns an independent exact v175 play state and feeds its
  transmitted shaft coordinates to an otherwise unchanged v171 runtime;
- simulator actuator reversal backlash on proximal axes 0--2 must be zero
  whenever v175 truth is selected, and launch rejects double application.

The legacy empty-checkpoint path remains unchanged. The hardware candidate is
`catheter_control/config/v175_grouped_hardware.yaml`; it does not contain the
hardware-output interlock, which remains an explicit launch-time decision.
