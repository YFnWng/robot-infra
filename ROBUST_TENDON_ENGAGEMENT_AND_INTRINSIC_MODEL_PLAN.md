# Robust Tendon Engagement and Intrinsic Distal Model Plan

Date: 2026-09-25

## Motivation

The identification recording and the phase-2e insertion-stratified recording
show different motor-to-distal-curvature hysteresis. Between these sessions the
catheter handle was carefully remounted after a robot repair. That remount can
change motor-to-handle backlash, compliance, preload, and the encoder position
at which the knob becomes mechanically engaged. The mechanics inside the same
catheter handle should remain approximately unchanged.

The deployed v171 history model was trained from one continuous identification
recording. Its raw-motor-to-distal recurrence can therefore conflate:

1. **session-dependent upstream transmission**: clamp placement, handle
   mounting, motor-to-knob play, preload, guide-tube compliance, and the
   direction-dependent travel needed to engage the handle; and
2. **session-independent intrinsic catheter mechanics**: the engaged
   tendon-to-distal equilibrium curve, loading/relaxing branch response,
   spatial strain distribution, passive relaxation, and short-time distal
   dynamics.

Because remounting is expected in normal use, a controller that assumes a
fixed reversal width or fixed raw-motor hysteresis cannot be the production
solution. The controller must infer engagement from feedback, while the distal
model should predict the catheter response after engagement.

## Current evidence

The full-sweep comparison uses the physical tendon motor position for the
horizontal coordinate and distal strain projected on the frozen v171 bending
mode for the vertical coordinate.

| Dataset | Tendon range | Observed modal-curvature range |
| --- | ---: | ---: |
| Identification `bend_out_and_back` | 0--14.91 mm | 58.18--138.08 1/m |
| Phase 2e, 13.333 mm insertion | 0.09--14.92 mm | 68.95--132.06 1/m |
| Phase 2e, 26.667 mm insertion | 0.10--14.86 mm | 69.25--129.94 1/m |
| Phase 2e, 40 mm insertion | 0.14--14.91 mm | 67.41--127.85 1/m |

The phase-2e 0-mm visit contains a sharp reconstructed-curvature dip near the
high-tendon endpoint and must not be interpreted mechanically without checking
the Cartesian reconstruction. Across the other phase-2e insertion plateaus,
the relaxed baseline and curvature span differ from identification, but there
is not yet a strong monotonic insertion dependence.

The immediate next diagnostic is to test whether the **engaged loading and
relaxing segments overlap after allowing only session-specific horizontal
registration and preload**, rather than comparing loops at the same raw motor
position. If they collapse, the difference is predominantly upstream
transmission. If they do not, the remaining difference must be assigned to
session-dependent intrinsic state, insertion/boundary dependence,
reconstruction bias, or model structural error.

Generated comparison artifacts:

- phase2e session:
  `20260921_174829_phase2e_video/v171_external_validation/`
- plot: `v171_identification_phase2e_hysteresis.png`
- summary: `v171_identification_phase2e_hysteresis.json`
- generator:
  `/home/chen-lab/Yifan/cr_meta_lnn/scripts/plot_v171_identification_phase2e_hysteresis.py`

## Target factorization

Use a factored causal model rather than a single raw-motor recurrence:

```text
raw tendon motor position and direction
        |
        v
session-dependent engagement/transmission belief
  - disengaged / uncertain / provisional / engaged
  - direction and engagement anchor
  - remaining-takeup interval or distribution
  - effective tendon coordinate and confidence
        |
        v
shared engaged catheter model
  - loading/relaxing equilibrium branch
  - spatial distal strain distribution
  - passive relaxation and short-time dynamics
  - optional insertion/boundary conditioning if supported by data
        |
        v
predicted distal shape and Cartesian response
```

The nominal backlash model remains useful as a prior, a joint-reserve bound,
and a probe-travel safety limit. It is not treated as an exact engagement
location.

## Phase 1: branch-aligned cross-session diagnostic

### 1.1 Data selection

Use complete continuous-history replay for each recording. Episode labels may
select measurement intervals but must never reset history.

- Identification: isolated compensated-bending episodes, beginning with
  `bend_out_and_back`.
- Phase 2e: both visits at 0, 13.333, 26.667, and 40 mm insertion.
- Retain the raw Cartesian fit quality, E-step posterior uncertainty, motor
  direction, speed, dwell state, and insertion level.

### 1.2 Fit only session nuisance variables first

For each session and direction, fit a causal monotone raw-motor-to-effective-
tendon map with:

- an engagement anchor;
- a nonnegative local scale;
- optional bounded preload offset;
- no ability to change the shared curvature curve arbitrarily.

Do not fit independent free-form curves per session at this stage. That would
make the shared-mechanics hypothesis untestable.

### 1.3 Compare registered engaged branches

Report for loading and relaxing separately:

- overlapping effective-tendon support;
- rigid-pose-invariant Cartesian centerline and tip residuals;
- distance to the identification-session one-dimensional shape manifold;
- progress along that manifold versus raw motor travel;
- median, P95, and maximum residual;
- residual versus insertion, speed, dwell time, and visit;
- bootstrap confidence intervals using contiguous blocks rather than
  independent frames.

Do not use estimated interface position, interface orientation, material roll,
posterior strain, or v171 modal curvature for this gate. Rigidly align observed
Cartesian centerlines without scale so that the comparison is invariant to
interface-pose gauge while preserving distal shape and size.

Gate A passes if a shared engaged curve plus session registration explains the
reliable data within the E-step/reconstruction uncertainty and materially
reduces the cross-session residual. Failure of Gate A means the intrinsic
model needs additional conditioning or the observations contain unresolved
bias; it does not automatically justify an unconstrained session-specific
distal model.

### 1.4 Gate A implementation and result (2026-09-25)

The original latent-frame diagnostic is retained for provenance, but its
decision is superseded because estimated interface pose and material roll can
corrupt cross-session Cartesian alignment. The authoritative gauge-free
diagnostic is implemented in:

- `/home/chen-lab/Yifan/cr_meta_lnn/scripts/evaluate_cartesian_shape_manifold_gate_a.py`
- `/home/chen-lab/Yifan/cr_meta_lnn/tests/test_cartesian_shape_manifold_gate_a.py`

The identification `bend_out_and_back` episode is the fixed reference. The
phase-2e fit excludes the known unreliable 0-mm high-tendon reconstruction
from the primary gate and uses both visits at 13.333, 26.667, and 40 mm.
Observed centerlines are aligned to a generalized identification template with
a proper rigid transform and no scale. Within each monotonic motor run,
comparison begins only after three persistent frames exceed a 0.15-mm
Procrustes shape change. Loading and relaxing motor coordinates are registered
independently by fitting the aligned Cartesian points. Episode boundaries do
not reset history.

The fitted phase-2e registrations were:

| Branch | Scale | Offset |
| --- | ---: | ---: |
| Loading | 0.8503 | +1.2783 mm |
| Relaxing | 0.7000 (lower bound) | +0.9147 mm |

Gate A is **PARTIAL** with two distinct conclusions:

1. **Shared spatial distal-shape manifold: PASS.** Phase-2e distance to the
   identification shape manifold has P95 0.175 mm. After Cartesian motor
   registration, centerline-error P95 is 0.699 mm and tip-error P95 is
   1.493 mm, both inside the 1.5-mm and 2.0-mm gates.
2. **Shared raw-motor-to-manifold progression: FAIL.** Shape progress remains
   strongly monotone with motor travel (median Spearman 0.979), but the median
   progression rate is only 0.493 times identification. It varies substantially
   by insertion, visit, and direction. Registered motor support overlap is
   62.6%, below the 70% gate, and relaxing scale reaches the allowed lower
   bound.

Thus, the sessions share essentially the same spatial family of distal shapes,
while raw motor travel reaches and traverses that family differently. This is
the expected signature of session-dependent transmission/history rather than
a need for unrelated session-specific spatial mechanics.

Artifacts:

- `20260921_174829_phase2e_video/v171_external_validation/cartesian_shape_manifold_gate_a.png`
- `20260921_174829_phase2e_video/v171_external_validation/cartesian_shape_manifold_gate_a.json`

Interpretation: preserve or retrain a shared spatial distal model, but remove
raw-motor timing and progression from that model's assumed invariant input.
The next model should infer a session-specific causal effective-tendon
coordinate with direction-dependent engagement and continuous within-session
history, then drive the shared distal-shape manifold from that coordinate.

## Phase 2: engaged-only distal model and training pipeline

This section is the authoritative distal-training design as of 2026-09-26.
Gate A showed that the recordings share a one-dimensional Cartesian
distal-shape manifold, while raw motor travel traverses that manifold at
session-, direction-, and operating-point-dependent rates. The replacement
must separate observable engaged catheter response from remount-dependent
transmission history.

### 2.1 Standalone model boundary

The new pipeline must train from observations, encoder records, and generic PCS
kinematics without loading a v171 checkpoint or copying any v171 learned
parameter. In particular, it must not initialize or regularize the spatial
mode, natural strain, equilibrium map, stiffness, damping, time constant, or
latent posterior from v171.

Existing source code may be refactored into model-neutral utilities for PCS
kinematics, Cartesian likelihoods, variable-step integration, dataset I/O, and
visualization. Reusing code is not permission to load v171 artifacts or its
motor-conditioned posterior. Give the new model a separate schema, entry point,
checkpoint, and loader with a regression test that training succeeds when all
v171 checkpoints are unavailable.

Keep v171 unchanged only as an optional external benchmark. Side-by-side metrics
and videos are useful, but they are not an acceptance dependency, initializer,
teacher, prior, fallback, or runtime component of the new pipeline.

### 2.2 Strict ownership boundary

The transmission/engagement belief owns:

- raw encoder history and direction;
- engagement phase, reversal start, direction, and response anchor;
- a direction- and operating-point-dependent take-up distribution;
- remaining take-up interval and confidence;
- recording/remount-specific engaged gain and offset;
- effective transmitted tendon coordinate and its uncertainty;
- all persistent state used to infer when raw knob motion becomes useful.

The engaged distal model owns:

- the physical distal PCS strain state;
- the shared one-dimensional equilibrium-shape manifold;
- separate engaged loading and relaxing maps onto that manifold;
- passive first-order motion toward the selected equilibrium;
- Cartesian kinematics from interface pose and distal strain.

The engaged distal model receives no raw motor position and contains no motor
play, take-up phase, reversal counter, persistent play bank, Maxwell bank,
filtered-equilibrium state, lead term, or learned raw-input-rate term.
Direction is an explicit confirmed belief input, not a hidden recurrent
variable. Interface pose and material roll remain E-step/UKF states, not
tendon-history states.

### 2.3 Mathematical model

For recording \(r\), let \(x^m_k\) be raw tendon-motor position, \(u_k\) the
belief-estimated effective tendon coordinate, \(d_k\in\{-1,+1\}\) confirmed
engaged direction, \(\Sigma^u_k\) effective-coordinate uncertainty,
\(\mathbf q_k\in\mathbb R^{24}\) distal PCS strain, and
\(\mathbf T_k\in SE(3)\) estimated interface pose.

While engaged, local recording-specific transmission is

\[
u_k=u_k^{\rm anchor}
+a_{r,d_k}\big(x^m_k-x_{r,d_k}^{\rm eng}\big),
\qquad a_{r,d}>0.
\]

The anchor is registered from observed distal state when engagement is
confirmed. Gain \(a_{r,d}\) is a belief nuisance parameter with a strong
population prior, not a shared distal parameter.

The shared engaged model maps effective coordinate to scalar manifold progress:

\[
z_k^\star=g_{d_k}(u_k;\theta_g).
\]

The loading and relaxing maps are separate monotone splines. One suitable
parameterization is

\[
g_d(u)=b_d+\int_{u_0}^{u}\operatorname{softplus}
\big(s_d(\xi)\big)\,d\xi .
\]

This permits different engaged slopes by direction without a hidden hysteresis
state. Equilibrium distal strain lies on one shared spatial manifold:

\[
\mathbf q_k^\star=\mathcal Q(z_k^\star;\theta_Q).
\]

The first implementation learns a linear manifold from observation-only
Cartesian/PCS initialization:

\[
\mathcal Q(z)=\mathbf q_{\rm natural}+\mathbf Bz,
\qquad \|\mathbf B\|_2=1.
\]

Initialize the Cartesian curve family by rigidly aligned functional PCA, then
fit \(\mathbf q_{\rm natural}\) and \(\mathbf B\) through the standalone PCS
observation model. Neither quantity is copied from v171.

Use zero torsional entries in \(\mathbf B\). A smooth low-rank residual is
allowed only if held-out Cartesian errors prove it necessary:

\[
\mathcal Q(z)=\mathbf q_{\rm natural}+\mathbf Bz+\mathbf R(z).
\]

The only recurrent distal state is physical strain. Stationary-input
relaxation is represented by physical strain dynamics, not an extra
tendon-history latent. The first standalone gate uses one identifiable positive
time constant,

\[
\dot{\mathbf q}=\tau^{-1}(\mathbf q^\star-\mathbf q),
\qquad \tau=\operatorname{softplus}(\rho_\tau)+\tau_{\min}.
\]

For variable timestamps, use the exact first-order transition

\[
\mathbf q_{k+1}=\mathbf q_k+
\left(1-e^{-\Delta t_k/\tau}\right)
\left(\mathbf q^\star_{k+1}-\mathbf q_k\right).
\]

Initialize and train \(\tau\) from confirmed-engaged stop/settle responses. Add
a small number of positive modal rates only if held-out engaged dynamics require
them; never introduce independently free stiffness and damping matrices because
only their ratio is observable. Cartesian prediction remains

\[
\widehat{\mathbf P}_k
=\operatorname{PCS}(\mathbf T_k,\mathbf q_k).
\]

### 2.4 Direction- and operating-point-dependent take-up prior

One constant width per direction is only a coarse online estimate. Offline
training learns a population prior over local reversal travel. For a reversal
starting at manifold progress \(z_k^{\rm rev}\),

\[
\log w_{r,d,k}\sim
\mathcal N\!\left(
\mu_d(z_k^{\rm rev})+\delta_{r,d},
\;\sigma_d^2(z_k^{\rm rev})+\sigma_{r,d}^2
\right).
\]

Here \(\mu_d(z)\) is a smooth positive directional population mean,
\(\sigma_d(z)\) heteroscedastic uncertainty, and
\(\delta_{r,d},\sigma_{r,d}\) recording/remount nuisance terms.

Pre-reversal manifold progress is the primary covariate because absolute raw
encoder position shifts after remounting. A centered raw reversal coordinate
may be added,

\[
\mu_d(z^{\rm rev},\widetilde x^{m,\rm rev})
=\mu_d(z^{\rm rev})+h_d(\widetilde x^{m,\rm rev}),
\]

only if leave-one-session-out validation improves. Insertion is optional under
the same rule.

The prior assigns zero probability to immediate engagement after a genuine
reversal and has a strictly positive lower quantile. Online, a reversal queries
the prior at current \(z^{\rm rev}\), initializes \([w^L,w^U]\), then updates
that interval from motor travel and causal response. The belief artifact and
engaged distal checkpoint remain separate so remount calibration never
requires distal retraining.

### 2.5 Data preparation and latent inference

Build one schema for identification, phase2e, and control sessions:

- source timestamps and variable \(\Delta t_k\);
- raw motor encoders and insertion level;
- accepted Cartesian centerlines/markers with quality covariance;
- episode labels only as sampling annotations;
- continuous-recording identifiers and physical discontinuity flags.

No state resets at episode boundaries.

Inference sequence:

1. **Standalone geometry-only observation pass.** Infer interface pose,
   material roll, and distal strain from Cartesian observations, PCS kinematics,
   temporal smoothness, and observation covariance only. Raw motor position,
   engagement phase, and every legacy model posterior are forbidden inputs.
2. **Gauge-free manifold coordinate.** Project each reliable centerline onto
   the Gate-A Cartesian shape manifold after proper rigid alignment, producing
   \(z_k^{\rm obs}\) and uncertainty without relying only on fitted curvature.
3. **Causal engagement pass.** Run the belief continuously. Infer reversal,
   first response, provisional hold, confirmed engagement, \(u_k\), \(d_k\),
   \([w^L_k,w^U_k]\), and \(\Sigma^u_k\).
4. **Engaged mask.** Train the distal map only when direction is confirmed,
   observation quality passes, and engagement probability exceeds a threshold.
   TAKEUP and PROVISIONAL train belief but contribute no distal timing loss.
5. **Contiguous windows.** Dynamic distal windows stay inside one confirmed
   branch and never cross take-up or observation discontinuities.

For uncertain but usable engaged samples, marginalize distal loss over the
belief distribution of \(u_k\) with deterministic sigma points. Do not replace
the distribution with a falsely precise point label.

#### Standalone generalized-EM boundary

The pipeline still uses a generalized-EM decomposition because interface pose,
material roll, PCS strain, manifold progress, engagement anchors, and effective
input are latent. It must not reproduce v171's joint motor-conditioned EM.

At outer iteration \(j\):

1. **Geometry E-step:** infer
   \(p(\mathbf T,\mathbf q,z\mid\mathbf P)\) using Cartesian observations,
   generic PCS kinematics, observation covariance, and temporal smoothness.
   Motor encoders, branch direction, and engagement state are excluded.
2. **Belief E-step:** infer
   \(p(\mathcal B,u,d\mid x^m,z,\mathbf P)\) causally with geometry samples
   fixed and distal parameters detached. A response cannot be assigned before
   observation evidence.
3. **Engaged-model M-step:** optimize \(\mathcal Q,g_+,g_-,\tau\) only on
   confirmed-engaged samples while belief posteriors are fixed.
4. **Belief/calibration M-step:** optimize reversal-travel and session
   transmission distributions while engaged-model parameters are fixed.

This is constrained block-coordinate generalized EM, not end-to-end joint
training. Do not differentiate Cartesian distal loss through engagement onset
or through the geometry E-step. If the observation-only posterior is already
stable, it may be computed once and the remaining stages trained conditionally;
repeat the geometry E-step only when held-out Cartesian likelihood improves
without changing causal onset labels.

### 2.6 Identifiability and gauge constraints

Belief gain and branch-map slope are multiplicatively ambiguous. Fix the gauge:

1. use identification loading as reference with \(a_{r_0,+}=1\) and zero
   effective-coordinate offset;
2. normalize \(\mathbf B\) and fix its sign;
3. pin one branch-map value and local slope at a central engaged point;
4. regularize other recording gains toward one and offsets toward zero;
5. enforce engagement-anchor continuity,

\[
g_{d_{\rm new}}(u_k^{\rm anchor})\approx z_k^{\rm obs},
\]

   so confirmed branch changes do not create an artificial equilibrium jump;
6. never optimize belief calibration and shared branch slope freely in the
   same unconstrained step.

These constraints let branch differences represent shared engaged mechanics
while recording calibration represents remount-dependent transmission.

### 2.7 Training stages

#### Stage A: observation-only initialization

Rigidly align reliable Cartesian centerlines and fit a shared one-dimensional
functional-PCA curve family. Initialize standalone PCS natural strain and mode
by reconstructing those curves. No motor signal or legacy checkpoint enters
this stage.

#### Stage B: fit belief labels with distal parameters fixed

Fit per-recording anchors, local widths, effective gain/offset, and the
population width prior from causal response events. Geometry-derived manifold
progress may be used as an observed covariate, but distal prediction must not be
used to move response onset earlier. Report interval coverage and onset errors
first.

#### Stage C: fit static engaged branches and manifold

Use high-confidence low-speed or settled engaged frames. Jointly refine
\(g_+\), \(g_-\), \(\mathbf q_{\rm natural}\), and \(\mathbf B\) under gauge,
monotonicity, smoothness, and Cartesian reconstruction constraints. Add
\(\mathbf R(z)\) only if held-out Cartesian gates fail. Do not add insertion
conditioning unless improvement is consistent across held-out visits.

#### Stage D: fit engaged physical dynamics

Initialize and train \(\tau\) only on contiguous confirmed-engaged windows
with real \(\Delta t_k\), emphasizing stop/settle responses. Start each window
from posterior \(\mathbf q\) and propagate physical strain. Use short-to-medium
TBPTT sufficient for the strain time constant; do not backpropagate distal loss
through take-up. Reversal-centered long history belongs to belief training.

#### Stage E: constrained alternation

Alternate with stop-gradient between blocks:

1. belief/calibration update with distal mechanics frozen;
2. distal update with belief posterior frozen;
3. geometry E-step with the belief and distal model blocks frozen.

Accept an alternation only if it improves an entire held-out recording or
visit, not random windows from the same loop.

#### Stage F: uninterrupted validation

Roll each recording continuously from one initial checkpoint. Belief evolves
through take-up. During TAKEUP or PROVISIONAL, hold the last confirmed distal
equilibrium input while continuing passive physical-state propagation with the
real $\Delta t_k$ and accepting marker corrections. Only confirmed engagement
may change the effective input or select a new engaged branch. Episode labels
stratify metrics but never reset state.

### 2.8 Training objectives

Let \(p_k^{\rm eng}\) be engagement probability, \(\mathbf R_k\) observation
covariance, and \(\mathbf J^u_k\Sigma^u_k\mathbf J_k^{u\mathsf T}\) propagated
effective-coordinate covariance. Use weight

\[
\omega_k=
\frac{p_k^{\rm eng}}
{\operatorname{tr}(\mathbf R_k+
\mathbf J^u_k\Sigma^u_k\mathbf J_k^{u\mathsf T})+\epsilon}.
\]

The distal objective is

\[
\begin{aligned}
\mathcal L_{\rm distal}={}&
\lambda_{\rm center}\mathcal L_{\rm centerline}
+\lambda_{\rm tip}\mathcal L_{\rm tip}
+\lambda_{\rm tail}\mathcal L_{\rm Cartesian\ tail}\\
&+\lambda_{\rm dyn}\mathcal L_{\rm dynamics}
+\lambda_{\rm eq}\mathcal L_{\rm settled}
+\lambda_{\rm mono}\mathcal L_{\rm monotone}\\
&+\lambda_{\rm spatial}\mathcal L_{\rm manifold\ regularity}
+\lambda_{\rm branch}\mathcal L_{\rm branch\ regularity}
+\lambda_{\rm anchor}\mathcal L_{\rm anchor\ continuity}.
\end{aligned}
\]

Fit robust Cartesian centerline residuals. The tail term is CVaR or a smooth
maximum over framewise tip and point errors so rare failures are not hidden by
MSE. Settled loss anchors equilibrium branches; dynamics loss uses engaged
transitions only. Select checkpoints using Cartesian P95 and maximum errors as
well as median/RMS.

Belief training is separate:

\[
\mathcal L_{\rm belief}=
-\log p(\text{response onset},w,d\mid\mathcal B)
+\lambda_{\rm cover}\mathcal L_{\rm interval\ coverage}
+\lambda_{\rm false}\mathcal L_{\rm false\ engagement}
+\lambda_{\rm cal}\mathcal L_{\rm session\ calibration}.
\]

Never apply distal Cartesian timing loss on TAKEUP frames; otherwise shared
engaged mechanics will relearn premature motion.

### 2.9 Split strategy and acceptance gates

Use hierarchical held-out splits:

- leave one complete recording/session out;
- leave one phase2e visit per insertion level out;
- hold out contiguous reversal-to-settle events;
- retain a no-rotation control recording for external validation.

Report loading and relaxing separately and jointly.

The engaged distal model passes only if:

- high-confidence engaged centerline, tip, and maximum-point errors pass
  absolute, predeclared Cartesian thresholds on identification;
- P95 and maximum engaged errors pass the corresponding absolute thresholds on
  phase2e and no-rotation control;
- held-out loading and relaxing slopes and equilibrium shapes are correct;
- coherent rollout is stable with variable timestamps;
- replacing session belief calibration leaves the distal checkpoint unchanged;
- no distal input change occurs in TAKEUP or PROVISIONAL;
- engagement handoff preserves strain and equilibrium continuity.

The belief prior passes only if:

- directional interval coverage is calibrated on held-out response events;
- first-response travel bias is small relative to interval width;
- false immediate engagement is zero for genuine reversals;
- uncertainty expands rather than collapses under session shift;
- position conditioning improves held-out onset likelihood and error over
  constant directional widths.

### 2.10 Artifact and runtime contract

Create two independently versioned artifacts.

**Engaged distal checkpoint**

- shared \(\mathcal Q,g_+,g_-,\tau\), or a later validated small positive
  modal-rate set;
- geometry/gauge metadata, source hashes, and held-out metrics;
- no raw motor reference, backlash width, play bank, or session calibration.

**Engagement-belief prior**

- direction/operating-point width mean and uncertainty functions;
- population engaged-gain/offset priors;
- calibration bounds and positive minimum reversal travel;
- response thresholds and provenance.

Deployment exposes an interface equivalent to:

    step_engaged(distal_state, effective_tendon, engaged_direction, dt)
        -> next_distal_state, predicted_shape

It rejects unconfirmed direction. During TAKEUP and PROVISIONAL, MPPI and
candidate engaged rollout are inactive. The estimator still propagates distal
strain toward the last confirmed equilibrium and applies marker corrections;
it does not invent a new effective-input increment. At confirmation:

1. reconcile strain from latest marker-corrected UKF posterior;
2. register branch anchor and effective coordinate;
3. verify branch-map anchor continuity;
4. discard the pre-engagement plan;
5. replan from the immutable belief snapshot.

### 2.11 Planned code reuse and additions

Refactor model-neutral code into new modules rather than importing a v171
loader or checkpoint:

- PCS kinematics and Cartesian observation likelihood;
- rigid curve alignment and functional-PCA initialization;
- variable-step integration and event-window utilities;
- dataset adapters, coherent rollout metrics, and overlay rendering.

Add as independent entry points that do not import any legacy model loader:

- networks/hybrid/engaged_distal.py;
- networks/hybrid/position_dependent_engagement.py;
- scripts/prepare_engaged_distal_dataset.py;
- scripts/infer_engagement_belief.py;
- scripts/train_real_engaged_distal.py;
- scripts/evaluate_real_engaged_distal.py;
- a strict artifact-only deployment loader;
- tests for monotonic branches, variable-\(\Delta t\), interval propagation,
  gauge constraints, no response before engagement, continuous history, and
  successful training with all legacy checkpoints absent.

The first implementation gate is deliberately minimal and standalone: a
linear learned shared manifold, two monotone engaged branch maps, one learned
positive physical time constant, and a position-dependent directional gap
prior. Add nonlinear spatial structure, multiple rates, or insertion
conditioning only after this baseline passes held-out sessions.

## Phase 3: online engagement belief

Replace exact-width tendon compensation with a per-direction belief state:

```text
ENGAGED
  -> reversal requested
UNCERTAIN_TAKEUP
  -> bounded low-speed probe
PROVISIONALLY_ENGAGED
  -> persistent response confirmation
ENGAGED_NEW_DIRECTION
```

### 3.1 Full engagement-belief state

For physical shafts
$i\in\{0:\text{chassis},1:\text{rotation},2:\text{knob/tendon}\}$ and
direction $s\in\{-,+\}$, the complete source-time belief is

$$
\mathcal B_k=(\mathbf b_{0,k},\mathbf b_{1,k},\mathbf b_{2,k},
\mathbf o_k,\mathcal H_k),
$$

$$
\begin{aligned}
\mathbf b_{i,k}=\big(&\phi_i,d_i^m,d_i^e,d_i^c,
\bar w_i^+,[w_i^{+,L},w_i^{+,U}],
\bar w_i^-,[w_i^{-,L},w_i^{-,U}],\\
&\bar r_i,[r_i^L,r_i^U],q_i^{\rm rev},q_i^{\rm eng},a_i,
q_i^{\rm eff},\sigma_i^{\rm eff},c_i,n_i,n_i^{\rm reject},
t_i^{\rm evidence},\\
&\widehat{\Delta q}_i^{\rm tx},e_i,\rho_i\big).
\end{aligned}
$$

Its fields are:

- phase $\phi_i\in\{\texttt{UNKNOWN},\texttt{TAKEUP},
  \texttt{PROVISIONAL},\texttt{ENGAGED},\texttt{FAILED}\}$;
- raw-motion, confirmed-engagement, and confirmation-window directions
  $d_i^m,d_i^e,d_i^c\in\{-1,0,+1\}$;
- directional full-gap means $\bar w_i^s$ and intervals
  $[w_i^{s,L},w_i^{s,U}]$;
- active remaining-gap mean $\bar r_i$ and interval $[r_i^L,r_i^U]$;
- reversal-start and response/engagement encoder anchors
  $q_i^{\rm rev},q_i^{\rm eng}$, plus accumulated take-up travel $a_i$;
- effective post-take-up motor coordinate $q_i^{\rm eff}$ and implemented
  uncertainty $\sigma_i^{\rm eff}=\tfrac12\max(0,r_i^U-r_i^L)$;
- confidence $c_i\in[0,1]$, confirmation count $n_i$, contradictory
  provisional count $n_i^{\rm reject}$, and evidence timestamp
  $t_i^{\rm evidence}$;
- inferred transmitted increment $\widehat{\Delta q}_i^{\rm tx}$,
  normalized attribution evidence $e_i$, and classification
  $\rho_i\in\{\texttt{NOT_EVALUATED},\texttt{INCONCLUSIVE},
  \texttt{CONFIRMED},\texttt{CONFIRMED_PERSISTENT},
  \texttt{CONTRADICTORY}\}$.

The shared response state is

$$
\mathbf o_k=(\epsilon_k^J,\Delta z_k^{\rm bend},
e_k^{\rm bend},\chi_k^{\rm bend}),
$$

containing joint-fit residual, marker-corrected distal bending increment,
normalized evidence, and tendon-response confirmation flag.
$\mathcal H_k$ contains prior raw encoders, pending sub-threshold increments,
prior response-window encoder/pose/distal-strain samples, reversal-observed
flags, and the response-window reset flag. Rewind/replay restores
$(\mathcal B_k,\mathcal H_k)$ from one common checkpoint.

### 3.2 Position-conditioned prior and session transmission extension

The engaged-only distal model requires the target belief state to extend the
currently implemented constant directional intervals. For each shaft, store

$$
\mathbf c_{i,k}=\left(z_i^{\rm rev},\mu_i^w,(\sigma_i^w)^2,
a_i^{\rm tx},b_i^{\rm tx},\Sigma_i^u,\nu_i^{\rm prior}\right),
$$

where $z_i^{\rm rev}$ is the pre-reversal engaged-manifold coordinate,
$(\mu_i^w,(\sigma_i^w)^2)$ is the queried local reversal-travel prior,
$a_i^{\rm tx}>0$ and $b_i^{\rm tx}$ are session transmission gain and offset,
$\Sigma_i^u$ is effective-coordinate uncertainty, and
$\nu_i^{\rm prior}$ identifies the prior/calibration version used. The full
target belief is therefore

$$
\mathcal B_k^{\rm target}=\left(\mathcal B_k,
\mathbf c_{0,k},\mathbf c_{1,k},\mathbf c_{2,k}\right).
$$

At a genuine reversal, query rather than assume a constant width:

$$
p(w_i\mid d_i,z_i^{\rm rev},\nu_i^{\rm prior})
=\operatorname{LogNormal}\!\left(
\mu_{i,d_i}(z_i^{\rm rev})+\delta_{i,d_i}^{\rm session},
\sigma_{i,d_i}^2(z_i^{\rm rev})+
(\sigma_{i,d_i}^{\rm session})^2\right).
$$

Initialize $[r_i^L,r_i^U]$ from conservative calibrated quantiles of this
distribution, subject to a strictly positive minimum travel. Immediate
engagement is not a hypothesis. Once response is confirmed, register the raw
anchor and map subsequent engaged travel into the distal coordinate:

$$
u_{i,k}=u_i^{\rm anchor}
+a_i^{\rm tx}\big(q_{i,k}^{\rm raw}-q_i^{\rm eng}\big),
\qquad
\Sigma_{i,k}^u=(a_i^{\rm tx})^2\Sigma_{i,k}^{q,\rm eng}
+\Sigma_i^{\rm calibration}.
$$

The branch direction, anchor, gain, offset, and covariance passed to the
distal model must all come from one immutable source-time belief snapshot.
The current ROS implementation has constant per-direction intervals and an
effective coordinate; the position-conditioned query, session gain/offset,
and full covariance above are planned extensions, not existing behavior.

### 3.3 Belief propagation and response update

For encoder increment $\Delta q_{i,k}$ above the motor-increment floor, let
$d_{i,k}=\operatorname{sign}(\Delta q_{i,k})$. A new direction initializes

$$
(\bar r_i,r_i^L,r_i^U)\leftarrow
(\bar w_i^{d_i},w_i^{d_i,L},w_i^{d_i,U}),\quad
q_i^{\rm rev}\leftarrow q_{i,k-1},\quad a_i\leftarrow0.
$$

Accumulated travel is updated in TAKEUP or PROVISIONAL,

$$
a_{i,k+1}=a_{i,k}+|\Delta q_{i,k}|,
$$

but geometric remaining-gap consumption occurs only in TAKEUP:

$$
\begin{aligned}
\bar r_{i,k+1}&=\max(0,\bar r_{i,k}-|\Delta q_{i,k}|),\\
r^L_{i,k+1}&=\max(0,r^L_{i,k}-|\Delta q_{i,k}|),\\
r^U_{i,k+1}&=\max(0,r^U_{i,k}-|\Delta q_{i,k}|),\\
q^{\rm eff}_{i,k+1}&=q^{\rm eff}_{i,k}
+d_{i,k}\max(0,|\Delta q_{i,k}|-\bar r_{i,k}).
\end{aligned}
$$

PROVISIONAL is normally a zero-command confirmation hold. If unexpected raw
motion occurs in that phase, the point estimator passes it to the effective
coordinate rather than re-enabling a take-up burst.

At an accepted marker correction, corrected interface increment
$\boldsymbol\xi_k$ is attributed jointly with causal Jacobian columns
$\mathbf J_i$ and state scale $\mathbf S$:

$$
\widehat{\boldsymbol\alpha}=
\arg\min_{0\le\alpha_i\le|\Delta q_i|}
\left\|\mathbf S^{-1}\!\left[
\boldsymbol\xi_k-\sum_i\mathbf J_i
\operatorname{sign}(\Delta q_i)\alpha_i\right]\right\|_2^2
+R_{\rm sparse},
$$

$$
\widehat{\Delta q}^{\rm tx}_i=
\operatorname{sign}(\Delta q_i)\widehat\alpha_i.
$$

Column-removal degradation defines $e_i$. The tendon axis may instead be
confirmed by a causal marker-corrected distal-mode response above its
stationary threshold. First credible response sets
$\phi_i=\texttt{PROVISIONAL}$ and $q_i^{\rm eng}=q_{i,k}$ and immediately
stops probing. Persistent accepted response while held at the anchor gives

$$
\phi_i\leftarrow\texttt{ENGAGED},\quad d_i^e\leftarrow d_i,\quad
(\bar r_i,r_i^L,r_i^U)\leftarrow(0,0,0),\quad
c_i\leftarrow\min(1,c_i+0.2).
$$

Only a confirmed reversal with an interface-attributed transmitted increment
learns a directional width. With prior $w_i^{d,0}$,
gain bounds $g_{\min},g_{\max}$, and rate $\eta$,

$$
w_i^{\rm obs}=\operatorname{clip}\!\left(
a_i-|\widehat{\Delta q}^{\rm tx}_i|,
g_{\min}w_i^{d,0},g_{\max}w_i^{d,0}\right),
$$

$$
\bar w_i^d\leftarrow(1-\eta)\bar w_i^d+\eta w_i^{\rm obs}.
$$

The interval bounds receive the same exponential update toward
$w_i^{\rm obs}\mp\epsilon_w$, clipped to prior gain bounds. Reversal halves
confidence. Inconclusive evidence preserves PROVISIONAL; repeated genuinely
contradictory evidence transitions to FAILED.

For the tendon axis, marker-corrected distal curvature is the primary response
evidence. Interface motion is secondary because the exposed distal catheter
can bend with little observed interface-pose change.

A credible response must be aligned with the expected branch, exceed the
stationary noise distribution, persist across accepted marker corrections,
and occur after the corresponding motor motion. The first credible response
terminates probing immediately.

## Phase 4: MPPI and command arbitration

### 4.1 Planning coordinate

MPPI samples effective, post-engagement actuation. The command compiler maps
the selected effective action into raw motor commands using the current
engagement belief.

### 4.2 Uncertain reversal evaluation

MPPI does not evaluate an immediate-engagement hypothesis because that outcome
is not physically plausible after a direction reversal. Every rollout remains
in effective post-engagement coordinates. A reversing candidate is charged one risk-adjusted transaction duration.
The upper interval bound supplies nominal conservative travel; interval width
and confidence enlarge that travel inside the same physical time estimate.
Raw joint reserve through the upper bound remains a hard feasibility check,
not another tunable penalty.

Continue-direction candidates remain available so a marginal or uncertain
reversal can be rejected without modifying the selected plan after
optimization.

### 4.3 Execution during take-up

Take-up is an atomic transaction outside MPPI. While any commanded joint is not
observation-confirmed:

- MPPI is inactive;
- only bounded, low-speed probe commands are issued on unconfirmed axes;
- all other commanded axes are held at zero;
- the first credible response stops probing immediately;
- zero is held while accepted marker corrections confirm response persistence;
- path progress is frozen by the absence of a new executable MPPI command.

The full post-engagement action is never mixed with take-up motion.

On confirmed engagement:

1. register the encoder engagement anchor;
2. reconcile effective tendon and distal-history state;
3. discard the pre-engagement plan;
4. replan from the latest marker-corrected UKF state;
5. ramp into the new engaged command.

If no response is observed before the conservative travel or joint-reserve
bound, stop the probe and replan with that reversal unavailable. Escalate to a
fault only when safety/freshness/limit conditions require it, rather than
equating target infeasibility with a hardware fault.

### 4.4 Complete implemented MPPI objective

For rollout $n\in\{1,\ldots,N\}$ and horizon step
$h\in\{1,\ldots,H\}$, let $\mathbf u_{n,h}$ be projected logical velocity,
$\tilde{\mathbf u}_{n,h}=\mathbf u_{n,h}/\mathbf u_{\max}$,
$\mathbf p_{n,h}$ the predicted tip, and $\mathbf p_h^\star$ the reference.
The simplified scored objective is

$$
J_n=C_{\rm tip}+C_{\rm slew}+C_{\rm boundary}
+C_{\rm takeup\text{-}risk}.
$$

There is no Cartesian shape target in the deployed task. Shape, effort,
horizon-reversal, switch-count, and projection-mismatch terms are therefore
not soft costs. Internal reversal and unsafe projection remain hard
feasibility decisions.

#### Tracking costs

For a point target,

$$
D_{n,h}=10^6\|\mathbf p_{n,h}-\mathbf p_h^\star\|_2^2
\quad[\mathrm{mm}^2].
$$

For a continuous-path reference with unit tangent $\mathbf t_h$, let
$\mathbf e_{n,h}=1000(\mathbf p_{n,h}-\mathbf p_h^\star)$ and

$$
e^\parallel_{n,h}=\mathbf e_{n,h}^{\mathsf T}\mathbf t_h,\qquad
\mathbf e^\perp_{n,h}=\mathbf e_{n,h}
-e^\parallel_{n,h}\mathbf t_h,
$$

$$
D_{n,h}=\|\mathbf e^\perp_{n,h}\|_2^2+
\operatorname{ReLU}(-e^\parallel_{n,h})^2.
$$

This penalizes cross-track error and lag but not forward path progress. With
$\omega_h=1$ and $\omega_H=\lambda_{\rm terminal}$,

$$
C_{\rm tip}=\lambda_{\rm tip}\sum_{h=1}^H\omega_hD_{n,h}.
$$

No marker-shape target is supplied, so no shape-tracking term is present.

#### Command cost

The only command regularizer is slew:

$$
C_{\rm slew}=\lambda_{\rm slew}\left[
\|\tilde{\mathbf u}_{n,1}-\tilde{\mathbf u}_{\rm prev}\|_2^2+
\sum_{h=2}^H\|\tilde{\mathbf u}_{n,h}
-\tilde{\mathbf u}_{n,h-1}\|_2^2\right].
$$

Effort is not penalized because velocity magnitude is not actuator energy and
a zero-command bias can cause stalling. Internal physical-shaft reversals are
hard-ineligible, so they need no additional horizon-reversal penalty. A
first-step reversal is priced by its physical take-up transaction below.

#### Projection and physical-limit costs

Velocity and position projection remain mandatory, but projection mismatch is
not separately penalized: the rollout is evaluated using the realized
projected command. The only soft limit term is boundary reserve.

For joint $j$, let span $s_j=q_j^{\max}-q_j^{\min}$, soft margin
$m_j=\beta s_j$, and clearance

$$
\gamma_{n,h,j}=\min(q_{n,h,j}-q_j^{\min},
q_j^{\max}-q_{n,h,j}).
$$

Boundary cost penalizes only deterioration relative to the root state:

$$
C_{\rm boundary}=\lambda_{\rm boundary}
\sum_{h,j}\left[
\operatorname{ReLU}\!\left(
\frac{m_j-\gamma_{n,h,j}}{m_j}-b_{0,j}
\right)\right]^2,
\qquad
b_{0,j}=\operatorname{ReLU}\!\left(
\frac{m_j-\gamma_{0,j}}{m_j}\right).
$$

Take-up limit accounting is stronger than a soft term. For a candidate first
physical direction $d_{n,i}$, use

$$
[L_{n,i},U_{n,i}]=
\begin{cases}
[w_i^{d,L},w_i^{d,U}], & \text{new direction},\\
[r_i^L,r_i^U], & \text{active TAKEUP/PROVISIONAL},\\
[0,0], & \text{otherwise}.
\end{cases}
$$

The upper bound is mapped through physical-shaft/logical-joint coupling and
added to the rollout root before projection:

$$
\Delta\mathbf q_n^{\rm reserve}=
\mathcal C\!\left(\alpha_{\rm reserve}
\mathbf d_n\odot\mathbf U_n\right).
$$

Candidates whose reserve worsens a limit violation or whose projected motion
is infeasible receive $J_n=+\infty$. Thus the previously named
$C_{\rm limit}$ is implemented by conservative root shifting, hard
eligibility, realized-command rollout, and $C_{\rm boundary}$, not by one
additional scalar term.

#### Unified engagement-belief transaction cost

For relevant reversing or still-unconfirmed axis $i$, let $[L_{n,i},U_{n,i}]$
be its current take-up interval and $c_i$ its belief confidence. The
risk-adjusted travel is

$$
W_{n,i}^{\rm risk}=
U_{n,i}+(1-c_i)(U_{n,i}-L_{n,i}).
$$

Because active axes take up concurrently, the transaction duration is

$$
T_n^{\rm risk}=
\max_i\frac{W_{n,i}^{\rm risk}}
{|v_i^{\rm takeup}|+\varepsilon}
+\mathbf 1_{\rm transaction}t_{\rm confirm},
$$

and the only reversal-related soft cost is

$$
C_{\rm takeup\text{-}risk}
=\lambda_{\rm takeup}T_n^{\rm risk}.
$$

Interval width and confidence remain distinct belief inputs, but they no
longer have independent MPPI weights. The upper interval bound is still used
unchanged for hard joint-reserve feasibility. There is no immediate-engagement
branch and no separate overshoot term; overshoot is controlled by slow atomic
probing, confirmation hold, and discard-and-replan after response.

### 4.5 Hard gates, grouped modes, and MPPI weighting

A rollout is ineligible ($J_n=+\infty$) if it reverses internally within the
horizon, enters a blocked physical direction, has upper-bound take-up reserve
that worsens a conservative limit violation, or produces a non-finite rollout.
Ordinary velocity projection remains feasible, is rolled out using the
realized command, and is graded only by $C_{\rm boundary}$. MPPI
does not produce executable output while any transaction axis is unconfirmed;
the arbiter owns that interval and commands only slow take-up or zero hold.

For each direction-leased axis, grouped MPPI constructs

$$
M_i\in\{U_i,C_i\},
$$

where $U_i$ is unconstrained and $C_i$ permits zero or leased-direction
continuation. With $A$ active leases there are $2^A$ joint modes. The best
complete trajectory per reversal mask is recorded, and the globally
lowest-cost eligible complete trajectory is selected. A plan is never edited
axis-by-axis after scoring.

For eligible set $\mathcal E$, the sampling weights are

$$
\pi_n=
\frac{\exp[-(J_n-J_{\min})/(\lambda_T\sigma_J)]}
{\sum_{m\in\mathcal E}
\exp[-(J_m-J_{\min})/(\lambda_T\sigma_J)]},
$$

where $\sigma_J$ is eligible-cost standard deviation with a numerical floor.
The weighted feasible sequence updates the next sampling nominal. With the
deployed best-candidate guard, the executable sequence is the minimum-cost
already-evaluated complete candidate rather than an unscored convex average.

Current defaults are:

| Symbol | Configuration field | Default |
|---|---|---:|
| $H$ | horizon_steps | 4 |
| $\Delta t$ | step_s | 0.040 s |
| $N$ | samples | 32 |
| $\lambda_T$ | temperature | 1.0 |
| $\lambda_{\rm tip}$ | tip_weight | 1.0 |
| $\lambda_{\rm terminal}$ | terminal_weight | 6.0 |
| $\lambda_{\rm slew}$ | slew_weight | 0.04 |
| $\lambda_{\rm takeup}$ | takeup_risk_weight | 4.0 |
| $t_{\rm confirm}$ | takeup_confirmation_time_s | 0.10 s |
| $\lambda_{\rm boundary}$ | boundary_weight | 0.10 |
| $\beta$ | boundary_margin_fraction | 0.10 |
| $\alpha_{\rm reserve}$ | takeup_limit_reserve_scale | 1.0 |

## Phase 5: raw and effective joint-limit accounting

Maintain separate coordinates:

- raw encoder position for physical hard-limit enforcement;
- effective tendon coordinate for distal prediction.

Before accepting a reversal, reserve raw travel through the conservative
take-up bound. After observed engagement, replace the estimate with the
measured anchor and release unused reserve. MPPI projection, the manager, and
firmware remain the final limit authorities.

## Phase 6: estimator and ROS integration

Learned transmission/history mechanics and state reconciliation belong in
`cr_meta_lnn`. ROS scheduling, immutable snapshot exchange, safety gating,
command arbitration, and trace publication belong in `robot-infra`.

The engagement observer should update inside the existing 50 Hz estimator
owner so that raw feedback, marker correction, rewind/replay, and model state
share a causal timeline. Planning consumes a deep-cloned immutable snapshot.
Do not add blocking work or another owner of mutable model state.

Rewind/replay must restore and replay both the distal UKF state and the
engagement/effective-tendon state from a common checkpoint.

Extend estimator/controller traces with:

- engagement phase, direction, and confidence;
- reversal-start and confirmation encoder positions;
- accumulated probe travel and bounds;
- response evidence and threshold;
- raw versus effective tendon displacement;
- selected engagement-delay hypothesis;
- engagement-to-replan latency;
- engaged gain/offset calibration;
- predicted and observed post-engagement response.

## Phase 7: verification sequence

1. **Offline replay** of phase2e and failed no-rotation MPPI sessions.
2. **Perturbed simulation** with randomized directional take-up, preload,
   engaged gain/offset, marker noise/outliers, and timing jitter.
3. **No-rotation sparse targets** using reachable model-generated targets.
4. **Farther sparse targets** to verify bounded progress and graceful timeout.
5. **Continuous path tracking** only after stable sparse-target branch
   selection and bounded reversal overshoot.

Primary verification metrics:

- false and missed engagement confirmations;
- motor travel and time from reversal to confirmation;
- first-response-to-command-stop/replan latency;
- tip overshoot after first credible response;
- reversal count per target/path length;
- maximum and P95 Cartesian error around reversals;
- predicted-versus-measured displacement magnitude and direction;
- planner deadline misses, command age, and stale/freshness faults;
- raw joint reserve and hard-limit margin.

## Safety invariants

- Never issue or add any `SET_ZERO` path.
- Preserve manager and firmware authority, command projection, freshness
  gates, watchdogs, and fault latching.
- Do not enable hardware command output merely to validate offline logic.
- New behavior remains behind explicit parameters until offline replay,
  simulation, and reviewed hardware gates pass.
- Hardware tests require explicit authorization and begin with low-speed,
  bounded no-rotation motions.

## Recommended implementation order

Gate A is complete: the sessions share a Cartesian one-dimensional shape
manifold, but raw-motor progression is not session invariant. Continue in this
order:

1. Add a standalone geometry-only data adapter and posterior export in
   `cr_meta_lnn`; make loading any legacy checkpoint impossible in this path.
2. Implement continuous causal event labeling and fit the directional,
   position-conditioned reversal-travel prior with session nuisance terms.
3. Implement the minimal engaged-only distal model: an independently learned
   linear manifold, monotone loading/relaxing maps, and one positive time
   constant estimated from engaged data.
4. Train static engaged branches first, then run held-out-session Cartesian
   gates before unfreezing any spatial residual or dynamic scale.
5. Add the strict engaged-distal artifact schema and standalone deployment
   loader. Retain the legacy runtime only as a separately launched A/B
   comparator, never as an automatic fallback or dependency.
6. Extend the online belief snapshot with the local prior query,
   transmission gain/offset, effective-coordinate covariance, and common
   rewind/replay checkpoint.
7. Connect the confirmed effective coordinate to MPPI and the estimator while
   preserving atomic slow take-up, confirmation hold, discard, and replan.
8. Add unit, continuous-replay, perturbed-simulation, and artifact-identity
   tests, then run staged no-rotation hardware validation only after all
   software gates pass.



## Phase 2 standalone implementation status (2026-09-26)

The first standalone offline gate is implemented in `cr_meta_lnn` without any
v171 checkpoint dependency:

- `scripts/prepare_engaged_distal_dataset.py` performs observation-only rigid
  curve registration, fixes the arbitrary PCA sign against measured motor
  progression, and exports `standalone_engaged_geometry_v1`;
- `scripts/infer_engagement_belief.py` performs continuous causal engagement
  labeling and fits a directional, shape-position-dependent lognormal
  reversal-travel prior;
- `networks/hybrid/engaged_distal.py` implements the shared smooth
  one-dimensional PCS shape manifold (linear, quadratic, and cubic spatial
  modes), separate monotone engaged loading and relaxing maps, an invertible
  progress-to-input registration, and exact variable-step first-order dynamics;
- `scripts/train_real_engaged_distal.py` fits geometry, response-registered
  branches, direction/session gains, and dynamics in separate blocks. Each
  confirmed engagement run is re-anchored at its first credible response; no
  TAKEUP samples enter the distal branch or timing loss and no free session
  offset may absorb an engagement-state error;
- static branch supervision now requires a stopped-motor dwell tail and low
  observed shape rate. Insufficient equilibrium evidence fails the run instead
  of falling back to all engaged samples;
- `scripts/evaluate_real_engaged_distal.py` holds the last finite target across
  rejected observations, performs uninterrupted response-registered rollout,
  queries the take-up prior in its raw PCA coordinate, and makes confirmed-
  engaged Cartesian error the primary metric. It separately reports TAKEUP,
  provisional, equilibrium-dwell, per-recording/direction, and per-segment
  diagnostics, plus geometry / registered-equilibrium / coherent-dynamics
  decomposition. Scalar rollout progress is deliberately excluded because the
  nonlinear strain manifold has no valid linear inverse chart;
- `deployment/engaged_distal_checkpoint.py` accepts only the strict
  `engaged_distal_standalone_v2` and
  `position_dependent_engagement_prior_v1` artifact kinds and rejects an
  artifact without the standalone provenance invariant;
- `scripts/run_real_engaged_distal_pipeline.sh` is the reproducible four-stage
  entry point.

Focused unit coverage includes causal positive-travel confirmation, invalid
geometry rejection, PCA sign consistency, rigid-alignment correctness,
monotone maps, variable-time dynamics, prior intervals, and strict artifact
loading. A synthetic prepare/infer/train/evaluate run and a read-only phase2e
schema smoke test pass. The phase2e preparation retained 6,843 valid frames;
causal inference found 15 usable reversal events with 93.3% empirical 90%
interval coverage.

A correction audit of the first joint fit found that globally accumulated
effective input drifted between reversals, while a per-session affine offset
hid part of that state error. The v2 fit therefore uses alternating
response-anchor E/M updates, direction-specific multiplicative gains, and no
offset. A second defect in the evaluator queried the take-up prior with
normalized model progress although the prior was trained on raw PCA progress;
that coordinate mismatch is corrected. Focused tests pass and an intentionally
short real-data smoke run processes all 11,039 valid frames, preserves 27
physical engagement events across isolated rejected observations, and restores
the prior's 88.9% empirical interval coverage. Smoke-fit Cartesian accuracy is
not a model-quality result because it used only eight geometry epochs.

This is an offline candidate, not the deployed controller model. Production
acceptance still requires full training, held-out-session evaluation, event
error inspection, and runtime wiring of the confirmed effective coordinate.
The current one-pass observation geometry is the permitted fixed-posterior
special case of the generalized-EM plan; an iterative geometry E-step should be
added only if held-out Cartesian likelihood shows it is needed.


### Full v2 evaluation correction and geometry decision (2026-09-26)

The completed joint identification/phase2e fit is an in-sample diagnostic, not
held-out evidence. The evaluator now records this provenance explicitly and
supports `--require-held-out` so a same-archive evaluation fails rather than
being misreported as generalization.

Acceptance uses pose-invariant Cartesian curves only. The prior scalar
`progress_error` was invalid: it projected PCS strain onto the leading strain
mode even though the learned quadratic/cubic residual modes are not orthogonal
to that mode. Re-registering a predicted curve into the fixed offline
Generalized-Procrustes PCA chart is also not unique, so no replacement scalar
progress error is used as a gate.

For the completed v2 artifact, confirmed-engaged error decomposes as follows:

| component | centerline RMS p95 | tip error p95 | max-point p95 |
|---|---:|---:|---:|
| observed-progress geometry | 1.082 mm | 3.238 mm | 5.710 mm |
| registered engaged equilibrium | 1.109 mm | 3.353 mm | 5.802 mm |
| coherent first-order dynamics | 1.185 mm | 3.618 mm | 6.042 mm |

Because the first row is evaluated using the observed shape coordinate, it
removes motor-rate registration, engagement timing, and temporal rollout from
the comparison. Most of the tail error therefore lies in the learned spatial
manifold/geometry; motor registration adds about 0.12 mm and dynamics about
0.27 mm to tip-error p95. The result does not support treating the failure as
primarily a temporal mismatch.

The v171 natural strain and gauge-fixed tendon bending mode may be evaluated as
an **initialization-only ablation**, copied into a self-contained candidate and
then allowed to refit. They must not be fixed as truth or loaded at runtime:

- v171 was fit to the identification mounting and its scalar tendon history
  conflates the session-dependent transmission that the new belief owns;
- importing v171 motor play, relaxation, lead/lag, or engaged input map would
  recreate the dependency this pipeline was designed to remove;
- natural strain and the spatial bending mode can improve optimization only if
  they reduce observed-progress Cartesian geometry error on both sessions.

The required comparison is therefore scratch versus v171-geometry warm start,
with identical response-registered branch fitting and held-out Cartesian
metrics. Warm start is accepted only if it improves both directions and the
held-out session without degrading conservative engagement-interval coverage.


## Phase 3--6 implementation status (2026-09-26)

Implemented in the ROS controller while leaving the distal model unchanged:

- directional lower/upper backlash intervals, remaining interval, reversal
  encoder anchor, first-response anchor, accumulated travel, effective motor
  coordinate, confidence, confirmation count, and causal evidence timestamp;
- source-time engagement-belief checkpoints restored at accepted marker time
  and replayed through newer encoder samples by the existing sole 50 Hz
  estimator owner;
- effective-only MPPI rollouts with no immediate-engagement hypothesis and no
  candidate-local take-up dynamics;
- conservative upper-bound raw joint reserve plus one risk-adjusted
  take-up transaction cost in seconds; interval width and confidence are
  belief inputs with no independent cost weights;
- atomic take-up arbitration for every commanded joint, including the tendon
  axis, with MPPI suspended until all axes are ENGAGED;
- a zero-command CONFIRMATION_HOLD after first credible response, followed by
  discard-and-replan after persistent confirmation;
- slower hardware-profile take-up rates of [2.0, 5.0, 1.0] in physical shaft
  rad/s;
- extended estimator/status traces for interval, anchor, evidence, and the
  selected unified take-up risk duration/cost.

The manager and firmware remain the final command, freshness, limit, watchdog,
and fault authorities.

Not yet implemented as of 2026-09-26:

- online direction/session engaged-gain adaptation and full
  effective-coordinate covariance;
- runtime replacement of `ScalarTendonChain` by the confirmed effective input
  and the accepted standalone engaged-distal checkpoint;
- a genuinely held-out archive/session with an online or independently
  calibrated session gain; `--require-held-out` now prevents accidental
  in-sample acceptance, but it cannot create held-out data;
- the optional v171 natural-strain/spatial-mode warm-start ablation described
  above.

Until those items pass the gates in Phase 2, v171 remains the deployed distal
baseline and the implemented online intervals remain constant per direction.


## Frozen-v171 mechanics plus belief-conditioned lambda pipeline (implemented 2026-09-26)

This section supersedes the earlier scratch-versus-warm-start recommendation.
The design now preserves the v171 physical structure while removing only the
history block that conflated intrinsic mechanics with mounting-dependent
take-up.

The retained distal transition is

\[
  (D+hK)(q_{k+1}-q_k)
  =
  h\left[-K(q_k-q^\star)+KB\lambda_{k+1}\right].
\]

Here \(q^\star,K,D,B\), PCS kinematics, section lengths, and rigid-tip
geometry are immutable copies of the referenced v171 artifact. The v171
ScalarTendonChain is not loaded by the candidate.

At the first persistent observed response, accepted posterior strain defines

\[
  \lambda_a
  =
  \frac{(KB)^\top K(q_a-q^\star)}
       {(KB)^\top(KB)}.
\]

For confirmed direction \(d\in\{-1,+1\}\), session \(s\), and accumulated
post-confirmation travel \(\Delta u_{\mathrm{eff}}\ge 0\), the replacement
input is

\[
  \lambda_k
  =
  \lambda_a
  + \sigma_d f_d\!\left(
      g_{s,d}\frac{\Delta u_{\mathrm{eff},k}}{u_{\mathrm{scale}}}
    \right).
\]

The map \(f_d(0)=0\) is strictly monotone, \(\sigma_d\in\{-1,+1\}\) is
the observed response sign, and \(g_{s,d}>0\) is a direction-specific session
gain. Identification is the fixed unit-gain reference. There is no free session
offset: remount-dependent offset and reversal travel remain in the belief.

There is no learned scalar time constant after \(f_d\). Frozen v171 damping
\(D\) is the only distal temporal closure; adding the v2 filter would
double-count lag.

The online contract is:

1. raw reversal enters TAKEUP and suspends MPPI;
2. slow take-up compensation moves the joint;
3. persistent distal response confirms engagement;
4. accepted UKF strain registers \(\lambda_a\);
5. the belief begins accumulating \(\Delta u_{\mathrm{eff}}\);
6. only then does MPPI resume with the frozen v171 transition.

Implemented files:

- networks/hybrid/v171_engaged_lambda.py: zero-anchored directional maps and
  the exact frozen-v171 implicit transition;
- deployment/v171_belief_lambda_checkpoint.py: strict hash-verified loader for
  v171, v174, and the engagement prior; legacy history reuse is rejected;
- scripts/prepare_v171_belief_lambda_dataset.py: continuous-history causal
  labeling, K-weighted anchoring, and exact 64-point material-grid preparation;
- scripts/train_v171_belief_lambda.py: response-coordinate initialization then
  coherent Cartesian rollout fitting through fixed \(K,D,B,q^\star\);
- scripts/evaluate_v171_belief_lambda.py: held-out Cartesian and lambda errors,
  spatial manifold floor, breakdowns, plot, states, and PASS/FAIL gate;
- scripts/run_v171_belief_lambda_pipeline.sh: reproducible entry point.

Phase-2e visit 2 is held out by default. Episodes select traces but never reset
history. The prepared artifact contains 32 confirmed segments: 17 training and
15 held-out. Median first-response travel differs between identification
(3.63 mm) and phase-2e (1.89 mm), supporting the separation of engagement
belief from intrinsic mechanics.

A two-static-epoch/one-dynamic-epoch CPU smoke fit completed end to end and
intentionally failed accuracy thresholds; it validates plumbing only.
Production training must use the full schedule. The deployed ROS v171 runtime
remains unchanged until the held-out gate passes.


### Full-training audit: frozen-v171 plus belief lambda (2026-09-26)

The completed artifact failed the original uninterrupted-segment gate:

| population | coherent centerline RMS p95 | coherent tip p95 | frozen-v171 manifold floor p95 |
|---|---:|---:|---:|
| training | 6.423 mm | 13.696 mm | 2.947 mm |
| held-out phase-2e visit 2 | 5.182 mm | 11.042 mm | 2.746 mm |

The configured absolute limits (1.5 mm centerline and 2.5 mm tip) were below
the held-out frozen-manifold floor and therefore could not be passed even by a
perfect engaged-input map. The absolute uninterrupted-segment gate is not a
valid promotion test for a short-horizon receding-horizon controller.

A posterior-conditioned horizon audit gives:

| horizon | held-out centerline RMS p95 | held-out tip p95 |
|---:|---:|---:|
| 1 frame | 0.332 mm | 0.704 mm |
| 4 frames | 1.238 mm | 2.613 mm |
| 8 frames | 2.117 mm | 4.634 mm |
| 16 frames | 3.511 mm | 7.372 mm |
| 32 frames | 4.874 mm | 10.529 mm |

An oracle using the posterior stationary lambda reduces held-out one-step
errors only to 0.260 mm centerline and 0.486 mm tip p95. Thus the fitted map is
locally close to the best input available under the frozen model. The dominant
failure is accumulated coherent drift, not an incorrect immediate response
direction.

The fitted phase-2e gains are strongly session and direction dependent:
0.834 for relaxing and 0.478 for loading relative to identification. This is
consistent with remount-dependent transmission, but those fitted gains are not
available a priori in a future hardware session. Online gain adaptation remains
a required deployment item.

The next acceptance correction is:

1. make the primary gate use the exact MPPI rollout timestep and horizon;
2. report oracle-input and frozen-manifold floors beside candidate error;
3. optimize the map with matched 1/4/8-step losses rather than using
   observed-state reinitialization inside randomly selected 64-frame windows;
4. retain uninterrupted segment rollout only as a drift diagnostic;
5. require online session-gain initialization/adaptation before ROS promotion.

Until that correction passes, the candidate checkpoint is an offline result
and must not replace the deployed v171 runtime.

### Equilibrium-only frozen-v171 manifold diagnostic (2026-09-26)

The earlier quantity called the frozen-v171 "spatial floor" was not a true
floor: it K-weighted-projects every posterior frame, including moving frames,
onto the static family in the estimated interface frame. The corrected
diagnostic is implemented by `scripts/diagnose_v171_equilibrium_manifold.py`
and the `diagnose-manifold` pipeline stage.

A frame is accepted only in the terminal tail of a persistent interval with
both centered motor speed below 0.05 mm/s and centered observed Cartesian
shape speed below 1.0 mm/s. The still interval must last at least 0.75 s; only
its final 0.5 s is evaluated. For every accepted frame the primary scalar
coordinate and a nuisance proper-rigid transform are selected directly in
Cartesian space:

\[
  (\lambda_C,R_C,t_C)
  = \arg\min_{\lambda,R\in SO(3),t}
    \sqrt{\frac{1}{N}\sum_{i=1}^{N}
      \left\|R p_i(q^\star+B\lambda)+t-p_i^{\rm posterior}\right\|_2^2}.
\]

For each candidate \(\lambda\), \(R_C,t_C\) are solved by Kabsch registration.
This tests intrinsic curve shape without allowing E-step interface-pose noise
to become a false manifold residual. Pose-fixed Cartesian optimization and the
K-weighted coordinate remain in the report as gauge/coordinate comparators,
not spatial floors.

The joint identification/phase2e archive yielded 201 dwell-tail frames over
persistent equilibrium intervals:

| population | frames | rigid-registered RMS median | rigid-registered RMS p95 | pose-fixed optimal RMS p95 | K-weighted pose-fixed RMS p95 |
|---|---:|---:|---:|---:|---:|
| identification | 39 | 0.005 mm | 0.009 mm | 0.116 mm | 0.142 mm |
| phase2e | 162 | 0.171 mm | 0.268 mm | 1.991 mm | 2.285 mm |
| phase2e visit 1 | 69 | 0.139 mm | 0.248 mm | 2.100 mm | 2.396 mm |
| phase2e visit 2 | 93 | 0.203 mm | 0.273 mm | 1.617 mm | 1.993 mm |

Therefore phase2e equilibrium shapes do **not** differ from the v171 intrinsic
one-dimensional manifold by 2--3 mm. Once interface-pose gauge is removed,
the phase2e tail is only 0.268 mm p95. The large reduction from 1.991 mm
pose-fixed to 0.268 mm rigid-registered shows that most of the apparent
cross-session spatial discrepancy is the estimated interface frame, not the
distal curve shape. Identification is nearly exact because v171 and its
posterior were jointly fit in that same gauge.

This supports retaining the v171 intrinsic spatial mechanics. The remaining
controller-model investigation should focus on belief/input registration,
interface-pose consistency, and matched-horizon dynamics. Promotion gates must
report the rigid-registered equilibrium diagnostic beside the pose-fixed
controller-frame error: the former tests intrinsic shape, while the latter
still matters operationally to Cartesian control. Moving-frame projection
residual remains a dynamics/interface-estimation diagnostic, not a manifold
floor.

### Motor-independent v171-constrained geometric E-step (implemented 2026-09-27)

Inspection showed that the previous phase2e "geometry posterior" was not
constrained by v171 mechanics. It used `process_weight=0`, jointly optimized a
five-DOF interface-pose spline and a free 24-dimensional PCS-strain spline, and
biased the interface axis toward the chord from material samples 0 to 6
(5.43 mm). The resulting curve fit was good, but pose and proximal strain could
exchange several degrees without an adequate physical factor. On equilibrium
dwell frames, an origin-constrained frozen-v171 fit preferred an interface
orientation 2.70 degrees away from the old posterior at the median and 4.51
degrees at p95. The old interface origin itself was much better determined
(0.283 mm median error to the first observed point).

The external E-step now supports `process_mode=free_input`. Let

\[
  \bar q_k = F_{171}(q_{k-1},0,\Delta t_k), \qquad
  s_k = F_{171}(q_{k-1},1,\Delta t_k)-\bar q_k .
\]

Because the frozen v171 transition is affine in its scalar effective tendon
input, the nuisance input for a proposed adjacent posterior pair is eliminated
analytically:

\[
  \lambda_k^\star
  = \frac{s_k^\top(q_k-\bar q_k)}{s_k^\top s_k}, \qquad
  r_k=q_k-\bar q_k-s_k\lambda_k^\star .
\]

The E-step penalizes \(r_k\), the transition component that no scalar v171
input can explain. It does not use motor position, motor-to-tendon
registration, backlash width, v171 tendon history, or engagement timing. Motor
motion is retained only to select the constant material-roll gauge, which does
not impose temporal registration. The curved point-6 chord tangent penalty is
disabled. The first observed point continues to anchor interface translation.
The inferred \(\lambda_k^\star\) and residual are saved in the posterior
artifact for audit.

The phase2e wrapper now uses this free-input factor for the primary geometry
posterior and retains the motor-history posterior only as a named sensitivity
comparison. Focused tests verify exact recovery of a scalar-input v171
transition and rejection of orthogonal strain motion. A 600-frame/two-epoch
recorded-data smoke run completed the optimization and schema-v2 artifact
export; it is plumbing evidence only, not a converged posterior. The full
80-epoch E-step must be rerun before regenerating the rollout and belief-lambda
dataset.

#### Standalone successor

Frozen v171 is transitional rather than the final dependency. The standalone
pipeline should reuse v171's alternating generalized-EM structure:

1. initialize a shared intrinsic manifold and first-order mechanics from
   multi-session equilibrium curves, with no motor registration in the E-step;
2. E-step: infer interface pose, PCS state, material gauge, and free effective
   input using the analytic nuisance-input projection above;
3. M-step: update shared \(q^\star,B,K,D\) and PCS geometry from confirmed
   engaged data using static-equilibrium plus matched 1/4/8-step Cartesian
   objectives;
4. fix the usual gauges explicitly (unit/sign convention for \(B\), stiffness
   or damping scale anchor, and one material-roll gauge);
5. keep remount/session backlash, take-up interval, engagement confidence, and
   motor-to-effective-input gain in the separate online belief model;
6. alternate until held-out-session pose-invariant shape, pose-fixed
   observation, and matched-horizon dynamics gates stop improving.

The standalone E-step must be invariant to shuffling or replacing motor
encoder values after the constant material gauge is fixed. Only the later
belief/transmission fit may associate motor travel with the inferred effective
input.



### Fresh-confirmation engaged-branch quality gate (implemented 2026-09-27)

The belief-lambda preparation formerly admitted any selected frame carrying an
ENGAGED state. Because engagement history is correctly continuous across
episodes, a selected compensated-bending episode could inherit engagement from
a preceding episode. The builder then created a new response anchor at the
episode boundary, allowing insertion-level changes or posterior gauge motion
to masquerade as a tendon response branch.

Supervised branches now satisfy all of the following:

1. a new reversal/response event is confirmed inside the selected episode;
2. confirmation has the causal response sign for that motor direction;
3. the post-confirmation branch contains at least 1.0 mm effective motor
   travel;
4. its signed endpoint response is at least 2.0 lambda units;
5. at least 75% of response increments are direction-consistent, allowing
   0.25 lambda units of per-frame posterior noise.

The causal response-sign convention is identified from the clean
identification bending episode. For the current coordinate it is

\[
  \operatorname{sign}(\Delta\lambda)
  = \operatorname{sign}(\Delta u_m).
\]

Phase2e confirmation therefore ignores an opposite-sign threshold crossing
caused by posterior drift and waits for persistent response in the physically
valid direction. This changes confirmation timing and take-up width rather
than post-processing or modifying a confirmed branch.

Every accepted frame now carries event ID, reversal and confirmation frame,
reversal and engagement motor position, event width, and response sign.
prepare.json reports all-recording widths separately from accepted selected
widths and includes accepted/rejected event details plus rejection reasons.

On the corrected free-input phase2e posterior:

| population | accepted branches | rejected selected events | supervised frames |
|---|---:|---:|---:|
| identification | 2 | 0 | 495 |
| phase2e visit 1 (training) | 5 | 1 | 1,426 |
| phase2e visit 2 (held out) | 6 | 2 | 591 |

The three rejected phase2e candidates consist of one wrong/insufficient
post-confirmation response and two branches with less than 1.0 mm engaged
travel. The corrected archive has seven training branches and six held-out
branches, satisfying the trainer's coverage gate without using inherited or
static pseudo-branches.


### Static engaged-map boundary and matched-horizon gate (implemented 2026-09-27)

The first corrected-branch training still allowed a dynamic Cartesian stage to
change the equilibrium motor-to-lambda map. With only one short phase2e loading
branch in training, that stage inflated the phase2e loading gain to 4.385 to
compensate frozen-v171 transient lag. Held-out loading then diverged.

The corrected boundary is now hard rather than weight-based:

- the engaged map is fit only to response-anchored equilibrium lambda data;
- dynamic refinement of that map is prohibited (`dynamic_epochs` must be zero);
- session gains are bounded to [0.5, 2.0];
- checkpoint kind/schema is v171_engaged_lambda_v2/version 2;
- the strict loader requires static-equilibrium-only provenance and rejects
  dynamically refined candidates;
- training records every branch endpoint gain ratio;
- evaluation reports posterior-initialized 1/4/8-frame candidate and
  observed-lambda oracle rollouts in addition to uninterrupted drift.

Retraining produced phase2e gains 1.021 (relaxing) and 1.694 (loading), versus
0.922 and the invalid 4.385 previously. Training endpoint gain ratios have
median 0.983, p95 1.160, and maximum 1.194.

Matched held-out results are:

| horizon | duration median | candidate centerline p95 | candidate tip p95 | oracle centerline p95 | oracle tip p95 |
|---:|---:|---:|---:|---:|---:|
| 1 frame | 0.033 s | 0.613 mm | 1.377 mm | 0.220 mm | 0.490 mm |
| 4 frames | 0.133 s | 2.269 mm | 5.103 mm | 0.796 mm | 1.783 mm |
| 8 frames | 0.267 s | 4.062 mm | 9.129 mm | 1.371 mm | 3.079 mm |

The candidate therefore remains offline-only. The static/dynamic conflation is
fixed, but two held-out confirmed branches have much weaker response than the
nominal engaged map: endpoint gain ratios 4.16 and 8.42. Because the
observed-lambda oracle passes the four-frame limits, the remaining four-frame
failure is input-response uncertainty rather than intrinsic spatial mechanics.
The next controller/model step is online engaged-gain belief adaptation and
uncertainty-aware MPPI rollout, not renewed dynamic fitting of the distal map.

## Engaged-gain belief and uncertainty-aware MPPI plan (2026-09-27)

### Scope and model boundary

The corrected engaged-only result supports a specific model boundary. The fixed
v171 mechanics remain the nominal spatial manifold, but one session/direction
gain does not predict every later engaged branch. The controller therefore
shall separate:

- a fixed nominal engaged response and distal mechanics;
- an online, direction-specific engaged-gain belief;
- an observation-terminated take-up transaction;
- gain-aware MPPI that operates only after every commanded axis is confirmed
  engaged.

This section is the authoritative extension of the earlier scalar
**a_i^tx** belief field and Phase 4 MPPI objective. It does not introduce a
second supervisor or a second set of confidence penalties.

### 1. Complete belief-state extension

For axis \(i\), retain the existing engagement state

\[
b_{i,k}=\left(
m_{i,k},d^{cmd}_{i,k},d^{eng}_{i,k},\mathcal G_{i,k},r_{i,k},
u^{rev}_{i,k},u^{eng}_{i,k},\tau^{eng}_{i,k},
\Delta u^{takeup}_{i,k},u^{eff}_{i,k},
P^{takeup}_{i,k},c^{eng}_{i,k},e_{i,k}
\right).
\]

For each engaged direction \(s\in\{-1,+1\}\), add

\[
G_{i,s,k}=\left(
\mu^\ell_{i,s,k},P^\ell_{i,s,k},
g^L_{i,s,k},g^U_{i,s,k},
n^g_{i,s,k},t^g_{i,s,k},
\nu^g_{i,s,k},\rho^g_{i,s,k}
\right).
\]

Here \(\ell=\log g\), so \(g=\exp(\ell)>0\);
\(\mu^\ell,P^\ell\) are the posterior log-gain moments;
\([g^L,g^U]\) is its credible interval; \(n^g\) is the accepted informative
sample count; \(t^g\) is the last accepted source timestamp; \(\nu^g\) is the
normalized innovation; and

\[
\rho^g\in\{
\mathrm{UNOBSERVED},\mathrm{LEARNING},
\mathrm{CONFIDENT},\mathrm{DEGRADED}
\}
\]

is derived from covariance, evidence, innovation, and age. It is not an
independently tuned latent.

The full controller belief becomes

\[
\mathcal B_k=\left(
b_{0,k},b_{1,k},b_{2,k},
\{G_{i,-,k},G_{i,+,k}\}_{i=0}^{2},
o_k,H_k
\right),
\]

where \(o_k\) is the estimator/observation state and \(H_k\) is rewindable
history. Loading and relaxing gains remain separate. Reversing activates the
opposite-direction posterior; it does not overwrite either posterior.

The static trainer bound \(g\in[0.5,2]\) is not an online support bound. The
weak held-out branches imply gains near \(0.1\)--\(0.25\) relative to the
nominal response. Initial online support shall cover at least

\[
g\in[g_{min},g_{max}]=[0.1,2.0],
\]

then be recalibrated using causal leave-one-session-out replay.

### 2. Engaged response and causal gain update

Let \(f_{i,s}(\Delta u^{eff})\) be the frozen, zero-anchored, directional
engaged response. For the tendon axis,

\[
\lambda_k-\lambda_{eng}
=
g_{i,s,k}
f_{i,s}\!\left(u^{eff}_k-u^{eff}_{eng}\right)+\epsilon_k.
\]

The accepted engagement observation fixes \(\lambda_{eng}\); no free branch
offset is learned. Use incremental observations online:

\[
z_k=\lambda^{obs}_k-\lambda^{obs}_{k-1},\qquad
\phi_k=f_{i,s}(\Delta u^{eff}_k)-f_{i,s}(\Delta u^{eff}_{k-1}),
\]

\[
z_k=\exp(\ell_{i,s,k})\phi_k+v_k,\qquad
v_k\sim\mathcal N(0,R^g_k).
\]

This cancels constant registration error and identifies the engaged response
rate needed by MPPI. Propagate a slow log-gain random walk using source time:

\[
\ell_{k+1}=\ell_k+w_k,\qquad
w_k\sim\mathcal N(0,Q^g_i\Delta t_k).
\]

For an accepted update, use a robust scalar EKF/RLS form:

\[
\hat z_k=\exp(\mu^-_k)\phi_k,\qquad
H^g_k=\exp(\mu^-_k)\phi_k,
\]

\[
S_k=H_k^{g\,2}P^-_k+R^g_k,\qquad
K_k=P^-_kH^g_k/S_k,
\]

\[
\mu^+_k=\mu^-_k+
K_k w^{rob}_k(z_k-\hat z_k),\qquad
P^+_k=(1-K_kH^g_k)P^-_k.
\]

Use a Huber or Student-t innovation weight \(w^{rob}\). An outlier reduces the
update and degrades confidence; it must not create an engagement transition.

The controller interval is

\[
[g^L,g^U]=
\left[
\exp(\mu^\ell-z_\alpha\sqrt{P^\ell}),
\exp(\mu^\ell+z_\alpha\sqrt{P^\ell})
\right]\cap[g_{min},g_{max}].
\]

Log every support-envelope clip. Repeated clipping is a model-validity warning,
not evidence of confidence.

### 3. Observability gates and ownership

Update \(G_{i,s}\) only when all conditions hold:

1. The axis is **ENGAGED_CONFIRMED** in the current effective-motion
   direction.
2. No axis is in take-up and no reversal occurred inside the observation
   interval.
3. \(|\phi_k|\ge\phi_{min}\); stopped frames do not identify gain.
4. Marker correction and estimator health are valid and source timestamps
   exist.
5. Distal response has the causally learned sign for this direction.
6. No joint saturation, command projection, position transaction, or safety
   stop affected the interval.
7. Cross-axis attribution is clean, or the joint regressor is sufficiently
   conditioned.
8. The innovation passes the robust statistical gate.

For tendon, use UKF-corrected distal bending as primary evidence, not interface
motion. Chassis/rotation transmission remains owned by the interface
pose/adaptive Jacobian. The distal gain updater and proximal Jacobian must
never adapt from the same unexplained residual.

Rejected or uninformative samples perform prediction-only propagation. Weak
response after confirmed engagement lowers gain or increases uncertainty; it
does not send the axis back into take-up. Only a commanded reversal opens a
new take-up transaction. Persistent monotone-response loss produces
**engaged_gain_degraded**, not repeated artificial take-up.

Delayed observations participate in estimator rewind/replay. Each history
checkpoint stores \(b_i\) and all \(G_{i,s}\); replay applies the delayed
correction/update at source time and deterministically propagates both beliefs.

### 4. Priors and branch transitions

Maintain directional session priors

\[
\ell^{session}_{i,s}\sim
\mathcal N(\mu^{session}_{i,s},P^{session}_{i,s}).
\]

At startup use a broad cross-session prior. After reliable within-session
evidence, update the session prior slowly. On a later reversal into direction
\(s\),

\[
\mu^\ell_{i,s}\leftarrow\mu^{session}_{i,s},\qquad
P^\ell_{i,s}\leftarrow
P^{session}_{i,s}+P^{reversal}_{i,s}.
\]

This reuses session evidence without assuming equal branch gains. It becomes
active only after observed engagement; there is no physically implausible
immediate-engagement hypothesis.

At confirmation:

- freeze the raw encoder engagement anchor;
- register the effective-coordinate origin;
- save the accepted distal-response anchor;
- activate the directional gain prior with inflated variance;
- invalidate every pre-take-up MPPI plan and request a fresh plan.

### 5. Gain-aware MPPI

MPPI samples effective post-take-up commands. It never commands raw take-up
travel. Arbitration is

\[
\text{belief}
\rightarrow
\text{slow take-up if required}
\rightarrow
\text{all required axes confirmed}
\rightarrow
\text{gain-aware MPPI}
\rightarrow
\text{raw command conversion}.
\]

If any required axis is unconfirmed, no MPPI output is executed. The slow
take-up compensator owns that axis and other axes are held unless an
independently verified transaction is explicitly supported. Confirmation
always causes a fresh MPPI plan from the new measured state.

At each planning cycle construct a small coherent scenario set

\[
\ell^{(m)}\sim\mathcal N(\mu^\ell,P^\ell),
\qquad m=1,\ldots,M,
\]

containing at least mean and lower/upper credible sigma points. Hold a sampled
gain constant through one rollout horizon because branch gain changes slowly.
Use common random numbers across candidates. For coupled gains, sample the
joint covariance rather than independent marginals.

For scenario \(m\),

\[
\lambda^{(m)}_{h+1}
=
\lambda_{eng}
+
g^{(m)}_{i,s}
f_{i,s}\!\left(
u^{eff}_{0:h+1}-u^{eff}_{eng}
\right),
\]

followed by frozen distal mechanics and kinematics.

Use one uncertainty-aware objective:

\[
J(U)=
\operatorname{Risk}_m\!\left[
C^{(m)}_{tip}(U)+C^{(m)}_{boundary}(U)
\right]
+C_{slew}(U)+C_{takeup-risk}(U),
\]

\[
\operatorname{Risk}_m[C]
=
\mathbb E_m[C]
+\beta_{risk}\operatorname{CVaR}_\alpha(C).
\]

This replaces nominal tip evaluation. Do not add separate
**takeup_uncertainty_weight**, **takeup_low_confidence_weight**, or
engaged-gain confidence penalties. Uncertainty changes the predicted outcome
distribution directly.

Hard constraints must hold for all conservative scenarios. Limit the first
effective step using the upper gain interval:

\[
g^U_{i,s}
\left|f_{i,s}(\Delta u^{eff})\right|
\le \Delta\lambda^{safe}_i.
\]

Broad uncertainty therefore yields smaller safe steps without modifying the
chosen MPPI plan after optimization.

MPPI may compare a reversal mode by predicting its eventual post-engagement
trajectory using the opposite-direction prior plus the existing take-up
transaction cost. If selected, execution is still a slow atomic take-up
transaction followed by replanning after confirmation. The pre-reversal plan
is never resumed.

### 6. Weak-response behavior

- Start each newly confirmed branch with a broad posterior and a small
  monotone effective step.
- Accepted weak response lowers estimated gain and preserves direction.
- No excitation leaves gain uncertain; it is not evidence for zero gain.
- Contradictory evidence widens/degrades the posterior.
- Gain uncertainty alone never commands reversal.
- If no candidate safely improves cost over the credible interval, hold and
  report **engaged_gain_unobservable** or **engaged_gain_degraded**.
- Never reverse solely to identify gain. Optional excitation must already
  improve the target robustly and remain safe over the full interval.

### 7. Implementation sequence

#### 7.1 cr_meta_lnn

1. Add an **EngagedGainBelief** value type with log-domain propagation,
   robust update, credible interval, and derived status.
2. Add gain states to streaming snapshots and rewind/replay checkpoints.
3. Expose accepted distal-coordinate increments and covariance from the UKF
   correction path.
4. Extend frozen engaged-map rollout to accept batched gain scenarios without
   changing nominal parameters.
5. Trace every proposed/accepted/rejected gain update with timestamp,
   direction, regressor, response, innovation, posterior, and reason.

#### 7.2 robot-infra

1. Give each planner job one immutable engagement-plus-gain snapshot.
2. Keep MPPI disabled during take-up and invalidate jobs at reversal and
   confirmation.
3. Add vectorized gain scenarios to grouped MPPI; optimize one intact plan per
   mode and never post-modify its axes.
4. Enforce scenario-wide limits and the upper-gain first-step cap.
5. Publish direction, gain mean/interval/status, update age, scenario costs,
   and selected risk cost.

#### 7.3 Minimal configuration surface

Expose only \(Q^g_i\), observation-noise floor, robust innovation threshold,
\([g_{min},g_{max}]\), credible level \(\alpha\), risk coefficient
\(\beta_{risk}\), \(\phi_{min}\), and
\(\Delta\lambda^{safe}\). Confidence is derived, not tuned by duplicate
weights.

### 8. Verification gates

#### G1: causal offline replay

Use leave-one-branch-out and leave-one-session-out replay without future
endpoint information. At each reversal, initialize the available prior,
confirm from observed response, then update only from past accepted samples.
Compare fixed gain, posterior mean, robust scenarios, and measured-\(\lambda\)
oracle. Report at 1/4/8 frames:

- centerline/tip median, p95, and maximum error;
- realized-slope credible-interval coverage and negative log likelihood;
- travel/time to useful confidence;
- false-confidence and support-clipping counts;
- weak held-out branch metrics.

Pass only if one-frame results do not regress, interval coverage is within 10
percentage points of nominal, 4/8-frame p95 materially improves over fixed
gain, and no branch becomes **CONFIDENT** without informative causal evidence.

#### G2: randomized closed-loop simulation

Randomize take-up width, gain over empirical support, modest gain drift,
latency, and marker rejection. Verify zero MPPI execution during take-up,
immediate replanning at confirmation, no uncertainty-only reversals, bounded
high-gain overshoot, weak-gain progress without false take-up, and planner
deadlines under full-stack load.

#### G3: hardware isolation

Only after G1/G2 pass and with explicit hardware authorization, run low-speed
no-rotation two-axis sparse targets, starting near. Audit gain convergence,
take-up/MPPI arbitration, increment prediction, overshoot, and reversal count
before farther points or continuous paths.

### 9. Promotion rule

Promote this path only if it improves causal multi-frame prediction and
closed-loop behavior without making engagement detection depend on predicted
timing. Until then, retain v171 mechanics as the deployed nominal model and
keep gain-aware control offline/simulation-only.



## Engaged-gain implementation and direct hardware gate (2026-09-27)

This section supersedes the earlier requirement that a randomized plant
simulation qualify promotion. The adaptive belief is driven by real accepted
UKF response, so the useful pre-hardware gates are deterministic unit tests,
exact real-model GPU timing, and a non-actuating full-stack identity check.
Physics simulation remains optional and is not evidence that the online gain
belief is correct.

### Implemented estimator path

- Each accepted delayed UKF update exports the distal equilibrium coordinate
  immediately before correction, \(\lambda^-_k\), and after history
  reconciliation, \(\lambda^+_k\). Rejected marker updates export no gain
  evidence.
- The rewindable transmission checkpoint owns a positive log-domain belief
  \(\log g_{i,s}\sim\mathcal N(\mu_{i,s},P_{i,s})\) for both directions.
- A reversal widens the selected branch to the reversal prior. No update is
  allowed in `UNKNOWN`, `TAKEUP`, `PROVISIONAL`, or `FAILED`; the first accepted
  sample after `ENGAGED` is only an anchor.
- Later accepted samples use

  \[
  \phi_k=\lambda^-_k-\lambda^-_{k-1},\qquad
  y_k=\lambda^+_k-\lambda^+_{k-1},\qquad
  y_k=g_{i,s}\phi_k+\epsilon_k.
  \]

  Small \( |\phi_k| \), opposite-sign response, and excessive normalized
  innovation do not change the mean. They retain or widen uncertainty and emit
  a reason code.

### Implemented MPPI path

For each intact grouped-MPPI control candidate, the runtime evaluates the
posterior mean, lower credible gain, and upper credible gain as three complete
rollouts around the current distal belief:

\[
\lambda^{(m)}_{k+h}=\lambda_k+g^{(m)}_{i,s}
  (\lambda^{nom}_{k+h}-\lambda_k).
\]

The candidate tracking cost is

\[
C_{track}=\mathbb E_m[C_m]+0.5\,\operatorname{CVaR}_{0.67}(C_m).
\]

The selected command is never modified after scoring. A candidate is
ineligible if any credible scenario changes distal equilibrium by more than
8.0 in its first 40 ms step. Physical joint limits separately reserve the
upper estimated take-up travel before rollout. There is no shape-target cost
and no duplicate confidence penalty.

During a take-up transaction MPPI execution is inhibited. Pending shafts move
at the bounded hardware rates `[1.0, 2.5, 0.5]`; confirmed shafts hold. Once
every requested shaft is response-confirmed, the arbiter emits a zero barrier,
discards the old plan, and requests a fresh gain-aware plan.

### Hardware profile and gates

Both `v175_grouped_hardware.yaml` and its no-rotation isolation variant enable
the belief and three-scenario MPPI. They use 384 control candidates, hence
1,152 model trajectories (4,608 candidate-steps) per planning tick. Hardware
output remains absent from the profiles and disabled by the launch default.

Verification completed on 2026-09-27:

- `cr_meta_lnn`: 35 focused runtime/UKF tests passed.
- `catheter_control`: 276 tests passed after a ROS rebuild.
- Exact CUDA gain-scenario preflight: 100/100 valid plans; p50 16.78 ms,
  p95 17.96 ms, p99 18.58 ms, maximum 18.69 ms against a 60 ms deadline.
- Report: `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260927_180602_phase5_preflight.json`.

This is **software-ready, not hardware-qualified**. Before powered motion,
verify the installed runtime identity with `command_output_enabled=false`,
then use the no-rotation axial and near two-axis experiments. Promotion to
rotation or a continuous path requires an audit of take-up/MPPI mutual
exclusion, gain updates, first-step cap rejections, deadline misses, overshoot,
and reversal count from those hardware sessions.
