# Gate 3 filter pipeline reference

Current research architecture, 2026-11-01. Use with the selected checkpoint
and configuration: implemented options are not automatically defaults.
[Current status](GATE3_FILTER_STATUS.md) distinguishes scalar research,
array-native shadow and the separately qualified deployed controller.

## 1. Purpose and state

Estimate distal deformation from four markers, infer transmission state,
learn engaged Cartesian response, and forecast a selected control point.
Maintain one authoritative deformation belief. Predictions never train parameters.

Research positions/translations use mm; rotation uses rad. Deployment APIs
retain their own unit contracts. Index k denotes images; n native samples.

| Symbol | Meaning |
| --- | --- |
| y, R | Four marker XYZ observations and full 12-by-12 noise covariance |
| u, c, kappa | Measured calibrated tendon, chassis and direct-knob coordinates |
| lambda, beta | Distal deformation and current visit's profile coefficients |
| pi | Tendon held/engaged response probability |
| m | Transmitted chassis-position memory, not motor position |
| alpha | One positive physical knob multiplier |
| p_T, p_I | Modeled tip/control-point and interface positions |
| b_T, b_I | Engaged axial Cartesian response vectors |
| j_T, j_I | Cartesian response vectors per unit observed lambda change |

The distal owner retains lambda/profile moments, correlations and visit history.
Chassis memory retains scalar support locations, weights and conditional full
Cartesian/parameter/history moments. Tip and interface share alpha but have
different response vectors. At EACH point, chassis and knob share an axial
vector, with knob multiplied by alpha. There is no separately learned angular J.

Previous-image/raw-error correlations are bookkeeping for observed increments,
not a second lambda belief. Old image coordinates are marginalized after
completion; native processing retains the declared pending-image correlations.

## 2. Information flow

```text
Previous accepted distal state + scalar chassis-memory distribution
                         |
Measured, calibrated, timestamp-aligned native encoders
                         |
Recovery/prior process -> native transport -> visit-prior preparation
                         |
Current four-marker image + full R -> geometric likelihood/partition
                         |
ONE distal held/engaged update -> accepted lambda/profile + probability
                         |
Memory-conditional Cartesian correction, protecting distal statistics
                         |
Correlated observed motion + refined interface evidence
                         |
Admission -> parameter-only update -> full correlation transport
                         |
Memory weighting -> protected-marginal reconciliation -> retention
                         |
ONE complete accepted image state
             |                              |
      Next measured interval        Private frozen-root forecast
                                    NO future markers or learning
```

Within-stage ordering belongs to the existing owners/saved settings. Recovery,
prior floors and covariance protection cannot be reordered just because their
means appear unchanged. No new history reset occurs at an image or reversal.

## 3. Geometry and deformation

For curve family g(lambda,s), current image orientation Q and marker stations s_i:

$$
y_{k,i}=p_{T,k}+Q_k[g(\lambda_k,s_i)-g(\lambda_k,s_T)]+\epsilon_{k,i}.
$$

Orientation and unanchored station coordinates are geometric nuisance variables.
M0's MATERIAL STATION is anchored; its observed XYZ still has sensor noise.
This does not force the posterior through raw M0 or freeze interface position.

The shape update uses the raw-image likelihood, not another filtered lambda
series. Complementary Cartesian evidence uses the same partition/correlation
accounting. Refined interface evidence carries correlations with that context;
it is not an independent sensor or second pose filter.

For direction d in {+1,-1}, v=d*u and f=d*lambda, the visit profile is

$$
f(v)=B(v)\beta,\qquad \beta\sim\mathcal N(\mu_\beta,P_\beta).
$$

Uniform cubic B-spline coefficients represent a projected GP prior. Query means
and covariances are B*mu_beta and B*P_beta*B-transpose.

Since 2026-11-02 (R1), B is evaluated at the actual oriented tendon input. On a
node of the saved input grid this is the stored row, bit for bit. Between
nodes it is direct basis evaluation. The saved grid fixes the identified data
support [a, b] (0.000893 to 10.766078 mm on the current artifact), the
slope-constraint locations and the dense-opening domain. Dense grid owners
still require exact nodes.

Support continuation (2026-11-02): with `profile_support_extension_mm` m, the
same coefficients continue linearly beyond each end:

$$
\tilde B(x)=B(b)+(x-b)\,B'(b)\ \ (b<x\le b+m),\qquad
\tilde B(x)=B(a)+(x-a)\,B'(a)\ \ (a-m\le x<a).
$$

Inputs beyond the continuation are rejected, and the estimator holds and then
faults. The real-time path selects m = 0.5 mm; the scalar reference keeps
m = 0. An admitted image beyond [a, b] withholds profile learning; lambda,
engagement and tip-J learning are unchanged. Shared visit-prior origins may
lie on the continuation. The minimum-slope constraint does not cover the
continuation. The support should cover the joint-limit range plus a margin;
the current artifact does not yet, pending full-range data. Queries share coefficient
uncertainty; they are not fresh independent noise. The selected real fixture
has22 coefficients; dimensions,artifacts and scales remain configuration-bound.

For engaged monotone travel, the basic actuation response is

$$
\lambda_k^-=\lambda_{k-1}+d[f(v_k)-f(v_{k-1})].
$$

Held response contributes no actuation-driven deformation increment.
Within-interval crossings use the existing episode/width prior and quadrature.
Return-frontier handling remains the existing owner policy; not every return
calculation is fully soft. Configured stationary lambda process uncertainty
is distinct from actuation-driven response.

The same image likelihood conditions temporary held/engaged laws. With
likelihoods L_H,L_E:

$$
\pi_k=\frac{\pi_k^-L_E}{(1-\pi_k^-)L_H+\pi_k^-L_E}.
$$

The owner normalizes log probabilities. Enabled directional evidence uses
truncated-Gaussian moments,not a veto on every wrong-sign noisy sample.
Held/unadmitted evidence protects engaged-profile parameters. A parameter
learning threshold coexists with soft state prediction. Opening adaptation
occurs once per visit; later images do not repeat initialization. Existing
local/persistent corrections and slope constraints retain their selected
settings; this is not a persistent global history map.

Opt-in in the owner: `SoftDeformationRepairs.learning_minimum_travel_mm` also
withholds profile admission when image-to-image tendon travel is below that
floor. Lambda, engagement evidence and tip-J deformation learning are unchanged.
The user enabled it at 0.01 mm for the real-time filter path
(`scripts/reduced_filter_session.py`, `reduced_filter_stream.py` and its research
ROS observer) on 2026-11-01. The scalar reference and control runtimes do not
select it; see the
[spike diagnosis](GATE3_FORECAST_SPIKE_DIAGNOSIS.md#stationary-tendon-profile-hold).

Temporary mode laws collapse by full moments:

$$
\bar x=\sum_jw_j\mu_j,\qquad
\bar P=\sum_jw_j[P_j+(\mu_j-\bar x)(\mu_j-\bar x)^T].
$$

This Gaussian approximation loses detailed mode dependence. Do not discard
crosses or replace it by simply scaling a learning increment by pi.

## 4. Transmission and Cartesian process

Fixed-width chassis play acts on each retained memory support:

$$
m_n=\operatorname{clip}(m_{n-1},c_n-w/2,c_n+w/2).
$$

Coincident memory positions do not justify merging different conditional
histories. Original recovery,regridding and observation weighting remain.
A wrong initial memory is uncertain,not a forever-certain trajectory.
Chassis full-gain admission and tendon engagement are different decisions.
Knob backlash is not separately modeled.

At either point q in {I,T}, the local response model is

$$
\Delta p_q=b_q(\Delta m+\alpha\Delta\kappa)
                 +j_q\Delta\lambda+\eta_q.
$$

No angular-J lever-arm decomposition is used for future tip displacement.
Deformation still supplies shape/control-point estimation,tendon-transmission
evidence and the deformation feature. Its Cartesian vector is learned,not
forced equal to the physical curve derivative.

State covariance uses local process linearization:

$$
P^-=FPF^T+Q_{\mathrm{process}}.
$$

This does not exactly integrate every gain-times-feature uncertainty product.
Optional J diffusion,lambda state noise and knob-variance replenishment are
distinct declared policies. Preserve checkpoint settings;cleanup is not tuning.

## 5. Observation-grounded parameter learning

Learning uses accepted consecutive-image observed motion and observed
Delta lambda=lambda_k-lambda_(k-1),with complete correlations. In particular:

$$
\operatorname{Var}(\Delta\lambda)
=P_{kk}+P_{k-1,k-1}-P_{k,k-1}-P_{k-1,k}.
$$

The parameter factor is based on the SAME physical response relation:

$$
r_q=\Delta p_q^{obs}-b_q(\Delta m+\alpha\Delta\kappa)
                         -j_q\Delta\lambda^{obs}.
$$

Observed increments are uncertain and correlated with parameters/raw errors.
They are not future predictions or independent pseudo-measurements. Never add
raw-marker covariance a second time or use a process prediction as observation.

The physical-alpha route uses one first-order recursive conditioning step.
It retains one gain,not simultaneous log/physical estimators. Positive-normal
moments followed by Gaussian closure preserve positivity approximately,not an
exact positive-support density. Explicit iterative-log/sigma-point comparisons
are not the selected physical-alpha method.

For a locally linear Gaussian observation,the usual conditioning quantities are

$$
S=HPH^T+R,\quad K=PH^TS^\dagger,\quad \mu^+=\mu+Kr,
$$
$$
P^+=(I-KH)P(I-KH)^T+KRK^T.
$$

The implementation uses supported covariance solves/constraints. The derived
motion factor already carries correlated uncertainty; it is not assigned a
fresh independent R. Protected rows have zero update gain while crosses still
transport. In block-local mode,interface evidence supplies interface-parameter
gains and tip evidence supplies tip-parameter gains,using the existing joint
factor/covariance law rather than assuming independent responses.

Full transport and derived nonlinear covariance terms lift into the retained
context,not just its parameter submatrix. Takeup/low excitation must not teach
zero engaged J. Existing full-gain,excitation and deformation admission determine
active parameter rows. Accepted distal/profile marginals stay protected through
Cartesian correction,parameter conditioning AND memory weighting,including
cross-covariance/projection side effects.

## 6. Initialization,forecast and limitations

Initialization explicitly supplies source/resource identities,geometric J,
knob scalar,distal/profile prior,memory distribution and correlations. Do not
implicitly reuse the old control fixed J or relabel incomplete checkpoint
history. Arbitrary-start uncertainty and insufficient excitation must be
reported separately from engaged control-performance metrics.

A frozen-root forecast advances transmission/profile on a private branch
without future images,parameter learning or posterior commit. Achieved future
encoder schedules and future observed lambda substitutions are privileged
diagnostics,not deployable forecasts. Predicted positions never train J.

Limits include changing Cartesian response,uncertain future profile/history,
imperfect nuisance geometry,Gaussian moment closure/protected updates and
uncalibrated forecast uncertainty. Supplied-image success does not qualify
continuous inference,all real branches,hardware control or real-time deadlines.

## 7. Existing owners

Paths are relative to cr_meta_lnn/training/hybrid/:

| Responsibility | Owner |
| --- | --- |
| Distal state/profile/episodes | distal_deformation_runtime.py, soft_deformation_episode.py, single_soft_engagement.py, spline_profile_gaussian.py |
| Geometry/registration | curve_marker_likelihood.py, curve_registration.py |
| Observed motion / Cartesian context | observed_tip_body_motion.py, tip_body_jacobian.py |
| Interface / parameter composition | interface_marker_parameter.py, interface_readout_transport.py, interface_pose_refinement.py |
| Positive knob conditioning | positive_knob_scale.py, recursive_knob_scale.py |
| Memory / full conditional orchestration | scalar_chassis_memory.py, memory_marker_runtime.py |
| Native replay/checkpoint/forecast | native_memory_marker_replay.py |
| Reduced real-time filter,arrival-order wrapper | reduced_law_filter.py, reduced_filter_stream.py; ROS observer in `runtime_supervision/reduced_filter_shadow.py`, commanding nothing |
| Batched four-step planner forecast (torch; same mean-only equations, scalar owner is the spec) | `distal_deformation_runtime.mean_forecast_snapshot`/`four_step_mean_forecast`, `spline_profile_gaussian.SplineMeanTensor`, `current_engagement_probability.engagement_log_transition_tensor`, `reduced_law_filter.ReducedFourStepForecast` |
| Shared batched helpers imported by the scalar filter | batched_interface_assembly.py, batched_interface_transport.py, batched_knob_prediction.py, batched_gaussian_validation.py |
| Planner rollout backend (fixed-shape kernels keyed by candidate count and active law count, 2026-11-02) | `reduced_filter_stream.ReducedFilterRolloutBackend`, `ReducedPlanRoot.law_count` |
| Commanding reduced-filter backend of `catheter_mppi` (robot-infra, 2026-11-03; `estimator_backend: reduced_filter`): the same owner, settings and commit-triggered planning behind the existing heartbeat, arming, gates and fault handling; not yet run on hardware | `catheter_control/node.py` (`_reduced_owner_tick`, reduced branches of `_plan_tick`), `catheter_control/orchestration/reduced_filter_settings.py`; see the [deployment gap audit](REALTIME_DEPLOYMENT_GAP_AUDIT.md#commanding-path-integration-2026-11-03) |
| Non-commanding ROS dry-run controller (robot-infra, 2026-11-02): filter ownership, plan roots, 5 Hz `CatheterMppi`. Since 2026-11-03, planning is commit-triggered: it runs in the owner callback right after a filter commit when `plan_due` (at least 0.9 × the plan period since the last plan), never on an independent timer | `catheter_control/orchestration/reduced_filter_owner.py`, `catheter_control/reduced_filter_dry_run.py`; see the [deployment gap audit](REALTIME_DEPLOYMENT_GAP_AUDIT.md#step-4-results-2026-11-02-non-commanding-dry-run-controller) and its [commit-triggered planning results](REALTIME_DEPLOYMENT_GAP_AUDIT.md#commit-triggered-planning-pattern-1-2026-11-03) |

The array-native conditional shadow (`batched_interface_step.py` and its
completion/transport stages) was archived on 2026-11-02, superseded by the
reduced filter. Update affected equations,flow and status when implementation
changes;put experiment narratives elsewhere.

[Pre-cleanup reference and historical evidence](../../../repository_archives/cleanup_20261101/history/robot-infra/audits/ros-realtime/GATE3_FILTER_PIPELINE_REFERENCE.md)
are preserved outside the repositories. The
[2026-11-02 archive](../../../repository_archives/cleanup_20261102/README.md)
holds the later experiment records and legacy research code.
