# Batched MPPI forecast for the reduced filter: plan

2026-11-02. **Approved 2026-11-02: 30 ms budget, steps 3a and 3b, and 3e
alongside 3d. 3a has run and found a blocker; see section 7. No code is
changed yet.** Offline
and non-actuating throughout: no planner selection, no configuration change,
no device link. This is step 3 of the shadow integration in
[GATE3_REDUCED_FILTER_SHADOW.md](GATE3_REDUCED_FILTER_SHADOW.md).

## 1. Goal

Give MPPI a tip forecast from the reduced filter for every candidate. The
forecast must be fast enough to sit inside the planner deadline. It must also
be the same frozen-root forecast that was scored at about 1.03 mm overall on
the whole session, not a new predictor.

The October planner (`engagement_twoaxis.yaml`) scores 512 candidates over
4 coarse steps of 0.2 s, so 2,048 tip points per plan. It has a 60 ms planning
deadline, with rotation velocity set to zero.

## 2. The forecast being batched, unchanged

`forecast_native_laws` in `cr_meta_lnn/training/hybrid/reduced_law_filter.py`
computes, from a committed image state at root time 0, for a future time t:

$$
\hat p_T(t)=\frac{\sum_L\sum_j w_{Lj}\big[p_L+b_L\big(\Delta m_j(t)+\alpha_L\Delta\kappa(t)\big)+j_L\,\Delta\bar\lambda(t)\big]}{\sum_L\sum_j w_{Lj}}
$$

- $w_{Lj}$: memory-cell masses of law L above $10^{-6}$. The
  $p_L,b_L,\alpha_L,j_L$ are law means, frozen. There is no learning and no
  image.
- $\Delta m_j(t)$: the existing fixed-width play clip (7.05 mm) applied to
  cell j along the chassis driver path.
- $\Delta\kappa(t)$: the knob driver change.
- $\Delta\bar\lambda(t)$: the mean lambda change from a private clone of the
  distal owner. That clone runs `_native_prefix` over the native-rate tendon
  path, then `_advance_without_image`.

That makes it a mean forecast only. Batching must not add uncertainty
scenarios, a command-to-motion model or any other mechanism.

### Where the time goes now

Two recorded roots (images 646 and 818, K=2, 129 cells per law), median per
forecast point:

| Part | ms |
| --- | ---: |
| Clone the distal owner | 0.06–0.08 |
| `_native_prefix` over 11–71 samples | 0.14–0.16 |
| `_advance_without_image`: full Gaussian response, variance and profile cross | **3.2–3.5** |
| Whole `forecast_native_laws` | 3.7–4.0 |

At about 3.7 ms per point, one plan of 2,048 points takes about 7.6 s. The
forecast reads only `branch.state.mean[0]`, yet it pays for the full response
covariance. The chassis and knob part costs under 0.5 ms per point.

## 3. Planner contract

The adapter implements the existing backend call:

`predict_control_sequence(root_state, motor_velocity_sequence[N,H,3] rad/s, dt) -> tip_base_m[N,H,3]`

It also returns optional `distal_lambda[N,H]`. The adapter would live in
`cr_meta_lnn`, and the planner in `robot-infra` would stay unchanged.

**Root.** The last committed image state, advanced over the **measured**
encoder samples since that image up to plan time. This uses the existing
measured-prefix input of the forecast. Commands never stand in for measured
motion.

**Future inputs.** Each candidate's commanded shaft rates are integrated
piecewise-constant over each step from the measured plan-time shaft angles.
They are then mapped with the same affine conversions the filter uses for
measured data:

- tendon = logical axis 2 of `counts_to_logical`;
- chassis/knob = `transmission.driver(shaft)*1000` with the v175 artifact.

The tendon path is sampled at the recorded encoder period, because the
direction detector consumes native-rate samples.

**Labeling.** These are commanded-input forecasts. They are kept separate from
achieved-input forecasts in every report.

**Fail closed.** The adapter raises, and the planner's existing rollout-failure
path rejects the plan, when:
- estimator health is `holding` or `fault`;
- a rotation rate is nonzero;
- an input is nonfinite;
- a law gain is not positive;
- the root is older than a declared age.

**Play is counted once.** The filter already carries chassis play and tendon
engagement. Planner-side play rollouts (`transmission_aware_rollout`,
`raw_motor_velocity_sequence`, `always_engaged`) must not be combined with this
backend. The adapter rejects those options rather than silently ignoring them.
How the planner's take-up risk cost should then be computed is a later
integration question, outside step 3.

## 4. Atomic steps

Each step has one hypothesis and a baseline chosen beforehand, and only
proceeds after the previous one passes.

All steps are scored on two short segments. Each starts from a fresh bootstrap
at the segment start and runs about 50 images, taking about 5–10 s.

- **Segment R: 552–604.** It contains the tendon takeup at 577, the chassis
  reversal at roots 580–592, and stationary roots.
- **Segment T: 800–850.** It contains tendon reversals and the 813/819 spikes.

Roots are every image in both segments, at every recorded image target within
0.8 s. The whole session is a later step needing separate approval.

### 3a. Mean-only distal branch

**Hypothesis.** The owner's existing mean-only cursor (`_evaluation_mean_only`)
gives the same $\bar\lambda(t)$ as the full Gaussian advance, at a small
fraction of the cost.

**Change.** The forecast clone uses that cursor. No equation changes. If the
cursor refuses a branch, for example a return frontier, the forecast reports
it and does not fall back.

**Pass.**
- $|\Delta\bar\lambda|\le 10^{-9}$ and $|\Delta\hat p|\le 10^{-9}$ mm against
  current `forecast_native_laws` at every root and target, using achieved
  encoders.
- Engagement log-weights match.
- Report the cost per point.

**If it fails:** report where the means differ. Do not use mean-only.

### 3b. Exact batched forecaster

**Change.** A new function in `reduced_law_filter.py` that takes N input
schedules and H targets.
- **Chassis and knob:** vectorized over candidates, steps and cells. Within a
  constant-rate step the driver path is monotone, so clipping at the step
  endpoint equals clipping every native sample. That makes it exact, and a
  test checks it.
- **Lambda:** one mean-only branch per candidate. Run the prefix to $t_1$,
  clone, advance; continue the prefix to $t_2$ from the un-advanced branch, and
  so on. This is the same as fresh per-target branches but avoids replaying
  the shared prefix H times.

**Pass.**
- Equal to looping the 3a forecast within $10^{-9}$ mm on the recorded
  achieved schedules.
- Equal again on synthetic commanded schedules: 64 per root, covering hold,
  each direction, and reversal at each step.
- Report the P50/P99 time for 512×4 on CPU.

**Expected result.** This probably misses the budget even if 3a gives a ten-
to fiftyfold speedup. 2,048 Python owner calls at 50–300 µs is about
0.1–0.6 s. If it does miss, it remains the exact reference for 3c.

### 3c. Tabulated lambda, only if 3b misses the budget (an approximation, separately authorized)

**Hypothesis.** Along a tendon path that is monotone from the root, the mean
$\bar\lambda$ at the endpoint depends only on the endpoint input, not on speed.

**Change.** For each direction, once per plan, sweep the existing mean-only
owner along an input grid covering the reachable travel: at most 4.5 mm/s ×
0.8 s = 3.6 mm. Monotone candidates read $\bar\lambda$ by linear
interpolation.

**Reversing candidates.** Candidates that reverse tendon direction inside the
horizon use exact 3b evaluation. Their count is reported each plan.

**Before building:**
1. Test speed independence directly: endpoint means from 3 speeds must agree
   within $10^{-9}$.
2. Measure the reversing-candidate fraction from the existing October sampler
   with its fixed seed.

**Pass, with tolerance declared now.**
- The grid is 0.01 mm.
- Against 3b: the tip forecast differs by at most 0.02 mm, and the lambda
  mean by at most 0.02 of its own units, at every root, target and candidate.
- The P99 time for 512×4 meets the budget.

### 3d. Planner adapter and offline timing

**Change.** The `predict_control_sequence` adapter from section 3, with unit
tests for units, frames (mm to m, robot_base), root composition, rejected
options and each fail-closed case.

**Timing.** In `compute_profile`-style recorded planner replay, with the
filter and planner in separate processes as in E6, non-actuating and without
ROS. Report the forecast, planner and filter P50/P95/P99/maximum and any
deadline misses. This is not live timing.

### 3e. Commanded versus achieved, diagnostic only

The recorded control session
`engagement_directional_twoaxis_..._20261008T213127347675Z` is the same recording the
filter replays. It contains `/catheter_mppi/planned_control`,
`/manager/control` and `/device/state`.

For each recorded executed plan, forecast the tip twice from the same root:
1. with the commanded sequence;
2. with the achieved encoders over the same window.

Score both against the marker readout, and report their difference as the
execution gap. Report it separately for these situations: held, chassis
moving, tendon moving, and reversals or takeup.

This is a diagnostic. A large gap would argue for a command-to-motion model,
which is a new mechanism and needs its own proposal. It is not added here.

## 5. What stays unchanged

- The filter update, the laws and K=2.
- Physical gain, the 7.05 mm width and the stationary hold.
- The scalar reference, the October planner and its configuration.
- The manager, firmware and safety gates.

No forecast trains anything. Selecting this backend in a planner, and live
observation (step 4), both need separate authorization.

## 6. Decisions needed

1. **Budget:** forecast P99 for 512×4. Proposed: at most 30 ms on CPU. That
   keeps the planner inside its 60 ms deadline, and with the estimator's
   50 ms keeps the cycle well under 200 ms at 5 Hz.
2. **Approach:** approve steps 3a and 3b now. Step 3c, being an
   approximation, needs separate approval after their results.
3. **Diagnostic:** whether step 3e runs alongside 3d or waits until after it.

## 7. Step 3a results, 2026-11-02

Read-only probes. Scripts are in the session scratchpad; no repository code
changed.

**Recorded schedules pass.** On segments R and T (102 roots, 773 forecast
points at image targets), the owner's mean-only cursor matches the full route:
- lambda within 7e-15;
- tip within 4.5e-15 mm;
- identical engagement probability.

It costs 1.04 ms per point against 3.05 ms, about 3 times faster. The
recorded horizons contain no tendon reversal.

**Planned inputs need query views.** The mean-only cursor requires both the
current and target tendon inputs to lie on the profile's declared input grid.
That grid comes from `gate3_compact_cycle_oracle_prior_20261025` and has 485
irregular points. It contains every image's tendon value in this recording and
all 6 recorded turning points, but only 73% of native samples.

The owner's existing route for arbitrary inputs is `forecast_native_mean_path`.
It adds the queried inputs as spline evaluation rows without refitting. On 8
synthetic schedules from every third root in R and T, including reversals
inside the horizon, it equals the full route within 2.1e-14 at all 743 points
where the full route runs. Two failure classes:

- The full route raises at off-grid reversal points (65 cases). The view route
  handles them, so the view route is the reference for planned inputs.
- Inputs outside the profile support of 0.0009 to 10.77 mm are rejected by
  both routes (62 cases). There is no extrapolation. Candidates leaving the
  support must be refused, not clamped.

The view route costs about 1.9 ms per target from a frozen root. That puts
exact 3b at roughly 4 s for 512×4, as expected.

**Blocker: the filter only accepts on-grid tendon inputs.** Shifting every
recorded tendon value in segment R by 1e-6 mm gives `holding` at images 553
and 554, then `fault` at 555, all with "Query must be on the exact declared
input grid". This holds for both the reduced filter and the scalar reference,
which share the distal owner.

So the filter has only ever run on the one recording whose inputs its prior
grid already contains. Any live data (step 4) or other recording would fault
within three images.

This is an unresolved issue that comes before 3b and before step 4. Next step,
not yet authorized: a read-only diagnosis of which owner operations need exact
grid indices on the image path. Then a separately proposed repair that
preserves the profile model's behavior on this recording.

## 8. Diagnosis: why the filter needs on-grid tendon inputs, 2026-11-02

Read-only. Exact-grid lookups were enumerated with a scratch-only wrapper.
Off-grid queries were logged and snapped to the nearest node so the run could
continue. Snapped numbers are discarded; snapping is not a repair. Segments R
and T were run with every tendon value shifted by 1e-6 mm.

### Sites hit on the image path

| Site | What the grid position is used for | Calls R / T, all off-grid when shifted |
| --- | --- | ---: |
| `single_soft_engagement._repaired_gp_deformation_step:760` | Index `b` of the current input. Used for the spline basis row `grid_evaluation[b]` (profile readout and anchored slope projection) and for `last_index=b` | 52 / 50 |
| `distal_deformation_runtime._observe:438` | Hard cursor index at the engaged frontier, set every engaged image | 41 / 46 |
| `distal_deformation_runtime._observe:433` | Hard cursor index at a new confirmation | 1 / 1 |

Not exercised by R or T, but present in the code:
- the return-frontier transitions (`_known_return_prediction` 172/188 and
  `_observe` 424, through `conditional_play_transition`);
- the dense visit opening (`VisitProfileTransfer.open_local_value`), used only
  when a visit opens without the shared visit prior;
- the initialization guard (`initialize` 134);
- the mean-only forecast (`predict_interpolated_gp_deformation_mean`, at both
  ends).

The native prefix and the engagement detector use no grid lookups.

### What the grid actually does

For the active compact spline profile (22 coefficients, 20 breakpoints), the
profile is a B-spline over fixed knots. The 485-node grid plays four roles:

1. **Readout rows.** The index only selects the basis row at the input.
   Evaluating the basis directly at a grid node reproduces every stored row
   **bit for bit** in both directions. So direct evaluation at the actual
   input is the same mathematics, and identical on this recording.
2. **Slope-constraint locations.** The minimum-gain inequalities are imposed
   between consecutive grid nodes. Those nodes are this recording's inputs.
3. **Hard cursor and `last_index`.** These are order and backtrack
   bookkeeping. `UniformSplineProfile.condition` already uses the
   `searchsorted` position for off-grid queries, so a precedent exists.
4. **Dense openings.** The local-value kernel is defined on grid values.

The support also comes from the grid: [0.000893, 10.766078] mm in each
direction. The recording's tendon range is 0.001042 to 10.766078 mm. A tendon
value of exactly 0, the logical home position, is outside the support, and so
is any value beyond the recorded maximum.

### Candidate repair, not authorized and not implemented

**R1: evaluate at the input instead of looking up a grid row.**
- Readout rows: evaluate the basis at the input instead of `grid_evaluation[index]`.
- `last_index`: store the `searchsorted` position.
- Hard cursor: carry the oriented input rather than requiring a node.
- Mean-only forecast: evaluate its two endpoints directly.
- The grid stays as the slope-constraint locations and the dense-opening domain.

Matched checks:
1. Bitwise-identical outputs on segments R and T for both the reduced filter
   and the scalar reference.
2. With the 1e-6 mm shift, the filter tracks every image. The change in
   lambda and tip should be of order the shift times the local slope.
3. Every image-path site above is covered by a test.

Separate issues R1 does not solve, each needing its own discussion:
- **Support range.** Live tendon values outside it, including home at 0, are
  still rejected. Widening the support changes the spline knots, and so the
  model.
- **Session-specific constraint locations.** The slope constraint stays tied to
  this recording's inputs.
- **Dense openings.** They need grid nodes. Whether the active configuration
  ever opens a visit this way was not established. A whole-session enumeration,
  about 1–1.5 min, would show it.

## 9. R1 results: spline profiles accept any supported tendon input, 2026-11-02

Authorized and implemented in `cr_meta_lnn/training/hybrid`. Uncommitted.

**Whole-session enumeration first.** Images 3–1224 with every tendon value
shifted by −1e-6 mm (snapping wrapper, scratch only). The image path uses
exactly four exact-grid sites:

| Site | Calls |
| --- | ---: |
| Image update | 1,221 |
| Engaged-frontier cursor (`_observe` 438) | 901 |
| Confirmation cursor (`_observe` 433) | 7 |
| Confirmation frontier (`_observe` 424) | 6 |

The dense opening and the return-frontier transitions are never reached. A
+1e-6 mm shift instead faults at image 620. There the tendon reaches the
recording's maximum, which is the support edge, so the shift pushes it out of
support. That independently confirms the support limit.

**Change.**

`spline_profile_gaussian.py` gains:
- `UniformSplineProfile._position`: support check plus insertion position;
  equal to `_index` on a node.
- `readout_row`: the stored row on a node, else direct basis evaluation.
- `profile_position` and `profile_value_at`: dense owners still require exact
  nodes.
- `profile_gain_projection(fixed_current_input=...)`.

Changed callers:
- the repaired image step (`single_soft_engagement`) and the mean-only forecast
  predictor;
- the hard cursor, which now carries `frontier_input_mm`;
- `confirmation_transition` and `conditional_play_transition`, which read the
  frontier input when it is present;
- the forecast query-view remap and the initialization support check.

No equation, setting, grid, constraint location or dense owner changed.

**Checks, all passed.**

| Check | Result |
| --- | --- |
| Reduced filter, segments R and T | Bitwise identical: all states, 773 forecasts, confirmation images |
| Reduced filter, whole session | Bitwise identical: 1,221 images, all 7 confirmations (78, 565, 664, 765, 806, 934, 980) |
| Scalar reference, images 559–565 and 800–806 | Bitwise identical |
| Segments R and T with ±1e-6 mm shift | Every image tracks, same confirmation images; forecasts differ by at most 4.2e-6 mm |

Under the shift, distal and law means move by at most 1e-6, model-coordinate
covariances by at most 4e-8 relative, and profile statistics by at most 4e-7
relative. Raw law covariances differ only by sign flips in the retained
marker-noise coordinates. That basis is defined only up to sign, and
magnitudes agree within 1.5e-7 relative. A 1e-6 mm chassis shift, which
touches no grid code, moves law covariances by 6.5e-8.

**Tests.**
- New: 8 tests in `tests/test_off_grid_tendon_inputs.py`. They cover node
  identity, off-node evaluation, support rejection, the dense-owner limit,
  projection equivalence, the off-grid image step, the frontier cursor and the
  play transition.
- The name clash with the older joint estimator's `input_mm` field was found
  by `test_frozen_joint_play` and fixed by naming the cursor field
  `frontier_input_mm`.
- Related suites pass: 179 distal, spline and frozen-forecast tests; 66
  real-time, scalar and bootstrap tests; 150 October deployment, engagement and
  owner tests.
- `robot-infra` tests that use `cr_meta_lnn`: 157 passed, with the same
  single `TipReferenceHorizon` bag failure as before.
- Two failures in `test_soft_deformation_episode.py` already failed on the
  committed code. They are unrelated: episode origin 0.2, expected 0.

**Still unresolved.**
1. **Support range.** Tendon values outside [0.000893, 10.766078] mm,
   including home at 0, are rejected. Widening the support changes the spline
   knots, and so the model.
2. **Constraint locations.** The slope constraint stays at this recording's
   input values.
3. **Dense openings.** These still need grid nodes. They are not reached with
   the shared visit prior.
4. **Whole-session off-grid run with R1.** Not run; the segments, scalar
   windows and tests are the evidence.

Live observation (step 4) remains blocked by item 1.

## 10. Proposal: widen the tendon profile support, 2026-11-02

Requested by the user: tendon inputs slightly outside the identified range, for
example overshoot when homing or at an upper limit, must not raise. This is a
model change: it defines the profile where no data exist. **Not implemented.**

### Facts

- Profile support from the prior artifact: [0.000893, 10.766078] mm, both
  directions. That is the 2026-10-08 recording's range.
- The recording sits within 0.05 mm of home for about 5,000 native samples from
  image 96 onward. It reaches its maximum around images 620–628.
- Active limits (`imricor_test`): tendon commands are clamped to [0, 15] mm,
  with a 0.002 mm feedback tolerance at the hard limits. The v171 encoder
  validity window corresponds to about [−1.49, 17.86] mm, and the controller
  aims to stop 1000 counts (0.149 mm) inside it.
- So 10.77–15 mm is operable but has no profile data. That is extrapolation,
  not overshoot.

### Options

**A. C1 linear continuation of the same spline (recommended).** Let the
oriented support be [a, b], with basis row B(x) and derivative row B'(x):

$$
\tilde B(x)=\begin{cases}B(a)+(x-a)\,B'(a) & a-m\le x<a\\ B(x) & a\le x\le b\\ B(b)+(x-b)\,B'(b) & b<x\le b+m\end{cases}
\qquad f(x)=\tilde B(x)\beta .
$$

- Same coefficients and no refit: everything inside [a, b] stays bitwise
  identical.
- Value and slope are continuous at the ends.
- It is linear in beta, so means, covariances and cross terms reuse the
  existing row machinery. Variance grows with distance through the variance of
  the end derivative.
- Beyond the margin m the filter still fails closed, because such an input
  means a fault or a real excursion the model has no basis for.

**B. Rebuild the spline on a declared wider support, such as [0 − m, 15 + m].**
This needs new knots, a new compressed prior and a GP prior for the unobserved
region. It changes the model inside the data range, so it is not bitwise, and
it needs a new prior artifact and forecast re-validation. It is the right
route for routine operation above 10.77 mm, but that needs data there.

**C. Clamp the input to the support.** That means zero response beyond the
support: lambda freezes during upper overshoot. Not recommended.

### Decisions needed for option A

1. **Margin m.** Proposed 0.5 mm on each end, about 3,400 counts. That covers
   overshoot with a wide reserve but does not reach 15 mm. Wider means more
   linear extrapolation.
2. **Profile learning while the input is beyond [a, b].** Proposed: withhold
   profile learning there, the same way the stationary hold does. Lambda,
   engagement and tip-J learning stay unchanged, so transient overshoot cannot
   rewrite the identified end slope. The alternative lets the extended rows
   learn; that also moves end coefficients the data range shares.
3. **Minimum-slope constraint.** The end derivative is not constrained, so the
   continuation slope could fall below the minimum gain. Proposed: no new
   constraint, since adding one would change in-range results; audit the end
   slopes instead.

### Planned checks (short segments first, about 1–2 min in total)

- Bitwise identity inside the support: segments R and T, the scalar windows,
  then the whole session, about 50 s.
- **Overshoot injection, upper end.** Images 610–635: add a smooth bump to
  tendon samples above 10.70 mm so the path reaches 10.766 + δ for
  δ ∈ {0.002, 0.05, 0.2} mm.
- **Overshoot injection, home end.** Images 90–115: dip samples below 0.05 mm
  to −δ.
- **Pass criteria:**
  - every image tracks, with no fault;
  - lambda, tip and forecasts are continuous in δ, with differences from the
    uninjected run shrinking as δ goes to 0;
  - the profile stays unchanged while beyond [a, b], if decision 2 is accepted;
  - inputs beyond m are rejected.
- **Unit tests:**
  - value and slope continuity at both ends;
  - rows linear in beta;
  - covariance consistency;
  - rejection beyond the margin;
  - dense owners unchanged.

## 11. Support extension implemented, 2026-11-02

**User decision.** The spline support should always cover the joint-limit
range plus a small margin. For now, keep the identified range
a = 0.000893 mm and b = 10.766078 mm and continue it 0.5 mm beyond each end.
Data covering the full joint range will be collected after end-to-end
real-time performance is verified.

The two unanswered choices from section 10 use the proposed defaults:
- profile learning is withheld beyond [a, b];
- no new slope constraint (end slopes to be audited).

**Change** (`cr_meta_lnn/training/hybrid`, uncommitted):

- `UniformSplineProfile(support_extension_mm=...)` keeps `data_support`
  fixed at construction. `evaluate` returns the C1 linear continuation
  B(end) + (x − end)·B'(end) on [a − m, a) and (b, b + m], and rejects inputs
  beyond. The data-support branch is the unchanged design-matrix call.
  `support` and `inside_data_support` are exposed. Openings and the runtime
  compression pass the setting through.
- `DistalDeformationRuntime(profile_support_extension_mm=0.)` is validated and
  requires the compact spline backend. `load_native_artifacts`,
  `probe_real_native_bootstrap`, `load_stream`
  (`REALTIME_PROFILE_SUPPORT_EXTENSION_MM=0.5`) and
  `scripts/reduced_filter_session.py` (`--profile-support-extension-mm`,
  default 0.5) forward it.
  - The real-time path and its ROS observer select 0.5 mm.
  - The scalar reference keeps 0 by default and is unchanged.
- **Learning gate.** In the repaired image step, an admitted image whose
  input lies beyond the data support withholds profile learning, with audit
  `profile_outside_data_support`. Lambda, engagement and tip-J deformation
  learning stay unchanged.
- **Visit-prior kernel.** `anchored_persistent_gain_covariance_at` accepts
  origins inside a caller-declared `origin_support`. It inserts the same
  integrated-gain law at either grid end. Grid origins keep the identical code
  path.
- The mean-only onset-quadrature bound uses the spline support.

**Checks.**

| Check | Result |
| --- | --- |
| Extension on, segments R and T | Bitwise identical to the pre-R1 baseline, including 773 forecasts |
| Extension on, whole session | Bitwise identical: 1,221 images, 7 confirmations |
| Scalar windows, extension off | Bitwise identical |
| Overshoot injection, images 610–635 (upper) and 90–115 (home), δ = 0.002, 0.05, 0.2 mm | Every image tracks and every forecast is produced |
| Overshoot injection, δ = 0.6 mm (beyond the margin) | `holding`, then `fault`, at the first out-of-support images (621/622 upper, 97/98 home); out-of-support forecasts refused |

In the injections, the identified profile equals the uninjected run's at
every image. The only profile change while beyond b, at image 628, is the
reversal switch to the other direction's bank, also present without injection.

Differences from the uninjected run:

| End | δ = 0.002 mm | δ = 0.05 mm | δ = 0.2 mm |
| --- | --- | --- | --- |
| Home: tip, forecast | 1e-4, 1e-4 mm | 0.004, 0.004 mm | 0.008, 0.009 mm |
| Upper: tip, forecast | 0.009, 0.009 mm | 0.11, 0.67 mm | 0.17, 0.54 mm |

The upper differences shrink as δ → 0. Larger δ changes the input path
itself: engaged travel at the top increases (engaged probability 0.917 → 0.998
→ 1.0), and the reversal point moves by δ. Forecasts crossing the reversal and
re-engagement therefore differ. This is likely a real response to a different
input, not instability, but the mechanism was not traced in detail.

**Tests.**
- `tests/test_off_grid_tendon_inputs.py` now has 13 tests. The new ones
  cover:
  - value and slope continuity and linearity at both ends;
  - bitwise data-support rows;
  - rejection beyond the margin and parameter validation;
  - the learning gate (admitted at 0.58, withheld at 0.62 with data end 0.6);
  - the visit-prior kernel;
  - the planned mean path on the continuation against the full forecast.
- In the session injections the upper end was held still, so the stationary
  hold also applied; the gate itself is tested directly.
- Suites: 186 distal, spline and frozen tests (the same 2 old failures), 66
  real-time, 150 October, and `robot-infra` 157 passed with the same single
  bag failure.

**Still open.**
- **Slope constraint.** No minimum-slope constraint applies on the
  continuation; end slopes are not yet audited.
- **Dense openings.** These remain exact-node only and are not reached with
  the shared visit prior.
- **Fault on out-of-support input.** Beyond the 0.5 mm margin the estimator
  faults and needs re-initialization. The planner adapter must refuse
  candidates that leave the support.
- **Full joint range.** Coverage of 0–15 mm and beyond needs the planned new
  data and a rebuilt profile (option B).

## 12. Steps 3a and 3b implemented, 2026-11-02

Uncommitted, in `cr_meta_lnn/training/hybrid`.

**3a.** `forecast_native_laws` reads lambda from the distal owner's mean-only
query route. The new `DistalDeformationRuntime.forecast_native_mean_at(ns, u,
indices)` uses the same setup as `forecast_native_mean_path`, which now calls
it. The full Gaussian advance is no longer used for forecasts. Against the
pre-change forecasts on segments R and T (773 forecast points), the largest
difference is 2.1e-14 mm, and filter states are bitwise identical. The
segment replays with forecasts at every root drop from about 20 s to 5 s.

**3b.** `forecast_native_laws_batch(laws, runtime, ns[S], tendon[N,S],
chassis[N,S], knob[N,S], targets[H]) -> [N,H,3]`.
- Chassis play steps every native sample for all candidates and cells at once,
  using `EnvelopeHistory.step` as before.
- The knob term is vectorized.
- Lambda is one mean-only query path per candidate, evaluated only at the
  targets.

Checks against looping `forecast_native_laws` (declared tolerance 1e-9 mm):

| Segment | Recorded schedules | 64 synthetic schedules per third root |
| --- | --- | --- |
| R | 52 roots, max 3.1e-13 mm | 17 roots, max 3.4e-13 mm |
| T | 50 roots, max 2.8e-14 mm | 16 roots, max 9.9e-14 mm |

The synthetic schedules cover hold, each direction, reversals, chassis
±10 mm/s and knob ±2 mm/s. Candidates leaving the extended support were
refused by the looped reference: 110 on R and 147 on T.

New unit test `test_batched_forecast_equals_one_schedule_at_a_time`. Suites:
102 distal, real-time and off-grid tests, 27 reduced-filter tests and 15
`robot-infra` shadow-contract tests pass.

**Timing.** 512 candidates × 4 targets at root 555 take **5.0 s**, against the
30 ms budget, as predicted. A 64-candidate profile puts about 14 ms per
candidate in the distal mean route, spread across roughly 24,000 Python calls
per candidate:
- onset quadrature and prediction construction in `_advance_without_image`,
  about 67%;
- per-sample detector stepping in `_native_prefix`, about 46% inclusive.

The chassis and knob part is negligible.

**Findings for later steps.**
- **Refusal granularity.** The batch call refuses all candidates when one
  leaves the support. The planner adapter (3d) needs per-candidate refusal.
- **Exact reversal evaluation is too slow.** At about 14 ms per candidate, 3c
  cannot afford exact evaluation for more than one or two reversing candidates
  inside 30 ms. Section 4's fallback for reversing candidates therefore needs
  revision before 3c is proposed for approval.

## 13. Proposal: batched distal mean on the GPU (step 3b-GPU), 2026-11-02

User direction: try the GPU for the 512-sample rollout before further 3c work.
**Proposal awaiting approval; not implemented.**

### Why vectorization is the work, not the device

The exact 3b path costs about 14 ms per candidate. That is Python overhead in
the scalar distal owner, not arithmetic. With `native_direction_events`, the
mean-only branch does the following per candidate and target:

1. The native prefix only finds direction changes; every nonzero sign change
   is an event.
2. At each event and at the target there is one mean-only advance
   (`predict_interpolated_gp_deformation_mean`).
3. Each event also triggers a switch (`_switch`): a new bank, a new episode
   origin, engagement logs reset to held, and a retained-return frontier if
   resuming.

With 4 constant-rate steps there are at most 4 monotone segments per
candidate.

### The recursion to batch (unchanged equations)

For a monotone segment in direction d with episode origin o, from input
u_s with mode logs ℓ_s to endpoint u, define the travels
t_s = d(u_s − o) and t = d(u − o). Then:

- **Engagement logs:** the existing survival/hazard law,

$$
\log S=\log\bar\Phi\!\Big(\tfrac{t-\mu_w}{\sigma_w}\Big)-\log\bar\Phi\!\Big(\tfrac{\max(t_s,w_{\min})-\mu_w}{\sigma_w}\Big),
$$
  with ℓ = [ℓ_s,0 + log S, logaddexp(ℓ_s,1, ℓ_s,0 + log(1 − S))],
  normalized, including the existing extreme-tail log-CDF branch.
- **Quadrature:** 32 Gauss–Legendre width nodes on
  [max(t_s, w_min), t], weighted by the truncated-normal density.
- **Mean**, with f the d-oriented bank spline mean (continuation included):

$$
\lambda=\lambda_s+d\Big[m_{\text{old}}\big(f(du)-f(du_s)\big)+m_{\text{new}}\big(f(du)-\textstyle\sum_i w_i f(d\,o+w_i^{node})\big)\Big].
$$

- **Retained return:** a known frontier adds the existing play-transition
  increment once it is crossed.

### Implementation

- A new batched owner function in `single_soft_engagement.py`, next to the
  scalar one, written in torch with float64 on CUDA. It takes a frozen root
  snapshot (state, banks, episode, frontier, cursor), candidate tendon paths
  [N,S] and targets [H]. It returns lambda [N,H] plus a per-candidate refusal
  mask (out of support, nonfinite).
- `forecast_native_laws_batch` gains a `device` option. The chassis play and
  knob terms move to torch as well.
- The scalar owner remains the executable specification; nothing is replaced.

### Checks

- **Equivalence** against the exact 3b batch, which matches the scalar owner,
  within the declared 1e-9 mm, on:
  - recorded schedules on segments R and T;
  - 512 synthetic schedules per third root, covering hold, each direction,
    reversals at each step, near-end and continuation inputs, and candidates
    leaving the support, where the refusal masks must match.
- **Targeted unit tests** for each branch: fresh episode, root continuation,
  retained return, first direction, the extreme-tail branch, and refusal.
- **Timing** for 512×4, after warm-up, over at least 200 plans, P50/P99:
  float64 on CUDA, float64 CPU-vectorized torch or numpy, and float32 on CUDA
  reported separately as information only. Budget: P99 ≤ 30 ms including
  transfers. The planner is not involved; this is not live timing.

### Expected effort and risk

This reimplements in arrays the event logic of `_native_prefix`, `_switch`,
`_known_return_prediction` and the soft episode. That is the largest fidelity
risk, and equivalence on reversing schedules is the gate. If equivalence fails
on a branch, report it and do not approximate; 3c stays the fallback.

Test runtime is about 1–2 min for both segments.

### Revision after reviewing the October GPU implementation

The first draft above did not consult the October GPU work. It now builds on
it.

**What October established.**
- `cr_meta_lnn/deployment/control_accelerator.py` (O3, commit `28ff41a`):
  - `FourStepRollout` is a fixed-shape four-step tensor recurrence that reuses
    the authoritative owners (transmission, history step, implicit mechanics,
    FK) instead of copying equations.
  - `rollout_inputs` validates outside the kernel.
  - `PreparedTensorKernel` gives owned buffers, warm-up, CUDA-graph capture
    and an error on signature change, never lazy compilation or fallback.
    Errors reach MPPI's fail-closed rollout path.
  - Measured on the RTX 4090 in float32: public rollout at 512×3 had P99
    3.33 ms; the grouped planner at 512×1 had P99 8.10 ms. Graph preparation
    takes about 80–109 ms and stays outside the deadline. Inductor failed and
    was not used.
  - Grouped sampling scores 518 actual candidates for 512 stochastic samples.
- **Owner tensor methods.** The engaged-profile priors expose
  `advance_tensor` and `advance_sequence_tensor` next to their NumPy
  `advance` (`networks/hybrid/engaged_profile.py`). The batched path lives in
  the owner, not in a parallel model.
- **Precision boundary (O2).** The estimator stays float64 on the CPU; the
  planner rolls out in float32 on CUDA from a snapshot cast at plan time
  (`clone_to`). The rollout is accepted against the float64 reference with
  `rtol=2e-5, atol=2e-7 m`.
- **Modeling difference.** The October engaged rollout was always-engaged:
  an external takeup arbiter owned slack. The Gate 3 forecast instead carries
  probabilistic takeup episodes inside the forecast. Batching keeps the Gate 3
  semantics. Adopting October's split would be a separate behavior change.

**Revised plan.**
1. **Owner tensor methods, not a new module.**
   - `UniformSplineProfile.evaluate_tensor`: cubic B-spline rows via
     precomputed per-interval polynomial coefficients, plus the linear
     continuation.
   - A tensor form of `_width_interval_log_survival` and the quadrature
     weights, using `torch.special.log_ndtr` with the same extreme-tail branch.
   - The batched mean-only segment recursion next to
     `predict_interpolated_gp_deformation_mean`.
   - The scalar functions stay the specification.
2. **Rollout object.** A Gate 3 four-step forecast object with the
   `FourStepRollout` structure. Inputs are a frozen root snapshot (filter laws
   and distal root, cast once per plan) and candidate rates [N,4,3]. Outputs
   are tips [N,4,3], lambda and a per-candidate refusal mask. Validation stays
   outside the kernel.
3. **Execution.** Reuse `PreparedTensorKernel` unchanged, with eager and
   CUDA-graph modes and a fixed shape of 518 candidates. Timing follows the
   O3 `benchmark_accelerator` protocol and is compared with its numbers.
4. **Two-tier checks.**
   - float64 CUDA against the exact 3b batch within 1e-9 mm, for correctness.
   - float32 CUDA against float64 within the O2 rollout tolerance
     (`rtol=2e-5, atol=2e-7 m`), for the deployment precision.
5. Schedules, branch unit tests, budget and the stop-on-failure rule are as
   above.

## 14. Step 3b-GPU implemented, 2026-11-02

Approved as revised in section 13. Uncommitted, in `cr_meta_lnn/training/hybrid`.

### Owners

- `spline_profile_gaussian.SplineMeanTensor` and `spline_mean_values`: a
  frozen profile mean as torch per-interval cubics, from scipy
  `PPoly.from_spline`, with the same linear continuation.
- `current_engagement_probability.engagement_log_transition_tensor`: a
  batched form of `predict_engagement_log_probabilities`, including the
  extreme-tail branch.
- `distal_deformation_runtime.mean_forecast_snapshot` and
  `four_step_mean_forecast`: the scalar mean-only branch, evaluated for
  piecewise-constant tendon rates. It covers:
  - native direction events, which happen only at step starts;
  - `_switch`, including the first-direction origin, the resume frontier and
    the per-direction widths;
  - `_known_return_prediction`;
  - the soft-episode onset bookkeeping;
  - frontier updates while the root is engaged.

  Predictions use the episode direction. A per-candidate refusal mask covers
  inputs outside the extended support and an inconsistent prior.
- `reduced_law_filter.reduced_forecast_snapshot` and `ReducedFourStepForecast`:
  the `FourStepRollout` pattern. Inputs are filter-coordinate rates [N,4,3]
  (tendon, chassis and knob in mm/s) and dt [4]; outputs are tips [N,4,3]
  (NaN where refused), lambda and the mask. Chassis play is clipped at step
  ends. That equals per-sample clipping for constant rates when root cells are
  inside the play band; this held at every tested root (0 violations).
- `deployment.control_accelerator.PreparedTensorKernel` is reused unchanged.

### Equivalence

Every third root in segments E (images 3–60, where the root has no direction
and no retained direction), R and T was tested with 64 synthetic schedules
each. Tendon rates were up to ±4.5 mm/s, chassis ±10 and knob ±2, with 15%
zero steps and the fixed hold, reversal and alternating schedules.

| Check | E | R | T |
| --- | ---: | ---: | ---: |
| Distal lambda vs scalar `forecast_native_mean_at` (CPU and CUDA float64) | 2.5e-14 | 4.3e-14 | 2.8e-14 |
| Points / scalar refusals | 4,256 / 152 | 3,964 / 97 | 3,596 / 125 |
| Full tips vs exact `forecast_native_laws_batch`, float64, mm | 1.4e-14 | 4.3e-14 | 2.1e-14 |
| float32 CUDA vs float64, max mm (tolerance 2e-4 + 2e-5·abs) | 7.5e-6 | 1.9e-5 | 1.2e-5 |

All refusal masks match, and there are no float32 tolerance failures.

**Not covered by the session.** No root in the session has a resuming
episode, and the scan found none. That branch is covered inside candidates
and by a constructed unit test. Recorded achieved paths are not
piecewise-constant, so they are outside this kernel's input contract; their
equivalence remains with 3b.

### Timing

518 candidates × 4 steps at root 557, RTX 4090, two CPU threads, 300 plans
after warm-up, all modes refusing 0:

| Mode | P50 ms | P95 ms | P99 ms | Max ms |
| --- | ---: | ---: | ---: | ---: |
| CPU float64, vectorized eager | 14.8 | 15.7 | 16.1 | 19.0 |
| CUDA float64, eager | 15.2 | 15.7 | 16.0 | 16.2 |
| CUDA float32, eager | 15.2 | 15.7 | 16.1 | 16.3 |
| CUDA float64, CUDA graph | 4.6 | 5.0 | 5.1 | 5.2 |
| CUDA float32, CUDA graph | 3.5 | 3.7 | 3.8 | 4.0 |

- Graph output is identical to eager, and graph preparation takes about 110 ms
  outside the deadline. Eager CUDA is launch-bound.
- The root snapshot is built on the host once per committed image: P50
  1.0 ms, P95 1.5 ms. The first, cold build reaches 119 ms and needs a warm-up.
- All modes meet the 30 ms budget, against 5.0 s for exact 3b. The CUDA graph
  is comparable to October's 3.3 ms public rollout.
- Isolated synthetic timing only: no planner, ROS or live load.

### Tests

- `tests/test_reduced_four_step_forecast.py`, 7 tests: spline table,
  engagement transition with tails, first-direction root, engaged roots in both
  directions with reversals, resuming root, and per-candidate refusal.
- `test_four_step_tensor_forecast_equals_the_exact_batch` in
  `test_reduced_law_filter.py`.
- Related suites: 160 passed; `robot-infra` shadow contract 15 passed.

### Next (3d, not yet authorized)

The planner adapter:
- convert motor rates to filter coordinates with the same transmission
  constants;
- root the forecast at plan time, using the measured encoder prefix since the
  committed image;
- warm prepared kernels per supported candidate signature;
- turn refused candidates into planner rejections.

3e, the commanded-versus-achieved diagnostic, runs alongside it. 3c is no
longer needed for the budget.

## 15. Steps 3d and 3e, 2026-11-02

Approved together. Uncommitted. Offline and non-actuating; no ROS graph, no
device link.

### 3d: planner adapter

**Plan-time root (`cr_meta_lnn`).**
- `reduced_forecast_snapshot(..., native_ns, tendon_mm, chassis_mm, knob_mm)`
  takes the measured path from the committed image to plan time.
- The distal branch consumes that prefix with the owner's mean-only native
  events (`_native_prefix`), with no extra advance at plan time.
- Memory cells are clipped along the prefix. Increments stay referenced to the
  committed image, using a separate `lam_ref` and knob offset.
- The distal snapshot carries the measured `position` and a `stale` flag. A
  turn at the first commanded step then advances to plan time, as the scalar
  `_native_prefix` would.
- Equivalence with `forecast_native_laws_batch` over prefix plus commanded
  future: within 1e-9 mm for still, moving and tendon-reversing prefixes
  (`test_plan_time_root_consumes_the_measured_prefix_exactly`).

**Backend (`reduced_filter_stream.ReducedFilterRolloutBackend`).**
- `root(stream, plan_ns)` fails closed when:
  - the estimator is holding or faulted;
  - encoder history is lost;
  - the root is older than 250 ms;
  - rotation moved since the image.
- `prepare(root, counts, step_s)` warms one `PreparedTensorKernel` per
  candidate count, before readiness.
- `predict_control_sequence` (also exposed as `_coarse` and `predict_sequence`):
  - converts motor shaft rad/s with the filter's own measured-input
    conversions: tendon mm per shaft rad from the caller's encoder
    calibration (`UNITS_PER_COUNT[2]/ENCODER_RADIANS_PER_COUNT`), and the v175
    driver scales for chassis and knob;
  - rejects nonzero rotation and any planner-side play or gain options;
  - returns `tip_base_m` in metres, NaN for refused candidates, plus a
    `refused` mask.
- It provides no `distal_lambda`, because Gate 3 lambda is not the October
  lambda. A planner with the first-step lambda guard enabled therefore fails
  closed ("engaged_gain_lambda_prediction_missing") until that semantics is
  decided.
- A float32 detour (`torch.as_tensor(float)` before the cast) was found by the
  units test and fixed.

**Planner (`robot-infra` `planning/mppi.py`).** If a prediction carries
`refused`, those candidates join `blocked_candidate`, and their tips are
replaced by the hold candidate's tips so the costs stay finite. Blocked
candidates can never be selected or weighted. A refused hold, a wrong mask
shape or gain scenarios give `invalid_rollout`. Backends without `refused`
behave as before. Tests: 2 new; `catheter_control` 770 passed, 2 skipped.
`robot-infra/install` was not rebuilt.

**Offline planner replay.**
- Setup: `CatheterMppi`, a fresh instance per plan as `compute_profile` does,
  512 samples giving 518 candidates, four coarse 0.2 s steps, the
  `imricor_test` contract with rotation disabled, an engaged backlash
  snapshot and a [1, 0, 1] lease.
- Data: the recorded session in arrival order. Roots come from the stream at
  each committed image, with plan time at the latest encoder sample (32–37 ms
  after capture). Targets come from the trial journal.
- Backend: float32 CUDA graph, prepared in about 200 ms before the loop.
  Filter and planner ran sequentially in one process.

| Segment | Plans valid | Planner P50 / P99 / max ms | Rollout P50 / P99 ms | Root build P50 / P99 ms | Refused of 518, median (max) |
| --- | --- | --- | --- | --- | --- |
| R (552–604) | 52/52 "ok" | 10.2 / 21.7 / 30.8 | 5.0 / 6.5 | 1.8 / 5.8 | 56 (59) |
| T (800–850) | 50/50 "ok" | 10.0 / 18.3 / 25.2 | 4.9 / 6.2 | 1.7 / 14.4 | 0 |

- No 60 ms deadline miss.
- Refusals on segment R come from candidates moving the tendon above the
  11.27 mm extended support, from 6–10 mm with up to 3.6 mm of travel. Full
  joint range needs the planned full-range data.
- Not yet run: the whole session, the E6-style separate-process concurrency,
  and live timing.

### 3e: commanded versus achieved (diagnostic only)

**What the recording contains.** It holds no full four-step commanded
sequence:
- `/catheter_mppi/planned_control` (779 messages) has only the first action's
  six logical velocities;
- `/catheter_mppi/response_trace` (699 messages) has the first executable
  action in motor rad/s over its 0.2 s window, plus the October model's own
  predicted and measured deltas;
- `/manager/control` has 7,860 velocity and 273 position commands.

3e was therefore run on the traced 0.2 s first actions, not 0.8 s. Only 80
traces command motion; stop-confirm-replace makes the rest holds.

**Method.** From the plan-time root at each trace start:
- the commanded forecast holds the trace's motor rates for 0.2 s (backend
  conversion);
- the achieved forecast uses the measured encoders to the due time (exact
  scalar route).

Both are scored against the marker-only tip readout at the image nearest the
due time, within ±60 ms, next to the October model's recorded endpoint error.
Two windows were run, about 15 s each.

| Window | Moving traces | Travel achieved / commanded | Execution gap median / P90 mm | Error vs readout, median: commanded / achieved / October model |
| --- | ---: | --- | --- | --- |
| A (140–180), chassis only | 33 | chassis 0.99 | 0.018 / 0.092 | 0.48 / 0.44 / 0.73 mm |
| B (555–620), tendon moving | 17 | tendon 0.71, chassis 0.38 | 0.47 / 1.28 | 0.81 / 0.47 / 0.98 mm |

Holds: 18 traces, with small gaps from residual motion after a stop (B: about
0.03 mm tendon and 0.1 mm chassis achieved under a commanded hold).

**Reading.**
- Chassis-only commands execute nearly exactly within 0.2 s.
- Tendon commands deliver about 71% of the commanded travel, so
  commanded-input forecasts overshoot where the tendon moves. The October
  model's commanded forecast was worse in both windows.
- A command-to-motion model (execution lag) would be a new mechanism and
  needs its own proposal. It is not added here.
- Limits: 50 moving traces in two windows; 0.2 s horizon only; the readout is
  not physical truth.

## 16. Dispatch check of the October command path, 2026-11-02

Read-only, recorded data only. This **corrects section 15's reading**: the
0.71 tendon ratio was not one command-execution gain.

**Data.** From the 2026-10-08 session bag:
- `/manager/control`: 7,860 `V` (six logical joint velocities) and 273 `P`
  (target plus speed) messages;
- `/device/command_tx`: 8,132 frames the serial bridge transmitted;
- `/device/state`: logical positions and raw encoders, each 13,073 samples at
  about 90 Hz, stamped at host receipt.

Only the four MPPI trial windows from `trials.jsonl` are analysed. Commands
are held from their transmit receipt. Measured velocity is the gradient of the
logical positions on a 5 ms grid.

| Check | Result |
| --- | --- |
| Frames reaching the serial link | 8,132 of 8,133; one velocity frame not transmitted (likely a stale rejection) |
| Position transactions during trials | 0 (homing only) |
| Velocity frame interval inside trials | P50 10.0, P95 11.6, P99 20.1, max 29.9 ms; no gap over 50 ms |
| Measured vs dispatched velocity, insertion | best lag 15 ms, correlation 0.99, gain 0.99 |
| Measured vs dispatched velocity, tendon | best lag 15 ms, correlation 0.985, gain 0.97 |
| Onset to 20% of commanded speed, from rest | insertion P50 22 ms (n=22); tendon P50 47 ms, max 57 ms (n=14) |
| Stop, to under 5% of speed | P50 about 12 ms, max 17 ms, both axes |
| Displacement per discrete command (command + 300 ms) | insertion median 1.00; tendon median 0.83 (P10 0.62) |

**The 80 traced 0.2 s first actions.**

| Ratio, median | Insertion | Tendon |
| --- | ---: | ---: |
| Dispatched / first action held | 1.00 | 0.98 |
| Measured / dispatched | 1.00 | 1.00 |
| Measured / first action held | 1.00 | 0.88 |

Window B's 15 tendon traces, per window:
- **Continuous 2 mm/s:** measured equals dispatched (0.399–0.402 mm against
  0.400).
- **Mid-window stops by the next plan:** dispatched falls to 0.22–0.29 mm, and
  measured follows it.
- **Starts from hold:** measured trails dispatched by about 0.1 mm, about
  50 ms at 2 mm/s.

**Conclusion.**
- **Transport is not the problem.** Commands reach the link at 100 Hz without
  meaningful loss, and the motors track dispatched velocity within about
  15 ms (host-received positions, so serial latency is included), with gain
  0.97–0.99.
- **First real effect: tendon start-up from rest.** It reaches 20% of
  commanded speed after about 47 ms, against 22 ms for insertion. That costs
  about 0.1 mm per start at 2 mm/s. A rollout that assumes instant velocity
  overshoots each start by that much.
- **Second real effect: the October planner's stop-and-go policy.** It
  repeatedly stops the tendon within its own planned actions, so the planned
  first action was often not held for 0.2 s. That is a planning-policy
  effect, not dispatch.
- **For the new pipeline.** The plan-time root uses measured encoders, so a
  late start is absorbed at the next plan. The start-up delay biases only
  commanded forecasts, by about 0.1 mm per tendon start. Each stop-start
  cycle repeats that loss.
- **Not covered.** Firmware-side timing (no firmware timestamps in
  `/device/state`) and rotation (locked). Only 14 tendon starts, 22 insertion
  starts and 80 traced windows.

## 17. Where the tendon start-up delay comes from, and why it matters, 2026-11-02

Read-only. Sources: the firmware source
(`firmware/teensy_tekceleo/teensy_tekceleo.ino`, `sendMotorCmds`,
`CoordinatedStart`) and the 2026-10-08 bag (raw encoders in `/device/state`,
transmitted frames in `/device/command_tx`).

### Firmware start path

A `V` frame carries logical joint velocities. The firmware decouples them
(`targetVel[0] -= targetVel[2]`): insertion motor = insertion − bend, because
the bend drive is coupled into the insertion screw. Each motor's speed is
rounded to an integer RPM.

If any active motor is disabled or changes direction, `sendMotorCmds`:
1. stops **all** active motors and waits 5 ms;
2. configures each active motor in turn with four acknowledged driver commands
   over its 115200-baud UART: `O0`, `L1`, direction `S` and speed `C`;
3. enables them together (`O1`, acknowledged).

Speed changes on a running motor need only `C`.

### Measured rise times

Starts whose command was held at least 200 ms, from the encoder of the
started motor:

| Motor | Commanded | n | 50% of speed | 90% of speed |
| --- | --- | ---: | ---: | ---: |
| Insertion (0) | 36 RPM (2 mm/s) | 2 | 16 ms | 16 ms |
| Insertion (0) | 71 RPM (4 mm/s) | 2 | 27 ms | 28 ms |
| Insertion (0) | 178 RPM (10 mm/s) | 1 | 24 ms | 34 ms |
| Tendon (2) | 101 RPM (2 mm/s) | 10 | 61 ms | 78 ms |

- Most tendon commands were "insertion 2, tendon 2". After decoupling, only
  motor 2 moves, so these are single-motor starts with the same firmware
  sequence as insertion-only starts.
- Motor 2 often holds near 25% of speed before reaching full speed.
- Insertion reaches a higher RPM faster. So the delay is neither the
  host/firmware transaction count nor RPM-proportional acceleration.
- It belongs to motor 2 itself: its ultrasonic drive under the tendon load,
  or its driver tuning. These recordings cannot separate the two; that needs
  a hardware characterization, which requires authorization.

### Consequence: joints do not execute the plan's proportions at starts

In a combined start (t = 56.9 s, insertion −2 and tendon +2, so motor 0 = −4):
- motor 0 reached −4 mm/s within 40 ms;
- motor 2 stayed near 0.5 mm/s until about 100 ms;
- logical insertion is motor 0 plus motor 2, so it moved at about −3.6 mm/s
  instead of −2 for roughly 80 ms.

The decoupling assumes both motors are at speed. During a tendon start the
executed joint direction therefore differs from the planned one, not only its
magnitude.

Once running, both motors follow dispatched velocity (gain 0.97–0.99, about
15 ms lag). The per-window ratios 0.71 (tendon) and 0.99 (insertion) were
against the *first* action. They also include the old planner's own mid-window
stops, which are a policy effect.

**Accuracy impact (expected, not yet quantified).** The estimator does not
drift, because roots use measured encoders. But every tendon start puts a
joint-space error into the executed path that the next plan must correct:
about −0.1 mm tendon and, when coupled, about +0.1 mm extra insertion. A
stop-and-go policy repeats it.

Commanded-input rollouts mispredict the first about 100 ms of every start.
That can bias candidate selection, and would contribute to the poor path
tracking seen in October. This is a hypothesis to test, not a demonstrated
cause.

**Options, none implemented. Each is a separate decision.**
1. **Planner policy:** avoid unnecessary tendon stops and restarts.
2. **Forecast:** model per-motor start-up dynamics. This is a new mechanism
   and needs a proposal.
3. **Firmware:** ramp the coupled insertion compensation with the measured
   motor-2 speed during starts. This is a safety-reviewed firmware change.
4. **Hardware:** characterize motor 2 starts loaded and unloaded, and in both
   directions.

## 18. Plan-to-execution latency, offline step 1 (2026-11-02)

**Approved and run (stage 1); results at the end of this section.** This follows the tendon start
results ([TENDON_MOTOR_START_TEST_PLAN.md section 8](TENDON_MOTOR_START_TEST_PLAN.md#8-results-2026-11-02-both-hardware-runs))
and the user's point that motors keep running the previous command while a new
plan is computed and dispatched.

### What is already handled and what is not

- **Handled: the camera delay.** `ReducedFilterRolloutBackend.root(stream, plan_ns)`
  starts from the last committed image and consumes the measured encoder
  history up to the newest encoder sample before planning (section 15, 3d).
  The roughly 64 ms image latency therefore does not make the actuator state
  stale; the model bridges it with measured motion.
- **Not handled: the gap after the root.** `predict_control_sequence` applies
  candidate commands from the root. In reality the previous command keeps
  running through planner compute (P99 about 22 ms offline), dispatch (about
  15 ms lag) and the firmware/motor transition, until the new command takes
  effect.

### Single hypothesis

Applying the new command sequence from the plan-time root, instead of after
the plan-to-transmit latency, causes a material part of the commanded-forecast
error. Prepending the in-flight command for the known latency removes most of
it.

The test is diagnostic. No owner changes: nothing in `cr_meta_lnn` or
`robot-infra` is edited. A scratchpad harness reuses the 3e setup:
- the image-indexed bootstrap `probe_real_native_bootstrap` with
  `learning_minimum_travel_mm=0.01` and `profile_support_extension_mm=0.5`;
- `ReducedEstimator` with K = 2;
- `reduced_forecast_snapshot` and `forecast_native_laws`;
- the 3e motor-rate conversion.

### Formulation

Recording: the 2026-10-08 session. It contains:
- the transmitted command timeline (`/device/command_tx`, held from its
  transmit stamp);
- measured encoders (`/device/state`);
- the stepped reduced filter.

The October planner's own plan times are not used.

For each command change k, transmitted at t_tx, k with new values:
- `u(t)`: the transmitted command timeline after t_tx, k, as filter rates
  (tendon, chassis, knob in mm/s). This is an oracle plan, equal to what was
  actually sent, so the test isolates timing from plan quality.
- `u⁻`: the command in force just before t_tx, k.
- `d`: a swept plan-to-transmit latency, d ∈ {20, 40, 60, 80} ms. Root time
  t_r is the newest encoder sample at or before t_tx, k − d. The root is built
  exactly as `root()` does: the latest committed image at or before t_r plus
  the measured prefix to t_r, within the 250 ms age limit.
- `x(t)`: measured filter inputs.

Input paths on [t_r, t_r + H], with H = 0.8 s:

| Path | Definition |
| --- | --- |
| achieved (reference) | `x(t)` |
| A0, the current backend | `x(t_r) + ∫ u(s + (t_tx − t_r)) ds`: the new sequence from the root |
| A_d̂, compensated | `x(t_r) + ∫ ũ ds`, with `ũ = u⁻` for s < t_r + d̂, then `u(s − d̂ + (t_tx − t_r))`. With d̂ = d this is the transmitted timeline exactly. |
| A_img, context only | New sequence from the committed image, without the measured prefix. It shows what the existing plan-time root already buys. |

Forecast `F(path)`: the tip from the same plan-time root, through the exact
route `forecast_native_laws`. Every path goes through the same route, so
differences are input-timing effects only. As agreed for forecast scoring,
every path is scored against `F(achieved)`, the forecast driven by the
physically achieved encoders, not against commands.

Metrics at h = 0.2, 0.4, 0.6 and 0.8 s after t_r:
- `e_tip(A, h) = ‖F(A)(t_r + h) − F(achieved)(t_r + h)‖` in mm;
- `e_in(A, h)` per axis = `|A(t_r + h) − x(t_r + h)|` in mm.

Events are classified by shaft and kind: start from rest, stop, running speed
change. The jitter sensitivity uses d̂ = d ± 10 ms.

### Pass/fail checks (declared before running)

| Check | Criterion | Consequence |
| --- | --- | --- |
| P1, materiality | At d = 40 ms, A0's median `e_tip(0.8 s)` ≥ 0.1 mm or P90 ≥ 0.2 mm over events that command motion | If neither holds, stop: the gap is not worth a planner change |
| P2, compensation | With d̂ = d, median and P90 `e_tip(0.8 s)` fall by ≥ 50% vs A0 for every d ≥ 40 ms; with d̂ = d ± 10 ms, by ≥ 30% | A pass supports proposing in-flight-command compensation as a separate atomic change |
| P3, consistency | With d̂ = d, the residual `e_in` at tendon starts is consistent with the characterized start loss: 0.10–0.20 mm, about 42–55 ms of dead time | A large mismatch means a harness or timing bug, investigated before any conclusion |

All classes, medians, P90, maxima, early failures and tails are reported.
Nothing is replaced with an easier subset.

### Segments, runtime and stop/resume

- **Stage 1, the short segments.** The 3e windows, each bootstrapped at its
  first image:
  - A (images 140–180), chassis only;
  - B (555–620), tendon moving.

  They contain chassis and tendon starts, stops and running speed changes.
  Expected wall time is about 2–4 minutes in total, dominated by the two
  bootstraps and about 105 filter steps. The forecasts take seconds: events ×
  4 values of d × 4 paths, at a few ms each.
- **Stage 2, only after stage 1 is stable and reviewed.** The remaining MPPI
  trial windows from `trials.jsonl`, with its own runtime estimate and
  approval.
- Results go to the scratchpad and are summarized here. Each window is
  independent, so a stopped run resumes per window.

### Limitations

- The recording used the October stop-confirm-replace planner, so the
  in-flight command is often a hold, and it contains no reversals.
  Continuous-MPPI transitions are represented only by the running speed
  changes it does contain.
- `F` is the model's mean forecast, so `e_tip` measures the model's
  sensitivity to input timing, not physical tip error. The physical
  comparison against markers was 3e's.
- The event count per window is small, and is reported.
- The oracle plan isolates timing. It does not test how a real planner would
  re-plan.

### Stage 1 results (2026-11-02)

**Run.**
- Harness: `scratchpad/s3/latency_step1.py`; outputs `lat_A.json`,
  `lat_B.json` and `lat_summary.txt`.
- Wall time: 4.5 s for window A and 8.4 s for window B, far below the
  estimate.
- Coverage: all 23 transmitted command changes in the two windows, at every
  d. By class at d = 40 ms:
  - 8 starts;
  - 9 stops;
  - 3 running speed changes;
  - 3 combined starts with a stop on the other shaft.
- 14 of the 23 command motion.
- The actual root-to-transmit time D is d plus up to one encoder period: a
  median of 26, 46, 66 and 83 ms for d = 20, 40, 60 and 80 ms.
- Integrity: each stored filter state reproduced its reference forecast
  exactly at use time.
- The existing K = 2 law-reduction cost emits NaN warnings when a law weight
  underflows (window B). Every result is finite. This is pre-existing and not
  investigated here.

**Tip error against the achieved-encoder forecast at 0.8 s**, moving events,
median / P90 / max in mm:

| d | A0 (current) | A_d (d̂ = d) | A_d − 10 ms | A_d + 10 ms | A_img (no prefix) |
| ---: | --- | --- | --- | --- | --- |
| 40 ms | 0.359 / 0.915 / 1.180 | 0.264 / 0.734 / 1.011 | 0.294 / 0.793 / 1.011 | 0.214 / 0.675 / 1.011 | 0.618 / 1.456 / 1.881 |
| 60 ms | 0.450 / 0.980 / 1.376 | 0.264 / 0.741 / 1.011 | 0.292 / 0.801 / 1.011 | 0.214 / 0.681 / 1.011 | 0.618 / 1.457 / 1.908 |
| 80 ms | 0.557 / 1.113 / 1.560 | 0.267 / 0.742 / 1.011 | 0.293 / 0.802 / 1.011 | 0.217 / 0.681 / 1.011 | 0.629 / 1.460 / 1.925 |

**Reduction of A_d relative to A0 by horizon**, median (P90):

| d | 0.2 s | 0.4 s | 0.6 s | 0.8 s |
| ---: | ---: | ---: | ---: | ---: |
| 40 ms | −71% (−35%) | −70% (−29%) | −30% (−24%) | −27% (−20%) |
| 60 ms | −64% (−49%) | −63% (−47%) | −39% (−43%) | −41% (−24%) |
| 80 ms | −73% (−59%) | −69% (−55%) | −49% (−54%) | −52% (−33%) |

**Checks as declared:**
- **P1, materiality: PASS.** At d = 40 ms, A0 at 0.8 s has median 0.359 mm
  and P90 0.915 mm.
- **P2, compensation: FAIL.** At 0.8 s, A_d reduces the median by only
  27 / 41 / 52% and the P90 by 20 / 24 / 33% for d = 40 / 60 / 80 ms,
  against the required 50%. With ±10 ms jitter, the 30% criterion holds only
  at d = 60 and 80 ms with d̂ + 10 ms.
- **P3, consistency: PASS.** The tendon-input residual of A_d at tendon starts
  is 0.100 mm (n = 5, 2 mm/s), matching the measured start loss of 0.109 mm
  at 2 mm/s. The chassis-start residual median is 0.06 mm.

**Boundary of the failure.**
- The A_d residual at long horizons is identical for every d and jitter
  (maximum 1.011 mm), so it is not timing. It is execution loss at starts:
  the oracle sequence applies each later start as an ideal step, while the
  measured motion contains the 42–55 ms start dead time.
- Split by whether another start falls inside the 0.8 s horizon (d = 40 ms):
  - with no later start (n = 5): A0 0.538 mm, A_d 0.167 mm at 0.8 s (−69%);
  - with a later start (n = 9): −82% at 0.4 s but only −13% at 0.8 s.
- The stop-and-go recording almost always contains another start within
  0.8 s.
- The P90 at short horizons is set by the event's own start loss when the new
  command is itself a start.

**Also observed (no new mechanism inferred).**
- A_d + 10 ms beats A_d at every d. That is consistent with the 15 ms
  dispatch-to-motion lag in section 16: the effective latency is transmit
  plus about 10–15 ms.
- A_img is much worse than A0 (0.62 vs 0.36 mm median at d = 40 ms), which
  confirms the value of the existing measured-prefix plan-time root.

**Conclusion.**
- The gap is material.
- In-flight compensation removes about 70% of the error until the next start,
  but the declared 0.8 s criterion fails.
- The failure is bounded by start-execution loss, which this recording's
  stop-and-go pattern repeats inside nearly every horizon. Timing compensation
  cannot address it.
- Stage 2 is not run: more windows of the same pattern would not change this
  boundary.
- No change is made to the backend.
