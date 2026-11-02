# Real-time Gate 3 filter: design specification

2026-11-01. **Research shadow, selected by no runtime.** Results are in section 8. The
[pipeline reference](GATE3_FILTER_PIPELINE_REFERENCE.md) still describes the scalar research
filter. Sections 1-7 are the original proposal, and section 4 timings are its unmeasured estimates.
Replay driver: `cr_meta_lnn/scripts/reduced_filter_session.py`.

## 1. Goal and what stays

Target: estimator P99 <= 50 ms per image at a 5 Hz first milestone, with accuracy
preserved against the log-gain scalar filter. Preferred forecast target: tip RMSE < 1 mm
over 0.8 s. The scalar filter misses this on some real roots already, so that gap is a
modeling problem and is reported separately from speed work.

Kept unchanged as models and rules:

- curve family g, full marker R, station anchoring, spline profile prior, visit-local profiles;
- fixed-width chassis play, wall atoms, 1% per-image recovery, measured encoder inputs;
- one authoritative distal lambda/profile belief, held/engaged modes, directional moments;
- response model and observed-motion parameter factor (section 3.5);
- admission rules, Schmidt protection, takeup not teaching engaged J;
- no learning from forecasts, and shape likelihood not re-weighted by Cartesian error.

Changed as computation:

| Current | Proposed | Character |
|---|---|---|
| 54-67 dimensional joint per cell | Two small blocks | Approximation: reduced crosses |
| 129-255 cell Gaussians | Mass table over K shared laws | Approximation: tied laws |
| 24 raw-marker noise coordinates | At most 14 row-noise coordinates | Exact at fixed linearization |
| Cartesian transport per native sample | Once per image | Exact for frozen parameters |
| About 4-5 converged SLSQP fits per image | One bounded damped solve | Approximation: observation change |
| Runtime covariance gates throughout | Factored forms, boundary checks | Implementation |

## 2. What "tied laws" means

Each memory cell currently asks: "if the chassis memory were exactly here, what would
everything else be?" It answers with its own full Gaussian, built from that cell's entire
history. Those are the conditional histories.

Tying means cells with the same recent behavior share one Gaussian, called a law. Each cell
keeps its own memory position and probability, and still gets its own likelihood from every
image. What it gives up is a private record of how it got there.

Analogy: 129 people each keep a personal notebook, nearly all with identical contents. Instead
they share a few notebooks, and each remembers which one they use.

Why this is plausible: an interior cell has zero transmitted displacement, so the image cannot
tell interior cells apart. Measured: about 232 of 255 cells had zero displacement per image in
sustained motion, and stationary images reduced to one law.

What could break: when cells that really did move differently are forced into one notebook,
a two-peaked belief is summarized as one peak. See section 5.

## 3. Algorithm

### 3.1 Belief

$$
p(m,x,\ldots)=\sum_{L=1}^{K} w_L(m)\,\mathcal N(c;\mu_L,P_L)\ \cdot\ p(\text{distal})
$$

- **Distal block, once:** $\lambda_k,\lambda_{k-1},\beta$ (24 dimensions) with
  held/engaged probability.
- **Cartesian law:** $c=[p_T,\alpha,b_T,j_T,b_I,j_I,\lambda_k,\lambda_{k-1},e_{k-1}]$. That is
  16 model coordinates, 2 read-only lambda couplings and at most 7 retained noise coordinates.
- **Mass table:** $w_L(m_j)$ over the existing grid. Cells can hold mass on several laws, so
  regridding moves mass without merging laws.

K is the speed-accuracy dial. Proposed test values are 4, 8 and 16.

### 3.2 Predict

Play, recovery and regrid run on the mass table with the existing transport laws, per native
sample. Recovery adds mass at uniform locations under one law holding the current total moments,
matching what `with_uniform_memory_error` assigns today.

Distal native events and profile queries keep their current equations.

Over one image with frozen parameters the Cartesian increment telescopes, so it runs once:

$$
\mu_{p,L}^-=\mu_{p,L}+b_T(\Delta m_L+\alpha\Delta\kappa)+j_T\Delta\lambda
$$

### 3.3 Laws for one interval

Cells split by whole-image displacement $\Delta m_j$:

| Group | Displacement | Law |
|---|---|---|
| Interior, never touches the wall | 0 | Existing interior laws |
| At the wall throughout | $\Delta c$ | Existing wall laws |
| Reaches the wall during the interval | Between | 1-3 new band laws at node displacements |

Each cell records its offset from its law, $a_j=\Delta m_j-\Delta m_L$.

### 3.4 One image factor

Solve once per image from the prediction, not per cell or per lambda value:

$$
\min_{T,s}\ \tfrac12\big\|W\big(h(T,\bar\lambda,s)-y\big)\big\|^2
\quad\text{with the existing station bounds and ordering}
$$

Use at most 3 damped Gauss-Newton steps with an explicit active set, warm-started from the previous
image's orientation and stations. Analytic spline and pose derivatives replace repeated small Torch
evaluations. There is no temporal pose prior, matching current behavior.

Eliminating pose and stations gives the same partition as `partition_tip_body_marker_evidence`:

- **Shape row:** $r_s=h_s\,\delta\lambda+\varepsilon_s$.
- **Tip Cartesian rows:** $r_c=H_c(\delta p_T,\delta\lambda)+\varepsilon_c$, orthogonal to the shape row.
- **Readouts:** $z_q(\lambda)=\bar z_q+g_q(\lambda-\bar\lambda)+G_q\varepsilon$ for $q\in\{I,T\}$.
  Here $g_q,G_q$ come from the same factorization. At a fixed active set they should equal the
  implicit derivatives used today, and this must be verified.

If the solve does not reach its acceptance test, the image holds parameters, is flagged unhealthy and
does not fall back silently to another estimator.

### 3.5 Updates, in this order

**(a) Distal.** Held/engaged conditioning, directional moments and full-moment collapse keep their
equations, using shape rows only. The local B-spline support makes this sparse. The result gives
$\lambda_k^+$ and $\mathrm{Var}(\Delta\lambda)$ using the existing formula.

**(b) Cell likelihood.** Every cell with mass on law $L$ gets a closed-form likelihood:

$$
\ell_{j,L}=\log\mathcal N\big(r_{c,L}-H_p\bar b\,a_j;\ 0,\ S_{L}(a_j)\big)
$$
$$
S_L(a)=H_c\big(P_L+a\,A_L+a^2 B_L\big)H_c^T+I
$$

$A_L,B_L$ are the linear and quadratic covariance sensitivities to displacement. Weights update
exactly as $w_L(m_j)\leftarrow w_L(m_j)e^{\ell_{j,L}}$, then normalize. Weighting does not depend on
interpolating anything between cells, and it preserves recovery revival.

**(c) Position.** Condition each law on $r_c$ with lambda and parameter rows protected, as
`condition_tip_body_cartesian` does today.

**(d) Parameters.** This is the current observed-motion factor, not direct filtering on the markers:

$$
r_{q,L}=z_{q,k}(\lambda_k^+)-z_{q,k-1}(\lambda_{k-1}^+)-b_q(\Delta m_L+\alpha\Delta\kappa)-j_q\Delta\lambda^+
$$

It uses the same physical-gain or log-gain conditioning step and existing admission. Protected rows get
zero gain. Because the factor uses two image readouts rather than the position state, earlier position
lag cannot enter J through prior crosses.

**(e) Noise compression.** Raw noise enters only through its projections, so only row noise needs
keeping: the shape row plus $G_T$ and $G_I$, at most 7 rows per image. The previous image's rows are
marginalized after use. This replaces the raw 24 coordinates without changing the linearized law.

**(f) Law reduction.** While more than K laws remain, merge the pair with the lowest Runnalls
Kullback-Leibler bound, using full moments including scatter. Mass tables add. Protection is then
checked on the mixture marginal.

### 3.6 Forecast

Frozen root: for each law, advance play on the mass table with the planned schedule, and apply the
response model with frozen parameters. The state is small enough to batch in the planner, though
MPPI integration is later work and outside this specification.

## 4. Estimated cost

Unmeasured estimates in NumPy, for K=8:

| Stage | Work | Estimate |
|---|---|---:|
| Play and mass table | 129 x K array operations per native sample | 1 ms |
| Image factor | 3 iterations at 4 stations, one factorization | 2-5 ms |
| Distal update | 2 modes, 24 sparse dimensions | 1-2 ms |
| Cell likelihoods | Up to 129 x K batched small solves | 1-3 ms |
| Position and parameter updates | K x about 25-dimensional updates | 3-6 ms |
| Law reduction, boundary checks | Small | 1-2 ms |
| **Total** | | **about 10-20 ms** |

Measured comparison: 3.5 s on the fixture image.

## 5. Objections from project history

Citations were read at source. Paths are in `repository_archives/cleanup_20261101/history/robot-infra/audits/ros-realtime/`.

| Finding | Source | Consequence |
|---|---|---|
| Direct conditioning lets earlier position lag reach J through prior crosses | `GATE3_OBSERVATION_GROUNDED_J_AUDIT.md:93-98` | Plain joint filtering dropped; factor kept (3.5d) |
| J depended on forecast: 0.2 to 0.7 moved J by 0.163 with identical markers | Same file, 437-441 | Becomes required test T1 |
| Full-joint engagement scoring collapsed: probability 0.775 to 2.8e-39 at 0.3 mm position error | `GATE3_TIP_BODY_JOINT_FILTER_PLAN.md:325-331` | Distal stays one-way protected (3.5a) |
| Merged histories at +/-0.3 mm: memory 0.074 mm, frozen position 0.017 mm, log evidence -1.58 | `GATE3_CHASSIS_MEMORY_REPAIR_PLAN.md:178-181` | Band laws plus K; test T3 |
| "A common covariance cannot generally represent compressed mixed histories" | Same file, 136-137 | Laws retain separate covariances; not one common covariance |
| Reweighting divergent knob histories moved held eta 0.0019 and failed protection at image 570 | `GATE3_FILTER_PIPELINE_REFERENCE.md:960-963` | Mixture protection retained; test T4 |
| M3 station at its upper bound on 21 of 30 images | `GATE3_KNOB_SCALE_INTERFACE_PLAN.md:2856-2858` | Explicit active set in 3.4; test T5 |
| Pose velocity priors and low-pass filtering failed | Reported by history sweep; not re-read | No temporal pose prior |

Not tested anywhere in the project, per a grep-based sweep: full-state memory below 129 cells, dropping distal
and Cartesian crosses, a fixed-iteration geometry solve, and raw-noise compression. These are open questions,
not evidence for or against. The value 129 has been shown sufficient, not necessary.

Counterweight: the current route's superiority on real forecasts is itself unproven, so the comparison must be
scored and not assumed. The ±0.3 mm stress error of 0.017 mm is also small against a 1 mm target, though it came
from a known-J fixture.

## 6. Experiments

One atomic change at a time. Each stage stops on failure, with the boundary reported before anything is added.
Runs over one minute require approval first.

Segments: 445-485 and 485-500 (sustained chassis, flips, admitted learning) and 552-576 (tendon onset, knob motion).
A chassis reversal segment must be added before E6, because none is confirmed in these.

| Stage | Single change | Baseline | Runtime |
|---|---|---|---|
| E0 | None; score scalar log gain with forecasts | Reference for all | About 5 min |
| E1 | Row-noise compression (3.5e) | Scalar and frozen fixtures | Under 1 min |
| E2 | Tied laws, with distal supplied from scalar (privileged) | E0 | 2-4 min |
| E3 | Reduced Cartesian block, dropping beta crosses | E2 | 2-4 min |
| E4 | One image factor (3.4) replacing refined fits | E3 | 2-4 min |
| E5 | Native distal block, sparse beta | E4 | 2-4 min |
| E6 | Fully autonomous filter, physical gain arm, timing | E0 | To be declared |

### Pass and fail checks, declared before running

The controller's real tolerance is unknown, because control has not yet run with an accurate model. These are
working limits, set by the user on 2026-11-01. They may be relaxed if they prove near impossible under the real-time
budget, but only as an explicit decision reported with the measured tradeoff, not to absorb a failed comparison.

Accuracy is scored on frozen-root 0.8 s forecasts at roots fixed in E0 before any candidate runs:
448, 452, 456, 460, 464, 468, 472, 476, 477, 480, 484, 488 and 556, 560, 562, 564, 568.

- **F1:** no root's forecast RMSE worsens by more than 0.1 mm, and segment RMSE worsens by no more than 5%.
- **F2:** current tip error against the scalar posterior stays within 0.05 mm RMSE.
- **F3:** full-gain admission decisions match on at least 95% of images, and the first admitted image stays within 1 image.
- **F4:** wall probability differs by no more than 0.1 outside 455-465. Inside that stretch it is reported without
  a limit, because the scalar behavior there may reflect model mismatch.
- **Absolute target:** RMSE < 1 mm reported per root for both filters. This is not a pass condition for E1-E5.

Required structural tests:

- **T1 forecast invariance:** repeat the 0.2 to 0.7 fixture. J change must be below 0.01, against 0.163 for the old route.
- **T2 overlap law:** two overlapping pairs must match the three-image reference at 1e-10 in the linear limit,
  as the current route does (`GATE3_OBSERVATION_GROUNDED_J_AUDIT.md:677-680`).
- **T3 merge stress:** the ±0.3 mm fixture must retain two laws when K allows. Report errors at K=2 and K=4.
- **T4 protection:** held parameter marginals stay within the existing 1e-10 limit, including through reduction.
- **T5 factor agreement:** at fixed active sets, $g_q,G_q$ match implicit derivatives. Bound-active images are
  reported separately.
- **T6 validity:** all covariances are positive semidefinite, using a per-cell normalized metric that skips zero-variance
  coordinates. Interpolated entrywise covariances failed this, so this check is not optional.

Timing is reported only from E6, as a distribution with P50, P95, P99 and maximum, and excluding any privileged supply.

## 7. Scope and contract notes

- The user accepted tied laws in place of per-cell conditional histories on 2026-11-01, as a tested experiment.
- This is a new research estimator beside the scalar filter, in `cr_meta_lnn`. The scalar filter stays unchanged as the
  reference. No deployment, ROS, hardware or default changes.
- The array-native plan's invariants (no moment collapse, byte-exact deduplication) continue to govern that plan. They
  do not bind this separate experiment, which is judged on accuracy.
- Log gain is the baseline. The physical scalar is the preferred end state if it stays valid and positive, tested as an E6 arm.
- The pipeline reference is updated only when an estimator here becomes the current research path.

## 8. Results so far

### E0: scalar log-gain baseline (complete)

Unchanged filter on fresh 445-500 and 552-576, completing without rejection. The 445-485 posterior rows match the
archived report exactly, so the positivity gate repair did not alter log-gain behavior. Forecasts use privileged achieved
future encoders and no future markers. RMSE is over sampled images in the horizon, against the anchored marker-only
readout, which is not physical truth. Scoring against the filter's own future posterior differs little.

| Root | Situation | 0.8 s tip RMSE, mm | Max, mm |
|---:|---|---:|---:|
| 448 / 452 | Chassis moving, not at wall | 0.354 / 0.355 | 0.603 / 0.599 |
| 456 / 460 | Wall probability flipping | **1.549 / 1.229** | 2.363 / 1.553 |
| 464 / 468 / 472 | Wall contact settling | 0.618 / 0.531 / 0.421 | 0.888 / 0.785 / 0.546 |
| 476 / 477 / 480 / 484 / 488 | Settled, admission from 475 | 0.367 / 0.429 / 0.306 / 0.289 / 0.263 | 0.516 / 0.588 / 0.509 / 0.349 / 0.392 |
| 556 / 560 | Tendon takeup, forecast crosses engagement | **1.829 / 3.898** | 3.809 / 5.409 |
| 562 / 564 / 568 | Just after tendon engagement | **1.168 / 1.161 / 1.422** | 2.358 / 1.842 / 1.688 |

Segment RMSE is 0.672 mm (445-500) and 2.173 mm (552-576). Current tip error against the readout is 0.124 and 0.120 mm.
Ten of 17 roots meet 1 mm. The misses cluster at transmission transitions, so they are a modeling boundary that speed
work will not remove. These 17 roots were fixed in advance but are not a population estimate.

### E1: row-noise compression (partial pass)

Read-only on real images 447-461, log gain, one representative conditional per image. The run hit its 50 s cap at 461.

- Rows kept from an image: rank 7, as predicted. Next image's operator on those raw coordinates: rank 6.
- State correlations with previous raw noise lie in the kept rows: residual at most 1.3e-15 relative.
- Factor covariance change from carrying correlations only through kept rows: at most 5.7e-10 in normalized units.
- Next image's operator lies up to 4.9e-4 outside the kept span, from relinearizing at a new lambda. That part carries
  no state correlation, so its variance is available from the marker covariance, assuming the raw marginal stays at R.

Extended to 502-516 and 554-564: rank is unchanged, state correlation residual stays at most 4.4e-15, factor covariance
change at most 4.8e-11, and operator residual at most 7.5e-4. T2 passes as pure linear algebra: compressed sequential updates
match the joint three-image reference at about 2e-15, with 9 instead of 18 dimensions.

Still not covered: only 1 of the extension images had chassis learning admitted, and no filter yet runs on compressed
states. E1 is complete as a read-only check. The real test comes when compression is implemented.

### E2: tied laws (passed declared limits)

Module `cr_meta_lnn/training/hybrid/tied_memory_filter.py`, with tests in `tests/test_tied_memory_filter.py`. The scalar route is
unchanged. Existing distal, observation, readout, admission and parameter owners run once per (law, displacement class). The
distal owner runs as normal rather than being replayed from the scalar run, because it does not depend on memory cells.
Log gain, 2 band nodes.

| Check | Limit | K=4 | K=8 | K=16 |
|---|---:|---:|---:|---:|
| F1 worst root change, mm | 0.1 | 0.003 | 0.003 | 0.003 |
| F1 segment RMSE change, 445-500 | 5% | -1.0% | -1.0% | -1.0% |
| F1 segment RMSE change, 552-576 | 5% | 0.0% | 0.0% | 0.0% |
| F2 tip against scalar RMSE, mm | 0.05 | 0.0087 | 0.0088 | 0.0088 |
| F3 admission agreement; first admitted | 95%; within 1 | 100%; same | 100%; same | 100%; same |
| F4 wall difference outside 455-465 | 0.1 | 0.000 | 0.000 | 0.000 |
| Wall difference inside 455-465, reported only | none | 0.219 | 0.215 | 0.215 |
| T4 held marginal change | 1e-10 | 1.6e-11 | 1.6e-11 | 1.6e-11 |
| Evaluations per image, median/P95/max, 445-500 | none | 8/17/17 | 12/27/30 | 20/39/44 |

T3 passes on its fixture; T6 is enforced by the retained constructor gates, with no rejection in any run. T1 and T5 do not apply,
because E2 changes neither the learning factor nor the image factor.

Limits of this result: 17 fixed roots, not a population. No confirmed chassis reversal in these segments. Forecasts drop cells below
1e-6 mass as a readout. Scoring on physically achieved future encoders is the intended basis for filter forecast accuracy (user,
2026-11-01); it is not a limitation. Controller execution error against commands is a separate later question. Within a displacement class, offset cells get their own likelihood,
but their mean offset is not added to the law's scatter. The flip-stretch wall difference does not shrink with K, so laws are not
what limits it. Python step timings under parallel load are not latency evidence. The module repeats the owner's first stage to
read its prior, and copies the protected-row policy; both should be folded into one owner on any promotion.

### E2 follow-up: law count and chassis reversal

Reversal test: fresh 552-604. Wall contact forms near 570, then the chassis reverses at 577 and travels 4.7 mm. Roots 572, 576, 580,
584, 588 and 592 were fixed before running. The scalar baseline scores 1.311, 1.236, 0.376, 0.320, 0.287 and 0.250 mm, so forecasts crossing
the reversal also miss 1 mm, and segment RMSE is 0.763 mm.

| K | Segment | Images | Evaluations median/P95/max | Worst root change, mm | Tip against scalar, RMSE/max, mm | Max wall difference | Result |
|---:|---|---:|---:|---:|---:|---:|---|
| 1 | 445-500 | 55 | 5/8/8 | 0.003 | 0.0064/0.034 | 0.002 outside flips; 0.242 inside | Pass |
| 1 | 552-604 | 49 of 52, budget stop | 4/8/8 | see below | 0.039/0.199 | **0.878** | **Fails F4** |
| 2 | 445-500 | 55 | 6/9/12 | 0.001 | 0.0089/0.048 | 0.00002 outside flips; 0.079 inside | Pass |
| 2 | 552-604 | 43 of 52, budget stop | 4/9/12 | 0.000 | 0.0001/0.0003 | 0.0001 | Pass on completed part |
| 4 | 552-604 | 52 | 7/15/17 | 0.000 | 0.0028/0.011 | 0.001 | Pass |

K=1 changes forecasts at roots 580-592 to 0.252, 0.254, 0.218 and 0.248 mm, lower than the scalar's against this readout, but its wall
probability departs from the scalar by up to 0.878. One shared law therefore loses memory information after a reversal. The lower forecast error is
an unexplained observation from four roots, not evidence that K=1 is better.

K=2 stopped at image 595 on its 55 s cap, after covering the reversal and all roots' start images, so images 596-604 are unscored. Results do not improve
monotonically with K: wall differences in 455-465 are 0.242, 0.079 and about 0.215 for K=1, 2 and 4-16. Likely the scalar filter's own
regrid mixing is part of the comparison, which is not separated here.

Decision: K=4 carries forward because it passes everything in full. K=2 is a cheaper candidate pending a complete run. K=1 is rejected.

### E3: reduced Cartesian block with compressed noise (FAILED F4 at one reversal)

Module `cr_meta_lnn/training/hybrid/reduced_law_filter.py`, with tests in `tests/test_reduced_law_filter.py`. Each law is a 24-dimensional
state: 16 Cartesian and parameter coordinates, previous lambda, and 7 whitened noise rows. Profile crosses are dropped. One learning-free
reference conditional runs the existing owners per image and supplies the distal block, marker operators and observed-motion rows. Log gain, 2 band nodes.

Implementation correction: the reference showed that image k also smooths the previous lambda (variance 0.064 to 0.036 at image 446). Law crosses
therefore move through the owner's own `shadow_cross_transport`. Unlike the owner, position is not corrected for this previous-image smoothing, and the
process is linearized at the observed lambda increment. These are deliberate parts of the reduced form.

| Check | Limit | K=4, 445-500 | K=4, 552-604 | K=2, 445-500 | K=2, 552-604 |
|---|---:|---:|---:|---:|---:|
| F1 worst root change, mm | 0.1 | 0.002 | 0.044 | 0.001 | 0.044 |
| F1 segment RMSE change | 5% | -0.5% | 0.0% | -0.7% | 0.0% |
| F2 tip against scalar RMSE, mm | 0.05 | 0.0054 | 0.0321 | 0.0069 | 0.0320 |
| F3 admission agreement | 95% | 100% | 100% | 100% | 100% |
| **F4 wall difference, outside 455-465** | **0.1** | 0.001 | **0.984 (image 579)** | 0.001 | **0.982 (image 579)** |
| T4 held change | 1e-10 | 3.3e-16 | 5.7e-11 | 3.3e-16 | 8.2e-12 |
| Evaluations median/P95/max | none | 8/14/17 | 7/15/15 | 6/9/12 | 5/9/9 |

Failure boundary: images 578-579 only, 2 of 36 images with a wall value. The scalar filter goes 0.044, 1.0, 0.041, tied laws in E2 match it, and E3 goes
0.38, 0.016, 0.001. They agree again from 581. K does not change this, so the cause is in the reduced form, not law count. It is not separated between dropped
profile crosses, position not being smoothed, and the observed-feature linearization.

Other effects of the reduced form: tip against scalar grows after the reversal, 0.008 mm RMSE before image 568 and 0.042 mm after 577; current tip against the readout
is 0.177 mm against the scalar's 0.161 mm on this segment; and roots 584-592 worsen by up to 0.044 mm while 580 improves by 0.038 mm. No forecast root lies inside
578-579, so the forecast effect of the failed images is unmeasured. Correlation eigenvalues stay positive; minimum is 0.045.

T1 is not numerically run: in this form the learning factor reads only accepted lambda and markers, so forecast independence is structural. T2 passes in a test using
the module's own basis functions at 1e-10. T5 does not apply. Timing, under four parallel processes: reference 0.55 s, law updates 0.10-0.16 s, about 15 ms per evaluation.
That per-evaluation cost is well above the section 4 estimate.

### E3 follow-up: forecasts from the failed images and profile

Roots 577, 578 and 579 were added after the failure, against the same readout, at K=4:

| Root | Wall, scalar / E3 | 0.8 s RMSE scalar, mm | E3, mm | Change, mm |
|---:|---:|---:|---:|---:|
| 577 | 0.000 / 0.000 | 0.683 | 0.685 | +0.002 |
| 578 | 0.044 / 0.380 | 0.536 | 0.725 | **+0.188** |
| 579 | 1.000 / 0.016 | 1.082 | 0.306 | -0.775 |

Root 578 exceeds the 0.1 mm F1 limit. These roots were chosen because of the failure and do not enter the fixed 17-root scores. Encoders and markers around the reversal show
chassis travel of 0.28 mm per image at 578 and 579 with raw M0 Z changing by 0.023 and -0.022 mm, and Z staying flat until 599. The user is reviewing a plot of these
trajectories to judge which contact belief is correct.

User decision, 2026-11-01, after reviewing that plot: E3 is more physically right. The chassis is still in takeup immediately after it starts moving, and true engagement comes
much later. E3's 0.38 at image 578 is not exactly right but better than the scalar's 1.0 at 579. The F4 exceedance at 578-579 is therefore accepted as a scalar artifact and E3 carries
forward. Root 578's +0.188 mm stays recorded as a known cost. F4 otherwise remains in force. This is a judgment on one reversal, not a general finding that E3's contact belief is better.

The mass regrid now uses `deposit_log_masses`, a direct array deposit tested against the existing regrid owner at 1e-14. It carries no moments, so it changes speed only.

### E4, first part: bounded interface readout (passed)

Single change: `bounded_interface_readout.py` replaces the refined SLSQP readouts behind the same factory interface. At most 3 damped Gauss-Newton steps at fixed lambda, warm-started from the
previous solve, with analytic derivatives and explicit active bounds. Same objective, no pose prior. Sensitivities use Gauss-Newton curvature rather than the full residual Hessian.

T5 against 543 cached real refined readouts, with no replay:

| Steps | Position difference median/P95/max, mm | Lambda sensitivity, max relative | Marker sensitivity, max relative | Time per readout, median |
|---:|---:|---:|---:|---:|
| 1 | 5.6e-7 / 0.002 / 0.021 | 1.2% | 2.1% | 1.5 ms |
| 3 (selected) | 3.6e-9 / 1.2e-6 / 6.5e-5 | 0.84% | 2.1% | 2.3 ms |
| 10 | 1.8e-9 / 1.7e-8 / 3.6e-8 | 0.84% | 2.1% | 2.8 ms |

The sensitivity difference does not shrink with more steps, so it comes from Gauss-Newton curvature, not convergence. Bound-active images were not reported separately, as T5 asks, and each key was solved in
a warm-start sequence that differs slightly from the live call order.

Filter results at K=4 and K=2 match E3: fixed-root worst change 0.044 mm, segment change at most 0.0% on 552-604 and -0.7% to -0.5% on 445-500, tip against scalar 0.032 mm RMSE at most, admission 100%, T4 at most 5.8e-11.
Wall difference above 0.1 occurs only at the accepted 578-579, plus inside 455-465 where no limit applies: 0.17 at K=4 and 0.036 at K=2. Roots 578 and 579 score 0.724 and 0.307 mm.

Timing under four parallel processes: about 0.25 s per image at K=4 and 0.22 s at K=2, against about 3.0 s for the scalar filter. The reference is about 0.20 s. Law updates are 0.019-0.049 s.

Profile over 12 images, with profiler overhead, 309 ms per image: 175 ms in 5 `curve_registration.fit` calls per image, reached through the distal step, `observe_tip_body_markers` and the evidence functions; 60 ms in law merge
costs; about 8 ms in the 13 reduced law updates. The next isolated target is those repeated per-image fits.

### E4, second part: warm-started curve fits and merge-cost cache (passed, with one gate rejection)

`BoundedFitCurve` serves `curve.fit` behind the same interface, with no scalar source change. These fits minimize the existing unweighted Procrustes objective, which differs from the full-R readout objective. One converged fit per kind
seeds later images, and exact repeats reuse their result. Merge costs are computed in batches and updated incrementally, tested to select the same merges as full recomputation.

Direct comparison with the original fits on real images: bounded free fit lambda within 9.2e-6 and fixed-lambda pose within 3.1e-6 mm at 3 steps. The free fit with unbounded stations, which feeds the hard detector, is poorly conditioned: lambda off by up to 0.65 at 3 steps and 0.05 at 6, so it alone gets 6 steps.
Pose now comes from spline points rather than the torch-exact reprojection.

Filter results match the first E4 part at the reported precision. The declared distal limits hold: lambda RMSE about 0 with maximum 4.4e-8 against the 0.05 limit, first engaged image 562 for both, engagement probability within 1.1e-8. Fit counts on 552-604: 3 cold, 258 bounded, 5 ending with a last step above 1e-3.

Timing, one process at a time, K=2, no forecasts, both segments, excluding the first two images (n=103): **P50 64 ms, P90 97 ms, P95 100 ms, P99 108 ms, maximum 123 ms**. None under 50 ms, 45 over 66.7 ms and 5 over 100 ms. Four BLAS threads give the same result. First images after a fresh start take 185-223 ms and are not representative of a
running filter. The slowest images are about 100 ms, mostly those with admitted chassis learning or a transition. Median split: reference 55 ms, laws 9 ms. This is an isolated replay and is not a live timing qualification. The P99 target of 50 ms is not met.

Gate rejection: a fresh start at 565 stops at image 573 with `External completion changed the original distal conditional`. Means match exactly; one profile covariance entry differs by 6.5e-9 with two threads and 8.4e-9, at a different entry, with one, against that gate's absolute 1e-10. The same start passes with slow fits, and 552-604 passes through 573 with fast fits. Likely roundoff in large-variance profile
coordinates, not confirmed. The gate was not changed. It belongs to the reference path that E5 would remove, and a stop in a live filter would need a defined response before any deployment.

Sweep of 16 fresh starts across the session, 25-60 images each, K=2: 15 complete with no rejection, and the one above stops. No accuracy scoring in this sweep.

Remaining profile, 84 ms per image with overhead: reference 71 ms, no single dominant item. Spline evaluation is 16 ms, three `prepare` SLSQP station solves are 14 ms, `interface_marker_step` is 20 ms, and validation is spread throughout.

### E5: reference removed (passed)

`native_image_step` runs the distal owner once and needs no Cartesian conditional. `distal_image_operators` keeps one small Gaussian over lambda, profile, previous-image lambda and retained noise rows. It uses the owner's native records, response operators, traces and conditioners. Accepted lambda and profile are installed from the owner's step. This scope differs from section 6 in one way:
the distal owner itself is kept, not rewritten. Nonzero lambda process noise rejects explicitly. A new internal check compares this block with the owner's traced conditional at 1e-8 mean and 1e-7 covariance, absolute or relative, in place of the removed path's absolute 1e-10.

T8, native operators beside the reference on every image of both segments, 107 images: distal block within 2.9e-6, lambda means within 3.4e-9, smoothing map within 7.3e-9, tip and interface sensitivities within 6.6e-7, observed-motion means within 1.4e-5. Fresh-noise coefficients in the row basis differ because the basis can rotate or change sign; they are not compared directly, and the filter results below cover them. These segments contain direction associations but no conditional visit opening, so the opening branch is covered only by construction.

Log-gain filter results are unchanged at the reported precision from E4. The sweep of 16 fresh starts now completes 16 of 16, including the start at 565. 22 focused tests pass.

Timing, one process, K=2, no forecasts, both segments, excluding the first two images (n=103):

| Route | P50 ms | P90 | P95 | P99 | Max | Over 50 ms | Distal, P50/P99 | Laws, P50/P99 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Log gain | 40.1 | 73.0 | 74.7 | 88.3 | 96.1 | 44 | 29.8 / 37.9 | 8.6 / 50.5 |
| **Physical gain** | **37.7** | **41.7** | **42.6** | **43.8** | **46.0** | **0** | 29.9 / 34.8 | 8.4 / 11.9 |

The log tail comes from `condition_positive_knob_scale`: 62 of 71 ms in the law step on admitted images, from about 29 relinearizations per call. The physical route uses one first-order step. First images after a fresh start take 100-140 ms on the log route and were not separately measured on physical. These are isolated replays, not live timing.

### Physical-gain arm at E5 (declared limits pass, one behavior to review)

Scored against the same log-gain scalar baseline, K=2, 17 fixed roots. A physical scalar baseline over these segments has not been run.

| Check | Limit | 445-500 | 552-604 |
|---|---:|---:|---:|
| F1 worst root change, mm | 0.1 | 0.001 | 0.057 |
| F1 segment RMSE change | 5% | -0.7% | -1.8% |
| F2 tip against scalar RMSE, mm | 0.05 | 0.0069 | 0.0222 |
| F3 admission agreement | 95% | 100% | 100% |
| F4 wall difference above 0.1 | 0.1 | none outside 455-465 | none |
| T4 held change | 1e-10 | 2.2e-16 | 6.0e-11 |

Root 562 improves from 1.168 to 0.901 mm and root 572 from 1.311 to 1.076 mm.

Behavior to review: F4 passes here because physical gain follows the scalar's wall probability at 578-579, including the 1.0 at image 579 that the user judged wrong under log gain. Root 579 returns to 1.082 mm, against 0.307 mm for the log-gain reduced route. Agreement with the log scalar is therefore not the better physical result at this image.

Positivity: alpha stayed positive in both segments and all 14 additional fresh starts. The lowest active law mean is 0.082, and the smallest margin is 1.24 standard deviations of mixture alpha. So this is positive so far, not guaranteed. The sweep has no accuracy scoring.

Profile over images 578-589 under cProfile: reference 0.64 s per image, of which cold refined interface readouts take about 0.41 s. Law step about 0.35 s, of which `_grid_masses`
takes about 0.27 s: 162 calls at 20 ms, using the general moment-carrying regrid on mass alone. The law updates themselves are a small remainder and were not separately resolved.

### E6: whole session, stop response and loaded timing (physical gain primary, K=2)

New in `reduced_law_filter.py`: `ReducedEstimator` commits each image all-or-nothing. A rejected image changes no state and reports `holding`, meaning stale. The next image spans the full measured path from the last commit. Three consecutive rejections latch `fault` until a new explicit initialization. It is estimator-only: it commands nothing and changes no safety gate. Guards added to the reduced path: positive physical gain, finite laws, covariance validity, and rejection of rotation motion. 26 focused tests pass.

**Whole session.** Continuous from image 3 through 1224, 1,221 images, on physical and log gain: no rejections. The recording's images are about 100 ms apart. Scoring against the marker-only readout, which is not physical truth, using achieved encoders, 248 roots with at least 6 scored images:

| Situation over the horizon (rough classifier from encoders) | Roots | Physical RMSE, mm | Physical max root, mm | Roots within 1 mm |
|---|---:|---:|---:|---:|
| Held | 89 | 0.232 | 0.33 | 100% |
| Chassis moving, same direction | 92 | 1.103 | 5.49 | 80% |
| Tendon or knob moving only | 59 | 1.285 | 6.22 | 78% |
| Chassis reversal | 4 | 0.829 | 1.12 | 75% |
| Tendon reversal | 4 | 3.378 | 5.74 | 25% |
| **All** | **248** | **1.032** | **6.22** | **85.9%** |

Per-root median is 0.30 mm and P95 is 1.63 mm. Log gain gives 0.995 mm overall and 85.5%. Per root, physical minus log has median 0.000 mm, with 10 roots worse and 11 better by over 0.1 mm. Roots are chosen on a regular stride plus listed events. The reversal classes are small, and the classifier is rough. This is not a matched whole-session scalar comparison, which was not run because it would take about an hour. At 18 roots shared with saved scalar runs from different starts, the mean absolute difference is 0.165 mm and the worst excess is 0.293 mm, at root 464. The earlier 2.68 mm full-session figure came from an older pipeline, with a different target, and is not comparable.

Positivity over the session: lowest law mean alpha 0.077 at image 642, smallest margin 1.11 standard deviations. Positive throughout, with a thin margin.

**Stop response on real data.** Bad markers injected into 552-604: isolated rejections hold, then resume, with tip within 0.046 mm of the uninterrupted run and 0.002-0.004 mm by image 604. Two consecutive rejections behave the same. Three latch a fault.

**Timing beside the planner.** Whole-session filter in one process while `compute_profile` replays the recorded 512-sample CUDA planner, non-actuating, excluding the first two images, n=1,219: **P50 36.5, P90 42.3, P95 43.8, P99 47.0, P99.9 55.4, maximum 59.4 ms**. 10 images over 50 ms, none over 66.7 ms. Planner P50/P95/P99 is 15.7/17.4/24.5 ms alone and 16.1/17.6/21.6 ms beside the filter, 82 plans, with no misses reported in the sample run. The planner profile uses its own v171 fixture, and these were separate processes with no ROS, camera or recording load. This does not qualify live timing.

**Reversals.** Under continuous history both routes hold wall probability near zero through the takeup at 577 until about 599, when M0 Z moves, as the user judged correct. The blip at 579 seen from a fresh start at 552 does not appear from image 3. So that behavior depended on the start, not only on gain route. Other spikes during apparent takeup remain, for example near 847-851. Takeup at these reversals looks like roughly 4-4.5 mm by eye, against the 7.05 mm calibrated width. Not diagnosed.
