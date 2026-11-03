# Prior identification session: plan (draft, awaiting approval)

2026-11-02. A short, scripted, open-loop data-gathering session before any control test, to identify the
reduced filter's lambda profile and Jacobian priors over the full tendon range and base insertion. Nothing
here has run. Hardware motion needs explicit authorization. Never SET_ZERO.

## Why

The filter's current priors come from one October recording and do not cover the hardware range:

| Prior | Current source | Problem |
| --- | --- | --- |
| Lambda profile, per direction | `gate3_compact_cycle_oracle_prior_20261025`: an "ORACLE same-cycle" GP fit to one Oct 8 cycle (images 561–716), labelled `causal_transfer_qualified: false` | Support is only [0.0009, 10.77] mm of tendon, while the joint range is 0–15 mm. The filter faults above 11.27 mm, so the controller limit is 10.8 mm |
| Engagement widths, per direction | `gate3_startup_width_comparison_20261026/widths_broad.json` (Oct 8 posterior) | Same recording; the loading full gap was never identified |
| Chassis axial response direction (J axial prior) | Explicit `trusted_vector` (Oct 27); no derivation found | 10.1° from the measured engaged chassis response on its own session (pre-lab tools) |
| Tip response per lambda (j) | Zero mean, diffuse; learned online after about 12 s of tendon motion | No prior at all |
| Marker covariance R | Oct 20 still window | Camera setup and registration have changed since |
| Chassis play width | 7.05 mm (calibrated) | Kept unless the data contradict it |

## Session design: four blocks, about 14 min of motion

All are existing `causal_experiment` schedules, with durations from the generator (`imricor_test`
limits, home at insertion 20 mm, tendon 0, 10 ms reference step):

| Block | Schedule and settings | Motion | Duration | Feeds |
| --- | --- | --- | ---: | --- |
| A, still | `stationary`, `static_s=30` | None | 30 s | R (`prior_refit covariance` needs ≥ 20 s) |
| B, insertion strokes | `insertion`, amplitude ±15 mm (minimum 12) around 20 mm; slow 2 and fast 5 mm/s; 3 repeats; tendon at 0 | Chassis only, beyond the 7.05 mm play in both directions | 314 s (≈ 210 s with 2 repeats) | Axial response direction and gain (`prior_refit axis`); play width check |
| C, full tendon sweeps | `compensated_bend_insertion_sweep` (as is): tendon 0 → 15 → 0 mm through a 7.5 mm bias, at insertion plateaus 0, 13.3, 26.7 and 40 mm; two passes in reversed plateau order, one slow and one fast cycle per plateau, branch order flipped | Tendon alone (`lin == bend` cancels raw shaft 0), plus chassis plateau moves with the tendon at 7.5 mm | 413 s | Lambda profile over the full range, both directions, at four insertions; j; chassis response with a bent tip |
| D, partial reversals | `tendon_reversal` (as is), bend limits 0.5–10.7 mm | Tendon reversals at several positions and speeds | 76 s | Engagement widths per direction |

**Hold-out by design.** Block C's two passes visit every plateau once each, so the plan is to
fit priors on pass 1 and validate on pass 2. A later session then checks transfer across days.
A same-session pass is not cross-session qualification.

**Run as one session or four.** The schedules are built to stop at each causal boundary. Running
them one after another inside one research session keeps camera registration fixed. Whether each run
re-homes, which would break physical history across blocks, is to be checked in the runner before
the lab.

## Offline processing after the session

| Step | Status |
| --- | --- |
| Export arrays (`qualify_model_data`) | Exists |
| R: `prior_refit covariance` on block A | Exists (validated on Oct 8) |
| Axial direction and gain: `prior_refit axis` on block B, cross-checked on block C's plateau moves | Exists (validated on Oct 8) |
| Per-image lambda observations: marker-only curve fits | Owner exists (bounded readout / marker likelihood); needs an export step |
| Engaged-arm annotation per direction, excluding takeup, from block D widths | To implement |
| Profile prior fit per direction on a **new support grid covering 0–15.5 mm**: restore the archived `distal_cycle_prior.fit_cycle_arm` (GP conditioning plus minimum-gain projection) | To implement and formulate. This changes the filter's profile support |
| j prior from block C at fixed chassis (dp = j·dlambda) | To implement and formulate |
| Validation on pass 2: lambda error, frozen-root forecasts on achieved encoders, against the current oracle prior | To implement |

Adopting any new prior is a separate, approved change to the filter's artifacts, with a matched
replay. A full-range profile would also let the controller tendon limit rise from 10.8 mm and the
0.5 mm support extension be reviewed. A single limit is maintained once full-range data exist (user,
2026-11-02).

## Before the lab (offline)

1. Simulation check of all four blocks with the firmware-mirroring simulator, as was done for
   `tendon_start`/`tendon_reversal`: completion, limits and durations.
2. Confirm in the runner how consecutive schedules start (re-home or not) and how the insertion
   amplitude override is passed.
3. Lab checklist: `home_check` after boot, camera stall diagnosis optional, then blocks A–D.

## Offline schedule check (2026-11-02)

Generator references at 10 ms, chained from home [20, 0, 0] with the `imricor_test` limits:

| Block | Launch overrides | Duration | Insertion mm | Tendon mm | Max speed ins/rot/ten | In limits | Ends at |
| --- | --- | ---: | --- | --- | --- | --- | --- |
| A | `schedule:=stationary static_s:=30` | 30 s | 20 | 0 | 0 / 0 / 0 | yes | home |
| B | `schedule:=insertion amplitudes:=15,75,5.5 minimum_amplitudes:=12,65,4.75 repeats:=2` | 220 s | 5–35 | 0 | 5 / 0 / 0 | yes | home |
| C | `schedule:=compensated_bend_insertion_sweep` | 413 s | 0–40 | 0–15 | 2 / 0 / 4 | yes | home |
| D | `schedule:=tendon_reversal` | 76 s | 20–30.8 | 0–7.35 | 10 / 0 / 4.5 | yes | home |

Total motion is 12.3 min. Notes:
- The runner plans each run from freshly measured POS and does not re-home, so physical history
  continues across blocks.
- Block B centres on its start, so it must start at insertion 20 mm with the tendon at 0. Every block
  ends there.
- Block D's reversals reach only 7.35 mm of tendon. Block C adds full-range reversals at 0 and 15 mm.

## Simulation check (2026-11-02, passed)

Approved and run: `simulation.launch.py` in isolated domain 216 with the controller left disarmed, plus
four `causal_experiment.launch.py use_sim:=true start_motor:=true` runs back to back with the overrides
above. Sessions are in the scratchpad only, not the session root.

| Block | Wall time | Achieved insertion mm | Achieved tendon mm | Return error | Result |
| --- | ---: | --- | --- | --- | --- |
| A | 37 s | 20 (after the runner's position-controlled start move from the simulator's 0) | 0 | 0.0000 mm | Complete |
| B | 224 s | 5.10–34.89 (plan 5–35) | 0 | 0.0017 mm | Complete |
| C | 417 s | 0.05–39.87 (plan 0–40) | 0–14.90 (plan 0–15) | 0.0032 mm | Complete |
| D | 79 s | 19.99–30.70 | 0–7.26 | 0.0036 mm | Complete |

- No abort, fault or timeout.
- Rotation stayed at 0 throughout.
- The only warnings are the expected `START_MOTOR sent`.
- Each session check fails only on `/device/command_tx`, which the simulator does not publish
  (known).
- Endpoints fall about 0.1 mm short of plan in velocity streaming, including the tendon top at
  14.90 mm.
- Physical history carried across blocks without re-homing, and every block returned to [20, 0, 0].
- The simulator does not model driver reversal behaviour, load near the 15 mm limit, or camera data.
  Those need hardware.

## Decisions (user, 2026-11-02)

1. **Tendon top:** 15 mm is physically safe. Block C sweeps to the hard limit.
2. **Knob gain alpha:** the knob drives the tendon, and they cannot move separately, so alpha is not
   separately identifiable and keeps its prior. The tendon mainly produces distal deformation and
   transverse interface motion. So:
   - tendon play is identified from distal deformation;
   - the tendon interface Jacobian is regularized by the transverse prior.
3. **Insertion block:** 2 repeats (about 210 s).
4. **Validation split:** fit on block C pass 1, hold out pass 2.

## Offline fitting pipeline: formulation (2026-11-02, awaiting approval; nothing implemented)

Notation follows the [pipeline reference](GATE3_FILTER_PIPELINE_REFERENCE.md):
- u is logical tendon in mm, c is chassis (v175 driver) in mm, and lambda is distal deformation;
- d = ±1 is direction, v = d·u, and f_d(v) is the visit profile.

All inputs are measured. Images use the filter's 90 ms selection. Fitting uses **block C pass 1 only**;
pass 2 is held out.

### 1. Per-image lambda observations (reuse; no new model)

For each image k, take the filter's own marker-only readout:

$$
(\hat\lambda_k,\ \sigma_k^2) = \text{mean}[3],\ \text{var}[3]\ \text{of}\ \texttt{observe\_tip\_body\_markers}(\mathcal L, t_k, Y_k, R)
$$

- 𝓛 is the unchanged curve likelihood (v171/v174 body mechanics, bounded stations).
- R is the session's re-fitted covariance if block A passes its declared checks (spatial criterion,
  whiteness), else the October R.
- This replaces October's offline-smoothed `free_geometry` lambda with the causal per-image readout
  the filter itself uses.

**Episode labels** come from the recorded causal trace: block, pass, plateau, speed tier and
waypoint arm. Block C arms run from 0 → 15 mm (rising, d = +1) and 15 → 0 mm (falling, d = −1). The
rising arm is split at the 7.5 mm bias by a plateau's insertion move.
- **Diagnostic only:** lambda just before and after each insertion move, at fixed tendon, tests the
  model's assumption that the profile does not depend on insertion.

### 2. Tendon play from distal deformation (user, 2026-11-02)

After each tendon reversal at u_r, with deformation lambda_r, engagement onset is the first image at
which the deformation leaves its held value and stays out:

$$
k^\* = \min\{k:\ |\hat\lambda_j-\hat\lambda_r| > 3\sqrt{\sigma_j^2+\sigma_r^2}\ \ \forall j\in[k,k+2]\},\qquad w = |u_{k^\*}-u_r|
$$

- Reversals come from block C (at 0 and 15 mm) and block D (0.5–7.3 mm).
- Per direction, the width prior is location = median(w), scale = 1.4826·MAD(w), with the existing
  minimum of 0.05 mm, in the existing `widths.json` format.
- Images from the reversal to onset are takeup: they never enter the profile fit. That keeps the
  research contract that takeup does not train engaged-profile parameters.

### 3. Lambda profile prior over the full range

Per direction, on a new oriented grid:

$$
G_d = \text{sorted unique}\ \big(\{d\cdot i h\}_{i=0}^{\lfloor 15.5/h\rfloor}\ \cup\ \{d\,u_k\}_{k\in\text{train}_d}\big),\quad h = 0.05\ \text{mm}
$$

- **Cold prior:** the same GP family and hyperparameters as the current seed, constructed by the
  existing profile-model owner (affine mean gain 8 lambda/mm through (0, 0), length scale 0.2 mm,
  curvature std 8.99, gain std 80, reopen gain std 8).
- **Conditioning:** the restored `distal_cycle_prior.fit_cycle_arm`, the existing sequential GP
  update on the engaged pass-1 observations of each arm in order, then
  `constrain_engaged_profile_mean` with minimum gain 0.1 lambda/mm. Observation variance stays at
  October's 1.0 (unchanged setting; per-image σ² is a separate later comparison).
- **Support:** the profile's identified support becomes [0, 15.5] mm, against [0.0009, 10.77]
  today. The runtime's spline projection, slope-constraint locations and dense-opening domain follow
  the grid, as they do now.

### 4. Jacobian priors

- **Chassis axial direction:** b/|b| from `prior_refit axis` on block B, cross-checked on block C's
  plateau moves (same relation dp = b·dc, tip bent). The prior keeps its convention: a unit direction
  with component std 0.05. Only the direction changes.
- **Tendon interface vector:** unchanged. It stays the transverse prior
  (`interface_tendon_prior_covariance`, transverse std 0.3, axial std 0.03), as decided.
- **Knob gain alpha:** unchanged prior. It is not identifiable, because the knob drives the tendon.
- **Tip response per lambda, j_T** (new prior; today it is zero and diffuse):

  $$
  \Delta p_{T,k} = j_T\,\Delta\hat\lambda_k + \varepsilon_k \quad \text{on engaged pass-1 intervals with chassis still}
  $$

  where p_T is the marker-only tip readout. Least squares through the origin, with a bootstrap over
  intervals. The prior mean is j_T. The prior covariance is the bootstrap covariance plus a
  (0.1·|j_T|)² isotropic floor, so the prior is never more confident than one session supports.
  - **Diagnostic:** j_T fitted per lambda bin shows how far one vector is from the true local
    response.

### 5. Validation on pass 2 (declared now)

| Check | Pass |
| --- | --- |
| Lambda increment prediction on pass-2 engaged arms, anchored at each arm's first engaged image (the runtime's basic actuation response) | Within [0, 10.77] mm, RMSE no worse than the October oracle prior. Over (10.77, 15] mm, reported (October cannot predict there) |
| Engagement onset | Pass-2 onset inside the width prior's central 90% in at least 80% of reversals |
| Filter replay of block C pass 2 with the new priors (frozen-root 0.8 s forecasts on achieved encoders) | No fault over the full tendon range (the old prior faults above 11.27 mm). Forecast RMSE within [0, 10.77] mm no worse than with the October priors. Current fit no worse by more than 0.02 mm |
| j_T | Pass-2 dp_T prediction from dlambda better than zero j_T |

A failure is reported as is: no retuning of hyperparameters or thresholds. A pass qualifies these
priors for this session only. A later session tests transfer across days.

### 6. Implementation steps (atomic, each with tests and a regression on October data)

1. **Session export and labels.** Arrays, 90 ms images, per-image lambda readout, causal-trace
   labels. Regression on Oct 8: per-image lambda against the filter's initialization readout,
   bit-identical on matched images.
2. **Onset and width estimator.** A pure function. Diagnostic on Oct 8 against the October widths
   (5.44 mm falling, 8.00 mm rising).
3. **Cold prior on a declared grid.** Equivalence: built on the October grid, it reproduces the
   October seed's cold and innovation kernels bit-for-bit.
4. **Arm fitting** with the restored `fit_cycle_arm`. Equivalence: re-fitting the October oracle
   observations reproduces the stored `initialization.npz` profiles bit-for-bit.
5. **j_T fit.**
6. **Per-session prior manifest for the filter (gap C3).** Explicit artifact paths and hashes
   replace the hard-coded October paths, with the October paths as the default. Equivalence: the
   October manifest reproduces the current replay bit-for-bit.
7. **Pass-2 validation harness** (section 5).

Steps 1–7 are offline and can be built and regression-tested on October data before the lab. Only the
fits themselves need the new session.

### Decision needed

**Spline resolution over the wider support.** The runtime projects the profile onto 20 breakpoints
over the support. Over 15.5 mm instead of 10.77 mm, knot spacing grows from 0.54 to 0.78 mm, coarser
at the sharp engagement knee.
- Keeping 0.54 mm needs 29 breakpoints, which means 9 more profile states and more cost per filter
  step.
- Either way it is a representation change, so it would be compared in step 7, not tuned.

## Implementation status (2026-11-02, uncommitted)

**Breakpoints:** 20, the unchanged validated setting, is the default. It is an explicit manifest field
(`profile_breakpoints`, `fit --breakpoints`), so 29 can be compared later without editing code.

| Step | Owner | Regression / tests |
| --- | --- | --- |
| 1. Export and labels | `runtime_supervision.prior_identification export` (`ros2 run runtime_supervision prior_identification export --session DIR [--prefix /sim] [--covariance R.npz] --output X.npz`). It reuses `read_events` and `extract`, the 90 ms selection, the filter's input owners, the marker-only readout and the causal-trace episode labels | Simulator blocks A and D exported (357/357 and 780/780 valid readouts; 61 episodes labelled). About 55 ms per image |
| 2. Onset and width | `cr_meta_lnn ... session_prior_identification.tendon_arms / engagement_onset / width_prior` | Synthetic arms, onset and width tests; tool arm-table test |
| 3. Cold prior on a declared grid | `joint_initialization_bundle.profile_owner` (factored out of `load_initialization`, behaviour-preserving), `declared_grid` | **Bit-identical:** October settings on the October grid rebuild both kernels exactly |
| 4. Arm fitting | `distal_cycle_prior.fit_cycle_arm`, restored unchanged from `repository_archives/cleanup_20261102`; `fit_direction_profile` pools pass-1 engaged observations sorted by input | **Bit-identical with single-threaded BLAS:** re-fitting October's observations reproduces the stored oracle profiles (multithreaded BLAS differs by ≤ 1.5e-9). The tool fits single-threaded |
| 5. j_T | `fit_tip_lambda_response`: dp_T = a·dknob + j_T·dlambda on chassis-still intervals (the knob drives the tendon) | Synthetic recovery; collinearity refused; covariance floor |
| 6. Prior manifest | `real_native_memory_bootstrap.write_prior_manifest`, `load_native_artifacts(prior_manifest=…)`, `load_stream(prior_manifest=…)`; optional `tip_lambda_prior` replaces only the tip-J lambda column at initialization | **Bit-identical:** an October manifest reproduces the default stream exactly. The tip prior lands in exactly the lambda column (mean and covariance), with chassis and knob columns unchanged |
| 7. Validation | `prior_identification fit` and `validate` (section 5 checks, plus cold-start replays of pass 2 with the October and session manifests) | Pure helpers tested; end-to-end run pending (below) |

**Tests:** cr_meta_lnn identification 9, restored `distal_cycle_prior` 3, and 89 passed (2 skipped)
across the 11 test files touching the changed owners. Lab tools 8. Lint clean.

**Finding from block D's export.** Only 2 of 64 arms show an onset: `tendon_reversal` pulses mostly
travel less than the play width. Width priors therefore rest mainly on block C's full arms: 4 falling
and 4 rising reversals in pass 1, which is thin but meets the minimum of 3.

### End-to-end simulator smoke run, 2026-11-03: failed at the tip-response fit

Approved run on the 2026-11-02 simulator sessions (plant: October v171, so the run checks mechanics,
not accuracy). Outputs are in the session scratchpad and are not retained as evidence.

| Stage | Result |
| --- | --- |
| Export B (insertion) | 2,222 images, all readouts valid, 2.5 min |
| Export C (sweep) | 4,154 images, all readouts valid, 5.2 min |
| Fit: widths | Falling 4.40 mm (scale 1.24), from 5 C pass-1 onsets plus 1 from D. Rising 3.75 mm (scale 1.28), from 5 C pass-1 onsets |
| Fit: profiles, axis | Written. Insertion-block axis [0.174, -0.097, 0.980] |
| Fit: tip-lambda response | **Failed**: `Knob and lambda increments are collinear` |
| Validate | Not run |

**Cause: a design mismatch, not a code fault.** The approved tip-response formulation (§ offline
pipeline) regresses `dp_T = a*dknob + j_T*dlambda` on chassis-still intervals of block C pass 1. Block
C is a *compensated* sweep, so the chassis moves whenever the knob moves: of 1,325 knob-moving
intervals, none has the chassis still (corr(dchassis, dknob) = -0.86 in pass 1). The 833 chassis-still
intervals all have `dknob = 0`, so the knob column has no support.

**Diagnostics only; nothing has been changed.**

| Data and formulation | Identifiable | j_T (mm per lambda unit) |
| --- | --- | --- |
| C pass 1, three columns [dchassis, dknob, dlambda], all intervals | Yes; normalized singular values 1.60, 0.57, 0.34 | [-0.006, 0.144, -0.497] |
| C pass 1, shared axis column (dchassis + dknob) plus dlambda | Yes; normalized singular values 1.00, 1.00 | [0.025, 0.193, -0.550] |
| D, three columns; 343 knob-moving chassis-still intervals | Yes | [0.104, 0.255, -0.391] |

The fitted chassis and knob columns are far from a unit axis (for example, chassis column norm about
0.6 on C), and the C and D estimates of j_T disagree. Before choosing a replacement formulation, this
needs to be understood on the simulator. A replacement is a separate decision.

### Tip-response diagnosis on the simulator, 2026-11-03

Offline, on the existing exports A–D and the bags' `/sim/catheter_sim/ground_truth_tip`; nothing was
changed. The simulator truth is the deployed v171/v174/v175 runtime. Its v175 interface transmission is
a play operator (`CausalInterfaceTransmission`), with the same artifact hash as the filter's
(`c0796…`). Its play is ±2.69 mm on the chassis and ±2.74 mm on the knob. I replayed it over the
chained encoder streams from the simulator's start. Its driver coordinate matches the exported
chassis and knob exactly (0.0 mm). Scripts: `scratchpad/priorid/diag/`.

| Check | C pass 1 (1,845 intervals) | D (779 intervals) |
| --- | --- | --- |
| Readout tip vs truth tip, RMS | 0.0006 mm | 0.0004 mm |
| Readout tip increments vs truth, RMS | 9e-6 mm | 8e-6 mm |
| Moving intervals in take-up (chassis, knob) | 36 %, 32 % | 81 %, 93 % |
| Chassis column norm: raw driver → transmitted | 0.60 → 0.79 | 0.07 → 0.74 |
| Knob column norm: raw driver → transmitted | 0.51 → 1.25 | 0.03 → 1.32 |
| Fit residual RMS: raw driver → transmitted | 0.132 → 0.093 mm | 0.064 → 0.015 mm |
| j_T, transmitted inputs | [-0.034, 0.073, -0.368] | [0.064, 0.207, -0.370] |

j_T per tendon range, transmitted inputs:

| Block | Tendon range (mm) | lambda range | j_T | abs(j_T) | Residual (mm) |
| --- | --- | --- | --- | --- | --- |
| C | 0–5 | 4.3–39.2 | [0.104, 0.254, -0.343] | 0.439 | 0.039 |
| C | 5–10 | 11.3–68.6 | [-0.032, 0.079, -0.380] | 0.390 | 0.069 |
| C | 10–15.5 | 40.2–76.6 | [-0.141, -0.074, -0.370] | 0.403 | 0.028 |
| D | 0–5 | 4.4–32.6 | [0.067, 0.210, -0.368] | 0.429 | 0.016 |
| D | 5–7.3 | 15.5–32.9 | [0.044, 0.187, -0.379] | 0.425 | 0.012 |

**Findings:**

1. **The readout is not the cause.** The marker-only readout reproduces the truth tip to below 1 µm.
2. **Primary cause: take-up in the regression inputs.** The fit regresses on raw driver increments.
   On D, 81–93 % of the moving intervals are inside interface play, so those columns collapse toward
   zero. Replacing raw driver with transmitted motion restores comparable chassis columns on C and D
   (0.79 and 0.74). Chassis-still selection does not avoid this, because knob take-up remains.
3. **j_T depends on the operating point.** Its magnitude is stable at about 0.39–0.44, but its
   direction rotates by about 58° between low lambda (about 4–39) and high lambda (about 40–77). The
   C/D disagreement is a range difference: D (lambda ≤ 33) agrees with C's low-tendon bin within about
   0.05. A single global j_T prior averages over this rotation.
4. **The truth's knob column is not the axis.** Its norm is about 1.3, with large transverse
   components from interface rotation (v175 angular knob entry 28.3 rad per transmitted metre). Its
   chassis column norm is about 0.8, not 1. Both are properties of the October-fitted simulator plant.
   They bear on the filter's axis-shaped initial chassis and knob columns, which the filter then
   adapts online. Not acted on.

**Open decision before re-running fit and validate:**

- The input definition for the tip-response fit: excluding take-up on hardware, where the truth
  transmission is unknown.
- How to represent the state dependence of j_T in a prior.
