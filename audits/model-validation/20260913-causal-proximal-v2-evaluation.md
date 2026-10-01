# Causal proximal v2 evaluation

Session:
`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260913_172104_causal_proximal`

## Disposition

The session is usable for causal calibration. All 29 episodes completed, the
normal return completed, the manager ended `MANAGER_READY`, and firmware ended
with no latched or enabled fault. The 40 motion-watchdog messages were 21
`SUSPECTED`, seven `RETRYING`, ten `RECOVERED`, and two suspected
wrong-direction transitions. None became `MOTION_CONFIRMED`.

Every advisory interval retained finite encoder feedback with continuous
count changes. They are annotations, not invalid-data intervals. The samples
remain in the fits. A future fit may down-weight an interval only if its
measured command-to-encoder or encoder-to-marker residual is an actual
statistical outlier.

## UKF evidence

- accepted posterior traces: 10,665;
- update interval: 33.63 ms median, 66.88 ms p95;
- marker RMS before/after correction: 0.503/0.453 mm median;
- NIS: 0.466 median, 2.047 p95, 6.086 maximum;
- observable rank: nine or ten throughout;
- covariance trace: 7.094 median and 9.331 p95.

The UKF was stable and accepted the complete run. Its normalized innovation is
conservative (median NIS per 12 marker coordinates 0.039), but this experiment
does not independently separate marker covariance from model-process
covariance. Keep initial covariance `0.25` and process standard deviation
`1.0` for the first replay rather than tuning both from one trajectory.

The static-start one-second p99 interface increments were 0.237 degrees and
0.085 mm. The candidate adaptation response floors are therefore 0.30 degrees
and 0.30 mm. Static-end motion reached about 1.5 degrees and 1.1 mm over one
second, demonstrating physical relaxation after the experiment rather than
stationary camera noise.

## Jacobian result

One-second, direction-pure windows from repetitions one and two were fitted;
repetition three was held out as a complete episode-level test.

| Physical shaft | Prior RMSE | Causal RMSE | Decision | Gain vs v174 | Direction cosine |
|---|---:|---:|---|---:|---:|
| insertion / shaft 0 | 2.293 | 1.121 | replace | 0.759 | 0.975 |
| rotation / shaft 1 | 1.893 | 0.872 | replace | 0.683 | 0.991 |
| isolated bend / shaft 2 | 3.318 | 3.150 | retain prior | 0.709 | 0.946 |

Errors are RMS norms in the v174 normalized tangent coordinates. Shaft 2 has
only a 5% held-out improvement and materially stronger sign dependence, so it
does not clear a 10% replacement gate. The hybrid candidate has condition
number 50.5 versus 60.8 for v174.

The candidate artifact is
`cr_meta_lnn/evaluation/real_joint_local_distal_causal_v2.json`. It is a
shadow candidate, not an active-control release artifact.

## Adaptation configuration

Raw marker onset fits show asymmetric, history-dependent take-up. Rounded-up
per-axis normalized holdoffs are `[4.5, 6.5, 4.0]` for shafts 0, 1, and 2.
The old scalar `0.02` is far below the observed scale. The runtime therefore
supports per-axis reversal holdoffs while retaining the scalar fallback for
old launches.

The complete conservative profile is
`src/catheter_control/config/causal_v2_shadow.yaml`. It keeps adaptation and
hardware command output disabled, requires eight observations, raises the
rotation floor, tightens directional/consistency and column bounds, and keeps
the simulation-qualified MPPI settings unchanged.

A full recorded replay showed that the original covariance-SNR threshold of
3.0 admitted no complete candidate update: rotation/translation SNR p95 were
1.07/0.98 because the UKF covariance is conservative. With SNR 1.0, while
retaining the 0.30 degree/0.30 mm absolute floors, per-axis take-up, 0.90
purity, and two-window confirmation, 36 windows reached `rls_shadow_ready`,
64 supplied first confirmation evidence, eight failed consistency, and three
failed the final safety gate. No update was committed.

The final fixed-J shadow comparison passed. Both priors accepted the same
7,965 observations. Replacing shafts 0 and 1 reduced full-tip prediction
error from 1.626 to 1.226 mm mean, 1.285 to 0.870 mm median, and 3.785 to
3.349 mm p95. Episode median/p95 reductions were 38.5%/25.5% for insertion
and 46.2%/39.2% for rotation. Bending changed by only 3.4%/0.1%, as expected
because its v174 column was retained. The machine-readable comparison is
saved beside the session as `causal_shadow_comparison.json`.

An accepted online update is now strictly column-selective. For selected
axis i, the runtime forms the partial residual
`delta_xi - sum(J_j * delta_a_j, j != i)` and changes only `J_i`; independent
per-column covariance prevents covariance cross-terms from modifying an
unexcited column. This makes the implementation match the single-axis causal
gate used by the experiment and replay.

The subsequent offline commit replay made 35 such updates. It preserved UKF
acceptance (7,962 versus 7,965 samples), but did not improve the already
strong fixed causal prior: overall median error increased 0.5%, insertion p95
increased 12.9%, rotation p95 increased 2.7%, and maximum error increased
7.4%. The adaptive branch therefore has disposition `REVIEW`, and
`adaptation_enabled` remains false. The implementation is available for
further offline tuning, but is not released for command-producing hardware.

## Release sequence

1. Keep the hybrid causal Jacobian fixed for the next short, interior hardware
   trial; do not enable online adaptation.
2. Use a future causal run to tune update rate/weight or identify separate
   loading/unloading columns, then repeat the offline commit replay.
3. Promote adaptation only if every accepted column clears the per-axis
   non-degradation gates and static/reversal windows make no commit.

## Directional take-up calibration amendment

The later fixed-width hardware trial `20260913_192956_mppi_demo` showed that
the normalized RLS holdoffs must not be copied into a symmetric feedforward
dead zone. They are conservative adaptation gates, while feedforward needs
motor-direction-specific raw travel to response onset.

Reprocessing the source-timestamped fast causal-v2 repetitions used a
sustained 0.25 mm RMS four-marker displacement threshold after each reversal.
Median raw motor travel was:

| Physical shaft | Negative motor direction | Positive motor direction |
|---|---:|---:|
| shaft 0 | 6.856 rad | 8.369 rad |
| shaft 1 | 7.090 rad | 7.020 rad |
| shaft 2 | 3.587 rad | 34.858 rad |

The three-repetition samples for shaft 2 were `3.374, 3.587, 5.004` rad in
negative direction and `34.388, 34.858, 36.040` rad in positive direction.
This reproduces the known pull/release asymmetry and rules out a symmetric
shaft-2 compensator. These onset values are the corrected simulation and
short-trial candidate; width learning and adaptive-Jacobian commits remain
disabled until a second hardware replay validates them.
