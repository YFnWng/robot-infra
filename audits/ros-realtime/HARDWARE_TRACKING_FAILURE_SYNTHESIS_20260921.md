# Hardware tracking failure synthesis

Date: 2026-09-21

## Question

Simulation passed both sparse-target and continuous-path tracking. Which
hardware effects explain the remaining grouped-MPPI failures, and is replacing
the v171 distal mechanics the highest-priority correction?

## Executive conclusion

Do not replace the v171 equilibrium/distal mechanics first. The hardware data
show that the controller can close the loop with v171 on short axial and local
two-axis targets. The failures appear when reversals and hidden transmission
history dominate. Two measured effects are large enough to invalidate the
rollout even when the static catheter shape model is good:

1. Shaft-2 reversal-to-distal-turnaround travel is usually about 5.3--6.8 mm,
   while the deployed v171 motor play turns after roughly 1.46 mm and its
   predicted equilibrium trend turns even earlier.
2. Axial insertion releases stored torsion. With the rotation encoder fixed,
   one insertion cycle changed posterior interface roll by a median 22.84
   degrees and raw-marker bending-plane angle by a median 17.94 degrees.

These are different states. The first belongs to the tendon command-to-distal
transmission boundary. The second is an insertion-coupled torsional reservoir
upstream of the distal catheter. Neither should be absorbed into a constant
local Jacobian or an altered v171 equilibrium force field.

## Evidence across hardware sessions

| Session | Isolation/result | Relevant observation |
| --- | --- | --- |
| `20260919_171058_mppi_demo` | Rotation-disabled axial `+Z/-Z` | Both targets reached at 0.723/0.785 mm. The v174 insertion column had direction cosine 0.9997 to the fitted response; its direction was not wrong. |
| `20260920_165501_mppi_demo` | Rotation-disabled axial gate | Both targets reached at 0.671/0.637 mm. Response endpoint error P50/P95 was 0.668/1.028 mm; estimator and planner remained healthy. |
| `20260920_175149_mppi_demo` | Local two-axis sparse gate | All four targets reached at 0.874--1.450 mm with exactly zero rotation command and encoder travel. Hardware forecast error P50/P95/max was 0.879/1.814/1.908 mm versus 0.252/0.482/0.495 mm in matched simulation. |
| `20260920_182044_mppi_demo` | Farther two-axis target | Point 2 passed within 1.005 mm but would not settle. At the model preview joint endpoint the hardware tip remained 3.275 mm away; closest approach needed about twice the previewed tendon travel plus a large insertion change. In 121 response windows, 33.9% of measured responses opposed the prediction. |
| `20260920_185221_causal_proximal` | Isolated tendon motor | Nine of 12 reversals had distal turnaround after 5.30--6.84 mm; overall median was 5.305 mm. Raw marker geometry and UKF distal bending agreed at correlation 0.9943. |
| `20260920_191451_causal_proximal` | Insertion/rotation coupling | All 12 insertion probes released roll with the sign of the prior rotation preload while rotation and tendon encoders were fixed. Median absolute full-cycle roll change was 22.84 degrees; raw-marker result was 17.94 degrees with correlation 0.9987 to UKF roll. |
| `20260916_165943_mppi_demo` | Continuous hardware path | 162 take-up transactions occurred in about 162 s; rotation transaction directions alternated 80 times. Rotation take-up moved the tip 1.516 mm median and up to 6.128 mm. Path progress was held for 75.5% of active updates. |
| `20260916_172704_mppi_demo` | Continuous hardware path | An insertion-dominant transaction moved the tip 7.312 mm and rotated local interface Z by -3.24 degrees, yet was rejected as unconfirmed because it disagreed with the nominal insertion Jacobian direction. |
| `20260920_163629_mppi_sim` | Corrected continuous simulation | Completed 100% of the circle. Forecast endpoint error P50/P95/max was only 0.030/0.198/0.782 mm and the entire path used only 3 nonzero rotation sign reversals. |

## Attribution

### 1. Tendon reversal timing is a real v171 rollout deficiency

This is observed, not inferred from MPPI behavior. The isolated tendon session
shows that after a shaft-2 reversal the physical distal shape normally keeps
moving in its old direction for several millimetres longer than v171 predicts.
That error can make MPPI price a reversal as immediately useful, then issue a
new plan before the distal catheter has actually turned around.

The result does not imply that the v171 spatial force field is wrong. Its
equilibrium and coherent-rollout shapes can remain accurate while the upstream
effective tendon coordinate reaches that force field too early.

### 2. Hidden torsional release is absent from the rollout

The insertion/rotation experiment proves that insertion is not conditionally
independent of rotation history. A fixed insertion Jacobian cannot represent
the effect because its sign follows the stored rotation preload and its
magnitude decays over repeated insertion legs. Simultaneous insertion and
rotation commands also did not reliably prevent stored twist in earlier path
data.

This explains why a fresh plan can reverse rotation after an insertion move:
the measured tip and interface roll have changed through an unmodeled state,
not merely through the commanded insertion column.

### 3. Reversal chatter is primarily a consequence of prediction error

Grouped MPPI reduced gratuitous reversals in simulation. On hardware, the far
two-axis target generated 17 insertion and 10 tendon sign reversals because
the observed short-horizon response alternated between overshoot and apparent
under-response. At the target's model-generated endpoint, the hardware was
still 3.275 mm away. The optimizer was repeatedly correcting a moving model
error, not simply failing to penalize switching.

Additional reversal cost may suppress symptoms, but excessive cost would also
prevent necessary corrections. The missing transmission states should be
corrected before another major cost sweep.

### 4. Perception and compute are not the primary cause in the clean gates

During the successful local two-axis hardware run, the estimator stayed
`TRACKING`, marker rejection count stayed zero, and planner deadline misses
stayed zero. During the failed far tendon target, marker RMS remained below
0.87 mm and the estimator stayed healthy. There are real historical gating and
latency defects, but they do not explain the reproducible model/plant endpoint
mismatch.

### 5. Target reachability and history still matter

Encoder homing does not reset distal remanence or torsional history. Targets
frozen from the first home can therefore cease to represent the same relative
motion after later homes. Some circular segments were also outside the
available workspace. These effects must be separated from model accuracy in
future comparisons.

## Recommended next analysis (no new hardware required)

Use the recorded sessions for a boundary-by-boundary counterfactual replay:

1. **Distal-only oracle replay:** feed v171 the recorded UKF interface pose and
   a reversal-delayed effective tendon coordinate fitted from
   `20260920_185221`. Measure reversal-local maximum Cartesian error, onset
   error, and 0.2/0.5/1.0 s forecast error.
2. **Torsion oracle replay:** replace predicted material roll with the recorded
   posterior roll in `20260920_191451` and failed path windows. This bounds the
   error attributable to the missing torsional reservoir.
3. **End-to-end replay:** restore predicted motor-to-interface transmission and
   compare the error increment against the two oracle cases. This identifies
   how much remains in chassis/knob transmission versus distal mechanics.
4. **Planner counterfactual:** rescore the recorded grouped candidates with
   each corrected rollout. Count whether the selected first-action sign and
   reversal decision change. Do not judge only by aggregate marker MAE.

Primary gates should be maximum and P95 Cartesian error around reversals,
response-direction error, false-benefit reversals, and candidate-rank changes.
Mean trajectory error alone can hide the short intervals that corrupt MPPI.

## Decision

Keep v171 as the frozen spatial distal baseline while adding and validating
the two missing upstream history states. Revisit the distal mechanics only if
the distal-only oracle replay still has material reversal-local Cartesian
error after using the recorded interface pose and a correctly delayed
effective tendon coordinate.
