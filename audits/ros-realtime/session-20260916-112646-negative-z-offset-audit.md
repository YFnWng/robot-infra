# Sparse +15 mm target-plane audit — 20260916_112646

## Scope

Passive analysis of:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260916_112646_mppi_sim/20260916_112646_mppi_sim_0.db3
```

No ROS state, hardware state, source, or experiment configuration was changed.
The bag contains two eight-point sparse trials. Both froze a circle plane at
`x0 + 15 mm`, but the second trial used a different observed home shape after
the first trial's hidden tendon remanence.

## Outcome

The target-plane shift removed the preceding dominant +x reachability error,
but exposed a repeatable upper-circle z deficit. Point 1 of the first trial
reached tolerance. Points 2--4 ended below their targets:

| Point | Target z (mm) | Final target-minus-tip z (mm) | Final error norm (mm) |
|---:|---:|---:|---:|
| 1 | 60.976 | +0.508 | 1.565 |
| 2 | 68.047 | +4.668 | 6.822 |
| 3 | 70.976 | +7.862 | 9.602 |
| 4 | 68.047 | +4.511 | 6.511 |
| 5 | 60.976 | +2.357 | 3.606 |

The second trial repeated the same directional pattern. Its upper points
10--12 ended +4.582, +8.206, and +5.467 mm below target. Its lower points
13--16 ended -1.088, -0.649, -0.627, and -0.877 mm in z and were within the
1.8 mm Euclidean tolerance. This is not a constant frame or marker offset.

## Localization

### Perception is not the source

The published simulated marker tip and ground-truth tip were identical for
every paired sample. UKF post-fit marker RMS remained approximately
0.38--0.40 mm at P50/P95 during first-trial points 2--4. The sign of the final
z residual also changes between the upper and lower circle, inconsistent with
a fixed registration offset.

### The rollout predicts a nonexistent positive-z zero-input response

For points 2--4, response traces whose executed command was exactly zero
showed:

| Point | Median predicted delta z (mm) | Median measured delta z (mm) | Median measured-minus-predicted z (mm) |
|---:|---:|---:|---:|
| 2 | +1.116 | 0.000 | -1.117 |
| 3 | +1.098 | 0.000 | -1.109 |
| 4 | +1.095 | 0.000 | -1.095 |

The same approximately +1.1 mm optimistic z response persisted on nonzero
insertion/rotation forecasts. At each plateau, the planner therefore rated
zero/hold as roughly 1.7--1.9 mm closer to the target than the observed tip
actually was. For example, point 2 observed 6.822 mm error while the zero
rollout predicted 5.002 mm terminal error; point 3 observed 9.602 mm while
zero predicted 7.748 mm.

The strongest source-localized explanation is dynamic inconsistency introduced
at the marker correction boundary. UKF directly corrects interface pose and distal
strain. It does not correct the distal history state that determines the
equilibrium shift. The next encoder replay and every MPPI rollout evolve the
corrected strain toward the unchanged history-conditioned equilibrium. The
independent truth plant has a continuously evolved, self-consistent
strain/history pair, so it does not exhibit the controller's repeated
post-correction +z relaxation. The ROS node publishes that replayed corrected
state directly as the planner root.

Relevant source boundaries are:

- UKF pose/strain correction:
  `cr_meta_lnn/deployment/v171_streaming_runtime.py:698`;
- unchanged history followed by implicit strain relaxation:
  `cr_meta_lnn/deployment/v171_streaming_runtime.py:418`;
- corrected/replayed state published to MPPI:
  `robot-infra/src/catheter_control/catheter_control/node.py:930`.

The response forecast has a 40 ms nominal horizon. Median marker-to-planner
root skew was about 20 ms, but the +z error remained near 1.1 mm across much
larger skew variation. Scheduling contributes to the forecast interval but
does not explain the directionally stable zero-input mismatch.

### Tendon release is repeatedly proposed and always vetoed

After point 1, first-trial points 2--8 issued no actual or effective tendon
command. They inherited point 1's transmitted tendon remanence. Nevertheless,
MPPI did request an axis-2 reversal 24--41 times per point. None was approved.

For points 2--4, the median/max per-horizon terminal-error benefit attributed
to tendon reversal was only:

| Point | Median benefit (mm) | Maximum benefit (mm) |
|---:|---:|---:|
| 2 | 0.021 | 0.077 |
| 3 | 0.018 | 0.092 |
| 4 | 0.032 | 0.063 |

All are below the configured 0.25 mm scheduler threshold. The scheduler thus
reported `reversal_cost_margin`, held axis 2 at zero, and allowed only
insertion/rotation to act. The upper-circle targets require straightening the
remanent tendon bend; insertion and rotation alone approached the correct
y-coordinate but plateaued below the requested z.

The false +z relaxation makes this arbitration worse: the short rollout
believes much of the required upward motion will occur without tendon release,
so the incremental value assigned to reversal is artificially small.

## Root cause and priority

The primary observed defect is not the +15 mm target plane. It is a phantom
positive-z hold response in the controller rollout. Source localizes the most
likely internal cause to estimator/rollout state consistency: marker correction
changes strain without reconciling the history-conditioned equilibrium. The
reversal scheduler then amplifies the effect by rejecting the tendon release
actually required for the upper arc.

Correction order should be:

1. Make the posterior dynamically consistent by estimating/reconciling the
   relevant history/equilibrium latent together with strain, or projecting the
   corrected strain/history pair onto a rollout-consistent posterior.
2. Add a regression in which a stationary exact-model plant is marker
   corrected and a zero-command 40 ms forecast must remain stationary within
   the simulated observation floor.
3. Re-run the +15 mm sparse trial and require zero-command response-trace bias
   to be small before retuning reversal policy.
4. Then replace the tendon reversal's single 160 ms terminal threshold with a
   macro-action/cumulative-benefit test appropriate to a costly release. Do
   not simply lower the threshold while the rollout is biased.

The current run is therefore not a clean reachability verdict for upper-circle
targets and is not ready to promote unchanged to hardware.
