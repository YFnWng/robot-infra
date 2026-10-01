# Phase 2 explicit proximal-transmission replay

Date: 2026-09-20
Source session: `20260919_190828_causal_proximal`
Status: offline model-boundary implementation complete; online integration rejected

## Implemented boundary

The learned-model repository now contains a batch-safe causal proximal state
with distinct coordinates for:

- chassis translation after chassis-drive play;
- axial rotation;
- knob translation after the small knob-drive play;
- handle-body clamp displacement;
- effective tendon displacement;
- catheter insertion, computed as chassis plus knob translation.

The v171 scalar tendon history has a second entry point for an already
transmitted tendon coordinate. Its legacy `step(raw_tendon)` path is unchanged,
including checkpoint interpretation. The explicit path bypasses only the first
gross motor-play element and retains the static map, persistent play bank,
relaxation bank, filter, lead/rate response, force port, and distal mechanics.

## Replay contract

The Phase 2 bag was exported to:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260919_190828_causal_proximal/phase2_transmission_replay.npz`

It contains 9,588 estimator-aligned rows and 9,587 matched marker rows. The
replay injects the accepted UKF interface pose at each frame and rolls distal
history and strain open loop. Consequently, marker error tests the
motor-to-effective-tendon boundary without adding interface-pose prediction
error.

Three variants were compared:

1. legacy v171 with its learned 1.877 mm full-reversal gross motor play;
2. explicit 0.50 mm knob-drive plus 7.12 mm handle-clamp gap, with no residual
   downstream gross play;
3. explicit 0.50 mm knob-drive plus 6.87 mm handle-clamp gap and a 0.25 mm
   shrinkage residual downstream play.

The two explicit variants keep the Phase 2 identified total at 7.62 mm. The
allocation is a physical prior, not an independently identified measurement.

## Result

| Variant | held-out rep-3 marker RMS median | held-out p95 |
| --- | ---: | ---: |
| legacy v171 | 1.033 mm | 3.452 mm |
| explicit, no residual | 2.180 mm | 3.273 mm |
| explicit, 0.25 mm residual | 2.180 mm | 3.273 mm |

The explicit model reduces the held-out tail modestly but more than doubles
the median error. The 0.25 mm residual is predictively indistinguishable at
this frozen parameterization.

## Interpretation

This rejects a **width-only model swap**. The frozen v171 static and memory
parameters were learned with raw `downstream[2]` and its internal gross play.
Replacing that input with a different effective-tendon history changes the
latent coordinate and its initialization. Retaining all downstream parameters
therefore does not preserve the learned recurrence.

The result does not reject the physical decomposition. Phase 2 independently
showed that chassis and knob motion contribute differently and that a large
cascaded knob-to-distal gap is present. It shows that the new boundary must be
trained as a coherent model:

1. burn in the explicit transmission from episode prehistory;
2. refit the downstream static/memory parameters using effective tendon;
3. select clamp/residual allocation by complete held-out repetitions;
4. require improvement over legacy both in median and p95 marker error;
5. only then migrate runtime rewind, UKF reconciliation, MPPI rollout, and ROS
   diagnostics to the new state schema.

No controller, safety gate, v171 artifact, or hardware command path was
changed.

Machine-readable result:
`session-20260919-190828-phase2-proximal-transmission-variants.json`.
