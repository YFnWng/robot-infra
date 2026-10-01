# Real hardware model-response evaluation — 2026-09-11

## Scope

This evaluation replays
`20260911_204642_mppi_demo` using achieved ENC feedback and four-marker
observations. It compares the fixed v174 `J0` plus v171 distal model with the
measured response over equal-duration, non-overlapping windows. It does not use
the online `response_*` diagnostics because those compared unequal time spans.

The reusable evaluator is
`audits/model-validation/evaluate_mppi_model_response.py`. Machine-readable
reports are stored with the source session:

- `real_model_response_evaluation.json` (250 ms windows);
- `real_model_response_evaluation_500ms.json` (500 ms windows).

## Data sufficiency

The 250 ms evaluation contains 212 moving windows. Its motor-increment matrix
has rank 3, singular values `[10.16, 6.95, 5.37]` rad, and condition number
1.89. The 500 ms evaluation contains 179 moving windows, also rank 3, with
condition number 2.01. Aggregate three-axis identification is therefore not
rank deficient.

Pure-axis coverage is uneven. Rotation dominates the record; only three
windows have at least 70% bending-axis purity. Bending-column conclusions are
therefore much weaker than the aggregate excitation numbers suggest.

The stationary-tip displacement p95 is 0.202 mm at 250 ms and 0.174 mm at
500 ms. Direction and gain summaries exclude observed responses below
0.25 mm, but endpoint-error summaries retain every moving-motor window.

## Full tip response

| Window | Moving windows | Measured norm p50 | Predicted norm p50 | Error norm p50 | Direction cosine p50* | Actual/model gain p50* |
|---|---:|---:|---:|---:|---:|---:|
| 250 ms | 212 | 0.114 mm | 1.659 mm | 1.615 mm | 0.382 | 0.103 |
| 500 ms | 179 | 0.115 mm | 2.771 mm | 2.710 mm | 0.590 | 0.074 |

`*` Direction/gain use only responses at least 0.25 mm: 33 samples at 250 ms
and 37 at 500 ms.

The nominal model therefore overpredicts useful observed tip response by about
an order of magnitude in the median qualifying window, and its predicted
Cartesian direction is only moderately aligned. This is much larger than the
roughly 0.2 mm stationary observation increment.

Rotation-dominant windows are the clearest discrepancy. At 500 ms, 99 windows
are rotation-dominant, but only nine exceed the 0.25 mm observation threshold;
their median actual/model signed gain is 0.050. The model predicts substantial
tip displacement for motor rotation while the observed catheter tip commonly
moves very little.

## Diagnostic interface-Jacobian fit

A least-squares fit of UKF-corrected interface-pose increments to achieved
motor-shaft increments gives the following stable patterns across the two
window lengths:

- insertion angular and linear components retain the general `J0` direction,
  but their fitted gain is only about 0.55--0.57;
- rotation angular direction remains similar, but fitted gain is about
  0.46--0.53;
- rotation linear response is inconsistent with the initialized column;
- bending linear response is close to `J0` (gain about 0.95), while bending
  angular response is poorly identified and directionally inconsistent.

Do not deploy this fitted matrix. The interface poses are outputs of a UKF
whose prior contains the same model, rather than independent interface-pose
measurements. The fit is diagnostic evidence about which blocks deserve a
dedicated excitation experiment, not a qualified replacement Jacobian.

## Conclusion

The fixed nominal model is not quantitatively consistent with this hardware
session. Marker noise alone cannot explain the scale of the full-tip error.
The bag cannot uniquely separate:

1. proximal `J0` error;
2. unmodelled mechanical transmission, backlash, or catheter slip between the
   measured motor shaft and the material interface;
3. distal-model error;
4. spatially correlated marker/registration bias.

The strongest next identification experiment is a slow, axis-separated,
bidirectional motor excitation with long holds and independent interface-pose
or full-shape reconstruction. Until then, keep online RLS disabled and use the
simulation robustness matrix to establish which mismatch classes reproduce
the observed closed-loop symptoms.

## Reproduction

```bash
cd /home/chen-lab/Yifan
source /opt/ros/humble/setup.bash
source robot-infra/install/setup.bash
source cr-venv/bin/activate

python robot-infra/audits/model-validation/evaluate_mppi_model_response.py \
  /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/\
20260911_204642_mppi_demo
```
