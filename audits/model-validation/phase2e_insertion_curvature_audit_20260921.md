# Phase-2e insertion-stratified tendon audit

> **Interpretation update:** the curvature-only inference below is superseded
> by the direct Cartesian check in
> `phase2e_static_cartesian_response_audit_20260921.md`. Static 0-to-15 mm
> tendon actuation produces nearly invariant whole-curve and tip displacement
> across insertion. The reported local-curvature differences are real as a
> property of the fitted curvature field, but they must not be interpreted as
> a 50% reduction in Cartesian bending response.

## Scope

- Four-view recording: `20260921_174829_phase2e_video`
- Reconstructed shape: `processed_full_shape_repaired/processed_shapes.h5`
- Synchronized robot session: `20260921_174915_causal_proximal`
- Schedule: chassis-compensated tendon sweeps over nominal insertion plateaus
  0, 13.333, 26.667, and 40 mm, with two continuous-history visits per
  plateau.

The response metric is the high-tendon minus low-tendon change in median
curvature over the central 52 of 64 reconstructed distal samples. The first
and last six samples are excluded to reduce endpoint sensitivity. High and low
states are the top and bottom 8% of measured tendon position within each
episode; no state is reset between episodes.

## Integrity verdict

The dataset is suitable for analyzing insertion-conditioned curvature.

- 12,366 paired frames; 12,306 reconstruction-valid (99.515%) and 12,262
  learning-valid (99.159%).
- Median frame interval 33.327 ms; p95 33.502 ms; maximum 66.889 ms.
- Robot alignment p95 4.764 ms. The large global maximum occurs outside the
  compensated-motion windows; analysis requires <=20 ms alignment.
- Encoder and position validity are both 99.992%; command validity is 99.951%.
- Encoder/position/command age p95: 5.315/5.293/10.247 ms.
- 46 short gaps were interpolated; only 2 whole-curve temporal outliers were
  identified. The 677 terminal-only outliers motivated the central-curvature
  metric and do not dominate it.
- Median active multi-view symmetric error is 0.702 px; median cross-rig
  terminal disagreement is 1.363 mm; median temporal adjustment is 0.117 mm.
- Reconstruction quality does not degrade systematically at 40 mm insertion,
  so the response rebound there is not explained by poorer image fitting.

## Curvature result

| Nominal insertion | Visit responses (1/m) | Median response (1/m) | Relative to 0 mm | Repeat CV | Integrated turning change (rad) |
|---:|---:|---:|---:|---:|---:|
| 0 mm | 20.445, 19.420 | 19.932 | 1.000 | 3.6% | 0.776 |
| 13.333 mm | 9.185, 14.966 | 12.075 | 0.606 | 33.8% | 0.621 |
| 26.667 mm | 10.975, 8.652 | 9.814 | 0.492 | 16.7% | 0.609 |
| 40 mm | 15.005, 16.791 | 15.898 | 0.798 | 7.9% | 0.830 |

All eight sweeps covered nearly the same measured tendon span (14.63--14.83
mm), so the response differences are not caused by a truncated tendon sweep.
The plateau effect is large (one-way eta-squared 0.846); the nominal ANOVA
p-value is 0.0419 and the exact label-permutation p-value is 0.0381. These
p-values should be treated as descriptive because there are only two visits
per plateau and speed, branch order, and continuous history are coupled to the
visit design.

The relationship is **not monotone**. Across individual episodes, Spearman
rho between measured insertion and response is -0.333 (p=0.420). Response
falls through 26.667 mm and then recovers at 40 mm.

The full-shape profiles also reject a pure scalar-gain explanation. At 26.667
mm the added curvature is suppressed over the earlier distal sections and
concentrated farther along the curve; at 40 mm it increases broadly again.
Insertion therefore changes both response magnitude and its spatial
distribution.

## Model consequence

The experiment supports the original hypothesis in its broad form: proximal
extrusion changes how tendon actuation is allocated to the distal segment.
It does **not** support a monotone non-increasing scalar allocation factor
`g(insertion)` as the final model.

The next model should retain the v171 reference behavior and add a small,
smooth insertion-conditioned spatial correction, for example

```text
q_equilibrium = q_natural + B(insertion) * tendon_state
B(insertion) = B_v171 + C * phi(insertion)
```

where `phi` is a low-dimensional smooth basis and `C` is regularized toward
zero. A separate positive scalar gain may still be useful, but it cannot be
the only insertion-dependent term. Training should use the reconstructed
full curve/strain, preserve continuous history across all named episodes, and
hold out one complete visit for validation. Because the 13.333-mm repeat is
variable, the next collection should independently counterbalance speed and
branch order at each plateau before interpreting the function as a static,
memory-free insertion law.

## Artifacts

- `phase2e_insertion_curvature_audit_20260921.json`: machine-readable metrics
- `phase2e_insertion_curvature_audit_20260921.png`: response, spatial profile,
  motor-curvature, and integrity plots
- `audit_phase2e_insertion_curvature.py`: reproducible audit script
