# Phase-2e static Cartesian response audit

## Correction to the curvature-only interpretation

The plotted Cartesian curves motivated a direct analysis of reconstructed 3D
positions. That analysis does **not** support the earlier interpretation that
distal bending response falls by roughly 50% at intermediate insertion.

For every compensated-bending episode, learning-valid frames were selected
only when:

- measured tendon position was within 0.25 mm of 0 or 15 mm;
- commanded tendon velocity was zero;
- camera-to-causal-trace alignment was at most 20 ms.

The median 64-point curve at 15 mm tendon actuation was compared with the
median curve at 0 mm after translating both proximal points to the origin.
No rotational or nonlinear registration was applied.

| Insertion | Whole-curve RMS displacement | Tip displacement | Terminal tangent change |
|---:|---:|---:|---:|
| 0 mm | 15.260 mm | 31.193 mm | 58.23 deg |
| 13.333 mm | 15.777 mm | 31.604 mm | 55.89 deg |
| 26.667 mm | 15.869 mm | 31.738 mm | 55.78 deg |
| 40 mm | 16.071 mm | 32.058 mm | 55.59 deg |

Across the four insertion plateaus, the full-tendon-sweep response varies by
only 5.3% in whole-curve RMS displacement and 2.8% in tip displacement. The
terminal tangent change varies by about 4.5%. These are small differences and
do not resemble a 50% loss of equilibrium bending authority.

## Why the curvature statistic disagreed

The prior statistic was the difference in the median magnitude of local
curvature over the central curve samples. It is not equivalent to Cartesian
shape displacement:

1. Curvature depends on second spatial derivatives, so small spline-position
   errors or changes in spline regularization are amplified.
2. Taking a median across arclength is nonlinear. Moving the same bending
   deformation along the catheter can change the median substantially while
   leaving the Cartesian curve nearly unchanged.
3. Curvature magnitude discards bending direction and signed/vector
   cancellation.
4. The earlier endpoint selection included percentile windows from the full
   episode; the Cartesian audit uses strictly zero-command static frames.
5. At 7.5 mm, the shape depends strongly on which branch approached the bias.
   Pooling those history states can produce misleading insertion trends.

Thus, the fitted local-curvature field can differ across insertion without a
corresponding large difference in the clinically relevant Cartesian shape.

## Consequence

The current data do not justify an insertion-dependent scalar attenuation of
the v171 equilibrium tendon response. The hypothesis may still be relevant to
transient timing, branch-dependent hysteresis, or force distribution, but it
is not demonstrated by these static Cartesian endpoints.

Any subsequent model comparison should prioritize:

- full-curve Cartesian RMS and maximum point error;
- tip position and terminal tangent error;
- transient onset/settling error after tendon reversals;
- branch-conditioned results at the 7.5 mm bias;
- local curvature only as a secondary diagnostic with uncertainty.

The insertion-conditioned model correction should therefore remain
experimental until it improves held-out Cartesian rollout metrics rather than
only curvature loss.

## Artifacts

- `phase2e_static_spatial_curves_20260921.png`: base-frame and
  proximal-aligned static curves
- `phase2e_static_spatial_curves_20260921.json`: frame selection and direct
  Cartesian metrics
- `plot_phase2e_static_spatial_curves.py`: reproducible extraction and plot
