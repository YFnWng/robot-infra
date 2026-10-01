# Rotation-disabled two-DOF hardware audit (2026-09-16 18:53:39)

## Scope

Read-only analysis of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260916_185339_mppi_demo/20260916_185339_mppi_demo_0.db3`

The bag has no `metadata.yaml`, but its SQLite integrity check passes. The
controller status and startup log both report velocity limits
`[10, 0, 4.5, 4, 25, 25]`, ordered insertion, rotation, tendon, then sheath
axes. Recorded planned rotation velocity and recorded rotation position change
were exactly zero in every analyzed target interval.

Each sparse target was issued as a separate one-waypoint action, so action
feedback was segmented using the three `/catheter_mppi/target_tip` publication
times rather than `waypoint_index` (which is zero for all three actions).

## Empirical bending plane

An affine plane fit by SVD to 702 accepted marker-tip samples from the three
active target intervals had:

- normal `[0.6934, -0.7206, 0.0026]` in the base frame;
- RMS tip residual `0.367 mm`;
- P95 absolute residual `0.825 mm`.

This is strong evidence that the rotation-disabled tip motion stayed on one
approximately planar reachable set. It is an empirical local plane, not a
proof that the full nonlinear reachable manifold is globally planar.

## Target results

Errors are decomposed along the fitted plane normal and inside the fitted
plane. `Plane minimum` is the target's perpendicular distance to the fitted
plane. `Best` is the smallest measured Euclidean target error during the
target interval.

| Point | Duration | Plane minimum | Initial total / in-plane | Best total / in-plane | Final total / in-plane |
|---|---:|---:|---:|---:|---:|
| 1 | 10.02 s | 16.34 mm | 16.71 / 1.40 mm | 16.37 / 2.91 mm | 21.94 / 14.41 mm |
| 2 | 10.02 s | 18.47 mm | 19.97 / 7.29 mm | 18.01 / 1.67 mm | 20.92 / 10.63 mm |
| 3 | 3.46 s | 23.57 mm | 26.95 / 11.90 mm | 23.84 / 6.06 mm | 26.09 / 11.24 mm |

Point 1 was already close to the plane projection at the beginning. Point 2
passed close to its plane projection after 2.57 seconds. Point 3 reduced its
in-plane residual before the encoder-range fault, but never got closer than
6.06 mm in-plane. None of the three actions ended at its closest observed
reachable point.

## Actuation and response

Rotation isolation was successful:

- planned rotation maximum absolute velocity: `0`;
- measured rotation POS range: `0`;
- insertion and tendon were active and commonly reached their configured
  maxima of `10 mm/s` and `4.5 mm/s`.

The remaining axes did not settle cleanly:

- Point 1 insertion/tendon position total variation was `59.18/22.77 mm`,
  with `7/3` command sign changes.
- Point 2 insertion/tendon position total variation was `68.15/10.27 mm`,
  with `2/0` command sign changes.
- Point 3 ended after 3.46 seconds on `encoder_feedback_out_of_range`.

The recorded first-action response comparisons also show weak directional
agreement between model and hardware:

| Point | Predicted response P50 | Measured response P50 | Direction cosine P50 | Negative cosine |
|---|---:|---:|---:|---:|
| 1 | 0.844 mm | 0.774 mm | -0.089 | 54.3% |
| 2 | 0.709 mm | 1.068 mm | -0.049 | 51.2% |
| 3 | 0.778 mm | 1.347 mm | +0.244 | 26.7% |

The response magnitudes were not negligible, but their directions often
disagreed with the model. This supports a remaining insertion/tendon model or
history mismatch rather than a lack of physical response.

## Conclusion

The hypothesis is only partially supported. Disabling rotation successfully
confined measured tip motion to a stable bending plane, and points 1--2 did
transiently pass close to the closest reachable point in that plane. The
controller did **not** converge and remain there: it continued aggressive
insertion/tendon motion, overshot the planar optimum, and finished points 1--2
farther away than their best transient states. Point 3 faulted before a full
trial.

Therefore rotation transmission is not the only blocker. With rotation held
fixed, the insertion/tendon controller and forward-response model still fail
to reliably minimize the reachable in-plane error.
