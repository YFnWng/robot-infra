# 2026-09-19 rotation-disabled axial Jacobian A/B

Compared sessions:

- v174: `20260919_171058_mppi_demo`
- causal-v2: `20260919_171917_mppi_demo`

Both runs used the same sparse-point experiment: home to encoder
`[20,0,0]`, track a frozen `+5 mm` base-Z target, home again, and track a
frozen `-5 mm` base-Z target. Rotation velocity was zero. Online adaptation
was disabled.

## Runtime identity

Recorded `/parameter_events` confirm that both runs used UKF estimation,
CUDA, 1024 samples, grouped MPPI, backlash compensation, take-up
transactions, and `controller_velocity_max=[10,0,4.5,4,25,25]`.

- v174 loaded `real_joint_local_distal_v174.json`.
- causal-v2 loaded `real_joint_local_distal_causal_v2.json`.

The launch manifest's `controller_parameters` and top-level Jacobian artifact
still describe pre-overlay launch defaults, so they are not authoritative.
The controller-profile entry plus recorded parameter events establish runtime
identity.

## Closed-loop outcome

| Metric | v174 | causal-v2 |
| --- | ---: | ---: |
| +Z duration | 1.774 s | 1.063 s |
| +Z action final error | 0.723 mm | 1.066 mm |
| +Z marker displacement | `[+0.360,+0.552,+5.730] mm` | `[+0.286,+0.135,+6.198] mm` |
| +Z POS delta, first 3 axes | `[+10.978,0,+0.462]` | `[+7.483,0,0]` |
| -Z run-start error | 7.021 mm | 10.383 mm |
| -Z duration | 1.433 s | 1.945 s |
| -Z action final error | 0.785 mm | 0.769 mm |
| -Z marker displacement | `[-0.333,-0.365,-7.587] mm` | `[-0.725,-0.381,-10.784] mm` |
| -Z POS delta, first 3 axes | `[-7.279,0,+2.331]` | `[-10.006,0,+4.544]` |

Both runs reached both targets, had one late insertion reversal per target,
kept rotation at zero, and had no planner deadline misses or controller
faults. The negative-Z motion used both retraction and tendon actuation in
both runs.

The runs did not start from the same hidden physical state. After the second
encoder home, the v174 run required about 7.0 mm of tip motion to its frozen
negative target, while the causal-v2 run required about 10.4 mm. Completion
time and motor travel must therefore not be interpreted as a controlled model
comparison by themselves.

## Prediction and local-column evidence

Offline 200-ms replay windows produced the following diagnostic fits:

| Quantity | v174 run with v174 | causal run with causal-v2 | causal run replayed with v174 |
| --- | ---: | ---: | ---: |
| insertion linear direction cosine | 0.9997 | 0.9998 | 0.9999 |
| measured/J insertion linear gain | 0.715 | 1.718 | 0.854 |
| full-tip prediction error P50 | 0.556 mm | 0.478 mm | 0.669 mm |
| full-tip direction cosine P50 | 0.842 | 0.802 | 0.848 |

The interface-pose fit is UKF-derived rather than independent ground truth.
Nevertheless, it gives strong evidence that both insertion columns point in
the correct direction. On the same causal-v2 trajectory, the v174 insertion
gain was substantially closer to the inferred interface response: `0.854`
versus `1.718` measured/model for causal-v2. The causal-v2 insertion column
therefore under-predicts local interface translation by roughly 42% in that
trial.

The causal-v2 full-tip P50 error was smaller on its own trajectory despite its
worse insertion-column gain. That aggregate includes tendon actuation, distal
history, nonlinear rollout, and a Jacobian-dependent UKF posterior; it is not
an isolated insertion metric.

## Scheduling and estimator health

| Metric | v174 | causal-v2 |
| --- | ---: | ---: |
| active-plan elapsed P50 | 45.355 ms | 43.031 ms |
| active-plan elapsed P95 | 49.552 ms | 49.643 ms |
| active-plan elapsed max | 52.462 ms | 49.680 ms |
| nonzero consecutive deadline misses | 0 | 0 |
| maximum marker rejection streak | 1 | 1 |

There is no evidence that scheduling or estimator rejection determined the
different motions in this comparison.

## Conclusion

1. The earlier no-rotation failure is not reproduced by this axial test with
   either Jacobian.
2. The v174 insertion column is not wrong in direction and is closer in local
   gain on the causal-v2 run's recorded motion.
3. Causal-v2 can still close the loop on these simple axial targets, so its
   earlier failure cannot be assigned solely to its insertion column.
4. The remaining likely causes are target geometry, history-dependent
   insertion/tendon coupling, and temporal actuation effects that become
   important on mixed-axis targets.

A statistically useful hardware comparison needs replicated, order-balanced
trials (for example ABBA) and conditioning on the measured run-start tip and
hidden-history proxies, because encoder homing does not reset physical tendon
or torsional state.
