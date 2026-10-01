# v175 no-rotation two-axis hardware audit: 20260920_175149_mppi_demo

## Verdict

**PASS as a rotation-disabled closed-loop controllability gate.** All four
independent model-generated targets entered the 1.8 mm tolerance, with no
controller fault, manager inhibition, marker-rejection streak, or planner
deadline miss. Logical and physical rotation remained exactly zero.

**Not yet a clean four-candidate forward-model validation.** The four targets
were frozen from the first home estimator state, while physical tendon/distal
history was preserved across later encoder homes. Consequently the later home
tips changed substantially. In particular, candidate 3 was reached with zero
tendon motion and therefore did not exercise its intended coupled
insertion/tendon actuation.

## Closed-loop result

| Target | Intended preview | Initial error (mm) | Minimum error (mm) | Final error (mm) | Active time (s) | POS change axes 0/1/2 |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 1 | insertion dominant | 4.054 | 0.796 | 0.874 | 1.053 | `[+5.885,0,+0.016]` |
| 2 | tendon dominant | 8.147 | 1.228 | 1.232 | 1.353 | `[-8.311,0,+3.194]` |
| 3 | positive insertion/tendon diagonal | 2.328 | 0.976 | 1.450 | 0.651 | `[+3.280,0,0]` |
| 4 | negative insertion/tendon diagonal | 9.939 | 1.270 | 1.270 | 1.804 | `[-11.856,0,+4.142]` |

Each action accumulated six in-tolerance feedback samples and returned
`reached=1, timed_out=0`. The experiment reported `4 reached, 0 timed out`.

The initial errors differ from the preview magnitudes
`[4.034,3.760,2.363,6.937] mm` because all absolute targets were frozen from
the first home tip `[24.816,19.211,68.742] mm`, while later encoder homes did
not reset distal history. This effect is largest on targets 2 and 4.

## Rotation isolation and limit evidence

- Maximum absolute logical rotation in 53 planned commands: exactly `0`.
- Maximum absolute manager-forwarded rotation in 491 active velocity samples:
  exactly `0`.
- Rotation POS and ENC remained exactly zero throughout all active intervals.
- Active axis-0 POS stayed in `[7.436,25.883] mm`, well inside the
  `[-10,50] mm` hard range and the autonomous insertion margin.
- Active tendon POS stayed in approximately `[-0.0013,4.144] mm`; the small
  negative sample is within the configured `0.002 mm` feedback-only
  quantization tolerance. Commands remained inside the exact domain.
- Raw encoder channels remained inside their model-valid envelopes. No limit
  or encoder-integrity event was recorded.

All active diagnostics reported the required identity:
`command_output_enabled=True`, UKF estimation, adaptation disabled,
`controller_velocity_max=[10,0,4.5,4,25,25]`, and CUDA with 1,024 samples.

## Commands, take-up, and reversal behavior

Every target used a short insertion take-up transaction. Status samples
reported `TAKEUP_ACTIVE` for 2/12, 1/15, 1/8, and 4/19 samples respectively;
the active mask was always `[1,0,0]`. Tendon transmission was observed through
the distal/tendon history path rather than a response-free axis-2 transaction.

Each target had one insertion sign change in planned and manager-forwarded
velocity. Tendon command did not reverse. Meaningful tendon POS travel occurred
only on targets 2 and 4. Target 3's tendon POS and encoder count were constant,
so its success must not be cited as a causal validation of the positive
two-axis diagonal.

## Model-response comparison

Forty causal forecast/measurement comparisons were recorded:

- endpoint error P50/P95/max: `0.879/1.814/1.908 mm`;
- valid direction cosine min/P50/P95: `-0.758/0.794/0.993`.

The corresponding qualified simulation had endpoint error
`0.252/0.482/0.495 mm` (P50/P95/max). Hardware therefore retained closed-loop
convergence but showed a clear model-to-hardware gap. Targets 2 and 4 were the
largest contributors: their mean measured-minus-predicted response biases were
approximately `[-0.456,-0.535,+0.361] mm` and
`[-0.509,-0.603,+0.271] mm` per comparison. Their worst response errors were
`1.822` and `1.908 mm`.

Negative direction-cosine samples occurred on targets 2 and 3. They did not
form a persistent divergence or fault: subsequent camera feedback corrected
the state and all targets converged. This result supports feedback robustness,
not equality of the v175 preview and physical transmission.

## Estimation and timing

- Estimator health was `TRACKING` for all 76 active estimator samples.
- Consecutive marker rejections remained zero.
- Marker diagnostics stayed `TRACKING` for every active status sample.
- Dual-camera timestamp skew was P50/P95/max
  `6.429/6.675/6.707 ms`; marker-processing latency was
  `4.092/4.881/6.803 ms`.
- Marker correction plus rewind/replay total update time was P50/P95/max
  `44.90/57.95/63.15 ms`. These updates exceeded the estimator timer period
  but did not starve planning or trip freshness gates in this short run.
- First-plan times for targets 1--4 were
  `45.434/43.795/44.357/52.290 ms` against the 60 ms deadline.
- The largest recorded rolling plan-duration maximum was `54.670 ms`;
  maximum consecutive deadline misses was zero.

The only runtime warnings were one startup
`observation_before_rewind_buffer` rejection before initialization and one
serial diagnostic token `Lnnnbn` during target 1. Neither affected readiness,
feedback validity, or completion. The manager safety topic remained
`MANAGER_READY` for the entire bag.

## Interpretation and next gate

The run demonstrates that grouped MPPI can use insertion and tendon, with
rotation disabled, to converge on four sparse Cartesian targets despite the
hardware model gap and preserved distal history. It is a major improvement
over the earlier no-rotation trials.

Before treating the result as validation of all four local model directions,
change the isolation runner to generate each candidate **after its own guarded
home**, from that target's current accepted estimator/history snapshot. Keep
the candidate displacement list fixed, but do not freeze all absolute
Cartesian targets from the first home. Then repeat at least the positive and
negative coupled candidates and require meaningful motion of both active
joints. If that gate passes, construct a conservative planar continuous path
inside the measured two-axis envelope; do not yet re-enable rotation.
