# Gate 3 forecast spike diagnosis

2026-11-01. Read-only replay of the reduced research filter on the whole recorded session: physical gain, K=2, images 3-1224. No estimator,
hardware or default changed. Forecasts use physically achieved future encoders and no future markers. Later data are used for scoring only.

Scored against the marker-only tip readout, which is not physical truth. Decomposition terms use root parameters and later filter lambda, so they
are diagnostic, correlate, and need not sum. Share figures use the 202 regular-stride roots (RMSE 0.974 mm), without extra roots placed around known spikes.

## Error is concentrated

The worst 10% of roots hold 85% of squared error. Mean squared error by term: deformation 0.703, axial transmission 0.182, response change 0.137,
root position 0.059, total 0.948 mm².

| Situation over 0.8 s | Roots | Share of squared error | RMSE, mm | Within 1 mm |
|---|---:|---:|---:|---:|
| Engaged tendon only | 16 | 34% | 2.02 | 69% |
| Engaged tendon with chassis motion | 12 | 21% | 1.82 | 75% |
| Tendon takeup crossing engagement | 6 | 20% | 2.53 | 33% |
| Chassis only | 40 | 8% | 0.62 | 85% |
| Startup, deformation response unlearned | 11 | 7% | 1.09 | 73% |
| Tendon takeup, no engagement in horizon | 31 | 4% | 0.49 | 94% |
| Tendon reversal in horizon | 2 | 4% | 1.91 | 50% |
| Held | 84 | 2% | 0.24 | 100% |

So about 79% of squared error is in roots where the tendon moves. The classes are small and the classifier is rough.

## Mechanisms seen

**Lambda forecast.** It is the largest term at the worst roots, off by up to 18 model units at the horizon end. This forecast comes from the unchanged distal owner, and the reduced filter's distal state matches the scalar filter within 5e-8 on the
scored segments, so these spikes are inferred not to come from the real-time restructuring. A scalar forecast at these exact roots was not run.

Lambda against tendon nearly repeats between visits: long flat takeup, then a sharply nonlinear rise, with engagement near 8.2 mm rising and 4.2 mm falling. Apparent slope differences between visits are therefore mostly position along one curve. Examples of forecast
and observed slope in lambda per mm: root 766 7.4 and 26.8; root 813 27.0 and 9.6; root 974 20.0 and 7.3; root 656 12.1 and 3.5.

Three candidate causes, addressed below: forecast into tendon range this direction has not yet reached (root 766 starts at the previous peak); a profile update at a knee (root 813);
and engagement timing (root 656 forecast starts falling about 4 images early). Root 801 forecasts no change while in takeup, as already recorded in `GATE3_CAUSAL_PREDICTION_ERROR_AUDIT.md`.

Earlier work bears on this. `GATE3_RECORDED_VISIT_PRIOR_TRANSFER.md` found that reusing an earlier adapted loading function helps, conditionally. The runtime already keeps a per-direction bank.

**Chassis takeup against calibrated width.** Chassis travel before the readout moves 0.35 mm axially, after removing deformation:

| Start | Kind | Travel at contact, mm |
|---:|---|---:|
| 446 | Reversal | 5.56 |
| 577 | Reversal | 4.58 |
| 687 | Reversal | 4.76 |
| 842 | Reversal | 4.60 |
| 884 then 936 | Reversal, stopped after 0.97 mm, resumed | 0.97 + 3.75 = 4.72 |
| Same-direction restarts, quiet | Control | 0.29-1.28 mostly |

The threshold itself accounts for about 0.35 mm of each value, and same-direction restarts include 936, which is listed above as a resumed reversal. The user reports that 7.05 mm was calibrated on an interval with chassis motion only and is accurate. A later section tests this; the short onset is not evidence for a smaller width.
Root 937 shows a related cost: a wall probability of 1.0 during takeup makes the forecast move immediately, for 3.07 mm of axial error.

**Startup.** Up to about image 120 the deformation response is still zero (roots 68-70), so tendon motion is not attributed to deformation. Initialization and insufficient excitation are reported separately under the pipeline contract, and this is not a control-time spike.

**Knob and tendon coincide.** Over this recording, knob and tendon increments have correlation 1.0 and ratio 1.0. If these are one physical motion here, this recording cannot separately identify knob gain from tendon-driven response. That may contribute to the response-change term and the thin positivity margin. Not established.

## Follow-up tests

**Width.** Whole session at 4.5 mm against 7.05 mm, nothing else changed: regular-stride RMSE 0.971 against 0.974 mm, 88.1% within 1 mm for both, 8 roots worse and 3 better by over 0.1 mm, with roots 936 and 938 worse by about 0.9 mm. No case for changing the width. It remains 7.05 mm.

**Reconciling 7.05 mm with the earlier onset.** The cleanest case is the reversal at 446, with no knob motion since the previous chassis motion and 22.8 mm of travel. Tip gain per 1 mm of travel, after removing deformation, is about 0 through 3 mm, 0.04-0.25 through 5 mm, 0.41 at 6 mm, then mostly 0.7-0.96 from 7 mm, with one 0.24 bin. So response starts near 4-5 mm and approaches full gain near 7 mm. This fits both a 7 mm takeup and an earlier first response, suggesting gradual rather than abrupt contact. One clean reversal; 842 shows 0.5-0.6 from 4-7 mm with noisy later bins; 577 and 687 travel under 5 mm.

Knob motion between reversals: 0.00 at 446, 0.70 at 577, 10.77 at 687, 4.15 at 842, while first response stays 4.6-5.6 mm. So first response does not scale with knob travel, which argues against a simple knob-proportional cause, on five events. A mid-gap starting memory does not obviously explain 446, 577 or 842, where contact had been reached, though contact itself may be gradual. 687 is marginal. Not settled.

**Lambda forecast.** 68 regular-stride tendon-moving roots, error in lambda at the horizon end:

| Class | Roots | Forecast error | Earlier same-direction visit, same span, error |
|---|---:|---:|---:|
| Span covered by an earlier visit | 41 | 4.42 | 2.31 |
| Range not reached earlier | 27 | 4.17 | none available |
| Covered, engaged and continuing | 7 | 4.90 | 1.36 |
| Covered, takeup crossing engagement | 4 | 7.07 | 3.25 |
| Covered, takeup with no engagement | 26 | 1.67 | 0.26 |
| Covered, tendon reversal | 2 | 3.69 | 9.00 |

The earlier-visit increment comes from past data only, so it is causal, but the classes are small, overlap in time, and the comparison was not a filter run. As the user notes, the profile does change within a session and the GP cannot predict unseen change; 27 of 68 roots are in unreached range. These results do not show earlier visits are reliably better, and reversal roots are worse.

Profile snapshots show what the forecast actually uses. The per-direction bank does carry, so the mean is not reset. Two effects appear:

- **Update near a knee.** Over span 4.15 to 3.10 mm, the falling-direction profile increment is about -9.5 from image 628 through 793, close to -10.2 observed at 813. It reaches -20.9 by image 805 and -28.4 by 813, with the tendon at 4.15 mm. Root 974 shows the same jump, -6.2 to -23, with -7.5 observed. Over span 4.73 to 3.70 mm, image 805 contributes -12.9 while lambda falls 4.8 across 0.19 mm of tendon, and images 806-814 contribute about -10.7 more at a nearly stationary tendon. Images 817-819 correct by +9.3 once that range is actually traversed.

  Most of the change is in coefficients within the observed point's spline support, whose span of about 2 mm overlaps the later range, and little comes from coefficients outside it: at most about -0.5 per image on this span, larger on the shorter span. So it is mainly local spline reach, not long-range prior correlation, though coefficient correlations within that support are about 0.99. During images 806-814 the chassis also moves near 807-810, so stationary-tendon lambda change may be relaxation, chassis coupling or both. Not separated.
- **Engagement onset.** The width is not static: the owner updates it after confirmation.

| Falling visit | Width in use | Tendon travel at first fast lambda change | Onset minus width |
|---|---:|---:|---:|
| 628 | 5.44 | 6.55 | +1.11 |
| 774 | 6.12 | 5.88 | -0.24 |
| 944 | 6.08 | 6.37 | +0.29 |

  Rising visits differ by only +0.15 to +0.36 mm, by a coarser lambda-departure threshold. So early onset is mainly a first falling-visit startup effect, explaining root 656 (about 1.1 mm early, about 4 images). It is not the main cause at later spikes such as root 974.

**Visit carry.** The distal owner initializes a new visit from the last same-direction visit, as the user noted: `_switch` restores the direction's bank, and `_prepare_visit_prior` adds an independent zero-mean covariance once per visit (value std 2.0, length 2.0 mm, persistent gain std 1.29), leaving the mean unchanged. Snapshots confirm the mean carries. So the earlier-visit comparison above does not reveal a missing mechanism. It reflects later local updates, untrained takeup range, or real change.

## Stationary-tendon profile hold

**Hypothesis.** Part of the knee overreach comes from profile updates while the tendon is not moving, so holding them improves forecasts where the tendon moves.

**Only change.** Opt-in `learning_minimum_travel_mm=0.01`, reusing the existing `JacobianMotionAdmission.minimum_mm` floor. Below it, profile mean and covariance are protected. Per-image travel is bimodal, about 0 or 0.2 mm, so the value is not sensitive. Off by default in the owner. It narrows the contract that engaged observations train the current visit's profile; the user accepted that on 2026-11-01 for the real-time filter path, with `scripts/reduced_filter_session.py` defaulting to 0.01 mm. The scalar reference and ROS runtimes do not select it. To be rechecked on the next independent recording.

**Checks declared before running, and results:**

| Check | Result |
|---|---|
| Gate off matches archive exactly | Strictly failed: differs by at most 5.8e-8 mm, from 7e-15 at image 53. Repeats from one checkpoint are bit-identical, and the archived runs agree with each other. Trigger not identified; far below the 0.03 mm decision size |
| Short segments: no rejections; lambda unchanged on first held image | Passed. Tip and lambda are bit-identical through images 805-816 |
| Short segments: roots 813 and 974 improve | Mixed. 813 went 6.22 to 3.76 mm. 974 was unchanged from the root at 955, because no stationary learning occurs in that segment before it |
| Whole session: regular-stride RMSE falls at least 0.03 mm | Passed: 0.974 to 0.885 mm |
| More roots better than worse by over 0.1 mm | Passed: 8 against 6. Including the extra spike roots it is 12 against 12 |
| Held-tendon roots worsen at most 0.02 mm | Passed: 0.234 unchanged |
| P99 at most 50 ms, no rejections, current fit no worse | Passed: 48.1 ms alone, 1,221 images, fit 0.241 mm unchanged |

The 76.6 s run with forecasts overlapped another replay, so its timing is void. The separate timing run (44 s, no forecasts) has P50 35.3 ms and max 270 ms, against 106 ms for the archived baseline. Images 44, 717, 774 and 885 took 118-270 ms against 51-57 ms there. These are not explained; the gate adds no computation, and the baseline was not rerun under the same conditions.

| Regular-stride roots | RMSE, baseline | RMSE, held | Within 1 mm, baseline | Within 1 mm, held |
|---|---:|---:|---:|---:|
| All, 202 | 0.974 | 0.885 | 88.1% | 89.6% |
| Tendon moving, 68 | 1.583 | 1.420 | 73.5% | 77.9% |
| Chassis only, 39 | 0.638 | 0.639 | 84.6% | 84.6% |
| Held, 95 | 0.234 | 0.234 | 100% | 100% |

Lambda error at the horizon end on tendon-moving roots falls from 4.32 to 3.66, each arm scored against its own later posterior.

**The gain is concentrated.** Roots 813, 975 and 819 supply most of it. Removing 813 and 975 from both arms gives 0.833 against 0.845 mm, so held is slightly worse elsewhere. Removing the winners is a one-sided view, but it shows the effect is a repair of one knee, on two visits, not a general improvement.

| Root | RMSE, baseline | RMSE, held | Lambda error, baseline | Lambda error, held |
|---:|---:|---:|---:|---:|
| 813 | 6.22 | 3.78 | -18.27 | -10.62 |
| 974 | 4.49 | 1.35 | -13.08 | -4.96 |
| 975 | 3.76 | 1.14 | -10.39 | -3.83 |
| 765 | 5.49 | 6.00 | -17.28 | -19.27 |
| 937 | 5.74 | 6.83 | 8.06 | 8.52 |
| 933 | 2.57 | 3.21 | 10.84 | 11.22 |
| 656 | 4.14 | 4.48 | -12.45 | -12.52 |

Root 974 improves only with the changed earlier history, so its earlier label as a local update effect is incomplete. Roots such as 656 worsen with lambda error nearly unchanged, so that cost is not through the lambda forecast; not diagnosed.

**Limits.** The hypothesis came from these same roots in this same session, so the gain at them is in-sample. No independent session has been tested. Evidence is in `gate3_realtime_filter_evidence_20261101/stationary_profile_hold`.

## Limits

One session, against a readout rather than truth. Mechanism labels at individual roots come from visual review and were not tested. Whether fixing any of these reduces error is untested.

Sessions cited in older audits may have been removed on 2026-11-01 at the user's direction. Results stay in the audit text; see `cleanup_documented_gate3_sessions_20261101.json` in the session root. Evidence for this note and the real-time specification is in `gate3_realtime_filter_evidence_20261101`.
