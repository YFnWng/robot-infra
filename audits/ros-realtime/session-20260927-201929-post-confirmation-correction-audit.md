# Session 20260927_201929 MPPI hardware audit

## Scope and evidence

- Session: `/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260927_201929_mppi_demo`
- Bag: `20260927_201929_mppi_demo_0.db3` (read-only analysis)
- Experiment: four model-generated, rotation-disabled, two-axis sparse targets; guarded encoder-position home before every target.
- Recorded controller identity: `command_output_enabled=true`, engaged-gain belief and scenarios enabled, grouped sampling enabled, transmission-aware rollout disabled, reversal-backlash rollout width zero, take-up confirmation timeout 1.0 s, take-up velocities `[1.0, 2.5, 0.5]`, rotation velocity limit zero.
- The working tree was dirty during the audit. This report identifies the bag and recorded runtime parameters rather than claiming a clean source revision.

## Outcome

| Point | Initial error | Minimum error | Final error | Result |
|---|---:|---:|---:|---|
| 1 | 4.003 mm | 0.288 mm | 0.715 mm | reached |
| 2 | 5.887 mm | 1.291 mm | 1.291 mm | reached |
| 3 | 5.847 mm | 1.906 mm | 3.800 mm | timed out |
| 4 | 7.982 mm | 0.888 mm | 3.957 mm | timed out |

The experiment completed without a controller fault: 2/4 points reached and 2/4 timed out. Rotation remained exactly zero in the recorded plans and physical position feedback.

## Take-up correction gate

The take-up confirmation correction passed its intended gate:

- estimator health was `TRACKING` for every armed diagnostic sample;
- no `backlash_takeup_unconfirmed` or confirmation-timeout fault occurred;
- all confirmation holds completed in 0.10--0.30 s, below the 1.0 s bound;
- point 3's initial coupled take-up encountered axis-0 saturation, emitted `replanning_after_takeup_saturation`, and did not continue leaking the coupled transaction;
- each subsequent pending axis reached `ENGAGED` with confirmation count 3.

Therefore the previous indefinite confirmation/stationary failure is repaired. The remaining static behavior is not an engagement-confirmation wait.

## Remaining planner failure

After useful initial motion, points 3 and 4 stopped receiving nonzero MPPI plans:

- point 3: 189 plans, 46 nonzero and 143 zero; last nonzero plan at 6.69 s;
- point 4: 173 plans, 38 nonzero and 135 zero; last nonzero plan at 6.17 s.

For point 3, the saturation event persisted `planner_blocked_motor_direction=[1,0,-1]` for 146/148 armed samples. Once the remaining useful candidates required that combined insertion/tendon direction, MPPI repeatedly selected its explicit zero candidate. This block outlived the transaction whose saturation created it.

For point 4 no blocked direction persisted, but the planner still made the zero candidate equal to the selected minimum-cost candidate for most of the latter run. The controller had already reached 0.888 mm, then moved away and settled near 3.96 mm. This is mode/cost selection after overshoot, not take-up arbitration.

The response evidence also shows why replanning was difficult: median causal forecast endpoint error was only 0.41--0.44 mm on points 3--4, but p95 rose to 3.08 mm and 2.15 mm, and direction cosine reached approximately -0.99. The model was occasionally directionally wrong even though the estimator remained healthy.

## Timing and freshness

- Device feedback interval: median 2.13 ms, p95 11.28 ms, max 24.67 ms.
- Marker interval: median 33.32 ms, p95 34.61 ms, one max gap 126.69 ms.
- Controller status interval: median 100.04 ms, p95 101.20 ms, max 200.44 ms.
- Armed MPPI plan time across targets: median 42.6--44.5 ms, p95 48.1--53.5 ms, max 67.2 ms.
- Two diagnostic deadline-miss samples occurred on each of points 3 and 4; no repeated-deadline fault or stale-feedback fault occurred.

These misses are secondary to the long stationary intervals: the controller continued planning and publishing status while choosing zero.

## Overshoot investigation

### O-1: action-bounded reachability

Confidence: observed.

The shape plot was corrected to stop each closest-approach search at the final action feedback, excluding the following encoder-home transaction. Point 3 did not enter the 1.8 mm tolerance during tracking: its minimum was 1.906 mm at 6.218 s. Point 4 entered tolerance for only 0.151 s, from approximately 5.651 to 5.802 s, shorter than the required 0.25 s settle interval. Its minimum was 0.888 mm at 5.730 s.

### O-2: tendon engagement was declared before the high-gain response region

Confidence: observed.

For point 3, axis 2 became `ENGAGED` at 3.958 s with motor angle about -2.44 rad. The motor then advanced to about -30.49 rad by 5.5 s while the tip response remained modest; the large Cartesian response arrived between roughly 5.5 and 6.3 s. For point 4, axis 2 became `ENGAGED` at 0.924 s near -0.53 rad, but the high-gain response again appeared only after the motor reached approximately -37 rad near 5.5 s. Confirmation was based on three small/noisy distal-response events. This proves that response onset was detected, but not that the subsequent engaged incremental gain was stationary.

### O-3: MPPI kept loading the delayed response and braked one observation too late

Confidence: observed.

At point 4, the controller still selected `[4.84,0,4.5]` at 5.558 s while measured error was 4.95 mm. At 5.658 s, with measured error already 1.77 mm, it selected `[3.00,0,3.00]`: its predicted terminal error was 0.859 mm versus 0.909 mm for zero, only a 0.050 mm predicted advantage. Zero was selected at 5.757 s, after the observed tip had crossed the target.

The causal response trace shows the plant response exceeded the forecast during this capture:

- at 5.723 s, predicted displacement norm was about 2.71 mm while measured displacement was about 4.61 mm (1.70x);
- after zero was selected, the 5.796 s forecast predicted about 0.90 mm residual motion while 3.16 mm was measured (3.53x);
- point 3 showed the same effect after zero: at 6.304 s, 1.67 mm residual motion was predicted and 3.58 mm was measured (2.14x).

Thus the overshoot was not caused by the take-up compensator issuing a simultaneous command at target capture. Both axes were recorded `ENGAGED`, take-up risk was zero, and the commands were ordinary MPPI plans. The main cause was delayed stored response that the 160 ms rollout and engaged-gain scenarios underpredicted.

### O-4: the gain belief could not represent the response surge

Confidence: observed for the posterior; inferred-high-confidence for the modeling consequence.

At point 4 capture, the learned tendon gain mean was about 1.00 with a 90 percent credible interval approximately `[0.96,1.04]`; the selected rollout scenarios were similarly narrow. The observed catch-up response was much larger. The belief adapts a multiplicative engaged gain, but it cannot represent a variable delay, stored untransmitted travel, or a regime change from low to high incremental gain. Axis-0 interface response was not covered by this distal-gain posterior.

### O-5: discrete minimum speed amplified the final correction

Confidence: observed.

The deployed minimum executable speeds were `[2,0,2]`, the planner used an exact scored candidate rather than a weighted small command, and the horizon was four 40 ms steps. Near point 4 the choice was therefore another finite motion or zero; there was no arbitrarily small braking command. Because the model gave the finite plan a marginal 0.050 mm terminal-error advantage, it selected motion.

### Recovery is a separate failure

After overshoot, point 3 retained the saturation-derived combined direction block `[1,0,-1]`, preventing or discouraging the corrective reversal. Point 4 had no such block, but the unrestricted optimizer still preferred same-direction insertion or zero because its local forecast was directionally wrong after the response surge. These explain failure to recover, not the initial overshoot itself.

## Verdict update

The primary overshoot mechanism is premature confidence in stationary post-engagement response: the take-up gate observed enough response to release MPPI, MPPI accumulated fast tendon motion through a low-response region, and stored response arrived faster and longer than its short-horizon model predicted. The highest-value correction is a response-aware capture policy: widen/reinitialize gain uncertainty after a newly confirmed engagement or abrupt gain innovation, impose a meaningful-improvement threshold before any nonzero plan near tolerance, and enter a zero-command observation hold long enough to measure residual motion before permitting further same-direction loading. This can be done without changing the slow take-up transaction itself.

## Verdict

The confirmation/saturation correction is qualified, but the complete controller is not yet qualified for farther two-axis tracking. The next controller change should:

1. scope a saturation-derived combined-direction block to the invalid root state/transaction and clear or revalidate it after confirmed engagement, meaningful state motion, or a fresh target-relative plan;
2. instrument and gate the explicit-zero branch with terminal-error improvement so zero cannot remain optimal after the tip has left tolerance unless all nonzero candidates are physically infeasible;
3. retain the bounded confirmation hold and fail-closed timeout unchanged.



## Implemented remediation (2026-09-27)

The overshoot findings above are now represented by four bounded controller changes:

1. **Scored point-capture branch.** For point targets only, deterministic zero is forced to be the eligible MPPI branch when its predicted terminal error is within `mppi_capture_radius_mm` and the best nonzero candidate improves terminal error by less than `mppi_capture_minimum_terminal_improvement_mm`. This decision occurs before MPPI weighting and execution; no selected plan is modified afterward. The hardware profiles use 2.5 mm and 0.25 mm respectively.
2. **Residual-response observation hold.** Once the capture branch wins, it remains selected for `mppi_capture_hold_s=0.30` s, longer than the sparse-target 0.25 s settle interval. Rollouts and diagnostics continue during the hold, but no additional load is issued while delayed response is observed. The hold is disabled by default and does not apply to continuous-path corridor targets.
3. **Gain-belief regime-change recovery.** A contradictory response or excessive normalized innovation now widens the directional log-gain posterior to at least `engaged_gain_contradiction_log_std=0.70`, resets its confidence count, and re-anchors the next local increment at the contradictory observation. This prevents repeated comparisons against one stale pre-surge anchor. Until the relevant directional posterior is `CONFIDENT`, tendon candidates are scaled before rollout and scoring by `mppi_engaged_gain_learning_velocity_scale=0.50`; deterministic zero remains unchanged.
4. **Saturation block lifecycle.** A saturation-derived coupled direction block is cleared when the complete direction becomes feasible again or when every involved shaft is freshly response-confirmed `ENGAGED` with zero remaining take-up. Hard joint-limit projection remains active after the block is released. The release reason is published diagnostically.

The two grouped hardware profiles enable these settings. Generic planner defaults keep capture disabled and learning-speed scale at 1.0, preserving other profiles until explicitly qualified. New diagnostics expose all configuration values, whether capture was applied, its predicted zero terminal error, the selected gain-learning scale, and the saturation-block release reason.

Verification:

- `catheter_control` rebuilt successfully with `colcon build --packages-select catheter_control --symlink-install`;
- 285 direct package tests passed with the ROS and workspace overlays sourced;
- focused planner, belief, and node regressions cover capture-hold persistence and release, pre-rollout gain scaling, innovation widening and re-anchoring, and saturation-block expiry;
- ROS flake8 reported no errors in the six changed Python source and test files.
