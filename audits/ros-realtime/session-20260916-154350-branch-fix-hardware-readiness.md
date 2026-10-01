# Corrected grouped-MPPI hardware-readiness audit

## Scope

This is a passive audit of the corrected simulation session:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260916_154350_mppi_sim`

It checks whether the branch-selection correction and current reviewed
controller profile are suitable for a staged real-hardware trial. No hardware
was armed, qualified, commanded, or reconfigured.

## Runtime result

The branch correction resolved the closed-circle endpoint deadlock.

- The path completed its full 80.811 mm progress and disarmed normally.
- Final geometric/reference error was 0.338 mm.
- Maximum geometric path error was 2.855 mm and maximum active-reference
  error was 2.865 mm.
- Governor samples were 568 `RUNNING`, 152 `SLOWED`, 144
  `TRANSMISSION_HOLD`, and 10 `FINAL_HOLD`; there was no `PAUSED` state.
- Six take-up transactions consumed 4.613 s total, with a maximum individual
  hold of 0.869 s.
- Rotation had three nonzero sign changes. The corrected endpoint logic did
  not reintroduce the earlier fine-tuning chatter.
- Response traces had 0.108 mm median and 0.240 mm p95 endpoint error; median
  predicted/measured direction cosine was 0.951.

MPPI used the reviewed grouped configuration: 1,024 CUDA samples, four rollout
steps, switch-cost weight 1, and take-up-delay weight 4.

## Timing

Across 290 completed plans, plan latency was 42.674 ms median, 50.605 ms p95,
53.863 ms p99, and 61.791 ms maximum against the 60 ms hardware deadline.
There was one isolated deadline miss and no repeated-deadline fault. This is
compatible with the existing fail-closed three-consecutive-miss hardware
policy, but it does not replace a full-camera UKF shadow measurement because
the hardware capture/marker workload is not identical to the simulated truth
path.

## Static hardware-path compatibility

The real and simulated controller use the same MPPI, backlash observer,
take-up transaction, grouped mode selection, UKF-history reconciliation,
hardware projection, and limit-reserve code. The reviewed hardware profile
selects the correct v171 distal checkpoint boundary, causal-v2 fixed Jacobian,
UKF, CUDA/1,024 samples, grouped sampling, directional take-up widths, and
three-miss hardware deadline threshold. The focused controller regressions
pass 90/90, and the installed editable build contains the corrected
`path_tracking.py` source hash.

Two deployment differences remain:

1. `continuous_circle_sim.yaml` uses a circle plane at `x0 + 15 mm`, whereas
   `continuous_circle_hardware.yaml` uses `x0 + 10 mm`. Therefore this run
   validates the controller and scheduler, not the exact proposed hardware
   geometry.
2. `causal_v2_fixed_hardware.yaml` deliberately retains
   no `command_output_enabled` entry. The launch rejects any controller or
   performance profile that declares the interlock and reapplies the explicit
   launch argument as a direct ROS `-p` rule. Thus an exact-node profile can
   neither enable output nor mask an explicit request. A performance/shadow
   overlay combined with enabled output is also rejected during launch setup.

## Readiness decision

**Conditional go for a staged hardware test; the output-interlock deployment
blocker is corrected, but a post-change camera/UKF shadow check remains the
next gate before motion.**

Before actuation:

1. run the reviewed controller profile in full-camera/UKF shadow mode and
   verify no repeated deadline misses, accepted marker updates, fresh POS/ENC,
   and `MANAGER_READY`;
2. restart the rebuilt controller with the explicit output interlock and
   verify the resulting `command_output_enabled` diagnostic before arming;
3. begin with an interior, low-speed target/path segment and preserved hard
   joint margin rather than assuming that the tested `x0 + 15 mm` simulated
   circle proves the `x0 + 10 mm` real trajectory reachable.

Manager inhibition, current-position projection, freshness gates, watchdogs,
fault latching, and the prohibition on encoder-zero mutation remain unchanged.
