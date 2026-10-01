# Session 20260915_200759 sparse-target envelope audit

## Scope

Passive analysis of:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260915_200759_mppi_sim/20260915_200759_mppi_sim_0.db3
```

No ROS process, controller parameter, simulator state, or hardware state was
changed.

## Result

The coupled model-encoder envelope correction passed its regression, but this
session is **not a hardware-equivalent controller qualification**.

Seven of eight independent sparse targets reached the 1.8 mm tolerance. Target
3 consumed its full 10.019 s budget and stopped at 2.267 mm error. It did not
fault, feedback continued, the experiment homed successfully, and targets 4--8
then completed. The prior failure mode is therefore corrected: an infeasible
point now times out without invalidating the feedback chain.

The highest observed first raw encoder channel was 109,994 counts against the
configured 110,000-count model-valid ceiling. The controller and manager
reported no runtime fault. The only manager inhibition samples occurred during
initial simulated driver qualification, before the experiment.

## Timing and health

During 271 `ACTIVE` status samples:

- estimator health was always `TRACKING`;
- position/encoder/marker age p95 was 3.04/22.41/30.72 ms;
- feedback-pair skew p95 was 0 ms and maximum was 7.31 ms;
- planning elapsed time was 20.31/27.63/30.28/32.27 ms at
  P50/P95/P99/maximum, with zero consecutive deadline misses;
- all commands remained model-valid;
- the response forecast endpoint error was 0.201 mm P50 and 0.540 mm P95;
- response direction cosine was 0.998 P50.

## Qualification mismatch

Controller diagnostics prove that this run used:

```text
compute_device=cpu
backlash_compensation_enabled=False
takeup_transaction_enabled=False
model_adaptation_enabled=False
```

It therefore exercised the default 32-sample CPU point controller and the new
coupled encoder-envelope projection, but did not exercise the causal-v2
asymmetric backlash observer/compensator, response-terminated take-up
transaction, or the 1,024-sample CUDA route intended for the next hardware
trial.

## Hardware gate

Do not treat this bag alone as authorization for the full sparse hardware run.
First repeat the same eight-target test in simulation with the exact intended
hardware controller settings: CUDA/1,024 samples, causal-v2 asymmetric
controller widths, matched asymmetric truth widths, response-terminated
take-up, and online Jacobian adaptation in the same state intended for the
hardware trial. Require:

1. no controller or manager fault;
2. target 3 timing out cleanly if it remains infeasible;
3. successful continuation through targets 4--8;
4. no repeated unconfirmed take-up transaction;
5. bounded planning below the 60 ms deadline; and
6. encoder feedback remaining inside the learned-model envelope.

After that gate, start hardware with a single known-reachable sparse target,
not the complete eight-point sequence. The real plant adds stick-slip,
feedback delay, model mismatch, and uncertain take-up that this default-mode
session did not exercise.
