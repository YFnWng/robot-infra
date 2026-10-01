# Session 20260915_201546 provisional axis-2 fault audit

## Scope

Passive analysis of the hardware-equivalent CUDA/asymmetric-backlash sparse
simulation:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
20260915_201546_mppi_sim/20260915_201546_mppi_sim_0.db3
```

The analysis used a read-only copy while the source bag was still open. No ROS
process, simulator state, controller parameter, or hardware state was changed.

## Result

This run is not ready for hardware. Point 1 reached 1.320 mm final error. Point
2 faulted after 7.46 s with:

```text
backlash_takeup_unconfirmed:axis_2
```

Feedback, estimation, CUDA planning, and model validity were healthy at the
fault. Active plans were 34.45/47.08/48.28/48.91 ms at
P50/P95/P99/maximum, below the 60 ms deadline, with no consecutive deadline
misses. The estimator remained `TRACKING` and the manager remained ready.

## Fault sequence

Transaction generation 9 requested negative physical rotation and negative
physical bend. Axis 2 consumed its nominal remaining gap and then produced:

```text
phase axis 2: TAKEUP -> PROVISIONAL
confirmation_count axis 2: 0 -> 1
response_evidence axis 2: 1.0
inferred transmitted increment axis 2: -0.555 rad
```

The arbiter correctly removed axis 2 from the pending mask immediately and
held it at zero while rotation remained pending. At the next accepted visual
update, axis 2 had no new confirmed contribution. The estimator incremented
its provisional rejection count, but before the configured two-observation
fallback could occur, the generic maximum-width check changed axis 2 directly
to `FAILED`. The node then faulted on that failed flag.

This contradicts the configured and documented provisional semantics: one
credible response should stop full-rate take-up, and two later inconsistent
moving observations should be required before returning to `TAKEUP`.

## Source mechanism

In `backlash.py::observe_response`, the unconfirmed-response branch first
increments `provisional_rejection_count`, but then unconditionally applies the
maximum calibrated-width failure test to both `TAKEUP` and still-valid
`PROVISIONAL` states. Because shaft 2 had already accumulated more than the
maximum-width threshold by its first credible response, the very next miss
faulted it even though the rejection count was only one of two.

The maximum-width fail-closed guard should continue to bound unconfirmed
`TAKEUP`, but it must not bypass the configured provisional rejection policy.
Apply it after `PROVISIONAL` has actually fallen back to `TAKEUP`, or otherwise
explicitly exempt a provisional state whose rejection count remains below the
configured limit.

## Additional observation

The simulator's transmitted axis-2 encoder stayed fixed during generation 9,
even though the controller inferred a first axis-2 response. Thus the first
provisional indication was not ground-truth shaft-2 transmission. It was
likely attribution of latent/estimator motion to the bend Jacobian column.
That does not invalidate provisional handoff: provisional exists precisely to
stop the aggressive macro on first evidence while requiring repeated evidence
for persistent engagement. It does make the two-observation fallback
essential, rather than optional.

## Required regression

Add a test in which a shaft exceeds its calibrated travel, receives one
credible response, then one inconsistent moving observation. It must remain
`PROVISIONAL`, not `FAILED`. A second inconsistent observation may return it to
`TAKEUP`, where the existing maximum-width guard can fail closed. Repeat this
same sparse simulation after the correction; do not proceed to hardware until
all eight points are processed without a controller fault.

## Remediation status

Implemented on 2026-09-15. The maximum-width guard now runs only when the
post-rejection phase is `TAKEUP`. The added regression verifies that the first
miss beyond the bound remains `PROVISIONAL`, while the configured second miss
falls back and fails closed on the same observation. The backlash suite passes
34/34, the integration-oriented controller subset passes 42/42, and
`catheter_control` builds cleanly. The required eight-point simulation rerun
remains pending.
