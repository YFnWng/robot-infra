# Axis-0 provisional-engagement hardware fault and remediation

## Scope

This audit uses the recorded hardware session:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260916_163300_mppi_demo`

No hardware was commanded or reconfigured during analysis or verification.

## Observed fault mechanism

The 1 mm/s path reached 2.278 mm progress before the controller faulted on
`backlash_takeup_unconfirmed:axis_0`. Axis 0 had nevertheless already produced
multiple strongly attributed, correctly directed responses. Representative
inferred increments were -0.106, -0.134, -0.511, and -0.170 rad with response
evidence approximately 1.0 and marker-fit RMS approximately 0.32--0.36 mm.

After entering `PROVISIONAL`, two later observations were non-confirming but
not contradictory: one inferred -0.074 rad in the commanded direction, below
the 0.10-rad confirmation floor, and another inferred zero. Source treated
both as rejection votes, returned the shaft to `TAKEUP`, and immediately
applied the retained accumulated-travel counter. Travel was about 11.404 rad,
above the configured negative-axis-0 fail bound of 1.5 x 6.856 = 10.284 rad.

This is an observed response-classification defect. The fault does not show
that axis 0 failed to engage.

## Remediation

The backlash observer now classifies each eligible response as `CONFIRMED`,
`INCONCLUSIVE`, or `CONTRADICTORY`.

- Missing evidence, accepted-noop corrections, and same-direction response
  below the inference floor preserve `PROVISIONAL` and its confirmation count.
- Only a resolved response of sufficient magnitude in the opposite modeled
  direction contributes a contradiction vote.
- Repeated contradiction fails closed; it does not restart full-rate take-up.
- The maximum-width travel bound remains active before the first credible
  response, but it is not retroactively applied to a provisional engagement.
- Status diagnostics expose the response classification and contradiction
  count. A contradiction fault is reported separately as
  `backlash_response_contradicted:axis_N`.

## Verification

- Focused backlash tests: 39 passed.
- Full `catheter_control` source suite: 232 passed.
- The recorded hardware stream was replayed through the corrected observer:
  axis 0 did not fail and remained safely `PROVISIONAL` through the formerly
  faulting interval.
- `catheter_control` rebuilt successfully with `--symlink-install`.

The replay verifies this gate correction only. The same session opened eight
take-up transactions in about seven seconds and spent 152/218 path feedback
samples in `TRANSMISSION_HOLD`; repeated reversal selection remains a separate
tracking-performance concern rather than the cause of this fault.
