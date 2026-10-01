# Hardware capture-release follow-up audit: 20260928_200735_mppi_demo

## Scope and evidence

This is a passive post-run audit of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260928_200735_mppi_demo`

The audit uses trajectory-action feedback, controller diagnostics, causal MPPI
response traces, planned controls, marker diagnostics, and the active sparse
point/controller configuration. No hardware was commanded.

## Outcome

- Points 1, 2, and 4 reached the configured target gate.
- Point 3 timed out at 15.027 s with a final error of 1.466 mm.
- This is below the configured 1.8 mm spatial tolerance, but the goal also
  requires 0.25 s continuously inside tolerance. Point 3 entered the band at
  14.977 s and had only 0.050 s of continuous in-band evidence before timeout.
- There was one earlier, isolated in-band sample at 6.060 s (1.426 mm), but
  the next sample was already outside the band at 1.992 mm.

The timeout is therefore a settle-time failure, not a contradiction between
the reported final error and the action result.

## Capture-response correction

`observed`

The response-verified capture correction operated as intended:

- point 3 selected `capture_hold_zero` only from about 6.2 to 6.4 s;
- the measured response contradicted the predicted passive approach;
- at about 6.5 s the controller reported
  `capture_response_shortfall_replan`;
- the cumulative capture-release count increased from 1 to 2;
- the passive-response scale fell from 0.470876 to its conservative floor of
  0.1.

Unlike the preceding `20260928_193411_mppi_demo` run, capture did not renew
for several seconds. The former indefinite zero-hold failure is fixed for this
trial.

## Point-3 control sequence

`observed`

The important controller-state intervals were:

| Relative time | State | Approximate duration |
| --- | --- | ---: |
| 0.2--0.7 s | initial take-up and replan | 0.6 s |
| 0.8--3.7 s | post-take-up MPPI control | 3.0 s |
| 3.8--5.5 s | second take-up transaction and replan | 1.8 s |
| 5.6--6.1 s | MPPI control; rapid approach/overshoot | 0.6 s |
| 6.2--6.4 s | bounded capture hold | 0.3 s |
| 6.5 s | response-shortfall release | one diagnostic cycle |
| 6.6--8.8 s | reversal take-up and response replan | 2.3 s |
| 8.9--15.0 s | post-take-up corrective MPPI control | 6.2 s |

During the first close approach, error changed from 7.732 mm at 5.71 s to
1.426 mm at 6.06 s, then rose to 3.141 mm by 6.21 s and 4.438 mm by 6.46 s.
The rapid residual response carried the tip through the target band before a
reverse correction could become effective. Clearing the reverse take-up then
consumed about 2.3 s. The final correction was converging and entered tolerance
again only 0.05 s before timeout.

Point 3 had no planner-deadline-miss diagnostic cycles. Its status counts were
40 `takeup_active`, 4 `takeup_confirmation_hold`, 2
`takeup_complete_replan`, 3 `capture_hold_zero`, and 1
`capture_response_shortfall_replan` cycle. Thus computation deadlines and the
new capture branch were not the limiting gates.

## Marker health

`observed`

During point 3, marker diagnostics contained 609 `TRACKING` and 18
`PARTIAL_MULTI_RIG_TRACKING` messages, with no `INSUFFICIENT_VALID_RIGS`
message. The partial multi-rig fallback continued providing accepted feedback;
marker loss did not cause this timeout.

## Finding

### F1: The point is now a near-success limited by late settling after physical overshoot

Severity: low for this isolation test; medium for efficient point control
Confidence: observed / inferred-high-confidence

The controller ultimately produced the correct corrective direction and put
the tip inside tolerance. It missed the pass criterion because the combination
of a large residual response, reverse take-up, and the 15 s budget left less
than the required 0.25 s settling interval. Extending the same goal by roughly
0.20 s would have been sufficient only if the final trend remained in band;
using an 18--20 s isolation-test timeout provides a more defensible margin.

This result does not justify weakening the 0.25 s settle gate. The gate is what
distinguishes a transient crossing, such as the 6.06 s event, from a stable
reach. For the next diagnostic run, increasing the point timeout is the least
invasive change. The remaining controller improvement is to reduce the first
post-engagement overshoot so that reversal take-up is not required, rather than
accepting a single in-band sample as success.

## Conclusion

The new capture-response correction passed its hardware gate: an inaccurate
passive forecast was released after one bounded hold and did not suppress
control indefinitely. Point 3's recorded 1.466 mm final error is a genuine
near-success, but it arrived too late to satisfy the 0.25 s dwell requirement.
The next run should preserve the settle gate and use a longer isolation-test
timeout while the residual-response overshoot is investigated separately.
