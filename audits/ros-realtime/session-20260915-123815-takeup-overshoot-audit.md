# Session 20260915_123815 take-up overshoot audit

Status: observed failure; source remediation implemented, rerun pending.

## Failure timeline

- The first recorded path reference was 0.16 mm from the tip and advanced in
  approximately the `(+x,-y,0)` direction.
- MPPI initially requested negative rotation (`-7.08 deg/s`), which the
  bounded take-up feedforward promoted to `-40 deg/s`.
- At 0.385 s, rotation was in `TAKEUP` with 6.34 rad remaining. At 0.785 s it
  was marked `ENGAGED`, despite the preceding status sample still reporting
  4.67 rad remaining and simulated transmitted rotation still being zero.
- Small bend commands then started shaft-0/shaft-2 take-up. Their independently
  exhausted gaps and prolonged maximum take-up commands produced a large
  coupled tip excursion. Closest-path error reached 15.05 mm at 2.640 s and
  the path action stopped with `hard path error`.

## Root cause

The engagement predicate combined a marker-derived response with an estimator
posterior that had already been propagated by upstream encoder motion. It did
not also require the separately tracked geometric gap to be exhausted. The
feedforward magnitude likewise treated the epistemic `TAKEUP` phase as proof
that maximum-speed geometric take-up was still required.

## Correction

The two meanings are now separated:

1. Positive `remaining_rad` controls the maximum-speed take-up magnitude.
2. `TAKEUP` controls the direction latch and persists until measured response
   confirms engagement.
3. Confirmation cannot transition to `ENGAGED` until `remaining_rad` is at or
   below the motor-increment tolerance.

This preserves fail-closed bounded travel and does not weaken any manager,
limit, freshness, watchdog, or hard-path-error gate.
