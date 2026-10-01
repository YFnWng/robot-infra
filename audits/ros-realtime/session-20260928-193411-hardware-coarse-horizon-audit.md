# Hardware coarse-horizon sparse-point audit: 20260928_193411_mppi_demo

## Scope and evidence

This is a passive post-run audit of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260928_193411_mppi_demo`

Evidence includes the rosbag topics for controller diagnostics, trajectory
action feedback, planned controls, MPPI response traces, targets, markers, and
device/manager state, plus the controller source and active no-rotation
hardware configuration. No hardware was commanded during the audit.

## Outcome

- The two axial gate targets and two-axis points 1, 2, and 4 reached the target
  tolerance.
- Two-axis point 3 timed out after 15.0 s.
- Point 3 was not simply unreachable: its observed error fell to 1.900 mm,
  only 0.100 mm outside the configured 1.8 mm tolerance, before increasing to
  3.084 mm at timeout.
- Rotation remained disabled throughout the goals.

## Timing gate

`observed`

The failed point had no planner deadline misses. Across the complete hardware
run, planner total time was:

| Statistic | Time |
| --- | ---: |
| P50 | 40.453 ms |
| P95 | 51.662 ms |
| P99 | 57.226 ms |
| Maximum | 64.888 ms |

There were two isolated plans above 60 ms, with a maximum consecutive miss
count of one; both occurred during the axial gates, not failed point 3. The
0.8 s, four-step rollout therefore ran adequately on hardware for this test.

Marker diagnostics were `TRACKING` for all 2600 recorded marker-status
messages. Timing and marker health do not explain point 3.

## Point-3 sequence

`observed`

The point-3 target was `[26.261, 22.833, 75.579]` mm. The controller initially
made substantial insertion/tendon motion and reduced the error from 5.86 mm to
1.90 mm. At approximately 6.26 s, the point-capture branch selected candidate
zero:

- measured error: 1.943 mm;
- predicted zero-command terminal error: 1.354 mm;
- selected candidate index: 0;
- selected total cost equaled zero-candidate total cost.

The zero branch was then retained or reselected for 88 status samples over
8.80 s. During this interval:

- measured error ranged from 1.943 to 3.290 mm;
- predicted zero-command terminal error remained between 1.354 and 2.483 mm;
- selected and zero costs were equal in all 88 samples;
- the final measured error was 3.100 mm while the zero rollout still predicted
  a 2.368 mm terminal error.

The one-step response traces also show that passive motion was much smaller
than predicted during the long hold:

| Zero-command response metric | P50 | P95 | Maximum |
| --- | ---: | ---: | ---: |
| Predicted displacement | 0.503 mm | 0.730 mm | 0.743 mm |
| Measured displacement | 0.133 mm | 0.322 mm | 2.417 mm |
| Predicted-vs-observed endpoint error | 0.486 mm | 0.814 mm | 2.061 mm |

The first zero response contains the large residual transient; afterward the
measured passive displacement is normally much smaller than the model's
prediction. Thus the controller waited for a modeled passive approach that did
not occur reliably.

## Root cause

### F1: capture hold can be renewed indefinitely from an inaccurate zero rollout

`inferred-high-confidence`

The configured hold duration is nominally 0.30 s, but it is not a total
lifetime. When that interval expires, a new solve can select the same hold
again whenever the model predicts the zero branch inside the 2.5 mm capture
radius and no nonzero branch improves terminal error by 0.25 mm. This condition
is implemented in `mppi.py` lines 1438-1470. While the timer is active, the ROS
wrapper sends zero without running another solve (`node.py` lines 1864-1875).

For point 3, the repeated model-only renewal converted a 0.30 s response pause
into an 8.80 s suppression of corrective control. The failed goal is therefore
primarily a capture-arbitration failure, not a horizon-length, compute-deadline,
marker-tracking, or take-up-engagement failure.

### F2: zero-input relaxation prediction is not trustworthy enough to gate execution

`inferred-high-confidence`

The learned dynamics may still predict passive distal relaxation as part of
the MPPI cost, but the evidence does not support using that forecast by itself
to repeatedly block all nonzero candidates. The observed error moved away from
the target while the zero rollout continued predicting a smaller terminal
error.

## Recommended correction gate

1. Make point capture response-verified and nonrenewable from model prediction
   alone. Record the observed error when the hold starts, permit one bounded
   0.30 s observation interval, and release the hold if the measured error has
   not improved by a small noise-aware amount or entered tolerance.
2. After a failed hold, inhibit another capture hold until either a nonzero
   command is executed, the target changes, or a clearly measured approach is
   observed. This prevents the same stale zero hypothesis from relatching.
3. Base the supervisory capture condition on measured error/tolerance. Retain
   the model-predicted zero rollout inside MPPI scoring, but do not allow it to
   be the sole reason all corrective candidates become ineligible.
4. Add a regression test in which the zero rollout predicts improvement but
   measured feedback is stationary or worsening; the planner must resume
   nonzero planning after one bounded hold.

Reducing `mppi_capture_radius_mm` from 2.5 mm to the 1.8 mm target tolerance
would likely have released this particular run after the first hold, but it is
only a parameter workaround. Response-verified, non-self-renewing capture is
the robust fix.

## Conclusion

The coarse 0.8 s horizon passed its first hardware compute and usefulness gate:
five of six total targets, including three of four two-axis targets, reached,
and the failed target approached to 1.90 mm. Point 3 failed after useful motion
because the anti-overshoot capture mechanism repeatedly forced zero based on
an optimistic passive-response prediction.
