# Circle-trajectory simulation audit — 20260912_203142_mppi_sim

## Scope and evidence

This is a passive, offline audit of:

`/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260912_203142_mppi_sim`

The session directory contains a 76.8 MB SQLite rosbag without `metadata.yaml`,
so evidence was read directly from its `topics` and `messages` tables using
the installed ROS message definitions. No live graph or actuator command was
used.

## Finding SIM-CIRCLE-01 — rotation-limit saturation caused the stop

**Confidence: observed. Severity: expected feasibility failure, not a ROS
scheduling failure.**

The circle generator issued all 36 targets. The controller was continuously
`ACTIVE` from bag-relative 9.720 s through 79.419 s, and then disarmed normally.
It did not fault or lose manager mode. However, the simulated catheter rotation
position approached its configured +270 degree hard limit:

| Evidence | Bag-relative time |
| --- | ---: |
| rotation reached +250.02 degrees | 51.769 s |
| rotation reached +260.07 degrees | 53.619 s |
| rotation reached +269.06 degrees | 54.049 s |
| final rotation | +269.50 degrees |

The `imricor_test` configuration limits catheter rotation to `[-270,+270]`
degrees. The circle began with rotation near zero and requested a continuous
full revolution. The MPPI/hardware projection correctly removed outward
rotation commands at the upper limit.

Motion degraded immediately afterward. Waypoint 24 began at 55.368 s and
produced only 0.293 mm net tip travel over its two-second budget. By waypoint
29, the realized command was identically zero for the entire waypoint. The
last nonzero samples were:

| Path | Last nonzero time |
| --- | ---: |
| MPPI planned command | 68.951 s |
| autonomy heartbeat | 69.009 s |
| manager output | 69.010 s |
| projected/realized plant command | 69.019 s |

This ordering demonstrates that commands stopped at the planner/projection
stage. There was no downstream manager, transport, or plant drop.

## Finding SIM-CIRCLE-02 — target advancement behaved as configured

**Confidence: observed.**

Targets continued to advance at approximately two-second intervals after the
rotation limit was reached because the YAML assigns a two-second independent
budget to each waypoint. The final target arrived at 77.414 s and the action
disarmed at 79.419 s, one final budget later. This explains the RViz behavior:
the target marker continued around the circle while the catheter remained at
its closest feasible limit-constrained configuration.

## Finding SIM-CIRCLE-03 — one planner deadline miss was transient

**Confidence: observed; not causal.**

At 22.620 s, controller status briefly reported
`planner_deadline_miss_zero`. It returned to `active` at 22.721 s and continued
moving for more than 46 seconds. This isolated miss did not cause the later
stop.

## Finding SIM-CIRCLE-04 — action outcome topics were not recorded

**Confidence: observed.**

The bag contains the 36 target messages but not the configured action feedback
or status topics. ROS action transport topics are hidden (`/_action/...`), and
the recorder command does not currently pass `--include-hidden-topics`.
Consequently, reached-versus-timed-out outcomes cannot be assigned directly to
individual waypoints from this bag. Ground-truth error and target timing still
show the limit saturation clearly.

## Conclusion and next experiment

The simulated stop is a valid joint-feasibility result, not a sim-to-real gap
or ROS real-time failure. A full 360-degree circle cannot be executed from a
zero-degree rotation start while monotonically rotating in either direction
under a +/-270-degree limit. Test one of the following before hardware:

1. an arc whose angular sweep stays comfortably inside the available rotation
   margin;
2. a pre-positioned simulated rotation configuration that leaves at least 360
   degrees of travel in the selected direction; or
3. a deliberately limit-aware trajectory split into feasible arcs, with a
   reviewed unwind/reversal segment that does not claim to be a continuous
   one-revolution path.

Do not relax the configured rotation limit to make the simulation pass; it is
part of the hardware contract.
