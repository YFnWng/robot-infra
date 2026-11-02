# Driver-native live A-update probe

Revised 2026-10-14 after the first hardware run was inconclusive. Normal MPPI
stop-confirm-replace is unchanged. This extension deliberately tests only one
positive, same-direction angle update; reversal and continuous live retargeting
are not qualified by it.

## Bounded scenario

The existing `position_stream_probe` adds `--mode live-update` with its
`encoder_stream` backend. It opens the existing manager-authorized session,
starts a +400-count native axis-0 endpoint at 36 RPM with a fixed 250 ms lease,
and waits for measured movement and ACK-qualified NATIVE_STARTED evidence.
It then sends exactly one `$A4.5` command (100 counts) while enabled, with no
O0, O1, direction, speed or mode command around that update. It never retries A
and never renews the original deadline.

Only this diagnostic suppresses the original endpoint-completion check, because
otherwise the supervisor would truncate a queued/replaced trajectory at the old
endpoint and invalidate the experiment. Hard bounds, freshness, no-progress,
fault inhibition and fixed lease stopping remain. Firmware independently limits
axis-0 travel to [-2,+667] counts from the initial anchor, other axes to 2 counts,
and requires only axis 0 enabled at 36 RPM. The manager checks the entire travel
envelope against logical hard limits and authorizes a single dispatch per ID.

Execution requires both `native_live_probe_enabled=true` in the manager and a
firmware image compiled with `TKCTL_NATIVE_LIVE_PROBE_ENABLED=1`, advertising
`native_live_update_bracket_probe_v1`. This capability requires a new reflash;
the older image is rejected before motion. Both gates are intended off outside
qualification; the user's explicitly enabled compile flags are preserved. The probe's special session
request fails before motion if either is unavailable. Raw device-service X is
unconditionally rejected: this is not a UART or inhibited-manager bypass.

The report records fresh hardware encoder samples immediately before the A UART
write and after its ACK, with device timestamps and converted shaft angles.
Driver receipt/reference position is treated as an interval between these
samples, not as the cached position in the old ACK event. A gets exactly one
transmission attempt even on ACK failure. Additional driver DA/DS queries are
not inserted into the moving transaction, to avoid perturbing it.

`--mode live-control` enters identical diagnostic lease supervision but sends
no A update. Home to the same six-axis encoder anchor before the live-update run
and pass that successful control report with `--compare-control`. This measures
the no-update endpoint including native overshoot instead of assuming 400.
The anchor is checked before motion. Both modes still record original lease
expiry and stationary post-deadline samples. Firmware rejects the scenario if
its fresh pre-write/control travel is already 300 counts or more.

Candidate endpoints relative to the original anchor are:

| Behavior | Expected endpoint |
| --- | --- |
| Replacement from original reference | +100 counts |
| Replacement from current reference | fresh write/ACK encoder bracket +100 counts |
| Queued increment after first move | +500 counts |
| Ignored update | measured matched-control endpoint |

Classification requires a matched control and a unique endpoint/interval match
within 15 counts; ambiguous or other behavior fails rather than guessing. This
tolerance is unchanged. ACK delay and native endpoint variability can still
make the result inconclusive. An ignored or queued update can
pass the bounded-observation check, but does **not** qualify live replacement.
One pass never establishes repeatability or safety for reversal/continuous MPPI.

## Host ordering fix

Finding LP-ORDER-1: `control_retry.json` in
`native_live_bracket_20261014T202138251208463Z` recorded no X transmission, while
the next Y was stamped only 33 microseconds after the X request (observed).
Both relay stages enforce shared monotonic source timestamps; cross-topic Y
overtaking X is a high-confidence explanation, not a directly recorded callback
ordering trace. The original move reached 400 and completed normally.

The probe now pauses Y publication after its one-shot X until matching serial TX
and LIVE_UPDATED evidence are observed. It continues spinning callbacks and
checking safety/freshness/travel. Waiting is bounded by 75 ms monotonic time and
the original absolute lease, whichever ends first. Missing evidence aborts to
the existing cleanup stop; X is never retried, no deadline is renewed, and no
manager/bridge freshness rule is weakened. Previously sent Y stamps are older
than X; subsequent Y stamps are generated only after this handshake.

Only the Python probe changed for this ordering repair; no additional Teensy
reflash or manager/bridge restart is required. Offline delayed-delivery tests
assert that no Y is published while X is pending, that Y resumes after completion,
and that missing completion fails within the timeout without retry/lease renewal.
The 161-test suite passes. Live full-stack ordering/wait distributions remain
unmeasured; the report now records `live_update_ordering_wait_ms` for the next run.
This scoped scheduling review follows the ROS real-time audit workflow.

## Build and explicit execution

Compile/reflash the updated Teensy sketch with both streamed position and the
diagnostic flag enabled. The agent has not flashed or executed hardware.
Rebuild/source `control_interface` and restart the interface with the diagnostic
opt-in in addition to the existing settings:

```bash
ros2 launch control_interface launch.py \
  position_stream_enabled:=true native_live_probe_enabled:=true
```

Use the established connection/power-qualification workflow first, with no other
controller or command publisher. Choose a new report directory under the external
session root. Preview does not open a session or send motion; execute independently
checks the diagnostic capability during its session handshake.

```bash
/home/chen-lab/Yifan/cr-venv/bin/python \
  -m control_interface_py.position_stream_probe \
  --mode live-control --output "$OUT/control_preview.json"

/home/chen-lab/Yifan/cr-venv/bin/python \
  -m control_interface_py.position_stream_probe \
  --execute --mode live-control --output "$OUT/control.json"

# Home through the existing guarded manager position path, then:
/home/chen-lab/Yifan/cr-venv/bin/python \
  -m control_interface_py.position_stream_probe \
  --mode live-update --compare-control "$OUT/control.json" \
  --output "$OUT/live_preview.json"

/home/chen-lab/Yifan/cr-venv/bin/python \
  -m control_interface_py.position_stream_probe \
  --execute --mode live-update --compare-control "$OUT/control.json" \
  --output "$OUT/live_update.json"
```

Restore the diagnostic flag/manager opt-in to off after qualification. Do not
enable live replacement in MPPI based solely on this report. Re-review observed
semantics and stopping behavior before any further live-nearer/farther/reversal
scenario.

## Paired hardware result: live replacement not qualified

Observed 2026-10-14 under external session
`native_live_bracket_20261014T202138251208463Z`:

- `control_ordered.json`: passed, initial raw counts all zero, target +400 at
  36 RPM, no A update, final axis-0 count 401 and others zero. Diagnostic control
  ordering wait was 5.22 ms; original lease expired after the motor had settled.
- `update_ordered.json`: one X transmitted and one A4.5 ACK-qualified completion.
  Fresh before-write count 235 at device time 783081291 us; after-ACK count 350
  at 783092572 us. Ordering wait was 15.45 ms. Motion continued through
  442, 515, 577, 635 and 691; final recorded count 694, stationary in the final
  samples, others zero. The 667-count guard stopped the trial before its lease.
- Candidate endpoints were 100 (original reference), 335..450 (current reference
  bracket), 500 (simple queued increment), and matched control 401 (ignored).
  The trajectory crossed all candidates. Since safety truncated it, its natural
  endpoint and exact internal reference semantics remain unknown.

Conclusion: delivery race repaired; driver live replacement remains unqualified.
Do not widen the envelope or repeat unchanged. Keep MPPI stop-confirm-replace.

[Driver-command recording and stopped-state DA/DS readback](../../../repository_archives/cleanup_20261102/files/robot-infra/audits/ros-realtime/DRIVER_NATIVE_EVIDENCE.md)
are implemented for the next non-moving diagnostic; physical readback is not yet
verified. Do not repeat live A on the basis of instrumentation availability.

### Next investigation, before further live-update motion

1. Audit exact UART sequence, acknowledgement attribution, retries, driver mode
   and A/C/S units against the protocol; distinguish host bookkeeping from driver
   state. Check whether any command could be duplicated or another response
   mistaken for its ACK. No conclusion about a driver fault from this alone.
2. Extend existing probe recording to capture actual outgoing driver command
   bytes/timestamps and bounded native DA/DS readback. Sample native angle/speed
   while stopped first; do not insert unsolicited queries into a moving command
   stream until response ownership is verified.
3. Characterize isolated position moves, with no replacement: compare requested
   angle, external encoder displacement, driver angle delta, measured travel time
   and peak/steady rates at separately qualified speeds. C's effect in native
   position mode is unknown; receipt-stamped encoder rates alone do not prove
   exact physical speed. Keep fixed deadlines and existing bounds.
4. Check native angle reference across separate completed moves and cancellations
   while disabled, then test stop-confirm replacement using readback. Never reset
   encoder calibration. Establish a reproducible native endpoint/speed contract
   before investigating any new live-update scenario.
5. If the protocol still does not establish live A semantics, request vendor
   confirmation with this trace. No normal-control live update, guard expansion,
   or further hardware motion is authorized by this plan.

## Implementation verification

161 interface/probe/source-policy tests passed; two default-off compile flag
assertions were deselected because the user intentionally enabled those flags.
Ten firmware component executables passed with strict C++11 warnings and UBSan.
`control_interface` rebuilt successfully in the temporary overlay under
`/tmp/catheter-position-m2`. The user compiled/flashed the sketch for the hardware
trials above; the agent did not compile it for the board or actuate hardware.
Physical live replacement semantics remain unqualified. The first old-probe run ended at 465
counts; its cached 203-count ACK sample was 15.6 ms old, and its current-reference
403-count hypothesis nearly coincided with the original 400-count target. The
new classifier does not reinterpret that old run as a pass.
