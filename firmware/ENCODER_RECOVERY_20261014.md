# Chassis-mark reference recovery — 2026-10-14

Incident-specific source preparation, not a hardware qualification. The operator
identified physical insertion approximately +20 mm by the original chassis mark
while reported POS was 0.0013137 mm and raw ENC was [1,0,-6,0,0,0]. Do not run a
negative target under the production reference or use the older recovery image.
No SET_ZERO, calibration reset or index command is used.

The restricted build restores raw [47535,0,-6,0,0,0]: reported frame plus 47534
axis-0 counts, equivalent to 19.999883 mm with the current transmission. This
is a mark-derived offset, not recovered exact historical counts. Expected
logical POS is approximately [20.001197,0,0.000893,0,0,0]. Other physical axis
references are NOT verified by the chassis mark.

## Prepare and verify, without motor power

1. Stop the recorded session/controller and leave the robot stationary. Turn
   motor-driver power OFF before flashing; do not move after recording the frame.
2. In `teensy_tekceleo/encoder_recovery_config.h`, set
   `TKCTL_ENCODER_RECOVERY_SEED_ENABLED` from 0 to 1. It is off by default.
3. Compile/upload the sketch. Its ordinary default stream/live-probe flags
   automatically become 0 in recovery; externally forcing stream=1 is rejected
   by the existing compile-time guard. No normal controller may run with this image.
4. Restart the control interface with the normal `imricor_test` limits and
   streamed-position/live-probe host flags false. Require identity prefix
   `tkctl:recovery-chassis-mark-20261014|`. Inspect ENC/POS with driver power off;
   stop if they differ from the expected frame or exceed stationary tolerance.

## Guarded return to original physical home

Apply driver power and qualify only through the existing manager workflow.
Require MANAGER_READY, stable expected ENC/POS and no fault. Use the manager's
ordinary absolute JOINT_POS transaction with target [0,0,0,0,0,0] and speed
[2,0,0,0,0,0], with its required command heartbeat. The recovery image rejects
other targets, all nonzero velocity commands and raw driver-debug forwarding;
it additionally rejects other-axis position speeds and insertion speed above
2 mm/s. It must move only chassis axis 0 toward the original mark. Existing
manager bounds, firmware limits, stop/deadman and integrity/fault latches remain.

Supervise the mechanical mark and stop through the manager if movement is wrong,
the mark is crossed, another shaft moves, or any fault occurs. Do not retry or
clear a fault automatically. A near-zero encoder endpoint is not proof of exact
mechanical alignment: the mark-derived offset and native angular positioning
both have uncertainty. Do not redefine the physical home to conceal that error.

## Return to production

After manager STOP and verified standstill, independently verify the original
physical home marks for all axes. If any other physical axis reference is
uncertain, STOP: do not flash production at an unverified multi-axis home.
With all axes at established home and driver power OFF, restore recovery flag
to 0 and compile/upload production. Verify boot-v1 identity and near-zero
ENC/POS before applying/qualifying power again. Reinitialization of the ordinary
boot counters is allowed only at the already-established physical home, not as
a runtime encoder-zero command. Never leave the recovery image installed.

The older 2026-09-12 document is historical; its identity/seed are not this build.
No firmware was flashed or motion performed by the agent.

Offline verification: the recovery configuration test compiled with strict
C++11 warnings and UBSan and passed, covering the offset arithmetic, preserved
other-axis frame, speed ceiling, wrong-axis speeds and nonfinite input rejection.
Eight existing firmware source-policy checks passed with one production
default-off assertion deselected; these textual checks are not a qualification
of the user's currently enabled native diagnostic configuration. Actual Teensy sketch
compilation/upload and physical recovery remain unverified.
