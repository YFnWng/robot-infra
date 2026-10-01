# Encoder reference recovery — 2026-09-12

This is a single-incident recovery procedure for a mechanism that was confirmed
stationary while motor-driver power corrupted three encoder counters.

## Identity and fixed reference

The temporary build is enabled by
`teensy_tekceleo/encoder_recovery_config.h` and has firmware identity:

```text
tkctl:recovery-exact-frame-v1|<compile date/time>
```

It restores only the confirmed pre-power raw frame:

```text
[-2, -2020, -5, 0, 0, 0]
```

There is no runtime interface for providing another frame. The recovery image
rejects every nonzero velocity command, every position target other than the
all-zero physical home, and raw driver-debug forwarding. The manager and serial
bridge continue to reject encoder-zero changes.

The verified build artifacts are stored outside the source repository at:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/20260912_encoder_recovery_build/
```

Recovery HEX SHA-256:

```text
a1f764d40c451764ddc25c4bc80dbb87f540b572f25f9eda888acfc2e5ba3578
```

## Safety barriers

1. Keep the controller disarmed.
2. Turn motor-driver power off before flashing. Flashing reboots the Teensy.
3. Do not move the mechanism between the confirmed observation and recovery.
4. Do not use MPPI with the recovery firmware.
5. Stop if the firmware identity, raw counts, fault class, or position differs
   from the expected values below.

## Stage 1: flash and verify without driver power

Open `teensy_tekceleo/teensy_tekceleo.ino` in Arduino IDE, select Teensy 4.0,
and upload while motor-driver power remains off. The source currently has the
incident-specific recovery seed enabled.

Restart the ROS control-interface launch and verify its log contains
`tkctl:recovery-exact-frame-v1`. Before applying driver power, inspect raw ENC
and POS. Raw ENC must be the fixed historical frame (within at most a few
stationary counts), and POS must be near:

```text
[-0.000097, -6.8175, 0.000744, 0, 0, 0]
```

If not, stop and leave driver power off.

## Stage 2: apply driver power and qualify

Apply motor-driver power, then use the normal manager qualification service.
The permanent stationary guard may detect the power-on transient. Qualification
will issue STOP, accept only an encoder-integrity-only latch with all motors
disabled, restore the fixed frame, and validate stable POS/ENC before probing
the drivers.

Require all of the following before motion:

- qualification returns success;
- `/manager/safety_status` is `MANAGER_READY`;
- raw ENC again matches `[-2, -2020, -5, 0, 0, 0]` within a few counts;
- POS is inside the configured hard limits.

## Stage 3: move only to physical home

Use the manager's absolute joint-position mode with an all-zero target and a
continuous command heartbeat. The recovery firmware rejects every other target
or nonzero velocity command. Keep MPPI disarmed.

After the transaction completes, issue the normal STOP through the manager and
verify raw ENC is near zero. Do not continue if any axis moved unexpectedly or
the manager/firmware reports a fault.

## Stage 4: return to production firmware

With the mechanism at its established physical home:

1. Turn motor-driver power off.
2. Change `TKCTL_ENCODER_RECOVERY_SEED_ENABLED` to `0` in
   `teensy_tekceleo/encoder_recovery_config.h`.
3. Recompile and upload the production firmware while the mechanism remains at
   home.
4. Confirm the firmware identity begins with `tkctl:boot-v1`, not
   `tkctl:recovery`.
5. Verify near-zero POS/ENC with driver power off before the next power-on and
   qualification test.

Do not leave the recovery image installed after homing.

## Permanent integrity behavior

The production and recovery builds now maintain a separate restoration frame.
It advances during commanded motion and for 250 ms after the last enabled motor
sample, then freezes. While every motor is disabled, all encoder samples are
checked cumulatively against this fixed frame with a 16-count allowance. A
power-on pulse train therefore cannot advance the restoration reference one
small frame at a time. Encoder-integrity reset writes the fixed restoration
frame back to all hardware counters before resuming telemetry.
