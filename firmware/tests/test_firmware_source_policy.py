"""Static safety contracts for command dispatch in the Teensy sketch."""

from pathlib import Path


SKETCH = (
    Path(__file__).resolve().parents[1]
    / "teensy_tekceleo"
    / "teensy_tekceleo.ino"
)
RECOVERY_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "teensy_tekceleo"
    / "encoder_recovery_config.h"
)


def _case_body(source: str, label: str, next_label: str) -> str:
    start = source.index(f"case {label}:")
    end = source.index(f"case {next_label}:", start)
    return source[start:end]


def test_raw_encoder_zero_command_is_rejected_in_firmware():
    source = SKETCH.read_text(encoding="utf-8")
    body = _case_body(source, "ZERO", "STOP")

    assert 'sendAckIfPending("ERR_ZERO_FORBIDDEN")' in body
    assert "enforceImmediateMotionStop()" in body
    assert "writeEncoderHardwareCounts" not in body


def test_incident_recovery_seed_is_disabled_by_default():
    source = RECOVERY_CONFIG.read_text(encoding="utf-8")

    assert (
        "#define TKCTL_ENCODER_RECOVERY_SEED_ENABLED 0" in source
    )


def test_native_startup_cancellation_preserves_terminal_status():
    source = SKETCH.read_text(encoding="utf-8")
    body = source.split("void updatePositionStream() {", 1)[1].split(
        "void reportPositionStream(", 1)[0]
    startup = body.split("const bool success = CoordinatedStart::run(", 1)[1]
    assert startup.index("if (!streamActive) return;") < startup.index("if (success)")
    assert "configureNativeMotorPosition(a, update.native_deci_degrees[a])" in startup
    assert "newJointVelCmd = false;" in startup


def test_live_probe_compile_opt_in_by_default():
    source = SKETCH.read_text(encoding="utf-8")
    assert '#define TKCTL_NATIVE_LIVE_PROBE_ENABLED 0' in source


def test_live_probe_sends_only_angle_without_renewal():
    source = SKETCH.read_text(encoding="utf-8")
    body = _case_body(source, 'NATIVE_LIVE_UPDATE', 'VEL')
    assert 'configureNativeMotorPosition(0, 45)' in body
    assert 'positionStream.expired(micros())' in body
    assert 'enforceImmediateMotionStop()' in body
    for forbidden in ('coordinatedEnableMask(', 'initializeMotorDriver(', 'configureMotorDirection(',
                      'configureMotorSpeed(', 'motionWatchdog.refresh('):
        assert forbidden not in body


def test_live_encoder_bracket_is_at_uart_write_and_ack_and_cannot_retry():
    source = SKETCH.read_text(encoding='utf-8')
    body = source.split('static bool writeMotorCommandWithAck(', 1)[1].split('static inline void stopMotorAxis', 1)[0]
    assert 'liveAngle ? 1 : MOTOR_ACK_ATTEMPTS' in body
    assert body.index('reportPositionStream(14,') < body.index('writeMotorCommand(axis, command, length);')
    assert body.index('current = waitForMotorAck(axis);') < body.index('reportPositionStream(15,')
    assert body.count('!readEncoders()') == 2


def test_stopped_readback_sends_only_information_queries():
    source = SKETCH.read_text(encoding='utf-8')
    body = source.split('void handleDriverDiagnosticCommand() {',1)[1].split('constexpr size_t kExpectedLength',1)[0]
    assert '!motorEnabled[a]' in body and '!motionWatchdog.armed()' in body
    assert 'readNativeDriverLine("$DA\\n", "Angle:"' in body
    assert 'readNativeDriverLine("$DS\\n", "speed:"' in body
    for command in ('initializeMotorDriver(', 'configureMotorSpeed(', 'configureMotorPosition(', 'coordinatedEnableMask('):
        assert command not in body
    assert 'DriverEvidence::validReadback(output,prefix)' in source


def test_uart_evidence_drains_only_disabled_and_preserves_write_capture():
    source=SKETCH.read_text(encoding='utf-8')
    assert 'pc_connected && !anyMotorAxisEnabled() && !motionWatchdog.armed()' in source
    assert 'driverEvidence.capture(started, axis, 0, command, written)' in source


def test_arduino_prototype_insertion_cannot_precede_stream_type_include():
    source=SKETCH.read_text(encoding='utf-8')
    include=source.index('#include "streamed_position_session.h"')
    first_stream_function=source.index('static StreamedPosition::Executor::Config streamConfig() {')
    helper_body=source.index('static void captureDriverEvidence(uint8_t axis, uint8_t phase, const void* bytes, size_t size) {')
    assert include < first_stream_function < helper_body
    assert 'void readRawEncoderCounts(int32_t* counts);' in source
    assert 'void readRawEncoderCounts(int32_t counts[numHWSerials])' not in source


def test_on_the_fly_velocity_is_a_default_off_test_build():
    source = SKETCH.read_text(encoding='utf-8')
    assert '#define TKCTL_ONTHEFLY_VELOCITY_ENABLED 0' in source
    assert '#if TKCTL_ONTHEFLY_VELOCITY_ENABLED && TKCTL_ENCODER_RECOVERY_SEED_ENABLED\n#error' in source
    identity = source.split('constexpr char FIRMWARE_BUILD_ID[] =\n    "tkctl:boot-v1|"', 1)[1].split(';', 1)[0]
    assert '#if TKCTL_ONTHEFLY_VELOCITY_ENABLED\n    "onthefly_velocity_v1|"\n#endif' in identity


def test_legacy_velocity_restart_path_is_preserved_when_flag_is_off():
    source = SKETCH.read_text(encoding='utf-8')
    body = source.split('void sendMotorCmds() {', 1)[1].split('} else if (newJointPosCmd) {', 1)[0]
    live, legacy = body.split('#if TKCTL_ONTHEFLY_VELOCITY_ENABLED', 1)[1].split('#else', 1)
    legacy = legacy.split('#endif', 1)[0]
    # Legacy: any start or reversal stops, settles and restarts every active axis.
    assert 'coordinatedStartRequired = true;' in legacy
    assert 'stopMotorMask(activeMask, true);\n      delay(MOTOR_DIRECTION_SETTLE_MS);' in legacy
    assert 'activeMask, numHWSerials,' in legacy
    # On the fly: only axes from rest are stopped/started, and failures stop every active axis.
    assert 'stopMotorMask(plan.start_mask, true);' in live
    assert 'plan.start_mask, numHWSerials,' in live
    assert 'configureMotorDirection(axis, true) && configureMotorSpeed(axis)' in live
    assert 'if (!success) stopMotorMask(activeMask, true);' in live
    assert '[&](uint8_t) { stopMotorMask(activeMask, true); }' in live
    assert 'if (success) setActiveMonitorCommands(activeMask);' in body.split('#endif', 1)[1]
