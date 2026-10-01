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
