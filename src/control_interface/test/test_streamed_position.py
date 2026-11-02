"""M1 contract and Arduino-independent firmware tests; never open devices."""
from pathlib import Path
import shutil
import subprocess

import pytest

from control_interface_py.streamed_position import (
    PAYLOAD, StreamedPositionCommand, newer_command_id,
)


def command(**changes):
    fields = dict(command_id=1, duration_us=66667, active_mask=5,
                  target_counts=(100, 0, -200, 0, 0, 0),
                  speed_rpm=(20, 0, 100, 0, 0, 0))
    fields.update(changes)
    return StreamedPositionCommand(**fields)


def test_payload_contract_and_immutable_storage():
    targets = [100, 0, -200, 0, 0, 0]
    c = command(target_counts=targets)
    targets[0] = 300
    assert c.target_counts[0] == 100
    assert len(c.encode_payload()) == PAYLOAD.size == 46
    assert c.encode_payload().hex() == (
        "01010000006b04010005640000000000000038ffffff000000000000000000000000"
        "140000006400000000000000")
    assert StreamedPositionCommand.decode_payload(c.encode_payload()) == c


@pytest.mark.parametrize("changes", [
    {"command_id": 0}, {"command_id": 2**32}, {"command_id": True},
    {"duration_us": 0}, {"duration_us": 250001}, {"duration_us": 66666.7},
    {"active_mask": 64}, {"active_mask": -1},
    {"target_counts": (0,)*5}, {"target_counts": (2**31, 0, 0, 0, 0, 0)},
    {"target_counts": (0.0,)*6}, {"speed_rpm": (251, 0, 100, 0, 0, 0)},
    {"speed_rpm": (0, 0, 100, 0, 0, 0)},
    {"speed_rpm": (20, 1, 100, 0, 0, 0)},
])
def test_rejects_invalid_contract(changes):
    with pytest.raises(ValueError):
        command(**changes)


def test_hold_is_empty_mask_not_a_zero_position_move():
    c = command(active_mask=0, speed_rpm=(0,)*6)
    assert c.target_counts[0] == 100
    assert StreamedPositionCommand.decode_payload(c.encode_payload()) == c


def test_duplicate_order_and_wrap():
    assert newer_command_id(1, 0xFFFFFFFF)
    assert not newer_command_id(1, 1)
    assert not newer_command_id(1, 2)
    assert not newer_command_id(0x80000001, 1)
    with pytest.raises(ValueError):
        newer_command_id(0, 1)


@pytest.fixture(scope="module")
def firmware_binary(tmp_path_factory):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.fail("g++ is required for the M1 offline firmware gate")
    repo = Path(__file__).resolve().parents[3]
    binary = tmp_path_factory.mktemp("streamed-position") / "executor_test"
    subprocess.run([
        compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-fsanitize=undefined", "-fno-sanitize-recover=all",
        str(repo / "firmware/tests/test_streamed_position_executor.cpp"),
        "-o", str(binary),
    ], check=True, capture_output=True, timeout=30)
    return binary


def test_firmware_executor_scenarios(firmware_binary):
    result = subprocess.run([str(firmware_binary)], check=True,
                            capture_output=True, timeout=10)
    assert b"9 streamed-position scenario groups passed" in result.stdout


@pytest.mark.parametrize("c", [command(),
    command(command_id=0xFFFFFFFF, duration_us=250000,
            target_counts=(-2**31, 2**31-1, -200, 0, 0, 0)),
    command(active_mask=0, speed_rpm=(0,)*6),
])
def test_python_firmware_codec_agreement(firmware_binary, c):
    payload = c.encode_payload()
    result = subprocess.run([str(firmware_binary), "codec"], input=payload,
                            check=True, capture_output=True, timeout=10)
    assert result.stdout == payload
    assert StreamedPositionCommand.decode_payload(result.stdout) == c


@pytest.mark.parametrize("mutation", ["version", "length", "duration", "mask", "rpm"])
def test_both_codecs_reject_bad_payload(firmware_binary, mutation):
    payload = bytearray(command().encode_payload())
    if mutation == "version":
        payload[0] = 2
    elif mutation == "length":
        payload.pop()
    elif mutation == "duration":
        payload[5:9] = bytes(4)
    elif mutation == "mask":
        payload[9] = 0x80
    else:
        payload[34:36] = b"\xfb\x00"
    with pytest.raises(ValueError):
        StreamedPositionCommand.decode_payload(payload)
    result = subprocess.run([str(firmware_binary), "codec"], input=payload,
                            capture_output=True, timeout=10)
    assert result.returncode == 2 and result.stdout == b""


def test_streamed_position_is_opt_in_by_default():
    repo = Path(__file__).resolve().parents[3]
    sketch = (repo / "firmware/teensy_tekceleo/teensy_tekceleo.ino").read_text()
    assert '#define TKCTL_STREAMED_POSITION_ENABLED 0' in sketch
    assert '#if TKCTL_STREAMED_POSITION_ENABLED' in sketch
    for name in ("manager.py", "device_serial_com.py"):
        source = (repo / "src/control_interface/control_interface_py" / name).read_text()
        assert "declare_parameter('position_stream_enabled', False)" in source
