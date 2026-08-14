import struct
from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Time
from control_interface.msg import ManagerEvent
from control_interface_py.device_serial_com import (
    SerialCommunication, encode_device_command)


def test_service_float_data_is_serialized_after_uuid():
    request_id = bytes(range(16))
    payload = encode_device_command(
        ManagerEvent.SET_TARGET_VEL, request_id, '', [1, 2, 3, 4, 5, 6])
    assert payload[:17] == bytes([ManagerEvent.SET_TARGET_VEL]) + request_id
    assert struct.unpack('<6f', payload[17:]) == pytest.approx(
        [1, 2, 3, 4, 5, 6])


def test_target_velocity_rejects_text_and_bad_request_id():
    with pytest.raises(ValueError):
        encode_device_command(
            ManagerEvent.SET_TARGET_VEL, bytes(16), 'unexpected', [1] * 6)
    with pytest.raises(ValueError):
        encode_device_command(ManagerEvent.FAULT_STATUS, bytes(15))


def test_protocol_v3_motion_event_exposes_driver_response():
    published = []
    fake = SimpleNamespace(
        event_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=Time)),
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
    )
    legacy_state = bytes([110] * 6)
    structured = struct.pack(
        '<BBBBBHfffHH', 3, ManagerEvent.MOTION_CONFIRMED,
        ManagerEvent.FAULT_DRIVER_COMMUNICATION, 4, 5, 7,
        25.0, 0.0, 0.0, 56, 0)
    diagnostics = bytes([
        ord('S'), 4, 3, 4, ord('!'), ord('E'), ord('R'), ord('R')])

    SerialCommunication.handle_device_event(
        fake, ManagerEvent.STALL, legacy_state + structured + diagnostics)

    assert len(published) == 1
    assert list(published[0].data[-7:]) == pytest.approx([
        4, 3, 4, ord('!'), ord('E'), ord('R'), ord('R')])
