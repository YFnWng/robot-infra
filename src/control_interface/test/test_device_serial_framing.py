import struct

import pytest

from control_interface.msg import ManagerEvent
from control_interface_py.device_serial_com import encode_device_command


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
