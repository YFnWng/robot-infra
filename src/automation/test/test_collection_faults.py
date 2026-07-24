from types import SimpleNamespace

import numpy as np
import pytest

from automation.collection.node import CollectionNode, parse_fault_status
from control_interface.msg import DeviceEvent, ManagerEvent


def test_collection_velocity_bounds_preserve_zero_and_lift_nonzero_speed():
    fake = SimpleNamespace(
        _min_speeds=np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        _max_speeds=np.array([10.0, 40.0, 1.0, 4.0, 25.0, 25.0]),
    )
    out = CollectionNode._apply_velocity_bounds(
        fake, np.array([0.5, 0.0, -2.0, 0.0, 0.0, 0.0]))
    assert out.tolist() == [2.0, 0.0, -1.0, 0.0, 0.0, 0.0]


def test_parse_fault_status():
    status = parse_fault_status("V1,L=05,E=00,Q=17,F=2,0,2,0,0,0")
    assert status["latched_mask"] == 0x05
    assert status["enabled_mask"] == 0
    assert status["sequence"] == 17
    assert status["faults"] == [2, 0, 2, 0, 0, 0]


@pytest.mark.parametrize("response", ["", "V2,L=00", "V1,L=00,E=00,Q=1"])
def test_parse_fault_status_rejects_malformed(response):
    with pytest.raises((KeyError, ValueError)):
        parse_fault_status(response)


def test_confirmed_fault_finishes_active_run():
    finished = []
    fake = SimpleNamespace(
        _t0=object(), _done=False, _returning=True,
        _return_status="in_progress", _hardware_fault=None,
        _run_status="running",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=123)),
        get_logger=lambda: SimpleNamespace(error=lambda message: None),
        _finish=lambda: finished.append(True),
    )
    msg = DeviceEvent()
    msg.predicate = ManagerEvent.STALL
    msg.text = "MOTION_CONFIRMED:DRIVER_COMMUNICATION"
    msg.data = [float(value) for value in [
        2, ManagerEvent.MOTION_CONFIRMED,
        ManagerEvent.FAULT_DRIVER_COMMUNICATION,
        1, -1, 9, 1.0, 0.0, 0.0, 2, 0, ord("O")]]

    CollectionNode._device_event_cb(fake, msg)

    assert finished == [True]
    assert fake._run_status == "hardware_fault"
    assert fake._return_status == "aborted_hardware_fault"
    assert fake._hardware_fault["driver_stage"] == "O"


def test_run_end_records_final_fault_status_separately_from_preflight():
    markers = []
    preflight = {"latched_mask": 0, "enabled_mask": 0}
    fake = SimpleNamespace(
        _finish_return_result={"error_target_minus_final": None},
        _run_status="hardware_fault",
        _hardware_fault={"axis": 0},
        _fault_status=preflight,
        _enc_seen=True,
        _shutdown_on_done=True,
        should_exit=False,
        _marker=lambda event, **fields: markers.append((event, fields)),
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None, info=lambda message: None),
    )
    response = SimpleNamespace(
        success=True, response="V1,L=05,E=00,Q=2,F=1,0,1,0,0,0")
    future = SimpleNamespace(result=lambda: response)

    CollectionNode._on_final_fault_status(fake, future)

    assert fake.should_exit
    assert markers[0][0] == "run_end"
    fields = markers[0][1]
    assert fields["preflight_fault_status"] is preflight
    assert fields["fault_status"]["latched_mask"] == 0x05
    assert fields["fault_status"]["faults"] == [1, 0, 1, 0, 0, 0]
