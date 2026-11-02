from types import SimpleNamespace
import threading

import pytest

from control_interface.msg import ManagerEvent
from control_interface_py.manager import ControlManager


FAULT = 'V1,L=05,E=00,Q=1,F=6,0,6,0,0,0'
CLEAN = 'V1,L=00,E=00,Q=3,F=0,0,0,0,0,0'


def fixture_manager(statuses=None, reset_response='OK'):
    obj = object.__new__(ControlManager)
    obj._qualification_lock = threading.Lock()
    obj._fault_latched = True
    obj._fault_reason = 'MOTION_CONFIRMED:DRIVER_COMMUNICATION'
    obj._driver_power_qualified = False
    obj._transport_ready = True
    obj.control_mode, obj.active_source = ManagerEvent.NONE, None
    obj.position_stream_session, obj.position_stream_source = 42, 'autonomy'
    obj._enc_history = [(0., [484., 0., 0., 0., 0., 0.])]
    obj.QUALIFICATION_ENC_TOLERANCE = 2.
    obj._command_zero_velocity = lambda: None
    obj._publish_safety_status = lambda **kwargs: None
    obj._wait_for_stable_feedback = lambda: (True, '')
    obj._feedback_is_stable = lambda: True
    obj._probe_all_drivers = lambda: (True, '')
    obj.calls = []
    replies = iter(statuses or [FAULT, FAULT, CLEAN])

    def call(predicate, **kwargs):
        obj.calls.append(predicate)
        # Motion stays inhibited at every maintenance call, including after R.
        assert ControlManager._motion_inhibit_reason(obj) == 'DRIVER_COMMUNICATION_RECOVERY_IN_PROGRESS'
        if predicate == ManagerEvent.FAULT_STATUS:
            return True, next(replies)
        if predicate == ManagerEvent.RESET_FAULT:
            assert obj._fault_latched and not obj._driver_power_qualified
            return True, reset_response
        assert predicate == ManagerEvent.STOP_MOTOR
        return True, 'OK'

    obj._call_device_sync = call
    return obj


def recover(obj):
    return ControlManager.recover_driver_communication(
        obj, None, SimpleNamespace(success=False, message=''))


def test_recovers_only_after_clean_requalification_and_invalidates_session():
    obj = fixture_manager()
    response = recover(obj)
    assert response.success
    assert obj.calls == [ManagerEvent.STOP_MOTOR, ManagerEvent.FAULT_STATUS,
                         ManagerEvent.FAULT_STATUS, ManagerEvent.RESET_FAULT,
                         ManagerEvent.FAULT_STATUS]
    assert not obj._fault_latched and obj._driver_power_qualified
    assert obj._fault_reason == '' and not obj._communication_recovery_active
    assert obj.position_stream_session == 0 and obj.position_stream_source is None
    assert obj.control_mode == ManagerEvent.NONE and obj.active_source is None
    assert obj._qualification_lock.acquire(blocking=False)
    obj._qualification_lock.release()


def test_restarted_manager_reconstructs_only_verified_firmware_communication_fault():
    obj = fixture_manager()
    obj._fault_latched = False
    obj._fault_reason = ''
    assert recover(obj).success
    assert not obj._fault_latched


@pytest.mark.parametrize('status', [
    'V1,L=05,E=01,Q=1,F=6,0,6,0,0,0',
    'V1,L=05,E=00,Q=1,F=6,0,7,0,0,0',
    'V1,L=05,E=00,Q=1,F=6,0,1,0,0,0',
    'V1,L=01,E=00,Q=1,F=6,0,6,0,0,0',
    CLEAN, 'invalid',
])
def test_rejects_enabled_mixed_ambiguous_clean_or_malformed_faults(status):
    obj = fixture_manager([status])
    assert not recover(obj).success
    assert ManagerEvent.RESET_FAULT not in obj.calls
    assert obj._fault_latched and not obj._driver_power_qualified


def test_refuses_new_fault_sequence_after_uart_checks():
    obj = fixture_manager([FAULT, FAULT.replace('Q=1', 'Q=2')])
    assert not recover(obj).success
    assert ManagerEvent.RESET_FAULT not in obj.calls


@pytest.mark.parametrize('stage', ['before', 'after'])
def test_uart_failure_does_not_qualify_power_or_clear_host_latch(stage):
    obj = fixture_manager()
    replies = iter([(False, 'axis 0 timeout')] if stage == 'before'
                   else [(True, ''), (False, 'axis 2 timeout')])
    obj._probe_all_drivers = lambda: next(replies)
    assert not recover(obj).success
    assert (ManagerEvent.RESET_FAULT in obj.calls) == (stage == 'after')
    assert obj._fault_latched and not obj._driver_power_qualified


@pytest.mark.parametrize('reply', ['ERR_NONZERO', 'OK_ENCODER_RESTORED'])
def test_requires_exact_non_encoder_reset_acknowledgement(reply):
    obj = fixture_manager(reset_response=reply)
    assert not recover(obj).success
    assert obj._fault_latched and not obj._driver_power_qualified


def test_rejects_dirty_final_status_and_encoder_drift():
    obj = fixture_manager([FAULT, FAULT, FAULT])
    assert not recover(obj).success
    obj = fixture_manager()
    obj._feedback_is_stable = lambda: (
        obj._enc_history.append((1., [900., 0., 0., 0., 0., 0.])) or True)
    assert not recover(obj).success
    assert obj._fault_latched and not obj._driver_power_qualified


def test_early_refusal_does_not_reset_another_fault_or_active_source():
    for key, value in [('_fault_reason', 'MOTION_CONFIRMED:ENCODER_INTEGRITY'),
                       ('active_source', 'autonomy'), ('_transport_ready', False)]:
        obj = fixture_manager()
        setattr(obj, key, value)
        assert not recover(obj).success
        assert obj.calls == [] and obj._fault_latched
    obj = fixture_manager()
    obj._qualification_lock.acquire()
    assert not recover(obj).success
    assert obj.calls == []
    obj._qualification_lock.release()


def test_stale_feedback_refuses_reset():
    obj = fixture_manager()
    obj._wait_for_stable_feedback = lambda: (False, 'stale ENC')
    assert not recover(obj).success
    assert ManagerEvent.RESET_FAULT not in obj.calls
