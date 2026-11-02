import threading
from types import SimpleNamespace
import pytest
from control_interface_py.driver_evidence import HEADER, decode_trace, parse_state
from control_interface_py.manager import ControlManager
from control_interface.msg import ManagerEvent


def reply(angle='Angle:21.0', speed='speed:0.0'):
    return 'OK_DRIVER_STATE,V1,A=0,ENC=401,T0=123,T1=234,DA='+angle.encode().hex()+',DS='+speed.encode().hex()


def test_query_response_is_not_a_command_ack_and_speed_semantics_remain_explicit():
    r=parse_state(reply())
    assert r['driver_angle_deg']==21 and r['encoder_count']==401
    assert r['driver_reported_rpm']==0
    assert parse_state(reply(angle='angle:338.085'))['driver_angle_deg']==338.085
    assert parse_state(reply(speed='vitesse:36.0'))['driver_reported_rpm']==36
    for text in (reply(angle='!'),reply(angle='Angle:NaN'),reply(speed='!OK'), 'ERR_DRIVER_STATE_REPLY'):
        with pytest.raises(ValueError): parse_state(text)


def test_trace_preserves_uart_bytes_and_explicit_loss_counter():
    raw=b'$A4.5\n'
    text, data=decode_trace(HEADER.pack(1,2,123,4,0,0,len(raw))+raw)
    assert text=='DRIVER_UART:TX:'+raw.hex()
    assert data[:6]==[2,123,4,0,0,6]
    assert bytes(int(x) for x in data[6:])==raw
    with pytest.raises(ValueError): decode_trace(b'bad')


@pytest.mark.parametrize('blocked', ['capability','mode','fault','moving','request','response'])
def test_guarded_readback_never_claims_or_enables_and_rejects_bad_state(blocked):
    calls=[]
    manager=SimpleNamespace(driver_evidence_capable=True, _qualification_lock=threading.Lock(),
        position_stream_opening=False,control_mode=ManagerEvent.NONE,active_source=None,
        _motion_inhibit_reason=lambda:'',_feedback_is_stable=lambda:True,
        _call_device_sync=lambda *a,**kw: (calls.append((a,kw)) or (True,reply())))
    request=SimpleNamespace(predicate=ManagerEvent.DRIVER_DIAGNOSTIC,cmd='0',data=[])
    if blocked=='capability': manager.driver_evidence_capable=False
    if blocked=='mode': manager.active_source='autonomy'
    if blocked=='fault': manager._motion_inhibit_reason=lambda:'FAULT'
    if blocked=='moving': manager._feedback_is_stable=lambda:False
    if blocked=='request': request.cmd='1'
    if blocked=='response': manager._call_device_sync=lambda *a,**kw:(True,'malformed')
    response=ControlManager.read_driver_state(manager,request,SimpleNamespace(success=True,response=''))
    assert not response.success and not manager.position_stream_opening
    assert not manager._qualification_lock.locked()
    assert calls==[]


def test_guarded_readback_only_dispatches_h_read_query():
    calls=[]
    manager=SimpleNamespace(driver_evidence_capable=True,_qualification_lock=threading.Lock(),
        position_stream_opening=False,control_mode=ManagerEvent.NONE,active_source=None,
        _motion_inhibit_reason=lambda:'',_feedback_is_stable=lambda:True,
        _call_device_sync=lambda *a,**kw:(calls.append((a,kw)) or (True,reply())))
    r=ControlManager.read_driver_state(manager,
        SimpleNamespace(predicate=ManagerEvent.DRIVER_DIAGNOSTIC,cmd='0',data=[]),
        SimpleNamespace(success=False,response=''))
    assert r.success and calls==[((ManagerEvent.DRIVER_DIAGNOSTIC,),{'cmd':'r0','timeout':1.})]
    assert manager.control_mode==ManagerEvent.NONE and manager.active_source is None


def test_readback_cli_cannot_be_combined_with_execution(tmp_path):
    from control_interface_py.position_stream_probe import main
    output=tmp_path/'not_created.json'
    with pytest.raises(SystemExit): main(['--driver-state','--execute','--output',str(output)])
    assert not output.exists()
