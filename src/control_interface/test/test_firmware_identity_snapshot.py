from pathlib import Path
from types import SimpleNamespace
from std_msgs.msg import String
from control_interface.msg import ManagerEvent
from control_interface_py.manager import ControlManager
from control_interface_py.device_serial_com import SerialCommunication

BUILD='tkctl:boot-v1|encoder_target_native_position_v1|native_live_update_bracket_probe_v1|driver_native_evidence_v1|date'


def manager():
    node=ControlManager.__new__(ControlManager)
    node._firmware_identity=''; node.position_stream_session=0; node.position_stream_source='autonomy'
    node.active_source=None; node.control_mode=ManagerEvent.NONE
    node._transport_ready=False; node._driver_power_qualified=False
    node._fault_latched=False
    node._command_zero_velocity=lambda: None
    node._publish_safety_status=lambda **kwargs: None
    node.get_logger=lambda:SimpleNamespace(error=lambda text:None)
    node._apply_firmware_capabilities('')
    return node


def test_snapshot_recovers_all_capabilities_without_boot_event():
    node=manager()
    node.firmware_identity_callback(String(data=BUILD))
    assert node.position_stream_capable and node.native_live_probe_capable and node.driver_evidence_capable
    assert not node._driver_power_qualified and node.position_stream_session==0
    # Older OPEN delivered after snapshot cannot permanently erase capability.
    node.transport_status_callback(String(data='SERIAL_OPEN:/dev/test'))
    assert not node.driver_evidence_capable
    node.transport_status_callback(String(data='SERIAL_READY:/dev/test'))
    assert node.driver_evidence_capable and not node._driver_power_qualified


def test_disconnect_and_old_firmware_clear_all_capabilities_and_sessions():
    node=manager(); node.firmware_identity_callback(String(data=BUILD))
    node.position_stream_session=42
    node.transport_status_callback(String(data='SERIAL_DISCONNECTED:test'))
    assert not node.driver_evidence_capable and not node.native_live_probe_capable and not node.position_stream_capable
    assert node.position_stream_session==0
    node.transport_status_callback(String(data='SERIAL_READY:/dev/test'))
    assert not node.driver_evidence_capable
    node.firmware_identity_callback(String(data='old-firmware'))
    assert not node.driver_evidence_capable


def test_replaying_same_identity_does_not_cancel_valid_session_but_change_does():
    node=manager(); node.firmware_identity_callback(String(data=BUILD))
    node.position_stream_session=42; node.active_source='autonomy'
    node.firmware_identity_callback(String(data=BUILD))
    assert node.position_stream_session==42 and not node._fault_latched
    node.firmware_identity_callback(String(data=BUILD+'new-build'))
    assert node.position_stream_session==0 and node._fault_latched and node.active_source is None


def test_snapshot_qos_separate_from_trace_and_bridge_clear_publication():
    root=Path(__file__).resolve().parents[1]/'control_interface_py'
    bridge=(root/'device_serial_com.py').read_text()
    owner=(root/'manager.py').read_text()
    assert "String, '/device/firmware_identity', transport_qos" in bridge
    assert "String, '/device/firmware_identity', self.firmware_identity_callback, transport_qos" in owner
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in bridge and 'ReliabilityPolicy.RELIABLE' in bridge
    published=[]
    fake=SimpleNamespace(firmware_identity_pub=SimpleNamespace(publish=published.append))
    SerialCommunication._publish_firmware_identity(fake,BUILD)
    SerialCommunication._publish_firmware_identity(fake,'')
    assert [m.data for m in published]==[BUILD,'']
    assert "self._publish_firmware_identity('')" in bridge.split('def close(',1)[1].split('def send_bytes',1)[0]
