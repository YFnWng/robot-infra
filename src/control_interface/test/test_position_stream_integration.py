from pathlib import Path
import subprocess
import pytest
from control_interface_py.streamed_position import (
    OPEN_SESSION, PositionEnvelope, StreamedPositionCommand, decode_status, STATUS_PAYLOAD,
)
from control_interface_py.streamed_position_native import Config, NativePositionSession
from control_interface_py.device_serial_com import encode_device_command
from control_interface_py.device_serial_com import SerialCommunication
from types import SimpleNamespace
import threading
from control_interface.msg import ManagerEvent, StreamedPositionControl
from control_interface_py.manager import ControlManager
from builtin_interfaces.msg import Time


@pytest.fixture
def native(tmp_path):
    root = Path(__file__).resolve().parents[3]
    library = tmp_path/'position.so'
    subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-shared', '-fPIC', '-I'+str(root/'firmware/teensy_tekceleo'),
                    str(root/'src/control_interface/native/position_stream.cpp'), '-o', str(library)],
                   check=True, capture_output=True, timeout=30)
    config = Config()
    for a in range(6):
        config.lower[a], config.upper[a] = -10000, 10000
        config.step[a], config.tolerance[a] = 1000, 2
        config.minimum_rpm[a], config.maximum_rpm[a] = 20, 250
    session = NativePositionSession(library, config)
    yield session
    session.close()


def command(command_id=1):
    return StreamedPositionCommand(command_id, 66667, 1, (100, 0, 0, 0, 0, 0), (20, 0, 0, 0, 0, 0))


def test_envelope_roundtrip_preserves_integer_bits():
    packet = PositionEnvelope(0xffffffff, 20000, command(0xffffffff))
    assert PositionEnvelope.decode_payload(packet.encode_payload()) == packet
    assert PositionEnvelope.from_data(packet.as_data()) == packet
    data = packet.as_data(); data[-1] = .5
    with pytest.raises(ValueError):
        PositionEnvelope.from_data(data)


def test_session_open_has_no_motion_payload():
    assert encode_device_command(OPEN_SESSION, bytes(16), data=[123]) == bytes([OPEN_SESSION])+bytes(16)+b'{\0\0\0'
    with pytest.raises(ValueError):
        encode_device_command(OPEN_SESSION, bytes(16), data=[1, 2])


def test_raw_device_service_cannot_bypass_manager_for_stream_motion():
    from control_interface_py.streamed_position import STREAM_COMMAND
    result = SerialCommunication.handle_device_command(SimpleNamespace(),
        SimpleNamespace(predicate=STREAM_COMMAND), SimpleNamespace(success=True, response=''))
    assert not result.success and result.response == 'POSITION_STREAM_MANAGER_ONLY'


def test_transport_age_and_heartbeat_do_not_extend_lease(native):
    assert native.open(False) == 0
    token = native.open(True)
    assert native.submit(PositionEnvelope(token, 20000, command()), 100000, [0]*6) == 0
    assert native.update(119999, [0]*6, [20, 0, 0, 0, 0, 0]).status == 1
    assert native.submit(PositionEnvelope(token, 10000, command()), 119999, [0]*6) == 1
    result = native.update(120000, [0]*6, [20, 0, 0, 0, 0, 0])
    assert result.status == 3 and result.rpm[0] == 0


def test_new_session_rejects_old_packets(native):
    first, second = native.open(True), native.open(True)
    assert second != first
    assert native.submit(PositionEnvelope(first, 66667, command()), 0, [0]*6) == -1
    assert native.submit(PositionEnvelope(second, 66667, command()), 0, [0]*6) == 0
    native.cancel()
    assert native.submit(PositionEnvelope(second, 66667, command()), 1000, [0]*6) == 1
    assert native.update(1000, [0]*6, [0]*6).status == 4


def test_status_contains_device_time_counts_and_applied_rpm():
    text, values = decode_status(STATUS_PAYLOAD.pack(1, 2, 1234, 1000, 2, 0,
                                100, 0, -200, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    assert text == 'POSITION_STREAM:COMPLETE'
    assert values[:5] == [1., 2., 1234., 1000., 2.]
    assert values[6:9] == [100., 0., -200.]


def test_native_start_event_and_old_firmware_capability_is_rejected():
    from control_interface_py.streamed_position import CAPABILITY
    assert CAPABILITY not in 'encoder_target_stream_v1'
    text, _ = decode_status(STATUS_PAYLOAD.pack(1, 2, 1234, 1000, 12, 0, *([0]*12)))
    assert text == 'POSITION_STREAM:NATIVE_STARTED'


def relay_fixture():
    sent = []
    clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10000000000))
    manager = SimpleNamespace(position_stream_enabled=True, position_stream_capable=True,
        position_stream_session=42, position_stream_source='catheter_mppi', active_source='catheter_mppi',
        control_mode=ManagerEvent.JOINT_VEL, deadman=False, estop=False,
        _lock_blocks=lambda src: False, _motion_inhibit_reason=lambda: '',
        get_clock=clock, _last_command_stamp_ns={'catheter_mppi': 0}, last_input_time={},
        COMMAND_MAX_AGE=.1, COMMAND_FUTURE_TOLERANCE=.05,
        _pos_lower=[-100]*6, _pos_upper=[100]*6, _vel_min=[0]*6, _vel_max=[20]*6,
        control_pub=SimpleNamespace(publish=sent.append),
        _command_zero_velocity=lambda: None, get_logger=lambda: SimpleNamespace(warn=lambda text: None))
    message = StreamedPositionControl()
    message.header.stamp, message.header.frame_id = Time(sec=10), 'catheter_mppi'
    message.session_id, message.command_id, message.duration_us = 42, 1, 66667
    message.deadline_ros_ns, message.active_mask = 10030000000, 1
    message.target_counts, message.speed_rpm = [100, 0, 0, 0, 0, 0], [36, 0, 0, 0, 0, 0]
    return manager, message, sent, clock


def test_manager_to_serial_uses_exact_binary_counts_and_reduced_lifetime():
    manager, message, sent, clock = relay_fixture()
    ControlManager.position_stream_callback(manager, message)
    assert len(sent) == 1
    wire, traces = [], []
    bridge = SimpleNamespace(position_stream_enabled=True, position_stream_session=42,
        firmware_identity='encoder_target_native_position_v1', control_tx_lock=threading.Lock(),
        get_clock=clock, _last_control_stamp_ns=0, command_max_age_s=.1,
        command_future_tolerance_s=.05,
        send_bytes=lambda payload: wire.append(payload) or True,
        command_tx_pub=SimpleNamespace(publish=traces.append))
    SerialCommunication.on_manager_control(bridge, sent[0])
    packet = PositionEnvelope.decode_payload(wire[0][1:])
    assert packet.session_id == 42 and packet.remaining_us == 30000
    assert packet.command.target_counts[0] == 100
    assert packet.command.duration_us == 66667
    assert len(traces) == 1


def test_native_stream_rejects_speed_below_reliable_physical_floor():
    manager, message, sent, _ = relay_fixture()
    manager._vel_min[0] = 2.0
    message.speed_rpm[0] = 1
    ControlManager.position_stream_callback(manager, message)
    assert sent == []


def test_live_update_manager_is_opt_in_source_bound_and_one_shot():
    from control_interface_py.streamed_position import LIVE_UPDATE, live_update_payload
    manager, goal, sent, _ = relay_fixture()
    manager.native_live_probe_enabled = True
    manager.native_live_probe_capable = True
    goal.header.frame_id = 'autonomy'
    manager.active_source = manager.position_stream_source = 'autonomy'
    manager._last_command_stamp_ns['autonomy'] = 0
    manager._native_live_probe_goal = goal
    manager._native_live_probe_used = None
    ControlManager.position_stream_callback(manager, goal)
    assert manager._native_live_probe_goal is goal
    sent.clear()
    update = SimpleNamespace(header=SimpleNamespace(frame_id='autonomy', stamp=Time(sec=10, nanosec=1)),
                             predicate=LIVE_UPDATE, data=[42., 1., 45.])
    ControlManager.native_live_update_callback(manager, update)
    assert sent == [update]
    manager._native_live_probe_goal = goal # ordinary heartbeat cannot reauthorize
    ControlManager.native_live_update_callback(manager, update)
    assert sent == [update]
    manager._native_live_probe_used = None
    manager.native_live_probe_enabled = False
    ControlManager.native_live_update_callback(manager, update)
    assert sent == [update]
    assert len(live_update_payload([42., 1., 45.])) == 10
    with pytest.raises(ValueError):
        live_update_payload([42., 1., 46.])


def test_live_update_raw_service_is_rejected():
    from control_interface_py.streamed_position import LIVE_UPDATE
    result = SerialCommunication.handle_device_command(SimpleNamespace(),
        SimpleNamespace(predicate=LIVE_UPDATE), SimpleNamespace(success=True, response=''))
    assert not result.success


@pytest.mark.parametrize('bad', ['disabled', 'session', 'source', 'deadline', 'mode'])
def test_manager_rejects_wrong_authority_or_expired_endpoint(bad):
    manager, message, sent, _ = relay_fixture()
    if bad == 'disabled': manager.position_stream_enabled = False
    elif bad == 'session': message.session_id = 43
    elif bad == 'source': message.header.frame_id = 'autonomy'
    elif bad == 'deadline': message.deadline_ros_ns = 10000000000
    else: manager.control_mode = ManagerEvent.JOINT_POS
    ControlManager.position_stream_callback(manager, message)
    assert sent == []
