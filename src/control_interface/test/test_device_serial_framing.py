import struct
import threading
import time
from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Time
from control_interface.msg import DeviceStream, ManagerEvent
from control_interface.srv import DeviceCmd
from rclpy.task import Future
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


def test_device_service_never_transmits_set_zero():
    fake = SimpleNamespace()
    request = SimpleNamespace(predicate=ManagerEvent.SET_ZERO)
    response = SimpleNamespace(success=True, response='')

    result = SerialCommunication.handle_device_command(fake, request, response)

    assert result.success is False
    assert result.response == 'SET_ZERO_FORBIDDEN'


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


def test_preconnect_firmware_status_reports_identity_without_transport_ready():
    published = []
    statuses = []
    information = []
    fake = SimpleNamespace(
        event_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=Time)),
        get_logger=lambda: SimpleNamespace(info=information.append),
        firmware_alive=threading.Event(),
        firmware_identity=None,
        _logged_firmware_identity=None,
        rx_ready=threading.Event(),
        _set_transport_status=statuses.append,
    )
    build = b'teensy_tekceleo:bootdiag-v1|built=test'
    body = bytes([1, 0]) + struct.pack('<I', 1234) + build

    SerialCommunication.handle_device_event(
        fake, ManagerEvent.FIRMWARE_STATUS, body)

    assert fake.firmware_alive.is_set()
    assert not fake.rx_ready.is_set()
    assert fake.firmware_identity == build.decode()
    assert published[0].text == f'FIRMWARE_ALIVE:{build.decode()}'
    assert list(published[0].data) == [1.0, 0.0, 1234.0]
    assert statuses == [
        f'SERIAL_FIRMWARE_ALIVE_AWAITING_CONNECT:{build.decode()}']
    assert information


def test_firmware_status_exposes_watchdog_stop_flag():
    published = []
    fake = SimpleNamespace(
        event_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=Time)),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        firmware_alive=threading.Event(),
        firmware_identity=None,
        _logged_firmware_identity=None,
        rx_ready=threading.Event(),
        _set_transport_status=lambda status: None,
    )
    body = bytes([1, 0x05]) + struct.pack('<I', 1500) + b'test-build'

    SerialCommunication.handle_device_event(
        fake, ManagerEvent.FIRMWARE_STATUS, body)

    assert published[0].text == 'FIRMWARE_WATCHDOG_STOP:test-build'
    assert published[0].data[1] == 5.0


def test_stall_retry_transition_is_named_and_preserves_attempt_number():
    published = []
    fake = SimpleNamespace(
        event_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=Time)),
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
    )
    legacy_state = bytes([110] * 6)
    structured = struct.pack(
        '<BBBBBHfffHH', 3, ManagerEvent.MOTION_RETRYING,
        ManagerEvent.FAULT_STALL, 1, 255, 8,
        -40.0, 0.0, 0.0, 89, 209)
    diagnostics = bytes([2, 0, 0, 0])

    SerialCommunication.handle_device_event(
        fake, ManagerEvent.STALL, legacy_state + structured + diagnostics)

    assert published[0].text == 'MOTION_RETRYING:STALL'
    assert published[0].data[4] == -1
    assert published[0].data[11] == 2


def test_stream_handler_rejects_wrong_payload_size():
    fake = SimpleNamespace()
    with pytest.raises(ValueError, match='requires 24 bytes'):
        SerialCommunication.handle_device_stream(
            fake, DeviceStream.POS, bytes(20))


def test_five_state_limit_snapshot_is_valid():
    published = []
    warnings = []
    fake = SimpleNamespace(
        event_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=Time)),
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    states = bytes([
        ManagerEvent.TRIGGER_N,
        ManagerEvent.TRIGGER_N,
        ManagerEvent.TRIGGER_N,
        ManagerEvent.TRIGGER_C,
        ManagerEvent.TRIGGER_M,
    ])

    SerialCommunication.handle_device_event(
        fake, ManagerEvent.LIMIT, states)

    assert list(published[0].state) == list(states)
    assert warnings


def test_serial_writes_are_atomic_across_threads():
    class DetectConcurrentPort:
        is_open = True

        def __init__(self):
            self.active = False
            self.overlap = False
            self.frames = []

        def write(self, frame):
            if self.active:
                self.overlap = True
            self.active = True
            time.sleep(0.001)
            self.frames.append(frame)
            self.active = False
            return len(frame)

    port = DetectConcurrentPort()
    fake = SimpleNamespace(
        serial_port=port,
        io_lock=threading.RLock(),
        startMarker=b'<',
        endMarker=b'>',
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None),
        close=lambda reason: None,
    )

    threads = [
        threading.Thread(
            target=SerialCommunication.send_bytes,
            args=(fake, bytes([value])))
        for value in range(10)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert port.overlap is False
    assert len(port.frames) == 10
    assert all(frame[:2] == b'<\x01' and frame[-1:] == b'>'
               for frame in port.frames)


def test_close_fails_pending_request_and_publishes_status(monkeypatch):
    statuses = []
    future = Future()
    fake = SimpleNamespace(
        connection_lock=threading.RLock(),
        io_lock=threading.RLock(),
        serial_port=None,
        is_connected=True,
        rx_ready=threading.Event(),
        firmware_alive=threading.Event(),
        firmware_identity='test-build',
        pending_lock=threading.Lock(),
        pending={bytes(16): {'future': future, 'deadline': 100.0}},
        _last_transport_status=None,
        transport_status_pub=SimpleNamespace(publish=statuses.append),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    fake.rx_ready.set()
    fake._fail_pending = lambda reason: SerialCommunication._fail_pending(
        fake, reason)
    fake._set_transport_status = (
        lambda status: SerialCommunication._set_transport_status(fake, status))
    monkeypatch.setattr(
        'control_interface_py.device_serial_com.rclpy.ok', lambda: True)

    SerialCommunication.close(fake, reason='test')

    assert future.done()
    assert future.result()['success'] is False
    assert fake.pending == {}
    assert fake.is_connected is False
    assert statuses[-1].data == 'SERIAL_DISCONNECTED:test'


def test_control_frame_requires_known_predicate_and_exact_length():
    errors = []
    sent = []
    fake = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(error=errors.append),
        send_bytes=sent.append,
    )
    msg = DeviceStream()
    msg.predicate = DeviceStream.VEL
    msg.data = [1.0, 2.0]

    SerialCommunication.on_manager_control(fake, msg)

    assert sent == []
    assert errors


def serial_control_fake(now_s=10.0):
    sent = []
    transmitted = []
    warnings = []
    fake = SimpleNamespace(
        control_tx_lock=threading.Lock(),
        _last_control_stamp_ns=0,
        command_max_age_s=0.1,
        command_future_tolerance_s=0.05,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(now_s * 1e9))),
        get_logger=lambda: SimpleNamespace(
            error=lambda message: None, warn=warnings.append),
        send_bytes=lambda payload: sent.append(payload) or True,
        command_tx_pub=SimpleNamespace(publish=transmitted.append),
    )
    fake.transmitted = transmitted
    return fake, sent, warnings


def test_serial_bridge_rejects_expired_and_nonmonotonic_motion():
    fake, sent, warnings = serial_control_fake()
    fresh = DeviceStream()
    fresh.header.stamp = Time(sec=10)
    fresh.predicate = DeviceStream.VEL
    fresh.data = [1.0] * 6
    SerialCommunication.on_manager_control(fake, fresh)

    older = DeviceStream()
    older.header.stamp = Time(sec=9, nanosec=999_000_000)
    older.predicate = DeviceStream.VEL
    older.data = [2.0] * 6
    SerialCommunication.on_manager_control(fake, older)

    assert len(sent) == 1
    assert len(fake.transmitted) == 1
    assert list(fake.transmitted[0].data) == pytest.approx([1.0] * 6)
    assert any('nonmonotonic' in warning for warning in warnings)

    fake._last_control_stamp_ns = 0
    expired = DeviceStream()
    expired.header.stamp = Time(sec=9)
    expired.predicate = DeviceStream.VEL
    expired.data = [3.0] * 6
    SerialCommunication.on_manager_control(fake, expired)
    assert len(sent) == 1
    assert any('expired' in warning for warning in warnings)


def test_serial_bridge_allows_stale_zero_to_preempt_motion():
    fake, sent, _ = serial_control_fake()
    fake._last_control_stamp_ns = 10_000_000_000
    stop = DeviceStream()
    stop.header.stamp = Time(sec=9)
    stop.predicate = DeviceStream.VEL
    stop.data = [0.0] * 6

    SerialCommunication.on_manager_control(fake, stop)

    assert len(sent) == 1
    assert sent[0][0] == DeviceStream.VEL
    assert struct.unpack('<6f', sent[0][1:]) == pytest.approx([0.0] * 6)


def test_connection_request_is_idempotent_when_transport_is_ready():
    connect_calls = []
    sent = []
    ready = threading.Event()
    ready.set()
    fake = SimpleNamespace(
        configured_port='/dev/ttyACM0',
        serial_port=SimpleNamespace(
            is_open=True, port='/dev/ttyACM0'),
        rx_ready=ready,
        firmware_alive=threading.Event(),
        serial_settle_s=0.0,
        firmware_status_wait_s=0.0,
        handshake_wait_s=0.01,
        connect=lambda port: connect_calls.append(port) or True,
        send_bytes=sent.append,
        close=lambda reason: pytest.fail('ready connect must not close'),
    )
    request = DeviceCmd.Request()
    request.predicate = ManagerEvent.CONNECTION
    request.cmd = '/dev/ttyACM0'
    response = DeviceCmd.Response()

    SerialCommunication.handle_device_command(fake, request, response)

    assert response.success is True
    assert response.response == 'Serial ready: /dev/ttyACM0'
    assert connect_calls == ['/dev/ttyACM0']
    assert sent == []


def test_connection_without_rx_stays_open_but_not_ready():
    statuses = []
    sent = []
    ready = threading.Event()
    fake = SimpleNamespace(
        configured_port='/dev/ttyACM0',
        serial_port=None,
        rx_ready=ready,
        firmware_alive=threading.Event(),
        serial_settle_s=0.0,
        firmware_status_wait_s=0.0,
        handshake_wait_s=0.001,
        connect=lambda port: True,
        send_bytes=lambda payload: sent.append(payload) or True,
        _set_transport_status=statuses.append,
        close=lambda reason: pytest.fail(
            'missing telemetry must not close an intentionally opened port'),
    )
    request = DeviceCmd.Request()
    request.predicate = ManagerEvent.CONNECTION
    request.cmd = '/dev/ttyACM0'
    response = DeviceCmd.Response()

    SerialCommunication.handle_device_command(fake, request, response)

    assert response.success is True
    assert response.response.startswith('Serial open; awaiting valid device frame')
    assert sent == [bytes([ManagerEvent.CONNECTION])]
    assert statuses == ['SERIAL_AWAITING_RX:/dev/ttyACM0']
    assert ready.is_set() is False


def test_autonomous_fault_event_does_not_claim_bidirectional_transport():
    statuses = []
    ready = threading.Event()
    fake = SimpleNamespace(
        rx_ready=ready,
        last_rx_time=None,
        serial_port=SimpleNamespace(port='/dev/ttyACM0'),
        configured_port='/dev/ttyACM0',
        _set_transport_status=statuses.append,
    )

    SerialCommunication._mark_rx_ready(fake, ManagerEvent.STALL)

    assert ready.is_set() is False
    assert statuses == []


def test_position_stream_proves_bidirectional_transport():
    statuses = []
    ready = threading.Event()
    fake = SimpleNamespace(
        rx_ready=ready,
        last_rx_time=None,
        serial_port=SimpleNamespace(port='/dev/ttyACM0'),
        configured_port='/dev/ttyACM0',
        _set_transport_status=statuses.append,
    )

    SerialCommunication._mark_rx_ready(fake, DeviceStream.POS)

    assert ready.is_set() is True
    assert statuses == ['SERIAL_READY:/dev/ttyACM0']


def test_connection_disconnect_is_explicit_not_a_toggle():
    closed = []
    fake = SimpleNamespace(
        close=lambda reason: closed.append(reason),
    )
    request = DeviceCmd.Request()
    request.predicate = ManagerEvent.CONNECTION
    request.cmd = 'disconnect'
    response = DeviceCmd.Response()

    SerialCommunication.handle_device_command(fake, request, response)

    assert response.success is True
    assert closed == ['service_request']


def test_shutdown_stop_is_direct_and_does_not_expect_an_ack():
    sent = []
    fake = SimpleNamespace(
        serial_port=SimpleNamespace(is_open=True),
        send_bytes=lambda payload: sent.append(payload) or True)

    assert SerialCommunication._send_shutdown_stop(fake) is True
    assert sent == [bytes([ManagerEvent.STOP_MOTOR])]
