#!/usr/bin/env python3
import struct
import math
import time
import threading
import uuid
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import String

import serial

from control_interface.msg import DeviceStream, DeviceEvent, ManagerEvent
from control_interface.srv import DeviceCmd

stream_prefix = [DeviceStream.POS, DeviceStream.VEL, DeviceStream.ENC]
event_prefix = [
    ManagerEvent.LIMIT, ManagerEvent.STALL, ManagerEvent.POSITION_STATUS]
response_prefix = [ManagerEvent.CONNECTION,
                    ManagerEvent.MODE, 
                    ManagerEvent.DEBUG,
                    ord('H'),  # DRIVER_DIAGNOSTIC; keeps mixed-build overlays safe
                    ManagerEvent.START_MOTOR,
                    ManagerEvent.STOP_MOTOR,
                    ManagerEvent.RESET_FAULT,
                    ManagerEvent.FAULT_STATUS,
                    ManagerEvent.SET_ZERO,
                    ManagerEvent.SET_TARGET_VEL]


def encode_device_command(predicate, req_id, cmd='', data=()):
    if len(req_id) != 16:
        raise ValueError('device request id must contain 16 bytes')
    if predicate == ManagerEvent.SET_TARGET_VEL and cmd:
        raise ValueError('SET_TARGET_VEL does not accept a text payload')
    values = [float(value) for value in data]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('device command data must be finite')
    packed = struct.pack('<' + 'f' * len(values), *values)
    payload = bytes([predicate]) + req_id + cmd.encode('utf-8') + packed
    if len(payload) > 255:
        raise ValueError('device command payload exceeds one-byte frame length')
    return payload

# ================================================================================= #
# message structure: [startMarker][len(prefix+payload)][prefix][payload][endMarker] #
# ================================================================================= #

class SerialCommunication(Node):
    def __init__(self):
        super().__init__(
            'device_serial_com')
 
        # ---- Parameters ----
        self.startMarker = b'<'
        self.endMarker = b'>'
        self.timeout_sec = 1.0
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout_s', 0.1)
        self.declare_parameter('connect_retries', 20)
        self.declare_parameter('serial_settle_s', 0.3)
        self.declare_parameter('handshake_wait_s', 1.0)
        self.configured_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout_s = float(
            self.get_parameter('serial_timeout_s').value)
        self.connect_retries = int(
            self.get_parameter('connect_retries').value)
        self.serial_settle_s = float(
            self.get_parameter('serial_settle_s').value)
        self.handshake_wait_s = float(
            self.get_parameter('handshake_wait_s').value)
        if (self.serial_timeout_s <= 0.0 or self.connect_retries <= 0
                or self.serial_settle_s < 0.0 or self.handshake_wait_s < 0.0):
            raise ValueError(
                'serial timeout/retries must be positive and connection '
                'delays must be non-negative')

        # ---- Serial ----
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.io_lock = threading.RLock()
        self.connection_lock = threading.RLock()
        self.rx_ready = threading.Event()
        self.stop_event = threading.Event()
        self.last_rx_time = None
        self.rx_errors = {}
        self._last_transport_status = None

        # Services may wait for firmware responses. Keep them out of the same
        # mutually-exclusive callback group as velocity traffic and watchdogs.
        self.control_group = ReentrantCallbackGroup()
        self.service_group = ReentrantCallbackGroup()

        # ---- ROS interfaces ----
        self.control_sub = self.create_subscription(
            DeviceStream, '/manager/control', self.on_manager_control, 10,
            callback_group=self.control_group,
        )

        self.state_pub = self.create_publisher(
            DeviceStream, '/device/state', 10
        )

        self.event_pub = self.create_publisher(
            DeviceEvent, '/device/event', 10
        )

        transport_qos = QoSProfile(depth=1)
        transport_qos.reliability = ReliabilityPolicy.RELIABLE
        transport_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.transport_status_pub = self.create_publisher(
            String, '/device/transport_status', transport_qos)

        self.cmd_srv = self.create_service(
            DeviceCmd, '/device/command', self.handle_device_command,
            callback_group=self.service_group,
        )

        # ---- Pending service requests ----
        self.pending = {}  # req_id -> dict(future, deadline)
        self.pending_lock = threading.Lock()

        # ---- RX thread ----
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # ---- Timeout watchdog ----
        self.create_timer(0.01, self.check_timeouts)
        self._set_transport_status('SERIAL_DISCONNECTED')


    # ============================================================
    # Serial connection
    # ============================================================

    def connect(self, port=None):
        """Idempotently open a configured serial port."""
        target = port or self.configured_port
        with self.connection_lock:
            with self.io_lock:
                current = self.serial_port
                if current and current.is_open and current.port == target:
                    self.is_connected = True
                    return True
                if current and current.is_open:
                    current.close()
                self.serial_port = None
                self.is_connected = False

            self.rx_ready.clear()
            self.last_rx_time = None
            for attempt in range(1, self.connect_retries + 1):
                try:
                    candidate = serial.Serial(
                        target, self.baudrate,
                        timeout=self.serial_timeout_s)
                    with self.io_lock:
                        self.serial_port = candidate
                        self.is_connected = candidate.is_open
                    self.get_logger().info(f"Serial connected: {target}")
                    self._set_transport_status(f'SERIAL_OPEN:{target}')
                    return self.is_connected
                except serial.SerialException as exc:
                    self.get_logger().warn(
                        f"Attempt {attempt}: cannot open {target}: {exc}")
                    time.sleep(0.1)

            self._set_transport_status(f'SERIAL_OPEN_FAILED:{target}')
            self.get_logger().error(f"Failed to open serial port {target}")
            return False

    def close(self, reason='requested'):
        """Idempotently close the port and fail outstanding transactions."""
        with self.connection_lock:
            with self.io_lock:
                port = self.serial_port
                self.serial_port = None
                self.is_connected = False
                self.rx_ready.clear()
                if port and port.is_open:
                    try:
                        port.close()
                    except (serial.SerialException, OSError):
                        pass
        self._fail_pending(f'Serial disconnected: {reason}')
        self._set_transport_status(f'SERIAL_DISCONNECTED:{reason}')
        if port and rclpy.ok():
            self.get_logger().info(
                f"Serial port {getattr(port, 'name', '')} closed: {reason}")
    
    # ============================================================
    # TX
    # ============================================================

    def send_bytes(self, payload: bytes) -> bool:
        """Atomically write one framed message and report delivery to pyserial."""
        if not payload or len(payload) > 255:
            self.get_logger().error('Refusing invalid serial payload length')
            return False
        frame = (self.startMarker + bytes([len(payload)]) + payload
                 + self.endMarker)
        try:
            with self.io_lock:
                port = self.serial_port
                if not port or not port.is_open:
                    self.get_logger().error("Serial port is not open.")
                    return False
                written = port.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f'partial serial write: {written}/{len(frame)} bytes')
            return True
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"Serial TX error: {exc}")
            self.close(reason=f'TX_ERROR:{exc}')
            return False
        
    def on_manager_control(self, msg: DeviceStream):
        expected = {DeviceStream.VEL: 6, DeviceStream.POS: 12}
        if msg.predicate not in expected or len(msg.data) != expected[msg.predicate]:
            self.get_logger().error(
                f'Invalid manager control frame predicate={msg.predicate} '
                f'length={len(msg.data)}')
            return
        if not all(math.isfinite(value) for value in msg.data):
            self.get_logger().error('Invalid non-finite manager control frame')
            return
        prefix = bytes([msg.predicate])
        data = struct.pack("<" + "f" * len(msg.data),
                           *[float(x) for x in msg.data])
        self.send_bytes(prefix + data)

    # ============================================================
    # RX LOOP
    # ============================================================

    def _set_transport_status(self, status):
        if status == self._last_transport_status:
            return
        self._last_transport_status = status
        if not rclpy.ok():
            return
        msg = String()
        msg.data = status
        self.transport_status_pub.publish(msg)

    def _record_rx_error(self, kind):
        count = self.rx_errors.get(kind, 0) + 1
        self.rx_errors[kind] = count
        if count == 1 or count % 100 == 0:
            self.get_logger().warn(f'Serial RX {kind}; count={count}')

    def _fail_pending(self, reason):
        with self.pending_lock:
            entries = list(self.pending.values())
            self.pending.clear()
        for entry in entries:
            future = entry['future']
            if not future.done():
                future.set_result({'success': False, 'response': reason})

    def _mark_rx_ready(self, prefix):
        # Fault/limit events can be emitted autonomously before the firmware has
        # processed CONNECT. They prove only Teensy -> host communication. POS
        # or ENC telemetry is gated by firmware pc_connected and therefore
        # proves that the host -> Teensy CONNECT frame was processed as well.
        if prefix not in stream_prefix:
            return
        self.last_rx_time = time.monotonic()
        if not self.rx_ready.is_set():
            self.rx_ready.set()
            port = self.serial_port
            name = getattr(port, 'port', self.configured_port)
            self._set_transport_status(f'SERIAL_READY:{name}')

    def rx_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            port = self.serial_port
            if not port or not port.is_open:
                time.sleep(0.1)
                continue

            try:
                # --- 1. Wait for start marker ---
                b = port.read(1)
                if not b or b != self.startMarker:
                    continue

                # --- 2. Read payload length ---
                length_bytes = port.read(1)
                if len(length_bytes) != 1:
                    self._record_rx_error('MISSING_LENGTH')
                    continue

                length = length_bytes[0]
                if length == 0:
                    self._record_rx_error('ZERO_LENGTH')
                    continue

                # --- 3. Read payload (prefix + data) ---
                payload = port.read(length)
                if len(payload) != length:
                    self._record_rx_error('TRUNCATED_PAYLOAD')
                    continue

                # --- 4. Validate end marker ---
                end = port.read(1)
                if end != self.endMarker:
                    self._record_rx_error('BAD_END_MARKER')
                    continue

                # --- 5. Dispatch ---
                prefix = payload[0] # int 0–255

                if prefix in stream_prefix:
                    self.handle_device_stream(prefix, payload[1:])
                elif prefix in event_prefix:
                    self.handle_device_event(prefix, payload[1:])
                elif prefix in response_prefix:
                    self.handle_device_response(prefix, payload[1:])
                else:
                    self._record_rx_error(f'UNKNOWN_PREFIX_{prefix}')
                    continue
                self._mark_rx_ready(prefix)

            except (struct.error, ValueError, IndexError) as exc:
                self._record_rx_error(f'MALFORMED_FRAME:{type(exc).__name__}')
            except (serial.SerialException, OSError, TypeError) as exc:
                if port is self.serial_port:
                    self.close(reason=f'RX_ERROR:{exc}')
                time.sleep(0.1)
            except Exception:
                if not rclpy.ok() or self.stop_event.is_set():
                    break
                raise
    
    # ============================================================
    # Handlers
    # ============================================================

    def handle_device_stream(self, prefix: int, body: bytes):
        # ENC = raw encoder counts (int32); POS/VEL = physical values (float32).
        if len(body) != 24:
            raise ValueError(
                f'stream predicate={prefix} requires 24 bytes, got {len(body)}')
        fmt = "<6i" if prefix == DeviceStream.ENC else "<6f"
        values = struct.unpack(fmt, body)

        msg = DeviceStream()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.predicate = prefix
        msg.data = [float(x) for x in values]  # int32 -> float64 is exact
        self.state_pub.publish(msg)

    def handle_device_event(self, prefix: int, body: bytes):
        if len(body) < 6:
            raise ValueError(
                f'event predicate={prefix} requires at least 6 state bytes')
        msg = DeviceEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.predicate = prefix
        # Legacy LIMIT/STALL payloads contain only the six axis-state bytes.
        # Motion-monitor protocol v1 appends structured diagnostics.
        msg.state = list(body[:6])
        if prefix == ManagerEvent.POSITION_STATUS and len(body) == 27:
            version, status, mask, *errors = struct.unpack('<BBB6f', body)
            names = {
                ManagerEvent.POSITION_COMPLETE: 'POSITION_COMPLETE',
                ManagerEvent.POSITION_TIMED_OUT: 'POSITION_TIMED_OUT',
                ManagerEvent.POSITION_REJECTED: 'POSITION_REJECTED',
            }
            msg.text = names.get(status, 'POSITION_UNKNOWN')
            msg.data = [float(version), float(status), float(mask), *errors]
            if status == ManagerEvent.POSITION_COMPLETE:
                self.get_logger().info(
                    f'{msg.text} mask=0x{mask:02X} errors={errors}')
            else:
                self.get_logger().warn(
                    f'{msg.text} mask=0x{mask:02X} errors={errors}')
        elif (prefix == ManagerEvent.STALL and len(body) >= 29
                and body[6] in (1, 2, 3)):
            values = struct.unpack_from("<BBBBBHfffHH", body, 6)
            (version, transition, fault, axis, coupled_axis, sequence,
             commanded_velocity, measured_velocity, displacement,
             target_rpm, window_ms) = values
            transition_names = {
                1: "MOTION_SUSPECTED",
                2: "MOTION_CONFIRMED",
                3: "MOTION_RECOVERED",
                4: "MOTION_RESET",
                5: "MOTION_UNOBSERVABLE",
                6: "MOTION_STATUS",
            }
            fault_names = {
                0: "NONE",
                1: "STALL",
                2: "OVERSPEED",
                3: "WRONG_DIRECTION",
                4: "FEEDBACK_FAULT",
                5: "SPEED_UNOBSERVABLE",
                6: "DRIVER_COMMUNICATION",
                7: "ENCODER_INTEGRITY",
            }
            msg.text = (
                f"{transition_names.get(transition, 'MOTION_UNKNOWN')}:"
                f"{fault_names.get(fault, 'UNKNOWN')}"
            )
            msg.data = [
                float(version), float(transition), float(fault), float(axis),
                float(-1 if coupled_axis == 255 else coupled_axis),
                float(sequence), float(commanded_velocity),
                float(measured_velocity), float(displacement),
                float(target_rpm), float(window_ms),
            ]
            if version >= 2 and len(body) >= 30:
                msg.data.append(float(body[29]))
            if version >= 3 and len(body) >= 33:
                ack_failure = body[30]
                ack_attempts = body[31]
                response_length = min(body[32], 8, len(body) - 33)
                msg.data.extend([
                    float(ack_failure), float(ack_attempts),
                    float(response_length),
                    *[float(value) for value in
                      body[33:33 + response_length]],
                ])
            self.get_logger().warn(f"{msg.text} data={list(msg.data)}")
        else:
            self.get_logger().warn(
                (bytes([prefix]) + body[:6]).decode("ascii", errors="replace")
            )
        self.event_pub.publish(msg)

    def handle_device_response(self, prefix: int, body: bytes):
        """
        Expected: body = req_id(16 bytes) + response_payload
        """
        if len(body) < 16:
            self.get_logger().warn("Malformed response")
            return

        req_id = body[:16]
        payload = body[16:]

        with self.pending_lock:
            entry = self.pending.pop(req_id, None)

        if not entry:
            self.get_logger().warn(f"Unmatched response id={req_id.hex()}")
            return

        decoded = payload.decode("utf-8", errors="ignore")
        if not entry["future"].done():
            entry["future"].set_result({
                "success": not decoded.startswith("ERR"),
                "response": decoded
            })

    # ============================================================
    # Service
    # ============================================================

    def handle_device_command(self, request, response):

        if request.predicate == ManagerEvent.CONNECTION:
            command = request.cmd.strip()
            if command.lower() in ('disconnect', 'close', 'off'):
                self.close(reason='service_request')
                response.success = True
                response.response = "Serial disconnected"
                return response

            # A Windows COM label from Slicer is descriptive only; the actual
            # serial endpoint belongs to the WSL node parameter.
            target = command if command.startswith('/dev/') \
                else self.configured_port
            was_open = bool(
                self.serial_port and self.serial_port.is_open
                and self.serial_port.port == target)
            if not self.connect(port=target):
                response.success = False
                response.response = f"Failed to open {target}"
                return response
            if not self.rx_ready.is_set():
                # Opening the Teensy's USB serial endpoint can reset it. Preserve
                # the proven settle interval before sending the one-byte stream
                # enable command, otherwise that command can be lost at startup.
                if not was_open and self.serial_settle_s:
                    time.sleep(self.serial_settle_s)
                if not self.send_bytes(bytes([request.predicate])):
                    response.success = False
                    response.response = 'Serial opened but handshake TX failed'
                    return response
                if not self.rx_ready.wait(timeout=self.handshake_wait_s):
                    # Driver power may intentionally be off at this stage. Keep
                    # the port open, but do not declare it ready: the manager's
                    # transport gate continues to inhibit all motion until a
                    # complete valid firmware frame arrives.
                    self._set_transport_status(
                        f'SERIAL_AWAITING_RX:{target}')
                    response.success = True
                    response.response = (
                        f'Serial open; awaiting valid device frame: {target}')
                    return response
            response.success = True
            response.response = f"Serial ready: {target}"
            return response

        if not self.is_connected or not self.rx_ready.is_set():
            response.success = False
            response.response = 'Serial transport is not ready'
            return response

        req_id = uuid.uuid4().bytes  # 16 bytes
        try:
            payload = encode_device_command(
                request.predicate, req_id, request.cmd, request.data)
        except (TypeError, ValueError, struct.error) as exc:
            response.success = False
            response.response = str(exc)
            return response

        # Register the pending request BEFORE sending, so a fast firmware ack
        # can't arrive before we're ready to match it.
        future = Future()
        with self.pending_lock:
            self.pending[req_id] = {
                "future": future,
                "deadline": time.monotonic() + self.timeout_sec
            }
        if not self.send_bytes(payload):
            with self.pending_lock:
                self.pending.pop(req_id, None)
            response.success = False
            response.response = 'Serial TX failed'
            return response

        # Poll for completion instead of rclpy.spin_until_future_complete(). The
        # future is resolved by handle_device_response() on the rx thread. Nesting
        # a spin inside this (MultiThreadedExecutor) callback DEADLOCKS the whole
        # executor when a command gets no firmware ack (e.g. START_MOTOR/STOP_MOTOR
        # have no firmware handler) — it wedges on_manager_control (velocity TX)
        # and check_timeouts forever. A plain timed poll can't deadlock.
        deadline = time.monotonic() + self.timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)

        with self.pending_lock:
            self.pending.pop(req_id, None)

        if future.done():
            result = future.result()
            response.success = result["success"]
            response.response = result["response"]
        else:
            response.success = False
            response.response = "Timeout"

        return response

    # ============================================================
    # Timeout watchdog
    # ============================================================

    def check_timeouts(self):
        now = time.monotonic()
        expired = []

        with self.pending_lock:
            for req_id, entry in list(self.pending.items()):
                if now > entry["deadline"]:
                    expired.append(self.pending.pop(req_id))

        for entry in expired:
            if not entry["future"].done():
                entry["future"].set_result({
                    "success": False,
                    "response": "Timeout"
                })

    def _send_shutdown_stop(self):
        if not self.serial_port or not self.serial_port.is_open:
            return False
        # A one-byte command intentionally has no request UUID, so firmware
        # executes STOP without returning an acknowledgement that could become
        # an unmatched response while this node is tearing down.
        return self.send_bytes(bytes([ManagerEvent.STOP_MOTOR]))

    def destroy_node(self):
        # This node owns the physical USB link, so it is the final shutdown
        # barrier if the manager service path is already disappearing.
        try:
            if self._send_shutdown_stop():
                message = 'shutdown STOP frame sent directly to firmware'
                if rclpy.ok():
                    self.get_logger().info(message)
                else:
                    print(message, flush=True)
                time.sleep(0.05)
        except Exception as exc:
            self.get_logger().error(
                f'failed to send shutdown STOP frame: {exc}')
        self.stop_event.set()
        self.close(reason='node_shutdown')
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=max(0.2, 2 * self.serial_timeout_s))
        return super().destroy_node()

def main():
    rclpy.init()

    node = SerialCommunication()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
