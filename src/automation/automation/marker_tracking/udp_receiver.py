"""Receive Windows marker-only feedback and publish validated ROS messages."""

from __future__ import annotations

from dataclasses import dataclass
import json
import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import ChannelFloat32, PointCloud


SCHEMA = "catheter_marker_feedback"
PROTOCOL_VERSION = 1
MARKER_COUNT = 4
MAXIMUM_DATAGRAM_BYTES = 4096


@dataclass(frozen=True)
class MarkerPacket:
    session_id: str
    sequence: int
    image_timestamp_ns: int
    send_timestamp_ns: int
    frame_id: str
    points_m: np.ndarray
    confidence: np.ndarray
    reprojection_error_px: np.ndarray
    source_rig_count: np.ndarray
    rig_ids: tuple[str, ...]


def decode_marker_packet(
        payload: bytes,
        now_ns: int,
        expected_frame_id: str,
        maximum_packet_age_s: float,
        maximum_future_skew_s: float,
        maximum_abs_position_m: float,
        maximum_reprojection_error_px: float,
) -> MarkerPacket:
    """Validate and decode an untrusted UDP marker datagram."""
    if not payload or len(payload) > MAXIMUM_DATAGRAM_BYTES:
        raise ValueError(f"invalid datagram size {len(payload)}")
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("datagram is not valid UTF-8 JSON") from exc
    if not isinstance(document, dict):
        raise ValueError("packet root must be an object")
    if document.get("schema") != SCHEMA:
        raise ValueError("unexpected schema")
    if document.get("version") != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol version")
    session_id = document.get("session_id")
    if (not isinstance(session_id, str)
            or not session_id or len(session_id) > 128):
        raise ValueError("invalid session_id")
    sequence = document.get("sequence")
    if (not isinstance(sequence, int)
            or isinstance(sequence, bool) or sequence < 1):
        raise ValueError("invalid sequence")
    image_timestamp_ns = document.get("image_timestamp_ns")
    send_timestamp_ns = document.get("send_timestamp_ns")
    for name, value in (
            ("image_timestamp_ns", image_timestamp_ns),
            ("send_timestamp_ns", send_timestamp_ns)):
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise ValueError(f"invalid {name}")
    frame_id = document.get("frame_id")
    if frame_id != expected_frame_id:
        raise ValueError(
            f"frame mismatch: received={frame_id!r}, "
            f"expected={expected_frame_id!r}")
    if document.get("marker_ids") != list(range(MARKER_COUNT)):
        raise ValueError("marker_ids must be exactly [0,1,2,3]")

    points = np.asarray(document.get("points_m"), dtype=np.float64)
    confidence = np.asarray(document.get("confidence"), dtype=np.float64)
    reprojection = np.asarray(
        document.get("reprojection_error_px"), dtype=np.float64)
    source_count = np.asarray(
        document.get("source_rig_count"), dtype=np.float64)
    if points.shape != (MARKER_COUNT, 3):
        raise ValueError("points_m must have shape (4,3)")
    for name, value in (
            ("points_m", points),
            ("confidence", confidence),
            ("reprojection_error_px", reprojection),
            ("source_rig_count", source_count)):
        expected_shape = (
            (MARKER_COUNT, 3) if name == "points_m"
            else (MARKER_COUNT,))
        if value.shape != expected_shape or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} has invalid shape or nonfinite values")
    if np.any(np.abs(points) > float(maximum_abs_position_m)):
        raise ValueError("marker position exceeds configured absolute bound")
    if np.any((confidence < 0.0) | (confidence > 1.0)):
        raise ValueError("confidence lies outside [0,1]")
    if np.any(
            (reprojection < 0.0)
            | (reprojection > float(maximum_reprojection_error_px))):
        raise ValueError("reprojection error exceeds receiver bound")
    if np.any((source_count < 1.0) | (source_count > 16.0)):
        raise ValueError("invalid source_rig_count")
    rig_ids = document.get("rig_ids")
    if (not isinstance(rig_ids, list) or not rig_ids
            or not all(isinstance(value, str) and value for value in rig_ids)):
        raise ValueError("invalid rig_ids")

    age_ns = int(now_ns) - int(image_timestamp_ns)
    if age_ns > int(float(maximum_packet_age_s) * 1e9):
        raise ValueError(f"stale image timestamp: age={age_ns * 1e-9:.3f}s")
    if age_ns < -int(float(maximum_future_skew_s) * 1e9):
        raise ValueError(
            f"image timestamp is in the future: {-age_ns * 1e-9:.3f}s")
    send_age_ns = int(now_ns) - int(send_timestamp_ns)
    if send_age_ns > int(float(maximum_packet_age_s) * 1e9):
        raise ValueError(
            f"stale send timestamp: age={send_age_ns * 1e-9:.3f}s")
    if send_age_ns < -int(float(maximum_future_skew_s) * 1e9):
        raise ValueError(
            f"send timestamp is in the future: {-send_age_ns * 1e-9:.3f}s")
    return MarkerPacket(
        session_id=session_id,
        sequence=sequence,
        image_timestamp_ns=image_timestamp_ns,
        send_timestamp_ns=send_timestamp_ns,
        frame_id=frame_id,
        points_m=points,
        confidence=confidence,
        reprojection_error_px=reprojection,
        source_rig_count=source_count,
        rig_ids=tuple(rig_ids),
    )


def marker_packet_point_cloud(packet: MarkerPacket) -> PointCloud:
    message = PointCloud()
    message.header.frame_id = packet.frame_id
    message.header.stamp.sec = int(
        packet.image_timestamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(
        packet.image_timestamp_ns % 1_000_000_000)
    message.points = [
        Point32(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        for point in packet.points_m
    ]
    message.channels = [
        ChannelFloat32(
            name="marker_id", values=[0.0, 1.0, 2.0, 3.0]),
        ChannelFloat32(
            name="confidence",
            values=[float(value) for value in packet.confidence]),
        ChannelFloat32(
            name="reprojection_error_px",
            values=[float(value)
                    for value in packet.reprojection_error_px]),
        ChannelFloat32(
            name="source_rig_count",
            values=[float(value) for value in packet.source_rig_count]),
    ]
    return message


class MarkerUdpReceiverNode(Node):
    """Publish only fresh, monotonic, complete marker UDP packets."""

    def __init__(self):
        super().__init__("marker_udp_receiver")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("udp_port", 50050)
        self.declare_parameter("allowed_sender", "")
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("marker_topic", "/shape_tracking/markers")
        self.declare_parameter(
            "diagnostics_topic", "/shape_tracking/marker_status")
        self.declare_parameter("poll_rate_hz", 200.0)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("maximum_packet_age_s", 0.5)
        self.declare_parameter("maximum_future_skew_s", 0.1)
        self.declare_parameter("maximum_abs_position_m", 2.0)
        self.declare_parameter("maximum_reprojection_error_px", 5.0)

        self.bind_address = str(self.get_parameter("bind_address").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.allowed_sender = str(self.get_parameter("allowed_sender").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.feedback_timeout_s = float(
            self.get_parameter("feedback_timeout_s").value)
        self.maximum_packet_age_s = float(
            self.get_parameter("maximum_packet_age_s").value)
        self.maximum_future_skew_s = float(
            self.get_parameter("maximum_future_skew_s").value)
        self.maximum_abs_position_m = float(
            self.get_parameter("maximum_abs_position_m").value)
        self.maximum_reprojection_error_px = float(
            self.get_parameter("maximum_reprojection_error_px").value)
        poll_rate_hz = float(self.get_parameter("poll_rate_hz").value)
        if not 1 <= self.udp_port <= 65535:
            raise ValueError("udp_port must be between 1 and 65535")
        if poll_rate_hz <= 0:
            raise ValueError("poll_rate_hz must be positive")

        self.marker_publisher = self.create_publisher(
            PointCloud, str(self.get_parameter("marker_topic").value),
            qos_profile_sensor_data)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value), 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.socket.setblocking(False)
        self.socket.bind((self.bind_address, self.udp_port))
        self.current_session_id = None
        self.last_sequence = 0
        self.last_send_timestamp_ns = 0
        self.last_valid_monotonic = None
        self.last_status_monotonic = 0.0
        self.accepted_packets = 0
        self.rejected_packets = 0
        self.network_drops = 0
        self.superseded_packets = 0
        self.timer = self.create_timer(1.0 / poll_rate_hz, self._poll)
        self.watchdog_timer = self.create_timer(0.25, self._watchdog)
        self.get_logger().info(
            f"marker UDP receiver listening on "
            f"{self.bind_address}:{self.udp_port}; "
            f"allowed_sender={self.allowed_sender or 'any'}")
        self._publish_diagnostic(DiagnosticStatus.OK, "LISTENING", {})

    def _poll(self):
        candidates = []
        rejection = None
        now_ns = time.time_ns()
        for _ in range(64):
            try:
                payload, source = self.socket.recvfrom(
                    MAXIMUM_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                break
            if self.allowed_sender and source[0] != self.allowed_sender:
                rejection = (
                    "SENDER_REJECTED",
                    f"received={source[0]},expected={self.allowed_sender}")
                self.rejected_packets += 1
                continue
            try:
                packet = decode_marker_packet(
                    payload, now_ns, self.frame_id,
                    self.maximum_packet_age_s,
                    self.maximum_future_skew_s,
                    self.maximum_abs_position_m,
                    self.maximum_reprojection_error_px)
            except (TypeError, ValueError) as exc:
                rejection = ("PACKET_REJECTED", str(exc))
                self.rejected_packets += 1
                continue
            candidates.append(packet)
        if not candidates:
            if rejection is not None:
                self._publish_diagnostic(
                    DiagnosticStatus.WARN, rejection[0],
                    {"detail": rejection[1]})
            return

        newest = max(
            candidates,
            key=lambda packet: (
                packet.send_timestamp_ns, packet.sequence))
        self.superseded_packets += len(candidates) - 1
        if self.current_session_id == newest.session_id:
            if newest.sequence <= self.last_sequence:
                self.rejected_packets += 1
                self._publish_diagnostic(
                    DiagnosticStatus.WARN, "OUT_OF_ORDER",
                    {
                        "sequence": str(newest.sequence),
                        "last_sequence": str(self.last_sequence),
                    })
                return
            self.network_drops += max(
                0, newest.sequence - self.last_sequence - 1)
        else:
            if newest.send_timestamp_ns <= self.last_send_timestamp_ns:
                self.rejected_packets += 1
                self._publish_diagnostic(
                    DiagnosticStatus.WARN, "OLD_SESSION_REJECTED",
                    {"session_id": newest.session_id})
                return
            self.current_session_id = newest.session_id
            self.last_sequence = 0
            self.get_logger().info(
                f"accepted marker sender session {newest.session_id}")

        self.last_sequence = newest.sequence
        self.last_send_timestamp_ns = newest.send_timestamp_ns
        self.marker_publisher.publish(marker_packet_point_cloud(newest))
        self.accepted_packets += 1
        self.last_valid_monotonic = time.monotonic()
        transport_ms = (
            time.time_ns() - newest.send_timestamp_ns) * 1e-6
        acquisition_age_ms = (
            time.time_ns() - newest.image_timestamp_ns) * 1e-6
        self._publish_diagnostic(
            DiagnosticStatus.OK, "TRACKING_UDP",
            {
                "session_id": newest.session_id,
                "sequence": str(newest.sequence),
                "rigs": ",".join(newest.rig_ids),
                "transport_latency_ms": f"{transport_ms:.3f}",
                "acquisition_age_ms": f"{acquisition_age_ms:.3f}",
                "accepted_packets": str(self.accepted_packets),
                "rejected_packets": str(self.rejected_packets),
                "network_drops": str(self.network_drops),
                "superseded_packets": str(self.superseded_packets),
            })

    def _watchdog(self):
        now = time.monotonic()
        if (self.last_valid_monotonic is not None
                and now - self.last_valid_monotonic
                <= self.feedback_timeout_s):
            return
        if now - self.last_status_monotonic < self.feedback_timeout_s:
            return
        age = "never" if self.last_valid_monotonic is None else (
            f"{now - self.last_valid_monotonic:.3f}")
        self._publish_diagnostic(
            DiagnosticStatus.ERROR, "FEEDBACK_STALE",
            {"age_s": age})

    def _publish_diagnostic(self, level: int, message: str, values: dict):
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        # ROS distributions expose uint8 constants as either int or one-byte
        # values; assigning the constant directly matches that distribution's
        # generated field setter.
        status.level = level
        status.name = "automation/four_ring_markers_udp"
        status.hardware_id = (
            f"udp:{self.bind_address}:{self.udp_port}")
        status.message = str(message)
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        report.status = [status]
        self.diagnostic_publisher.publish(report)
        self.last_status_monotonic = time.monotonic()

    def close(self):
        self.socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MarkerUdpReceiverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
