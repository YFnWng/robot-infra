"""Write one timestamped rosbag for each Slicer keyboard-control session."""

from collections import deque
from datetime import datetime
from pathlib import Path
import re
import threading
import time

from control_interface.msg import (
    ControlStream, DeviceEvent, DeviceStream, ManagerEvent)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import String

try:
    from ros2_igtl_bridge.msg import Transform
except ImportError:  # Optional without the tracking bridge overlay.
    Transform = None


TOPIC_TYPES = {
    '/teleop/control': (
        ControlStream, 'control_interface/msg/ControlStream'),
    '/teleop/event': (
        ManagerEvent, 'control_interface/msg/ManagerEvent'),
    '/manager/control': (
        DeviceStream, 'control_interface/msg/DeviceStream'),
    '/device/state': (
        DeviceStream, 'control_interface/msg/DeviceStream'),
    '/device/event': (
        DeviceEvent, 'control_interface/msg/DeviceEvent'),
    '/manager/event': (
        ManagerEvent, 'control_interface/msg/ManagerEvent'),
    '/manager/safety_status': (
        ManagerEvent, 'control_interface/msg/ManagerEvent'),
    '/IGTL_TRANSFORM_IN': (
        Transform, 'ros2_igtl_bridge/msg/Transform'),
}
DEFAULT_TOPICS = list(TOPIC_TYPES)
COIL_TRANSFORM_PATTERN = re.compile(r"^RX[1-9][0-9]*(_filtered)?$")


def allocate_session_dir(output_root, prefix, now=None):
    """Create a collision-resistant timestamped session directory."""
    timestamp = (now or datetime.now()).strftime('%Y%m%d_%H%M%S')
    root = Path(output_root).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    candidate = root / f'{prefix}_{timestamp}'
    suffix = 1
    while candidate.exists():
        candidate = root / f'{prefix}_{timestamp}_{suffix:02d}'
        suffix += 1
    candidate.mkdir()
    return candidate


class TeleopBagRecorder(Node):
    """Write subscribed messages only while keyboard control is enabled."""

    def __init__(self):
        super().__init__('teleop_bag_recorder')
        self.declare_parameter(
            'output_root', '/mnt/d/robot-dev/catheter_sessions')
        self.declare_parameter('session_prefix', 'teleop')
        self.declare_parameter('topics', DEFAULT_TOPICS)
        self.declare_parameter('heartbeat_timeout_s', 1.0)
        self.declare_parameter('max_queue_messages', 20000)
        self.declare_parameter('backlog_warn_messages', 2000)

        self.output_root = self.get_parameter('output_root').value
        self.session_prefix = self.get_parameter('session_prefix').value
        self.topics = list(self.get_parameter('topics').value)
        self.heartbeat_timeout_s = float(
            self.get_parameter('heartbeat_timeout_s').value)
        self.max_queue_messages = int(
            self.get_parameter('max_queue_messages').value)
        self.backlog_warn_messages = int(
            self.get_parameter('backlog_warn_messages').value)
        if not self.output_root:
            raise ValueError('output_root must not be empty')
        if not self.session_prefix:
            raise ValueError('session_prefix must not be empty')
        unknown_topics = set(self.topics) - set(TOPIC_TYPES)
        if not self.topics or unknown_topics:
            raise ValueError(
                f'unsupported recording topics: {sorted(unknown_topics)}')
        if self.heartbeat_timeout_s <= 0.0:
            raise ValueError('heartbeat_timeout_s must be positive')
        if self.max_queue_messages <= 0:
            raise ValueError('max_queue_messages must be positive')
        if self.backlog_warn_messages <= 0:
            raise ValueError('backlog_warn_messages must be positive')
        self.active_topics = [
            topic for topic in self.topics
            if TOPIC_TYPES[topic][0] is not None
        ]
        unavailable_topics = set(self.topics) - set(self.active_topics)
        if unavailable_topics:
            self.get_logger().warn(
                'optional message packages unavailable; not recording: '
                f'{sorted(unavailable_topics)}')

        self.writer = None
        self.session_dir = None
        self.last_slicer_heartbeat = None
        self._write_queue = deque()
        self._write_condition = threading.Condition()
        self._writer_thread = None
        self._stop_writer = False
        self._writer_error = None
        self._dropped_state_messages = 0
        self._last_backlog_warning = 0.0

        record_qos = QoSProfile(depth=1000)
        record_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos = QoSProfile(depth=1000)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._record_subscriptions = []
        for topic in self.active_topics:
            message_type = TOPIC_TYPES[topic][0]
            if topic == '/teleop/event':
                callback = self.event_callback
                qos = record_qos
            elif topic == '/teleop/control':
                callback = self.control_callback
                qos = record_qos
            elif topic == '/manager/safety_status':
                callback = self.safety_callback
                qos = safety_qos
            elif topic == '/IGTL_TRANSFORM_IN':
                callback = self.coil_transform_callback
                qos = record_qos
            else:
                callback = (
                    lambda msg, topic_name=topic:
                    self.record_message(topic_name, msg))
                qos = record_qos
            self._record_subscriptions.append(
                self.create_subscription(message_type, topic, callback, qos))

        self.status_pub = self.create_publisher(
            String, '/teleop/recording_status', 10)
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info(
            f'teleop bag recorder ready: output_root={self.output_root}')

    @property
    def is_recording(self):
        return self.writer is not None

    @staticmethod
    def _slicer_mode(msg):
        if (msg.header.frame_id != 'slicer'
                or msg.predicate != ManagerEvent.MODE):
            return None
        return msg.text

    def event_callback(self, msg):
        mode = self._slicer_mode(msg)
        if mode == chr(ManagerEvent.JOINT_VEL):
            self.start_recording()
            self.record_message('/teleop/event', msg)
        elif mode == chr(ManagerEvent.NONE):
            self.record_message('/teleop/event', msg)
            self.stop_recording('keyboard disabled')
        else:
            self.record_message('/teleop/event', msg)

    def control_callback(self, msg):
        if msg.header.frame_id == 'slicer' and self.is_recording:
            self.last_slicer_heartbeat = time.monotonic()
        self.record_message('/teleop/control', msg)

    def safety_callback(self, msg):
        self.record_message('/manager/safety_status', msg)
        if self.is_recording and msg.text != 'MANAGER_READY':
            self.stop_recording(f'manager inhibited: {msg.text}')

    def coil_transform_callback(self, msg):
        if COIL_TRANSFORM_PATTERN.fullmatch(msg.name):
            self.record_message('/IGTL_TRANSFORM_IN', msg)

    def _publish_status(self, state, detail):
        msg = String()
        msg.data = f'{state}:{detail}'
        self.status_pub.publish(msg)

    def _open_writer(self, bag_path):
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''),
        )
        for topic in self.active_topics:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic,
                type=TOPIC_TYPES[topic][1],
                serialization_format='cdr',
            ))
        return writer

    def start_recording(self):
        if self.is_recording:
            self.get_logger().warn(
                f'keyboard bag already recording: {self.session_dir}')
            return
        try:
            session_dir = allocate_session_dir(
                self.output_root, self.session_prefix)
            writer = self._open_writer(session_dir / 'rosbag')
        except (OSError, RuntimeError, ValueError) as exc:
            self.get_logger().error(f'cannot start keyboard rosbag: {exc}')
            self._publish_status('ERROR', str(exc))
            return

        with self._write_condition:
            self.writer = writer
            self.session_dir = session_dir
            self._write_queue.clear()
            self._stop_writer = False
            self._writer_error = None
            self._dropped_state_messages = 0
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            args=(writer,),
            name='teleop_bag_writer',
            daemon=True,
        )
        self._writer_thread.start()
        self.last_slicer_heartbeat = time.monotonic()
        self.get_logger().info(f'keyboard rosbag started: {session_dir}')
        self._publish_status('RECORDING', str(session_dir))

    def record_message(self, topic, msg):
        if not self.is_recording:
            return
        try:
            serialized = serialize_message(msg)
        except Exception as exc:
            self.get_logger().error(f'cannot serialize {topic}: {exc}')
            self._publish_status('ERROR', f'serialize:{topic}:{exc}')
            return

        now_ns = self.get_clock().now().nanoseconds
        timestamp_ns = self._message_timestamp_ns(msg, now_ns)
        dropped = False
        queue_size = 0
        with self._write_condition:
            if self.writer is None or self._stop_writer:
                return
            if len(self._write_queue) >= self.max_queue_messages:
                if topic == '/device/state':
                    self._dropped_state_messages += 1
                    dropped = True
                else:
                    dropped = self._discard_oldest_device_state_locked()
            if not dropped or topic != '/device/state':
                self._write_queue.append((topic, serialized, timestamp_ns))
                queue_size = len(self._write_queue)
                self._write_condition.notify()

        if dropped and topic == '/device/state':
            self._warn_backlog(
                'teleop bag queue full; dropping incoming device state')
        elif queue_size >= self.backlog_warn_messages:
            self._warn_backlog(
                f'teleop bag writer backlog: {queue_size} messages')

    @staticmethod
    def _message_timestamp_ns(msg, fallback_ns):
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is not None:
            timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if timestamp_ns > 0:
                return timestamp_ns
        return fallback_ns

    def _discard_oldest_device_state_locked(self):
        for index, queued in enumerate(self._write_queue):
            if queued[0] == '/device/state':
                del self._write_queue[index]
                self._dropped_state_messages += 1
                return True
        return False

    def _warn_backlog(self, message):
        now = time.monotonic()
        if now - self._last_backlog_warning >= 5.0:
            self._last_backlog_warning = now
            self.get_logger().warn(message)

    def _writer_loop(self, writer):
        while True:
            with self._write_condition:
                self._write_condition.wait_for(
                    lambda: self._write_queue or self._stop_writer)
                if not self._write_queue and self._stop_writer:
                    return
                topic, serialized, timestamp_ns = self._write_queue.popleft()
            try:
                writer.write(topic, serialized, timestamp_ns)
            except Exception as exc:
                self._writer_error = f'{topic}:{exc}'
                self.get_logger().error(
                    f'keyboard rosbag write failed on {topic}: {exc}')
                with self._write_condition:
                    self._write_queue.clear()
                    self._stop_writer = True
                    self._write_condition.notify_all()
                return

    def watchdog_callback(self):
        if self.is_recording and self._writer_error is not None:
            self.stop_recording('write failure')
            return
        if (self.is_recording and self.last_slicer_heartbeat is not None
                and time.monotonic() - self.last_slicer_heartbeat
                > self.heartbeat_timeout_s):
            self.stop_recording('Slicer heartbeat lost')

    def stop_recording(self, reason='shutdown'):
        with self._write_condition:
            if self.writer is None:
                return
            writer = self.writer
            session_dir = self.session_dir
            writer_thread = self._writer_thread
            self.writer = None
            self.session_dir = None
            self._stop_writer = True
            self._write_condition.notify_all()
        self.last_slicer_heartbeat = None
        if writer_thread is not None:
            writer_thread.join()
        self._writer_thread = None
        try:
            writer.close()
        except Exception as exc:
            self.get_logger().error(
                f'keyboard rosbag finalization failed: {exc}')
            self._publish_status('ERROR', f'finalization:{exc}')
            return
        if self._writer_error is not None:
            self._publish_status('ERROR', f'write:{self._writer_error}')
        if self._dropped_state_messages:
            self.get_logger().warn(
                'keyboard rosbag dropped '
                f'{self._dropped_state_messages} /device/state messages')
        self.get_logger().info(
            f'keyboard rosbag stopped ({reason}): {session_dir}')
        self._publish_status('STOPPED', f'{reason}:{session_dir}')

    def destroy_node(self):
        self.stop_recording('recorder node shutdown')
        return super().destroy_node()


def main():
    rclpy.init()
    node = TeleopBagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
