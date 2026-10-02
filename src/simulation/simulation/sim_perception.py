"""Independent exact-model marker process driven by simulated ENC feedback."""
from __future__ import annotations

import json
import threading

from control_interface.msg import DeviceStream
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32, PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import ChannelFloat32, PointCloud
from std_srvs.srv import Trigger
import torch

from .sim_perturbations import (
    JacobianConfig, JacobianPerturbation, MarkerSensorConfig,
    MarkerSensorModel, set_physical_jacobian)
from catheter_control.safety.hardware_contract import ENCODER_RADIANS_PER_COUNT


# First-posterior pose from the selected v171 identification.  This is used
# only as configurable simulation truth; it is never passed to the controller
# or used as a live-hardware estimator prior.
DEFAULT_TRUTH_INTERFACE_POSE = [
    -0.852797151, -0.473069161, 0.221229315, -0.000312328,
    0.495341808, -0.866912186, 0.055673875, 0.000244854,
    0.165448815, 0.157062665, 0.973631382, 0.009205841,
    0.0, 0.0, 0.0, 1.0,
]


def _stamp_ns(message):
    return (int(message.header.stamp.sec)*1_000_000_000
            + int(message.header.stamp.nanosec))


def _quaternion_from_rotation(rotation):
    """Convert a proper 3x3 rotation matrix to ROS quaternion ordering."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    # Eigenvector extraction is stable at both identity and pi rotations.
    k = np.array([
        [matrix[0, 0]-matrix[1, 1]-matrix[2, 2],
         matrix[1, 0]+matrix[0, 1],
         matrix[2, 0]+matrix[0, 2],
         matrix[1, 2]-matrix[2, 1]],
        [matrix[1, 0]+matrix[0, 1],
         matrix[1, 1]-matrix[0, 0]-matrix[2, 2],
         matrix[2, 1]+matrix[1, 2],
         matrix[2, 0]-matrix[0, 2]],
        [matrix[2, 0]+matrix[0, 2],
         matrix[2, 1]+matrix[1, 2],
         matrix[2, 2]-matrix[0, 0]-matrix[1, 1],
         matrix[0, 1]-matrix[1, 0]],
        [matrix[1, 2]-matrix[2, 1],
         matrix[2, 0]-matrix[0, 2],
         matrix[0, 1]-matrix[1, 0],
         matrix.trace()],
    ], dtype=float)/3.0
    values, vectors = np.linalg.eigh(k)
    quaternion = vectors[:, int(np.argmax(values))]
    # The symmetric K construction above yields the passive/conjugate
    # convention. ROS geometry messages use the active x,y,z,w convention.
    quaternion[:3] *= -1.0
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


class SimulatedPerceptionNode(Node):
    def __init__(self):
        super().__init__("catheter_sim_perception")
        self.declare_parameter("simulation_only", True)
        self.declare_parameter("frame_id", "robot_base")
        self.declare_parameter("model_manifest", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("marker_rate_hz", 30.0)
        self.declare_parameter("torch_intraop_threads", 1)
        self.declare_parameter("torch_interop_threads", 1)
        self.declare_parameter("robustness_seed", 0)
        self.declare_parameter("marker_noise_std_mm", 0.0)
        self.declare_parameter("marker_common_bias_mm", [0.0]*3)
        self.declare_parameter("marker_specific_bias_mm", [0.0]*12)
        self.declare_parameter("marker_latency_ms", 0.0)
        self.declare_parameter("marker_timestamp_jitter_ms", 0.0)
        self.declare_parameter("marker_dropout_probability", 0.0)
        self.declare_parameter("marker_outlier_probability", 0.0)
        self.declare_parameter("marker_outlier_magnitude_mm", 0.0)
        self.declare_parameter(
            "truth_initial_interface_pose", DEFAULT_TRUTH_INTERFACE_POSE)
        self.declare_parameter(
            "plant_jacobian_angular_column_gain", [1.0]*3)
        self.declare_parameter(
            "plant_jacobian_linear_column_gain", [1.0]*3)
        if not bool(self.get_parameter("simulation_only").value):
            raise ValueError("simulation_only must be true")
        torch.set_num_threads(int(
            self.get_parameter("torch_intraop_threads").value))
        torch.set_num_interop_threads(int(
            self.get_parameter("torch_interop_threads").value))
        from cr_meta_lnn.deployment import load_runtime_bundle
        self.runtime_bundle = load_runtime_bundle(
            self.get_parameter("model_manifest").value,
            device=self.get_parameter("device").value,
            options={"adaptation_enabled": False})
        self.runtime = self.runtime_bundle.runtime
        self.jacobian_config = JacobianConfig(
            angular_column_gain=tuple(self.get_parameter(
                "plant_jacobian_angular_column_gain").value),
            linear_column_gain=tuple(self.get_parameter(
                "plant_jacobian_linear_column_gain").value))
        jacobian_perturbation = JacobianPerturbation(self.jacobian_config)
        perturbed = jacobian_perturbation.apply(
            self.runtime.initial_jacobian.jacobian)
        self.runtime.initial_jacobian = set_physical_jacobian(
            self.runtime.initial_jacobian.copy(), perturbed)
        initial_pose = np.asarray(self.get_parameter(
            "truth_initial_interface_pose").value, dtype=float)
        if initial_pose.shape != (16,) or not np.isfinite(initial_pose).all():
            raise ValueError(
                "truth_initial_interface_pose must contain 16 finite values")
        self.truth_initial_interface_pose = initial_pose.reshape(4, 4)
        self.sensor_config = MarkerSensorConfig(
            seed=int(self.get_parameter("robustness_seed").value),
            noise_std_m=1e-3*float(self.get_parameter(
                "marker_noise_std_mm").value),
            common_bias_m=tuple(1e-3*np.asarray(self.get_parameter(
                "marker_common_bias_mm").value, dtype=float)),
            marker_bias_m=tuple(1e-3*np.asarray(self.get_parameter(
                "marker_specific_bias_mm").value, dtype=float)),
            latency_s=1e-3*float(self.get_parameter(
                "marker_latency_ms").value),
            timestamp_jitter_s=1e-3*float(self.get_parameter(
                "marker_timestamp_jitter_ms").value),
            dropout_probability=float(self.get_parameter(
                "marker_dropout_probability").value),
            outlier_probability=float(self.get_parameter(
                "marker_outlier_probability").value),
            outlier_magnitude_m=1e-3*float(self.get_parameter(
                "marker_outlier_magnitude_mm").value))
        self.sensor = MarkerSensorModel(self.sensor_config)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self._lock = threading.Lock()
        self._pending = None
        self._truth_reset_count = 0
        self._interface_transmission_state = None
        self.create_subscription(
            DeviceStream, "/sim/catheter_sim/transmitted_state",
            self._encoder_cb,
            qos_profile_sensor_data)
        self.reset_service = self.create_service(
            Trigger, "/sim/catheter_sim/reset_perception",
            self._reset_cb)
        self.marker_pub = self.create_publisher(
            PointCloud, "/sim/shape_tracking/markers", 10)
        self.truth_marker_pub = self.create_publisher(
            PointCloud, "/sim/catheter_sim/ground_truth_markers", 10)
        self.marker_status_pub = self.create_publisher(
            DiagnosticArray, "/sim/shape_tracking/marker_status", 10)
        self.tip_pub = self.create_publisher(
            PointStamped, "/sim/catheter_sim/ground_truth_tip", 10)
        self.interface_pose_pub = self.create_publisher(
            PoseStamped,
            "/sim/catheter_sim/ground_truth_interface_pose", 10)
        marker_rate = float(self.get_parameter("marker_rate_hz").value)
        if marker_rate <= 0.0:
            raise ValueError("marker_rate_hz must be positive")
        self.timer = self.create_timer(1.0/marker_rate, self._tick)
        self.get_logger().warn(
            "SIMULATION ONLY truth-model perception with configurable "
            "robustness perturbations active under /sim")

    def _reset_cb(self, _request, response):
        """Forget truth recurrence so the next transmitted ENC reinitializes."""
        with self._lock:
            self._pending = None
            self.runtime.reset()
            self._interface_transmission_state = None
            self.sensor = MarkerSensorModel(self.sensor_config)
            self._truth_reset_count += 1
        response.success = True
        response.message = "simulation truth recurrence reset"
        self.get_logger().info(response.message)
        return response

    def _encoder_cb(self, message):
        if (message.predicate != DeviceStream.ENC
                or len(message.data) != 6
                or not all(np.isfinite(message.data))):
            return
        sample = (_stamp_ns(message), np.asarray(message.data, dtype=float))
        if sample[0] <= 0:
            return
        with self._lock:
            if self._pending is None or sample[0] >= self._pending[0]:
                self._pending = sample

    def _tick(self):
        with self._lock:
            sample = self._pending
            self._pending = None
        if sample is None:
            return
        timestamp_ns, counts = sample
        try:
            interface_motor = None
            artifacts = getattr(
                self.runtime, "interface_transmission_artifacts", None)
            if artifacts is not None:
                raw_motor = torch.as_tensor(
                    counts[:3]*ENCODER_RADIANS_PER_COUNT,
                    dtype=self.runtime.dtype, device=self.runtime.device)
                if self._interface_transmission_state is None:
                    self._interface_transmission_state = (
                        artifacts.transmission.initial_state(raw_motor))
                else:
                    self._interface_transmission_state = (
                        artifacts.transmission.advance(
                            self._interface_transmission_state, raw_motor))
                interface_motor = artifacts.transmission.equivalent_shaft_angle(
                    self._interface_transmission_state)
            if self.runtime.state is None:
                self.runtime.initialize(
                    timestamp_ns, counts,
                    interface_pose=self.truth_initial_interface_pose,
                    interface_motor_angle_rad=interface_motor)
            elif timestamp_ns > self.runtime.state.timestamp_ns:
                self.runtime.advance_encoder(
                    timestamp_ns, counts,
                    interface_motor_angle_rad=interface_motor)
            else:
                return
            markers = np.asarray(
                self.runtime.current_markers().detach().cpu(), dtype=float)
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(
                f"simulated perception fault: {type(error).__name__}: {error}")
            return
        stamp = rclpy.time.Time(nanoseconds=timestamp_ns).to_msg()
        truth = self._cloud(stamp, markers)
        self.truth_marker_pub.publish(truth)
        tip = PointStamped()
        tip.header = truth.header
        tip.point.x, tip.point.y, tip.point.z = [
            float(value) for value in markers[-1]]
        self.tip_pub.publish(tip)
        pose_matrix = np.asarray(
            self.runtime.state.interface_pose.detach().cpu(), dtype=float)
        pose = PoseStamped()
        pose.header = truth.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [
            float(value) for value in pose_matrix[:3, 3]]
        quaternion = _quaternion_from_rotation(pose_matrix[:3, :3])
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = [
            float(value) for value in quaternion]
        self.interface_pose_pub.publish(pose)

        pushed = self.sensor.push(timestamp_ns, markers)
        if not pushed:
            self._publish_status(stamp, "SIMULATED_DROPOUT",
                                 DiagnosticStatus.WARN)
        for packet in self.sensor.pop_ready(timestamp_ns):
            observed_stamp = rclpy.time.Time(
                nanoseconds=packet.observation_timestamp_ns).to_msg()
            self.marker_pub.publish(self._cloud(
                observed_stamp, packet.points_m))
            self._publish_status(
                observed_stamp,
                "SIMULATED_OUTLIER" if packet.outlier_injected
                else "TRACKING",
                DiagnosticStatus.WARN if packet.outlier_injected
                else DiagnosticStatus.OK)

    def _cloud(self, stamp, markers):
        cloud = PointCloud()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frame_id
        cloud.points = [Point32(x=float(point[0]), y=float(point[1]),
                                z=float(point[2])) for point in markers]
        cloud.channels = [
            ChannelFloat32(name="marker_id", values=[0.0, 1.0, 2.0, 3.0]),
            ChannelFloat32(name="confidence", values=[1.0]*4),
            ChannelFloat32(name="reprojection_error_px", values=[0.1]*4),
            ChannelFloat32(name="source_rig_count", values=[2.0]*4)]
        return cloud

    def _publish_status(self, stamp, message, level):
        report = DiagnosticArray()
        report.header.stamp = stamp
        report.header.frame_id = self.frame_id
        status = DiagnosticStatus()
        status.level = level
        status.name = "automation/four_ring_markers"
        status.hardware_id = "simulated_v171"
        status.message = message
        status.values = [
            KeyValue(key="source", value="perturbed_exact_model"),
            KeyValue(key="robustness_seed", value=str(
                self.sensor_config.seed)),
            KeyValue(key="marker_noise_std_mm", value=str(
                1e3*self.sensor_config.noise_std_m)),
            KeyValue(key="marker_latency_ms", value=str(
                1e3*self.sensor_config.latency_s)),
            KeyValue(key="marker_timestamp_jitter_ms", value=str(
                1e3*self.sensor_config.timestamp_jitter_s)),
            KeyValue(key="marker_dropout_probability", value=str(
                self.sensor_config.dropout_probability)),
            KeyValue(key="marker_outlier_probability", value=str(
                self.sensor_config.outlier_probability)),
            KeyValue(key="sensor_packets_dropped", value=str(
                self.sensor.dropped_packets)),
            KeyValue(key="sensor_packets_pending", value=str(
                self.sensor.pending_packets)),
            KeyValue(key="plant_jacobian_angular_column_gain", value=(
                json.dumps(list(self.jacobian_config.angular_column_gain)))),
            KeyValue(key="plant_jacobian_linear_column_gain", value=(
                json.dumps(list(self.jacobian_config.linear_column_gain)))),
            KeyValue(key="truth_initial_interface_pose", value=json.dumps(
                self.truth_initial_interface_pose.tolist())),
            KeyValue(key="truth_reset_count", value=str(
                self._truth_reset_count))]
        report.status = [status]
        self.marker_status_pub.publish(report)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
