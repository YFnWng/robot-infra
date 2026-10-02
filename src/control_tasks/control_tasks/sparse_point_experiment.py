"""Run independent sparse circle targets from one repeatable joint home."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time

from control_interface.action import TrackTipTrajectory
from control_interface.msg import ControlStream, DeviceStream, ManagerEvent
from control_interface.srv import GenerateSparseTargets
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud
from std_srvs.srv import Trigger
import yaml

from .target_offset import controller_status, measured_tip
from .trajectory_file import _circle_waypoints


@dataclass(frozen=True)
class SparsePointSpec:
    frame_id: str
    action_name: str
    marker_topic: str
    position_topic: str
    status_topic: str
    safety_topic: str
    control_topic: str
    event_topic: str
    planned_control_topic: str
    manager_control_topic: str
    source_name: str
    home_position: np.ndarray
    home_speed: np.ndarray
    home_tolerance: np.ndarray
    home_timeout_s: float
    home_mode_settle_s: float
    home_settle_s: float
    post_home_settle_s: float
    decoupled_tendon_prehome: bool
    target_timeout_s: float
    target_tolerance_mm: float
    target_settle_s: float
    plant_reset_mode: str
    plant_reset_services: tuple[str, ...]
    plant_reset_timeout_s: float
    plant_reset_settle_s: float
    required_controller_status: dict
    rotation_guard_enabled: bool
    rotation_guard_axis: int
    rotation_guard_tolerance: float
    generator: dict


def _finite_vector(mapping, key, size=6, *, nonnegative=False):
    value = np.asarray(mapping[key], dtype=np.float64)
    if value.shape != (size,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{key} must contain {size} finite values")
    if nonnegative and np.any(value < 0.0):
        raise ValueError(f"{key} must be nonnegative")
    return value


def _positive(mapping, key):
    value = float(mapping[key])
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{key} must be finite and positive")
    return value


def load_sparse_point_experiment(path) -> SparsePointSpec:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("sparse-point YAML root must be a mapping")
    home = document.get("home")
    generator = document.get("generator")
    if not isinstance(home, dict) or not isinstance(generator, dict):
        raise ValueError("home and generator mappings are required")
    strings = {
        key: str(document[key]) for key in (
            "frame_id", "action_name", "marker_topic", "position_topic",
            "status_topic", "safety_topic", "control_topic", "event_topic",
            "source_name")
    }
    strings["planned_control_topic"] = str(document.get(
        "planned_control_topic", "/catheter_mppi/planned_control"))
    strings["manager_control_topic"] = str(document.get(
        "manager_control_topic", "/manager/control"))
    if not all(strings.values()):
        raise ValueError("ROS endpoint and source names must be nonempty")
    position = _finite_vector(home, "joint_position")
    speed = _finite_vector(home, "joint_speed", nonnegative=True)
    tolerance = _finite_vector(home, "joint_tolerance", nonnegative=True)
    if np.any(speed <= 0.0) or np.any(tolerance <= 0.0):
        raise ValueError("home speeds and tolerances must be positive")
    decoupled_prehome = home.get("decoupled_tendon_prehome", False)
    if not isinstance(decoupled_prehome, bool):
        raise ValueError("home.decoupled_tendon_prehome must be boolean")
    generator_type = str(generator.get("type", "")).strip()
    if generator_type == "circle_yz_from_current_tip":
        if bool(generator.get("close_circle", False)):
            raise ValueError(
                "sparse independent targets must not duplicate the first "
                "point")
        count = int(generator.get("waypoint_count", 8))
        if count < 3 or count > 36:
            raise ValueError("sparse waypoint_count must be in [3,36]")
    elif generator_type == "relative_offsets_from_current_tip":
        offsets = np.asarray(generator.get("offsets_mm"), dtype=np.float64)
        if (offsets.ndim != 2 or offsets.shape[1:] != (3,)
                or not 1 <= len(offsets) <= 36
                or not np.all(np.isfinite(offsets))):
            raise ValueError(
                "relative offsets must contain 1..36 finite XYZ vectors")
        if np.any(np.linalg.norm(offsets, axis=1) <= 0.0):
            raise ValueError("relative target offsets must be nonzero")
        if len(np.unique(np.round(offsets, decimals=12), axis=0)) != len(
                offsets):
            raise ValueError("relative target offsets must be unique")
    elif generator_type == "absolute_targets_m":
        targets = np.asarray(generator.get("targets_m"), dtype=np.float64)
        if (targets.ndim != 2 or targets.shape[1:] != (3,)
                or not 1 <= len(targets) <= 36
                or not np.all(np.isfinite(targets))):
            raise ValueError(
                "absolute targets must contain 1..36 finite XYZ vectors")
        if np.any(np.linalg.norm(targets, axis=1) <= 0.0):
            raise ValueError("absolute targets must be nonzero")
        if np.any(np.abs(targets) > 1.0):
            raise ValueError("absolute targets must be expressed in metres")
        if len(np.unique(np.round(targets, decimals=12), axis=0)) != len(
                targets):
            raise ValueError("absolute targets must be unique")
    elif generator_type == "controller_model_joint_displacements":
        displacements = np.asarray(
            generator.get("logical_displacements"), dtype=np.float64)
        if (displacements.ndim != 2 or displacements.shape[1:] != (3,)
                or not 1 <= len(displacements) <= 16
                or not np.all(np.isfinite(displacements))):
            raise ValueError(
                "model logical_displacements must contain 1..16 triplets")
        if np.any(np.linalg.norm(displacements, axis=1) <= 0.0):
            raise ValueError("model logical displacements must be nonzero")
        if np.any(displacements[:, 1] != 0.0):
            raise ValueError("model sparse no-rotation targets require d1=0")
        steps = int(generator.get("rollout_steps", 0))
        if steps < 1 or steps > 250:
            raise ValueError("model rollout_steps must be in [1,250]")
        reserve = np.asarray(
            generator.get("minimum_endpoint_reserve"), dtype=np.float64)
        if (reserve.shape != (6,) or not np.all(np.isfinite(reserve))
                or np.any(reserve < 0.0)):
            raise ValueError(
                "minimum_endpoint_reserve must contain six nonnegative values")
        service = str(generator.get(
            "service", "/catheter_mppi/generate_sparse_targets"))
        if not service:
            raise ValueError("model target service must be nonempty")
        minimum_tip = float(generator.get("minimum_tip_displacement_mm", 0.0))
        maximum_tip = float(generator.get("maximum_tip_displacement_mm", 0.0))
        if (not math.isfinite(minimum_tip) or not math.isfinite(maximum_tip)
                or minimum_tip <= 0.0 or maximum_tip <= minimum_tip):
            raise ValueError("invalid model target tip-displacement bounds")
        regenerate = generator.get("regenerate_after_each_home", False)
        if not isinstance(regenerate, bool):
            raise ValueError("regenerate_after_each_home must be boolean")
    else:
        raise ValueError(
            "generator.type must be circle_yz_from_current_tip or "
            "relative_offsets_from_current_tip or absolute_targets_m or "
            "controller_model_joint_displacements")
    plant_reset = document.get("plant_reset", {})
    if not isinstance(plant_reset, dict):
        raise ValueError("plant_reset must be a mapping")
    reset_mode = str(
        plant_reset.get("mode", "history_preserving")).strip().lower()
    if reset_mode not in ("history_preserving", "full_simulation"):
        raise ValueError(
            "plant_reset.mode must be history_preserving or full_simulation")
    services = tuple(str(value) for value in plant_reset.get("services", []))
    if reset_mode == "full_simulation" and len(services) != 3:
        raise ValueError(
            "full_simulation reset requires device, perception, and "
            "controller services")
    if any(not value.startswith("/sim/") for value in services):
        raise ValueError("plant reset services must be under /sim")
    reset_timeout_s = float(plant_reset.get("timeout_s", 5.0))
    reset_settle_s = float(plant_reset.get("settle_time_s", 1.0))
    if (not math.isfinite(reset_timeout_s) or reset_timeout_s <= 0.0
            or not math.isfinite(reset_settle_s) or reset_settle_s <= 0.0):
        raise ValueError("plant reset timeout and settle time must be positive")
    requirements = document.get("required_controller_status", {})
    if not isinstance(requirements, dict):
        raise ValueError("required_controller_status must be a mapping")
    rotation_guard = document.get("rotation_guard", {})
    if not isinstance(rotation_guard, dict):
        raise ValueError("rotation_guard must be a mapping")
    guard_enabled = bool(rotation_guard.get("enabled", False))
    guard_axis = int(rotation_guard.get("axis", 1))
    guard_tolerance = float(rotation_guard.get("tolerance", 1.0e-9))
    if guard_axis < 0 or guard_axis >= 6:
        raise ValueError("rotation_guard.axis must be in [0,5]")
    if (not math.isfinite(guard_tolerance) or guard_tolerance < 0.0):
        raise ValueError("rotation_guard.tolerance must be finite and nonnegative")
    return SparsePointSpec(
        **strings, home_position=position, home_speed=speed,
        home_tolerance=tolerance,
        home_timeout_s=_positive(home, "timeout_s"),
        home_mode_settle_s=_positive(home, "mode_settle_time_s"),
        home_settle_s=_positive(home, "settle_time_s"),
        post_home_settle_s=_positive(home, "post_settle_time_s"),
        decoupled_tendon_prehome=decoupled_prehome,
        target_timeout_s=_positive(document, "target_timeout_s"),
        target_tolerance_mm=_positive(document, "target_tolerance_mm"),
        target_settle_s=_positive(document, "target_settle_time_s"),
        plant_reset_mode=reset_mode,
        plant_reset_services=services,
        plant_reset_timeout_s=reset_timeout_s,
        plant_reset_settle_s=reset_settle_s,
        required_controller_status=dict(requirements),
        rotation_guard_enabled=guard_enabled,
        rotation_guard_axis=guard_axis,
        rotation_guard_tolerance=guard_tolerance,
        generator=dict(generator))


def decoupled_tendon_prehome_target(position, home_position):
    """Return a tendon-only target that holds physical chassis axis 0.

    Logical catheter insertion is physical chassis translation plus knob
    translation.  The firmware converts logical targets with
    ``physical_axis_0 = logical_axis_0 - logical_axis_2``.  Consequently,
    sending the final [20, 0, 0] home atomically moves axes 0 and 2 together.
    A tendon lower-bound transition then deliberately stops both axes.  This
    staging target first moves logical axis 2 while preserving the current
    physical axis-0 target, so that the later final home is an axis-0 move.
    """
    position = np.asarray(position, dtype=np.float64)
    home_position = np.asarray(home_position, dtype=np.float64)
    if position.shape != (6,) or home_position.shape != (6,):
        raise ValueError("position and home_position must contain six values")
    if not np.all(np.isfinite(position)) or not np.all(
            np.isfinite(home_position)):
        raise ValueError("position and home_position must be finite")
    target = position.copy()
    target[0] = position[0]-position[2]+home_position[2]
    target[2] = home_position[2]
    return target


def sparse_circle_targets(spec: SparsePointSpec, home_tip_m):
    """Freeze absolute nonduplicated targets from the first home tip."""
    if spec.generator["type"] == "circle_yz_from_current_tip":
        return _circle_waypoints(spec.generator, home_tip_m)
    if spec.generator["type"] == "absolute_targets_m":
        return np.asarray(
            spec.generator["targets_m"], dtype=np.float64).copy()
    if spec.generator["type"] == "controller_model_joint_displacements":
        raise ValueError("model targets require the controller preview service")
    offsets = np.asarray(spec.generator["offsets_mm"], dtype=np.float64)
    return np.asarray(home_tip_m, dtype=np.float64)[None, :]+1e-3*offsets


class SparsePointExperiment(Node):
    def __init__(self, spec: SparsePointSpec):
        super().__init__("catheter_sparse_point_experiment")
        self.spec = spec
        self.tip = None
        self.tip_time = None
        self.position = None
        self.position_time = None
        self.manager_ready = False
        self.safety_text = None
        self.safety_time = None
        self.controller_armed = None
        self.controller_state = None
        self.controller_reason = None
        self.controller_estimator_health = None
        self.controller_accepted_observations = None
        self.controller_consecutive_rejections = None
        self.controller_values = {}
        self.controller_time = None
        self.active_goal_handle = None
        self.monitor_rotation = False
        self.rotation_guard_violation = None
        self.control_pub = self.create_publisher(
            ControlStream, spec.control_topic, 10)
        self.event_pub = self.create_publisher(
            ManagerEvent, spec.event_topic, 10)
        self.create_subscription(
            PointCloud, spec.marker_topic, self._marker_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            DeviceStream, spec.position_topic, self._position_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, spec.status_topic, self._status_cb, 10)
        if spec.rotation_guard_enabled:
            self.create_subscription(
                ControlStream, spec.planned_control_topic,
                self._planned_control_cb, 10)
            self.create_subscription(
                DeviceStream, spec.manager_control_topic,
                self._manager_control_cb, 10)
        safety_qos = QoSProfile(depth=1)
        safety_qos.reliability = ReliabilityPolicy.RELIABLE
        safety_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            ManagerEvent, spec.safety_topic, self._safety_cb, safety_qos)
        self.action = ActionClient(
            self, TrackTipTrajectory, spec.action_name)
        self.reset_clients = [
            self.create_client(Trigger, name)
            for name in spec.plant_reset_services]
        self.model_target_client = (
            self.create_client(
                GenerateSparseTargets,
                str(spec.generator.get(
                    "service", "/catheter_mppi/generate_sparse_targets")))
            if spec.generator["type"]
            == "controller_model_joint_displacements" else None)

    def _marker_cb(self, message):
        try:
            self.tip = np.asarray(
                measured_tip(message, self.spec.frame_id), dtype=np.float64)
            self.tip_time = time.monotonic()
        except ValueError:
            return

    def _position_cb(self, message):
        if message.predicate != DeviceStream.POS or len(message.data) != 6:
            return
        value = np.asarray(message.data, dtype=np.float64)
        if np.all(np.isfinite(value)):
            self.position = value
            self.position_time = time.monotonic()

    def _safety_cb(self, message):
        self.manager_ready = message.text == "MANAGER_READY"
        self.safety_text = message.text
        self.safety_time = time.monotonic()

    def _status_cb(self, message):
        try:
            armed, state, reason = controller_status(message)
        except ValueError:
            return
        self.controller_armed = armed
        self.controller_state = state
        self.controller_reason = reason
        for status in message.status:
            if status.name != "catheter_control/mppi":
                continue
            values = {item.key: item.value for item in status.values}
            self.controller_values = values
            self.controller_estimator_health = values.get(
                "estimator_health")
            try:
                self.controller_accepted_observations = int(
                    values.get("accepted_observations", "-1"))
                self.controller_consecutive_rejections = int(
                    values.get("consecutive_rejections", "-1"))
            except ValueError:
                self.controller_accepted_observations = None
                self.controller_consecutive_rejections = None
        self.controller_time = time.monotonic()

    def _planned_control_cb(self, message):
        if not self.monitor_rotation:
            return
        axis = self.spec.rotation_guard_axis
        if len(message.joint_vel) != 6:
            self.rotation_guard_violation = "planned control has invalid size"
            return
        value = float(message.joint_vel[axis])
        if (not math.isfinite(value)
                or abs(value) > self.spec.rotation_guard_tolerance):
            self.rotation_guard_violation = (
                f"planned rotation axis {axis} was {value:.9g}, exceeding "
                f"{self.spec.rotation_guard_tolerance:.9g}")

    def _manager_control_cb(self, message):
        if not self.monitor_rotation or message.predicate != DeviceStream.VEL:
            return
        axis = self.spec.rotation_guard_axis
        if len(message.data) != 6:
            self.rotation_guard_violation = "manager velocity has invalid size"
            return
        value = float(message.data[axis])
        if (not math.isfinite(value)
                or abs(value) > self.spec.rotation_guard_tolerance):
            self.rotation_guard_violation = (
                f"manager rotation axis {axis} was {value:.9g}, exceeding "
                f"{self.spec.rotation_guard_tolerance:.9g}")

    @staticmethod
    def _status_value_matches(actual, expected):
        if isinstance(expected, (list, tuple)):
            try:
                observed = np.asarray(json.loads(actual), dtype=np.float64)
                wanted = np.asarray(expected, dtype=np.float64)
            except (TypeError, ValueError, json.JSONDecodeError):
                return False
            return (observed.shape == wanted.shape
                    and np.allclose(observed, wanted, rtol=0.0, atol=1.0e-12))
        if isinstance(expected, bool):
            return str(actual).strip().lower() == str(expected).lower()
        return str(actual) == str(expected)

    def _controller_requirements_met(self):
        return all(
            key in self.controller_values
            and self._status_value_matches(self.controller_values[key], value)
            for key, value in self.spec.required_controller_status.items())

    def _controller_requirement_detail(self):
        differences = []
        for key, expected in self.spec.required_controller_status.items():
            actual = self.controller_values.get(key, "missing")
            if (key not in self.controller_values
                    or not self._status_value_matches(actual, expected)):
                differences.append(
                    f"{key}=expected:{expected!r}/actual:{actual!r}")
        return ", ".join(differences) or "satisfied"

    def _reset_full_simulation(self, index):
        if self.spec.plant_reset_mode != "full_simulation":
            return
        if self.controller_armed is not False:
            raise RuntimeError(
                "refusing full simulation reset while MPPI is armed")
        for service_name, client in zip(
                self.spec.plant_reset_services, self.reset_clients):
            if not client.wait_for_service(
                    timeout_sec=self.spec.plant_reset_timeout_s):
                raise RuntimeError(
                    f"simulation reset service unavailable: {service_name}")
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self.spec.plant_reset_timeout_s)
            result = future.result()
            if result is None or not result.success:
                detail = "no response" if result is None else result.message
                raise RuntimeError(
                    f"simulation reset failed at {service_name}: {detail}")

        # Require observations produced after all three resets. This prevents
        # a pre-reset marker or status sample from defining the next target.
        self.tip = None
        self.tip_time = None
        self.controller_state = None
        self.controller_reason = None
        self.controller_estimator_health = None
        self.controller_accepted_observations = None
        self.controller_consecutive_rejections = None
        self.controller_time = None

        def reset_ready():
            now = time.monotonic()
            return bool(
                self.tip is not None and self.tip_time is not None
                and now-self.tip_time < 0.5
                and self.controller_armed is False
                and self.controller_state == "DISARMED"
                and self.controller_estimator_health == "TRACKING"
                and self.controller_accepted_observations is not None
                and self.controller_accepted_observations > 0
                and self.controller_consecutive_rejections == 0)

        # A single queued pre-reset TRACKING diagnostic is not proof that the
        # new estimator has initialized.  Require the complete healthy state
        # to remain true for the configured settle interval.
        deadline = time.monotonic()+self.spec.plant_reset_timeout_s
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if self.controller_state == "FAULTED":
                raise RuntimeError(
                    f"point {index+1} controller faulted during simulation "
                    f"reset: {self.controller_reason}")
            if reset_ready():
                if stable_since is None:
                    stable_since = now
                elif now-stable_since >= self.spec.plant_reset_settle_s:
                    break
            else:
                stable_since = None
        else:
            raise RuntimeError(
                f"point {index+1} simulation reset did not reinitialize "
                "truth and controller estimator: "
                f"state={self.controller_state}, "
                f"reason={self.controller_reason}, "
                f"health={self.controller_estimator_health}, "
                f"accepted={self.controller_accepted_observations}, "
                f"rejections={self.controller_consecutive_rejections}")
        self.get_logger().info(
            f"point {index+1} full simulation plant/controller reset complete; "
            f"accepted={self.controller_accepted_observations}")

    def _spin_until(self, predicate, timeout_s):
        deadline = time.monotonic()+float(timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return False

    def generate_targets(self, home_tip_m, candidate_indices=None):
        if self.model_target_client is None:
            if candidate_indices is not None:
                raise ValueError(
                    "candidate selection requires controller model targets")
            return sparse_circle_targets(self.spec, home_tip_m)
        timeout_s = float(self.spec.generator.get("service_timeout_s", 10.0))
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("model target service_timeout_s must be positive")
        if not self.model_target_client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("model target preview service unavailable")
        all_displacements = np.asarray(
            self.spec.generator["logical_displacements"],
            dtype=np.float64)
        if candidate_indices is None:
            indices = np.arange(len(all_displacements), dtype=int)
        else:
            indices = np.asarray(candidate_indices, dtype=int)
            if (indices.ndim != 1 or len(indices) < 1
                    or np.any(indices < 0)
                    or np.any(indices >= len(all_displacements))
                    or len(np.unique(indices)) != len(indices)):
                raise ValueError("invalid model candidate indices")
        selected_displacements = all_displacements[indices]
        request = GenerateSparseTargets.Request()
        request.logical_displacements = selected_displacements.reshape(
            -1).tolist()
        request.rollout_steps = int(self.spec.generator["rollout_steps"])
        request.minimum_endpoint_reserve = np.asarray(
            self.spec.generator["minimum_endpoint_reserve"],
            dtype=np.float64).tolist()
        request.minimum_tip_displacement_mm = float(
            self.spec.generator["minimum_tip_displacement_mm"])
        request.maximum_tip_displacement_mm = float(
            self.spec.generator["maximum_tip_displacement_mm"])
        future = self.model_target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None:
            raise RuntimeError("model target preview timed out")
        if not response.success:
            raise RuntimeError(
                f"model target preview rejected: {response.message}")
        targets = np.asarray(
            [[point.x, point.y, point.z] for point in response.targets],
            dtype=np.float64)
        expected = len(indices)
        if (targets.shape != (expected, 3)
                or not np.all(np.isfinite(targets))):
            raise RuntimeError("model target preview returned invalid targets")
        realized = np.asarray(
            response.realized_logical_displacements,
            dtype=np.float64).reshape(expected, 6)
        endpoint = np.asarray(
            response.endpoint_joint_positions,
            dtype=np.float64).reshape(expected, 6)
        for response_index, candidate_index in enumerate(indices):
            self.get_logger().info(
                "model candidate %d/%d from current home "
                "tip=[%.3f, %.3f, %.3f] mm, "
                "realized joint delta=%s, endpoint=%s" % (
                    candidate_index+1, len(all_displacements),
                    *(1000.0*targets[response_index]),
                    np.round(realized[response_index], 4).tolist(),
                    np.round(endpoint[response_index], 4).tolist()))
        return targets

    def _publish_mode(self, mode):
        message = ManagerEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.spec.source_name
        message.predicate = ManagerEvent.MODE
        message.text = chr(mode)
        self.event_pub.publish(message)

    def _publish_home(self, target=None):
        message = ControlStream()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.spec.source_name
        if target is None:
            target = self.spec.home_position
        message.joint_pos = np.asarray(target, dtype=np.float64).tolist()
        message.joint_vel = self.spec.home_speed.tolist()
        self.control_pub.publish(message)

    def _run_home_stage(self, target, axes, deadline, label):
        """Drive one guarded position stage without leaving position mode."""
        target = np.asarray(target, dtype=np.float64)
        axes = np.asarray(axes, dtype=np.int64)
        within_since = None
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if (not self.manager_ready or self.safety_time is None
                    or now-self.safety_time > 1.0):
                raise RuntimeError(
                    "manager became inhibited or stale while homing")
            if now >= next_publish:
                self._publish_home(target)
                next_publish = now+0.10
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if (self.position is None or self.position_time is None
                    or now-self.position_time > 0.5):
                within_since = None
                continue
            within = np.all(
                np.abs(self.position[axes]-target[axes])
                <= self.spec.home_tolerance[axes])
            if not within:
                within_since = None
            elif within_since is None:
                within_since = now
            elif now-within_since >= self.spec.home_settle_s:
                self.get_logger().info(
                    "%s complete: %s" % (
                        label, np.round(self.position, 4).tolist()))
                return
        if self.position is None:
            detail = "no POS feedback"
        else:
            residual = target-self.position
            detail = (
                f"position={np.round(self.position, 4).tolist()} "
                f"residual={np.round(residual, 4).tolist()} "
                f"tolerance={self.spec.home_tolerance.tolist()}")
        raise RuntimeError(f"{label} timed out: {detail}")

    def wait_for_inputs(self, timeout_s):
        def ready():
            now = time.monotonic()
            return bool(
                self.tip is not None and self.position is not None
                and self.tip_time is not None and self.position_time is not None
                and self.safety_time is not None
                and self.controller_time is not None
                and self.manager_ready and self.controller_armed is False
                and self._controller_requirements_met()
                and now-self.tip_time < 0.5
                and now-self.position_time < 0.5
                and now-self.safety_time < 1.0
                and now-self.controller_time < 0.5)
        if not self._spin_until(ready, timeout_s):
            now = time.monotonic()

            def age(timestamp):
                return ("missing" if timestamp is None
                        else f"age={1000.0*(now-timestamp):.1f}ms")

            raise RuntimeError(
                "timed out waiting for startup gates: "
                f"tip={age(self.tip_time)}, POS={age(self.position_time)}, "
                f"manager={self.safety_text or 'missing'} "
                f"({age(self.safety_time)}), "
                f"controller={self.controller_state or 'missing'} "
                f"armed={self.controller_armed} "
                f"({age(self.controller_time)}), requirements="
                f"{self._controller_requirement_detail()}")

    def wait_for_disarmed(self, timeout_s=5.0):
        if not self._spin_until(
                lambda: self.controller_armed is False, timeout_s):
            raise RuntimeError("controller did not report disarmed")

    def home(self, index):
        if not self.manager_ready:
            raise RuntimeError("manager is not ready before homing")
        if self.controller_armed is not False:
            raise RuntimeError("refusing to home while MPPI is armed")
        self._publish_mode(ManagerEvent.JOINT_POS)
        # MODE handling emits a manager-stamped zero-velocity command.  Give
        # that command time to cross the manager/device boundary before
        # publishing a source-stamped position command; otherwise DDS topic
        # interleaving can make the first position stamp appear nonmonotonic.
        mode_ready = time.monotonic()+self.spec.home_mode_settle_s
        while rclpy.ok() and time.monotonic() < mode_ready:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.manager_ready:
                self._publish_mode(ManagerEvent.NONE)
                raise RuntimeError(
                    "manager became inhibited while entering position mode")
        deadline = time.monotonic()+self.spec.home_timeout_s
        try:
            if (self.spec.decoupled_tendon_prehome
                    and self.position is not None
                    and abs(self.position[2]-self.spec.home_position[2])
                    > self.spec.home_tolerance[2]):
                prehome = decoupled_tendon_prehome_target(
                    self.position, self.spec.home_position)
                self.get_logger().info(
                    "point %d tendon prehome: target=%s; physical axis 0 "
                    "held at %.4f" % (
                        index+1, np.round(prehome, 4).tolist(),
                        self.position[0]-self.position[2]))
                self._run_home_stage(
                    prehome, [0, 2], deadline,
                    f"point {index+1} tendon prehome")
            self._run_home_stage(
                self.spec.home_position, range(6), deadline,
                f"point {index+1} home")
        except RuntimeError:
            self._publish_mode(ManagerEvent.NONE)
            raise
        self._publish_mode(ManagerEvent.NONE)
        settle_deadline = time.monotonic()+self.spec.post_home_settle_s
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        self._reset_full_simulation(index)

    def run_target(self, index, count, target):
        if not self._controller_requirements_met():
            raise RuntimeError(
                "controller runtime identity does not match experiment: "
                f"{self._controller_requirement_detail()}")
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("timed out waiting for trajectory action server")
        goal = TrackTipTrajectory.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.spec.frame_id
        point = Point()
        point.x, point.y, point.z = np.asarray(target).tolist()
        goal.waypoints = [point]
        goal.waypoint_timeouts_s = [self.spec.target_timeout_s]
        goal.tolerance_mm = self.spec.target_tolerance_mm
        goal.settle_time_s = self.spec.target_settle_s
        goal.auto_arm = True
        home_tip = self.tip.copy()
        error = 1000.0*(np.asarray(target)-home_tip)
        self.get_logger().info(
            "point %d/%d target=[%.3f, %.3f, %.3f] mm; "
            "home error=[%.3f, %.3f, %.3f] mm" % (
                index+1, count, *(1000.0*np.asarray(target)), *error))
        self.rotation_guard_violation = None
        self.monitor_rotation = self.spec.rotation_guard_enabled
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.monitor_rotation = False
            raise RuntimeError(f"point {index+1} goal was rejected")
        self.active_goal_handle = handle
        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.rotation_guard_violation is not None:
                detail = self.rotation_guard_violation
                self.cancel_active()
                self._publish_mode(ManagerEvent.NONE)
                self.monitor_rotation = False
                raise RuntimeError(
                    f"point {index+1} rotation guard tripped: {detail}")
            if not self._controller_requirements_met():
                detail = self._controller_requirement_detail()
                self.cancel_active()
                self._publish_mode(ManagerEvent.NONE)
                self.monitor_rotation = False
                raise RuntimeError(
                    "controller runtime identity changed during target: "
                    f"{detail}")
        self.monitor_rotation = False
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"point {index+1} returned no result")
        result = wrapped.result
        self.get_logger().info(
            "point %d/%d finished: reached=%d timed_out=%d error=%.3f mm" % (
                index+1, count, result.reached_waypoints,
                result.timed_out_waypoints, result.final_error_mm))
        if not result.success:
            raise RuntimeError(
                f"point {index+1} controller action failed: {result.message}")
        self.active_goal_handle = None
        self.wait_for_disarmed()
        return result

    def cancel_active(self):
        self.monitor_rotation = False
        handle = self.active_goal_handle
        if handle is None:
            return
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.active_goal_handle = None


def _arguments(argv):
    parser = argparse.ArgumentParser(description=(
        "Home before each independent sparse-circle tip target"))
    parser.add_argument("yaml_file")
    parser.add_argument("--startup-timeout-s", type=float, default=10.0)
    result = parser.parse_args(remove_ros_args(args=argv)[1:])
    if (not math.isfinite(result.startup_timeout_s)
            or result.startup_timeout_s <= 0.0):
        parser.error("--startup-timeout-s must be finite and positive")
    return result


def main(args=None):
    argv = sys.argv if args is None else args
    parsed = _arguments(argv)
    spec = load_sparse_point_experiment(parsed.yaml_file)
    rclpy.init(args=argv)
    node = SparsePointExperiment(spec)
    results = []
    try:
        node.wait_for_inputs(parsed.startup_timeout_s)
        regenerate = bool(spec.generator.get(
            "regenerate_after_each_home", False))
        if regenerate:
            count = len(spec.generator["logical_displacements"])
            for index in range(count):
                node.home(index)
                if node.tip is None:
                    raise RuntimeError(
                        f"no measured tip after point {index+1} home")
                home_tip = node.tip.copy()
                target = node.generate_targets(
                    home_tip, candidate_indices=[index])[0]
                node.get_logger().info(
                    "generated point %d/%d after its guarded home "
                    "tip=[%.3f, %.3f, %.3f] mm" % (
                        index+1, count, *(1000.0*home_tip)))
                results.append(node.run_target(index, count, target))
        else:
            node.home(0)
            if node.tip is None:
                raise RuntimeError("no measured tip after initial home")
            home_tip = node.tip.copy()
            targets = node.generate_targets(home_tip)
            node.get_logger().info(
                "froze %d sparse targets from home "
                "tip=[%.3f, %.3f, %.3f] mm" % (
                    len(targets), *(1000.0*home_tip)))
            for index, target in enumerate(targets):
                if index:
                    node.home(index)
                results.append(node.run_target(
                    index, len(targets), target))
        reached = sum(result.reached_waypoints for result in results)
        timed_out = sum(result.timed_out_waypoints for result in results)
        node.get_logger().info(
            f"sparse-point experiment complete: {reached} reached, "
            f"{timed_out} timed out")
    except KeyboardInterrupt:
        node.cancel_active()
        node._publish_mode(ManagerEvent.NONE)
        node.get_logger().warn("sparse-point experiment interrupted")
    except Exception:
        node.cancel_active()
        node._publish_mode(ManagerEvent.NONE)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
