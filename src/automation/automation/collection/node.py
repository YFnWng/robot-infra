"""Automated data-collection node: drive the robot with velocity commands.

Publishes ``control_interface/ControlStream`` velocity commands on
``/teleop/control`` (same path as manual teleop) after selecting ``JOINT_VEL``
mode via ``/teleop/event``. Emits run markers on ``/collection/events``.

This first stage supports ``mode:=constant`` (a single joint at a fixed,
speed-clamped velocity) for safe bring-up. ``mode:=sinusoidal`` (reusing the SOFA
``SinusoidalGenerator``) is added next.

Safety
------
* Velocities are clamped to ``joint_max_speeds`` per joint.
* Motors are NOT started unless ``start_motor:=true`` (default false) — so the
  node's output can be verified with ``ros2 topic echo`` before touching hardware.
* On stop / shutdown the node commands zero velocity.
"""
from __future__ import annotations

import json

from control_interface.msg import ControlStream, DeviceStream, ManagerEvent
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Joint order matches teleop/config/params.yaml
JOINTS = [
    "catheter_lin", "catheter_rot", "catheter_bend",
    "sheath_lin", "sheath_rot", "sheath_bend",
]
TARGET_JOINTS = {"catheter": [0, 1, 2], "sheath": [3, 4, 5]}
ROT_JOINTS = {1, 4}   # rotation joints (deg, wrap-around): catheter_rot, sheath_rot
DEFAULT_MAX_SPEEDS = [5.0, 30.0, 1.0, 5.0, 30.0, 30.0]


class CollectionNode(Node):
    """Velocity-command source for automated data collection."""

    def __init__(self) -> None:
        super().__init__("collection")

        self.declare_parameter("source_name", "autonomy")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("mode", "constant")          # constant | sinusoidal
        self.declare_parameter("target", "catheter")        # catheter | sheath
        self.declare_parameter("joint_max_speeds", DEFAULT_MAX_SPEEDS)
        self.declare_parameter("auto_enable", True)          # send MODE=JOINT_VEL
        self.declare_parameter("start_motor", False)         # send START_MOTOR
        self.declare_parameter("shutdown_on_done", True)
        # constant-mode params
        self.declare_parameter("test_joint", 0)              # absolute index 0..5
        self.declare_parameter("test_velocity", 0.0)         # units per joint
        # sinusoidal-mode params (target = the 3 driven joints)
        self.declare_parameter("sofa_sim_path", "/home/wangyf/sofa-cosserat-sim")
        self.declare_parameter("seed", -1)                   # <0 => random
        self.declare_parameter("joint_lower", [0.0, -180.0, 0.0])   # target joints
        self.declare_parameter("joint_upper", [0.1, 180.0, 0.01])   # PLACEHOLDER: set real limits
        self.declare_parameter("speed_factor", 1.3)
        self.declare_parameter("ramp_duration", 2.0)
        self.declare_parameter("freq_change_interval", 10.0)
        self.declare_parameter("amp_range", [0.2, 1.0])
        # per-catheter pos + vel limits from YAML (overrides the joint_lower/upper
        # and joint_max_speeds params when limits_file is set)
        self.declare_parameter("limits_file", "")
        self.declare_parameter("catheter", "default")
        self.declare_parameter("expect_enc", True)      # warn if no raw-ENC frames
        # return-to-start: after the trajectory, servo the target joints back to
        # the pose captured at run_start (proportional velocity, saturated to
        # vel_max, shortest angle for rotation joints).
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("return_kp", 1.0)          # gain, 1/s
        self.declare_parameter("return_tol", 0.5)         # mm or deg, per joint
        self.declare_parameter("return_timeout_s", 30.0)

        self._source = self.get_parameter("source_name").value
        self._rate = float(self.get_parameter("rate_hz").value)
        self._duration = float(self.get_parameter("duration_s").value)
        self._mode = self.get_parameter("mode").value
        self._target = self.get_parameter("target").value
        self._max_speeds = np.asarray(
            self.get_parameter("joint_max_speeds").value, dtype=float)
        self._start_motor = bool(self.get_parameter("start_motor").value)
        self._shutdown_on_done = bool(self.get_parameter("shutdown_on_done").value)
        self._test_joint = int(self.get_parameter("test_joint").value)
        self._test_velocity = float(self.get_parameter("test_velocity").value)

        # per-catheter limits (6-joint pos_lower/upper + vel_max) override the
        # joint_lower/upper and joint_max_speeds params if a limits_file is given.
        self._pos_lower6, self._pos_upper6, _vel_max6 = self._load_limits()
        if _vel_max6 is not None:
            self._max_speeds = _vel_max6

        if self._target not in TARGET_JOINTS:
            raise ValueError(f"target must be catheter|sheath, got {self._target}")
        self._target_idx = TARGET_JOINTS[self._target]

        self._gen = None
        self._shortest_delta = None
        self._h = 1.0 / self._rate
        if self._mode == "sinusoidal":
            self._build_generator()

        self._control_pub = self.create_publisher(ControlStream, "/teleop/control", 10)
        self._event_pub = self.create_publisher(ManagerEvent, "/teleop/event", 10)
        self._marker_pub = self.create_publisher(String, "/collection/events", 10)

        # Latest device state (firmware reports predicate 'P'/'V'/'E' + 6 values),
        # captured so the run_start marker records the initial joint configuration.
        self._last_state = None
        self._last_enc = None          # latest raw-ENC frame (start encoder counts)
        self._last_pos = None          # latest reported POS 6-vector (return feedback)
        self._enc_seen = False
        self.create_subscription(DeviceStream, "/device/state", self._state_cb, 10)

        self._t0 = None
        self._done = False
        self._returning = False
        self._return_t0 = None
        self._start_pos = None         # pose captured at run_start (return target)
        self.should_exit = False

        # Enable after a short delay so publishers finish discovery.
        self._start_timer = self.create_timer(0.5, self._start)

    # -- lifecycle -------------------------------------------------------- #
    def _start(self) -> None:
        self._start_timer.cancel()
        if self.get_parameter("auto_enable").value:
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.JOINT_VEL))
            self.get_logger().info("requested JOINT_VEL mode")
        if self._start_motor:
            self._send_event(ManagerEvent.START_MOTOR)
            self.get_logger().warn("START_MOTOR sent — hardware may move")
        self._marker("run_start", mode=self._mode, target=self._target,
                     seed=int(self.get_parameter("seed").value),
                     start_state=self._state_snapshot())
        self._start_pos = list(self._last_pos) if self._last_pos is not None else None
        if self.get_parameter("return_to_start").value and self._start_pos is None:
            self.get_logger().warn("no POS at run_start — return-to-start disabled")
        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        if self.get_parameter("expect_enc").value:       # early ENC warning
            self._enc_timer = self.create_timer(3.0, self._check_enc_early)
        self.get_logger().info(
            f"collection running: mode={self._mode} target={self._target} "
            f"rate={self._rate}Hz duration={self._duration}s source={self._source}")

    def _check_enc_early(self) -> None:
        self._enc_timer.cancel()
        if not self._enc_seen:
            self.get_logger().error(
                "NO raw-ENC frames on /device/state after 3s — is the Teensy flashed "
                "with the ENC firmware? Recorded data will lack raw encoders.")

    def _tick(self) -> None:
        if self._done:
            return
        if self._returning:
            self._return_tick()
            return
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        done = t >= self._duration or (self._gen is not None and self._gen.is_done(t))
        if done:
            self._begin_return()
            return
        vel = np.clip(self._velocity(t), -self._max_speeds, self._max_speeds)
        self._publish_velocity(vel)

    # -- return to start -------------------------------------------------- #
    def _begin_return(self) -> None:
        """Trajectory finished: settle at zero, then either servo the target
        joints back to the start pose or finish immediately."""
        for _ in range(5):
            self._publish_velocity(np.zeros(6))
        if (self.get_parameter("return_to_start").value
                and self._start_pos is not None and self._last_pos is not None):
            self._returning = True
            self._return_t0 = self.get_clock().now()
            tgt = [round(self._start_pos[j], 2) for j in self._target_idx]
            self._marker("return_start", target_joints=list(self._target_idx),
                         start_pose=tgt)
            self.get_logger().info(f"returning joints {self._target_idx} to start {tgt}")
        else:
            if self.get_parameter("return_to_start").value:
                self.get_logger().warn("return-to-start skipped (no POS feedback)")
            self._finish()

    def _return_step(self):
        """Proportional velocity to drive target joints toward the start pose.
        Returns (vel6, done); rotation joints use the shortest signed angle."""
        kp = float(self.get_parameter("return_kp").value)
        tol = float(self.get_parameter("return_tol").value)
        v = np.zeros(6)
        done = True
        for j in self._target_idx:
            cur = self._last_pos[j]
            tgt = self._start_pos[j]
            if j in ROT_JOINTS:
                err = (tgt - cur + 180.0) % 360.0 - 180.0
            else:
                err = tgt - cur
            if abs(err) > tol:
                done = False
                v[j] = float(np.clip(kp * err,
                                     -self._max_speeds[j], self._max_speeds[j]))
        return v, done

    def _return_tick(self) -> None:
        elapsed = (self.get_clock().now() - self._return_t0).nanoseconds * 1e-9
        v, done = self._return_step()
        if done or elapsed > float(self.get_parameter("return_timeout_s").value):
            for _ in range(5):
                self._publish_velocity(np.zeros(6))
            self._returning = False
            self.get_logger().info(
                "returned to start" if done else "return-to-start timed out")
            self._finish()
            return
        self._publish_velocity(v)

    def _finish(self) -> None:
        self._done = True
        self._timer.cancel()
        for _ in range(5):                       # settle at zero
            self._publish_velocity(np.zeros(6))
        self._marker("run_end", enc_seen=self._enc_seen)
        if self._start_motor:
            self._send_event(ManagerEvent.STOP_MOTOR)
        if self.get_parameter("auto_enable").value:
            # release the manager's exclusive-control lock and return it to idle
            self._send_event(ManagerEvent.MODE, text=chr(ManagerEvent.NONE))
        if self.get_parameter("expect_enc").value and not self._enc_seen:
            self.get_logger().error(
                "run finished with NO raw-ENC frames — recorded data lacks raw "
                "encoders (flash the ENC firmware).")
        self.get_logger().info("collection complete")
        if self._shutdown_on_done:
            self.should_exit = True

    # -- command generation ---------------------------------------------- #
    def _build_generator(self) -> None:
        import os
        import sys
        sofa_path = self.get_parameter("sofa_sim_path").value or os.environ.get(
            "SOFA_SIM_PATH", "")
        if sofa_path and sofa_path not in sys.path:
            sys.path.insert(0, sofa_path)
        try:
            from data_collection.generators.base import InputGenerator
            from data_collection.generators.sinusoidal import SinusoidalGenerator
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                f"cannot import SOFA generator from '{sofa_path}'; set sofa_sim_path "
                f"or SOFA_SIM_PATH: {exc}")

        self._shortest_delta = InputGenerator.shortest_rotation_delta
        idx = self._target_idx
        if self._pos_lower6 is not None:                 # from limits_file
            lower = self._pos_lower6[idx]
            upper = self._pos_upper6[idx]
        else:                                            # from direct params
            lower = np.asarray(self.get_parameter("joint_lower").value, dtype=float)
            upper = np.asarray(self.get_parameter("joint_upper").value, dtype=float)
        seed = int(self.get_parameter("seed").value)
        self._gen = SinusoidalGenerator(
            joint_lower_limits=lower,
            joint_upper_limits=upper,
            dt=self._h,
            joint_max_speeds=self._max_speeds[idx],
            speed_factor=float(self.get_parameter("speed_factor").value),
            duration=self._duration,
            ramp_duration=float(self.get_parameter("ramp_duration").value),
            freq_change_interval=float(self.get_parameter("freq_change_interval").value),
            amp_range=tuple(self.get_parameter("amp_range").value),
            seed=None if seed < 0 else seed,
        )
        self.get_logger().info(
            f"sinusoidal generator: target={self._target} joints={idx} "
            f"lower={lower} upper={upper} max_speeds={self._max_speeds[idx]} seed={seed}")

    def _velocity(self, t: float) -> np.ndarray:
        """Return a length-6 velocity vector for time t."""
        v = np.zeros(6)
        if self._mode == "constant":
            if 0 <= self._test_joint < 6:
                v[self._test_joint] = self._test_velocity
        elif self._mode == "sinusoidal":
            # velocity feed-forward = finite difference of the generator's
            # position, with the rotation joint (local index 1) unwrapped.
            q0 = self._gen.step(t)
            q1 = self._gen.step(t + self._h)
            v[self._target_idx] = self._shortest_delta(q0, q1) / self._h
        else:
            raise ValueError(f"unknown mode '{self._mode}'")
        return v

    # -- state ----------------------------------------------------------- #
    def _state_cb(self, msg: DeviceStream) -> None:
        snap = {
            "predicate": chr(msg.predicate),          # 'P' pos, 'V' vel, 'E' enc
            "data": [float(x) for x in msg.data],
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        self._last_state = snap
        if msg.predicate == DeviceStream.POS:
            self._last_pos = snap["data"]             # feedback for return-to-start
        elif msg.predicate == DeviceStream.ENC:
            self._enc_seen = True
            self._last_enc = snap                     # raw encoder counts

    def _load_limits(self):
        """Load per-catheter (pos_lower, pos_upper, vel_max), each 6 joints, from
        the YAML `limits_file` under profile `catheter`. Returns (None,None,None)
        if no file is set (fall back to the joint_lower/upper/max_speeds params)."""
        path = self.get_parameter("limits_file").value
        if not path:
            return None, None, None
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
        name = self.get_parameter("catheter").value
        profiles = cfg.get("catheters", {})
        if name not in profiles:
            raise ValueError(f"catheter '{name}' not in {path}; have {list(profiles)}")
        p = profiles[name]
        out = tuple(np.asarray(p[k], dtype=float)
                    for k in ("pos_lower", "pos_upper", "vel_max"))
        for arr, k in zip(out, ("pos_lower", "pos_upper", "vel_max")):
            if arr.shape != (6,):
                raise ValueError(f"'{k}' for catheter '{name}' must have 6 values")
        self.get_logger().info(f"limits: catheter '{name}' from {path}")
        return out

    def _state_snapshot(self):
        """run_start initial condition: the latest device frame plus the latest
        raw-ENC frame (starting encoder counts, the model input)."""
        if self._last_state is None:
            self.get_logger().warn("no /device/state yet — run_start has no start_state")
        return {"last": self._last_state, "enc": self._last_enc}

    # -- publishing helpers ---------------------------------------------- #
    def _publish_velocity(self, vel: np.ndarray) -> None:
        msg = ControlStream()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._source
        msg.joint_vel = [float(x) for x in vel]
        self._control_pub.publish(msg)

    def _send_event(self, predicate: int, text: str = "") -> None:
        ev = ManagerEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.header.frame_id = self._source
        ev.predicate = predicate
        ev.text = text
        self._event_pub.publish(ev)

    def _marker(self, event: str, **fields) -> None:
        payload = {"event": event, "stamp_ns": self.get_clock().now().nanoseconds}
        payload.update(fields)
        self._marker_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollectionNode()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
