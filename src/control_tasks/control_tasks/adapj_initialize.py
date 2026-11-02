"""Explicit initialization using the existing guarded sparse-task homing owner."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time
import sys

import numpy as np
import rclpy
from std_srvs.srv import SetBool, Trigger
from rclpy.utilities import remove_ros_args
from control_interface.msg import ManagerEvent
from ament_index_python.packages import get_package_share_directory
from catheter_control.safety.hardware_contract import (
    HardwareContract, load_hardware_contract)
from catheter_control.planning.adapj import (
    initialization_shaft_step, initialization_response_confirmed)

from .sparse_point_experiment import SparsePointExperiment, load_sparse_point_experiment


def initialization_positions(home):
    """Small deterministic two-axis calibration; never change encoder zero."""
    home = np.asarray(home, dtype=float)
    offsets = [(0, 0), (2, 0), (4, 0), (2, 0), (0, 0),
               (0, 2), (0, 4), (0, 2), (0, 0),
               (2, 2), (4, 4), (2, 4), (4, 2), (0, 0)]
    positions = []
    for insertion, tendon in offsets:
        point = home.copy()
        point[[0, 2]] += [insertion, tendon]
        positions.append(point)
    return positions


def capture(node, enabled):
    client = node.create_client(SetBool, "/catheter_mppi/adapj_initialization")
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError("AdapJ initialization service unavailable")
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        if not node._spin_until(future.done, 10.):
            raise RuntimeError("AdapJ initialization service timed out")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("no initialization response" if response is None else response.message)
        return response.message
    finally:
        node.destroy_client(client)


def run_compensated_initialization(node, spec, contract, *, step_mm=.5,
                                   maximum_acquisition_mm=10., calibration_mm=2.,
                                   timeout_s=120.):
    """Bounded acquisition -> stopped confirmation -> same-direction samples.

    No AdapJ actions or automatic fault clearing are used. The existing manager
    position stages own every move; all failure exits stop their transaction.
    """
    for value in (step_mm, maximum_acquisition_mm, calibration_mm, timeout_s):
        if not np.isfinite(value) or value <= 0:
            raise ValueError("compensated initialization bounds must be finite and positive")
    if step_mm > maximum_acquisition_mm:
        raise ValueError("acquisition step exceeds its travel bound")
    deadline = time.monotonic()+timeout_s
    # Use low, publishable profile speeds; do not change manager speed floors.
    speeds = spec.home_speed.copy()
    speeds[[0, 2]] = np.maximum(contract.velocity_min[[0, 2]],
                                np.minimum(speeds[[0, 2]], 2.))
    node.spec = replace(spec, home_speed=speeds)
    node.get_logger().info(
        f"compensated AdapJ calibration: step={step_mm} mm, acquisition bound="
        f"{maximum_acquisition_mm} mm/leg, calibration={calibration_mm} mm/leg, "
        f"total budget={timeout_s} s; physical shaft isolation")

    def guard():
        now = time.monotonic()
        try:
            phases = json.loads(node.controller_values["backlash_phase"])
            if not isinstance(phases, list) or len(phases) != 3:
                raise ValueError("expected three shaft phases")
        except (KeyError, ValueError, TypeError) as error:
            raise RuntimeError("shared confirmation status unavailable or invalid") from error
        if "FAILED" in phases:
            raise RuntimeError("shared confirmation failure during compensated initialization")
        if (not rclpy.ok() or now >= deadline or not node.manager_ready or node.controller_armed is not False
                or node.controller_state == "FAULTED" or not node._controller_requirements_met()
                or any(stamp is None or now-stamp > age for stamp, age in
                       ((node.position_time, .5), (node.tip_time, .5),
                        (node.controller_time, .5), (node.safety_time, 1.)))):
            raise RuntimeError("compensated initialization deadline, identity or fresh-input gate failed")

    acquisition_evidence_ns = None

    def confirmed(axis, direction, since, provisional=False):
        guard()
        return initialization_response_confirmed(node.controller_values, axis, direction, since,
                                  node.get_clock().now().nanoseconds, provisional=provisional,
                                  acquisition_evidence_ns=(None if provisional else acquisition_evidence_ns))

    def position_mode():
        node._publish_mode(ManagerEvent.JOINT_POS)
        until = time.monotonic()+spec.home_mode_settle_s
        while rclpy.ok() and time.monotonic() < until:
            guard()
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        # Acquire tendon first so chassis calibration does not overlap its
        # unconfirmed home state. Each reversal gets a new evidence boundary.
        for leg, (axis, direction) in enumerate(((2, 1), (0, 1), (0, -1), (2, -1))):
            acquisition_evidence_ns = None  # Never inherit a prior leg's proof.
            guard()
            since = node.get_clock().now().nanoseconds
            anchor = HardwareContract.logical_position_to_motor_axis_position(node.position)[axis]
            position_mode()
            acquired = False
            while not acquired:
                guard()
                current = HardwareContract.logical_position_to_motor_axis_position(node.position)[axis]
                remaining = maximum_acquisition_mm-direction*(current-anchor)
                if remaining <= spec.home_tolerance[axis]:
                    raise RuntimeError(f"axis {axis} acquisition travel exhausted without confirmation")
                target = initialization_shaft_step(node.position, axis, direction*min(step_mm, remaining), contract)
                interrupted = node._run_home_stage(target, range(6), deadline,
                    f"AdapJ acquisition leg {leg+1} axis {axis}",
                    stop_when=lambda: confirmed(axis, direction, since, provisional=True))
                if interrupted or confirmed(axis, direction, since, provisional=True):
                    # Latch the freshly observed onset before the 200-ms stop
                    # barrier and mode settling can age its event timestamp.
                    acquisition_evidence_ns = json.loads(
                        node.controller_values['backlash_last_evidence_timestamp_ns'])[axis]
                    # Reuse the verified stopped-feedback barrier before any
                    # further position command. It never clears a safety fault.
                    node._stop_before_home_retry(deadline, time.monotonic())
                    hold_target = node.position.copy()
                    hold_end = min(deadline, time.monotonic()+2.)
                    while rclpy.ok() and time.monotonic() < hold_end:
                        acquired = confirmed(axis, direction, since)
                        if acquired:
                            break
                        node._publish_home(hold_target)
                        rclpy.spin_once(node, timeout_sec=.02)
                    if not acquired:
                        raise RuntimeError(f"axis {axis} response did not persist to engagement confirmation")
            start = node.position.copy()
            node.get_logger().info(f"AdapJ acquisition confirmed: leg={leg+1}, axis={axis}, "
                                   f"direction={direction}, position={start.tolist()}")
            target = initialization_shaft_step(start, axis, direction*calibration_mm, contract)
            # A loss of confirmation aborts calibration rather than silently
            # labeling another acquisition interval as useful motion.
            def calibration_guard():
                if not confirmed(axis, direction, since):
                    raise RuntimeError(f"axis {axis} confirmation lost during calibration")
                return False
            node._run_home_stage(target, range(6), deadline,
                                 f"AdapJ useful calibration leg {leg+1}",
                                 stop_when=calibration_guard)
        # Fit before final home, so returning home is recorded but not fitted.
        node._publish_mode(ManagerEvent.NONE)
        result = capture(node, False)
        guard()
        remaining = deadline-time.monotonic()-spec.home_mode_settle_s-spec.post_home_settle_s
        if remaining <= 0:
            raise RuntimeError("initialization budget exhausted before final home")
        node.spec = replace(spec, home_timeout_s=min(spec.home_timeout_s, remaining))
        node.home(4)
        return result
    finally:
        node.spec = spec
        node._publish_mode(ManagerEvent.NONE)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("yaml_file")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--limits-file", type=Path)
    parser.add_argument("--catheter", default="imricor_test")
    parser.add_argument("--acquisition-step-mm", type=float, default=.5)
    parser.add_argument("--acquisition-max-travel-mm", type=float, default=10.)
    parser.add_argument("--calibration-travel-mm", type=float, default=2.)
    ros_args = sys.argv if argv is None else ["adapj_initialize", *argv]
    args = parser.parse_args(remove_ros_args(args=ros_args)[1:])
    if not args.execute:
        parser.error("--execute is required: initialization moves the robot")
    spec = load_sparse_point_experiment(args.yaml_file)
    if spec.required_controller_status.get("control_method") != "adapj":
        parser.error("initialization requires an AdapJ experiment profile")
    compensated = spec.required_controller_status.get("takeup_transaction_enabled") is True
    contract = None
    if compensated:
        bounds = (args.acquisition_step_mm, args.acquisition_max_travel_mm, args.calibration_travel_mm)
        if not all(np.isfinite(value) and value > 0 for value in bounds) or bounds[0] > bounds[1]:
            parser.error("initialization travel bounds must be positive; step must not exceed acquisition bound")
        limits = args.limits_file or Path(get_package_share_directory("control_interface"))/"config/catheter_limits.yaml"
        contract = load_hardware_contract(limits, args.catheter)
    rclpy.init(args=ros_args)
    node = SparsePointExperiment(spec)
    completed = False
    try:
        node.wait_for_inputs(60.)
        node.home(0)
        capture(node, True)
        if compensated:
            print(run_compensated_initialization(node, spec, contract,
                step_mm=args.acquisition_step_mm,
                maximum_acquisition_mm=args.acquisition_max_travel_mm,
                calibration_mm=args.calibration_travel_mm))
            completed = True
            return 0
        started = time.monotonic()
        for index, position in enumerate(initialization_positions(spec.home_position)):
            if time.monotonic()-started > 120.:
                raise RuntimeError("AdapJ initialization exceeded 120 s")
            node.spec = replace(spec, home_position=position, decoupled_tendon_prehome=False,
                                home_timeout_s=min(spec.home_timeout_s, 120.-(time.monotonic()-started)))
            node.home(index)
        print(capture(node, False))
        completed = True
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node._publish_mode(ManagerEvent.NONE)
        if not completed:
            client = node.create_client(Trigger, "/catheter_mppi/adapj_cancel_initialization")
            try:
                if client.wait_for_service(timeout_sec=1.):
                    future = client.call_async(Trigger.Request())
                    node._spin_until(future.done, 2.)
            finally:
                node.destroy_client(client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
