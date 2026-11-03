import numpy as np
import pytest
import torch

from pathlib import Path

from catheter_control.safety.hardware_contract import (
    ENCODER_DEGREES_PER_COUNT, ENCODER_RADIANS_PER_COUNT, HardwareContract,
    MOTOR_AXIS_UNITS_PER_ENCODER_COUNT, load_hardware_contract)
from simulation.sim_plant import (
    FirmwarePositionTracker, ModelInLoopPlant, POSITION_MAX_CORRECTIONS,
    POSITION_RETRY_IDLE_S, POSITION_TIMEOUT_S, SimulatedActuatorPlant,
    firmware_position_move, firmware_position_tolerance)
from simulation.sim_perturbations import (
    ActuatorConfig, ActuatorPerturbation)


class FakeRuntime:
    def __init__(self):
        self.calls = []
        self.counts = np.zeros(6)

    def initialize(self, timestamp_ns, counts):
        self.calls.append(("initialize", int(timestamp_ns)))
        self.counts = np.asarray(counts, dtype=float).copy()

    def advance_encoder(self, timestamp_ns, counts):
        self.calls.append(("advance", int(timestamp_ns)))
        self.counts = np.asarray(counts, dtype=float).copy()

    def current_markers(self):
        displacement = self.counts[:3] * 1e-7
        points = np.zeros((4, 3), dtype=np.float32)
        points[:, 2] = np.linspace(0.0, 0.06, 4)
        points[-1] += displacement
        return torch.as_tensor(points)


LIMITS = (Path(__file__).resolve().parents[2]
          /"control_interface/config/catheter_limits.yaml")


def imricor_plant(logical_position):
    """Plant on the deployed contract, started at a whole-count pose."""
    real = load_hardware_contract(str(LIMITS), "imricor_test")
    counts = np.rint(real.logical_position_to_motor_axis_position(
        np.asarray(logical_position, dtype=float))
        / MOTOR_AXIS_UNITS_PER_ENCODER_COUNT)
    return real, SimulatedActuatorPlant(
        real, timestamp_ns=1_000_000_000, step_s=0.01,
        initial_encoder_counts=counts)


@pytest.fixture
def contract():
    return HardwareContract(
        position_lower=np.array([-100.0] * 6),
        position_upper=np.array([100.0] * 6),
        velocity_min=np.zeros(6),
        velocity_max=np.array([20.0, 60.0, 10.0, 20.0, 60.0, 60.0]),
        model_encoder_count_lower=np.array([-200000.0] * 3),
        model_encoder_count_upper=np.array([200000.0] * 3),
    )


def test_zero_command_holds_counts_and_advances_time(contract):
    runtime = FakeRuntime()
    plant = ModelInLoopPlant(
        runtime, contract, timestamp_ns=1_000_000_000, step_s=0.01)
    snapshot = plant.step()
    assert snapshot.timestamp_ns == 1_010_000_000
    assert snapshot.encoder_counts == pytest.approx(np.zeros(6))
    assert snapshot.joint_position == pytest.approx(np.zeros(6))
    assert snapshot.tip_base_m == pytest.approx([0.0, 0.0, 0.06])


def test_native_driver_stops_at_endpoint_independent_of_coarse_sample(contract):
    plant = SimulatedActuatorPlant(contract, timestamp_ns=1000000000, step_s=.1)
    plant.set_native_position([100, 0, 0, 0, 0, 0], [36, 0, 0, 0, 0, 0])
    plant.step()
    assert plant.encoder_counts[0] == 100
    plant.step()
    assert plant.encoder_counts[0] == 100
    assert plant.last_projection.realized_logical_velocity[0] == pytest.approx(0.)


def test_subcount_motion_accumulates_and_feedback_is_count_derived(contract):
    runtime = FakeRuntime()
    plant = ModelInLoopPlant(
        runtime, contract, timestamp_ns=1_000_000_000, step_s=0.01)
    plant.set_command([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    snapshots = [plant.step() for _ in range(10)]
    assert snapshots[-1].encoder_counts[0] > 0
    assert snapshots[-1].joint_position == pytest.approx(
        contract.encoder_counts_to_logical_position(
            snapshots[-1].encoder_counts))
    assert runtime.calls[-1][0] == "advance"


def test_stop_clears_held_command(contract):
    runtime = FakeRuntime()
    plant = ModelInLoopPlant(
        runtime, contract, timestamp_ns=1_000_000_000, step_s=0.01)
    plant.set_command([5.0, 20.0, 2.0, 0.0, 0.0, 0.0])
    plant.step()
    plant.stop()
    before = plant.motor_angle_rad.copy()
    snapshot = plant.step()
    assert snapshot.requested_logical_velocity == pytest.approx(np.zeros(6))
    assert plant.motor_angle_rad == pytest.approx(before)


def test_stop_preserves_actuator_backlash_engagement(contract):
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.02,)*6,
        reversal_backlash_negative_rad=(0.03,)*6,
        initial_backlash_unengaged=True), 0.01)
    plant = ModelInLoopPlant(
        FakeRuntime(), contract, timestamp_ns=1_000_000_000, step_s=0.01,
        actuator_perturbation=actuator)
    command = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    plant.set_command(command)
    for _ in range(20):
        plant.step()
    plant.stop()
    before = plant.motor_angle_rad.copy()
    plant.step()
    plant.set_command(command)
    after = plant.step()
    assert after.motor_angle_rad[0] > before[0]


def test_fast_feedback_step_can_defer_expensive_model_update(contract):
    runtime = FakeRuntime()
    plant = ModelInLoopPlant(
        runtime, contract, timestamp_ns=1_000_000_000, step_s=0.01)
    initial_calls = len(runtime.calls)
    first = plant.step(advance_model=False)
    assert len(runtime.calls) == initial_calls
    second = plant.step(advance_model=True)
    assert len(runtime.calls) == initial_calls+1
    assert second.timestamp_ns > first.timestamp_ns


def test_malformed_input_and_nonmonotonic_time_fail_closed(contract):
    plant = ModelInLoopPlant(
        FakeRuntime(), contract, timestamp_ns=1_000_000_000)
    with pytest.raises(ValueError, match="six finite"):
        plant.set_command([1.0] * 5)
    with pytest.raises(ValueError, match="strictly increasing"):
        plant.step(plant.timestamp_ns)


def test_actuator_feedback_is_integer_count_and_position_consistent(contract):
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01)
    plant.set_command([2.0, 4.0, 1.0, 0.0, 0.0, 0.0])
    snapshot = None
    for _ in range(20):
        snapshot = plant.step()
    assert snapshot is not None
    assert np.all(snapshot.encoder_counts == np.rint(snapshot.encoder_counts))
    assert snapshot.joint_position == pytest.approx(
        contract.encoder_counts_to_logical_position(
            snapshot.encoder_counts))


def test_backlash_is_downstream_of_simulated_encoder(contract):
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.2,)*6,
        reversal_backlash_negative_rad=(0.2,)*6,
        initial_backlash_unengaged=True), 0.01)
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01,
        actuator_perturbation=actuator)
    plant.set_command([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    snapshot = plant.step()

    assert snapshot.encoder_counts[0] != 0.0
    assert snapshot.transmitted_encoder_counts[0] == 0.0


def test_simulation_transmission_reset_preserves_shaft_and_clears_memory(
        contract):
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.2,)*6,
        reversal_backlash_negative_rad=(0.2,)*6,
        initial_backlash_unengaged=True), 0.01)
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01,
        actuator_perturbation=actuator)
    plant.set_command([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(20):
        plant.step()
    shaft_before = plant.motor_angle_rad.copy()
    assert not np.allclose(
        plant.transmitted_motor_angle_rad, shaft_before)

    plant.reset_transmission()

    assert plant.motor_angle_rad == pytest.approx(shaft_before)
    assert plant.transmitted_motor_angle_rad == pytest.approx(shaft_before)
    assert plant.requested_velocity == pytest.approx(np.zeros(6))
    plant.set_command([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    snapshot = plant.step()
    assert snapshot.transmitted_encoder_counts == pytest.approx(
        np.rint(shaft_before/ENCODER_RADIANS_PER_COUNT))


def test_firmware_position_tolerance_is_three_motor_degrees_per_axis():
    # |jointPRate| * 3 deg: linear 67.319841/20 mm/rev, rotation 0.375/5,
    # bend 1.190625 mm/rev.
    linear, rotation, bend = 67.319841/20*3/360, .375/5*3, 1.190625*3/360
    assert firmware_position_tolerance(3.0) == pytest.approx(
        [linear, rotation, bend, linear, rotation, rotation], rel=1e-12)


def firmware_position_error(counts, target):
    """Motor-axis residual, the quantity the firmware tracker compares."""
    return np.abs(
        HardwareContract.logical_position_to_motor_axis_position(target)
        - HardwareContract.encoder_counts_to_motor_axis_position(counts))


def run_position_move(plant, target, speed, *, steps=2000, intervals=(0.01,)):
    """Drive one transaction as the simulated device does; return status."""
    target = np.asarray(target, dtype=float)
    endpoint, rpm = firmware_position_move(plant.encoder_counts, target, speed)
    plant.set_native_position(endpoint, rpm)
    tracker = FirmwarePositionTracker()
    tracker.begin(plant.timestamp_ns*1e-9, rpm != 0.0,
                  HardwareContract.logical_position_to_motor_axis_position(target),
                  firmware_position_tolerance(3.0),
                  HardwareContract.encoder_counts_to_motor_axis_position(
                      plant.encoder_counts))
    for index in range(steps):
        step_s = intervals[index % len(intervals)]
        plant.step(plant.timestamp_ns+int(round(step_s*1e9)))
        update = tracker.update(
            plant.timestamp_ns*1e-9,
            HardwareContract.encoder_counts_to_motor_axis_position(
                plant.encoder_counts))
        assert not np.any(update.retry_axes)
        if update.status is not None:
            plant.stop()
            return update.status
    raise AssertionError("position move did not finish")


def test_position_move_uses_whole_degrees_firmware_rpm_and_decoupling():
    # Insertion 1 mm with bend 0.1 mm: motor 0 moves insertion - bend.
    endpoint, rpm = firmware_position_move(
        np.zeros(6), [1.0, 0.0, 0.1, 0.0, 0.0, 0.0],
        [5.0, 20.0, 2.25, 0.0, 0.0, 0.0])
    linear_deg, bend_deg = 67.319841/20/360, -1.190625/360
    expected_deg = np.array([
        np.floor(0.9/linear_deg+0.5), 0.0, np.floor(0.1/abs(bend_deg)+0.5),
        0.0, 0.0, 0.0])
    np.testing.assert_allclose(np.abs(endpoint)*ENCODER_DEGREES_PER_COUNT,
                               expected_deg)
    assert np.sign(endpoint[2]) == -1.0  # negative jointPRate on bend
    # round(5/(3.366/60)) = 89, round(2.25/(1.190625/60)) = 113.
    np.testing.assert_array_equal(rpm, [89.0, 0.0, -113.0, 0.0, 0.0, 0.0])


def test_position_move_rejects_zero_rpm_for_a_moving_axis():
    with pytest.raises(ValueError, match=r"zero speed on axes \[0\]"):
        firmware_position_move(
            np.zeros(6), [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], np.zeros(6))


def test_position_move_reaches_bend_lower_limit_unlike_velocity_projection():
    # 2026-11-02 simulated return: bend 0.0862 mm, target 0 (the imricor_test
    # lower limit). The VEL projection refuses motion toward a limit within
    # velocity_min * guard horizon (0.1 mm); the firmware POS path does not.
    real, plant = imricor_plant([20.03, 0.0, 0.0862, 0.0, 0.0, 0.0])
    target = np.array([19.972, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert real.project_velocity(
        np.array([0.0, 0.0, -2.0, 0, 0, 0]),
        plant.joint_position).logical_velocity[2] == 0.0
    assert run_position_move(
        plant, target, [5.0, 20.0, 2.25, 0.0, 0.0, 0.0]) == "complete"
    assert abs(plant.joint_position[2]) < 0.05
    assert abs(plant.joint_position[0]-target[0]) < 0.05


def test_position_move_drives_quantized_plant_out_and_back(contract):
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01)
    speed = np.array([5.0, 20.0, 2.0, 0.0, 0.0, 0.0])
    for target in ([20.04, 0.0, 0.14, 0.0, 0.0, 0.0], np.zeros(6)):
        assert run_position_move(plant, target, speed) == "complete"
        # Whole-degree moves land within half a motor degree.
        assert np.all(firmware_position_error(plant.encoder_counts, target)
                      <= firmware_position_tolerance(0.5)+1e-12)


def test_position_move_endpoint_holds_under_long_callback_intervals(contract):
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01)
    target = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert run_position_move(
        plant, target, [5.0, 20.0, 2.0, 0.0, 0.0, 0.0],
        intervals=(0.01, 0.08, 0.035, 0.12)) == "complete"
    reached = plant.encoder_counts.copy()
    for _ in range(5):
        plant.step(plant.timestamp_ns+120_000_000)
    np.testing.assert_array_equal(plant.encoder_counts, reached)


def test_shaft_rpm_ceiling_allows_firmware_rounding_of_the_speed_limit():
    # imricor_test tendon maximum 4.5 mm/s rounds to 227 rpm (4.504 mm/s),
    # which the firmware runs; 228 rpm is above any rounded legal speed.
    _, plant = imricor_plant([20.0, 0.0, 6.0, 0.0, 0.0, 0.0])
    _, rpm = firmware_position_move(
        plant.encoder_counts, [20.0, 0.0, 7.0, 0.0, 0.0, 0.0],
        [5.0, 0.0, 4.5, 0.0, 0.0, 0.0])
    assert abs(rpm[2]) == 227.0
    plant.set_native_position(plant.encoder_counts-1000.0, rpm)
    plant.step()
    rpm[2] = np.sign(rpm[2])*228.0
    plant.set_shaft_rpm(rpm)
    with pytest.raises(RuntimeError, match="ceiling"):
        plant.step()


def tracker_for(active, target, start=0.0):
    tracker = FirmwarePositionTracker()
    tolerance = np.full(6, 0.01)
    tracker.begin(start, active, target, tolerance, np.zeros(6))
    return tracker


def test_tracker_stops_each_axis_after_its_own_settle():
    active = np.array([True, False, True, False, False, False])
    tracker = tracker_for(active, [1.0, 0, 2.0, 0, 0, 0])
    first = tracker.update(0.05, [1.0, 0, 1.0, 0, 0, 0])
    assert first.status is None and not np.any(first.stop_axes)
    second = tracker.update(0.16, [1.0, 0, 2.0, 0, 0, 0])
    np.testing.assert_array_equal(second.stop_axes, [1, 0, 0, 0, 0, 0])
    assert second.status is None
    third = tracker.update(0.27, [1.0, 0, 2.0, 0, 0, 0])
    np.testing.assert_array_equal(third.stop_axes, [0, 0, 1, 0, 0, 0])
    assert third.status == "complete" and not tracker.running


def test_tracker_with_no_active_axis_completes_at_once():
    tracker = tracker_for(np.zeros(6, dtype=bool), np.zeros(6))
    assert tracker.update(0.01, np.zeros(6)).status == "complete"


def test_tracker_retries_idle_axis_with_progress_then_times_out():
    active = np.array([True, False, False, False, False, False])
    tracker = tracker_for(active, [1.0, 0, 0, 0, 0, 0])
    # Moved halfway, then idle: one correction per idle period.
    tracker.update(0.1, [0.5, 0, 0, 0, 0, 0])
    retry = tracker.update(0.1+POSITION_RETRY_IDLE_S, [0.5, 0, 0, 0, 0, 0])
    assert retry.retry_axes[0] and retry.status is None
    # No further progress: the next idle period times the transaction out.
    late = tracker.update(0.1+2*POSITION_RETRY_IDLE_S, [0.5, 0, 0, 0, 0, 0])
    assert late.status == "timed_out" and late.stop_axes[0]


def test_tracker_corrections_are_bounded():
    active = np.array([True, False, False, False, False, False])
    tracker = tracker_for(active, [10.0, 0, 0, 0, 0, 0])
    now, position = 0.0, 0.0
    for _ in range(POSITION_MAX_CORRECTIONS):
        position += 1.0
        tracker.update(now+0.01, [position, 0, 0, 0, 0, 0])
        now += 0.01+POSITION_RETRY_IDLE_S
        assert tracker.update(now, [position, 0, 0, 0, 0, 0]).retry_axes[0]
    position += 1.0
    tracker.update(now+0.01, [position, 0, 0, 0, 0, 0])
    now += 0.01+POSITION_RETRY_IDLE_S
    assert tracker.update(now, [position, 0, 0, 0, 0, 0]).status == "timed_out"


def test_tracker_hard_timeout():
    active = np.array([True, False, False, False, False, False])
    tracker = tracker_for(active, [1.0, 0, 0, 0, 0, 0])
    # Dithering by more than one tolerance is motion, so it is never idle,
    # but it never reaches the target either.
    for step in range(1, int(POSITION_TIMEOUT_S/0.05)):
        position = 0.5+0.02*(step % 2)
        assert tracker.update(step*0.05, [position, 0, 0, 0, 0, 0]).status is None
    assert tracker.update(
        POSITION_TIMEOUT_S, [0.5, 0, 0, 0, 0, 0]).status == "timed_out"
