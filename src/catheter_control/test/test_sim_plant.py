import numpy as np
import pytest
import torch

from catheter_control.hardware_contract import (
    ENCODER_RADIANS_PER_COUNT, HardwareContract)
from catheter_control.sim_plant import (
    ModelInLoopPlant, SimulatedActuatorPlant, position_servo_velocity)
from catheter_control.sim_perturbations import (
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


def test_position_servo_is_bounded_and_stops_inside_tolerance():
    command = position_servo_velocity(
        current=[0.0, -2.0, 0.01, 0.0, 0.0, 0.0],
        target=[2.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        speed=[1.0, 2.0, 0.5, 0.0, 0.0, 0.0],
        tolerance=[0.1, 0.5, 0.05, 0.1, 0.5, 0.5],
        step_s=0.01)
    assert command == pytest.approx([1.0, 2.0, 0.0, 0.0, 0.0, 0.0])


def test_position_servo_rejects_zero_speed_for_a_moving_axis():
    with pytest.raises(ValueError, match=r"zero speed on axes \[0\]"):
        position_servo_velocity(
            current=np.zeros(6), target=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed=np.zeros(6), tolerance=np.full(6, 0.1), step_s=0.01)


def test_position_servo_drives_quantized_plant_out_and_back(contract):
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01)
    tolerance = np.array([0.1, 0.5, 0.05, 0.1, 0.5, 0.5])
    speed = np.array([5.0, 20.0, 2.0, 0.0, 0.0, 0.0])
    for target_value in (20.0, 0.0):
        target = np.zeros(6)
        target[0] = target_value
        for _ in range(500):
            command = position_servo_velocity(
                plant.joint_position, target, speed, tolerance,
                plant.step_s)
            plant.set_command(command)
            plant.step()
            if np.all(np.abs(target-plant.joint_position) <= tolerance):
                break
        assert plant.joint_position == pytest.approx(target, abs=0.1)


def test_position_servo_uses_actual_delayed_integration_step(contract):
    plant = SimulatedActuatorPlant(
        contract, timestamp_ns=1_000_000_000, step_s=0.01)
    target = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tolerance = np.array([0.1, 0.5, 0.05, 0.1, 0.5, 0.5])
    speed = np.array([5.0, 20.0, 2.0, 0.0, 0.0, 0.0])
    # Exercise nonuniform callback intervals much longer than the nominal
    # 10 ms device period. Passing the real integration interval prevents the
    # final bounded step from overshooting and reversing indefinitely.
    intervals = [0.01, 0.08, 0.035, 0.12]
    for index in range(200):
        step_s = intervals[index % len(intervals)]
        command = position_servo_velocity(
            plant.joint_position, target, speed, tolerance, step_s)
        plant.set_command(command)
        plant.step(plant.timestamp_ns+int(round(step_s*1e9)))
        if np.all(np.abs(target-plant.joint_position) <= tolerance):
            break
    assert np.all(np.abs(target-plant.joint_position) <= tolerance)
