from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest

from catheter_control.safety.hardware_contract import (
    ENCODER_RADIANS_PER_COUNT,
    HardwareContract,
    MOTOR_AXIS_UNITS_PER_ENCODER_COUNT,
    MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM,
    load_hardware_contract,
)


ROOT = Path(__file__).resolve().parents[2]
LIMITS = ROOT / "control_interface" / "config" / "catheter_limits.yaml"


@pytest.fixture
def contract():
    return load_hardware_contract(LIMITS, "imricor_test")


def test_loads_the_same_limits_as_the_production_manager(contract):
    assert np.allclose(contract.position_lower, [-10, -270, 0, 0, -180, -360])
    assert np.allclose(contract.position_upper, [50, 270, 15, 80, 180, 360])
    assert np.allclose(contract.velocity_min, [2, 7, 2, 0, 0, 0])
    assert np.allclose(contract.velocity_max, [10, 40, 4.5, 4, 25, 25])
    assert np.allclose(contract.control_position_margin, [1, 0, 0, 0, 0, 0])
    assert np.allclose(contract.control_position_lower, [-9, -270, 0, 0, -180, -360])
    assert np.allclose(contract.control_position_upper, [49, 270, 15, 80, 180, 360])
    assert np.allclose(
        contract.feedback_limit_tolerance,
        [0.001, 0.01, 0.002, 0.001, 0.01, 0.01])
    assert np.allclose(contract.model_encoder_count_margin, [1000]*3)


def test_encoder_counts_convert_directly_to_motor_shaft_radians(contract):
    counts = np.array([8000, -8000, 1, 0, 100, -100])
    radians = contract.encoder_counts_to_motor_radians(counts)
    assert radians[0] == pytest.approx(2 * np.pi)
    assert radians[1] == pytest.approx(-2 * np.pi)
    assert radians[2] == pytest.approx(ENCODER_RADIANS_PER_COUNT)


def test_encoder_counts_reconstruct_firmware_position_feedback(contract):
    counts = np.array([-2, 2, -5, 0, 0, 0], dtype=float)
    motor = counts * MOTOR_AXIS_UNITS_PER_ENCODER_COUNT
    expected = motor.copy()
    expected[0] += expected[2]
    expected[5] -= expected[4]
    assert contract.encoder_counts_to_motor_axis_position(
        counts) == pytest.approx(motor)
    assert contract.encoder_counts_to_logical_position(
        counts) == pytest.approx(expected)


def test_feedback_validity_envelopes_fail_closed(contract):
    assert contract.position_is_valid([20, 0, 7, 40, 0, 0])
    assert not contract.position_is_valid([500000, 0, 7, 40, 0, 0])
    measured = [19.99829864501953, 0.0, -0.0007441406487487257,
                0.0, 0.0, 0.0]
    assert not contract.position_is_valid(measured)
    assert contract.feedback_position_is_valid(measured)
    assert not contract.feedback_position_is_valid(
        [20.0, 0.0, -0.0021, 0.0, 0.0, 0.0])
    assert contract.model_encoder_counts_are_valid(
        [-35806, -79999, -100404, 0, 0, 0])
    assert not contract.model_encoder_counts_are_valid(
        [1197348480, -1032295180, -24, 0, 0, 0])


def test_projection_matches_manager_speed_floor_and_position_guard(contract):
    position = np.array([20, 269.95, 7, 40, 0, 0], dtype=float)
    requested = np.array([0.5, 20, -99, 10, 30, -30], dtype=float)
    projected = contract.project_logical_velocity(requested, position)
    assert projected == pytest.approx([2, 0, -4.5, 4, 25, -25])


def test_controller_local_contract_can_disable_one_axis(contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)

    projected = local.project_logical_velocity(
        [5, 40, 3, 0, 0, 0], [20, 0, 7, 40, 0, 0])

    assert projected == pytest.approx([5, 0, 3, 0, 0, 0])


def test_projection_only_allows_motion_back_inside_limits(contract):
    position = np.array([-11, 271, -1, 81, 181, 361], dtype=float)
    outward = np.array([-3, 10, -2, 1, 1, 1], dtype=float)
    inward = -outward
    assert contract.project_logical_velocity(
        outward, position) == pytest.approx([0, 0, 0, 0, 0, 0])
    assert contract.project_logical_velocity(
        inward, position) == pytest.approx([3, -10, 2, -1, -1, -1])


def test_autonomous_projection_preserves_insertion_reserve(contract):
    assert contract.position_is_valid([49.5, 0, 7, 40, 0, 0])
    assert contract.project_logical_velocity(
        [5, 0, 0, 0, 0, 0], [49.5, 0, 7, 40, 0, 0]
    ) == pytest.approx(np.zeros(6))
    assert contract.project_logical_velocity(
        [-5, 0, 0, 0, 0, 0], [49.5, 0, 7, 40, 0, 0]
    ) == pytest.approx([-5, 0, 0, 0, 0, 0])


def test_logical_to_motor_coupling_matches_firmware(contract):
    logical = np.array([5, 20, 3, 4, 10, -2], dtype=float)
    motor = contract.logical_to_motor_axis_velocity(logical)
    assert motor == pytest.approx([2, 20, 3, 4, 10, 8])
    assert contract.motor_axis_to_logical_velocity(motor) == pytest.approx(
        logical)


def test_firmware_rpm_quantization_and_sign_are_reproduced(contract):
    requested_rpm = np.array([0.49, -0.5, 1.5, -3.2, 249.6, -300.0])
    motor_velocity = requested_rpm * MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM
    rpm, radians_per_second, realized = contract.quantize_motor_velocity(
        motor_velocity)
    assert rpm.tolist() == [0, 1, 2, 3, 250, 250]
    signed_rpm = np.array([0, -1, 2, -3, 250, -250])
    assert radians_per_second == pytest.approx(
        signed_rpm * 2 * np.pi / 60)
    assert realized == pytest.approx(
        signed_rpm * MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM)


def test_full_projection_returns_publish_command_and_model_rate(contract):
    result = contract.project_velocity(
        [5, 20, 3, 0, 0, 0], [20, 0, 7, 40, 0, 0])
    assert result.logical_velocity == pytest.approx([5, 20, 3, 0, 0, 0])
    assert result.requested_motor_axis_velocity == pytest.approx(
        [2, 20, 3, 0, 0, 0])
    assert result.motor_radians_per_second.shape == (6,)
    expected_logical = contract.motor_axis_to_logical_velocity(
        result.realized_motor_axis_velocity)
    assert result.realized_logical_velocity == pytest.approx(expected_logical)


def test_projection_stops_at_coupled_model_encoder_envelope(contract):
    counts = np.array([109900, -1000, -574, 0, 0, 0], dtype=float)
    position = contract.encoder_counts_to_logical_position(counts)
    outward = contract.project_velocity(
        [10, 0, 0, 0, 0, 0], position)
    count_rate = (
        outward.realized_motor_axis_velocity[:3]
        / MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[:3])
    future_counts = counts[:3] + count_rate*contract.position_guard_horizon_s
    assert np.all(future_counts <= contract.model_encoder_count_upper)
    assert np.all(future_counts >= contract.model_encoder_count_lower)
    assert outward.logical_velocity[0] < 10.0

    inward = contract.project_velocity(
        [-10, 0, 0, 0, 0, 0], position)
    assert inward.logical_velocity[0] == pytest.approx(-10.0)


def test_publishable_projection_holds_instead_of_emitting_subfloor_velocity(
        contract):
    counts = np.array([108900, 0, 0, 0, 0, 0], dtype=float)
    position = contract.encoder_counts_to_logical_position(counts)

    guarded = contract.project_velocity(
        [10, 0, 0, 0, 0, 0], position).logical_velocity
    publishable = contract.project_publishable_velocity(
        [10, 0, 0, 0, 0, 0], position)

    assert 0.0 < guarded[0] < contract.velocity_min[0]
    assert publishable[0] == 0.0


def test_publishable_projection_preserves_coupled_floor_at_roundoff_boundary(
        contract):
    # Recorded point-2 take-up used equal insertion/bend logical speeds to
    # hold physical chassis shaft 0 while moving tendon shaft 2. Coupling
    # returned the 2.0 minimum one ulp low; that must not become a zero hold.
    position = np.array([19.69021224975586, 0, 0, 0, 0, 0], dtype=float)
    request = np.array([
        np.nextafter(2.0, 0.0), 0.0, np.nextafter(2.0, 0.0), 0, 0, 0])

    publishable = contract.project_publishable_velocity(request, position)

    np.testing.assert_allclose(publishable, [2.0, 0.0, 2.0, 0.0, 0.0, 0.0])
    motor = contract.logical_to_motor_axis_velocity(publishable)
    assert motor[0] == pytest.approx(0.0)
    assert motor[2] > 0.0


def test_model_encoder_reserve_never_synthesizes_recovery_motion(contract):
    counts = np.array([109792, 6, -6379, 0, 0, 0], dtype=float)
    position = contract.encoder_counts_to_logical_position(counts)

    hold = contract.project_publishable_velocity(np.zeros(6), position)
    outward = contract.project_publishable_velocity(
        [10, 0, 4.5, 0, 0, 0], position)
    inward = contract.project_publishable_velocity(
        [-10, 0, 0, 0, 0, 0], position)

    np.testing.assert_allclose(hold, np.zeros(6))
    # Logical insertion equal to bend means coupled motor shaft 0 is held.
    assert outward[0] == pytest.approx(outward[2])
    assert inward[0] < 0.0


def test_batched_projection_obeys_model_encoder_envelope(contract):
    counts = np.array([109900, -1000, -574, 0, 0, 0], dtype=float)
    position = contract.encoder_counts_to_logical_position(counts)
    result = contract.project_velocity_batch(
        np.asarray([[10, 0, 0, 0, 0, 0],
                    [-10, 0, 0, 0, 0, 0]], dtype=float),
        np.broadcast_to(position, (2, 6)))
    count_rate = (
        result.realized_motor_axis_velocity[:, :3]
        / MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[None, :3])
    future = counts[None, :3] + count_rate*contract.position_guard_horizon_s
    assert np.all(future <= contract.model_encoder_count_upper[None])
    assert np.all(future >= contract.model_encoder_count_lower[None])
    assert result.logical_velocity[0, 0] < 10.0
    assert result.logical_velocity[1, 0] == pytest.approx(-10.0)


def test_motor_angle_integration_requires_positive_finite_dt(contract):
    angle = np.arange(6, dtype=float)
    rate = np.ones(6)
    assert contract.integrate_motor_angles(angle, rate, 0.02) == pytest.approx(
        angle + 0.02)
    assert contract.integrate_joint_positions(
        angle, rate, 0.02) == pytest.approx(angle + 0.02)
    for invalid in (0.0, -0.1, np.nan):
        with pytest.raises(ValueError, match="strictly positive"):
            contract.integrate_motor_angles(angle, rate, invalid)
        with pytest.raises(ValueError, match="strictly positive"):
            contract.integrate_joint_positions(angle, rate, invalid)


def test_contract_rejects_nonfinite_and_malformed_inputs(contract):
    with pytest.raises(ValueError, match="six finite"):
        contract.encoder_counts_to_motor_radians([1, 2, 3])
    with pytest.raises(ValueError, match="six finite"):
        contract.project_logical_velocity([0] * 5 + [np.nan], [0] * 6)


def test_contract_has_no_encoder_zero_mutation_api():
    forbidden = {"set_zero", "zero_encoders", "encoder_offset"}
    assert forbidden.isdisjoint(dir(HardwareContract))
