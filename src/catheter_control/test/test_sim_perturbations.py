import numpy as np
import pytest

from catheter_control.backlash import BacklashConfig, BacklashStateEstimator
from catheter_control.sim_perturbations import (
    ActuatorConfig, ActuatorPerturbation, JacobianConfig,
    JacobianPerturbation, MarkerSensorConfig, MarkerSensorModel,
    set_physical_jacobian)


def test_default_marker_sensor_is_identity_and_immediate():
    model = MarkerSensorModel(MarkerSensorConfig(seed=4))
    points = np.arange(12, dtype=float).reshape(4, 3)*1e-3
    assert model.push(100, points)
    packets = model.pop_ready(100)
    assert len(packets) == 1
    assert packets[0].observation_timestamp_ns == 100
    assert packets[0].points_m == pytest.approx(points)


def test_marker_sensor_is_seeded_and_delayed():
    config = MarkerSensorConfig(
        seed=9, noise_std_m=1e-4, latency_s=0.02,
        timestamp_jitter_s=1e-3)
    first = MarkerSensorModel(config)
    second = MarkerSensorModel(config)
    points = np.zeros((4, 3))
    assert first.push(1_000_000_000, points)
    assert second.push(1_000_000_000, points)
    assert first.pop_ready(1_019_999_999) == []
    left = first.pop_ready(1_020_000_000)[0]
    right = second.pop_ready(1_020_000_000)[0]
    assert left.observation_timestamp_ns == right.observation_timestamp_ns
    assert left.points_m == pytest.approx(right.points_m)


def test_marker_dropout_and_outlier_are_counted():
    dropped = MarkerSensorModel(MarkerSensorConfig(
        dropout_probability=1.0))
    assert not dropped.push(1, np.zeros((4, 3)))
    assert dropped.dropped_packets == 1
    outlier = MarkerSensorModel(MarkerSensorConfig(
        outlier_probability=1.0, outlier_magnitude_m=0.01))
    assert outlier.push(1, np.zeros((4, 3)))
    packet = outlier.pop_ready(1)[0]
    assert packet.outlier_injected
    assert np.max(np.linalg.norm(packet.points_m, axis=1)) == pytest.approx(
        0.01)


def test_default_actuator_is_identity_and_gain_is_applied():
    requested = np.arange(1.0, 7.0)
    identity = ActuatorPerturbation(ActuatorConfig(), 0.01)
    assert identity.apply(requested) == pytest.approx(requested)
    half = ActuatorPerturbation(
        ActuatorConfig(gain=(0.5,)*6), 0.01)
    assert half.apply(requested) == pytest.approx(0.5*requested)


def test_actuator_reports_shaft_motion_while_backlash_blocks_transmission():
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_rad=(0.02,)*6,
        initial_backlash_unengaged=True), 0.01)

    shaft, transmitted = actuator.apply_components(np.ones(6))

    assert shaft == pytest.approx(np.ones(6))
    assert transmitted == pytest.approx(np.zeros(6))


def test_virtual_motor_estimate_matches_unengaged_simulated_transmission():
    positive = (0.20, 0.30, 0.40)
    negative = (0.10, 0.15, 0.20)
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=positive+(0.0,)*3,
        reversal_backlash_negative_rad=negative+(0.0,)*3,
        initial_backlash_unengaged=True), 0.01)
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(0.0,)*3,
        width_positive_rad=positive,
        width_negative_rad=negative,
        minimum_motor_increment_rad=1e-4))
    shaft_angle = np.zeros(6)
    transmitted_angle = np.zeros(6)
    estimator.advance_motor(shaft_angle[:3])

    for requested in (
            np.array([2.0, 3.0, 4.0, 0, 0, 0]),)*20:
        shaft_rate, transmitted_rate = actuator.apply_components(requested)
        shaft_angle += 0.01*shaft_rate
        transmitted_angle += 0.01*transmitted_rate
        effective = estimator.advance_motor(shaft_angle[:3])
        np.testing.assert_allclose(
            effective, transmitted_angle[:3], atol=1e-12)


def test_actuator_delay_lag_deadband_and_reversal_backlash():
    delayed = ActuatorPerturbation(
        ActuatorConfig(command_delay_s=0.02), 0.01)
    request = np.ones(6)
    assert delayed.apply(request) == pytest.approx(np.zeros(6))
    assert delayed.apply(request) == pytest.approx(np.zeros(6))
    assert delayed.apply(request) == pytest.approx(request)

    lagged = ActuatorPerturbation(
        ActuatorConfig(time_constant_s=(0.1,)*6), 0.01)
    output = lagged.apply(request)
    assert np.all(output > 0.0)
    assert np.all(output < request)

    dead = ActuatorPerturbation(
        ActuatorConfig(deadband_rad_s=(2.0,)*6), 0.01)
    assert dead.apply(request) == pytest.approx(np.zeros(6))

    backlash = ActuatorPerturbation(
        ActuatorConfig(reversal_backlash_rad=(0.02,)*6), 0.01)
    assert backlash.apply(request) == pytest.approx(request)
    assert backlash.apply(-request) == pytest.approx(np.zeros(6))
    assert backlash.apply(-request) == pytest.approx(np.zeros(6))
    assert backlash.apply(-request) == pytest.approx(-request)

    asymmetric = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.01,)*6,
        reversal_backlash_negative_rad=(0.03,)*6), 0.01)
    assert asymmetric.apply(request) == pytest.approx(request)
    assert asymmetric.apply(-request) == pytest.approx(np.zeros(6))
    assert asymmetric.apply(-request) == pytest.approx(np.zeros(6))
    assert asymmetric.apply(-request) == pytest.approx(np.zeros(6))
    assert asymmetric.apply(-request) == pytest.approx(-request)
    assert asymmetric.apply(request) == pytest.approx(np.zeros(6))
    assert asymmetric.apply(request) == pytest.approx(request)

    initially_unengaged = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.02,)*6,
        reversal_backlash_negative_rad=(0.01,)*6,
        initial_backlash_unengaged=True), 0.01)
    assert initially_unengaged.apply(request) == pytest.approx(np.zeros(6))
    assert initially_unengaged.apply(request) == pytest.approx(np.zeros(6))
    assert initially_unengaged.apply(request) == pytest.approx(request)


def test_actuator_halt_preserves_same_direction_engagement():
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.02,)*6,
        reversal_backlash_negative_rad=(0.03,)*6,
        initial_backlash_unengaged=True), 0.01)
    request = np.ones(6)
    assert actuator.apply(request) == pytest.approx(np.zeros(6))
    assert actuator.apply(request) == pytest.approx(np.zeros(6))
    assert actuator.apply(request) == pytest.approx(request)

    actuator.halt()
    assert actuator.apply(np.zeros(6)) == pytest.approx(np.zeros(6))
    assert actuator.apply(request) == pytest.approx(request)


def test_actuator_partial_reversal_unwinds_without_full_new_gap():
    actuator = ActuatorPerturbation(ActuatorConfig(
        reversal_backlash_positive_rad=(0.02,)*6,
        reversal_backlash_negative_rad=(0.03,)*6), 0.01)
    positive = np.ones(6)
    negative = -positive
    assert actuator.apply(positive) == pytest.approx(positive)

    # Move 0.02 rad into a 0.03 rad negative gap, then return toward the
    # still-engaged positive side. Only that partial 0.02 rad is unwound.
    assert actuator.apply(negative) == pytest.approx(np.zeros(6))
    assert actuator.apply(negative) == pytest.approx(np.zeros(6))
    assert actuator.apply(positive) == pytest.approx(np.zeros(6))
    assert actuator.apply(positive) == pytest.approx(np.zeros(6))
    assert actuator.apply(positive) == pytest.approx(positive)


def test_jacobian_blocks_and_normalized_state_are_consistent():
    original = np.arange(1.0, 19.0).reshape(6, 3)
    perturbation = JacobianPerturbation(JacobianConfig(
        angular_column_gain=(0.5, 1.0, 2.0),
        linear_column_gain=(1.0, 0.0, -1.0)))
    changed = perturbation.apply(original)
    assert changed[:3, 0] == pytest.approx(0.5*original[:3, 0])
    assert changed[3:, 1] == pytest.approx(np.zeros(3))
    assert changed[3:, 2] == pytest.approx(-original[3:, 2])

    class Model:
        action_scale = np.array([2.0, 3.0, 4.0])
        state_scale = np.arange(1.0, 7.0)
        J_normalized = np.zeros((6, 3))

        @property
        def jacobian(self):
            return (self.state_scale[:, None]*self.J_normalized
                    / self.action_scale[None, :])

    model = set_physical_jacobian(Model(), changed)
    assert model.jacobian == pytest.approx(changed)


@pytest.mark.parametrize("config", [
    MarkerSensorConfig(noise_std_m=-1.0),
    MarkerSensorConfig(dropout_probability=1.1),
])
def test_invalid_sensor_configuration_fails_closed(config):
    with pytest.raises(ValueError):
        MarkerSensorModel(config)
