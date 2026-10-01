import numpy as np
import pytest

from catheter_control.tracking import TipForecastMonitor, tip_tracking_error_mm


def test_tip_tracking_error_uses_target_minus_observed_and_mm():
    error, norm = tip_tracking_error_mm(
        [0.012, -0.003, 0.041], [0.010, -0.001, 0.037])

    assert error == pytest.approx([2.0, -2.0, 4.0])
    assert norm == pytest.approx(np.sqrt(24.0))


@pytest.mark.parametrize("value", ([1.0, 2.0], [1.0, np.nan, 3.0]))
def test_tip_tracking_error_rejects_invalid_positions(value):
    with pytest.raises(ValueError):
        tip_tracking_error_mm(value, [0.0, 0.0, 0.0])


def test_forecast_monitor_waits_for_horizon_and_reports_response():
    monitor = TipForecastMonitor()
    monitor.add(
        1_000_000_000, 990_000_000, 0.16,
        [0.010, 0.020, 0.030], [0.012, 0.020, 0.030])

    assert monitor.observe(
        1_159_999_999, [0.0115, 0.020, 0.030]) is None
    result = monitor.observe(
        1_170_000_000, [0.0115, 0.0205, 0.030])

    assert result.predicted_delta_mm == pytest.approx([2.0, 0.0, 0.0])
    assert result.measured_delta_mm == pytest.approx([1.5, 0.5, 0.0])
    assert result.endpoint_error_mm == pytest.approx([-0.5, 0.5, 0.0])
    assert result.endpoint_error_norm_mm == pytest.approx(np.sqrt(0.5))
    assert result.direction_cosine == pytest.approx(3.0/np.sqrt(10.0))
    assert result.start_observation_skew_ms == pytest.approx(10.0)
    assert monitor.pending_count == 0


def test_forecast_monitor_uses_newest_matured_forecast():
    monitor = TipForecastMonitor()
    monitor.add(1_000, 900, 1e-7, [0, 0, 0], [0.001, 0, 0])
    monitor.add(1_100, 1_000, 1e-7, [0, 0, 0], [0, 0.001, 0])

    result = monitor.observe(1_200, [0, 0.0005, 0])

    assert result.start_timestamp_ns == 1_100
    assert result.predicted_delta_mm == pytest.approx([0, 1, 0])


def test_forecast_result_retains_command_state_and_jacobian_metadata():
    monitor = TipForecastMonitor()
    jacobian = np.arange(18, dtype=float)
    pose = np.arange(16, dtype=float)
    monitor.add(
        1_000_000_000, 990_000_000, 0.04,
        [0.01, 0.02, 0.03], [0.011, 0.02, 0.03],
        command_logical_velocity=np.arange(6, dtype=float),
        motor_radians_per_second=np.arange(6, dtype=float) + 10,
        joint_position=np.arange(6, dtype=float) + 20,
        model_motor_angle_rad=[30, 31, 32],
        interface_pose=pose,
        interface_jacobian=jacobian,
        target_tip_m=[0.02, 0.03, 0.04],
        capture_hold=True)

    result = monitor.observe(
        1_050_000_000, [0.0105, 0.0201, 0.03])

    assert result.command_logical_velocity == pytest.approx(np.arange(6))
    assert result.motor_radians_per_second == pytest.approx(
        np.arange(6) + 10)
    assert result.joint_position == pytest.approx(np.arange(6) + 20)
    assert result.model_motor_angle_rad == pytest.approx([30, 31, 32])
    assert result.interface_pose == pytest.approx(pose)
    assert result.interface_jacobian == pytest.approx(jacobian)
    assert result.target_tip_m == pytest.approx([0.02, 0.03, 0.04])
    assert result.start_tip_m == pytest.approx([0.01, 0.02, 0.03])
    assert result.predicted_terminal_tip_m == pytest.approx(
        [0.011, 0.02, 0.03])
    assert result.observed_tip_m == pytest.approx([0.0105, 0.0201, 0.03])
    assert result.capture_hold
