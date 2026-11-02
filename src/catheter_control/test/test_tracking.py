import numpy as np
import pytest

from catheter_control.planning.tracking import TipForecastMonitor, tip_tracking_error_mm


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

    monitor.note_command(990_000_000, np.zeros(6))
    # Exact root sample avoids attributing earlier motion to this forecast.
    assert monitor.observe(1_000_000_000, [0.010, 0.020, 0.030]) is None
    for t in (1_050_000_000, 1_100_000_000, 1_150_000_000, 1_170_000_000):
        monitor.note_command(t, np.zeros(6))
    assert monitor.observe(1_150_000_000, [0.011, 0.020, 0.030]) is None
    result = monitor.observe(
        1_170_000_000, [0.0115, 0.0205, 0.030])

    assert result.predicted_delta_mm == pytest.approx([2.0, 0.0, 0.0])
    assert result.measured_delta_mm == pytest.approx([1.25, 0.25, 0.0])
    assert result.endpoint_error_mm == pytest.approx([-0.75, 0.25, 0.0])
    assert result.endpoint_error_norm_mm == pytest.approx(np.sqrt(0.625))
    assert not result.comparison_valid  # missing forecast command metadata
    assert result.direction_cosine is None
    assert result.start_observation_skew_ms == 0
    assert result.observation_timestamp_ns == 1_160_000_000
    assert result.source_observation_timestamp_ns == 1_170_000_000
    assert monitor.pending_count == 0


def test_forecast_monitor_uses_newest_matured_forecast():
    monitor = TipForecastMonitor()
    monitor.add(1_000, 900, 1e-7, [0, 0, 0], [0.001, 0, 0])
    monitor.add(1_100, 1_000, 1e-7, [0, 0, 0], [0, 0.001, 0])

    result = monitor.observe(1_200, [0, 0.0005, 0])

    assert result.start_timestamp_ns == 1_100
    assert result.due_timestamp_ns == 1_200


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
    assert result.start_tip_m == pytest.approx(
        [0.010083333333333333, 0.020016666666666667, 0.03])
    assert result.predicted_terminal_tip_m == pytest.approx(
        [0.011, 0.02, 0.03])
    assert result.observed_tip_m == pytest.approx(
        [0.010416666666666667, 0.020083333333333333, 0.03])
    assert result.capture_hold


def test_aligned_response_excludes_previous_motion_and_endpoint_overshoot():
    monitor = TipForecastMonitor()
    command = np.ones(6)
    monitor.add(1_000_000_000, 950_000_000, .2,
                [0, 0, 0], [.003, 0, 0],
                command_logical_velocity=command,
                model_start_tip_m=[.001, 0, 0])
    for t in range(950_000_000, 1_251_000_000, 50_000_000):
        monitor.note_command(t, command)
    for t, x in [(1_000_000_000, .001), (1_100_000_000, .002),
                 (1_150_000_000, .0025)]:
        assert monitor.observe(t, [x, 0, 0]) is None
    result = monitor.observe(1_250_000_000, [.0035, 0, 0])
    assert result.comparison_valid
    assert result.predicted_delta_mm == pytest.approx([2, 0, 0])
    assert result.measured_delta_mm == pytest.approx([2, 0, 0])
    assert result.endpoint_error_norm_mm == pytest.approx(0)
    assert result.direction_cosine == pytest.approx(1)


@pytest.mark.parametrize("changed", [False, True])
def test_interrupted_or_missing_command_is_not_direction_evidence(changed):
    monitor = TipForecastMonitor()
    command = np.ones(6)
    monitor.add(1_000_000_000, 1_000_000_000, .1,
                [0, 0, 0], [.001, 0, 0],
                command_logical_velocity=command, capture_hold=True,
                model_start_tip_m=[0, 0, 0])
    if changed:
        monitor.note_command(990_000_000, command)
        monitor.note_command(1_050_000_000, -command)
        monitor.note_command(1_100_000_000, command)
    result = monitor.observe(1_100_000_000, [-.001, 0, 0])
    assert not result.comparison_valid
    assert result.direction_cosine is None
    assert result.comparison_reason == (
        "published_command_changed" if changed
        else "missing_published_command_at_root")


def test_marker_gap_is_not_extrapolated_and_clear_resets_coverage():
    monitor = TipForecastMonitor()
    monitor.add(1_000_000_000, 900_000_000, .2,
                [0, 0, 0], [.001, 0, 0])
    result = monitor.observe(1_300_000_000, [.003, 0, 0])
    assert not result.comparison_valid
    assert result.comparison_reason == "unaligned_observations"
    monitor.clear()
    assert monitor.pending_count == 0
    assert not monitor._commands and not monitor._observations


def test_model_root_offset_is_separate_from_motion_prediction():
    monitor = TipForecastMonitor()
    monitor.add(1_000_000_000, 1_000_000_000, .1,
                [0, 0, 0], [.0015, 0, 0],
                command_logical_velocity=np.zeros(6),
                model_start_tip_m=[.0005, 0, 0])
    for t in (990_000_000, 1_050_000_000, 1_100_000_000):
        monitor.note_command(t, np.zeros(6))
    result = monitor.observe(1_100_000_000, [.001, 0, 0])
    assert result.comparison_valid
    assert result.root_readout_error_mm == pytest.approx([.5, 0, 0])
    assert result.predicted_delta_mm == pytest.approx([1, 0, 0])
    assert result.measured_delta_mm == pytest.approx([1, 0, 0])
    assert result.endpoint_error_mm == pytest.approx([-.5, 0, 0])


def test_missing_command_heartbeat_coverage_invalidates_comparison():
    monitor = TipForecastMonitor()
    monitor.add(1_000_000_000, 1_000_000_000, .2,
                [0, 0, 0], [.001, 0, 0],
                command_logical_velocity=np.zeros(6), model_start_tip_m=[0, 0, 0])
    monitor.note_command(990_000_000, np.zeros(6))
    monitor.note_command(1_250_000_000, np.zeros(6))
    monitor.observe(1_100_000_000, [.0005, 0, 0])
    result = monitor.observe(1_200_000_000, [.001, 0, 0])
    assert not result.comparison_valid
    assert result.comparison_reason == "published_command_coverage_gap"
