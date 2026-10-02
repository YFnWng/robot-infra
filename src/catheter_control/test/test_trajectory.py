import numpy as np
import pytest

from catheter_control.planning.trajectory import TrajectorySequencer


def test_reached_waypoint_advances_and_final_reach_completes():
    sequence = TrajectorySequencer(
        [[.001, 0, 0], [.002, 0, 0]], [2.0, 3.0],
        tolerance_mm=.2, settle_time_s=.1)
    sequence.start(10.0)
    assert not sequence.update([.001, 0, 0], 10.05).complete
    update = sequence.update([.001, 0, 0], 10.16)
    assert update.target_changed
    assert update.waypoint_index == 1
    assert update.elapsed_s == 0.0
    assert update.tracking_error_mm == pytest.approx(1.0)
    assert update.reached_waypoints == 1
    sequence.update([.002, 0, 0], 10.20)
    update = sequence.update([.002, 0, 0], 10.31)
    assert update.complete
    assert update.reached_waypoints == 2
    assert update.timed_out_waypoints == 0


def test_timeout_advances_regardless_of_tracking_error():
    sequence = TrajectorySequencer(
        [[.001, 0, 0], [.002, 0, 0]], [.5, .25],
        tolerance_mm=.1, settle_time_s=0.0)
    sequence.start(1.0)
    update = sequence.update([0, 0, 0], 1.5)
    assert update.target_changed
    assert update.timed_out_waypoints == 1
    assert update.elapsed_s == 0.0
    assert update.remaining_s == pytest.approx(.25)
    update = sequence.update([0, 0, 0], 1.75)
    assert update.complete
    assert update.reached_waypoints == 0
    assert update.timed_out_waypoints == 2


def test_timeout_is_reset_for_each_waypoint():
    sequence = TrajectorySequencer(
        [[0, 0, 0], [1, 0, 0]], [1.0, 2.0],
        tolerance_mm=.1, settle_time_s=0.0)
    sequence.start(5.0)
    first = sequence.update([0, 0, 0], 5.2)
    assert first.target_changed
    second = sequence.update([0, 0, 0], 6.9)
    assert not second.complete
    assert second.remaining_s == pytest.approx(.3)


@pytest.mark.parametrize("waypoints,timeouts", [
    ([], []),
    ([[0, 0]], [1]),
    ([[0, 0, np.nan]], [1]),
    ([[0, 0, 0]], []),
    ([[0, 0, 0]], [0]),
])
def test_invalid_trajectory_is_rejected(waypoints, timeouts):
    with pytest.raises(ValueError):
        TrajectorySequencer(
            waypoints, timeouts, tolerance_mm=.5, settle_time_s=0.1)
