from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parent))

from identify_phase2_transmission import (  # noqa: E402
    Window,
    asymmetric_play,
    marker_pairwise_distances,
    search_play,
)


def test_asymmetric_play_has_expected_reversal_gap():
    signal = np.array([0.0, 1.0, 2.0, 1.0, 0.0, -1.0])
    result = asymmetric_play(signal, 0.4, 0.2)
    np.testing.assert_allclose(
        result, [0.0, 0.6, 1.6, 1.2, 0.2, -0.8], atol=1e-12)


def test_marker_pairwise_geometry_is_rigid_motion_invariant():
    points = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.2, 0.0, 2.0],
        [0.5, 0.1, 3.0],
    ])
    angle = 0.7
    rotation = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    transformed = points @ rotation.T + np.array([3.0, -2.0, 0.4])
    np.testing.assert_allclose(
        marker_pairwise_distances(points),
        marker_pairwise_distances(transformed), atol=1e-12)


def test_play_search_recovers_width_and_improves_held_out_prediction():
    one_cycle = np.concatenate((
        np.linspace(-1.0, 1.0, 51),
        np.linspace(1.0, -1.0, 51)[1:],
    ))
    signal = np.tile(one_cycle, 3)
    transmitted = asymmetric_play(signal, 0.4, 0.2)
    response_state = np.column_stack((
        2.0 * transmitted,
        -0.5 * transmitted,
    ))
    windows = []
    response = []
    cycle_length = len(one_cycle)
    for end in range(2, len(signal)):
        repetition = end // cycle_length + 1
        if repetition > 3:
            repetition = 3
        windows.append(Window(
            end - 2, end, "shaft_2", "slow", repetition,
            f"synthetic_rep{repetition}"))
        response.append(response_state[end] - response_state[end - 2])
    response = np.asarray(response)
    result = search_play(
        signal, response, windows, np.ones(2), holdout_repetition=3,
        grid_size=15, maximum_side_width=0.7)
    assert abs(result["positive_width_rad"] - 0.4) < 0.08
    assert abs(result["negative_width_rad"] - 0.2) < 0.08
    assert result["holdout_fractional_improvement"] > 0.8
