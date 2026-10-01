from types import SimpleNamespace

import numpy as np

from catheter_control.camera_overlay import (
    draw_camera_overlay,
    nearest_sample,
)


def test_nearest_sample_enforces_alignment_gate():
    samples = [(100, "first"), (200, "second")]
    assert nearest_sample(samples, 180, 25) == (200, "second")
    assert nearest_sample(samples, 150, 25) is None


def test_overlay_draws_measured_markers_and_estimated_shape():
    registration = SimpleNamespace(
        K=np.eye(3),
        left_camera_T_base=np.eye(4),
        right_camera_T_base=np.eye(4))

    def project_points(_intrinsics, _transform, points):
        points = np.asarray(points)
        pixels = points[:, :2].copy()
        return pixels, np.ones(len(points), dtype=bool)

    image = np.zeros((100, 100, 3), dtype=np.uint8)
    output = draw_camera_overlay(
        image, registration, project_points, (100, 100), "left",
        measured_markers_mm=np.array([
            [20., 20., 1.], [30., 30., 1.],
            [40., 40., 1.], [50., 50., 1.]]),
        centerline_mm=np.array([
            [10., 10., 1.], [20., 15., 1.], [30., 25., 1.]]),
        target_path_mm=np.array([
            [60., 70., 1.], [80., 70., 1.]]),
        estimator_health="TRACKING", marker_skew_ms=2.,
        estimator_skew_ms=-3., marker_crosshair_size=8,
        marker_crosshair_line_width=1, target_path_line_width=1)

    assert output.shape == image.shape
    assert np.any(output != image)
    assert output[70, 70, 1] > 0
    assert output[70, 70, 2] > 0
