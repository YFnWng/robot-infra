import numpy as np
import pytest

from experiments.path_analysis import plot_path_3d


def test_3d_plot_uses_mm_equal_scale_and_breaks_marker_gaps(tmp_path, monkeypatch):
    from matplotlib.figure import Figure
    captured = {}

    def save(figure, path, **kwargs):
        captured["axes"] = figure.axes[0]
        captured["path"] = path

    monkeypatch.setattr(Figure, "savefig", save)
    output = tmp_path / "path.png"
    assert plot_path_3d(output, [
        (0, [.001, 0, 0]), (100_000_000, [.002, 0, 0]),
        (500_000_000, [.003, 0, 0]),
    ], [[0, 0, 0], [.004, .002, 0]]) == output
    axes = captured["axes"]
    assert captured["path"] == output
    assert len(axes.lines) == 3  # target and two disconnected marker segments
    np.testing.assert_allclose(axes.lines[0].get_data_3d()[0], [0, 4])
    np.testing.assert_allclose(axes.lines[1].get_data_3d()[0], [1, 2])
    np.testing.assert_allclose(axes.lines[2].get_data_3d()[0], [3])
    spans = [np.ptp(limits) for limits in
             (axes.get_xlim(), axes.get_ylim(), axes.get_zlim())]
    np.testing.assert_allclose(spans, [spans[0]] * 3)


@pytest.mark.parametrize("samples, target", [
    ([], [[0, 0, 0], [1, 0, 0]]),
    ([(0, [float("nan"), 0, 0])], [[0, 0, 0], [1, 0, 0]]),
    ([(0, [0, 0, 0])], [[0, 0]]),
])
def test_3d_plot_rejects_missing_or_invalid_coordinates(tmp_path, samples, target):
    with pytest.raises(ValueError):
        plot_path_3d(tmp_path / "path.png", samples, target)
