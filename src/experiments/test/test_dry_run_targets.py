"""Dry-run target publisher: target loading and the fail-closed controller gate. No ROS graph."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from experiments.dry_run_targets import dry_run_status, load_targets

SOURCE = Path(__file__).resolve().parents[1]/'experiments'/'dry_run_targets.py'


def status(hardware_id='reduced_filter_dry_run', output='False', capable='False'):
    values = [SimpleNamespace(key='command_output_enabled', value=output),
              SimpleNamespace(key='command_capable', value=capable)]
    return SimpleNamespace(status=[SimpleNamespace(name='catheter_mppi', hardware_id=hardware_id, values=values)])


def test_gate_opens_only_for_the_non_commanding_dry_run():
    assert dry_run_status(status()) == (True, None)
    assert not dry_run_status(status(hardware_id='v171'))[0]
    assert not dry_run_status(status(output='True'))[0]
    assert not dry_run_status(SimpleNamespace(status=[]))[0]


def test_targets_load_from_experiment_file_and_reject_millimetres(tmp_path):
    path = tmp_path/'experiment.yaml'
    path.write_text('generator:\n  targets_m:\n    - [0.026, 0.022, 0.084]\n')
    np.testing.assert_allclose(load_targets(path, [(0.03, 0.03, 0.05)]), [[.026, .022, .084], [.03, .03, .05]])
    with pytest.raises(ValueError, match='metres'):
        load_targets(None, [(26., 22., 84.)])


def test_publisher_only_publishes_targets():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    publishers = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'attr', None) == 'create_publisher']
    assert [(p.args[0].id, p.args[1].value) for p in publishers] == [('PointStamped', '/catheter_mppi/target_tip')]
    assert 'create_client' not in SOURCE.read_text() and 'SET_ZERO' not in SOURCE.read_text()
