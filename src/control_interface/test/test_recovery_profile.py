"""The temporary measured-offset profile must not widen physical travel."""
from pathlib import Path

import yaml


def test_measured_translation_recovery_is_narrow_and_other_targets_locked():
    profiles = yaml.safe_load((Path(__file__).parents[1] / 'config' /
                               'catheter_limits.yaml').read_text())['catheters']
    recovery = profiles['imricor_translation_offset_recovery']
    normal = profiles['imricor_test']
    assert normal['pos_lower'][0] == -10. and normal['pos_upper'][0] == 50.
    assert recovery['pos_lower'][0] <= -20. <= recovery['pos_upper'][0]
    assert recovery['pos_lower'][0]+20. >= normal['pos_lower'][0]
    assert recovery['pos_upper'][0]+20. <= normal['pos_upper'][0]
    assert recovery['pos_lower'][1:] == recovery['pos_upper'][1:] == [0.]*5
    assert recovery['vel_min'][0] == recovery['vel_max'][0] == 2.
    assert all(value > 0 for value in recovery['vel_max'])
