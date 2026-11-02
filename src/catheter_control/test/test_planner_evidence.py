"""Bounded, trusted-local diagnostic recording without hardware."""
import json
import pickle

import pytest

from catheter_control.orchestration.planner_evidence import PlannerEvidenceWriter


def test_records_are_frozen_and_count_bounded(tmp_path):
    writer = PlannerEvidenceWriter(tmp_path, {'schema': 'test'}, max_records=1)
    record = {'sequence': 1, 'values': [1]}
    writer.submit(record)
    record['values'][0] = 99
    writer.submit({'sequence': 2})
    writer.close()
    saved = pickle.loads((writer.directory/'plan_000001.pkl').read_bytes())
    assert saved['values'] == [1]
    summary = json.loads((writer.directory/'summary.json').read_text())
    assert summary['accepted'] == summary['written'] == 1
    assert summary['dropped'] == 1
    assert summary['writer_finished']


def test_byte_limit_drops_without_writing(tmp_path):
    writer = PlannerEvidenceWriter(tmp_path, {}, max_bytes=1)
    writer.submit({'sequence': 1})
    writer.close()
    assert writer.dropped == 1
    assert not list(writer.directory.glob('plan_*.pkl'))


def test_bounds_require_positive_values(tmp_path):
    with pytest.raises(ValueError):
        PlannerEvidenceWriter(tmp_path, {}, capacity=0)
