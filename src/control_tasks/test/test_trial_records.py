import json

import pytest

from control_tasks.trial_records import TrialRecords, finite_error


def test_journal_is_durable_and_exclusive(tmp_path):
    path = tmp_path / "trials.jsonl"
    journal = TrialRecords(path)
    journal.write("trial_started", 123, trial=1)
    assert json.loads(path.read_text())["ros_time_ns"] == 123
    journal.close()
    with pytest.raises(FileExistsError):
        TrialRecords(path)
    assert finite_error(float("nan")) is None


def test_optional_journal_has_no_file_output():
    journal = TrialRecords(None)
    journal.write("trial_started", 123)
    journal.close()
