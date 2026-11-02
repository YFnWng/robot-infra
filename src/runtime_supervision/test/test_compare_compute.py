import io
from pathlib import Path
import tarfile
import zipfile

import pytest

from runtime_supervision.compare_compute import digest, prepare_packages, safe_member, unpack_models, verify_imports


def test_variant_order():
    from runtime_supervision.compare_compute import variant_order
    assert variant_order("both", 0) == ("old", "new")
    assert variant_order("both", 1) == ("new", "old")
    assert variant_order("new", 1) == ("new",)
    assert variant_order("old", 0) == ("old",)
    with pytest.raises(ValueError):
        variant_order("unknown", 0)


@pytest.mark.parametrize("name", ["/etc/passwd", "../escape", "cr_meta_lnn/../../escape"])
def test_archive_path_rejected(name):
    with pytest.raises(ValueError, match="unsafe"):
        safe_member(name)


def test_tar_symlink_rejected(tmp_path):
    archive = tmp_path/"backup.tar.gz"
    with tarfile.open(archive, "w:gz") as stream:
        member = tarfile.TarInfo("cr_meta_lnn/link")
        member.type = tarfile.SYMTYPE
        member.linkname = "/etc/passwd"
        stream.addfile(member)
    with pytest.raises(ValueError, match="regular"):
        unpack_models(archive, tmp_path/"out")


def test_isolation_fails_closed(monkeypatch, tmp_path):
    from runtime_supervision import compare_compute
    from types import SimpleNamespace
    monkeypatch.setattr(compare_compute.importlib, "import_module", lambda _: SimpleNamespace(__file__="/tmp/other.py"))
    with pytest.raises(RuntimeError, match="isolation failed"):
        verify_imports(tmp_path)


def test_prepare_preserves_backup_without_editable_hooks(tmp_path):
    backup = tmp_path/"backup"
    backup.mkdir()
    runtime = "cr_meta_lnn/deployment/v171_streaming_runtime.py"
    fk = "cr_common/kinematics.py"
    with tarfile.open(backup/"previous-packages.tar.gz", "w:gz") as stream:
        for name, data in ((runtime, b"old"), ("__editable__.pth", b"do not activate")):
            member = tarfile.TarInfo(name)
            member.size = len(data)
            stream.addfile(member, io.BytesIO(data))
    for filename, name, data in (("cr_meta_lnn-1.whl", runtime, b"new"), ("cr_common-1.whl", fk, b"common")):
        with zipfile.ZipFile(backup/filename, "w") as stream:
            stream.writestr(name, data)
    import hashlib
    (backup/"previous-source.sha256").write_text(
        hashlib.sha256(b"old").hexdigest()+" /old/"+runtime+"\n"+
        hashlib.sha256(b"common").hexdigest()+" /old/"+fk+"\n")
    output = tmp_path/"output"
    output.mkdir()
    packages = prepare_packages(backup, output)
    assert packages["old"]["runtime_sha256"] != packages["new"]["runtime_sha256"]
    assert packages["old"]["fk_sha256"] == packages["new"]["fk_sha256"]
    assert not (Path(packages["old"]["root"])/"__editable__.pth").exists()
    assert digest(output/"promotion_backup"/"previous-packages.tar.gz") == digest(backup/"previous-packages.tar.gz")


def test_bootstrap_uses_existing_environment(monkeypatch):
    from catheter_control import bootstrap as owner
    from runtime_supervision import bootstrap
    calls = []
    monkeypatch.setattr(owner, "run_in_venv", calls.append)
    bootstrap.compare_compute()
    assert calls == ["runtime_supervision.compare_compute"]


@pytest.mark.parametrize("fail", [False, True, "timeout"])
@pytest.mark.parametrize("evolving", [False, True])
def test_split_workers_coordinated_and_reaped(tmp_path, monkeypatch, fail, evolving):
    import os
    import subprocess
    import sys
    from runtime_supervision import compare_compute
    real_popen = subprocess.Popen
    children, commands = [], []
    # Lightweight owned subprocesses exercise orchestration without ROS/CUDA.
    script = '''
import json, pathlib, sys, time
out, barrier, role, fail = sys.argv[1:]
out, barrier = pathlib.Path(out), pathlib.Path(barrier)
out.mkdir()
if fail == 'True' and role == 'estimator': sys.exit(7)
if fail == 'timeout': time.sleep(30)
(barrier/(role+'.ready')).touch()
while not (barrier/'epoch.json').exists(): time.sleep(.01)
epoch = json.loads((barrier/'epoch.json').read_text())['epoch_monotonic_s']
(out/'report.json').write_text(json.dumps({'replay_epoch_monotonic_s':epoch, 'stages':{'ros_marker_owner':{'count':1}}, 'plans':[{'source_ns':1},{'source_ns':2}], 'handoff_qualification':{'passed':True}}))
'''
    def launch(command, **kwargs):
        commands.append((command, kwargs['env']['ROS_DOMAIN_ID']))
        process = real_popen([sys.executable, '-c', script,
            command[command.index('--output')+1], command[command.index('--coordination-dir')+1],
            command[-1], str(fail)], **kwargs)
        children.append(process)
        return process
    monkeypatch.setattr(compare_compute.subprocess, 'Popen', launch)
    command = [sys.executable, '-m', 'runtime_supervision.compare_compute', '_worker',
               'root', 'ros', 'identity', 'session', '--output', 'unused',
               '--ros-ablation', 'combined', '--device', 'cuda', '--estimator-device', 'cpu',
               '--ros-plan-fixture', 'plan.pt']
    if evolving:
        command = command[:-2]+['--ros-evolving-snapshots']
    row = compare_compute.run_split_logged(command, tmp_path, 'split', os.environ.copy(),
                                          .2 if fail == 'timeout' else 5)
    assert row['exit_code'] == (-1 if fail else 0)
    assert all(child.poll() is not None for child in children)
    assert [domain for _, domain in commands] == ['213', '214']
    assert ('--ros-plan-fixture' in commands[0][0]) is not evolving
    assert '--ros-plan-fixture' not in commands[1][0]
    assert commands[1][0][commands[1][0].index('--device')+1] == 'cpu'
    if evolving:
        assert '--exchange-dir' in commands[0][0]
        assert '--exchange-dir' in commands[1][0]


def test_split_configuration_rejected_before_loading(tmp_path):
    from runtime_supervision.compare_compute import main
    with pytest.raises(SystemExit):
        main(['session', '--backup', 'backup', '--model-manifest', 'model',
              '--limits-file', 'limits', '--output', str(tmp_path/'output'),
              '--ros-processes', 'split'])
    assert not (tmp_path/'output').exists()


@pytest.mark.parametrize('offsets', [[], [1,1], [float('nan')], [-1], list(range(9)), [1000,1001]])
def test_multiwindow_bounds(offsets):
    from runtime_supervision.compare_compute import replay_windows
    with pytest.raises(ValueError):
        replay_windows(offsets, 8, 2)


def test_window_identifiers_preserve_requested_order():
    from runtime_supervision.compare_compute import replay_windows
    assert replay_windows([61,94,111], 8, 2) == [(1,61), (2,94), (3,111)]


def test_per_window_summary_does_not_hide_deadline_miss():
    from runtime_supervision.compare_compute import ros_window_summary
    runs = []
    for index, elapsed in [(1,20), (2,70)]:
        runs.append({'window_index': index, 'report': {
            'plans': [{'elapsed_ms': elapsed, 'valid': elapsed<60}],
            'deadline_misses': int(elapsed>60), 'handoff_qualification': {'passed': True},
            'plan_coverage': {'passed': True}, 'recorded_position_window': {'samples': 4},
            'pre_window_warmup': []}})
    rows = ros_window_summary(runs, [(1,60), (2,94)], 8)
    assert rows[0]['deadline_misses'] == 0
    assert rows[1]['deadline_misses'] == 1
    assert rows[1]['invalid_plans'] == 1
    assert rows[1]['timing_ms']['elapsed_ms']['max_ms'] == 70
