#!/usr/bin/env bash
# Qualified wheel promotion only: no ROS launch, device access or commands.
set -euo pipefail

TASK_ROOT=/home/chen-lab/Yifan
TASK_PYTHON="$TASK_ROOT/cr-venv/bin/python"
TASK_SITE="$TASK_ROOT/cr-venv/lib/python3.10/site-packages"
TASK_CANDIDATE=/tmp/optimized-eager-qualification.i3WCjL

if [[ $# != 1 || $1 != --install ]]; then
    echo "Usage: bash $0 --install (stop controllers and shadow/replay processes first)" >&2
    exit 2
fi
if tr '\0' ' ' </proc/1/cmdline | grep -q 'codex-linux-sandbox'; then
    echo 'Refusing promotion from an isolated process namespace; run in a host terminal.' >&2
    exit 2
fi
# Match executable/module arguments, not shell commands containing this script.
check_stopped() {
local TASK_RUNNING
TASK_RUNNING=$(ps -eo pid=,comm=,args= | awk '
    $2 ~ /python|catheter_mppi|engagement/ &&
    $0 ~ /catheter_control.node|catheter_mppi|engagement_shadow|engagement_g4/ {print}')
if [[ -n $TASK_RUNNING ]]; then
    echo "Stop these processes before replacing their shared runtime:" >&2
    echo "$TASK_RUNNING" >&2
    exit 2
fi
}
check_stopped

cd /tmp
sha256sum --check --strict <<CHECKSUMS
4c65b5138f7b27f77bf52888bb06e6e06a586a05811dde84194d7cbc54078afa  $TASK_CANDIDATE/repaired-wheels/cr_meta_lnn-1.0.0-py3-none-any.whl
d2750749735fed7213e2b19a5a24e4d049345a69115f78c963060c4329fced5a  $TASK_CANDIDATE/wheels/cr_common-0.1.0-py3-none-any.whl
CHECKSUMS

# Verify CUDA marker arithmetic in the exact isolated candidate before installing.
env PYTHONPATH="$TASK_CANDIDATE/repaired-installed" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    "$TASK_PYTHON" -c 'import torch; assert torch.cuda.is_available(), "CUDA qualification requires CUDA"'
env PYTHONPATH="$TASK_CANDIDATE/repaired-installed" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    "$TASK_PYTHON" -m pytest -q \
    "$TASK_ROOT/cr_meta_lnn/tests/test_compute_optimizations.py"

TASK_BACKUP=$(mktemp -d /tmp/optimized-eager-promotion.XXXXXXXX)
echo "Promotion record and rollback backup: $TASK_BACKUP"
# RECORD also covers editable-install .pth/finder files; package directories
# need not exist in site-packages. Refuse unexpected paths before installation.
env -u PYTHONPATH "$TASK_PYTHON" - "$TASK_SITE" <<'PY' >"$TASK_BACKUP/previous-files.txt"
from importlib.metadata import distribution
from pathlib import Path
import sys

site = Path(sys.argv[1]).resolve()
files = set()
for name in ('cr-meta-lnn', 'cr-common'):
    records = distribution(name).files
    assert records, f'Missing installed RECORD: {name}'
    for entry in records:
        path = Path(distribution(name).locate_file(entry))
        assert path.resolve().is_relative_to(site), f'External installed file: {path}'
        if path.is_file():
            relative = str(path.relative_to(site))
            assert '\n' not in relative, f'Unsupported filename: {relative!r}'
            files.add(relative)
print('\n'.join(sorted(files)))
PY
tar -czf "$TASK_BACKUP/previous-packages.tar.gz" -C "$TASK_SITE" \
    --verbatim-files-from -T "$TASK_BACKUP/previous-files.txt"
tar -tzf "$TASK_BACKUP/previous-packages.tar.gz" >/dev/null
env -u PYTHONPATH "$TASK_PYTHON" - <<'PY' >"$TASK_BACKUP/previous-source.sha256"
import hashlib
import importlib
from pathlib import Path

for name in ('cr_meta_lnn.deployment.v171_streaming_runtime', 'cr_common.kinematics'):
    path = Path(importlib.import_module(name).__file__).resolve()
    print(hashlib.sha256(path.read_bytes()).hexdigest(), path)
PY
cp "$TASK_CANDIDATE/repaired-wheels/cr_meta_lnn-1.0.0-py3-none-any.whl" \
    "$TASK_CANDIDATE/wheels/cr_common-0.1.0-py3-none-any.whl" "$TASK_BACKUP/"
trap 'echo "Promotion failed: keep controllers stopped. Backup: $TASK_BACKUP" >&2; echo "Restore: $TASK_PYTHON -m pip uninstall -y cr-meta-lnn cr-common && tar -xzf $TASK_BACKUP/previous-packages.tar.gz -C $TASK_SITE" >&2' ERR
check_stopped

env -u PYTHONPATH "$TASK_PYTHON" -m pip install --no-deps --no-index --force-reinstall \
    "$TASK_BACKUP/cr_common-0.1.0-py3-none-any.whl" \
    "$TASK_BACKUP/cr_meta_lnn-1.0.0-py3-none-any.whl"
cmp "$TASK_CANDIDATE/repaired-installed/cr_meta_lnn/deployment/v171_streaming_runtime.py" \
    "$TASK_SITE/cr_meta_lnn/deployment/v171_streaming_runtime.py"
cmp "$TASK_CANDIDATE/repaired-installed/cr_common/kinematics.py" \
    "$TASK_SITE/cr_common/kinematics.py"

env -u PYTHONPATH "$TASK_PYTHON" - "$TASK_SITE" <<'PY'
import hashlib
import importlib
from pathlib import Path
import sys

site = Path(sys.argv[1]).resolve()
for name in ('cr_meta_lnn.deployment.v171_streaming_runtime', 'cr_common.kinematics'):
    path = Path(importlib.import_module(name).__file__).resolve()
    assert path.is_relative_to(site), f'Unexpected import path: {path}'
    print(name, path, hashlib.sha256(path.read_bytes()).hexdigest(), flush=True)
runtime = importlib.import_module('cr_meta_lnn.deployment.v171_streaming_runtime')
assert hasattr(runtime.V171StreamingCatheterRuntime, 'prepare_control_accelerator')
PY
sha256sum "$TASK_SITE/cr_meta_lnn/deployment/v171_streaming_runtime.py" \
    "$TASK_SITE/cr_common/kinematics.py" >"$TASK_BACKUP/promoted-source.sha256"

# ROS-independent historical replay; only the ROS package source enters PYTHONPATH.
env PYTHONPATH="$TASK_ROOT/robot-infra/src/catheter_control" \
    "$TASK_PYTHON" -m catheter_control.safety.conformance \
    --model-manifest "$TASK_SITE/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json" \
    --output "$TASK_BACKUP/installed-conformance.json"

echo 'Installed eager runtime passed historical conformance. No hardware was started.'
echo "Keep $TASK_BACKUP until representative full-stack timing passes."
echo "If installation/verification failed, keep controllers stopped and restore with:"
echo "'$TASK_PYTHON' -m pip uninstall -y cr-meta-lnn cr-common && tar -xzf '$TASK_BACKUP/previous-packages.tar.gz' -C '$TASK_SITE'"
