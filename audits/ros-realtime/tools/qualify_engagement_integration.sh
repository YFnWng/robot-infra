#!/usr/bin/env bash
# Step 5: wheel isolation, inert real-owner tests, matched recorded compute.
# Never starts ROS nodes, camera/serial transport, or a command publisher.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 7 ]]; then
  echo 'Usage: bash qualify_engagement_integration.sh G4_ROOT NEW_OUTPUT [cpu|cuda] [SAMPLES] [DURATION_S] [task-boundary|normal] [fitted_profile|v171_static]' >&2
  exit 2
fi
g4=$(realpath -- "$1")
out=$(realpath -m -- "$2")
device=${3:-cuda}; samples=${4:-512}; duration=${5:-4}
gc_policy=${6:-task-boundary}
input_prior=${7:-fitted_profile}
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
workspace=$(dirname -- "$repo")
python=${CR_VENV:-$workspace/cr-venv}/bin/python
[[ "$device" == cpu || "$device" == cuda ]]
[[ "$gc_policy" == task-boundary || "$gc_policy" == normal ]]
[[ "$input_prior" == fitted_profile || "$input_prior" == v171_static ]]
[[ -x "$python" && -f "$g4/manifest.json" && -f "$g4/sensor_events.json" && ! -e "$out" ]]
mkdir -p -- "$out/wheels" "$out/packages"
# A fresh build tree avoids setuptools retaining deleted modules from a dirty
# repository build/lib cache. Stage only the maintained distribution inputs.
mkdir -p -- "$out/build_sources/model/artifacts/manifests" "$out/build_sources/model/artifacts/deployed" "$out/build_sources/common"
cp -R -- "$workspace/cr_meta_lnn/deployment" "$workspace/cr_meta_lnn/networks" "$out/build_sources/model/"
cp -- "$workspace/cr_meta_lnn/pyproject.toml" "$workspace/cr_meta_lnn/README.md" "$workspace/cr_meta_lnn/__init__.py" "$out/build_sources/model/"
cp -- "$workspace/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json" "$out/build_sources/model/artifacts/manifests/"
cp -R -- "$workspace/cr_meta_lnn/artifacts/deployed/20260929_175554_grouped_no_rotation" "$out/build_sources/model/artifacts/deployed/"
cp -R -- "$workspace/cr-common/cr_common" "$out/build_sources/common/"
cp -- "$workspace/cr-common/pyproject.toml" "$out/build_sources/common/"
# Build and install ONLY into this suite's target directory. No network,
# dependency resolution, activation hooks, or replacement of the active venv.
cd -- "$out"
"$python" -m pip wheel --no-deps --no-build-isolation --no-index \
  "$out/build_sources/model" "$out/build_sources/common" -w "$out/wheels" >"$out/wheel.log" 2>&1
"$python" -m pip install --no-deps --no-index --target "$out/packages" \
  "$out"/wheels/*.whl >>"$out/wheel.log" 2>&1
sha256sum "$out"/wheels/*.whl >"$out/wheels.sha256"
# Include fixtures only, never deployment source: these existing tests import
# their synthetic mechanics fixtures through cr_meta_lnn.tests.
mkdir -p -- "$out/packages/cr_meta_lnn/tests"
cp -- "$workspace/cr_meta_lnn/tests/test_joint_engagement_filter.py" \
  "$workspace/cr_meta_lnn/tests/test_engagement_runtime.py" \
  "$workspace/cr_meta_lnn/tests/test_v171_deployment.py" \
  "$workspace/cr_meta_lnn/tests/test_v171_static_prior.py" \
  "$out/packages/cr_meta_lnn/tests/"
# Use rebuilt/sourced ROS packages, not an injected source-package path.
export PYTHONPATH="$out/packages:${PYTHONPATH:-}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
manifest="$g4/manifest.json"
if [[ "$input_prior" == v171_static ]]; then
  manifest="$out/manifest.json"
  "$python" - "$g4/manifest.json" "$manifest" <<'PY'
import json,sys
from pathlib import Path
from cr_meta_lnn.deployment.experimental.engagement_bundle import make_static_prior_manifest
Path(sys.argv[2]).write_text(json.dumps(make_static_prior_manifest(sys.argv[1]),indent=2)+'\n')
PY
fi
export ENGAGEMENT_TEST_MANIFEST="$manifest"
"$python" -c 'import sys,json; from pathlib import Path; from runtime_supervision.qualify_engagement_integration import installed_identity; print(json.dumps(installed_identity(Path(sys.argv[1]),Path(sys.argv[2])),indent=2))' \
  "$out/packages" "$workspace" >"$out/package_identity.json"
failed=0
echo "Installed-wheel owner/transition tests; log: $out/tests.log"
if ! timeout 600 "$python" -m pytest -q --rootdir="$out" --junitxml="$out/tests.xml" \
  "$repo/src/catheter_control/test/test_g5_node.py" \
  "$repo/src/catheter_control/test/test_configuration.py" \
  "$repo/src/catheter_control/test/test_parameters.py" \
  "$repo/src/catheter_control/test/test_engagement_control.py" \
  "$repo/src/catheter_control/test/test_mppi.py" \
  "$repo/src/catheter_control/test/test_path_plan_commit.py" \
  "$repo/src/catheter_control/test/test_path_startup.py" \
  "$repo/src/catheter_control/test/test_path_tracking.py" \
  "$repo/src/catheter_control/test/test_hardware_contract.py" \
  "$repo/src/catheter_control/test/test_backlash.py" \
  "$repo/src/catheter_control/test/test_backlash_ownership.py" \
  "$repo/src/catheter_control/test/test_confirmation_policy_characterization.py" \
  "$repo/src/catheter_control/test/test_planner_process.py" \
  "$repo/src/catheter_control/test/test_task_gc.py" \
  "$repo/src/control_tasks/test/test_path_completion.py" \
  "$repo/src/runtime_supervision/test/test_compute_profile.py" \
  "$repo/src/runtime_supervision/test/test_engagement_g4.py" \
  "$repo/src/runtime_supervision/test/test_engagement_integration.py" \
  "$out/packages/cr_meta_lnn/tests/test_engagement_runtime.py" \
  "$out/packages/cr_meta_lnn/tests/test_joint_engagement_filter.py" \
  "$out/packages/cr_meta_lnn/tests/test_v171_static_prior.py" \
  >"$out/tests.log" 2>&1; then failed=1; fi
baseline="$out/packages/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json"
limits="$repo/src/control_interface/config/catheter_limits.yaml"
# Windows retain causal prefix history. Alternate model evaluation is serial,
# never a live scheduling measurement. All rows use recorded targets.
for offset in 40 55 72; do
  echo "Matched CPU-estimator/$device-worker replay: offset ${offset}s; log: $out/window_$offset.log"
  if ! timeout 600 "$python" -m runtime_supervision.qualify_engagement_integration \
      --manifest "$manifest" --baseline-manifest "$baseline" \
      --events "$g4/sensor_events.json" --limits "$limits" \
      --package-root "$out/packages" --source-root "$workspace" \
      --device "$device" --samples "$samples" \
      --gc-policy "$gc_policy" \
      --start-offset-s "$offset" --duration-s "$duration" \
      --output "$out/window_$offset" >"$out/window_$offset.log" 2>&1; then failed=1; fi
done
"$python" - "$out" "$device" "$failed" "$gc_policy" "$input_prior" "$manifest" <<'PY'
import json,sys,xml.etree.ElementTree as ET
from pathlib import Path
out=Path(sys.argv[1]); failures=[]; reports=[]
try:
    cases=ET.parse(out/'tests.xml').findall('.//testcase')
    bad=[c.attrib for c in cases if any(c.find(k) is not None for k in ('failure','error','skipped'))]
    if not cases or bad: failures.append('owner_tests_failed_or_skipped')
except Exception as error:
    cases=[]; failures.append('test_report_missing:'+str(error))
for offset in (40,55,72):
    path=out/f'window_{offset}/report.json'
    if not path.is_file(): failures.append(f'window_{offset}_report_missing'); continue
    row=json.loads(path.read_text()); reports.append({'offset_s':offset,'summary':row['summary'],'passed':row['passed'],'report':str(path),'gc':row['gc']})
    if row['gc']['policy'] != sys.argv[4]: failures.append(f'window_{offset}_gc_policy_mismatch')
    if row['distal_input_prior'] != sys.argv[5]: failures.append(f'window_{offset}_prior_mismatch')
    if not row['passed']: failures.append(f'window_{offset}_failed')
if int(sys.argv[3]): failures.append('subprocess_failure')
payload={'schema':'engagement_integration_suite_v1','passed':not failures,
    'qualification':'OFFLINE_COMPONENTS_ONLY','hardware_authorized':False,
    'executed_commands':0,'device':sys.argv[2],'gc_policy':sys.argv[4],'owner_test_count':len(cases),
    'distal_input_prior':sys.argv[5],'manifest':sys.argv[6],
    'failures':failures,'windows':reports,'package_identity':str(out/'package_identity.json'),
    'limits':'Sequential recorded solves plus inert actual-owner callback tests; not physical tracking, ROS contention, sensor-to-actuator timing or active-package promotion.'}
(out/'suite.json').write_text(json.dumps(payload,indent=2)+'\n')
print(json.dumps({'suite':str(out/'suite.json'),'passed':payload['passed']}))
sys.exit(0 if payload['passed'] else 1)
PY
