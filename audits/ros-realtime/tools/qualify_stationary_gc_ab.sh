#!/usr/bin/env bash
# Sequential, non-commanding parent-GC comparison; sensors already running.
set -euo pipefail
if [[ $# != 3 ]]; then
  echo 'Usage: bash qualify_stationary_gc_ab.sh G4_ROOT QUALIFIED_PACKAGE_ROOT NEW_OUTPUT' >&2
  exit 2
fi
g4=$(realpath -- "$1")
qualified=$(realpath -- "$2")
out=$(realpath -m -- "$3")
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
workspace=$(dirname -- "$repo")
python=${CR_VENV:-$workspace/cr-venv}/bin/python
[[ "$ROS_DOMAIN_ID" == 42 && ! -e "$out" && -f "$qualified/suite.json" ]]
export PYTHONPATH="$qualified/packages:${PYTHONPATH:-}"
cd /tmp
"$python" - "$qualified" <<'PY'
import json,sys,hashlib,importlib
from pathlib import Path
root=Path(sys.argv[1]);suite=json.loads((root/'suite.json').read_text())
assert suite['passed'] and suite['device']=='cuda'
for name,entry in json.loads((root/'package_identity.json').read_text()).items():
    assert Path(importlib.import_module(name).__file__).resolve()==Path(entry['path']).resolve()
    package='cr_common' if name.startswith('cr_common.') else 'cr_meta_lnn'
    for relative,digest in entry['python_files'].items():
        assert hashlib.sha256((root/'packages'/package/relative).read_bytes()).hexdigest()==digest
PY
mkdir -p -- "$out"
"$python" "$workspace/cr_meta_lnn/scripts/prepare_engagement_control_trial.py" \
  --shadow-manifest "$g4/manifest.json" --output "$out/manifest.json"
baseline="$qualified/packages/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json"
for family in experimental baseline; do
  if [[ "$family" == experimental ]]; then
    manifest="$out/manifest.json"; stack=hardware_engagement_twoaxis
  else
    manifest="$baseline"; stack=hardware_grouped_fixed_gain_no_rotation
  fi
  for policy in normal live-stationary; do
    arm="$out/${family}_${policy}"
    echo "Stationary GC comparison: $family / $policy (60s; no commands)"
    status=0
    "$python" -u -m runtime_supervision.qualify_live_compute \
      --stack-config "$stack" --model-manifest "$manifest" \
      --limits-file "$repo/src/control_interface/config/catheter_limits.yaml" \
      --device cuda --duration-s 60 --startup-timeout-s 60 \
      --target-offset-mm 0 0 5 --gc-policy "$policy" --gc-max-growth-mb 256 \
      --output "$arm" 2>&1 | tee "$out/${family}_${policy}.log" || status=$?
    echo "$family / $policy: exit $status; log $out/${family}_${policy}.log"
    # Qualification exit 2 is data, not permission to discard the failing arm.
  done
done
"$python" - "$out" <<'PY'
import json,sys
from pathlib import Path
out=Path(sys.argv[1]);rows=[];missing=[]
for family in ('experimental','baseline'):
    for policy in ('normal','live-stationary'):
        path=out/f'{family}_{policy}'/'report.json'
        if not path.is_file():missing.append(str(path));continue
        r=json.loads(path.read_text());g=r.get('gc_experiment') or {}
        rows.append(dict(family=family,policy=policy,report=str(path),passed=r['passed'],
            coverage=r.get('plan_coverage'),deadline_misses=r.get('deadline_misses'),
            valid_elapsed_ms=r.get('valid_plan_elapsed_ms'),failures=r.get('failures'),
            gc_restored=(g.get('original_enabled')==g.get('restored_enabled') and
                         g.get('original_thresholds')==g.get('restored_thresholds')),
            long_gc=[x for x in r.get('parent_pause_evidence',{}).get('collection_intervals',[])
                     if x['wall_ms']>=10]))
(out/'comparison.json').write_text(json.dumps(dict(scope='Stationary non-commanding GC A/B; not hardware qualification',
    rows=rows,missing_reports=missing),indent=2)+'\n')
print(json.dumps({'comparison':str(out/'comparison.json'),'missing_reports':missing}))
PY
