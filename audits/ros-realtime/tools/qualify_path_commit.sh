#!/usr/bin/env bash
# Non-actuating recorded-preview qualification; no hardware stack is launched.
set -euo pipefail
if [[ ${ROS_DOMAIN_ID:-} != 213 ]]; then
  echo 'Use ROS_DOMAIN_ID=213, never the hardware domain.' >&2
  exit 2
fi
if [[ $# != 2 ]]; then
  echo 'Usage: bash qualify_path_commit.sh SESSION_ROOT NEW_OUTPUT_DIRECTORY' >&2
  exit 2
fi
base=$1
out=$2
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
python=${CR_VENV:-/home/chen-lab/Yifan/cr-venv}/bin/python
manifest=/home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json
limits="$repo/src/control_interface/config/catheter_limits.yaml"
sessions=(
  continuous_twoaxis_circle_home_plain_v171_n512_speed1_mount01_repeat02_20261005T235130732298Z
  continuous_twoaxis_circle_home_plain_takeup_v171_n512_speed1_mount01_repeat01_20261005T235431449181Z
  continuous_twoaxis_circle_home_grouped_takeup_v171_n512_speed1_mount01_repeat01_20261005T235706766686Z
  continuous_twoaxis_circle_home_grouped_no_takeup_v171_n512_speed1_mount01_repeat01_20261006T000017898562Z
)
names=(plain plain_takeup grouped_takeup grouped_no_takeup)
[[ -x "$python" && -f "$manifest" && -f "$limits" && ! -e "$out" ]]
for session in "${sessions[@]}"; do
  [[ -d "$base/$session" ]]
done
mkdir -p -- "$out"
failed=0
run_window() {
  local name=$1 session=$2 offset=$3 duration=$4
  echo "Replay $name: task offset ${offset}s, measurement ${duration}s"
  if "$python" -m runtime_supervision.compare_ros "$base/$session" \
      --model-manifest "$manifest" --limits-file "$limits" \
      --device cuda --estimator-device cpu --ros-ablation combined \
      --planner-process --path-reference-commit \
      --start-offset-s "$offset" --duration-s "$duration" --output "$out/$name" \
      >"$out/$name.log" 2>&1; then
    echo "PASS $name; report: $out/$name/report.json"
  else
    echo "FAIL $name; inspect $out/$name.log and report.json" >&2
    failed=1
  fi
}
for index in "${!sessions[@]}"; do
  run_window "${names[$index]}" "${sessions[$index]}" 1 8
done
run_window plain_final_hold "${sessions[0]}" 16 3
echo "Qualification artifacts: $out (predicate replay, no executed commands)"
exit "$failed"
