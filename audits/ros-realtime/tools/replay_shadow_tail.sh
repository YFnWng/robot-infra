#!/usr/bin/env bash
# Non-actuating paced DDS replay. Two experimental observers approximate
# concurrent compute load; neither is the production baseline controller.
set -euo pipefail
if [[ ${ROS_DOMAIN_ID:-} != 214 ]]; then
  echo 'Set ROS_DOMAIN_ID=214 in a fresh terminal; never replay in hardware domain 42.' >&2
  exit 2
fi
if [[ $# != 3 && $# != 4 ]]; then
  echo 'Usage: bash replay_shadow_tail.sh FINALIZED_BAG G4_MANIFEST NEW_OUTPUT_DIRECTORY [normal|deferred|task-boundary]' >&2
  exit 2
fi
bag=$1
manifest=$2
out=$3
gc_policy=${4:-normal}
if [[ "$gc_policy" != normal && "$gc_policy" != deferred && "$gc_policy" != task-boundary ]]; then
  echo 'GC policy must be normal, deferred or task-boundary.' >&2; exit 2
fi
[[ -f "$bag/metadata.yaml" && -f "$manifest" && ! -e "$out" ]]
mkdir -p "$out"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then kill -INT "$pid"; fi
  done
  for attempt in {1..10}; do
    alive=false
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then alive=true; fi
    done
    if [[ "$alive" == false ]]; then break; fi
    sleep 1
  done
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then kill -TERM "$pid"; fi
  done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
for variant in grouped plain; do
  extra=()
  if [[ "$variant" == plain ]]; then extra+=(--plain); fi
  python -m runtime_supervision.engagement_shadow \
    --manifest "$manifest" --device cuda --planner-process --samples 512 \
    --duration-s 180 --stop-after-targets 4 --outdir "$out/$variant" \
    --gc-policy "$gc_policy" --gc-max-growth-mb 512 \
    "${extra[@]}" --ros-args -r "__node:=tail_$variant" -p use_sim_time:=true \
    >"$out/$variant.log" 2>&1 &
  pids+=("$!")
done
# Wait for both processes to finish loading before starting causal history.
for variant in grouped plain; do
  ready=false
  for attempt in {1..60}; do
    if [[ $(<"$out/$variant.log") == *shadow_waiting_for_observed_preparation* ]]; then
      ready=true; break
    fi
    if ! kill -0 "${pids[0]}" 2>/dev/null || ! kill -0 "${pids[1]}" 2>/dev/null; then
      echo 'Observer startup failed; inspect logs.' >&2; exit 1
    fi
    sleep 1
  done
  if [[ "$ready" != true ]]; then echo 'Observer startup timed out.' >&2; exit 1; fi
done
# Start from the beginning: initialization/history is required for the late
# segment. Only read-only evidence topics are published, never command topics.
timeout --signal=INT --kill-after=10s 125s ros2 bag play "$bag" --clock 100 \
  --topics /device/state /shape_tracking/markers /shape_tracking/marker_status \
  /catheter_mppi/status /catheter_mppi/target_tip \
  /catheter_mppi/control_cycle_timing /catheter_mppi/estimator_trace \
  >"$out/player.log" 2>&1 &
player=$!
pids+=("$player")
set +e
wait "$player"
player_status=$?
set -e
if [[ $player_status != 0 && $player_status != 124 ]]; then
  echo "Bag player failed ($player_status); inspect player.log." >&2; exit 1
fi
echo "Replay artifacts: $out (reports finalized on observer shutdown)"
