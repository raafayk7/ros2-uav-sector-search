#!/usr/bin/env bash
# Puts the three session-8 targets into the running Gazebo world.
#
# swarm.launch.py starts the arena empty: the ground-truth detector reads
# `target_*` models, and nothing in the launch creates them (docs/probes/
# belief_live_check.py does it for the probes). Without targets a mission ends
# DONE after the sweep with no finds, which would pass a smoke test for the
# wrong reason. Cells (1,2), (4,5), (6,1) on the 8x8 grid at 5 m.
#
#   ci/spawn_targets.sh [WAIT_SECONDS]     (default: wait up to 120 s for the world)
set -u

WAIT=${1:-120}
SERVICE=/world/runway/create

for _ in $(seq 1 "$WAIT"); do
  gz service -l 2>/dev/null | grep -q "^$SERVICE\$" && break
  sleep 1
done
gz service -l 2>/dev/null | grep -q "^$SERVICE\$" || { echo "no $SERVICE after ${WAIT}s"; exit 1; }

spawn() {   # name x y rgba
  local sdf="/tmp/$1.sdf"
  cat >"$sdf" <<EOF
<?xml version="1.0"?><sdf version="1.9"><model name="$1"><static>true</static>
<link name="l"><visual name="v"><geometry><box><size>1.5 1.5 0.1</size></box></geometry>
<material><ambient>$4</ambient><diffuse>$4</diffuse></material>
</visual></link></model></sdf>
EOF
  out=$(gz service -s "$SERVICE" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 5000 --req "sdf_filename: \"$sdf\", pose: {position: {x: $2, y: $3, z: 0.05}}, name: \"$1\"")
  echo "spawn $1 ($2, $3): $out"
  echo "$out" | grep -q "data: true"
}

spawn target_red   8.0 12.0 '1 0 0 1' &&
spawn target_green 21.0 27.0 '0 1 0 1' &&
spawn target_blue  33.0  6.0 '0 0 1 1'
