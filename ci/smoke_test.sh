#!/usr/bin/env bash
# CI smoke test: 1 leader + 1 follower, ground-truth detector, headless.
#
# Run inside the ros2 container, from anywhere, with the workspace built:
#   docker compose -f docker/docker-compose.yml exec -T ros2 bash -lc \
#     'source /usr/local/bin/entrypoint.sh; /workspaces/multi-agent-uavs/ci/smoke_test.sh'
#
# Pass = smoke_check.py exits 0 (mission DONE, or the sweep completing; the
# roadmap's CI scope). A cold start can leave ROS /clock silent (docker-notes
# 33), which shows up as "the sweep never started" (exit 3): the sim is killed
# and launched again, up to ATTEMPTS times. Any other failure is final.
#
# Needs no camera or GPU: the ground-truth detector reads Gazebo's scene, and
# the swarm launch is headless by default (gui:=false).
set -u

ATTEMPTS=${ATTEMPTS:-2}
TIMEOUT=${TIMEOUT:-600}        # wall seconds per attempt, the roadmap's 10 min
STALL=${STALL:-240}            # wall seconds for the sweep to start
LOGDIR=${LOGDIR:-/tmp/smoke}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$LOGDIR"

cleanup() {
  # pkill leaves arducopter and the XRCE agent behind unless named explicitly
  # (CLAUDE.md); a second attempt would otherwise fight the first for ports.
  pkill -INT -f 'ros2 launch uav_planning swarm.launch.py' 2>/dev/null
  sleep 5
  pkill -9 -f 'arducopter|MicroXRCEAgent|mavproxy|gz sim|ruby.*gz|swarm.launch.py' 2>/dev/null
  sleep 2
}
trap cleanup EXIT

rc=1
for attempt in $(seq 1 "$ATTEMPTS"); do
  echo "=== attempt $attempt of $ATTEMPTS ==="
  cleanup
  ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=1 \
    >"$LOGDIR/launch-$attempt.log" 2>&1 &
  # Project nodes after the sim is up, not in the same breath (gotcha 33):
  # swarm.launch.py gates its followers on /clock itself.
  # The arena starts empty; the detector needs target_* models to see.
  "$HERE/spawn_targets.sh" 120 || { echo "targets not spawned"; rc=1; break; }
  python3 -u "$HERE/smoke_check.py" --timeout "$TIMEOUT" --stall "$STALL"
  rc=$?
  [ "$rc" -ne 3 ] && break
  echo "sweep never started; the launch log tail follows"
  tail -n 15 "$LOGDIR/launch-$attempt.log"
done

echo "=== smoke test exit $rc (logs in $LOGDIR) ==="
exit "$rc"
