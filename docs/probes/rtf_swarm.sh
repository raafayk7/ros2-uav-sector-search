#!/bin/bash
# Real-time factor for N iris from swarm.launch.py, sim only (no followers),
# idle on the ground, sampled only after every SITL is ready (T7, D23).
#
#   bash docs/probes/rtf_swarm.sh N [extra swarm.launch.py args...]
#   LAUNCH=$(python3 docs/probes/rtf_variant.py nomav) bash docs/probes/rtf_swarm.sh 4
#
# Run inside the container under `bash -lc` (MAVProxy on PATH), one run per
# container: `docker compose restart ros2` between runs (gotcha 23).
# Ready means: N "JSON received" lines and /ap/v1 .. /ap/vN topics listed.
# Prints mean/min/max RTF over SAMPLES /stats messages, RAM and swap, the top
# processes by CPU, and each drone's gimbal joint angles.
source /usr/local/bin/entrypoint.sh >/dev/null 2>&1
source /workspaces/multi-agent-uavs/install/setup.bash
N=$1; shift
SAMPLES=${SAMPLES:-40}
SETTLE=${SETTLE:-15}
LOG=/tmp/rtf_swarm_$N.log
# Fresh SITL directory: a saved eeprom.bin beats defaults files (gotcha 40).
rm -rf /tmp/sitl_rtf && mkdir -p /tmp/sitl_rtf && cd /tmp/sitl_rtf
LAUNCH=${LAUNCH:-uav_planning swarm.launch.py}
timeout -s INT 600 ros2 launch $LAUNCH \
  num_followers:=$N followers:=false "$@" > $LOG 2>&1 &
T0=$(date +%s)

ready=""
for i in $(seq 1 240); do
  if [ "$(grep -c 'JSON received' $LOG)" -ge "$N" ]; then
    topics=$(ros2 topic list 2>/dev/null)
    ok=1
    for k in $(seq 1 $N); do
      echo "$topics" | grep -q "^/ap/v$k/" || ok=0
    done
    [ $ok = 1 ] && { ready=$(( $(date +%s) - T0 )); break; }
  fi
  sleep 1
done
echo "N=$N launch=[$LAUNCH] args=[$*] JSON received: $(grep -c 'JSON received' $LOG)/$N, ready: ${ready:-NO} s"
echo "mavproxy processes: $(pgrep -fc mavproxy.py), arducopter: $(pgrep -xc arducopter)"
[ -z "$ready" ] && exit 1
sleep $SETTLE

timeout 60 gz topic -e -t /stats -n $SAMPLES 2>/dev/null | grep real_time_factor |
  awk '{s+=$2; c++; if (min==""||$2<min) min=$2; if ($2>max) max=$2}
       END {if (c) printf "RTF mean %.3f min %.3f max %.3f over %d samples\n", s/c, min, max, c}'
free -m | sed -n 2,3p
top -b -n 2 -d 5 -w 200 | awk '/^top -/{n++} n==2' | sed -n '7,20p' | awk '{printf "  %6s%%  %s\n", $9, $12}'

for k in $(seq 1 $N); do
  timeout 15 gz topic -e -n 1 -t /world/runway/model/iris_$k/joint_state 2>/dev/null |
    python3 -c '
import math, re, sys
txt = sys.stdin.read()
out = []
for block in txt.split("joint {")[1:]:
    name = re.search(r"name: \"([^\"]+)\"", block)
    pos = re.search(r"axis1 \{.*?\n\s*position: ([-0-9.e]+)", block, re.S)
    if name and pos and name.group(1) in ("roll_joint", "pitch_joint", "yaw_joint"):
        out.append("%s %.1f" % (name.group(1)[:-6], math.degrees(float(pos.group(1)))))
print("  iris_'$k' gimbal:", ", ".join(out) or "no joints parsed")'
done
