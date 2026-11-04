#!/bin/bash
# Measures real-time factor for N iris at idle on the ground, using a throwaway
# patched copy of upstream multiagent.launch.py.
source /usr/local/bin/entrypoint.sh >/dev/null 2>&1
N=$1
python3 - "$N" <<'EOF'
import re, sys
n = int(sys.argv[1])
src = open("/opt/ardupilot_ws/src/ardupilot_gz/ardupilot_gz_bringup/launch/multiagent.launch.py").read()
rows = ",\n".join(
    '{"name": "iris_%d", "model": Vehicle.IRIS, "position": ["%d.0", "0.0", "0.195", "0", "0", "1.5708"]}' % (i + 1, i * 5)
    for i in range(n))
out, c = re.subn(r"robots = \[.*?\]\n\n    def generate_launch_actions",
                 "robots = [\n" + rows + "\n]\n\n    def generate_launch_actions", src, count=1, flags=re.S)
assert c == 1
open("/tmp/rtf_probe.launch.py", "w").write(out)
EOF
ros2 launch /tmp/rtf_probe.launch.py gui:=false rviz:=false > /tmp/rtf_$N.log 2>&1 &
for i in $(seq 1 180); do
  [ "$(grep -c 'JSON received' /tmp/rtf_$N.log)" -ge "$N" ] && break
  sleep 1
done
echo "N=$N JSON received: $(grep -c 'JSON received' /tmp/rtf_$N.log)"
sleep 25
timeout 25 gz topic -e -t /stats -n 6 2>/dev/null | grep real_time_factor | awk '{s+=$2; c++} END {if (c) printf "mean RTF %.2f over %d samples\n", s/c, c}'
free -h | sed -n 2,3p
