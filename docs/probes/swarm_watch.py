"""Watch N followers: status and goal sector per drone, goal changes, RTF.

Session 4 (2026-11-12) evidence tool for the swarm launch (T5). Prints one
line a second and a summary at the end: wall time of the first sustained
all-ARRIVED, goal changes per drone, and whether two drones ever held the same
goal sector. Usage, in the container with the workspace sourced:
  python3 docs/probes/swarm_watch.py [N] [seconds] [resolution]
"""
import math
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
res = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
robots = [f'iris_{i + 1}' for i in range(n)]

rclpy.init()
node = rclpy.create_node('swarm_watch')
rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
st = {r: {'status': None, 'goal': None, 'goal_t': None, 'pose': None,
          'changes': 0, 'sectors': []} for r in robots}
clock = {'sim': None}


def on_goal(r, m):
    sector = (math.floor(m.point.x / res), math.floor(m.point.y / res))
    s = st[r]
    if sector != s['goal']:
        if s['goal'] is not None:
            s['changes'] += 1
        s['sectors'].append(sector)
        s['goal'] = sector
    s['goal_t'] = time.time()


for r in robots:
    node.create_subscription(String, f'/{r}/planner_status',
                             lambda m, r=r: st[r].__setitem__('status', m.data), rel)
    node.create_subscription(PointStamped, f'/{r}/goal_sector',
                             lambda m, r=r: on_goal(r, m), rel)
    node.create_subscription(PoseStamped, f'/{r}/pose',
                             lambda m, r=r: st[r].__setitem__('pose', m.pose.position), rel)
node.create_subscription(
    Clock, '/clock',
    lambda m: clock.__setitem__('sim', m.clock.sec + m.clock.nanosec * 1e-9), be)

t0 = time.time()
first_all = None        # wall time all ARRIVED on distinct goals
shared = []             # (t, sector, robots) whenever two goals coincide
last_sim, last_wall = None, None
while time.time() - t0 < duration:
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    now = time.time()
    rtf = ''
    if clock['sim'] is not None:
        if last_sim is not None and now > last_wall:
            rtf = f'rtf {(clock["sim"] - last_sim) / (now - last_wall):.2f}'
        last_sim, last_wall = clock['sim'], now
    # A goal counts while it was repeated in the last 5 s (D28).
    live = {r: s['goal'] for r, s in st.items()
            if s['goal_t'] is not None and now - s['goal_t'] <= 5.0}
    by_sector = {}
    for r, g in live.items():
        by_sector.setdefault(g, []).append(r)
    for g, rs in by_sector.items():
        if len(rs) > 1:
            shared.append((now - t0, g, rs))
    all_arrived = (all(st[r]['status'] == 'ARRIVED' for r in robots)
                   and len(live) == n and len(by_sector) == n)
    if all_arrived and first_all is None:
        first_all = now - t0
    parts = []
    for r in robots:
        s, p = st[r], st[r]['pose']
        z = f'{p.z:5.1f}' if p else '  ?  '
        parts.append(f"{r[-1]}:{(s['status'] or '-')[:4]} {s['goal']} z{z}")
    print(f"t+{now - t0:5.0f}s {rtf:9} | " + ' | '.join(parts), flush=True)

print('\nsummary')
print(f'  first all-ARRIVED on distinct goals: '
      f'{"never" if first_all is None else f"t+{first_all:.0f} s wall"}')
for r in robots:
    print(f"  {r}: {st[r]['changes']} goal changes, sectors {st[r]['sectors']}")
print(f'  seconds with a shared live goal: {len(shared)}'
      + (f', first {shared[0]}' if shared else ''))
rclpy.shutdown()
