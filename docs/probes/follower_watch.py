"""Print one follower's pose, planner status and goal once a second.

Session 2 (2026-11-08) evidence tool for the first planner flight. Usage, in
the container with the workspace sourced:
  python3 docs/probes/follower_watch.py iris_1 1 [seconds]
"""
import math
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String

robot, sysid = sys.argv[1], int(sys.argv[2])
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0

rclpy.init()
n = rclpy.create_node('follower_watch')
s = {}
rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(PoseStamped, f'/{robot}/pose', lambda m: s.__setitem__('pose', m), rel)
n.create_subscription(String, f'/{robot}/planner_status', lambda m: s.__setitem__('status', m.data), rel)
n.create_subscription(PointStamped, f'/{robot}/goal_sector', lambda m: s.__setitem__('goal', m), rel)
n.create_subscription(GeoPointStamped, f'/ap/v{sysid}/goal_lla', lambda m: s.__setitem__('lla', m), latched)

t0 = time.time()
while time.time() - t0 < duration:
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    p, g = s.get('pose'), s.get('goal')
    pos = p.pose.position if p else None
    line = f"t+{time.time() - t0:5.0f}s sim {p.header.stamp.sec if p else '?':>5} "
    line += f"pose ({pos.x:6.2f}, {pos.y:6.2f}, {pos.z:5.2f})" if pos else "pose none"
    line += f" | {s.get('status', 'no status'):9}"
    if g:
        d = math.hypot(pos.x - g.point.x, pos.y - g.point.y) if pos else float('nan')
        line += f" | goal ({g.point.x:.1f}, {g.point.y:.1f}) stamp {g.header.stamp.sec} dist {d:.2f} m"
    else:
        line += " | no goal"
    print(line, flush=True)
rclpy.shutdown()
