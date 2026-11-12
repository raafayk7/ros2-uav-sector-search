"""Silence one follower and time how long its sector takes to be released.

Session 4 (2026-11-12) evidence for T5. Against a running 3-follower swarm
with every drone ARRIVED:
  1. kill ROBOT's follower_planner_node (its goal_sector stops) and watch
     the others for HOLD_S sim seconds;
  2. kill ROBOT's localization_node too (its pose stops) and time, in sim
     seconds, until another drone's goal_sector names ROBOT's old sector.
Usage, in the container with the workspace sourced:
  python3 docs/probes/peer_release.py [ROBOT] [N] [HOLD_S] [resolution]
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from rosgraph_msgs.msg import Clock

robot = sys.argv[1] if len(sys.argv) > 1 else 'iris_1'
n = int(sys.argv[2]) if len(sys.argv) > 2 else 3
hold_s = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
res = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
robots = [f'iris_{i + 1}' for i in range(n)]

rclpy.init()
node = rclpy.create_node('peer_release')
rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
goal = {}
sim = {'t': None}
log = []


def on_goal(r, m):
    s = (math.floor(m.point.x / res), math.floor(m.point.y / res))
    if goal.get(r) != s:
        log.append((sim['t'], r, goal.get(r), s))
        print(f'  sim {sim["t"]:8.2f}  {r}: {goal.get(r)} -> {s}', flush=True)
    goal[r] = s


for r in robots:
    node.create_subscription(PointStamped, f'/{r}/goal_sector',
                             lambda m, r=r: on_goal(r, m), rel)
node.create_subscription(
    Clock, '/clock',
    lambda m: sim.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), be)


def spin_sim(seconds, until=None):
    """Spin until ``seconds`` of sim time pass or ``until()`` is true."""
    while sim['t'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    end = sim['t'] + seconds
    while sim['t'] < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if until is not None and until():
            return True
    return False


spin_sim(3.0)
target = goal.get(robot)
print(f'goals before: {goal}; {robot} holds {target}')

wall0, sim0 = time.time(), sim['t']
subprocess.run(['pkill', '-f', f'follower_planner_node.*__ns:=/{robot}'])
print(f'step 1: killed {robot} planner at sim {sim0:.2f}')
others = [r for r in robots if r != robot]
taken = lambda: any(goal.get(r) == target for r in others)
moved = spin_sim(hold_s, until=taken)
print(f'step 1: after {sim["t"] - sim0:.1f} sim s ({time.time() - wall0:.1f} wall): '
      f'{"RELEASED (another drone took it)" if moved else "still blocked"}; goals {goal}')

wall1, sim1 = time.time(), sim['t']
subprocess.run(['pkill', '-f', f'localization_node.*__ns:=/{robot}'])
print(f'step 2: killed {robot} localization at sim {sim1:.2f}')
moved = spin_sim(60.0, until=taken)
print(f'step 2: {"released" if moved else "NOT released"} after '
      f'{sim["t"] - sim1:.2f} sim s ({time.time() - wall1:.1f} wall)')
spin_sim(30.0)
print(f'goals after 30 more sim s: {goal}')
rclpy.shutdown()
