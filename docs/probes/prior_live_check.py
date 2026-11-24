"""Watches a clue-driven mission and prints what the clue changed (session 17).

Run inside the container while
  ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=3 clue:="..."
is up (and the targets spawned, ci/spawn_targets.sh). It only listens.

Prints, in order: the prior the map was built from (raised cells), the mission
state changes, each follower's goal sector as it changes (cell = (col, row) on
the 8x8 grid, 5 m, and whether the clue raised it), the confirmed finds, and at
the end the mission report. Exit 0 on DONE, 1 on TIMED_OUT or --timeout.

  python3 -u docs/probes/prior_live_check.py [--followers iris_2,iris_3,iris_4]
"""
import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
RES = 5.0


class Watch(Node):
    def __init__(self, followers):
        super().__init__('prior_live_check')
        self.t0 = time.monotonic()
        self.raised = set()
        self.state = None
        self.goals = {f: None for f in followers}
        self.first_goals = {}
        self.done = None
        self.create_subscription(String, '/mission_prior', self._prior, LATCHED)
        self.create_subscription(String, '/mission_state', self._state, LATCHED)
        self.create_subscription(String, '/confirmed_finds', self._finds, LATCHED)
        self.create_subscription(String, '/mission_report', self._report, LATCHED)
        for f in followers:
            self.create_subscription(
                PointStamped, f'/{f}/goal_sector',
                lambda m, f=f: self._goal(f, m), RELIABLE)

    def say(self, text):
        print(f'[{time.monotonic() - self.t0:6.1f} s] {text}', flush=True)

    def _prior(self, msg):
        data = json.loads(msg.data)
        start = data['start']
        self.raised = {(c, r) for r, row in enumerate(start)
                       for c, v in enumerate(row) if v > 0.5}
        self.say(f"prior: provider {data['provider']}, failed {data['failed']}, "
                 f"{len(data['regions'])} regions, raised cells "
                 f"{sorted(self.raised)}; warnings {data['warnings']}")

    def _state(self, msg):
        s = json.loads(msg.data)
        if s.get('state') != self.state:
            self.state = s.get('state')
            self.say(f"mission {self.state} ({s.get('reason', '')})")
            if self.state in ('DONE', 'TIMED_OUT'):
                self.done = self.state

    def _goal(self, robot, msg):
        cell = (int(msg.point.x // RES), int(msg.point.y // RES))
        if cell == self.goals[robot]:
            return
        self.goals[robot] = cell
        mark = 'raised by the clue' if cell in self.raised else 'not raised'
        self.say(f'{robot} goal {cell} ({mark})')
        if robot not in self.first_goals and self.state == 'SEARCHING':
            self.first_goals[robot] = cell

    def _finds(self, msg):
        self.say(f'confirmed finds: {msg.data}')

    def _report(self, msg):
        self.say(f'report: {msg.data}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--followers', default='iris_2,iris_3,iris_4')
    ap.add_argument('--timeout', type=float, default=900.0, help='wall seconds')
    args = ap.parse_args()
    rclpy.init()
    node = Watch(args.followers.split(','))
    try:
        while rclpy.ok() and node.done is None \
                and time.monotonic() - node.t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
        rclpy.spin_once(node, timeout_sec=1.0)      # the latched report
    finally:
        node.say(f"first goals once SEARCHING began: {node.first_goals}; "
                 f"final state {node.done}")
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if node.done == 'DONE' else 1)


if __name__ == '__main__':
    main()
