"""Watches a running swarm launch and says whether the mission got through.

Run inside the container while
  ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=1
is up (ci/smoke_test.sh does both). It only listens; it never commands.

Exit code:
  0  mission DONE, or (at the timeout) the leader's sweep COMPLETE
  1  mission TIMED_OUT, or the timeout passed without the sweep completing
  3  the sweep never started within --stall seconds: the silent /clock cold
     start (docker-notes 33) or a node that died at launch. The wrapper
     restarts the sim and tries again on this code only.

Why latched QoS: /mission_state, /mission_report, /confirmed_finds and the
sweep status are TRANSIENT_LOCAL, so a late joiner (this script) still gets
the last value, however long ago it was published.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Watch(Node):
    def __init__(self, sweep_topic):
        super().__init__('smoke_check')
        self.t0 = time.monotonic()
        self.sweep = None
        self.state = None
        self.finds = []
        self.report = None
        self.create_subscription(String, sweep_topic, self._on_sweep, LATCHED)
        self.create_subscription(String, '/mission_state', self._on_state, LATCHED)
        self.create_subscription(String, '/confirmed_finds', self._on_finds, LATCHED)
        self.create_subscription(String, '/mission_report', self._on_report, LATCHED)

    def _say(self, text):
        print(f'[{time.monotonic() - self.t0:6.1f} s] {text}', flush=True)

    def _on_sweep(self, msg):
        if msg.data != self.sweep:
            self.sweep = msg.data
            self._say(f'sweep {self.sweep}')

    def _on_state(self, msg):
        state = json.loads(msg.data)
        if state.get('state') != self.state:
            self.state = state.get('state')
            self._say(f"mission {self.state} ({state.get('reason', '')})")

    def _on_finds(self, msg):
        finds = json.loads(msg.data)
        if len(finds) != len(self.finds):
            self._say(f'finds: {[(f["cell"], f["drone"]) for f in finds]}')
        self.finds = finds

    def _on_report(self, msg):
        self.report = json.loads(msg.data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=600.0,
                    help='wall seconds to wait for the end (default 600)')
    ap.add_argument('--stall', type=float, default=240.0,
                    help='wall seconds to wait for the sweep to start')
    ap.add_argument('--min-finds', type=int, default=1,
                    help='finds a DONE mission needs to pass (default 1); an '
                         'empty arena ends DONE with none, which proves nothing')
    ap.add_argument('--sweep-topic', default='/iris_1/sweep_status')
    args = ap.parse_args()

    rclpy.init()
    w = Watch(args.sweep_topic)
    code = None
    try:
        while code is None:
            rclpy.spin_once(w, timeout_sec=0.5)
            elapsed = time.monotonic() - w.t0
            if w.state == 'DONE':
                code = 0 if len(w.finds) >= args.min_finds else 1
            elif w.state == 'TIMED_OUT':
                code = 1
            elif w.sweep in (None, 'WAITING') and elapsed > args.stall:
                w._say(f'sweep not started after {args.stall:.0f} s')
                code = 3
            elif elapsed > args.timeout:
                code = 0 if w.sweep == 'COMPLETE' else 1
    finally:
        w.destroy_node()
        rclpy.shutdown()

    n = len(w.finds)
    if w.state == 'DONE' and code == 0:
        verdict = f'PASS: mission DONE, {n} find(s)'
    elif w.state == 'DONE':
        verdict = f'FAIL: mission DONE with {n} find(s), needed {args.min_finds}'
    elif code == 0:
        verdict = f'PASS (sweep only): sweep COMPLETE, mission {w.state}, {n} find(s)'
    elif code == 3:
        verdict = 'RETRY: the sweep never started'
    else:
        verdict = (f'FAIL: mission {w.state}, sweep {w.sweep}, {n} find(s)')
    print(verdict, flush=True)
    if w.report:
        print(f"report: sim_s={w.report.get('sim_s')} wall_s={w.report.get('wall_s')} "
              f"finds={w.report.get('finds')}", flush=True)
    sys.exit(code)


if __name__ == '__main__':
    main()
