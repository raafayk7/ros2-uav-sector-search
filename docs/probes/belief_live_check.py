"""Live check of belief_map_node with a leader and followers (session 9).

Spawns the session-8 targets, then once a second logs the leader's sweep
status, each follower's planner status, goal and height, confirmations and
how many cells are known. Every 10 s it samples Gazebo's real-time factor.
At the end it prints the map with target cells marked, per-drone frame
counts and the RTF summary.

Run inside the container after
  swarm.launch.py leader:=true num_followers:=N
is up (the sim must exist to spawn into; the drones need not be armed yet):

  python3 belief_live_check.py [SECS] [N] [--no-spawn]
"""
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
SECS = float(ARGS[0]) if ARGS else 240.0
N = int(ARGS[1]) if len(ARGS) > 1 else 1
SPAWN = '--no-spawn' not in sys.argv
LEADER = 'iris_1'
FOLLOWERS = [f'iris_{k + 2}' for k in range(N)]
TARGETS = {   # session 8's three, one per lane, in cells (1, 2), (4, 5), (6, 1)
    'target_red': (8.0, 12.0, '1 0 0 1'),
    'target_green': (21.0, 27.0, '0 1 0 1'),
    'target_blue': (33.0, 6.0, '0 0 1 1'),
}
SDF = """<?xml version="1.0"?><sdf version="1.9"><model name="{n}"><static>true</static>
<link name="l"><visual name="v"><geometry><box><size>1.5 1.5 0.1</size></box></geometry>
<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>
</visual></link></model></sdf>"""


def spawn():
    for name, (x, y, c) in TARGETS.items():
        path = f'/tmp/{name}.sdf'
        with open(path, 'w') as f:
            f.write(SDF.format(n=name, c=c))
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/runway/create',
             '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req',
             f'sdf_filename: "{path}", pose: {{position: {{x: {x}, y: {y}, z: 0.05}}}}, name: "{name}"'],
            capture_output=True, text=True)
        print('spawn', name, (x, y), r.stdout.strip() or r.stderr.strip(), flush=True)


class Watch(Node):
    def __init__(self):
        super().__init__('belief_live_check')
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid = None
        self.confirmed = []
        self.sweep = None
        self.status, self.goal, self.pose, self.frames = {}, {}, {}, {}
        self.create_subscription(OccupancyGrid, '/heatmap',
                                 lambda m: setattr(self, 'grid', m), latched)
        self.create_subscription(PoseArray, '/confirmed_sectors', self._conf, latched)
        self.create_subscription(String, f'/{LEADER}/sweep_status',
                                 lambda m: setattr(self, 'sweep', m.data), latched)
        for r in [LEADER] + FOLLOWERS:
            self.frames[r] = 0
            self.create_subscription(PoseStamped, f'/{r}/pose',
                                     lambda m, r=r: self.pose.__setitem__(r, m), rel)
            self.create_subscription(Detection2DArray, f'/{r}/detections',
                                     lambda m, r=r: self._frame(r), rel)
        for r in FOLLOWERS:
            self.create_subscription(String, f'/{r}/planner_status',
                                     lambda m, r=r: self.status.__setitem__(r, m.data), rel)
            self.create_subscription(PointStamped, f'/{r}/goal_sector',
                                     lambda m, r=r: self.goal.__setitem__(r, m), rel)

    def _frame(self, r):
        self.frames[r] += 1

    def _conf(self, m):
        self.confirmed = [(p.position.x, p.position.y) for p in m.poses]

    def cell(self, x, y):
        return (int(math.floor(x / 5.0)), int(math.floor(y / 5.0)))


RTF = []


def rtf_loop(stop):
    while not stop.is_set():
        try:
            out = subprocess.run(['gz', 'topic', '-e', '-t', '/stats', '-n', '3'],
                                 capture_output=True, text=True, timeout=15).stdout
            vals = [float(l.split()[1]) for l in out.splitlines()
                    if l.strip().startswith('real_time_factor')]
            if vals:
                RTF.append((time.time(), sum(vals) / len(vals)))
        except (subprocess.TimeoutExpired, ValueError, IndexError):
            pass
        stop.wait(10.0)


def main():
    rclpy.init()
    w = Watch()
    if SPAWN:
        spawn()
    stop = threading.Event()
    threading.Thread(target=rtf_loop, args=(stop,), daemon=True).start()
    t0 = time.time()
    last_line = None
    events = []
    while time.time() - t0 < SECS:
        rclpy.spin_once(w, timeout_sec=0.2)
        t = time.time() - t0
        known = sum(v >= 0 for v in w.grid.data) if w.grid else 0
        parts = [f'sweep {w.sweep}', f'known {known}', f'conf {len(w.confirmed)}']
        for r in FOLLOWERS:
            p = w.pose.get(r)
            g = w.goal.get(r)
            gs = w.cell(g.point.x, g.point.y) if g else None
            z = f'{p.pose.position.z:.0f}' if p else '-'   # whole m: fewer lines
            parts.append(f'{r} {w.status.get(r)} goal {gs} z {z}')
        line = ' | '.join(parts)
        if line != last_line:
            print(f'{t:6.1f}s  {line}', flush=True)
            events.append((t, line))
            last_line = line
    stop.set()

    print('\nframes per drone:', w.frames)
    print('confirmed sector centres:', w.confirmed)
    tcells = {w.cell(x, y) for x, y, _ in TARGETS.values()}
    if w.grid:
        print('final /heatmap (row 7 at top; * = target cell):')
        d = list(w.grid.data)
        for row in range(7, -1, -1):
            print('   ' + ' '.join(
                f"{d[row * 8 + col]:3d}{'*' if (col, row) in tcells else ' '}"
                for col in range(8)))
        empties = [d[r * 8 + c] for r in range(8) for c in range(8)
                   if (c, r) not in tcells and d[r * 8 + c] >= 0]
        print(f'empty cells known {len(empties)} of 61, above 50: '
              f'{sum(v > 50 for v in empties)}, median {sorted(empties)[len(empties) // 2] if empties else None}')
    if RTF:
        v = [x for _, x in RTF]
        print(f'RTF samples {len(v)}: min {min(v):.2f} median {sorted(v)[len(v) // 2]:.2f} '
              f'max {max(v):.2f}; series {[round(x, 2) for x in v]}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
