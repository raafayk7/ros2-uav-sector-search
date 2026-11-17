"""Live checks during one leader sweep (session 8, T8 verify-first).

Spawns coloured target_* boxes on the ground (the ground-truth detector sees
them as targets, the camera sees them as colours), then for SECS seconds:

1. Pointing in motion: in every camera frame, each colour's centroid against
   the pixel a nadir camera would see from the odometry pose interpolated at
   the frame's own stamp. Hover gave 1-4 px (T6); the question is whether the
   gimbal holds nadir while the drone accelerates, banks and turns.
2. Detection lag: each /detections box projected to the ground with the
   latest /pose, as leader_node does, against the nearest true target.
3. The heatmap at the end, with target cells and unknown cells marked.

Run inside the container after the sim, sweep, detector and leader_node are
up (docs/planner-decisions.md, session 8 notes):

  python3 sweep_live_check.py [SECS] [--no-spawn]
"""
import bisect
from collections import Counter
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from uav_perception.core.projection import (NADIR_QUAT, camera_pose,
                                            project_ground_point)

R = 'iris_1'
SECS = float(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1][0].isdigit() else 150.0
SPAWN = '--no-spawn' not in sys.argv
SIZE = 1.5   # box side, m
MIN_Z = 5.0  # m: frames below this are on the ground or climbing, not hover
FAR = 3.0    # m: a detection farther than this from every target is a false alarm
TARGETS = {
    # name: (arena x, y, rgba). One per lane, in three different cells.
    'target_red': (8.0, 12.0, '1 0 0 1'),
    'target_green': (21.0, 27.0, '0 1 0 1'),
    'target_blue': (33.0, 6.0, '0 0 1 1'),
}
SDF = """<?xml version="1.0"?><sdf version="1.9"><model name="{n}"><static>true</static>
<link name="l"><visual name="v"><geometry><box><size>{s} {s} 0.1</size></box></geometry>
<material><ambient>{c}</ambient><diffuse>{c}</diffuse><emissive>{c}</emissive></material>
</visual></link></model></sdf>"""


def secs(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def tilt_of(q):
    """Angle between body z and world z, degrees; q = (x, y, z, w)."""
    x, y = q[0], q[1]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


class Check(Node):
    def __init__(self):
        super().__init__('sweep_live_check')
        self.K = None
        self.odom_t, self.odom = [], []     # sim stamp, (pos, quat)
        self.pose = None                    # latest arena pose (as leader_node)
        self.poses = []                     # (wall, x, y) for speed
        self.pix = []                       # (speed, tilt, colour, err px, off-centre px)
        self.det = []                       # (speed, ground err m)
        self.status = []
        self.yaws = []                      # pose yaw (deg ENU) while cruising
        self.grid = None
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, f'/{R}/camera/image', self._img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f'/{R}/camera/camera_info', self._ci,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{R}/odometry', self._od, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{R}/pose', self._pose, rel)
        self.create_subscription(Detection2DArray, '/detections', self._det, rel)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/heatmap',
                                 lambda m: setattr(self, 'grid', m), latched)
        self.create_subscription(String, f'/{R}/sweep_status',
                                 lambda m: self.status.append((time.time(), m.data)), latched)

    def _ci(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _od(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom_t.append(secs(m.header.stamp))
        self.odom.append(((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def _pose(self, m):
        self.pose = m
        self.poses.append((time.time(), m.pose.position.x, m.pose.position.y))
        q = m.pose.orientation
        if self.speed() > 3.0:
            self.yaws.append(math.degrees(math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    def speed(self):
        """Ground speed over the last ~0.5 s of /pose, m/s."""
        if len(self.poses) < 2:
            return 0.0
        t1, x1, y1 = self.poses[-1]
        for t0, x0, y0 in reversed(self.poses):
            if t1 - t0 >= 0.5:
                return math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
        return 0.0

    def odom_at(self, t):
        """Odometry pose linearly interpolated at sim time t (nearest quat)."""
        i = bisect.bisect_left(self.odom_t, t)
        if i == 0 or i >= len(self.odom_t):
            return None
        t0, t1 = self.odom_t[i - 1], self.odom_t[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        p0, p1 = np.array(self.odom[i - 1][0]), np.array(self.odom[i][0])
        q = self.odom[i if a >= 0.5 else i - 1][1]
        return p0 + a * (p1 - p0), q, t1 - t0

    def _img(self, m):
        if self.K is None:
            return
        od = self.odom_at(secs(m.header.stamp))
        if od is None:
            return
        pos, q, _ = od
        if pos[2] < MIN_Z:
            return  # on the ground: run 3 counted coloured pixels seen there
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        rgb = arr[:, :, :3] if m.encoding.startswith('rgb') else arr[:, :, 2::-1]
        r, g, b = (rgb[:, :, i].astype(int) for i in range(3))
        masks = {'target_red': (r > 150) & (g < 80) & (b < 80),
                 'target_green': (g > 150) & (r < 80) & (b < 80),
                 'target_blue': (b > 150) & (r < 80) & (g < 80)}
        cam, Rw = camera_pose(tuple(pos), q, NADIR_QUAT)
        fx, fy, cx, cy = self.K
        spd, tilt = self.speed(), tilt_of(q)
        for name, (x, y, _) in TARGETS.items():
            ys, xs = np.nonzero(masks[name])
            if len(xs) < 30:
                continue
            # Skip boxes cut by the image edge: their centroid is biased.
            if xs.min() == 0 or ys.min() == 0 or xs.max() == m.width - 1 or ys.max() == m.height - 1:
                continue
            pred = project_ground_point(x, y, cam, Rw, fx, fy, cx, cy)
            if pred is None:
                continue
            obs = (xs.mean(), ys.mean())
            self.pix.append((spd, tilt, name, math.hypot(pred[0] - obs[0], pred[1] - obs[1]),
                             math.hypot(obs[0] - cx, obs[1] - cy)))

    def _det(self, m):
        if self.pose is None or self.K is None:
            return
        p, q = self.pose.pose.position, self.pose.pose.orientation
        cam, Rw = camera_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), NADIR_QUAT)
        fx, fy, cx, cy = self.K
        for d in m.detections:
            if max((h.hypothesis.score for h in d.results), default=0) < 0.5:
                continue    # D38 false alarms score 0.30-0.70; hits 0.60-0.95
            u, v = d.bbox.center.position.x, d.bbox.center.position.y
            ray = Rw @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            if ray[2] >= 0:
                continue
            g = cam + ray * (-cam[2] / ray[2])
            err = min(math.hypot(g[0] - x, g[1] - y) for x, y, _ in TARGETS.values())
            self.det.append((self.speed(), err))


def spawn():
    for name, (x, y, c) in TARGETS.items():
        path = f'/tmp/{name}.sdf'
        with open(path, 'w') as f:
            f.write(SDF.format(n=name, s=SIZE, c=c))
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/runway/create',
             '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req',
             f'sdf_filename: "{path}", pose: {{position: {{x: {x}, y: {y}, z: 0.05}}}}, name: "{name}"'],
            capture_output=True, text=True)
        print('spawn', name, (x, y), r.stdout.strip() or r.stderr.strip(), flush=True)


def stats(vals):
    if not vals:
        return 'n=0'
    a = np.array(vals)
    return 'n=%d median %.1f p90 %.1f max %.1f' % (len(a), np.median(a), np.percentile(a, 90), a.max())


if SPAWN:
    spawn()
rclpy.init()
n = Check()
end = time.time() + SECS
last = 0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last > 10:
        last = time.time()
        s = n.status[-1][1] if n.status else '?'
        p = n.pose.pose.position if n.pose else None
        print('t-%3.0f s  status %-8s  pose %s  speed %.1f  frames %d  dets %d' % (
            end - time.time(), s, '?' if p is None else '(%.1f, %.1f, %.1f)' % (p.x, p.y, p.z),
            n.speed(), len(n.pix), len(n.det)), flush=True)
    if n.status and n.status[-1][1] == 'COMPLETE' and time.time() - n.status[-1][0] > 15:
        break   # let the last frames reach the map, then stop

print('\n== 1. pointing: marker pixel error vs nadir prediction at the frame stamp ==')
print('odometry period: %.1f ms' % (1000 * np.median(np.diff(n.odom_t)) if len(n.odom_t) > 1 else float('nan')))
for lo, hi, label in ((0, 0.5, 'hover <0.5 m/s'), (0.5, 3.0, 'accel 0.5-3'), (3.0, 99, 'cruise >3 m/s')):
    print('%-15s %s' % (label, stats([e for s, _, _, e, _ in n.pix if lo <= s < hi])))
for lo, hi in ((0, 3), (3, 8), (8, 90)):
    print('tilt %2d-%2d deg  %s' % (lo, hi, stats([e for _, t, _, e, _ in n.pix if lo <= t < hi])))
print('max ground speed seen %.1f m/s' % max((s for s, *_ in n.pix), default=0))
print('cruise heading, deg ENU (10 deg bin: samples):', Counter(
    int(round(y / 10.0)) * 10 for y in n.yaws).most_common(6))
print('\n== 2. detection lag: ground error of score>=0.5 boxes, latest /pose ==')
print('(over %.0f m from every target = a false alarm; 5 px noise is ~0.2 m)' % FAR)
for lo, hi, label in ((0, 0.5, 'hover'), (0.5, 3.0, 'accel'), (3.0, 99, 'cruise')):
    errs = [e for s, e in n.det if lo <= s < hi]
    print('%-7s hits %s m | false alarms %d' % (
        label, stats([e for e in errs if e <= FAR]), sum(e > FAR for e in errs)))
print('\n== 3. heatmap (row 0 = south; T = target cell; ? = unknown) ==')
g = n.grid
if g is None:
    print('no heatmap')
else:
    res, W, H = g.info.resolution, g.info.width, g.info.height
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    data = np.array(g.data).reshape(H, W)
    cells = {(int((x - ox) // res), int((y - oy) // res)) for x, y, _ in TARGETS.values()}
    for row in range(H - 1, -1, -1):
        print('  '.join('%3d%s' % (data[row, c], 'T' if (c, row) in cells else ' ')
                        for c in range(W)))
    tv = [data[r, c] for c, r in cells]
    ev = [data[r, c] for r in range(H) for c in range(W) if (c, r) not in cells]
    print('target cells %s | empty cells: unknown %d, >50 %d, max %d, median %.0f' % (
        tv, sum(v < 0 for v in ev), sum(v > 50 for v in ev), max(ev), np.median(ev)))
print('sweep status history:', [s for _, s in n.status])
rclpy.shutdown()
