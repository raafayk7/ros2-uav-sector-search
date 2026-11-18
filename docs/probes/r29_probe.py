"""Offline R29 probe: replay the session-8 sweep with D38 noise, compare rules.

The evidence behind D40 (session 9 notes). Pure Python + numpy, run from
anywhere: python3 docs/probes/r29_probe.py. Not project code. Uses uav_planning.core.sweep (lane plan) and
uav_perception.core (projection + noise) from the repo.
"""
import math
import sys
from collections import defaultdict

import numpy as np

REPO = str(__import__('pathlib').Path(__file__).resolve().parents[2] / 'src')
sys.path[:0] = [f'{REPO}/uav_planning', f'{REPO}/uav_perception']
from uav_planning.core.planner import GridSpec  # noqa: E402
from uav_planning.core.sweep import Camera, lane_half_width, plan_sweep  # noqa: E402
from uav_perception.core.noise import NoiseModel, frame_detections  # noqa: E402
from uav_perception.core.projection import NADIR_QUAT, camera_pose  # noqa: E402

GRID = GridSpec(width=8, height=8, resolution=5.0, origin_x=0.0, origin_y=0.0)
CAM = Camera(205.47, 205.47, 320.0, 240.0, 640, 480)
K = (CAM.fx, CAM.fy, CAM.cx, CAM.cy)
SIZE = (CAM.width, CAM.height)
TARGETS = [(8.0, 12.0), (21.0, 27.0), (33.0, 6.0)]
L = lambda p: math.log(p / (1 - p))  # noqa: E731
L_OCC, L_FREE, L_MAX = L(0.7), L(0.45), L(0.97)
FPS = 2.0


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def pix_to_ground(u, v, cam_pos, R):
    d = R @ np.array([(u - K[2]) / K[0], (v - K[3]) / K[1], 1.0])
    if abs(d[2]) < 1e-9:
        return None
    t = -cam_pos[2] / d[2]
    if t <= 0:
        return None
    h = cam_pos + t * d
    return h[0], h[1]


def in_quad(px, py, q):
    inside, j = False, len(q) - 1
    for i in range(len(q)):
        xi, yi = q[i]
        xj, yj = q[j]
        if ((yi > py) != (yj > py)) and px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def footprint_cells(cam_pos, R):
    q = [pix_to_ground(u, v, cam_pos, R) for u, v in
         [(0, 0), (CAM.width, 0), (CAM.width, CAM.height), (0, CAM.height)]]
    if any(p is None for p in q):
        return set()
    out = set()
    for r in range(GRID.height):
        for c in range(GRID.width):
            if in_quad((c + .5) * 5, (r + .5) * 5, q):
                out.add((c, r))
    return out


def cell_of(x, y):
    c, r = int(math.floor(x / 5)), int(math.floor(y / 5))
    return (c, r) if 0 <= c < 8 and 0 <= r < 8 else None


def leader_track(speed=5.0, climb=1.5, dwell=1.5, end_hover=5.0):
    """(t, x, y, z, yaw) at FPS: climb at home, fly the D39 sweep, hover."""
    hw = lane_half_width(CAM, 8.0, 0.8)
    wps = plan_sweep(GRID, hw, (0.0, 0.0))
    out, t, dt = [], 0.0, 1 / FPS
    z = 0.2
    while z < 8.0:
        out.append((t, 0.0, 0.0, z, math.pi / 2))
        z += climb * dt
        t += dt
    here = (0.0, 0.0)
    yaw = math.pi / 2
    for wp in wps:
        d = math.dist(here, wp)
        if d > 1e-6:
            yaw = math.atan2(wp[1] - here[1], wp[0] - here[0])
            n = max(1, int(d / (speed * dt)))
            for k in range(n):
                f = (k + 1) / n
                out.append((t, here[0] + f * (wp[0] - here[0]),
                            here[1] + f * (wp[1] - here[1]), 8.0, yaw))
                t += dt
        for _ in range(int(dwell * FPS)):
            out.append((t, wp[0], wp[1], 8.0, yaw))
            t += dt
        here = wp
    for _ in range(int(end_hover * FPS)):
        out.append((t, here[0], here[1], 8.0, yaw))
        t += dt
    return out


def frames(track, targets, rng, noise=NoiseModel()):
    """Per frame: (t, observed cells, {cell: best hit score})."""
    out = []
    for t, x, y, z, yaw in track:
        cp, R = camera_pose((x, y, z), yaw_quat(yaw), NADIR_QUAT)
        seen = footprint_cells(cp, R)
        hits = {}
        for d in frame_detections(targets, cp, R, K, SIZE, noise, rng):
            if d.score < 0.25:
                continue
            g = pix_to_ground(d.u, d.v, cp, R)
            c = g and cell_of(*g)
            if c:
                hits[c] = hits.get(c, 0) + d.score * L_OCC   # leader_node sums per box
        out.append((t, seen, hits))
    return out


# ---------------------------------------------------------------- rules
class PerFrame:
    """R0, today's leader_node: every frame counts fully."""
    name = 'R0 per frame (today)'

    def __init__(self, w=1.0):
        self.w = w
        self.l = {}

    def step(self, t, seen, hits):
        for c in seen | set(hits):
            d = hits[c] if c in hits else (L_FREE if c in seen else 0)
            self.l[c] = float(np.clip(self.l.get(c, 0.0) + self.w * d, -L_MAX, L_MAX))


class Weighted(PerFrame):
    def __init__(self, w):
        super().__init__(w)
        self.name = f'R3 weight {w} per frame'


class RateLimit(PerFrame):
    """R1: one update per cell per drone every tau s."""

    def __init__(self, tau=1.0):
        super().__init__()
        self.tau, self.last = tau, {}
        self.name = f'R1 rate limit {1 / tau:g} Hz per cell'

    def step(self, t, seen, hits):
        keep = {c for c in seen | set(hits) if t - self.last.get(c, -1e9) >= self.tau - 1e-9}
        for c in keep:
            self.last[c] = t
        super().step(t, seen & keep, {c: h for c, h in hits.items() if c in keep})


class LookCap(PerFrame):
    """R2: frames add as today, but one look's total is capped at +-cap.
    A look ends when the cell is out of view for > gap s, or after max_look s
    (so a hover becomes one new look every max_look s)."""

    def __init__(self, cap=L(0.7), gap=1.0, max_look=4.0):
        super().__init__()
        self.cap, self.gap, self.max_look = cap, gap, max_look
        self.look = {}   # cell -> [start, last_seen, base_l, acc]
        self.name = f'R2 look cap +-{cap:.2f}, new look every {max_look:g} s'

    def step(self, t, seen, hits):
        for c in seen | set(hits):
            d = hits[c] if c in hits else L_FREE
            lk = self.look.get(c)
            if lk is None or t - lk[1] > self.gap or t - lk[0] >= self.max_look:
                lk = [t, t, self.l.get(c, 0.0), 0.0]
                self.look[c] = lk
            lk[1] = t
            lk[3] = float(np.clip(lk[3] + d, -self.cap, self.cap))
            self.l[c] = float(np.clip(lk[2] + lk[3], -L_MAX, L_MAX))


def pct(l):
    return 100 / (1 + math.exp(-l))


def run(rule_factory, seeds=20):
    target_cells = {cell_of(*t) for t in TARGETS}
    track = leader_track()
    tv, edge, mid, emax, unknown = [], [], [], [], []
    for s in range(seeds):
        rng = np.random.default_rng(s)
        rule = rule_factory()
        for t, seen, hits in frames(track, TARGETS, rng):
            rule.step(t, seen, hits)
        for r in range(8):
            for c in range(8):
                if (c, r) in target_cells:
                    tv.append(pct(rule.l[(c, r)]))
                    continue
                if (c, r) not in rule.l:
                    unknown.append(1)
                    continue
                p = pct(rule.l[(c, r)])
                (edge if c in (0, 7) else mid).append(p)
        emax.append(max(pct(rule.l[k]) for k in rule.l if k not in target_cells))
    f = lambda a: f'{np.median(a):5.1f} [{np.percentile(a, 10):4.1f}-{np.percentile(a, 90):4.1f}]'  # noqa
    print(f'{rule.name:42s} targets {f(tv)}  edge cols {f(edge)}  middle {f(mid)}  '
          f'empty max (median over seeds) {np.median(emax):4.1f}  unknown {len(unknown)}')
    return rule


def coverage_counts():
    track = leader_track()
    fr = frames(track, [], np.random.default_rng(0), NoiseModel(p_false=0.0))
    nframes = defaultdict(int)
    looks = LookCap()
    for t, seen, hits in fr:
        for c in seen:
            nframes[c] += 1
    # look count
    nlooks, last, start = defaultdict(int), {}, {}
    for t, seen, _ in fr:
        for c in seen:
            if c not in last or t - last[c] > looks.gap or t - start[c] >= looks.max_look:
                nlooks[c] += 1
                start[c] = t
            last[c] = t
    print(f'track: {len(track)} frames, {track[-1][0]:.0f} s')
    print('frames per cell (row 7 at top):')
    for r in range(7, -1, -1):
        print('   ' + ' '.join(f'{nframes[(c, r)]:3d}' for c in range(8)))
    print('looks per cell (gap 1 s, 4 s max look):')
    for r in range(7, -1, -1):
        print('   ' + ' '.join(f'{nlooks[(c, r)]:3d}' for c in range(8)))


def hover_tests(rule_factories, seeds=200):
    """A follower hovering at 10 m over sector centre (17.5, 17.5).
    (a) empty sector with a prior of 0.85: seconds until it reads below 0.5.
    (b) target at the centre, start unknown: seconds until N consecutive hit
        frames in that sector (D32), for N = 1..4.
    (c) empty sector: false confirmations per hour of hover for N = 1..3."""
    cell = (3, 3)
    cp, R = camera_pose((17.5, 17.5, 10.0), yaw_quat(math.pi / 2), NADIR_QUAT)
    print('\n(a) follower hovering on an empty sector with prior 0.85: seconds to < 0.5')
    for fac in rule_factories:
        ts = []
        for s in range(seeds):
            rng = np.random.default_rng(1000 + s)
            rule = fac()
            rule.l[cell] = L(0.85)
            t = 0.0
            while pct(rule.l[cell]) >= 50 and t < 300:
                seen = footprint_cells(cp, R)
                hits = {}
                for d in frame_detections([], cp, R, K, SIZE, NoiseModel(), rng):
                    g = pix_to_ground(d.u, d.v, cp, R)
                    c = g and cell_of(*g)
                    if c and d.score >= 0.25:
                        hits[c] = hits.get(c, 0) + d.score * L_OCC
                rule.step(t, seen, hits)
                t += 1 / FPS
            ts.append(t)
        print(f'   {fac().name:42s} median {np.median(ts):5.1f} s, p90 {np.percentile(ts, 90):5.1f} s')

    print('\n(b) hovering over a target: seconds to N consecutive hit frames in the sector')
    for n in (1, 2, 3, 4):
        ts = []
        for s in range(seeds):
            rng = np.random.default_rng(5000 + s)
            run_, t = 0, 0.0
            while run_ < n:
                hit = any(cell_of(*pix_to_ground(d.u, d.v, cp, R)) == cell
                          for d in frame_detections([(17.5, 17.5)], cp, R, K, SIZE,
                                                    NoiseModel(), rng) if d.score >= 0.25)
                run_ = run_ + 1 if hit else 0
                t += 1 / FPS
            ts.append(t)
        print(f'   N={n}: median {np.median(ts):4.1f} s, p90 {np.percentile(ts, 90):4.1f} s')

    print('\n(c) hovering over an empty sector: false-alarm frames in the sector')
    rng = np.random.default_rng(9)
    nf = 200_000
    hit = np.zeros(nf, bool)
    for i in range(nf):
        if rng.random() >= 0.05:      # fast path: no false box this frame
            continue
        u, v = rng.uniform(0, 640), rng.uniform(0, 480)
        hit[i] = cell_of(*pix_to_ground(u, v, cp, R)) == cell
    p1 = hit.mean()
    print(f'   P(false box in own sector per frame) = {p1:.4f} ({hit.sum()} of {nf})')
    for n in (1, 2, 3):
        per_hour = p1 ** n * FPS * 3600
        print(f'   N={n}: about {per_hour:.3g} false confirmations per hour of hover')


if __name__ == '__main__':
    coverage_counts()
    print()
    for fac in (PerFrame, lambda: RateLimit(1.0), lambda: Weighted(0.25),
                lambda: LookCap(L(0.7), 1.0, 4.0), lambda: LookCap(L(0.8), 1.0, 4.0),
                lambda: LookCap(1.0, 1.0, 2.0)):
        run(fac)
    hover_tests([PerFrame, lambda: RateLimit(1.0), lambda: Weighted(0.25),
                 lambda: LookCap(L(0.7), 1.0, 4.0), lambda: LookCap(L(0.8), 1.0, 4.0)])
    # Frame-rate dependence: a 2 Hz wall-clock detector gives 4 frames per
    # sim second at RTF 0.5.
    for fps in (1.0, 4.0):
        FPS = fps
        print(f'\n--- {fps:g} frames per sim second')
        for fac in (PerFrame, lambda: LookCap(L(0.7), 1.0, 4.0)):
            run(fac, seeds=10)
