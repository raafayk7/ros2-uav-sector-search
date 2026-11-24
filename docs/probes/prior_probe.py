"""Session 16 probe (no ROS, no sim): the open language-prior questions.

  python3 docs/probes/prior_probe.py raster     rasterization rules
  python3 docs/probes/prior_probe.py mission    alpha and the outside level
  python3 docs/probes/prior_probe.py feasible   wrong-landmark availability

`mission` is a 2D point-mass run (2 Hz frames, 5 m/s, perfect tracking, no
climb) of the real planner, sweep, detector-noise and belief code. It is a
design probe, not the Dec 20 eval: one region shape, invented landmarks, no
Gazebo. It writes the prior into BeliefMap.logodds directly, so cells outside
the regions are only left unknown when the outside level is 0.5.
"""
import math
import statistics as st
import sys

import numpy as np

sys.path[:0] = ['src/uav_mapping', 'src/uav_planning', 'src/uav_perception']
from uav_mapping.core.belief import BeliefMap, BeliefParams, logit, project_frame
from uav_mapping.core.geometry import Camera, Grid, camera_pose, NADIR_QUAT
from uav_perception.core.noise import NoiseModel, frame_detections
from uav_planning.core.mission import open_cells
from uav_planning.core.planner import (GridSpec, Peer, Sector, plan_step,
                                       values_from_occupancy)
from uav_planning.core.sweep import lane_half_width, plan_sweep

CAM = Camera()
LEVEL = (0.0, 0.0, 0.0, 1.0)
N = 20                                  # samples per cell side


def samples(grid):
    r = grid.resolution
    s = (np.arange(N) + 0.5) / N * r
    xs = grid.origin_x + np.arange(grid.width)[:, None] * r + s
    ys = grid.origin_y + np.arange(grid.height)[:, None] * r + s
    return xs, ys


def overlap(grid, c, radius):
    """Fraction of each cell inside a circle, [row, col]."""
    xs, ys = samples(grid)
    out = np.zeros((grid.height, grid.width))
    for row in range(grid.height):
        for col in range(grid.width):
            dx, dy = xs[col][:, None] - c[0], ys[row][None, :] - c[1]
            out[row, col] = np.mean(dx ** 2 + dy ** 2 <= radius ** 2)
    return out


def centre(grid, c, radius):
    out = np.zeros((grid.height, grid.width))
    for row in range(grid.height):
        for col in range(grid.width):
            out[row, col] = math.dist(grid.centre((col, row)), c) <= radius
    return out


RULES = {
    'centre': centre,
    'overlap': overlap,
    'touch': lambda g, c, r: (overlap(g, c, r) > 0).astype(float),
}


def raster():
    rng = np.random.default_rng(0)
    g = Grid()
    print('radius rule     empty%  target-cell-missed%  mean value at target'
          '  cells>0  raised area / true area')
    for radius in (1.5, 2.5, 4, 6, 8, 12):
        rows = {k: [] for k in RULES}
        for _ in range(300):
            c = rng.uniform(radius, 40 - radius, 2)
            a, d = rng.uniform(0, 2 * math.pi), radius * math.sqrt(rng.random())
            cell = g.cell_of(c[0] + d * math.cos(a), c[1] + d * math.sin(a))
            for k, f in RULES.items():
                m = f(g, c, radius)
                rows[k].append((m.max() == 0, m[cell[1], cell[0]] == 0,
                                m[cell[1], cell[0]], (m > 0).sum(),
                                m.sum() * 25 / (math.pi * radius ** 2)))
        for k, v in rows.items():
            e, miss, val, n, ar = np.mean(v, axis=0)
            print(f'{radius:5.1f}  {k:8s} {100*e:6.1f} {100*miss:12.1f}'
                  f' {val:18.2f} {n:14.1f} {ar:10.2f}')


def run(side, seed, cond, alpha, outside, n_targets=None, radius=8.0,
        timeout=900.0):
    """One mission. cond: uniform | accurate | misleading | noneg."""
    rng = np.random.default_rng(seed)
    cells = int(side / 5)
    g, spec = Grid(cells, cells), GridSpec(cells, cells)
    marks = [tuple(rng.uniform(8, side - 8, 2)) for _ in range(8)]
    k = n_targets or int(rng.integers(1, 4))
    true = [marks[i] for i in rng.choice(8, k, replace=False)]
    targets = []
    for m in true:
        a, d = rng.uniform(0, 2 * math.pi), 6 * math.sqrt(rng.random())
        targets.append((min(max(m[0] + d * math.cos(a), 1), side - 1),
                        min(max(m[1] + d * math.sin(a), 1), side - 1)))
    if cond in ('misleading', 'noneg'):     # furthest wrong landmarks
        wrong = sorted((m for m in marks if m not in true),
                       key=lambda m: -min(math.dist(m, t) for t in targets))
        regions = wrong[:k]
    else:
        regions = true
    params = BeliefParams(p_miss=0.5 if cond == 'noneg' else 0.45)
    bm = BeliefMap(g, params)
    if cond != 'uniform':
        u = np.max([overlap(g, c, radius) for c in regions], axis=0)
        prior = outside + (1.0 - outside) * u
        p = alpha * prior + (1 - alpha) * 0.5
        lo = np.log(p / (1 - p))
        bm.logodds = np.where(np.abs(lo) < 1e-9, np.nan, lo)

    noise, det_rng = NoiseModel(), np.random.default_rng(1000 + seed)
    layers = {'L': 8.0, 1: 10.0, 2: 12.0, 3: 14.0}
    pos = {'L': (2.5, 2.5)}
    pos.update({i: (2.5 + 5 * i, -5.0) for i in (1, 2, 3)})
    goal = {1: None, 2: None, 3: None}
    hw = lane_half_width(CAM, 8.0)
    wps, wi = plan_sweep(spec, hw, pos['L']), 0
    first_seen, conf, sweep_done, quiet, t = {}, {}, None, None, 0.0
    while t < timeout:
        # leader
        if wi < len(wps):
            x, y = pos['L']
            d = math.dist((x, y), wps[wi])
            s = min(1.0, 2.5 / d) if d > 0 else 1.0
            pos['L'] = (x + (wps[wi][0] - x) * s, y + (wps[wi][1] - y) * s)
            if math.dist(pos['L'], wps[wi]) < 1.0:
                wi += 1
                if wi == len(wps):
                    sweep_done = t
        occ = bm.occupancy_data()
        values = values_from_occupancy(occ, spec)
        for i in (1, 2, 3):
            if t % 1.0 == 0:
                peers = [Peer(j, pos[j], t, goal[j], t if goal[j] else -math.inf)
                         for j in (1, 2, 3) if j != i]
                goal[i] = plan_step(spec, values, i, pos[i], goal[i], peers, t).goal
            if goal[i] is not None:
                gx, gy = spec.sector_centre(goal[i])
                d = math.dist(pos[i], (gx, gy))
                if d > 0:
                    s = min(1.0, 2.5 / d)
                    pos[i] = (pos[i][0] + (gx - pos[i][0]) * s,
                              pos[i][1] + (gy - pos[i][1]) * s)
        for k_, z in layers.items():
            bp = (*pos[k_], z)
            cam_pos, R = camera_pose(bp, LEVEL)
            dets = frame_detections(targets, cam_pos, R,
                                    (CAM.fx, CAM.fy, CAM.cx, CAM.cy),
                                    (CAM.width, CAM.height), noise, det_rng)
            f = project_frame(g, CAM, bp, LEVEL, [(d.u, d.v, d.score) for d in dets])
            if any(d.is_true for d in dets):
                for n, tg in enumerate(targets):
                    for d in dets:
                        gx = cam_pos + (R @ np.array([(d.u - CAM.cx) / CAM.fx,
                             (d.v - CAM.cy) / CAM.fy, 1.0])) * z
                        if math.dist(gx[:2], tg) <= 2.0:
                            first_seen.setdefault(n, t)
            c = bm.update(str(k_), t, bp, f, confirmer=k_ != 'L')
            if c is not None:
                conf[c] = t
        still = open_cells(bm.occupancy_data(), cells, bm.confirmed)
        if sweep_done is not None and not still:
            quiet = t if quiet is None else quiet
            if t - quiet >= 30:
                break
        else:
            quiet = None
        t += 0.5
    tcells = {g.cell_of(*tg) for tg in targets}
    got = [conf[c] for c in tcells if c in conf]
    return dict(k=k, seen=len(first_seen), first=min(first_seen.values(), default=timeout),
                cfirst=min(got, default=timeout),
                call=max(got) if len(got) == len(tcells) else timeout,
                missed=len(tcells) - len(got), false=len([c for c in conf if c not in tcells]),
                end=t, done=t < timeout, sweep=sweep_done,
                unseen=int(np.isnan(bm.logodds).sum()))


def mission(sides=(40, 80), seeds=20):
    print('side cond        alpha out  first-detect  first-confirm  all-confirm'
          '  (medians s)  seeds-with-miss  targets-missed  false  timeouts  median-end')
    for side in sides:
        for cond, alpha, outside in (
                ('uniform', 0, 0.5),
                *[(c, a, o) for c in ('accurate', 'misleading')
                  for o in (0.5, 0.0) for a in (0.3, 0.5, 0.7)],
                ('noneg', 0.5, 0.5), ('noneg', 0.5, 0.0)):
            r = [run(side, s, cond, alpha, outside) for s in range(seeds)]
            m = lambda key: st.median(x[key] for x in r)  # noqa: E731
            print(f"{side:4d} {cond:11s} {alpha:4.1f} {outside:4.1f} {m('first'):10.1f}"
                  f" {m('cfirst'):12.1f} {m('call'):12.1f}"
                  f" {sum(x['missed'] > 0 for x in r):18d}"
                  f" {sum(x['missed'] for x in r):6d}/{sum(x['k'] for x in r)}"
                  f" {sum(x['false'] for x in r):10d} {sum(not x['done'] for x in r):8d}"
                  f" {m('end'):10.0f}", flush=True)


def feasible():
    """How often k wrong landmarks exist at all, for a given clearance between
    the region edge (r = 8 m) and every target."""
    print('side landmarks clearance  seeds (of 200) with enough wrong landmarks')
    for side in (40, 80, 120):
        for n in (6, 8):
            for clear in (0, 7, 16):
                ok = 0
                for seed in range(200):
                    rng = np.random.default_rng(seed)
                    marks = [tuple(rng.uniform(8, side - 8, 2)) for _ in range(n)]
                    k = int(rng.integers(1, 4))
                    idx = rng.choice(n, k, replace=False)
                    tg = []
                    for i in idx:
                        a, d = rng.uniform(0, 2 * math.pi), 6 * math.sqrt(rng.random())
                        tg.append((marks[i][0] + d * math.cos(a), marks[i][1] + d * math.sin(a)))
                    good = [m for j, m in enumerate(marks) if j not in idx and
                            min(math.dist(m, t) for t in tg) >= 8 + clear]
                    ok += len(good) >= k
                print(f'{side:4d} {n:6d} {clear:9d} {ok:12d}')


if __name__ == '__main__':
    {'raster': raster, 'mission': mission, 'feasible': feasible}[sys.argv[1]]()
