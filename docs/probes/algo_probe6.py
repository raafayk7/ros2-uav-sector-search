"""Does the no-gradient finding depend on spikes? Compare the local thesis step (with a
stay rule) and direct flight on peaks spread as Gaussians of width sigma sectors, using
the approved conflict rules: timed flights, simultaneous decisions, one tick stale.
Success = a drone within 1 sector of each peak centre (exact centre shown too)."""
import math
import random
import statistics as st

MARGIN, TICKS = 0.02, 80


def Z(c, N):
    r, q = c
    return {(r + a, q + b) for a in (-1, 0, 1) for b in (-1, 0, 1)
            if 0 <= r + a < N and 0 <= q + b < N}


def cheb(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def peaks(rng, N, k, spacing):
    cells = [(r, q) for r in range(N) for q in range(N)]
    for _ in range(100000):
        s = rng.sample(cells, k)
        if all(cheb(a, b) >= spacing for i, a in enumerate(s) for b in s[i + 1:]):
            return s
    raise RuntimeError


def make_map(N, ks, sigma, bg=0.03, top=0.97):
    p = {}
    for r in range(N):
        for q in range(N):
            if sigma == 0:
                g = 1.0 if (r, q) in ks else 0.0
            else:
                g = max(math.exp(-((r - k[0]) ** 2 + (q - k[1]) ** 2) / (2 * sigma ** 2)) for k in ks)
            p[(r, q)] = round((bg + (top - bg) * g) * 100) / 100  # map values are whole percent
    return p


def step(c, g):
    return (c[0] + (g[0] > c[0]) - (g[0] < c[0]), c[1] + (g[1] > c[1]) - (g[1] < c[1]))


def run(rule, p, N, starts):
    pos, goal = list(starts), list(starts)
    for t in range(TICKS):
        pp, pg = list(pos), list(goal)
        for i in range(len(pos)):
            blk = set()
            for j in range(len(pos)):
                if j == i:
                    continue
                if pp[j] == pg[j]:
                    blk |= Z(pp[j], N)
                if j < i:
                    blk |= Z(pg[j], N)
            blocked_goal = pg[i] in blk
            if pp[i] != pg[i] and not blocked_goal:
                continue
            cur = 0.0 if blocked_goal else p[pg[i]]
            cand = [c for c in (p if rule == 'direct' else Z(pp[i], N)) if c not in blk]
            if not cand:
                continue
            best = max(p[c] for c in cand)
            if best > cur + MARGIN:
                tops = [c for c in cand if p[c] >= best - 1e-9]
                goal[i] = min(tops, key=lambda c: (cheb(c, pp[i]), c))
        pos = [step(c, g) for c, g in zip(pos, goal)]
    return pos


print('3 drones, 3 peaks at least 3 sectors apart, 200 seeds. all-3 = a drone on / within 1 sector of every peak centre')
print(f"{'sigma (sectors)':>15s} {'grid':>6s} {'local step: on':>15s} {'near':>6s} {'direct: on':>11s} {'near':>6s}")
for N in (8, 12):
    for sigma in (0, 0.5, 1, 2):
        res = {}
        for rule in ('local', 'direct'):
            on = near = 0
            for seed in range(200):
                rng = random.Random(seed)
                ks = peaks(rng, N, 3, 3)
                p = make_map(N, ks, sigma)
                pos = run(rule, p, N, rng.sample(list(p), 3))
                on += set(ks) <= set(pos)
                near += all(any(cheb(k, x) <= 1 for x in pos) for k in ks)
            res[rule] = (on / 2, near / 2)
        print(f"{sigma:15.1f} {N:3d}x{N:<2d} {res['local'][0]:14.0f}% {res['local'][1]:5.0f}% {res['direct'][0]:10.0f}% {res['direct'][1]:5.0f}%")
