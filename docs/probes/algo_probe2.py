"""Faster version of the probe: fewer seeds, sizes 4/8/12, fixed BLL block."""
import math
import random
import statistics as st


def L(c, n):
    r, q = c
    return [(r + a, q + b) for a in (-1, 0, 1) for b in (-1, 0, 1)
            if (a or b) and 0 <= r + a < n and 0 <= q + b < n]


def blocked(me, pos, n, closed):
    out = set()
    for i, x in enumerate(pos):
        if i == me:
            continue
        out.update(L(x, n))
        if closed:
            out.add(x)
    return out


def slope(ks, p, blk, c, gamma):
    return max((p[k] * gamma ** max(abs(c[0] - k[0]), abs(c[1] - k[1]))
                for k in ks if k not in blk), default=0.0)


def run(p, ks, n, starts, stay, closed, gamma, rng, rounds=40):
    # ks: cells worth attracting to. Background keeps its own value under gamma.
    pos = list(starts)
    moved = 0
    for _ in range(rounds):
        moved = 0
        for a in range(len(pos)):
            blk = blocked(a, pos, n, closed)
            def u(c):
                if c in blk:
                    return 0.0
                return p[c] if gamma is None else max(p[c], slope(ks, p, blk, c, gamma))
            acts = L(pos[a], n) + ([pos[a]] if stay else [])
            vals = {c: u(c) for c in acts}
            best = max(vals.values())
            if stay and vals[pos[a]] >= best - 1e-9:
                continue
            c = rng.choice([c for c, v in vals.items() if v >= best - 1e-9])
            if c != pos[a]:
                pos[a] = c
                moved += 1
    return pos, moved


def peaks(n, k, rng):
    cells = [(r, q) for r in range(n) for q in range(n)]
    while True:
        s = rng.sample(cells, k)
        if all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) >= 2
               for i, a in enumerate(s) for b in s[i + 1:]):
            return s


VARIANTS = [
    ('A thesis literal: no stay, peer cell free', dict(stay=False, closed=False, gamma=None)),
    ('B + stay action + peer cell penalised', dict(stay=True, closed=True, gamma=None)),
    ('C = B + gamma 0.8 slope toward open peaks', dict(stay=True, closed=True, gamma=0.8)),
]
print('Sparse map: 3 peaks 0.97 on swept 0.03 background, 3 drones, 200 seeds')
print(f"{'variant':44s} {'grid':>6s} {'peaks held':>11s} {'all 3':>6s} {'moving at end':>14s} {'stacked':>8s}")
for n in (4, 8, 12):
    for name, kw in VARIANTS:
        held, full, moving, stacked = [], 0, 0, 0
        for seed in range(200):
            rng = random.Random(seed)
            ks = peaks(n, 3, rng)
            p = {(r, q): 0.03 for r in range(n) for q in range(n)}
            p.update({k: 0.97 for k in ks})
            pos, m = run(p, ks, n, rng.sample(list(p), 3), rng=rng, **kw)
            h = len(set(pos) & set(ks))
            held.append(h)
            full += h == 3
            moving += m > 0
            stacked += len(set(pos)) < 3
        print(f'{name:44s} {n:3d}x{n:<2d} {st.mean(held):8.2f}/3 {full / 2:5.0f}% {moving / 2:13.0f}% {stacked / 2:7.0f}%')

print('\nOne peak, one drone, thesis literal (no stay): last 4 positions, 8x8')
rng = random.Random(1)
p = {(r, q): 0.03 for r in range(8) for q in range(8)}
p[(4, 4)] = 0.97
pos = [(3, 3)]
trace = []
for _ in range(6):
    acts = L(pos[0], 8)
    best = max(p[c] for c in acts)
    pos[0] = rng.choice([c for c in acts if p[c] >= best])
    trace.append(pos[0])
print('  ', trace, '<- never settles' if len(set(trace[-4:])) > 1 else '')

print('\nFlat map, stay rule, hold when no neighbour is strictly better: 3 drones')
for n in (4, 12):
    moved = 0
    for seed in range(200):
        rng = random.Random(seed)
        p = {(r, q): 0.5 for r in range(n) for q in range(n)}
        s = rng.sample(list(p), 3)
        pos, _ = run(p, [], n, s, stay=True, closed=True, gamma=None, rng=rng)
        moved += pos != s
    print(f'  {n}x{n}: runs where any drone moved {moved / 2:.0f}% (only drones inside a peer neighbourhood move)')

print('\nCrowding: share of map each drone sees zeroed by 2 peers')
for n in (4, 8, 12):
    fr = []
    for seed in range(500):
        rng = random.Random(seed)
        pos = rng.sample([(r, q) for r in range(n) for q in range(n)], 3)
        fr.append(len(blocked(0, pos, n, True)) / (n * n))
    print(f'  {n}x{n}: {st.mean(fr) * 100:.0f}%')

print('\nAdjacent targets: 2 peaks side by side, 2 drones, variant C, 8x8')
both = one = 0
ks = [(3, 3), (3, 4)]
for seed in range(200):
    rng = random.Random(seed)
    p = {(r, q): 0.03 for r in range(8) for q in range(8)}
    p[ks[0]] = p[ks[1]] = 0.97
    pos, _ = run(p, ks, 8, rng.sample(list(p), 2), True, True, 0.8, rng)
    h = len(set(pos) & set(ks))
    both += h == 2
    one += h == 1
print(f'  both held {both / 2:.0f}%, one held {one / 2:.0f}%')

print('\nBLL as written (softmax on u in [0,1]), 3 peaks, 3 drones, sparse map')
for n in (4, 12):
    firsts, occ, leave = [], [], 0.0
    for seed in range(40):
        rng = random.Random(seed)
        ks = peaks(n, 3, rng)
        p = {(r, q): 0.03 for r in range(n) for q in range(n)}
        p.update({k: 0.97 for k in ks})
        pos = rng.sample(list(p), 3)
        first, on, N = None, 0, 8000
        for t in range(N):
            a = rng.randrange(3)
            blk = blocked(a, pos, n, True)
            u = lambda c: 0.0 if c in blk else p[c]
            alt = rng.choice(L(pos[a], n))
            pm = math.exp(u(alt)) / (math.exp(u(alt)) + math.exp(u(pos[a])))
            if rng.random() < pm:
                pos[a] = alt
            h = len(set(pos) & set(ks))
            if first is None and h == 3:
                first = t
            on += h
        firsts.append(first if first is not None else N)
        occ.append(on / N)
    print(f'  {n}x{n}: median iters to first all-3-on-peaks {st.median(firsts):.0f}; long-run mean peaks held {st.mean(occ):.2f}/3')
print('  P(leave a 0.97 peak for a 0.03 neighbour) per attempt = %.2f' % (math.exp(.03) / (math.exp(.03) + math.exp(.97))))
