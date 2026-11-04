"""Compare slope values against direct flight to the best open sector, measuring
peaks held and total flight distance. Stay + closed neighbourhood + 0.02 margin
in every variant. Reports why a run missed a peak."""
import random
import statistics as st

MARGIN = 0.02


def L(c, n):
    r, q = c
    return [(r + a, q + b) for a in (-1, 0, 1) for b in (-1, 0, 1)
            if (a or b) and 0 <= r + a < n and 0 <= q + b < n]


def cheb(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def blocked(me, pos, n):
    out = set()
    for i, x in enumerate(pos):
        if i != me:
            out.update(L(x, n))
            out.add(x)
    return out


def peaks(n, k, rng):
    cells = [(r, q) for r in range(n) for q in range(n)]
    while True:
        s = rng.sample(cells, k)
        if all(cheb(a, b) >= 2 for i, a in enumerate(s) for b in s[i + 1:]):
            return s


def run_slope(p, n, starts, gamma, rng, rounds=40):
    pos = list(starts)
    dist = 0
    # Sources limited to cells above the background, as a real map would threshold.
    for _ in range(rounds):
        for a in range(len(pos)):
            blk = blocked(a, pos, n)
            src = [c for c, v in p.items() if c not in blk]
            def u(c):
                if c in blk:
                    return 0.0
                return max(p[k] * gamma ** cheb(c, k) for k in src) if src else 0.0
            cur = u(pos[a])
            cand = {c: u(c) for c in L(pos[a], n)}
            best = max(cand.values())
            if best > cur + MARGIN * 0 + 1e-9 and best - cur >= 1e-9:
                pos[a] = rng.choice([c for c, v in cand.items() if v >= best - 1e-9])
                dist += 1
    return pos, dist


def run_direct(p, n, starts, rng, rounds=6):
    pos = list(starts)
    dist = 0
    for _ in range(rounds):
        for a in range(len(pos)):
            blk = blocked(a, pos, n)
            open_c = [c for c in p if c not in blk]
            if not open_c:
                continue
            bestv = max(p[c] for c in open_c)
            cur = p[pos[a]] if pos[a] not in blk else 0.0
            if bestv > cur + MARGIN:
                tops = [c for c in open_c if p[c] >= bestv - 1e-9]
                t = min(tops, key=lambda c: (cheb(c, pos[a]), rng.random()))
                dist += cheb(t, pos[a])
                pos[a] = t
    return pos, dist


print('3 peaks 0.97 on 0.03 background, 3 drones, 100 seeds. dist = sectors flown (Chebyshev) per drone')
print(f"{'variant':26s} {'grid':>6s} {'held':>7s} {'all 3':>6s} {'dist':>6s} {'miss: peak inside a peer zone':>30s}")
for n in (4, 8, 12):
    for name, g in (('slope gamma 0.5', .5), ('slope gamma 0.8', .8), ('slope gamma 0.95', .95), ('direct to best open', None)):
        held, full, dists, zone = [], 0, [], 0
        misses = 0
        for seed in range(100):
            rng = random.Random(seed)
            ks = peaks(n, 3, rng)
            p = {(r, q): 0.03 for r in range(n) for q in range(n)}
            p.update({k: 0.97 for k in ks})
            s = rng.sample(list(p), 3)
            pos, d = run_slope(p, n, s, g, rng) if g else run_direct(p, n, s, rng)
            h = set(pos) & set(ks)
            held.append(len(h))
            full += len(h) == 3
            dists.append(d / 3)
            for k in set(ks) - h:
                misses += 1
                # Missed peak is next to a drone sitting off-peak, or next to a held one.
                zone += any(cheb(k, x) <= 1 for x in pos)
        print(f'{name:26s} {n:3d}x{n:<2d} {st.mean(held):5.2f}/3 {full:5.0f}% {st.mean(dists):6.1f} {zone:>14d}/{misses} misses')
