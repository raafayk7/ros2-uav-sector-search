"""Simultaneous decisions with flights that take time: 1 sector per tick, decisions
every tick from peers' state one tick stale. A drone re-plans only on arrival or if
its goal is blocked (approved stepping rule). Variants differ in what blocks a goal."""
import random
import statistics as st

N, MARGIN, TICKS = 8, 0.02, 80


def Z(c):
    r, q = c
    return {(r + a, q + b) for a in (-1, 0, 1) for b in (-1, 0, 1)
            if 0 <= r + a < N and 0 <= q + b < N}


def cheb(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def peaks(rng, k, spacing):
    cells = [(r, q) for r in range(N) for q in range(N)]
    while True:
        s = rng.sample(cells, k)
        if all(cheb(a, b) >= spacing for i, a in enumerate(s) for b in s[i + 1:]):
            return s


def step(c, g):
    return (c[0] + (g[0] > c[0]) - (g[0] < c[0]), c[1] + (g[1] > c[1]) - (g[1] < c[1]))


def blocked_set(i, pos, goal, variant):
    blk = set()
    for j in range(len(pos)):
        if j == i:
            continue
        arrived = pos[j] == goal[j]
        if variant == 'A':      # approved: peer position and goal both block
            blk |= Z(pos[j]) | Z(goal[j])
        elif variant == 'B':    # A, but a higher sysid yields to lower sysid goals only
            blk |= Z(pos[j])
            if j < i:
                blk |= Z(goal[j])
        elif variant == 'C':    # goals block by priority; a passing drone does not block
            if arrived:
                blk |= Z(pos[j])
            if j < i:
                blk |= Z(goal[j])
        elif variant == 'D':    # goals block regardless of priority; passing does not
            if arrived:
                blk |= Z(pos[j])
            blk |= Z(goal[j])
    return blk


def run(variant, ks, starts):
    p = {(r, q): 0.03 for r in range(N) for q in range(N)}
    p.update({k: 0.97 for k in ks})
    pos, goal = list(starts), list(starts)
    changes = last = 0
    for t in range(TICKS):
        prev_pos, prev_goal = list(pos), list(goal)
        for i in range(len(pos)):
            blk = blocked_set(i, prev_pos, prev_goal, variant)
            here = prev_pos[i] == prev_goal[i]
            goal_blocked = prev_goal[i] in blk
            if not here and not goal_blocked:
                continue
            cur_v = 0.0 if goal_blocked else p[prev_goal[i]]
            open_c = [c for c in p if c not in blk]
            if not open_c:
                continue
            best = max(p[c] for c in open_c)
            if best > cur_v + MARGIN:
                tops = [c for c in open_c if p[c] >= best - 1e-9]
                g = min(tops, key=lambda c: (cheb(c, prev_pos[i]), c))
                if g != goal[i]:
                    goal[i] = g
                    changes += 1
                    last = t
        pos = [step(c, g) for c, g in zip(pos, goal)]
    return len(set(pos) & set(ks)), changes, last


NAMES = {'A': 'A approved: positions + goals block',
         'B': 'B + lower sysid keeps a conflicting goal',
         'C': 'C = B, but a passing drone does not block',
         'D': 'D goals always block, passing does not'}
print('8x8, 3 drones, flights 1 sector/tick, simultaneous decisions, one tick stale, 200 seeds')
print(f"{'variant':44s} {'spacing':>7s} {'all 3':>6s} {'mean held':>10s} {'goal changes':>13s} {'changing at end':>16s}")
for spacing in (2, 3):
    for v in 'ABCD':
        full, live, ch, held = 0, 0, [], []
        for seed in range(200):
            rng = random.Random(seed)
            ks = peaks(rng, 3, spacing)
            s = rng.sample([(r, q) for r in range(N) for q in range(N)], 3)
            h, c, last = run(v, ks, s)
            full += h == 3
            live += last >= TICKS - 5
            ch.append(c)
            held.append(h)
        print(f'{NAMES[v]:44s} {spacing:7d} {full / 2:5.0f}% {st.mean(held):8.2f}/3 {st.mean(ch):13.1f} {live / 2:15.0f}%')
