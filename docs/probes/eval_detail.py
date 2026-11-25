"""Session 19 (no ROS): what the results table does not show. Reads
docs/results/eval.json and replays the same runs (they are deterministic, and
it checks that they are the same) to see where the extra confirmations were.
Written after the table was seen, so everything here is descriptive.

  PYTHONPATH=src/uav_mapping:src/uav_planning:src/uav_perception:src/uav_eval \
      python3 docs/probes/eval_detail.py
"""
import json
import math
from collections import Counter

from uav_eval import run as cli
from uav_eval.core import arms, scenario as sc, sim

w, g, path = cli.load('worlds', 'valley')
data = json.load(open('docs/results/eval.json'))
by = {(d['arm'], d['seed']): d for d in data}
seeds = sorted({d['seed'] for d in data})
first, last = (lambda d: min(d['found'])), (lambda d: max(d['found']))


def five(xs):
    return ' / '.join(f'{arms.quantile(xs, q):+.1f}' for q in (0, .25, .5, .75, 1))


print('paired differences, s: min / q25 / median / q75 / max')
for a, b in (('accurate', 'uniform'), ('misleading', 'uniform'), ('misleading', 'accurate'),
             ('fixed', 'misleading')):
    for name, f in (('first', first), ('all', last)):
        print(f'  {a} - {b}, {name}: {five([f(by[a, s]) - f(by[b, s]) for s in seeds])}')
print('slowest run to find all:', {a: max(last(by[a, s]) for s in seeds) for a in arms.ARMS})
print('targets per run:', Counter(len(by['uniform', s]['found']) for s in seeds))
print('median time to all by targets per run:')
for k in (1, 2, 3):
    few = [s for s in seeds if len(by['uniform', s]['found']) == k]
    print(f'  {k} ({len(few)} runs):', {a: arms.quantile([last(by[a, s]) for s in few], .5)
                                        for a in arms.ARMS})
print('found by:', {a: dict(sorted(Counter(x for s in seeds for x in by[a, s]['found_by']).items()))
                    for a in arms.ARMS})
print('targets found at or before 1 s after the start:',
      {a: sum(t <= 1.0 for s in seeds for t in by[a, s]['found']) for a in arms.ARMS})

kept = []
real = sim.BeliefMap


class Kept(real):
    def __init__(self, *a):
        super().__init__(*a)
        kept.append(self)


sim.BeliefMap = Kept
provider = arms.strict_replay(path)
print('extra confirmed sectors: seed, distance from the sector centre to the nearest target, '
      'and from its edge, m')
for arm in arms.ARMS:
    for s in seeds:
        x = sc.draw(w, g, s)
        r = arms.run_arm(arm, x, w, g, provider).result
        assert r.found == by[arm, s]['found'], (arm, s)
        cells = {g.cell_of(*t) for t in x.targets}
        for c in kept[-1].confirmed:
            if c not in cells:
                cx = g.origin_x + (c[0] + .5) * g.resolution
                cy = g.origin_y + (c[1] + .5) * g.resolution
                h = g.resolution / 2
                edge = min(math.hypot(max(abs(t[0] - cx) - h, 0), max(abs(t[1] - cy) - h, 0))
                           for t in x.targets)
                print(f'  {arm:10} seed {s:2}: centre {min(math.dist((cx, cy), t) for t in x.targets):5.1f}'
                      f'  edge {edge:5.2f}')
