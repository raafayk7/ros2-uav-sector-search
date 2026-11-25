"""Session 19 (no ROS): look at the recorded answers for a set of seeds before
flying them. Are the regions where the clue says, do the accurate ones cover
the targets, and does a misleading one touch a target's neighbourhood (D54)?

  PYTHONPATH=src/uav_mapping:src/uav_planning:src/uav_perception:src/uav_eval \
      python3 docs/probes/recordings_check.py [seeds, default 0-19]
"""
import sys
from collections import Counter

from uav_eval import run as cli
from uav_eval.core import arms, scenario as sc
from uav_eval.core.arms import covers, prior_for
from uav_mapping.core.landmarks import landmark_distance
from uav_mapping.core.prior import rasterize

seeds = cli.parse_seeds(sys.argv[1] if len(sys.argv) > 1 else '0-19')
w, g, path = cli.load('worlds', 'valley')
provider = arms.strict_replay(path)
radii, doubt, touched, targets, off, uncovered = Counter(), Counter(), 0, 0, 0, 0
for s in seeds:
    x = sc.draw(w, g, s)
    targets += len(x.targets)
    for arm in ('accurate', 'misleading'):
        names = x.names if arm == 'accurate' else [x.swap[n] for n in x.names]
        prior = prior_for(arm, x, w, g, provider)
        for warning in prior.warnings:
            print(f'seed {s} {arm}: warning {warning}')
        for r in prior.regions:
            radii[r.radius] += 1
            doubt[r.weight, r.confidence] += 1
            for q in ([r.center] if hasattr(r, 'center') else [r.start, r.end]):
                if min(float(landmark_distance(w.landmark(n), *q)) for n in names) > 0.5:
                    off += 1
                    print(f'seed {s} {arm}: a region is off its place: {r}')
        for n in names:
            if not any(covers(r, q) for r in prior.regions for q in w.landmark(n).points):
                print(f'seed {s} {arm}: no region on {n}')
        if arm == 'accurate':
            for n, t in zip(x.names, x.targets):
                if not any(covers(r, t) for r in prior.regions):
                    uncovered += 1
                    print(f'seed {s}: target at {float(landmark_distance(w.landmark(n), *t)):.1f} m '
                          f'from the {n} is outside every accurate region')
        else:
            u = rasterize(g, prior.regions)
            for t in x.targets:
                c, r = g.cell_of(*t)
                if u[max(r - 1, 0):r + 2, max(c - 1, 0):c + 2].max() > 0:
                    touched += 1
                    print(f'seed {s}: a misleading region touches the zone of the target at {t}')
print(f'{len(seeds)} seeds, {targets} targets; regions off their place {off}; accurate targets '
      f'uncovered {uncovered}; misleading regions touching a target zone {touched}')
print('radii', dict(sorted(radii.items())))
print('(weight, confidence)', dict(doubt))
