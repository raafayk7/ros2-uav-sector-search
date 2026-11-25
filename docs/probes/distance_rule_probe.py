"""Session 19 probe (no ROS, no language model): R12, does a distance discount
beat the best-open-sector rule on LLM-shaped maps?

  PYTHONPATH=src/uav_mapping:src/uav_planning:src/uav_perception:src/uav_eval \
      python3 docs/probes/distance_rule_probe.py      (about 5 min, 5 processes)

The flying rule takes the highest value anywhere and uses distance only to break
ties. The alternatives score a sector as value * exp(-distance / L):

  pick-L   which sector to move to changes; whether to move does not (the raw
           values and the stay margin decide that, as today)
  full-L   the score also decides whether to move

Written before any number was seen: an alternative is worth putting to the user
only if, on the first 200 seeds, its median time to all found is lower with an
accurate clue and no higher with a uniform or misleading one, and it is faster
in more seeds than it is slower in all three arms, and the same holds on a
second 200 seeds that chose nothing.

Outcome (2026-11-25): no alternative met the first half (every discount is
slower with an accurate clue), so the second 200 seeds were not needed and are
not run. The run that showed this was stopped during them.

The "model" is lane_order_probe's stand-in (gemini-3.8-flash's pilot habits), so
these are design numbers, not results. Seeds 4000+, never the eval's. The last
block repeats the comparison on the 8 pilot seeds' live recordings.
"""
import math
from concurrent.futures import ProcessPoolExecutor

from lane_order_probe import answer, g, p, w
from uav_eval.core import arms, scenario as sc
from uav_planning.core import planner

RULES = ('today', 'pick-30', 'pick-60', 'pick-120', 'full-60')
ARMS = ('uniform', 'accurate', 'misleading')
today = planner.choose_goal


def discounted(kind, L):
    def choose(grid, values, my_pose, current_goal, blocked, params=planner.PlannerParams()):
        d = today(grid, values, my_pose, current_goal, blocked, params)
        if d.reason == 'nothing-open' or (kind == 'pick' and d.reason == 'hold'):
            return d
        here = grid.cell_of(*my_pose)
        held = current_goal if current_goal is not None else (here if grid.contains(here) else None)
        held_value = 0.0 if held is None or held in blocked else values[held.row][held.col]

        def score(s):
            return values[s.row][s.col] * math.exp(-math.dist(my_pose, grid.sector_centre(s)) / L)
        cands = [s for s in grid.sectors() if s not in blocked]
        if kind == 'pick':
            cands = [s for s in cands if values[s.row][s.col] - held_value > params.move_margin + 1e-9]
        goal = max(cands, key=lambda s: (score(s), -grid.index(s)))
        if kind == 'full':
            base = 0.0 if held is None or held in blocked else score(held)
            if score(goal) - base <= params.move_margin + 1e-9:
                return planner.Decision(held, held != current_goal, 'hold')
        return planner.Decision(goal, goal != current_goal, 'move')
    return choose


def job(a):
    rule, arm, seed, live = a
    kind, _, L = rule.partition('-')
    planner.choose_goal = today if rule == 'today' else discounted(kind, float(L))
    return arms.run_arm(arm, sc.draw(w, g, seed), w, g,
                        arms.strict_replay(p) if live else answer)


def compare(title, seeds, live=False):
    print(f'\n{title}: {len(seeds)} seeds')
    with ProcessPoolExecutor(5) as ex:
        jobs = [(r, a, s, live) for r in RULES for a in ARMS for s in seeds]
        runs = dict(zip(jobs, ex.map(job, jobs, chunksize=8)))
    for a in ARMS:
        for r in RULES:
            rs = [runs[r, a, s, live] for s in seeds]
            x = arms.summary(rs)
            d = [arms.paired({'x': rs, 't': [runs['today', a, s, live] for s in seeds]}, 'x', 't', k)
                 for k in ('first', 'all')]
            print(f"{a:10} {r:8} first {x['first']} all {x['all']} missed {x['targets_missed']}/{x['targets']}"
                  f" done {x['done']} | faster/slower/tie: first {d[0]['faster']}/{d[0]['slower']}/{d[0]['ties']}"
                  f" all {d[1]['faster']}/{d[1]['slower']}/{d[1]['ties']} (median diff {d[1]['median_diff']:+.1f})", flush=True)


if __name__ == '__main__':
    compare('first', range(4000, 4200))
    compare('live pilot', range(1000, 1008), live=True)
