"""The eval's four arms, and how their results are summarised (no ROS).

  uniform      no clue: every cell starts unknown
  accurate     the clue names each target's landmark
  misleading   the same sentence, every landmark swapped (D47)
  fixed        the misleading clue with negative evidence off: an empty look
               changes nothing (the condition arXiv 2611.12317 measured)

Priors come from recordings a language model wrote, strictly: a hand-written
recording, a missing one or an invalid answer raises (D48), because a quiet
uniform prior in one arm would contaminate the comparison. All arms of a seed
share the targets and the detector's noise seed, so they are paired.
"""

import math
from dataclasses import replace
from typing import Dict, List, NamedTuple, Optional, Sequence

import numpy as np

from uav_mapping.core.geometry import Grid, Point, segment_distance
from uav_mapping.core.landmarks import World
from uav_mapping.core.prior import Prior, Region, load_replay, make_prior, parse_recordings

from .scenario import Scenario
from .sim import Result, SimParams, simulate

ARMS = ('uniform', 'accurate', 'misleading', 'fixed')


def strict_replay(path):
    """The replay provider for an eval: live recordings of one model only."""
    with open(path, encoding='utf-8') as f:
        recordings = parse_recordings(f.read())
    models = {r.model for r in recordings}
    if len(models) > 1:
        raise ValueError(f"recordings of several models: {sorted(models)}")
    return load_replay(path, require_live=True)


def prior_for(arm: str, scenario: Scenario, world: World, grid: Grid,
              provider) -> Optional[Prior]:
    if arm == 'uniform':
        return None
    clue = scenario.accurate if arm == 'accurate' else scenario.misleading
    return make_prior(clue, world, grid, provider, strict=True)


def covers(region: Region, p: Point) -> bool:
    a, b = (region.center, region.center) if hasattr(region, 'center') \
        else (region.start, region.end)
    return bool(segment_distance(p[0], p[1], a, b) <= region.radius)


class Run(NamedTuple):
    arm: str
    scenario: Scenario
    covered: List[bool]         # per target: some region of the prior covers it
    regions: int
    result: Result


def run_arm(arm: str, scenario: Scenario, world: World, grid: Grid, provider,
            params: SimParams = SimParams()) -> Run:
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
    prior = prior_for(arm, scenario, world, grid, provider)
    if arm == 'fixed':
        params = replace(params, negative_evidence=False)
    regions = prior.regions if prior else ()
    result = simulate(grid, scenario.targets, prior.start if prior else None,
                      scenario.seed, params)
    return Run(arm, scenario, [any(covers(r, t) for r in regions)
                               for t in scenario.targets], len(regions), result)


# ---- Summaries --------------------------------------------------------------

def quantile(times: Sequence[Optional[float]], q: float) -> float:
    """Quantile with censored times (None) after every real one; inf if the
    quantile falls among them. Lower interpolation, so no made-up midpoint
    between a time and a censored run."""
    ordered = sorted(math.inf if t is None else t for t in times)
    return ordered[max(0, math.ceil(q * len(ordered)) - 1)]


def summary(runs: Sequence[Run]) -> dict:
    """One arm's line of the table."""
    results = [r.result for r in runs]
    targets = [t for r in results for t in r.found]
    line = {'runs': len(runs), 'targets': len(targets),
            'targets_missed': sum(t is None for t in targets),
            'targets_covered_by_clue': sum(sum(r.covered) for r in runs),
            'found_by_leader': sum(w == 'iris_1' for r in results for w in r.found_by),
            'extra_confirms': sum(r.extra_confirms for r in results),
            'done': sum(r.done for r in results),
            'coverage_min': min(r.coverage for r in results)}
    for name, times in (('first', [r.first for r in results]),
                        ('all', [r.all for r in results])):
        line[name] = tuple(quantile(times, q) for q in (0.25, 0.5, 0.75))
        line[f'{name}_censored'] = sum(t is None for t in times)
    return line


def sign_p(faster: int, slower: int) -> float:
    """Two-sided exact sign test: how often a fair coin would split the
    untied seeds at least this unevenly."""
    n, k = faster + slower, min(faster, slower)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def paired(runs: Dict[str, Sequence[Run]], a: str, b: str, what: str) -> dict:
    """Seed by seed, arm ``a`` against ``b`` on 'first' or 'all'. Both
    censored is a tie; one censored counts as that arm being slower."""
    diffs, faster, slower = [], 0, 0
    for x, y in zip(runs[a], runs[b]):
        assert x.scenario.seed == y.scenario.seed
        tx, ty = (math.inf if t is None else t
                  for t in (getattr(x.result, what), getattr(y.result, what)))
        faster += tx < ty
        slower += tx > ty
        if math.isfinite(tx) and math.isfinite(ty):
            diffs.append(tx - ty)
    return {'faster': faster, 'slower': slower,
            'ties': len(runs[a]) - faster - slower,
            'sign_p': sign_p(faster, slower),
            'median_diff': float(np.median(diffs)) if diffs else math.nan,
            'both_found': len(diffs)}
