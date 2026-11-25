"""One eval scenario: where the targets are, and the clues about them (no ROS).

Everything here is drawn from the seed and from landmark geometry, before any
language model is asked, and it is the same for every arm of a seed:

* 1 to 3 targets, each near its own landmark: uniform over the ground within
  ``near`` metres of it (a polygon's inside counts), at least ``margin`` from
  the map's edge.
* The accurate clue names those landmarks. The misleading clue is the same
  sentence with every landmark swapped (D47). If a draw has no valid swap the
  targets are drawn again, for all arms, and the redraws are counted.
"""

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

from uav_mapping.core.geometry import Grid, Point
from uav_mapping.core.landmarks import World, landmark_distance, misleading_swap

MAX_TARGETS = 3
NEAR = 8.0          # m: how close "near" puts a target
CLEAR = 20.0        # m: D47's clearance round a wrong landmark. Above the largest
                    # radius the model gave "near" in the pilot (18 m)
MARGIN = 1.0        # m from the map's edge
MAX_REDRAWS = 1000

_COUNTS = {2: 'Two hikers are missing', 3: 'Three hikers are missing'}


@dataclass(frozen=True)
class Scenario:
    seed: int
    targets: Tuple[Point, ...]
    names: Tuple[str, ...]          # each target's landmark, same order
    swap: Dict[str, str]            # landmark -> the one a misleading clue names
    redraws: int                    # draws thrown away for want of a swap

    @property
    def accurate(self) -> str:
        return clue_text(self.names)

    @property
    def misleading(self) -> str:
        return clue_text([self.swap[n] for n in self.names])


def clue_text(names: Sequence[str]) -> str:
    """The operator's sentence. One template for both kinds of clue."""
    if len(names) == 1:
        return f"One hiker is missing, last seen near the {names[0]}."
    where = [f"one last seen near the {names[0]}"] + \
        [f"one near the {n}" for n in names[1:]]
    return f"{_COUNTS[len(names)]}: {', '.join(where[:-1])} and {where[-1]}."


def _near(world: World, name: str, rng) -> Point:
    """A point uniform over the ground within NEAR of a landmark."""
    lm = world.landmark(name)
    x0, y0, x1, y1 = world.bounds
    xs, ys = zip(*lm.points)
    lo = max(min(xs) - NEAR, x0 + MARGIN), max(min(ys) - NEAR, y0 + MARGIN)
    hi = min(max(xs) + NEAR, x1 - MARGIN), min(max(ys) + NEAR, y1 - MARGIN)
    while True:
        p = rng.uniform(lo, hi)
        if landmark_distance(lm, p[0], p[1]) <= NEAR:
            return float(p[0]), float(p[1])


def draw(world: World, grid: Grid, seed: int) -> Scenario:
    world.check_grid(grid)
    rng = np.random.default_rng([seed, 0])
    all_names: List[str] = [lm.name for lm in world.landmarks]
    for redraws in range(MAX_REDRAWS):
        k = int(rng.integers(1, MAX_TARGETS + 1))
        names = [all_names[i] for i in rng.choice(len(all_names), k, replace=False)]
        targets = [_near(world, n, rng) for n in names]
        swap = misleading_swap(grid, world, names, targets, CLEAR, rng)
        if swap is not None:
            return Scenario(seed, tuple(targets), tuple(names), swap, redraws)
    raise RuntimeError(f"no scenario with a valid swap for seed {seed}")
