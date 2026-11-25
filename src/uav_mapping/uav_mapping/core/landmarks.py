"""A world's named places, and which of them make a clue misleading (no ROS).

`worlds/<name>/landmarks.yaml` is the only description of the ground the
language model gets: places with names, in arena ENU metres.

  world: runway
  bounds: [0, 0, 40, 40]            # min x, min y, max x, max y
  landmarks:
    - {name: hut, type: point, points: [[33, 8]]}
    - {name: creek, type: polyline, points: [[0, 30], [18, 26], [40, 34]]}
    - {name: copse, type: polygon, points: [[4, 4], [12, 4], [9, 12]]}

An optional `description` per landmark goes to the language model as well.
The loader is strict (unknown keys, duplicate names and points off the map
are errors): a bad file is a mistake by us, found at launch, not something to
fly around.

The misleading-clue rule (D47) is here too, so it is fixed before any
language model is called and judged on landmark geometry alone.
"""

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from .geometry import Cell, Grid, Point, coverage, segment_distance

_MIN_POINTS = {'point': 1, 'polyline': 2, 'polygon': 3}


@dataclass(frozen=True)
class Landmark:
    name: str
    type: str                       # 'point' | 'polyline' | 'polygon'
    points: Tuple[Point, ...]
    description: str = ''


@dataclass(frozen=True)
class World:
    name: str
    bounds: Tuple[float, float, float, float]    # min x, min y, max x, max y
    landmarks: Tuple[Landmark, ...]

    def landmark(self, name: str) -> Landmark:
        for lm in self.landmarks:
            if lm.name == name:
                return lm
        raise KeyError(name)

    @property
    def fingerprint(self) -> str:
        """Hash of the content (not the file's bytes: comments and layout do
        not matter). A recording made against other landmarks or another map
        size does not match it (D48)."""
        data = [self.name, self.bounds,
                [[lm.name, lm.type, lm.points, lm.description]
                 for lm in self.landmarks]]
        return hashlib.sha256(json.dumps(data).encode()).hexdigest()[:16]

    def check_grid(self, grid: Grid):
        """Raise if the grid the drones fly is not the map the file describes."""
        got = (grid.origin_x, grid.origin_y,
               grid.origin_x + grid.width * grid.resolution,
               grid.origin_y + grid.height * grid.resolution)
        if not np.allclose(got, self.bounds):
            raise ValueError(
                f"world {self.name!r} covers {self.bounds}, the grid {got}")


# ---- Loading ----------------------------------------------------------------

def _need(ok, message):
    if not ok:
        raise ValueError(f"landmarks: {message}")


def _keys(data, required, optional, what):
    _need(isinstance(data, dict), f"{what} must be a mapping")
    missing, extra = required - set(data), set(data) - required - optional
    _need(not missing, f"{what} lacks {sorted(missing)}")
    _need(not extra, f"{what} has unknown keys {sorted(map(str, extra))}")


def _numbers(values, count, what) -> Tuple[float, ...]:
    _need(isinstance(values, list) and len(values) == count and
          all(isinstance(v, (int, float)) and not isinstance(v, bool) and
              math.isfinite(v) for v in values),
          f"{what} must be {count} finite numbers")
    return tuple(float(v) for v in values)


def parse_world(text: str) -> World:
    data = yaml.safe_load(text)
    _keys(data, {'world', 'bounds', 'landmarks'}, set(), 'the file')
    _need(isinstance(data['world'], str) and data['world'], 'world must be a name')
    x0, y0, x1, y1 = bounds = _numbers(data['bounds'], 4, 'bounds')
    _need(x0 < x1 and y0 < y1, 'bounds must be min x, min y, max x, max y')
    _need(isinstance(data['landmarks'], list) and data['landmarks'],
          'landmarks must be a non-empty list')
    out: Dict[str, Landmark] = {}
    for item in data['landmarks']:
        _keys(item, {'name', 'type', 'points'}, {'description'}, 'a landmark')
        name, kind, desc = item['name'], item['type'], item.get('description', '')
        _need(isinstance(name, str) and name, f"bad name {name!r}")
        _need(name not in out, f"{name!r} appears twice")
        _need(kind in _MIN_POINTS, f"{name!r} has unknown type {kind!r}")
        _need(isinstance(desc, str), f"{name!r} description must be text")
        _need(isinstance(item['points'], list), f"{name!r} points must be a list")
        points = tuple(_numbers(p, 2, f"{name!r} point") for p in item['points'])
        n = _MIN_POINTS[kind]
        _need(len(points) == 1 if kind == 'point' else len(points) >= n,
              f"{name!r}: a {kind} needs {n}{'' if kind == 'point' else '+'} points")
        _need(all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in points),
              f"{name!r} leaves the map")
        out[name] = Landmark(name, kind, points, desc)
    return World(data['world'], bounds, tuple(out.values()))


def load_world(path) -> World:
    with open(path, encoding='utf-8') as f:
        return parse_world(f.read())


# ---- Misleading clues (D47) -------------------------------------------------

def landmark_distance(lm: Landmark, x, y):
    """Distance from points (x, y) to a landmark. Arrays welcome. A polygon's
    inside counts as distance 0."""
    pts = lm.points
    if lm.type == 'point':
        segments = [(pts[0], pts[0])]
    elif lm.type == 'polyline':
        segments = list(zip(pts, pts[1:]))
    else:
        segments = list(zip(pts, pts[1:] + pts[:1]))
    dist = np.min([segment_distance(x, y, a, b) for a, b in segments], axis=0)
    return np.where(_in_polygon(x, y, pts), 0.0, dist) if lm.type == 'polygon' else dist


def near_landmark(grid: Grid, lm: Landmark, radius: float) -> np.ndarray:
    """Fraction of each cell within ``radius`` of a landmark, [row, col]."""
    return coverage(grid, lambda x, y: landmark_distance(lm, x, y) <= radius)


def _in_polygon(x, y, pts) -> np.ndarray:
    """Even-odd ray cast on arrays, as geometry.point_in_quad."""
    out = np.zeros(np.shape(x), dtype=bool)
    for (xi, yi), (xj, yj) in zip(pts, pts[-1:] + pts[:-1]):
        with np.errstate(divide='ignore', invalid='ignore'):
            cross = (xj - xi) * (y - yi) / (yj - yi) + xi
        out ^= ((yi > y) != (yj > y)) & (x < cross)
    return out


def _zone(grid: Grid, cell: Cell) -> List[Cell]:
    """The cell and its 8 neighbours, on the grid."""
    col, row = cell
    return [(c, r) for c in range(col - 1, col + 2)
            for r in range(row - 1, row + 2)
            if 0 <= c < grid.width and 0 <= r < grid.height]


def wrong_landmarks(grid: Grid, world: World, true_names: Iterable[str],
                    targets: Sequence[Point], radius: float) -> List[str]:
    """Landmarks a misleading clue may name, in file order (D47).

    Not the true landmark of any target, and nothing within ``radius`` of it
    reaches any target's cell or that cell's 8 neighbours.
    """
    true_names = set(true_names)
    for name in true_names:
        world.landmark(name)                  # a typo is an error, not a pass
    cells = [grid.cell_of(*t) for t in targets]
    if None in cells:
        raise ValueError("a target lies off the grid")
    keep_clear = {z for cell in cells for z in _zone(grid, cell)}
    out = []
    for lm in world.landmarks:
        if lm.name in true_names:
            continue
        near = near_landmark(grid, lm, radius)
        if all(near[row, col] == 0.0 for col, row in keep_clear):
            out.append(lm.name)
    return out


def misleading_swap(grid: Grid, world: World, true_names: Sequence[str],
                    targets: Sequence[Point], radius: float,
                    rng) -> Optional[Dict[str, str]]:
    """True landmark -> the wrong one a misleading clue names instead (D47).

    Every distinct true landmark gets a different wrong one, drawn by ``rng``
    (numpy Generator) from wrong_landmarks. None if there are too few: the
    caller then draws other target positions, before any arm runs.
    """
    distinct = list(dict.fromkeys(true_names))
    wrong = wrong_landmarks(grid, world, distinct, targets, radius)
    if len(wrong) < len(distinct):
        return None
    picks = rng.choice(len(wrong), size=len(distinct), replace=False)
    return {name: wrong[int(i)] for name, i in zip(distinct, picks)}
