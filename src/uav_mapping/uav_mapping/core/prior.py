"""The language prior: an operator's clue -> a starting belief (no ROS).

The language model's only output is this prior. It returns text, the text is
parsed strictly into regions, and the regions become an array of starting
probabilities. Nothing else of it goes anywhere: no setpoint, no mode change.

Rules, all decided with the user (docs/planner-decisions.md):

* D44, rasterization by area overlap. A cell gets the region's value times
  the fraction of the cell inside it, so a small region never vanishes and a
  big one is not inflated.
* D45, raise only. A cell starts at 0.5 + 0.5 * alpha * u, u in [0, 1] (the
  roadmap's alpha * prior + (1 - alpha) * 0.5 with prior = 0.5 + 0.5 u). Cells
  in no region start at 0.5: unknown, as without a clue. alpha 0.5 by default,
  0.7 at most. A region's value is weight * confidence.
* D46, several targets. A flat list of regions, one per place; overlaps take
  the maximum; nothing is normalized, so naming more places weakens none.
* D48, replay. A recording is found by world, clue (case and spacing aside)
  and the landmark file's fingerprint, exactly or not at all.

Contract, one JSON object and nothing around it:

  {"regions": [
    {"shape": "circle", "center": [x, y], "radius": r,
     "weight": w, "confidence": c},
    {"shape": "capsule", "start": [x, y], "end": [x, y], "radius": r,
     "weight": w, "confidence": c}]}

Arena ENU metres; radius > 0; weight and confidence in [0, 1]. Anything else
is invalid: one retry, then a uniform prior with a warning. A region that
reaches past the map is cut off at its edge; one wholly off it is dropped.
"""

import json
import math
from dataclasses import dataclass, field
from typing import Callable, List, NamedTuple, Optional, Sequence, Tuple, Union

import numpy as np

from .geometry import Grid, Point, coverage, segment_distance
from .landmarks import World

ALPHA = 0.5            # D45
ALPHA_MAX = 0.7        # the roadmap's cap: the 0.5 share never vanishes
MAX_REGIONS = 16
MIN_RADIUS = 1.0       # m: smaller regions are widened to this, so the overlap
                       # lattice never steps over one


class PriorError(ValueError):
    """The language model's output breaks the contract."""


class NoRecording(LookupError):
    """Replay has nothing recorded for this clue."""


# ---- Regions ----------------------------------------------------------------

@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float
    weight: float
    confidence: float


@dataclass(frozen=True)
class Capsule:
    """Everything within ``radius`` of the segment start-end."""

    start: Point
    end: Point
    radius: float
    weight: float
    confidence: float


Region = Union[Circle, Capsule]

_FIELDS = {'circle': ('center',), 'capsule': ('start', 'end')}
_COMMON = ('radius', 'weight', 'confidence')


def _number(value, what) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PriorError(f"{what} must be a number, got {value!r}")
    try:
        value = float(value)
    except OverflowError:
        value = math.inf
    if not math.isfinite(value):
        raise PriorError(f"{what} must be finite")
    return value


def _point(value, what) -> Point:
    if not isinstance(value, list) or len(value) != 2:
        raise PriorError(f"{what} must be [x, y], got {value!r}")
    return _number(value[0], what), _number(value[1], what)


def _region(item, n) -> Region:
    what = f"region {n}"
    if not isinstance(item, dict):
        raise PriorError(f"{what} must be an object")
    shape = item.get('shape')
    if not isinstance(shape, str) or shape not in _FIELDS:
        raise PriorError(f"{what} has unknown shape {shape!r}")
    want = {'shape', *_FIELDS[shape], *_COMMON}
    if set(item) != want:
        raise PriorError(f"{what} ({shape}) must have exactly {sorted(want)}, "
                         f"got {sorted(item)}")
    points = [_point(item[k], f"{what} {k}") for k in _FIELDS[shape]]
    radius, weight, confidence = (_number(item[k], f"{what} {k}") for k in _COMMON)
    if radius <= 0.0:
        raise PriorError(f"{what} radius must be positive, got {radius}")
    for name, v in (('weight', weight), ('confidence', confidence)):
        if not 0.0 <= v <= 1.0:
            raise PriorError(f"{what} {name} must be in [0, 1], got {v}")
    if shape == 'circle':
        return Circle(points[0], radius, weight, confidence)
    return Capsule(points[0], points[1], radius, weight, confidence)


def _unique_keys(pairs):
    out = dict(pairs)
    if len(out) != len(pairs):
        raise PriorError("an object repeats a key")
    return out


def _no_constants(name):
    raise PriorError(f"{name} is not JSON")


def parse_regions(text: str) -> List[Region]:
    """The contract, strictly. Raises PriorError on anything else. An empty
    list is valid: the clue said nothing about where."""
    try:
        data = json.loads(text, object_pairs_hook=_unique_keys,
                          parse_constant=_no_constants)
    except PriorError:
        raise
    except (TypeError, ValueError, RecursionError) as e:
        raise PriorError(f"not JSON: {e}") from None
    if not isinstance(data, dict) or set(data) != {'regions'}:
        raise PriorError('the output must be {"regions": [...]} and nothing else')
    if not isinstance(data['regions'], list):
        raise PriorError("regions must be a list")
    if len(data['regions']) > MAX_REGIONS:
        raise PriorError(f"{len(data['regions'])} regions, at most {MAX_REGIONS}")
    return [_region(item, n) for n, item in enumerate(data['regions'])]


# ---- Regions -> grid --------------------------------------------------------

def overlap(grid: Grid, region: Region) -> np.ndarray:
    """Fraction of each cell inside the region, [row, col] (D44)."""
    a, b = (region.center, region.center) if isinstance(region, Circle) \
        else (region.start, region.end)
    radius = max(region.radius, MIN_RADIUS)
    return coverage(grid, lambda x, y: segment_distance(x, y, a, b) <= radius)


def rasterize(grid: Grid, regions: Sequence[Region]) -> np.ndarray:
    """The clue as u in [0, 1] per cell, [row, col]: 0 = nothing said (D46)."""
    u = np.zeros((grid.height, grid.width))
    for r in regions:
        u = np.maximum(u, r.weight * r.confidence * overlap(grid, r))
    return u


def starting_belief(u: np.ndarray, alpha: float = ALPHA) -> np.ndarray:
    """Starting probabilities for BeliefMap (D45)."""
    if not 0.0 <= alpha <= ALPHA_MAX:
        raise ValueError(f"alpha must be in [0, {ALPHA_MAX}], got {alpha}")
    return 0.5 + 0.5 * alpha * u


# ---- Providers --------------------------------------------------------------

def clue_key(clue: str) -> str:
    """The clue with case and spacing ignored, nothing else (D48)."""
    return ' '.join(clue.split()).casefold()


class Recording(NamedTuple):
    world: str
    clue: str
    landmarks: str      # World.fingerprint it was made against
    source: str         # 'live' (a language model wrote it) | 'hand-written'
    model: str          # 'live' only
    date: str           # 'live' only
    output: str         # the raw text, exactly as returned


_SOURCES = ('live', 'hand-written')


def parse_recordings(text: str) -> List[Recording]:
    """The committed recordings. Strict: a bad file is our mistake."""
    data = json.loads(text)
    if not isinstance(data, dict) or set(data) != {'recordings'}:
        raise ValueError('recordings file must be {"recordings": [...]}')
    out = []
    for n, item in enumerate(data['recordings']):
        if not isinstance(item, dict) or set(item) != set(Recording._fields) or \
                not all(isinstance(v, str) for v in item.values()):
            raise ValueError(f"recording {n} must have exactly the text fields "
                             f"{sorted(Recording._fields)}")
        rec = Recording(**item)
        if rec.source not in _SOURCES:
            raise ValueError(f"recording {n} has unknown source {rec.source!r}")
        if (rec.source == 'live') != bool(rec.model and rec.date):
            raise ValueError(f"recording {n}: a live recording names its model "
                             "and date, a hand-written one neither")
        out.append(rec)
    return out


class ReplayProvider:
    """Answers from committed recordings, exactly or not at all (D48).

    require_live refuses a file holding anything a language model did not
    write; the eval sets it, so no hand-written prior reaches a result.
    """

    name = 'replay'

    def __init__(self, recordings: Sequence[Recording], require_live=False):
        if require_live:
            bad = [r.clue for r in recordings if r.source != 'live']
            if bad:
                raise ValueError(f"not live recordings: {bad}")
        self._outputs = {}
        for r in recordings:
            key = (r.world, r.landmarks, clue_key(r.clue))
            if key in self._outputs:
                raise ValueError(f"two recordings for {r.clue!r} in {r.world!r}")
            self._outputs[key] = r.output

    def __call__(self, clue: str, world: World) -> str:
        key = (world.name, world.fingerprint, clue_key(clue))
        if key not in self._outputs:
            raise NoRecording(
                f"no recording for {clue!r} in world {world.name!r} with "
                f"landmarks {world.fingerprint}")
        return self._outputs[key]


def load_replay(path, require_live=False) -> ReplayProvider:
    with open(path, encoding='utf-8') as f:
        return ReplayProvider(parse_recordings(f.read()), require_live)


# ---- Clue -> prior ----------------------------------------------------------

@dataclass(frozen=True)
class Prior:
    start: np.ndarray               # starting probabilities, [row, col]
    regions: Tuple[Region, ...]     # what it was built from
    failed: bool                    # True: uniform because something went wrong
    warnings: Tuple[str, ...] = field(default=())


def make_prior(clue: str, world: World, grid: Grid,
               provider: Callable[[str, World], str], alpha: float = ALPHA,
               strict: bool = False) -> Prior:
    """The whole chain: ask, parse, one retry if invalid, rasterize, blend.

    Whatever the provider does, a mission gets a prior: uniform if need be,
    with ``failed`` set and the reason in ``warnings`` for the caller to log.
    ``strict`` (the eval) raises instead, since a silent uniform prior would
    contaminate the other arms. A grid that does not match the world raises
    either way: that is a launch mistake, not a bad answer.
    """
    world.check_grid(grid)
    starting_belief(np.zeros(1), alpha)       # a bad alpha fails before asking
    warnings: List[str] = []
    for attempt in (1, 2):
        try:
            regions, notes = _on_map(grid, parse_regions(provider(clue, world)))
            return Prior(starting_belief(rasterize(grid, regions), alpha),
                         tuple(regions), False, tuple(warnings + notes))
        except NoRecording as e:              # asking again cannot help
            error: Optional[Exception] = e
            warnings.append(str(e))
            break
        except Exception as e:                # noqa: BLE001 - see docstring
            error = e
            warnings.append(f"attempt {attempt}: {type(e).__name__}: {e}")
    if strict:
        raise error
    warnings.append('flying with a uniform prior')
    return Prior(np.full((grid.height, grid.width), 0.5), (), True,
                 tuple(warnings))


def _on_map(grid: Grid, regions: Sequence[Region]):
    """Regions that touch the map, and a note for each one changed. If there
    were regions and none is left, the output is invalid."""
    kept, notes = [], []
    for n, r in enumerate(regions):
        if not overlap(grid, r).any():
            notes.append(f"region {n} lies off the map, dropped")
            continue
        if r.radius < MIN_RADIUS:
            notes.append(f"region {n} widened to {MIN_RADIUS} m")
        kept.append(r)
    if regions and not kept:
        raise PriorError("every region lies off the map")
    return kept, notes
