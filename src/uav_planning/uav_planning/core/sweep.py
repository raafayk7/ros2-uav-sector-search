"""Leader boustrophedon sweep: pure functions, no ROS.

The sweep flies parallel lanes over the sector grid, back and forth like a
lawnmower, so that every cell centre passes under the camera at least once.
leader_node counts a cell as looked at in a frame when the cell's centre is
inside the camera's ground footprint, so coverage reduces to one rule: every
cell centre must lie within the footprint's half-width of some lane.

Choices pinned in docs/planner-decisions.md (D39):

* Lane spacing comes from the footprint's *short* side times ``overlap``
  (0.8). A circle of that radius fits inside the footprint at any heading, so
  coverage does not depend on which way ArduPilot turns the drone.
* Lanes run between the first and last cell-centre rows, inside the arena.
* One waypoint at a time; the next goes out once the drone is within
  ``tolerance`` of the current one and at its layer. No horizontal command
  before the drone first reaches its layer (D26).
* "Complete" means the last waypoint was reached. Whether every cell really
  was observed is the heatmap's business, not this module's.

The sweep never looks at detections: the roadmap's coverage guarantee says the
leader always finishes, so nothing here can stop it early.

Same conventions as planner.py: arena ENU frame, x East, y North, metres.
"""

import math
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Sequence, Tuple

from .follower import at_layer
from .planner import GridSpec

WAITING = 'WAITING'
SWEEPING = 'SWEEPING'
COMPLETE = 'COMPLETE'

# Lane direction: 'north' lanes run North-South (spaced in x), 'east' lanes
# East-West. Not 'y'/'x': ROS's parameter YAML parser reads a bare y as true.
LANE_AXES = ('north', 'east')

Point = Tuple[float, float]


@dataclass(frozen=True)
class Camera:
    """Pinhole intrinsics, as a CameraInfo's K and image size."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def footprint_half_extents(camera: Camera, altitude: float) -> Tuple[float, float]:
    """Ground half-extents (along image u, along image v) of a nadir camera
    ``altitude`` metres above flat ground. Uses the nearer image edge, so an
    off-centre principal point never overstates the footprint."""
    half_u = altitude * min(camera.cx, camera.width - camera.cx) / camera.fx
    half_v = altitude * min(camera.cy, camera.height - camera.cy) / camera.fy
    return half_u, half_v


def lane_half_width(camera: Camera, altitude: float, overlap: float = 0.8) -> float:
    """Half the distance between lanes: the short footprint side times
    ``overlap`` (D39). Valid at any heading."""
    if not 0.0 < overlap <= 1.0:
        raise ValueError(f"overlap must be in (0, 1], got {overlap}")
    return overlap * min(footprint_half_extents(camera, altitude))


def lane_positions(lo: float, hi: float, half_width: float) -> List[float]:
    """Fewest evenly spaced lanes whose strips of ``half_width`` cover
    [lo, hi]. Lane i sits at the centre of the i-th of n equal slices."""
    if half_width <= 0:
        raise ValueError(f"half_width must be positive, got {half_width}")
    span = hi - lo
    n = max(1, math.ceil(span / (2.0 * half_width) - 1e-9))
    return [lo + span * (2 * i + 1) / (2 * n) for i in range(n)]


def _centre_span(origin: float, cells: int, resolution: float) -> Tuple[float, float]:
    """First and last cell-centre coordinate along one grid axis."""
    return origin + 0.5 * resolution, origin + (cells - 0.5) * resolution


def lanes(grid: GridSpec, half_width: float, axis: str = 'north') -> List[Tuple[Point, Point]]:
    """Each lane as its two end points, in lane-index order (West to East for
    'north' lanes, South to North for 'east' lanes)."""
    if axis not in LANE_AXES:
        raise ValueError(f"axis must be one of {LANE_AXES}, got {axis!r}")
    xs = _centre_span(grid.origin_x, grid.width, grid.resolution)
    ys = _centre_span(grid.origin_y, grid.height, grid.resolution)
    if axis == 'north':
        return [((x, ys[0]), (x, ys[1])) for x in lane_positions(*xs, half_width)]
    return [((xs[0], y), (xs[1], y)) for y in lane_positions(*ys, half_width)]


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def plan_sweep(grid: GridSpec, half_width: float, start: Point,
               axis: str = 'north',
               lane_order: Optional[Sequence[int]] = None) -> List[Point]:
    """The sweep's waypoints, two per lane.

    ``lane_order`` lists lane indices in flying order (a permutation of all
    lanes: the sweep always covers everything). By default the lanes go in
    index order from whichever outer lane is nearer ``start``. Each lane is
    entered at the end nearer the previous point, which makes consecutive
    lanes alternate direction: the boustrophedon.
    """
    all_lanes = lanes(grid, half_width, axis)
    n = len(all_lanes)
    if lane_order is None:
        first, last = all_lanes[0], all_lanes[-1]
        near_first = min(_dist(start, first[0]), _dist(start, first[1]))
        near_last = min(_dist(start, last[0]), _dist(start, last[1]))
        order = list(range(n)) if near_first <= near_last else list(range(n - 1, -1, -1))
    else:
        order = list(lane_order)
        if sorted(order) != list(range(n)):
            raise ValueError(
                f"lane_order {order} must list each of the {n} lanes once")
    waypoints: List[Point] = []
    here = start
    for i in order:
        a, b = all_lanes[i]
        if _dist(here, b) < _dist(here, a):
            a, b = b, a
        waypoints += [a, b]
        here = b
    return waypoints


def path_length(start: Point, waypoints: Sequence[Point]) -> float:
    """Horizontal metres from ``start`` through every waypoint."""
    total, here = 0.0, start
    for p in waypoints:
        total += _dist(here, p)
        here = p
    return total


def parse_lane_order(text: str) -> Optional[List[int]]:
    """Launch string "2,0,1" -> [2, 0, 1]; empty -> None (default order)."""
    text = text.strip()
    if not text:
        return None
    return [int(v) for v in text.split(',')]


class SweepOutput(NamedTuple):
    status: str
    index: int                      # waypoint now being flown to
    command: Optional[Point]        # arena (x, y) to fly to at layer
    reason: str


def sweep_tick(waypoints: Optional[Sequence[Point]], index: int,
               pose: Optional[Tuple[float, float, float]], layer: float,
               tolerance: float = 1.0,
               layer_tolerance: float = 0.5) -> SweepOutput:
    """One tick: status, the waypoint index after this tick, and the command.

    Advances at most one waypoint per tick. The layer gate (D26) holds back
    only the first command; mid-sweep a drone off its layer keeps flying (the
    command re-sends the layer altitude) but cannot advance until back on it.
    """
    if waypoints is None:
        return SweepOutput(WAITING, index, None, 'no-plan')
    if index >= len(waypoints):
        return SweepOutput(COMPLETE, index, None, 'done')
    if pose is None:
        return SweepOutput(WAITING, index, None, 'no-pose')
    x, y, z = pose
    on_layer = at_layer(z, layer, layer_tolerance)
    if index == 0 and not on_layer:
        return SweepOutput(WAITING, index, None, 'climbing')
    if on_layer and _dist((x, y), waypoints[index]) <= tolerance:
        index += 1
        if index >= len(waypoints):
            return SweepOutput(COMPLETE, index, None, 'done')
        return SweepOutput(SWEEPING, index, waypoints[index], 'advanced')
    return SweepOutput(SWEEPING, index, waypoints[index], 'flying')
