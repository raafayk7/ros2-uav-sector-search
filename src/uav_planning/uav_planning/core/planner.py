"""Follower sector planner: pure functions, no ROS.

Every rule here is pinned in docs/planner-decisions.md (D-numbers below), which
wins over the thesis wherever they disagree. Nothing in this module imports
rclpy or a message type: callers hand in plain numbers and get plain data back.

Conventions (named once, used everywhere):

* A sector is ``Sector(col, row)``: col counts along x (East), row along y
  (North). leader_node's ``_world_to_cell`` returns ``(col, row)`` but indexes
  its array ``[row, col]``; we follow the same split. Map values are
  ``values[row][col]`` and an OccupancyGrid's flat ``data`` is
  ``data[row * width + col]``. A sector's *index* is that same flat position,
  which makes "lowest index" mean row-major order (D20).
* World coordinates are the ENU arena frame localization_node publishes.
* Time is a plain float of seconds (sim time in the node).
* Map values are per-cell, independent probabilities in [0, 1] that the cell
  holds a target. They do not sum to 1 (D31). Unknown (-1) counts as 0.5 (D18).

The decision loop a caller runs each tick is ``plan_step``: it keeps a flying
drone committed to its goal and only re-decides on arrival or when a peer
blocks the goal (D10). ``choose_goal`` is the pure choice underneath.
"""

import math
import random
from dataclasses import dataclass
from typing import Iterable, List, NamedTuple, Optional, Sequence, Tuple

# WGS84 semi-major axis (metres). Must equal localization_node's WGS84_A: the
# two projections are inverses and a round-trip test pins them together.
WGS84_A = 6378137.0

# Floating-point slack for "more than margin" and "equal value" comparisons.
_EPS = 1e-9

UNKNOWN_VALUE = 0.5


class Sector(NamedTuple):
    col: int  # along x / East
    row: int  # along y / North


@dataclass(frozen=True)
class GridSpec:
    """Sector grid geometry. Mirrors an OccupancyGrid's info block."""

    width: int
    height: int
    resolution: float = 5.0
    origin_x: float = 0.0
    origin_y: float = 0.0

    def index(self, s: Sector) -> int:
        """Flat row-major position, ``row * width + col``."""
        return s.row * self.width + s.col

    def contains(self, s: Sector) -> bool:
        return 0 <= s.col < self.width and 0 <= s.row < self.height

    def sectors(self) -> Iterable[Sector]:
        """Every sector in index order."""
        for row in range(self.height):
            for col in range(self.width):
                yield Sector(col, row)

    def cell_of(self, x: float, y: float) -> Sector:
        """Sector a point falls in, even if that is outside the grid."""
        return Sector(int(math.floor((x - self.origin_x) / self.resolution)),
                      int(math.floor((y - self.origin_y) / self.resolution)))

    def world_to_sector(self, x: float, y: float) -> Optional[Sector]:
        """World (x, y) -> Sector, or None outside the grid (D20)."""
        s = self.cell_of(x, y)
        return s if self.contains(s) else None

    def sector_centre(self, s: Sector) -> Tuple[float, float]:
        return (self.origin_x + (s.col + 0.5) * self.resolution,
                self.origin_y + (s.row + 0.5) * self.resolution)

    def zone(self, s: Sector) -> frozenset:
        """The sector plus its 8 neighbours, clipped to the grid (D24, D25)."""
        return frozenset(
            n for n in (Sector(s.col + dc, s.row + dr)
                        for dr in (-1, 0, 1) for dc in (-1, 0, 1))
            if self.contains(n))

    def neighbours(self, s: Sector) -> List[Sector]:
        """The 8-connected neighbourhood L(s) without s itself, in index order."""
        return sorted((n for n in self.zone(s) if n != s), key=self.index)


def chebyshev(a: Sector, b: Sector) -> int:
    return max(abs(a.col - b.col), abs(a.row - b.row))


# ---------------------------------------------------------------------------
# Map values
# ---------------------------------------------------------------------------

def cell_value(raw: int) -> float:
    """OccupancyGrid int8 -> probability. -1 (unknown) counts as 0.5 (D18)."""
    if raw < 0:
        return UNKNOWN_VALUE
    return min(raw, 100) / 100.0


def values_from_occupancy(data: Sequence[int], grid: GridSpec) -> List[List[float]]:
    """Flat OccupancyGrid data -> ``values[row][col]`` (D18, D31)."""
    if len(data) != grid.width * grid.height:
        raise ValueError(
            f"map has {len(data)} cells, grid is {grid.width}x{grid.height}")
    return [[cell_value(data[row * grid.width + col])
             for col in range(grid.width)] for row in range(grid.height)]


def _value(values: Sequence[Sequence[float]], s: Sector) -> float:
    return values[s.row][s.col]


# ---------------------------------------------------------------------------
# Peers and the blocked set
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class PlannerParams:
    move_margin: float = 0.02     # D16: move only if best beats current by more
    peer_timeout: float = 5.0     # D21, D28: seconds of silence before ignoring
    arrival_radius: float = 2.0   # D27: horizontal metres to the goal centre


@dataclass(frozen=True)
class Peer:
    """What this drone last heard about another follower.

    ``pose`` is the peer's (x, y) in the arena frame and ``goal`` its announced
    goal sector, each with the time it was last received. Either may be absent.
    """

    sysid: int
    pose: Optional[Tuple[float, float]] = None
    pose_stamp: float = -math.inf
    goal: Optional[Sector] = None
    goal_stamp: float = -math.inf


def _fresh(stamp: float, now: float, params: PlannerParams) -> bool:
    return now - stamp <= params.peer_timeout


def pose_valid(peer: Peer, now: float, params: PlannerParams) -> bool:
    return peer.pose is not None and _fresh(peer.pose_stamp, now, params)


def goal_valid(peer: Peer, now: float, params: PlannerParams) -> bool:
    """A goal lives ``peer_timeout`` after its last stamp, whatever the pose
    is doing (D28): stopping the 1 Hz repeats is how a goal ends."""
    return peer.goal is not None and _fresh(peer.goal_stamp, now, params)


def peer_arrived(grid: GridSpec, peer: Peer, now: float,
                 params: PlannerParams) -> bool:
    """Is the peer parked where its pose says (so its position blocks)?

    Arrived = pose is fresh and (it has no valid goal, or it is within
    ``arrival_radius`` of that goal's centre). A drone with no valid goal, for
    example a disabled one, therefore still blocks where it is parked, which is
    the safe reading. D25 leaves this open; see the session notes.
    """
    if not pose_valid(peer, now, params):
        return False
    if not goal_valid(peer, now, params):
        return True
    return is_arrived(grid, peer.pose, peer.goal, params)


def is_arrived(grid: GridSpec, pose: Tuple[float, float], goal: Sector,
               params: PlannerParams) -> bool:
    """Horizontal arrival (D27). The altitude half of arrival is the node's."""
    gx, gy = grid.sector_centre(goal)
    return math.hypot(pose[0] - gx, pose[1] - gy) <= params.arrival_radius


def blocked_set(grid: GridSpec, my_sysid: int, peers: Iterable[Peer],
                now: float, params: PlannerParams = PlannerParams()) -> frozenset:
    """Sectors this drone may not pick (D24, D25, D21, D28).

    * An arrived peer blocks the zone around where it is parked.
    * A flying peer's position blocks nothing.
    * A peer's valid goal zone blocks only if that peer has the lower sysid, so
      when two goals conflict the higher sysid yields and the lower keeps its
      goal.
    * Stale poses and expired goals count for nothing.
    """
    blocked = set()
    for peer in peers:
        if peer.sysid == my_sysid:
            continue
        if peer_arrived(grid, peer, now, params):
            here = grid.world_to_sector(*peer.pose)
            if here is not None:
                blocked |= grid.zone(here)
        if peer.sysid < my_sysid and goal_valid(peer, now, params) \
                and grid.contains(peer.goal):
            blocked |= grid.zone(peer.goal)
    return frozenset(blocked)


# ---------------------------------------------------------------------------
# Choosing a goal
# ---------------------------------------------------------------------------

class Decision(NamedTuple):
    goal: Optional[Sector]   # what to fly to / hold at (None: nothing to hold)
    changed: bool            # goal differs from the one passed in
    reason: str              # 'flying' | 'move' | 'hold' | 'nothing-open'


def choose_goal(grid: GridSpec, values: Sequence[Sequence[float]],
                my_pose: Tuple[float, float], current_goal: Optional[Sector],
                blocked: frozenset,
                params: PlannerParams = PlannerParams()) -> Decision:
    """Best open sector anywhere on the grid, with a stay margin (D16, D17, D18).

    The current goal (or, with none yet, the sector under the drone) is always
    a candidate; a blocked one is worth 0. Move only if the best open sector
    beats it by more than ``move_margin``. Ties among the best go to the
    nearest sector (Chebyshev, from the drone's own sector), then the lowest
    index, so the choice is deterministic. Nothing open: keep the goal.
    """
    here = grid.cell_of(*my_pose)
    held = current_goal if current_goal is not None \
        else (here if grid.contains(here) else None)

    open_sectors = [s for s in grid.sectors() if s not in blocked]
    if not open_sectors:
        return Decision(current_goal, False, 'nothing-open')

    held_value = 0.0 if held is None or held in blocked \
        else _value(values, held)
    best = max(_value(values, s) for s in open_sectors)
    if best - held_value <= params.move_margin + _EPS:
        return Decision(held, held != current_goal, 'hold')

    tops = [s for s in open_sectors if _value(values, s) >= best - _EPS]
    goal = min(tops, key=lambda s: (chebyshev(s, here), grid.index(s)))
    return Decision(goal, goal != current_goal, 'move')


def plan_step(grid: GridSpec, values: Sequence[Sequence[float]], my_sysid: int,
              my_pose: Tuple[float, float], current_goal: Optional[Sector],
              peers: Iterable[Peer], now: float,
              params: PlannerParams = PlannerParams()) -> Decision:
    """One planner tick (D10). Re-decide only when there is no goal, the drone
    has arrived, or a peer now blocks the goal; otherwise stay committed."""
    blocked = blocked_set(grid, my_sysid, peers, now, params)
    if (current_goal is not None and current_goal not in blocked
            and not is_arrived(grid, my_pose, current_goal, params)):
        return Decision(current_goal, False, 'flying')
    return choose_goal(grid, values, my_pose, current_goal, blocked, params)


# ---------------------------------------------------------------------------
# Arena <-> geographic (D8)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Datum:
    """The shared world origin every localization_node measures from (D7)."""

    lat: float
    lon: float
    alt: float


def arena_to_lla(datum: Datum, x: float, y: float,
                 up: float = 0.0) -> Tuple[float, float, float]:
    """Arena ENU (x East, y North, up) -> (lat, lon, alt).

    Exact inverse of localization_node's equirectangular projection:
    east = A cos(lat0) rad(lon - lon0), north = A rad(lat - lat0), up = alt - alt0.
    """
    lat = datum.lat + math.degrees(y / WGS84_A)
    lon = datum.lon + math.degrees(
        x / (WGS84_A * math.cos(math.radians(datum.lat))))
    return lat, lon, datum.alt + up


# ---------------------------------------------------------------------------
# Thesis baselines (D4, D17): measured against, never flown
# ---------------------------------------------------------------------------

def sector_utility(grid: GridSpec, values: Sequence[Sequence[float]],
                   s: Sector, others: Iterable[Sector],
                   penalise_peer_cell: bool = False) -> float:
    """Thesis utility: 0 if s lies in another drone's neighbourhood L(x_-a),
    else p. Read literally, L excludes the peer's own sector; set
    ``penalise_peer_cell`` for the closed neighbourhood the planner uses."""
    for o in others:
        if (s in grid.neighbours(o)) or (penalise_peer_cell and s == o):
            return 0.0
    return _value(values, s)


def thesis_dga_step(grid: GridSpec, values: Sequence[Sequence[float]],
                    me: Sector, others: Iterable[Sector],
                    allow_stay: bool = False,
                    penalise_peer_cell: bool = False) -> Sector:
    """Thesis DGA: move to the highest-utility sector in L(x_a).

    Literal by default: no stay action (so it never settles, R10) and the
    peer's own sector is not penalised. Ties go to the lowest index, preferring
    to stay when ``allow_stay`` and tied."""
    others = list(others)
    cands = grid.neighbours(me) + ([me] if allow_stay else [])
    return max(cands, key=lambda s: (
        sector_utility(grid, values, s, others, penalise_peer_cell),
        s == me, -grid.index(s)))


def bll_move_probability(u_alt: float, u_here: float) -> float:
    """Thesis BLL: P(move) = exp(u(a_l)) / (exp(u(a_l)) + exp(u(x_a)))."""
    return math.exp(u_alt) / (math.exp(u_alt) + math.exp(u_here))


class BllStep(NamedTuple):
    sector: Sector       # where the drone is after the step
    candidate: Sector    # the random alternative it considered
    p_move: float
    moved: bool


def bll_step(grid: GridSpec, values: Sequence[Sequence[float]], me: Sector,
             others: Iterable[Sector], rng: random.Random,
             penalise_peer_cell: bool = False) -> BllStep:
    """One BLL update for one drone: pick a random neighbour, softmax between
    moving and staying. The caller picks which drone updates (the thesis picks
    one at random per iteration)."""
    others = list(others)
    candidate = rng.choice(grid.neighbours(me))
    p = bll_move_probability(
        sector_utility(grid, values, candidate, others, penalise_peer_cell),
        sector_utility(grid, values, me, others, penalise_peer_cell))
    moved = rng.random() < p
    return BllStep(candidate if moved else me, candidate, p, moved)
