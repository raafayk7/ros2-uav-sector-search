"""Mission state machine: when a search starts, when it ends, what it found.
No ROS.

The followers already pick their own sectors (planner.py) and the belief map
already confirms finds (D40), so the mission only owns the lifecycle: switch
the planners on, decide when the search is over, switch them off, report.
Choices pinned in docs/planner-decisions.md (D14, D41):

* States: WAITING (until the sweep status, the map and every follower's
  planner status have been seen) -> SEARCHING -> DONE or TIMED_OUT. Dispatch
  and localization are not mission states: each follower dispatches itself
  from the map, and a find is a map event logged inside SEARCHING.
* Planners are enabled only in SEARCHING, so t0 is the moment the mission
  enables them. They are disabled again at the end, which holds each drone
  where it is (D29).
* End rule (D14): the leader's sweep is COMPLETE and every unconfirmed cell is
  below ``threshold`` for ``hold`` seconds. An unknown cell counts as 0.5, so
  nothing unseen can end the mission. The hold is on the wall clock, like the
  belief map whose evidence it waits on.
* Timeout: ``timeout`` seconds of sim time after t0. The caller supplies sim
  time (the node reads it from the followers' pose stamps). Where it is missing,
  wall time stands in (a tick with no sim time, or the gap between t0 and
  the first sim time seen), which ends a run early, never late, at RTF
  below 1.

* Return to find (D41 amended, 2026-11-22): after DONE, ``assign_returns``
  gives each find a follower to hover over it, so the run ends with drones on
  the finds instead of on whatever sectors they had moved on to. The finder
  keeps its own find; a drone that found two keeps the earlier, and the other
  goes to the nearest drone without one. It is the node's job to fly them.

``Mission.tick`` is called once a second with the latest inputs and returns
what to publish. Same conventions as planner.py: cells are (col, row).
"""

import json
from dataclasses import dataclass
from typing import Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

WAITING = 'WAITING'
SEARCHING = 'SEARCHING'
DONE = 'DONE'
TIMED_OUT = 'TIMED_OUT'
ENDED = (DONE, TIMED_OUT)

SWEEP_COMPLETE = 'COMPLETE'   # core/sweep.py's last status

Cell = Tuple[int, int]


@dataclass(frozen=True)
class MissionParams:
    threshold: float = 0.5     # D14: a cell at or above this is still open
    hold: float = 30.0         # D14: wall seconds the map must stay quiet
    timeout: float = 600.0     # D41: sim seconds after t0


class Find(NamedTuple):
    cell: Cell
    drone: str
    wall: float                # mission wall seconds when the mission saw it
    sim: Optional[float]       # mission sim seconds, if sim time was known


class TickOutput(NamedTuple):
    state: str
    enable: bool               # planner_enable for every follower
    changed: bool              # state changed on this tick
    reason: str


def open_cells(occupancy: Sequence[int], width: int,
               confirmed: Iterable[Cell], threshold: float = 0.5) -> List[Cell]:
    """Unconfirmed cells at or above ``threshold``, from OccupancyGrid data
    (0-100, -1 unknown, row-major). Unknown counts as 50."""
    done = set(confirmed)
    limit = threshold * 100.0
    out = []
    for i, v in enumerate(occupancy):
        cell = (i % width, i // width)
        if cell not in done and (50 if v < 0 else v) >= limit:
            out.append(cell)
    return out


class Mission:
    """One mission's state, from WAITING to DONE or TIMED_OUT."""

    def __init__(self, followers: Sequence[str],
                 params: MissionParams = MissionParams()):
        if not followers:
            raise ValueError("a mission needs at least one follower")
        self.followers = list(followers)
        self.params = params
        self.state = WAITING
        self.t0_wall: Optional[float] = None
        self.t0_sim: Optional[float] = None
        self.end_wall: Optional[float] = None
        self.end_sim: Optional[float] = None
        self.quiet_since: Optional[float] = None
        self.sweep_started: Optional[float] = None     # mission wall seconds
        self.sweep_done: Optional[float] = None
        self.finds: Dict[Cell, Find] = {}
        self.open_at_end: List[Cell] = []
        self.status_at_end: Dict[str, Optional[str]] = {}

    # ------------------------------------------------------------------
    def tick(self, wall: float, sim: Optional[float],
             sweep_status: Optional[str],
             occupancy: Optional[Sequence[int]], width: int,
             finds: Iterable[Tuple[Cell, str]],
             follower_status: Dict[str, Optional[str]]) -> TickOutput:
        """One tick. ``finds`` is every confirmed (cell, drone) so far;
        ``follower_status`` maps follower name to its last planner_status."""
        if self.state in ENDED:
            return TickOutput(self.state, False, False, 'ended')

        if self.state == WAITING:
            missing = self._missing(sweep_status, occupancy, follower_status)
            if missing:
                return TickOutput(WAITING, False, False, 'waiting for ' + missing)
            self.state = SEARCHING
            self.t0_wall, self.t0_sim = wall, sim
            return TickOutput(SEARCHING, True, True, 'all inputs seen')

        # SEARCHING
        elapsed_sim = self._elapsed_sim(wall, sim)
        self._log(wall, sim, sweep_status, finds)
        p = self.params
        still_open = open_cells(occupancy, width, self.finds, p.threshold) \
            if occupancy is not None else None
        quiet = (sweep_status == SWEEP_COMPLETE and still_open == [])
        if not quiet:
            self.quiet_since = None
        elif self.quiet_since is None:
            self.quiet_since = wall
        if quiet and wall - self.quiet_since >= p.hold:
            return self._end(DONE, wall, sim, still_open, follower_status,
                             f'quiet for {p.hold:.0f} s')
        if elapsed_sim >= p.timeout:
            return self._end(TIMED_OUT, wall, sim, still_open or [],
                             follower_status, f'{p.timeout:.0f} sim-s')
        if sweep_status != SWEEP_COMPLETE:
            why = 'sweep not complete'
        elif still_open is None:
            why = 'no heatmap'
        elif still_open:
            why = f'{len(still_open)} cells open'
        else:
            why = f'quiet {wall - self.quiet_since:.0f} s'
        return TickOutput(SEARCHING, True, False, why)

    # ------------------------------------------------------------------
    def report(self) -> dict:
        """The mission's outcome as plain data (the node sends it as JSON)."""
        return {
            'state': self.state,
            'followers': self.followers,
            'wall_s': _span(self.t0_wall, self.end_wall),
            'sim_s': _span(self.t0_sim, self.end_sim),
            'sweep_started_s': self.sweep_started,
            'sweep_complete_s': self.sweep_done,
            'finds': [{'cell': list(f.cell), 'drone': f.drone,
                       'wall_s': f.wall, 'sim_s': f.sim}
                      for f in sorted(self.finds.values(), key=lambda f: f.wall)],
            'open_cells': [list(c) for c in self.open_at_end],
            'planner_status': self.status_at_end,
            'params': {'threshold': self.params.threshold,
                       'hold': self.params.hold,
                       'timeout': self.params.timeout},
        }

    # ------------------------------------------------------------------
    def _missing(self, sweep_status, occupancy, follower_status) -> str:
        names = []
        if sweep_status is None:
            names.append('sweep_status')
        if occupancy is None:
            names.append('heatmap')
        names += [f for f in self.followers if follower_status.get(f) is None]
        return ', '.join(names)

    def _log(self, wall, sim, sweep_status, finds):
        t = wall - self.t0_wall
        if sweep_status == SWEEP_COMPLETE:
            if self.sweep_done is None:
                self.sweep_done = t
        elif sweep_status is not None and sweep_status != 'WAITING' \
                and self.sweep_started is None:
            self.sweep_started = t
        for cell, drone in finds:
            cell = tuple(cell)
            if cell not in self.finds:
                self.finds[cell] = Find(cell, drone, t, _span(self.t0_sim, sim))

    def _elapsed_sim(self, wall, sim) -> float:
        if sim is None:
            return wall - self.t0_wall
        if self.t0_sim is None:
            # Sim time first seen after t0: count the gap as if RTF were 1,
            # which overstates sim time below RTF 1, so the timeout is early.
            self.t0_sim = sim - (wall - self.t0_wall)
        return sim - self.t0_sim

    def _end(self, state, wall, sim, still_open, follower_status, why):
        self.state = state
        self.end_wall, self.end_sim = wall, sim
        self.open_at_end = list(still_open)
        self.status_at_end = {f: follower_status.get(f) for f in self.followers}
        return TickOutput(state, False, True, why)


def _span(start: Optional[float], end: Optional[float]) -> Optional[float]:
    return None if start is None or end is None else end - start


# ---- Return to find -------------------------------------------------------

def cell_centre(cell: Cell, resolution: float = 5.0,
                origin: Tuple[float, float] = (0.0, 0.0)) -> Tuple[float, float]:
    """Arena (x, y) of a cell's centre; cells are (col, row)."""
    return (origin[0] + (cell[0] + 0.5) * resolution,
            origin[1] + (cell[1] + 0.5) * resolution)


def assign_returns(finds: Sequence[Tuple[Cell, str]],
                   poses: Dict[str, Tuple[float, float]],
                   followers: Sequence[str], resolution: float = 5.0,
                   origin: Tuple[float, float] = (0.0, 0.0)) -> Dict[str, Cell]:
    """Follower -> the find it should hover over at the end.

    ``finds`` is every (cell, finder) in the order they were found; ``poses``
    maps follower to its latest arena (x, y). A finder takes its own find,
    the earliest one if it found several. Each remaining find goes to the
    nearest follower that has none yet (ties by name), and is left out if
    every follower already has one or none has a pose. No follower gets two.
    """
    plan: Dict[str, Cell] = {}
    rest: List[Cell] = []
    for cell, drone in finds:
        cell = tuple(cell)
        if drone in followers and drone not in plan:
            plan[drone] = cell
        else:
            rest.append(cell)
    for cell in rest:
        centre = cell_centre(cell, resolution, origin)
        free = [f for f in followers if f not in plan and f in poses]
        if not free:
            break
        best = min(free, key=lambda f: (
            (poses[f][0] - centre[0]) ** 2 + (poses[f][1] - centre[1]) ** 2, f))
        plan[best] = cell
    return plan


# ---- Helpers for mission_node (still no ROS) ------------------------------

def parse_pairs(spec: str, cast=float) -> Dict[str, float]:
    """``"iris_2:10,iris_3:12"`` -> ``{"iris_2": 10.0, "iris_3": 12.0}``.
    One string, since an empty array launch param has no type in ROS 2."""
    out = {}
    for item in (s.strip() for s in spec.split(',')):
        if not item:
            continue
        name, sep, value = item.partition(':')
        if not sep or not name.strip():
            raise ValueError(f"'{item}' must be name:value")
        out[name.strip()] = cast(value)
    return out


def parse_names(spec: str) -> List[str]:
    """``"iris_2, iris_3"`` -> ``["iris_2", "iris_3"]``; empty items dropped."""
    return [s.strip() for s in spec.split(',') if s.strip()]


def parse_finds(text: str) -> List[Tuple[Cell, str]]:
    """belief_map_node's /confirmed_finds JSON (a list of {cell, drone, t})
    -> [(cell, drone)]. Raises ValueError on anything else."""
    try:
        items = json.loads(text)
        return [((int(f['cell'][0]), int(f['cell'][1])), str(f['drone']))
                for f in items]
    except (ValueError, TypeError, KeyError, IndexError) as e:
        raise ValueError(f"not a confirmed_finds list: {e}") from None


class SimClock:
    """Sim time from the latest stamped message seen (the node uses the
    followers' pose stamps), carried forward by wall time since it
    arrived. Below RTF 1 that overstates sim time by at most the gap between
    messages, so a timeout read from it comes early, never late."""

    def __init__(self):
        self.stamp: Optional[float] = None
        self.received: Optional[float] = None

    def update(self, stamp: float, wall: float):
        if self.stamp is None or stamp >= self.stamp:
            self.stamp, self.received = stamp, wall

    def now(self, wall: float) -> Optional[float]:
        if self.stamp is None:
            return None
        return self.stamp + max(0.0, wall - self.received)
