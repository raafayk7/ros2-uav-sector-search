"""What follower_planner_node does each tick, as pure functions (no ROS).

planner.py decides *which* sector; this module adds what the node owns on top
of that decision (docs/planner-decisions.md): the altitude gate (D26), the
status a drone reports (D12), when a flight command goes out (D8), the hold on
disable (D29), and parsing the launch strings for peers (D9) and the stub map
(D19). The node only translates messages to and from these calls.

Status names are fixed by D12. Reading used here, from the state diagram:
HOLDING = not flying anywhere (disabled, no pose, no map, not yet at layer, or
nothing to fly to); ASCENDING = flying to a goal (climbing the map, not in
altitude); ARRIVED = within ``arrival_radius`` of the goal centre and at layer.
"""

import math
from dataclasses import dataclass
from typing import Iterable, List, NamedTuple, Optional, Sequence, Tuple

from .planner import GridSpec, Peer, PlannerParams, Sector, is_arrived, plan_step

ASCENDING = 'ASCENDING'
ARRIVED = 'ARRIVED'
HOLDING = 'HOLDING'


class TickOutput(NamedTuple):
    status: str
    goal: Optional[Sector]                 # announce on goal_sector; None: nothing
    command: Optional[Tuple[float, float]]  # arena (x, y) to fly to at layer
    reason: str


def at_layer(z: float, layer: float, tolerance: float = 0.5) -> bool:
    """D26: within ``tolerance`` metres of the drone's altitude layer."""
    return abs(z - layer) <= tolerance


def follower_tick(grid: Optional[GridSpec],
                  values: Optional[Sequence[Sequence[float]]],
                  my_sysid: int,
                  my_pose: Optional[Tuple[float, float, float]],
                  layer: float,
                  current_goal: Optional[Sector],
                  peers: Iterable[Peer],
                  now: float,
                  params: PlannerParams = PlannerParams(),
                  layer_tolerance: float = 0.5,
                  enabled: bool = True) -> TickOutput:
    """One 1 Hz tick: status, goal to announce, and command to send.

    The altitude gate only holds back the *first* goal. A drone already flying
    keeps its goal if it drifts off layer, since every command re-sends the
    layer altitude; it just cannot report ARRIVED until it is back on layer.
    """
    if not enabled:
        return TickOutput(HOLDING, None, None, 'disabled')
    if my_pose is None:
        return TickOutput(HOLDING, None, None, 'no-pose')
    if grid is None or values is None:
        return TickOutput(HOLDING, None, None, 'no-map')
    x, y, z = my_pose
    on_layer = at_layer(z, layer, layer_tolerance)
    if current_goal is None and not on_layer:
        return TickOutput(HOLDING, None, None, 'below-layer')

    d = plan_step(grid, values, my_sysid, (x, y), current_goal, peers, now, params)
    if d.goal is None:
        return TickOutput(HOLDING, None, None, d.reason)
    if on_layer and is_arrived(grid, (x, y), d.goal, params):
        return TickOutput(ARRIVED, d.goal, None, d.reason)
    return TickOutput(ASCENDING, d.goal, grid.sector_centre(d.goal), d.reason)


def hold_command(my_pose: Optional[Tuple[float, float, float]]
                 ) -> Optional[Tuple[float, float]]:
    """D29: on disable, one goal at the current (x, y); the caller adds the
    layer. None when the pose is unknown, so nothing is sent."""
    if my_pose is None:
        return None
    return (my_pose[0], my_pose[1])


def make_peer(grid: Optional[GridSpec], sysid: int,
              pose_xy: Optional[Tuple[float, float]], pose_stamp: float,
              goal_xy: Optional[Tuple[float, float]], goal_stamp: float) -> Peer:
    """Raw last-heard peer data -> library Peer.

    ``goal_sector`` carries the goal's centre as a point, so the sector is
    recovered on our own grid. With no map yet, or a point off the grid, the
    peer has no goal.
    """
    goal = None
    if grid is not None and goal_xy is not None:
        goal = grid.world_to_sector(*goal_xy)
    return Peer(sysid,
                pose_xy, pose_stamp if pose_xy is not None else -math.inf,
                goal, goal_stamp if goal is not None else -math.inf)


# ---------------------------------------------------------------------------
# Launch strings
# ---------------------------------------------------------------------------

def parse_peers(spec: str) -> List[Tuple[str, int]]:
    """``"iris_2:2,iris_3:3"`` -> ``[("iris_2", 2), ("iris_3", 3)]`` (D9).

    One string rather than two parallel arrays, because an empty array launch
    param has no type in ROS 2 and fails to declare.
    """
    peers = []
    for item in (s.strip() for s in spec.split(',')):
        if not item:
            continue
        name, sep, sysid = item.partition(':')
        if not sep or not name.strip():
            raise ValueError(f"peer '{item}' must be robot:sysid")
        peers.append((name.strip(), int(sysid)))
    return peers


def parse_peaks(spec: str) -> List[Tuple[int, int, int]]:
    """``"3:4:90,6:1:80"`` -> ``[(col, row, percent), ...]`` (D19)."""
    peaks = []
    for item in (s.strip() for s in spec.split(',')):
        if not item:
            continue
        parts = item.split(':')
        if len(parts) != 3:
            raise ValueError(f"peak '{item}' must be col:row:percent")
        peaks.append(tuple(int(p) for p in parts))
    return peaks


def stub_map_data(width: int, height: int, background: int,
                  peaks: Iterable[Tuple[int, int, int]]) -> List[int]:
    """Flat OccupancyGrid data, ``data[row * width + col]``, peaks on a flat
    background. Values are percent, 0-100 (-1 is allowed as unknown)."""
    peaks = list(peaks)
    for v in (background, *(p for _, _, p in peaks)):
        if not -1 <= v <= 100:
            raise ValueError(f"cell value {v} outside -1..100")
    data = [background] * (width * height)
    for col, row, value in peaks:
        if not (0 <= col < width and 0 <= row < height):
            raise ValueError(f"peak ({col}, {row}) outside {width}x{height}")
        data[row * width + col] = value
    return data
