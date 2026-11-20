"""The belief map: one posterior fused from every drone's camera (no ROS).

Rules, all decided with the user (docs/planner-decisions.md, D31, D32, D13,
D40):

* Each cell holds the log-odds that it contains at least one target (D31).
  Cells are independent; values need not sum to 1. NaN = never observed,
  published as -1 (unknown), which the planner counts as 0.5 (D18).

* Per frame, as leader_node: a cell with a detection gets +score x l_occ, a
  cell in the footprint with none gets l_free, clamped to +-l_max.

* R29, the look cap (D40). Frames a moment apart from nearly the same view
  are not independent evidence, so per drone and cell, one *look* changes the
  cell by at most +-look_cap. A look is a run of frames in which that drone
  sees the cell; it ends after look_gap s out of view, or after look_max s,
  so a hover counts as a new look every look_max s. Looks from different
  drones add. The cap is applied to increments, so two drones seeing a cell
  at once both count. Offline probe (session 9): the map no longer depends on
  the frame rate, and a false positive held in view needs about 4 looks
  instead of 2.5 s to look certain.

* D32 confirmation: a confirming drone (a follower, never the leader) within
  arrival_radius of a sector centre and at least confirm_min_alt up sees a
  detection in that sector on confirm_frames consecutive frames of its own
  detector. A frame without one, or leaving the radius, restarts the count.

* D13 suppression: a confirmed sector publishes as 0 from then on, whatever
  later evidence says. The posterior underneath keeps updating, for the eval.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

from .geometry import (NADIR_QUAT, Camera, Cell, Grid, camera_pose,
                       cells_in_quad, footprint, pixel_to_ground)


def logit(p: float) -> float:
    return math.log(p / (1.0 - p))


@dataclass(frozen=True)
class BeliefParams:
    p_hit: float = 0.7            # evidence of one full-score detection
    p_miss: float = 0.45          # evidence of one empty look at a cell
    p_clamp: float = 0.97         # belief never leaves [1 - p_clamp, p_clamp]
    min_score: float = 0.25       # weaker boxes are ignored
    look_cap: float = logit(0.7)  # max change per look (D40); <= 0 disables
    look_gap: float = 1.0         # s out of view that ends a look
    look_max: float = 4.0         # s after which a continuing view is a new look
    confirm_frames: int = 3       # D32's N (D40)
    arrival_radius: float = 2.0   # m, horizontal, as the planner (D27)
    confirm_min_alt: float = 5.0  # m: no confirmations on the ground or climbing

    @property
    def l_occ(self) -> float:
        return logit(self.p_hit)

    @property
    def l_free(self) -> float:
        return logit(self.p_miss)

    @property
    def l_max(self) -> float:
        return logit(self.p_clamp)


@dataclass
class Frame:
    """One detector frame of one drone, projected onto the grid."""

    seen: Set[Cell]                 # cells inside the footprint
    hits: Dict[Cell, float]         # cell -> summed score of its detections


def project_frame(grid: Grid, camera: Camera, body_pos, body_quat,
                  detections: Sequence[Tuple[float, float, float]],
                  min_score: float = 0.25, mount_quat=NADIR_QUAT,
                  mount_xyz=(0.0, 0.0, 0.0)) -> Frame:
    """The cells a frame covered and the cells its detections fall in.

    detections are (u, v, score) box centres. Boxes below min_score, or that
    land outside the grid, are dropped. With no valid footprint (on the
    ground, or the horizon in view) nothing counts as seen.
    """
    cam_pos, world_R_cam = camera_pose(body_pos, body_quat, mount_quat, mount_xyz)
    quad = footprint(cam_pos, world_R_cam, camera)
    seen = cells_in_quad(grid, quad) if quad else set()
    hits: Dict[Cell, float] = {}
    for u, v, score in detections:
        if score < min_score:
            continue
        g = pixel_to_ground(u, v, cam_pos, world_R_cam, camera)
        cell = g and grid.cell_of(*g)
        if cell:
            hits[cell] = hits.get(cell, 0.0) + score
    return Frame(seen, hits)


@dataclass
class _Look:
    start: float
    last: float
    acc: float = 0.0


class BeliefMap:
    """The posterior, the looks that feed it, and confirmed sectors."""

    def __init__(self, grid: Grid, params: BeliefParams = BeliefParams()):
        self.grid = grid
        self.params = params
        self.logodds = np.full((grid.height, grid.width), np.nan)
        self.confirmed: Dict[Cell, Tuple[str, float]] = {}   # cell -> (drone, t)
        self._looks: Dict[Tuple[str, Cell], _Look] = {}
        self._streak: Dict[str, Tuple[Cell, int]] = {}

    # ------------------------------------------------------------------
    def update(self, drone: str, t: float, body_pos, frame: Frame,
               confirmer: bool = False) -> Optional[Cell]:
        """Apply one frame. Returns the sector it confirmed, if any.

        t is in seconds on any one clock shared by all drones' frames.
        """
        p = self.params
        for cell in frame.seen | set(frame.hits):
            if cell in frame.hits:
                delta = frame.hits[cell] * p.l_occ
            else:
                delta = p.l_free
            self._evidence(drone, t, cell, delta)
        if confirmer:
            return self._confirm_step(drone, t, body_pos, frame)
        return None

    def _evidence(self, drone: str, t: float, cell: Cell, delta: float):
        p = self.params
        if p.look_cap > 0.0:
            key = (drone, cell)
            look = self._looks.get(key)
            if look is None or t - look.last > p.look_gap \
                    or t - look.start >= p.look_max:
                look = self._looks[key] = _Look(t, t)
            look.last = t
            acc = min(max(look.acc + delta, -p.look_cap), p.look_cap)
            delta, look.acc = acc - look.acc, acc
        col, row = cell
        cur = self.logodds[row, col]
        cur = 0.0 if math.isnan(cur) else cur   # first observation: p = 0.5
        self.logodds[row, col] = min(max(cur + delta, -p.l_max), p.l_max)

    def _confirm_step(self, drone: str, t: float, body_pos,
                      frame: Frame) -> Optional[Cell]:
        p = self.params
        x, y, z = body_pos
        cell = self.grid.cell_of(x, y)
        near = (cell is not None and z >= p.confirm_min_alt and
                math.dist((x, y), self.grid.centre(cell)) <= p.arrival_radius)
        if not near or cell in self.confirmed:
            self._streak.pop(drone, None)
            return None
        prev, n = self._streak.get(drone, (None, 0))
        if cell in frame.hits:
            n = n + 1 if prev == cell else 1
        else:
            n = 0
        if n >= p.confirm_frames:
            self._streak.pop(drone, None)
            self.confirmed[cell] = (drone, t)
            return cell
        self._streak[drone] = (cell, n)
        return None

    def finds(self) -> List[dict]:
        """Confirmed sectors as plain data, in the order they were found."""
        return [{'cell': list(cell), 'drone': drone, 't': t}
                for cell, (drone, t) in sorted(self.confirmed.items(),
                                               key=lambda kv: kv[1][1])]

    # ------------------------------------------------------------------
    def probability(self, cell: Cell) -> Optional[float]:
        """Posterior of one cell (None if never observed), suppression aside."""
        col, row = cell
        lo = self.logodds[row, col]
        return None if math.isnan(lo) else 1.0 / (1.0 + math.exp(-lo))

    def occupancy_data(self) -> List[int]:
        """Row-major OccupancyGrid data: 0-100, -1 unknown, 0 if confirmed."""
        out = []
        for row in range(self.grid.height):
            for col in range(self.grid.width):
                if (col, row) in self.confirmed:
                    out.append(0)
                    continue
                prob = self.probability((col, row))
                out.append(-1 if prob is None else int(round(prob * 100.0)))
        return out
