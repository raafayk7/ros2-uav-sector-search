"""One simulated mission: point-mass drones, the flying code for the rest.

What is real: the follower rule (follower_tick), the sweep (plan_sweep,
sweep_tick), the mission's end rule (Mission, D14), the belief map and its
confirmations (BeliefMap), the camera footprint, and the detector's misses,
false alarms and pixel noise (NoiseModel, D38), at the rates the nodes use
(frames 2 Hz, map and planner 1 Hz).

What is not: drones are points that move in straight lines at a constant speed
on their layer, with no climb, acceleration, tilt or tracking error, no lost
messages and no latency. Times are therefore good for comparing arms, not for
predicting a Gazebo run.

Found (scoring only; nothing in the loop reads it) = a true detection, by any
drone, that projects within 2 m of the target's true position.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from uav_mapping.core.belief import BeliefMap, BeliefParams, project_frame
from uav_mapping.core.geometry import Camera, Cell, Grid, Point, camera_pose, pixel_to_ground
from uav_perception.core.noise import NoiseModel, frame_detections
from uav_planning.core.follower import follower_tick
from uav_planning.core.mission import DONE, ENDED, Mission, MissionParams
from uav_planning.core.planner import GridSpec, Peer, values_from_occupancy
from uav_planning.core.sweep import (lane_half_width, lane_order_by_prior,
                                     plan_sweep, sweep_tick)

LEVEL = (0.0, 0.0, 0.0, 1.0)
LEADER = 'iris_1'


@dataclass(frozen=True)
class SimParams:
    followers: int = 3
    speed: float = 5.0              # m/s, every drone (D39's sweep speed)
    leader_layer: float = 8.0
    follower_layer: float = 10.0    # plus 2 m per further follower (D5)
    dt: float = 0.5                 # s: one detector frame
    timeout: float = 900.0          # s after the start (15 sim-minutes)
    found_radius: float = 2.0       # m, scoring
    negative_evidence: bool = True  # False: an empty look changes nothing
    prior_lanes: bool = False       # the leader flies the clue's lanes first (D50,
                                    # off since D53: it slowed both kinds of clue)
    noise: NoiseModel = NoiseModel()
    camera: Camera = Camera()


@dataclass
class Result:
    found: List[Optional[float]]            # per target, s; None = never
    found_by: List[Optional[str]]
    confirmed: List[Optional[float]]        # per target: its sector confirmed, s
    extra_confirms: int                     # confirmed sectors with no target in them
    end: float
    done: bool                              # D14's end rule, not the timeout
    coverage: float                         # share of cells any camera saw by the end
    sweep_done: Optional[float]
    path: List[int] = field(default_factory=list)   # lane order flown

    @property
    def first(self) -> Optional[float]:
        return min((t for t in self.found if t is not None), default=None)

    @property
    def all(self) -> Optional[float]:
        return None if None in self.found else max(self.found)


def _step(pos: Point, to: Point, dist: float) -> Point:
    d = math.dist(pos, to)
    if d <= dist:
        return to
    return pos[0] + (to[0] - pos[0]) * dist / d, pos[1] + (to[1] - pos[1]) * dist / d


def simulate(grid: Grid, targets: Sequence[Point],
             start: Optional[np.ndarray], seed: int,
             params: SimParams = SimParams()) -> Result:
    """Fly one mission. ``start`` is the starting belief (None: no clue).
    ``seed`` fixes the detector's noise."""
    spec = GridSpec(grid.width, grid.height, grid.resolution,
                    grid.origin_x, grid.origin_y)
    belief = BeliefMap(grid, BeliefParams() if params.negative_evidence
                       else BeliefParams(p_miss=0.5), start)
    rng = np.random.default_rng([seed, 1])
    cam = params.camera
    intrinsics, size = (cam.fx, cam.fy, cam.cx, cam.cy), (cam.width, cam.height)

    # swarm.launch.py: the leader in the first sector, followers 5 m south of
    # the arena and 5 m apart, sysids 2, 3, ...
    half = 0.5 * grid.resolution
    sysids = {f'iris_{i + 2}': i + 2 for i in range(params.followers)}
    pos: Dict[str, Point] = {LEADER: (grid.origin_x + half, grid.origin_y + half)}
    pos.update({f: (grid.origin_x + half + 5.0 * (n - 1), grid.origin_y - 5.0)
                for f, n in sysids.items()})
    layer = {LEADER: params.leader_layer}
    layer.update({f: params.follower_layer + 2.0 * (n - 2) for f, n in sysids.items()})

    width = lane_half_width(cam, params.leader_layer)
    order = None
    if start is not None and params.prior_lanes:
        order = lane_order_by_prior(spec, width, np.asarray(start) - 0.5, pos[LEADER])
    waypoints = plan_sweep(spec, width, pos[LEADER], lane_order=order)

    mission = Mission(list(sysids), MissionParams(timeout=params.timeout))
    goals = {f: None for f in sysids}
    status = {f: 'HOLDING' for f in sysids}
    commands: Dict[str, Optional[Point]] = {f: None for f in sysids}
    found: List[Optional[float]] = [None] * len(targets)
    found_by: List[Optional[str]] = [None] * len(targets)
    waypoint, sweep_status, sweep_done = 0, 'WAITING', None
    per_second = round(1.0 / params.dt)
    tick, t = 0, 0.0

    while mission.state not in ENDED:
        t = tick * params.dt
        sweep = sweep_tick(waypoints, waypoint, (*pos[LEADER], layer[LEADER]),
                           layer[LEADER])
        waypoint, sweep_status = sweep.index, sweep.status
        if sweep_status == 'COMPLETE' and sweep_done is None:
            sweep_done = t

        if tick % per_second == 0:
            occupancy = belief.occupancy_data()
            enabled = mission.tick(
                t, t, sweep_status, occupancy, grid.width,
                [(cell, who) for cell, (who, _) in belief.confirmed.items()],
                status).enable
            values = values_from_occupancy(occupancy, spec)
            for f, n in sysids.items():
                peers = [Peer(m, pos[g], t, goals[g],
                              t if goals[g] is not None else -math.inf)
                         for g, m in sysids.items() if g != f]
                out = follower_tick(spec, values, n, (*pos[f], layer[f]), layer[f],
                                    goals[f], peers, t, enabled=enabled)
                goals[f], status[f] = out.goal, out.status
                if out.command is not None:
                    commands[f] = out.command

        if sweep.command is not None:
            pos[LEADER] = _step(pos[LEADER], sweep.command, params.speed * params.dt)
        for f in sysids:
            if commands[f] is not None:
                pos[f] = _step(pos[f], commands[f], params.speed * params.dt)

        tick += 1
        t = tick * params.dt
        for who in pos:
            body = (*pos[who], layer[who])
            cam_pos, R = camera_pose(body, LEVEL)
            dets = frame_detections(targets, cam_pos, R, intrinsics, size,
                                    params.noise, rng)
            for d in dets:
                if not d.is_true:
                    continue
                ground = pixel_to_ground(d.u, d.v, cam_pos, R, cam)
                for i, target in enumerate(targets):
                    if found[i] is None and \
                            math.dist(ground, target) <= params.found_radius:
                        found[i], found_by[i] = t, who
            frame = project_frame(grid, cam, body, LEVEL,
                                  [(d.u, d.v, d.score) for d in dets])
            belief.update(who, t, body, frame, confirmer=who != LEADER)

    cells = [grid.cell_of(*p) for p in targets]
    return Result(
        found, found_by,
        [belief.confirmed[c][1] if c in belief.confirmed else None for c in cells],
        sum(c not in cells for c in belief.confirmed), t, mission.state == DONE,
        float(np.mean(~np.isnan(belief.logodds))), sweep_done,
        list(order) if order is not None else [])
