"""Camera-to-ground geometry for the belief map (no ROS).

The same maths as leader_node, lifted out so it can be tested and shared:
pixel -> ray (pinhole K) -> arena frame (drone pose o static mount) -> flat
ground z = 0 -> grid cell.

  arena frame   ENU, origin at home (what /pose is expressed in)
  body frame    FLU (x forward, y left, z up)
  camera frame  optical (x right, y down, z out of the lens)
  pixel         u = column, v = row, origin top-left

The camera pose uses the full body quaternion, as leader_node and the
ground-truth detector do, so a detection made there projects back to the
same ground point here. The real gimbal holds nadir during tilt (session 8),
so in hard accelerations the footprint is off by about altitude x tilt
(about 1 m at 8 m and 8 deg): small against 5 m cells.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

Cell = Tuple[int, int]          # (col, row)
Point = Tuple[float, float]

# body_R_cam for a straight-down camera (image up = forward, image right =
# starboard). Equals leader_node's _NADIR_BODY_R_CAM.
NADIR_QUAT = (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)


@dataclass(frozen=True)
class Grid:
    """The sector grid, as an OccupancyGrid's info (D39 defaults)."""

    width: int = 8
    height: int = 8
    resolution: float = 5.0
    origin_x: float = 0.0
    origin_y: float = 0.0

    def cell_of(self, x: float, y: float) -> Optional[Cell]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def centre(self, cell: Cell) -> Point:
        col, row = cell
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)


@dataclass(frozen=True)
class Camera:
    """Pinhole intrinsics and image size. Defaults are the sim camera
    (docker-notes gotcha 47), used until a camera_info arrives."""

    fx: float = 205.47
    fy: float = 205.47
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480


def quat_to_rot(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix (child -> parent)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_pose(body_pos, body_quat, mount_quat=NADIR_QUAT,
                mount_xyz=(0.0, 0.0, 0.0)):
    """(cam_pos, world_R_cam) from the drone pose and the static mount."""
    world_R_body = quat_to_rot(*body_quat)
    cam_pos = np.asarray(body_pos, dtype=float) + \
        world_R_body @ np.asarray(mount_xyz, dtype=float)
    return cam_pos, world_R_body @ quat_to_rot(*mount_quat)


def pixel_to_ground(u, v, cam_pos, world_R_cam, camera: Camera) -> Optional[Point]:
    """Ground point (x, y) seen at pixel (u, v), or None if the ray misses
    the ground (parallel, or ground behind the camera)."""
    d = world_R_cam @ np.array([(u - camera.cx) / camera.fx,
                                (v - camera.cy) / camera.fy, 1.0])
    if abs(d[2]) < 1e-9:
        return None
    t = -cam_pos[2] / d[2]
    if t <= 0.0:
        return None
    hit = cam_pos + t * d
    return float(hit[0]), float(hit[1])


def footprint(cam_pos, world_R_cam, camera: Camera) -> Optional[List[Point]]:
    """The image's four corners on the ground, or None if any corner misses
    it (camera at or below ground, or looking at the horizon)."""
    quad = [pixel_to_ground(u, v, cam_pos, world_R_cam, camera)
            for u, v in ((0, 0), (camera.width, 0),
                         (camera.width, camera.height), (0, camera.height))]
    if any(p is None for p in quad):
        return None
    return quad


def point_in_quad(px: float, py: float, quad: Sequence[Point]) -> bool:
    """Even-odd ray-cast point-in-polygon."""
    inside = False
    j = len(quad) - 1
    for i in range(len(quad)):
        xi, yi = quad[i]
        xj, yj = quad[j]
        if ((yi > py) != (yj > py)) and \
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def cells_in_quad(grid: Grid, quad: Sequence[Point]) -> Set[Cell]:
    """Cells whose centre lies inside the ground quad (leader_node's rule).
    Only cells in the quad's bounding box are tested."""
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    r = grid.resolution
    col_lo = max(0, int(math.floor((min(xs) - grid.origin_x) / r)))
    col_hi = min(grid.width - 1, int(math.floor((max(xs) - grid.origin_x) / r)))
    row_lo = max(0, int(math.floor((min(ys) - grid.origin_y) / r)))
    row_hi = min(grid.height - 1, int(math.floor((max(ys) - grid.origin_y) / r)))
    out = set()
    for row in range(row_lo, row_hi + 1):
        for col in range(col_lo, col_hi + 1):
            if point_in_quad(*grid.centre((col, row)), quad):
                out.add((col, row))
    return out
