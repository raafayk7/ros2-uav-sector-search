"""Pinhole projection of ground points into the camera image (no ROS).

The inverse of what leader_node does: it turns a pixel into a ground point,
this turns a ground point into a pixel. Same conventions, so a detection made
here lands back on the same ground point there.

  arena frame   ENU, origin at home (what /pose is expressed in)
  body frame    FLU (x forward, y left, z up)
  camera frame  optical (x right, y down, z out of the lens)
  pixel         u = column (0..width), v = row (0..height), origin top-left
"""

import math

import numpy as np

# Rotation body_R_cam for a straight-down camera, as a quaternion
# (x, y, z, w): 180 deg about the axis (1, -1, 0) / sqrt(2). Equals the matrix
# leader_node calls _NADIR_BODY_R_CAM (a test checks they match).
NADIR_QUAT = (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)


def quat_to_rot(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix (child frame -> parent)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def camera_pose(body_pos, body_quat, mount_quat, mount_xyz=(0.0, 0.0, 0.0)):
    """Camera position and world_R_cam from the drone pose and the mount.

    body_pos (x, y, z) and body_quat (x, y, z, w) are the drone in the arena
    frame; mount_quat is body_R_cam; mount_xyz is the lens offset in the body.
    """
    world_R_body = quat_to_rot(*body_quat)
    cam_pos = np.asarray(body_pos, dtype=float) + \
        world_R_body @ np.asarray(mount_xyz, dtype=float)
    return cam_pos, world_R_body @ quat_to_rot(*mount_quat)


def project_ground_point(x, y, cam_pos, world_R_cam, fx, fy, cx, cy):
    """Pixel (u, v) and depth of the ground point (x, y, 0), or None if it is
    behind the camera. The pixel can lie outside the image: use in_frame()."""
    d = world_R_cam.T @ (np.array([x, y, 0.0]) - cam_pos)
    if d[2] <= 1e-6:
        return None
    return fx * d[0] / d[2] + cx, fy * d[1] / d[2] + cy, float(d[2])


def in_frame(u, v, width, height):
    return 0.0 <= u < width and 0.0 <= v < height
