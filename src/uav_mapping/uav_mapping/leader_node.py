"""leader_node -- detection-to-heatmap for the leader drone.

This is the first algorithm-relevant node. The leader sweeps the arena,
perception_node emits camera detections, and this node turns each detection
into a world (x, y) on the ground and accumulates a probability heatmap that
the follower planners (DGA/BLL) will consume.

Pipeline per detection:

    pixel (u, v) in the image
      -> back-project through the pinhole model (camera_info K)  ... a ray
      -> rotate the ray into the arena frame (drone pose o camera mount)
      -> intersect with the ground plane z = 0                   ... a point
      -> find the sector (OccupancyGrid cell) it falls in
      -> Bayesian log-odds update of that cell

----------------------------------------------------------------------------
DESIGN DECISIONS (settled with Raafay this session)
----------------------------------------------------------------------------

* Map type: nav_msgs/OccupancyGrid. Its int8 cells encode 0-100 (percent
  probability) or -1 (unknown), which lines up exactly with the thesis's
  per-sector p_ij in [0,1]. One cell == one thesis sector s_ij.

* Geometry (thesis-matched, all launch params):
    - resolution = 5.0 m/cell      (thesis 5 m markers ~ 1 marker per sector)
    - grid_width = grid_height = 4  (4 x 4 over the 20 x 20 m thesis arena)
    - origin = (0, 0) by default    (CORNER placement: home at cell [0,0]'s
      corner, grid extends +x/+y, matching the thesis leader that takes off
      from a corner and sweeps). Centered-on-home is origin:=(-10, -10).
  Changing resolution / grid_width / grid_height reshapes the grid with no
  code change.

* Frames: everything is in the arena frame (ENU, origin at home) -- the same
  frame localization_node stamps on /pose. We never touch the simulator's tf
  tree, so this stays valid when AP_DDS / real hardware replace the sim.

* Camera pose = /pose (drone body in arena) composed with a STATIC camera
  mount transform (body -> camera optical frame). The mount is a parameter
  (quaternion + translation), defaulting to a nadir / straight-down camera.
  We do the geometry explicitly here rather than via tf2 so every step is
  visible and the node has no dependency on sim-specific frame names.

* Intrinsics from /camera/camera_info (we read K live, never hardcode).

* Ground model: flat earth, z = 0 in the arena frame. The leader flies at a
  fixed altitude over flat terrain, so a single ground plane is exact enough.

* Probability: Bayesian log-odds inverse-sensor-model.
    - Each cell stores a log-odds value l; p = 1 / (1 + exp(-l)).
    - A detection landing in a cell adds +l_occ (scaled by the detection's
      confidence) -- a "hit".
    - Cells inside the camera's ground footprint that received NO detection
      add l_free -- a "miss" -- so a swept-but-empty sector decays toward
      empty. This is the part that makes it a real search map, not just a
      detection tally.
    - l is clamped to [l_min, l_max] so one cell can't saturate to absolute
      certainty and can always be revised by later evidence.
    - Cells never observed stay UNKNOWN (-1).

Topic-naming strategy mirrors localization_node: inputs are
absolute-by-default parameters (/pose, /detections, /camera/camera_info) so
namespacing the node doesn't rewrite them; the output (/heatmap) is a
project-level topic, intentionally unnamespaced (one shared map).
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray


def quat_to_rot(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix (numpy).

    Standard formula; assumes (x, y, z, w) order (ROS convention). Returns the
    rotation that takes a vector expressed in the rotated frame into the parent
    frame (i.e. world_R_body for a body-orientation quaternion).
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# Default mount: a nadir (straight-down) camera, expressed as the rotation
# body_R_cam that maps the camera OPTICAL frame (REP-103: x=right, y=down,
# z=forward/out-of-lens) into the body frame (FLU: x=forward, y=left, z=up).
#   optical z (view dir) -> body (0, 0, -1)  = straight down
#   optical x (img right) -> body (0, -1, 0) = starboard
#   optical y (img down)  -> body (-1, 0, 0) = aft (so image-up = forward)
# Columns of the matrix are those three body-frame vectors. det = +1.
_NADIR_BODY_R_CAM = np.array([
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])


def _rot_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w). Used only to expose the default
    nadir mount as a quaternion parameter default."""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


_NADIR_QUAT = _rot_to_quat(_NADIR_BODY_R_CAM)  # (x, y, z, w)


def prob_to_logodds(p):
    return math.log(p / (1.0 - p))


class LeaderNode(Node):
    """Accumulates camera detections into a probability OccupancyGrid."""

    def __init__(self):
        super().__init__('leader_node')

        # ---- Input / output topics (absolute defaults; see module docstring).
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('heatmap_topic', '/heatmap')
        self.declare_parameter('arena_frame', 'arena')

        # ---- Grid geometry (thesis-matched defaults; all reshape-safe).
        self.declare_parameter('resolution', 5.0)      # m per cell
        self.declare_parameter('grid_width', 4)        # cells (x / East)
        self.declare_parameter('grid_height', 4)       # cells (y / North)
        # OccupancyGrid origin = world (x, y) of cell [0,0]'s lower-left corner.
        # Default (0, 0) = CORNER placement (home at the corner, thesis-style).
        # Centered-on-home: origin_x:=-10.0 origin_y:=-10.0 for a 20x20 m grid.
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)

        # ---- Camera mount (body -> camera optical). Default = nadir.
        self.declare_parameter('mount_quat', _NADIR_QUAT)            # x,y,z,w
        self.declare_parameter('mount_xyz', [0.0, 0.0, 0.0])         # in body

        # ---- Inverse sensor model (log-odds).
        self.declare_parameter('p_hit', 0.7)     # evidence from a detection
        self.declare_parameter('p_miss', 0.45)   # evidence from an empty look
        self.declare_parameter('p_clamp', 0.97)  # max/min belief (symmetric)
        # Cells whose footprint coverage we trust enough to apply miss updates.
        self.declare_parameter('apply_free_updates', True)
        # Min detection score to count as a hit (reject junk boxes).
        self.declare_parameter('min_detection_score', 0.25)
        # Publish cadence (s). We accumulate continuously, publish on a timer.
        self.declare_parameter('publish_period', 1.0)

        g = self.get_parameter
        self.pose_topic = g('pose_topic').value
        self.det_topic = g('detections_topic').value
        self.cinfo_topic = g('camera_info_topic').value
        self.heatmap_topic = g('heatmap_topic').value
        self.arena_frame = g('arena_frame').value

        self.res = float(g('resolution').value)
        self.gw = int(g('grid_width').value)
        self.gh = int(g('grid_height').value)
        self.ox = float(g('origin_x').value)
        self.oy = float(g('origin_y').value)

        mq = list(g('mount_quat').value)
        self.body_R_cam = quat_to_rot(mq[0], mq[1], mq[2], mq[3])
        self.mount_xyz = np.array(list(g('mount_xyz').value), dtype=float)

        p_hit = float(g('p_hit').value)
        p_miss = float(g('p_miss').value)
        p_clamp = float(g('p_clamp').value)
        self.l_occ = prob_to_logodds(p_hit)
        self.l_free = prob_to_logodds(p_miss)
        self.l_max = prob_to_logodds(p_clamp)
        self.l_min = -self.l_max
        self.apply_free = bool(g('apply_free_updates').value)
        self.min_score = float(g('min_detection_score').value)

        # ---- State.
        # Log-odds grid, row-major [row (y), col (x)]. NaN = never observed.
        self.logodds = np.full((self.gh, self.gw), np.nan, dtype=float)
        self.K = None              # 3x3 intrinsics, set from camera_info
        self.img_w = None
        self.img_h = None
        self.pose = None           # latest PoseStamped (drone body in arena)

        # ---- QoS.
        # Inputs: pose is RELIABLE/VOLATILE (matches localization_node's
        # publisher); detections RELIABLE depth 10 (skill convention);
        # camera_info BEST_EFFORT sensor data. A BEST_EFFORT subscriber is
        # compatible with any publisher, so we use sensor QoS for camera_info.
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        det_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # OUTPUT: RELIABLE + TRANSIENT_LOCAL. TRANSIENT_LOCAL means the
        # publisher latches the last message, so a follower planner that
        # subscribes AFTER the leader has been mapping still immediately
        # receives the current heatmap instead of waiting for the next publish.
        heatmap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            PoseStamped, self.pose_topic, self._on_pose, pose_qos)
        self.create_subscription(
            Detection2DArray, self.det_topic, self._on_detections, det_qos)
        self.create_subscription(
            CameraInfo, self.cinfo_topic, self._on_camera_info,
            qos_profile_sensor_data)
        self._heatmap_pub = self.create_publisher(
            OccupancyGrid, self.heatmap_topic, heatmap_qos)

        self.create_timer(
            float(g('publish_period').value), self._publish_heatmap)

        self.get_logger().info(
            f"leader_node up | in: pose='{self.pose_topic}', "
            f"det='{self.det_topic}', cinfo='{self.cinfo_topic}' | "
            f"out: heatmap='{self.heatmap_topic}'")
        self.get_logger().info(
            f"grid: {self.gw}x{self.gh} @ {self.res} m/cell, "
            f"origin=({self.ox}, {self.oy}) frame='{self.arena_frame}'")
        self.get_logger().info("Waiting for camera_info + pose...")

    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo):
        # K is a 9-element row-major 3x3 (msg.k in Jazzy). Latch once.
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.img_w = msg.width
        self.img_h = msg.height

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg

    # ------------------------------------------------------------------
    def _camera_pose_in_arena(self):
        """Return (cam_pos[3], world_R_cam[3x3]) from /pose o static mount.

        world_R_body comes from the pose quaternion; world_R_cam =
        world_R_body @ body_R_cam. Camera position = body position offset by
        the mount translation rotated into the arena frame.
        """
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        world_R_body = quat_to_rot(q.x, q.y, q.z, q.w)
        cam_pos = np.array([p.x, p.y, p.z]) + world_R_body @ self.mount_xyz
        world_R_cam = world_R_body @ self.body_R_cam
        return cam_pos, world_R_cam

    def _pixel_to_ground(self, u, v, cam_pos, world_R_cam):
        """Back-project pixel (u, v) and intersect the z=0 ground plane.

        Returns world (x, y) or None if the ray doesn't hit the ground in
        front of the camera (points up, or camera at/below ground).
        """
        # Ray direction in the camera optical frame: K^-1 [u, v, 1].
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        # Into the arena frame.
        d_world = world_R_cam @ d_cam
        # Intersect z = 0:  cam_pos.z + t * d_world.z = 0.
        if abs(d_world[2]) < 1e-9:
            return None                      # ray parallel to ground
        t = -cam_pos[2] / d_world[2]
        if t <= 0.0:
            return None                      # ground is behind the camera
        hit = cam_pos + t * d_world
        return hit[0], hit[1]

    def _world_to_cell(self, x, y):
        """World (x, y) -> (col, row) or None if outside the grid."""
        col = int(math.floor((x - self.ox) / self.res))
        row = int(math.floor((y - self.oy) / self.res))
        if 0 <= col < self.gw and 0 <= row < self.gh:
            return col, row
        return None

    def _bump(self, row, col, delta):
        """Apply a clamped log-odds update to one cell (NaN-aware)."""
        cur = self.logodds[row, col]
        if math.isnan(cur):
            cur = 0.0                        # first observation: prior p=0.5
        self.logodds[row, col] = float(
            np.clip(cur + delta, self.l_min, self.l_max))

    # ------------------------------------------------------------------
    def _on_detections(self, msg: Detection2DArray):
        """Project every detection to the ground and update the heatmap."""
        if self.K is None or self.pose is None:
            self.get_logger().warn(
                "detection before camera_info/pose ready; skipping",
                throttle_duration_sec=5.0)
            return

        cam_pos, world_R_cam = self._camera_pose_in_arena()

        # 1) Compute the ground footprint (project the 4 image corners) so we
        #    know which cells were actually LOOKED AT this frame -- needed for
        #    the miss / free-space update.
        observed_cells = set()
        if self.apply_free and self.img_w and self.img_h:
            corners = [(0, 0), (self.img_w, 0),
                       (self.img_w, self.img_h), (0, self.img_h)]
            ground = []
            for (u, v) in corners:
                g = self._pixel_to_ground(u, v, cam_pos, world_R_cam)
                if g is not None:
                    ground.append(g)
            if len(ground) == 4:
                observed_cells = self._cells_in_quad(ground)

        # 2) Hits: each detection's bbox center -> ground -> cell.
        hit_cells = set()
        for det in msg.detections:
            score = self._best_score(det)
            if score < self.min_score:
                continue
            bb = det.bbox
            u = bb.center.position.x        # vision_msgs: center is a Pose2D
            v = bb.center.position.y
            g = self._pixel_to_ground(u, v, cam_pos, world_R_cam)
            if g is None:
                continue
            cell = self._world_to_cell(g[0], g[1])
            if cell is None:
                continue
            col, row = cell
            # Confidence-scaled hit: a low-confidence box nudges less.
            self._bump(row, col, score * self.l_occ)
            hit_cells.add((col, row))

        # 3) Misses: observed-but-no-detection cells decay toward empty.
        if self.apply_free:
            for (col, row) in observed_cells:
                if (col, row) not in hit_cells:
                    self._bump(row, col, self.l_free)

    def _best_score(self, det):
        """Largest hypothesis score on a Detection2D (0 if none)."""
        best = 0.0
        for res in det.results:
            s = res.hypothesis.score
            if s > best:
                best = s
        return best

    def _cells_in_quad(self, quad):
        """Set of (col, row) cells whose center lies inside the ground quad.

        quad is 4 world (x, y) points (image corners projected to z=0). We
        only test cells in the quad's bounding box, so this stays cheap.
        """
        xs = [p[0] for p in quad]
        ys = [p[1] for p in quad]
        col_lo = max(0, int(math.floor((min(xs) - self.ox) / self.res)))
        col_hi = min(self.gw - 1, int(math.floor((max(xs) - self.ox) / self.res)))
        row_lo = max(0, int(math.floor((min(ys) - self.oy) / self.res)))
        row_hi = min(self.gh - 1, int(math.floor((max(ys) - self.oy) / self.res)))
        out = set()
        for row in range(row_lo, row_hi + 1):
            for col in range(col_lo, col_hi + 1):
                cx = self.ox + (col + 0.5) * self.res
                cy = self.oy + (row + 0.5) * self.res
                if _point_in_quad(cx, cy, quad):
                    out.add((col, row))
        return out

    # ------------------------------------------------------------------
    def _publish_heatmap(self):
        """Render the log-odds grid to an OccupancyGrid and publish."""
        if self.pose is None:
            return  # nothing meaningful yet; avoids an all-unknown spam map

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.arena_frame
        grid.info.resolution = self.res
        grid.info.width = self.gw
        grid.info.height = self.gh
        grid.info.origin.position.x = self.ox
        grid.info.origin.position.y = self.oy
        grid.info.origin.orientation.w = 1.0

        # log-odds -> p -> 0..100, NaN -> -1 (unknown). OccupancyGrid data is
        # row-major starting at the origin corner, which matches our [row,col].
        data = np.empty(self.gh * self.gw, dtype=np.int8)
        flat = self.logodds.reshape(-1)
        for i, l in enumerate(flat):
            if math.isnan(l):
                data[i] = -1
            else:
                p = 1.0 / (1.0 + math.exp(-l))
                data[i] = int(round(p * 100.0))
        grid.data = data.tolist()
        self._heatmap_pub.publish(grid)


def _point_in_quad(px, py, quad):
    """Even-odd ray-cast point-in-polygon for a 4-vertex polygon."""
    inside = False
    n = len(quad)
    j = n - 1
    for i in range(n):
        xi, yi = quad[i]
        xj, yj = quad[j]
        if ((yi > py) != (yj > py)) and \
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
