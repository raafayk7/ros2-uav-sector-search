"""belief_map_node: the single owner of the posterior (roadmap, D13, D40).

Every drone's detector publishes a Detection2DArray on every frame, including
empty ones (R30). This node pairs each frame with that drone's latest pose,
projects the camera footprint and the detections onto the grid, and feeds
uav_mapping.core.belief, where every rule lives (look cap, D32 confirmation,
D13 suppression). Followers only read the result. It replaces leader_node,
which stays as the verified single-drone reference.

  in   /<robot>/detections          Detection2DArray, one per drone in `robots`
  in   /<robot>/pose                PoseStamped, arena ENU (localization_node)
  in   /<robot>/camera/camera_info  optional: the sim-camera defaults are used
                                    until one arrives, so a follower without a
                                    bridged camera still has a footprint
  out  /heatmap                     OccupancyGrid, RELIABLE + TRANSIENT_LOCAL,
                                    confirmed sectors as 0
  out  /confirmed_sectors           PoseArray of confirmed sector centres,
                                    latched, republished when it grows

Time: wall clock (use_sim_time false), like the detectors that drive it, so
a look (D40) spans the same number of detector frames at any RTF, and the
node adds no /clock subscriber (docker-notes 34).

Topic parameters are templates with {robot}, absolute by default as in the
other nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray

from uav_mapping.core.belief import BeliefMap, BeliefParams, project_frame
from uav_mapping.core.geometry import NADIR_QUAT, Camera, Grid


def _names(text: str):
    return [s.strip() for s in text.split(',') if s.strip()]


def _best_score(det) -> float:
    return max((r.hypothesis.score for r in det.results), default=0.0)


class BeliefMapNode(Node):

    def __init__(self):
        super().__init__('belief_map_node')
        d = self.declare_parameter
        d('robots', 'iris_1')                 # every drone whose camera counts
        d('confirmers', '')                   # followers: D32 confirmation
        d('pose_topic', '/{robot}/pose')
        d('detections_topic', '/{robot}/detections')
        d('camera_info_topic', '/{robot}/camera/camera_info')
        d('heatmap_topic', '/heatmap')
        d('confirmed_topic', '/confirmed_sectors')
        d('arena_frame', 'arena')
        d('publish_period', 1.0)
        # Grid: must match the planners' and the sweep's (D39 defaults).
        d('resolution', 5.0)
        d('grid_width', 8)
        d('grid_height', 8)
        d('origin_x', 0.0)
        d('origin_y', 0.0)
        # Camera: mount, and intrinsics until a camera_info arrives.
        d('mount_quat', list(NADIR_QUAT))
        d('mount_xyz', [0.0, 0.0, 0.0])
        cam = Camera()
        d('fx', cam.fx)
        d('fy', cam.fy)
        d('cx', cam.cx)
        d('cy', cam.cy)
        d('image_width', cam.width)
        d('image_height', cam.height)
        # Fusion (D40 defaults live in BeliefParams).
        bp = BeliefParams()
        for name in ('p_hit', 'p_miss', 'p_clamp', 'look_cap', 'look_gap',
                     'look_max', 'arrival_radius', 'confirm_min_alt'):
            d(name, getattr(bp, name))
        d('min_detection_score', bp.min_score)
        d('confirm_frames', bp.confirm_frames)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.robots = _names(g('robots'))
        self.confirmers = set(_names(g('confirmers')))
        unknown = self.confirmers - set(self.robots)
        if unknown:
            raise ValueError(f"confirmers {sorted(unknown)} are not in robots")
        self.frame_id = g('arena_frame')
        self.grid = Grid(int(g('grid_width')), int(g('grid_height')),
                         float(g('resolution')), float(g('origin_x')),
                         float(g('origin_y')))
        self.mount_quat = tuple(g('mount_quat'))
        self.mount_xyz = tuple(g('mount_xyz'))
        default_cam = Camera(float(g('fx')), float(g('fy')), float(g('cx')),
                             float(g('cy')), int(g('image_width')),
                             int(g('image_height')))
        self.params = BeliefParams(
            p_hit=float(g('p_hit')), p_miss=float(g('p_miss')),
            p_clamp=float(g('p_clamp')),
            min_score=float(g('min_detection_score')),
            look_cap=float(g('look_cap')), look_gap=float(g('look_gap')),
            look_max=float(g('look_max')),
            confirm_frames=int(g('confirm_frames')),
            arrival_radius=float(g('arrival_radius')),
            confirm_min_alt=float(g('confirm_min_alt')))
        self.map = BeliefMap(self.grid, self.params)

        self.pose = {r: None for r in self.robots}
        self.camera = {r: default_cam for r in self.robots}
        self.frames = {r: 0 for r in self.robots}

        reliable = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        for r in self.robots:
            topic = lambda name: g(name).format(robot=r)  # noqa: E731
            self.create_subscription(
                PoseStamped, topic('pose_topic'),
                lambda m, r=r: self.pose.__setitem__(r, m), reliable)
            self.create_subscription(
                Detection2DArray, topic('detections_topic'),
                lambda m, r=r: self._on_frame(r, m), reliable)
            self.create_subscription(
                CameraInfo, topic('camera_info_topic'),
                lambda m, r=r: self._on_camera_info(r, m),
                qos_profile_sensor_data)
        self.heatmap_pub = self.create_publisher(
            OccupancyGrid, g('heatmap_topic'), latched)
        self.confirmed_pub = self.create_publisher(
            PoseArray, g('confirmed_topic'), latched)
        self._publish_confirmed()

        self.create_timer(float(g('publish_period')), self._publish_heatmap)
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f"belief_map_node up | robots={self.robots} "
            f"confirmers={sorted(self.confirmers)} | grid {self.grid.width}x"
            f"{self.grid.height} @ {self.grid.resolution} m | look cap "
            f"{self.params.look_cap:.2f} per {self.params.look_max:g} s, "
            f"N={self.params.confirm_frames}")

    # ------------------------------------------------------------------
    def _on_camera_info(self, robot, msg: CameraInfo):
        k = msg.k
        self.camera[robot] = Camera(k[0], k[4], k[2], k[5],
                                    int(msg.width), int(msg.height))

    def _on_frame(self, robot, msg: Detection2DArray):
        pose = self.pose[robot]
        if pose is None:
            self.get_logger().warn(f"{robot}: frame before pose; skipped",
                                   throttle_duration_sec=10.0)
            return
        p, q = pose.pose.position, pose.pose.orientation
        pos = (p.x, p.y, p.z)
        dets = [(det.bbox.center.position.x, det.bbox.center.position.y,
                 _best_score(det)) for det in msg.detections]
        frame = project_frame(self.grid, self.camera[robot], pos,
                              (q.x, q.y, q.z, q.w), dets,
                              self.params.min_score, self.mount_quat,
                              self.mount_xyz)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.frames[robot] += 1
        cell = self.map.update(robot, now, pos, frame,
                               confirmer=robot in self.confirmers)
        if cell is not None:
            prob = self.map.probability(cell)
            self.get_logger().info(
                f"CONFIRMED sector {cell} by {robot} at ({p.x:.1f}, {p.y:.1f}, "
                f"{p.z:.1f}); posterior {prob:.2f}; now published as 0")
            self._publish_confirmed()
            self._publish_heatmap()

    # ------------------------------------------------------------------
    def _publish_heatmap(self):
        if all(p is None for p in self.pose.values()):
            return      # nothing observed yet: no all-unknown spam map
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.grid.resolution
        grid.info.width = self.grid.width
        grid.info.height = self.grid.height
        grid.info.origin.position.x = self.grid.origin_x
        grid.info.origin.position.y = self.grid.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = self.map.occupancy_data()
        self.heatmap_pub.publish(grid)

    def _publish_confirmed(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for cell in self.map.confirmed:
            pose = Pose()
            pose.position.x, pose.position.y = self.grid.centre(cell)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.confirmed_pub.publish(msg)

    def _report(self):
        known = sum(v >= 0 for v in self.map.occupancy_data())
        self.get_logger().info(
            f"frames {self.frames} | known cells {known}/"
            f"{self.grid.width * self.grid.height} | confirmed "
            f"{list(self.map.confirmed)}")


def main(args=None):
    rclpy.init(args=args)
    node = BeliefMapNode()
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
