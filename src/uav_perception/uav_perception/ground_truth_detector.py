"""ground_truth_detector -- a stand-in for YOLO that cheats, on purpose.

Where YOLO looks at pixels and guesses, this node asks Gazebo where every
target really is, works out where each one would appear in the drone's
camera, and publishes that as a vision_msgs/Detection2DArray: the same
message YOLO would publish, on the same topic (/detections). leader_node
cannot tell the difference. To keep it honest, it then makes the errors a
real detector makes (core/noise.py): it misses some targets, shifts the
pixel a little, and sometimes boxes something that is not there.

It publishes on EVERY frame, including frames with nothing in them, as an
empty array. That matters: leader_node lowers a sector's value only when a
detections message arrives, so a detector that stays silent when it sees
nothing leaves swept-but-empty sectors high forever, with no error
(TODOS.md "Empty frames must reach the map").

Where the inputs come from:
  targets   Gazebo models whose name starts with target_prefix, read with
            `gz service .../scene/info` every scene_period seconds (the
            container has no gz Python bindings), or targets_xy for a fixed
            list (target_source:=param, no simulator needed).
  pose      /pose, the drone in the arena frame (localization_node).
  camera    /camera/camera_info once it arrives, else the fx/cx/... params
            (the sim camera's: 640x480, 2.0 rad HFOV, fx 205.47). The mount
            (mount_quat) must match leader_node's.

A "frame" here is a timer tick at frame_rate (wall time, default 2 Hz; the
sim camera runs at 10 Hz), not an image: no
camera has to render, which is what lets this run without a GPU. The pose
used is the latest one, not the pose at the frame's timestamp.

Gazebo's world frame and the arena frame coincide when localization_node
uses a shared datum at the world origin (swarm.launch.py); if not, set
world_origin_x/y to the arena origin's position in the Gazebo world.
"""

import subprocess
import threading

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (Detection2DArray, Detection2D,
                             ObjectHypothesisWithPose)

from uav_perception.core.noise import NoiseModel, frame_detections
from uav_perception.core.projection import NADIR_QUAT, camera_pose
from uav_perception.core.scene import parse_scene_models, select_targets


class GroundTruthDetector(Node):

    def __init__(self):
        super().__init__('ground_truth_detector')

        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('frame_rate', 2.0)            # Hz, wall time
        self.declare_parameter('class_id', 'marker')
        self.declare_parameter('marker_size', 1.0)           # m, box size only

        # Camera: same mount as leader_node, intrinsics until camera_info.
        self.declare_parameter('mount_quat', list(NADIR_QUAT))   # x,y,z,w
        self.declare_parameter('mount_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fx', 205.47)
        self.declare_parameter('fy', 205.47)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        # Targets.
        self.declare_parameter('target_source', 'gz')        # 'gz' or 'param'
        self.declare_parameter('world_name', 'runway')
        self.declare_parameter('target_prefix', 'target_')
        self.declare_parameter('scene_period', 5.0)          # s between queries
        # x0,y0,x1,y1,... (an odd or empty list means no targets).
        self.declare_parameter('targets_xy', [0.0])
        self.declare_parameter('world_origin_x', 0.0)
        self.declare_parameter('world_origin_y', 0.0)

        # Error model: agreed defaults (docs/planner-decisions.md, session 7).
        self.declare_parameter('p_miss', 0.20)
        self.declare_parameter('p_false', 0.05)
        self.declare_parameter('pixel_sigma', 5.0)
        self.declare_parameter('hit_score', [0.60, 0.95])
        self.declare_parameter('false_score', [0.30, 0.70])
        self.declare_parameter('seed', -1)                   # -1: random

        g = self.get_parameter
        self.frame_id = g('frame_id').value
        self.class_id = g('class_id').value
        self.marker_size = float(g('marker_size').value)
        self.mount_quat = tuple(g('mount_quat').value)
        self.mount_xyz = tuple(g('mount_xyz').value)
        self.size = (int(g('image_width').value), int(g('image_height').value))
        self.K = (float(g('fx').value), float(g('fy').value),
                  float(g('cx').value), float(g('cy').value))
        self.origin = (float(g('world_origin_x').value),
                       float(g('world_origin_y').value))
        self.noise = NoiseModel(
            p_miss=float(g('p_miss').value),
            p_false=float(g('p_false').value),
            pixel_sigma=float(g('pixel_sigma').value),
            hit_score=tuple(g('hit_score').value),
            false_score=tuple(g('false_score').value))
        seed = int(g('seed').value)
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.pose = None
        self._targets = []                  # [(name, x, y)] in the arena frame
        self.counts = {'frames': 0, 'true': 0, 'false': 0}

        reliable = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            PoseStamped, g('pose_topic').value, self._on_pose, reliable)
        self.create_subscription(
            CameraInfo, g('camera_info_topic').value, self._on_camera_info,
            qos_profile_sensor_data)
        self._pub = self.create_publisher(
            Detection2DArray, g('detections_topic').value, reliable)

        source = g('target_source').value
        if source == 'gz':
            self._scene_period = float(g('scene_period').value)
            self._scene_cmd = [
                'gz', 'service', '-s',
                f"/world/{g('world_name').value}/scene/info",
                '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
                '--timeout', '3000', '--req', '']
            self._prefix = g('target_prefix').value
            self._stop = threading.Event()
            threading.Thread(target=self._scene_loop, daemon=True).start()
        elif source == 'param':
            xy = list(g('targets_xy').value)
            self._targets = [(f'target_{i}', xy[2 * i], xy[2 * i + 1])
                             for i in range(len(xy) // 2)]
        else:
            raise ValueError(f"target_source must be 'gz' or 'param', not {source!r}")

        self.create_timer(1.0 / float(g('frame_rate').value), self._on_frame)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"ground_truth_detector up | targets={source} | miss={self.noise.p_miss} "
            f"false={self.noise.p_false}/frame sigma={self.noise.pixel_sigma}px "
            f"seed={'random' if seed < 0 else seed}")

    # ------------------------------------------------------------------
    def _on_pose(self, msg):
        self.pose = msg

    def _on_camera_info(self, msg):
        # K is row-major 3x3: fx, 0, cx, 0, fy, cy, 0, 0, 1.
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.size = (int(msg.width), int(msg.height))

    # ------------------------------------------------------------------
    def _scene_loop(self):
        """Query Gazebo for target positions until shutdown (own thread, so
        the one-shot `gz` process never blocks a frame)."""
        while not self._stop.is_set():
            try:
                out = subprocess.run(self._scene_cmd, capture_output=True,
                                     text=True, timeout=8.0)
                if out.returncode == 0 and out.stdout:
                    found = select_targets(parse_scene_models(out.stdout),
                                           self._prefix)
                    targets = [(n, x - self.origin[0], y - self.origin[1])
                               for n, x, y in found]
                    if [t[0] for t in targets] != [t[0] for t in self._targets]:
                        self.get_logger().info(
                            f"targets: {[t[0] for t in targets]}")
                    self._targets = targets
                else:
                    self.get_logger().warn(
                        "gz scene/info gave no data (is the world up?)",
                        throttle_duration_sec=30.0)
            except (subprocess.TimeoutExpired, OSError) as e:
                self.get_logger().warn(f"gz scene/info failed: {e}",
                                       throttle_duration_sec=30.0)
            self._stop.wait(self._scene_period)

    # ------------------------------------------------------------------
    def _on_frame(self):
        if self.pose is None:
            self.get_logger().warn("no /pose yet; not publishing",
                                   throttle_duration_sec=10.0)
            return
        p, q = self.pose.pose.position, self.pose.pose.orientation
        cam_pos, world_R_cam = camera_pose(
            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
            self.mount_quat, self.mount_xyz)

        targets = self._targets
        dets = frame_detections(
            [(x, y) for _, x, y in targets], cam_pos, world_R_cam, self.K,
            self.size, self.noise, self.rng, self.marker_size)

        now = self.get_clock().now().to_msg()
        arr = Detection2DArray()
        arr.header.stamp = now
        arr.header.frame_id = self.frame_id
        for d in dets:
            det = Detection2D()
            det.header = arr.header
            det.bbox.center.position.x = float(d.u)
            det.bbox.center.position.y = float(d.v)
            det.bbox.size_x = det.bbox.size_y = float(d.size_px)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.class_id
            hyp.hypothesis.score = float(d.score)
            det.results.append(hyp)
            arr.detections.append(det)
        self._pub.publish(arr)        # an empty array is still published

        self.counts['frames'] += 1
        self.counts['true'] += sum(d.is_true for d in dets)
        self.counts['false'] += sum(not d.is_true for d in dets)

    def _report(self):
        c = self.counts
        self.get_logger().info(
            f"frames={c['frames']} true_detections={c['true']} "
            f"false_alarms={c['false']} targets_known={len(self._targets)}")

    def destroy_node(self):
        if hasattr(self, '_stop'):
            self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from launch / kill: rclpy has shut down already.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
