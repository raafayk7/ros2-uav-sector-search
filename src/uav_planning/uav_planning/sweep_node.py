"""sweep_node: the leader's boustrophedon sweep, a thin ROS wrapper.

Every rule lives in core/sweep.py (lanes, order, when to advance). This node
plans once, when it has both the camera's intrinsics and a pose, then on a
timer calls ``sweep_tick`` and turns the answer into messages. Design: D39 in
docs/planner-decisions.md (arena 8x8 at 5 m, 5 m/s, short-side spacing).

The node never subscribes to detections. The roadmap's coverage guarantee
says the leader always completes the full sweep, so nothing a detector says
may stop it. Arming and the speed limit are arm_takeoff's job (D15): /ap
services hang under this container's CycloneDDS.

Topics (input topics absolute, outputs relative; launch in namespace
``/<robot>``):

  in   pose_topic         PoseStamped, arena ENU (localization_node)
  in   camera_info_topic  CameraInfo, read once for K and image size
  out  sweep_status       String WAITING | SWEEPING | COMPLETE, RELIABLE +
                          TRANSIENT_LOCAL, so a late mission state machine
                          still sees COMPLETE
  out  /ap/vN/cmd_gps_pose  GlobalPosition, resent every tick (D8)

Timer: wall clock by default (use_sim_time false), so the node does not
subscribe to /clock (about 45 % of a core per subscriber, gotcha 34).
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy, qos_profile_sensor_data)

from ardupilot_msgs.msg import GlobalPosition
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from uav_planning.core.planner import Datum, GridSpec, arena_to_lla
from uav_planning.core.sweep import (
    SWEEPING, Camera, lane_half_width, lanes, parse_lane_order,
    path_length, plan_sweep, sweep_tick)
from uav_planning.follower_planner_node import build_gps_command


class SweepNode(Node):

    def __init__(self):
        super().__init__('sweep_node')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('robot', 'iris_1')
        self.declare_parameter('sysid', 1)
        self.declare_parameter('pose_topic', '')          # '' -> /<robot>/pose
        self.declare_parameter('camera_info_topic', '')   # '' -> /<robot>/camera/camera_info
        # Leader layer, m above home (D26). Also the camera's height above
        # the flat ground, which sets the footprint.
        self.declare_parameter('layer', 8.0)
        self.declare_parameter('layer_tolerance', 0.5)
        # Arena: must match leader_node's grid (D39 default 8x8 at 5 m).
        self.declare_parameter('grid_width', 8)
        self.declare_parameter('grid_height', 8)
        self.declare_parameter('resolution', 5.0)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('overlap', 0.8)
        self.declare_parameter('lane_axis', 'north')   # or 'east'
        # "2,0,1": lane indices in flying order; '' = from the nearer side.
        self.declare_parameter('lane_order', '')
        self.declare_parameter('tolerance', 1.0)          # m, waypoint reached
        self.declare_parameter('rate', 2.0)               # Hz
        # Warn if no waypoint is reached for this long while sweeping (wall
        # s): commands are dropped silently unless GUIDED and armed (D8).
        self.declare_parameter('stall_timeout', 60.0)
        self.declare_parameter('datum_lat', float('nan'))
        self.declare_parameter('datum_lon', float('nan'))
        self.declare_parameter('datum_alt', float('nan'))

        g = self.get_parameter
        self.robot = g('robot').value
        self.sysid = int(g('sysid').value)
        self.layer = float(g('layer').value)
        self.layer_tolerance = float(g('layer_tolerance').value)
        self.grid = GridSpec(int(g('grid_width').value), int(g('grid_height').value),
                             float(g('resolution').value), float(g('origin_x').value),
                             float(g('origin_y').value))
        self.overlap = float(g('overlap').value)
        self.axis = g('lane_axis').value
        self.lane_order = parse_lane_order(g('lane_order').value)
        self.tolerance = float(g('tolerance').value)
        self.stall_timeout = float(g('stall_timeout').value)
        datum = [float(g(n).value) for n in ('datum_lat', 'datum_lon', 'datum_alt')]
        if not all(math.isfinite(v) for v in datum):
            raise ValueError(
                "datum_lat, datum_lon and datum_alt are required and must "
                f"match localization_node's (got {datum})")
        self.datum = Datum(*datum)
        pose_topic = g('pose_topic').value or f'/{self.robot}/pose'
        info_topic = g('camera_info_topic').value or f'/{self.robot}/camera/camera_info'

        # ---- State -------------------------------------------------------
        self.pose = None            # (x, y, z) arena
        self.camera = None          # Camera, from the first CameraInfo
        self.waypoints = None       # planned once
        self.index = 0
        self.status = None
        self.progress_at = None     # wall s of the last advance

        # ---- QoS (same profiles as follower_planner_node) ----------------
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped, pose_topic, self._on_pose, state_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, 'sweep_status', status_qos)
        self.cmd_pub = self.create_publisher(
            GlobalPosition, f'/ap/v{self.sysid}/cmd_gps_pose', cmd_qos)
        self.create_timer(1.0 / float(g('rate').value), self._tick)

        self.get_logger().info(
            f"sweep_node up | robot={self.robot} sysid={self.sysid} "
            f"layer={self.layer} m | grid {self.grid.width}x{self.grid.height} "
            f"at {self.grid.resolution} m | pose '{pose_topic}' | "
            f"camera_info '{info_topic}'")

    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def _on_camera_info(self, msg: CameraInfo):
        if msg.k[0] <= 0 or msg.k[4] <= 0:
            self.get_logger().warn("camera_info with no focal length; ignored",
                                   throttle_duration_sec=10.0)
            return
        self.camera = Camera(msg.k[0], msg.k[4], msg.k[2], msg.k[5],
                             msg.width, msg.height)
        # Intrinsics do not change: one message is enough.
        self.destroy_subscription(self.info_sub)

    def _plan(self):
        hw = lane_half_width(self.camera, self.layer, self.overlap)
        start = self.pose[:2]
        self.waypoints = plan_sweep(self.grid, hw, start, self.axis, self.lane_order)
        n = len(lanes(self.grid, hw, self.axis))
        length = path_length(start, self.waypoints)
        self.get_logger().info(
            f"planned {n} {self.axis} lanes, {len(self.waypoints)} "
            f"waypoints, {length:.0f} m from ({start[0]:.1f}, {start[1]:.1f}); "
            f"lane half-width {hw:.2f} m (fx {self.camera.fx:.1f}, "
            f"{self.camera.width}x{self.camera.height})")
        for i, (x, y) in enumerate(self.waypoints):
            self.get_logger().info(f"  waypoint {i}: ({x:.2f}, {y:.2f})")

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.waypoints is None and self.camera is not None and self.pose is not None:
            try:
                self._plan()
            except ValueError as e:
                self.get_logger().error(f"cannot plan the sweep: {e}",
                                        throttle_duration_sec=30.0)
        out = sweep_tick(self.waypoints, self.index, self.pose, self.layer,
                         self.tolerance, self.layer_tolerance)
        if out.index != self.index:
            self.get_logger().info(
                f"reached waypoint {self.index} at "
                f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, {self.pose[2]:.2f})")
            self.progress_at = now
        self.index = out.index
        if out.status != self.status:
            self.get_logger().info(f"status {out.status} ({out.reason})")
            if out.status == SWEEPING and self.progress_at is None:
                self.progress_at = now
            self.status = out.status
            # Latched: publish on change, so the last value is COMPLETE.
            self.status_pub.publish(String(data=out.status))
        if out.command is None:
            return
        lat, lon, _ = arena_to_lla(self.datum, *out.command)
        self.cmd_pub.publish(build_gps_command(
            lat, lon, self.layer, self.get_clock().now().to_msg()))
        if self.status == SWEEPING and now - self.progress_at > self.stall_timeout:
            self.get_logger().warn(
                f"no waypoint reached for {now - self.progress_at:.0f} s "
                f"(flying to {self.index}): is the drone GUIDED and armed?",
                throttle_duration_sec=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
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
