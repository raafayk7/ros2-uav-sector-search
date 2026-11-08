"""follower_planner_node: a thin ROS wrapper around uav_planning.core.

Every rule lives in core/planner.py (which sector) and core/follower.py (gate,
status, when to command). This node only stores the latest messages and, once
a second, calls ``follower_tick`` and turns the answer into messages. The
design is pinned in docs/planner-decisions.md (D8-D12, D26-D29).

Threading: one 1 Hz timer on the node clock (sim time under use_sim_time) and
the default single-threaded executor (D10). Callbacks never run concurrently,
so subscription callbacks just overwrite fields and the timer reads them; no
locks. The node never calls a service: arming is arm_takeoff's job (D15),
because /ap services hang under this container's CycloneDDS.

Topics (input topics are absolute, outputs relative, as in the other nodes;
launch the node in namespace ``/<robot>`` so outputs land at ``/<robot>/...``):

  in   /<robot>/pose             PoseStamped, arena ENU (localization_node)
  in   /<peer>/pose              one per named peer
  in   /<peer>/goal_sector       PointStamped, the peer's goal sector centre
  in   heatmap_topic (/heatmap)  OccupancyGrid, RELIABLE + TRANSIENT_LOCAL
  in   planner_enable            Bool; false sends one hold command (D29)
  in   /ap/vN/goal_lla           GeoPointStamped, the autopilot's accepted target
  out  goal_sector               PointStamped at 1 Hz while the goal is real (D28)
  out  planner_status            String: ASCENDING | ARRIVED | HOLDING (D12)
  out  /ap/vN/cmd_gps_pose       GlobalPosition, BEST_EFFORT, resent at 1 Hz
                                 until arrival (D8)
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String

from uav_planning.core.follower import (
    ASCENDING, follower_tick, hold_command, make_peer, parse_peers)
from uav_planning.core.planner import (
    WGS84_A, Datum, GridSpec, PlannerParams, arena_to_lla, pose_valid,
    values_from_occupancy)

# Position-only setpoint: AP_DDS ignores velocity, acceleration and yaw.
POSITION_ONLY = (
    GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY |
    GlobalPosition.IGNORE_VZ | GlobalPosition.IGNORE_AFX |
    GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ |
    GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE)

# goal_lla within this many metres of what we sent counts as accepted.
GOAL_MATCH_M = 1.0


def build_gps_command(lat: float, lon: float, rel_alt: float,
                      stamp=None) -> GlobalPosition:
    """The only command message AP_DDS accepts here (D8): frame_id exactly
    "map" (anything else is dropped silently) and FRAME_GLOBAL_REL_ALT, since
    Gazebo and autopilot absolute altitudes differ by ~574 m."""
    msg = GlobalPosition()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
    msg.type_mask = POSITION_ONLY
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = float(rel_alt)
    return msg


def _secs(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _ground_distance(lat0, lon0, lat1, lon1) -> float:
    """Metres between two nearby lat/lons (same flat-earth model as core)."""
    north = WGS84_A * math.radians(lat1 - lat0)
    east = WGS84_A * math.cos(math.radians(lat0)) * math.radians(lon1 - lon0)
    return math.hypot(east, north)


class FollowerPlannerNode(Node):

    def __init__(self):
        super().__init__('follower_planner_node')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('robot', 'iris_1')
        self.declare_parameter('sysid', 1)
        # Named peers, "robot:sysid,robot:sysid" (D9). Empty: flying alone.
        self.declare_parameter('peers', '')
        self.declare_parameter('heatmap_topic', '/heatmap')
        # Altitude layer above home, metres (D5: followers 10 + 2 x index).
        self.declare_parameter('layer', 10.0)
        self.declare_parameter('layer_tolerance', 0.5)
        self.declare_parameter('move_margin', PlannerParams.move_margin)
        self.declare_parameter('peer_timeout', PlannerParams.peer_timeout)
        self.declare_parameter('arrival_radius', PlannerParams.arrival_radius)
        # The shared world datum (D7). Required: without it an arena (x, y)
        # cannot become a lat/lon. Must equal localization_node's datum.
        self.declare_parameter('datum_lat', float('nan'))
        self.declare_parameter('datum_lon', float('nan'))
        self.declare_parameter('datum_alt', float('nan'))
        self.declare_parameter('start_enabled', True)
        # Warn if the autopilot has not echoed our target this long after
        # we first sent it (sim seconds).
        self.declare_parameter('goal_confirm_timeout', 5.0)

        g = self.get_parameter
        self.robot = g('robot').value
        self.sysid = int(g('sysid').value)
        self.layer = float(g('layer').value)
        self.layer_tolerance = float(g('layer_tolerance').value)
        self.params = PlannerParams(
            move_margin=float(g('move_margin').value),
            peer_timeout=float(g('peer_timeout').value),
            arrival_radius=float(g('arrival_radius').value))
        datum = [float(g(n).value) for n in ('datum_lat', 'datum_lon', 'datum_alt')]
        if not all(math.isfinite(v) for v in datum):
            raise ValueError(
                "datum_lat, datum_lon and datum_alt are required and must "
                f"match localization_node's (got {datum})")
        self.datum = Datum(*datum)
        self.enabled = bool(g('start_enabled').value)
        self.goal_confirm_timeout = float(g('goal_confirm_timeout').value)
        peers = parse_peers(g('peers').value)

        if self.get_namespace().strip('/') != self.robot:
            self.get_logger().warn(
                f"namespace '{self.get_namespace()}' is not '/{self.robot}': "
                "peers look for /<robot>/goal_sector and will not find ours")

        # ---- State (written by callbacks, read by the timer) ------------
        self.pose = None            # (x, y, z) arena
        self.grid = None            # GridSpec from the latest heatmap
        self.values = None          # values[row][col]
        self.goal = None            # current Sector, or None
        # name -> [sysid, pose_xy, pose_stamp, goal_xy, goal_stamp, was_valid]
        self.peers = {name: [sysid, None, -math.inf, None, -math.inf, None]
                      for name, sysid in peers}
        self.sent = None            # (lat, lon) of the command in flight
        self.sent_at = None         # sim time it was first sent
        self.confirmed = False      # goal_lla matched self.sent
        self.last_status = None

        # ---- QoS ---------------------------------------------------------
        # Pose: RELIABLE + VOLATILE depth 10, what localization_node offers.
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        # Heatmap: RELIABLE + TRANSIENT_LOCAL depth 1, so a planner started
        # after the map was published still receives it.
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # goal_sector: RELIABLE + VOLATILE (D28). Never latched: a late
        # joiner must not be handed a goal that has already ended.
        goal_qos = state_qos
        # Command to AP_DDS: BEST_EFFORT + VOLATILE, what its reader expects.
        # Losses are fine because we resend at 1 Hz.
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # planner_enable: RELIABLE depth 1, a command (skill QoS convention).
        enable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # goal_lla: AP_DDS latches it. A BEST_EFFORT + VOLATILE reader accepts
        # any writer, so we cannot mismatch.
        echo_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ---- Subscriptions / publishers ---------------------------------
        self.create_subscription(
            PoseStamped, f'/{self.robot}/pose', self._on_pose, state_qos)
        for name in self.peers:
            self.create_subscription(
                PoseStamped, f'/{name}/pose',
                lambda m, n=name: self._on_peer_pose(n, m), state_qos)
            self.create_subscription(
                PointStamped, f'/{name}/goal_sector',
                lambda m, n=name: self._on_peer_goal(n, m), goal_qos)
        heatmap_topic = g('heatmap_topic').value
        self.create_subscription(
            OccupancyGrid, heatmap_topic, self._on_heatmap, map_qos)
        self.create_subscription(
            Bool, 'planner_enable', self._on_enable, enable_qos)
        self.create_subscription(
            GeoPointStamped, f'/ap/v{self.sysid}/goal_lla', self._on_goal_lla,
            echo_qos)

        self.goal_pub = self.create_publisher(PointStamped, 'goal_sector', goal_qos)
        self.status_pub = self.create_publisher(String, 'planner_status', state_qos)
        self.cmd_pub = self.create_publisher(
            GlobalPosition, f'/ap/v{self.sysid}/cmd_gps_pose', cmd_qos)

        # Created on the node clock, so it ticks in sim time (D10).
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"follower_planner_node up | robot={self.robot} sysid={self.sysid} "
            f"layer={self.layer} m | peers={peers or 'none'} | "
            f"heatmap='{heatmap_topic}' | enabled={self.enabled}")

    # ------------------------------------------------------------------
    # Callbacks: store, never decide
    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def _on_peer_pose(self, name, msg: PoseStamped):
        rec = self.peers[name]
        rec[1] = (msg.pose.position.x, msg.pose.position.y)
        rec[2] = _secs(msg.header.stamp)

    def _on_peer_goal(self, name, msg: PointStamped):
        rec = self.peers[name]
        rec[3] = (msg.point.x, msg.point.y)
        rec[4] = _secs(msg.header.stamp)

    def _on_heatmap(self, msg: OccupancyGrid):
        info = msg.info
        grid = GridSpec(info.width, info.height, info.resolution,
                        info.origin.position.x, info.origin.position.y)
        try:
            values = values_from_occupancy(msg.data, grid)
        except ValueError as e:
            self.get_logger().error(f"heatmap ignored: {e}")
            return
        if grid != self.grid:
            self.get_logger().info(
                f"heatmap {grid.width}x{grid.height} at {grid.resolution} m, "
                f"origin ({grid.origin_x}, {grid.origin_y})")
            if self.goal is not None and not grid.contains(self.goal):
                self.goal = None
        self.grid, self.values = grid, values

    def _on_enable(self, msg: Bool):
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        if self.enabled:
            self.get_logger().info("planner enabled")
            return
        # D29: one goal at the current position and layer, then nothing.
        self.goal = None
        self.sent = None
        hold = hold_command(self.pose)
        if hold is None:
            self.get_logger().warn("planner disabled with no pose: no hold sent")
            return
        lat, lon, _ = arena_to_lla(self.datum, *hold)
        self.cmd_pub.publish(build_gps_command(
            lat, lon, self.layer, self.get_clock().now().to_msg()))
        self.get_logger().info(
            f"planner disabled: hold at ({hold[0]:.1f}, {hold[1]:.1f}) "
            f"{self.layer} m, sent once")

    def _on_goal_lla(self, msg: GeoPointStamped):
        if self.sent is not None and _ground_distance(
                *self.sent, msg.position.latitude,
                msg.position.longitude) <= GOAL_MATCH_M:
            self.confirmed = True

    # ------------------------------------------------------------------
    # The 1 Hz tick
    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        out = follower_tick(
            self.grid, self.values, self.sysid, self.pose, self.layer,
            self.goal, self._peer_list(now), now, self.params,
            self.layer_tolerance, self.enabled)

        if out.goal != self.goal:
            self.get_logger().info(
                f"goal {self.goal} -> {out.goal} ({out.reason})")
        self.goal = out.goal
        if out.status != self.last_status:
            self.get_logger().info(f"status {out.status} ({out.reason})")
            self.last_status = out.status
        self.status_pub.publish(String(data=out.status))

        stamp = self.get_clock().now().to_msg()
        if out.goal is not None:
            cx, cy = self.grid.sector_centre(out.goal)
            pt = PointStamped()
            pt.header.stamp = stamp
            pt.header.frame_id = 'arena'
            pt.point.x, pt.point.y, pt.point.z = cx, cy, self.layer
            self.goal_pub.publish(pt)

        if out.status == ASCENDING and out.command is not None:
            lat, lon, _ = arena_to_lla(self.datum, *out.command)
            if self.sent != (lat, lon):
                self.sent, self.sent_at, self.confirmed = (lat, lon), now, False
            self.cmd_pub.publish(build_gps_command(lat, lon, self.layer, stamp))
            if (not self.confirmed
                    and now - self.sent_at > self.goal_confirm_timeout):
                self.get_logger().warn(
                    f"/ap/v{self.sysid}/goal_lla has not matched our target "
                    f"after {now - self.sent_at:.0f} s: is the drone GUIDED "
                    "and armed?", throttle_duration_sec=10.0)

    def _peer_list(self, now):
        peers = []
        for name, rec in self.peers.items():
            sysid, pose_xy, pose_stamp, goal_xy, goal_stamp, was_valid = rec
            peer = make_peer(self.grid, sysid, pose_xy, pose_stamp,
                             goal_xy, goal_stamp)
            valid = pose_valid(peer, now, self.params)
            if valid != was_valid:
                if valid:
                    self.get_logger().info(f"peer {name} reporting")
                elif was_valid is None:
                    self.get_logger().warn(f"peer {name} absent: no pose yet")
                else:
                    self.get_logger().warn(
                        f"peer {name} silent for {self.params.peer_timeout} s: "
                        "no longer blocking")
                rec[5] = valid
            peers.append(peer)
        return peers


def main(args=None):
    rclpy.init(args=args)
    node = FollowerPlannerNode()
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
