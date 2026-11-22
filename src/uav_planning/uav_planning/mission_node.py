"""mission_node: the mission state machine, a thin ROS wrapper (D41).

Every rule lives in core/mission.py (when to start, when it is over, what the
report says). This node stores the latest messages and, once a second, calls
``Mission.tick`` and turns the answer into messages.

Why planner_enable is resent every tick rather than latched: it is a
command, and the planners subscribe to it RELIABLE + VOLATILE (no history),
so a planner that restarts would never see a latched value. Resending at
1 Hz means any planner, however late, has the current command within a
second, and the planner ignores a repeat of what it already has (a second
"false" sends no second hold). /mission_state and /mission_report are the
opposite case, records rather than commands: latched (TRANSIENT_LOCAL), so a
probe or recorder that joins after the end still receives the outcome.

Topics (templates with {robot}, absolute by default as in the other nodes):

  in   sweep_status_topic   String, the leader's sweep, latched
                            ('' -> /<leader>/sweep_status)
  in   heatmap_topic        OccupancyGrid, latched (/heatmap)
  in   finds_topic          String JSON, latched (/confirmed_finds)
  in   status_topic         String per follower, every planner tick
                            (/{robot}/planner_status)
  in   pose_topic           PoseStamped per follower; its stamp (the navsat
                            sim stamp, copied by localization_node) is the
                            node's only sim time (/{robot}/pose)
  out  enable_topic         Bool per follower, every tick
                            (/{robot}/planner_enable)
  out  state_topic          String JSON {state, reason}, latched, on change
                            (/mission_state)
  out  report_topic         String JSON, Mission.report(), latched, once at
                            the end (/mission_report)
  out  return_topic         String JSON, latched, on change (/mission_return)
  out  /ap/vN/cmd_gps_pose  GlobalPosition per follower, BEST_EFFORT, only
                            during the return

Return to find (D41 amended, optional, ``return_to_finds``): one tick after
DONE the node asks ``assign_returns`` which follower hovers over which find,
then resends a ``cmd_gps_pose`` to each of them every tick, at that drone's own
layer, until all are within ``arrival_radius`` or ``return_timeout`` wall
seconds pass. A timed-out mission skips it. The planners are disabled by then
and send nothing, so the two never fight over a drone. Its progress is on
latched ``/mission_return`` (JSON: status RETURNING | AT_FINDS | TIMED_OUT,
and each drone's cell, x, y and layer). The commands are the same message the
planners send (``build_gps_command``, D8), so ``cmd_gps_pose`` needs the
shared datum and each follower's sysid and layer.

Time: wall clock (use_sim_time false), so no /clock subscriber (about 45 % of
a core each, docker-notes 34). The D14 hold is in wall seconds, like the
belief map; the timeout is in sim seconds read from the followers' pose
stamps. Poses flow from the first GPS fix, before t0, so sim time is known
for the whole mission. (Session 11 first used goal_sector stamps, which
start only once a follower has a goal, about 50 wall-s in; the gap was then
counted at RTF 1 and the report's sim_s ran about 30 % high.)
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from ardupilot_msgs.msg import GlobalPosition
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String

from uav_planning.core.mission import (
    DONE, ENDED, Mission, MissionParams, SimClock, assign_returns,
    cell_centre, parse_finds, parse_names, parse_pairs)
from uav_planning.core.planner import Datum, arena_to_lla
from uav_planning.follower_planner_node import build_gps_command

RETURNING = 'RETURNING'
AT_FINDS = 'AT_FINDS'
RETURN_TIMED_OUT = 'TIMED_OUT'


def _secs(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ---- Parameters -------------------------------------------------
        d = self.declare_parameter
        d('followers', 'iris_2,iris_3,iris_4')
        d('leader', 'iris_1')
        d('threshold', MissionParams.threshold)   # D14
        d('hold', MissionParams.hold)             # D14, wall s
        d('timeout', MissionParams.timeout)       # D41, sim s after t0
        d('sweep_status_topic', '')               # '' -> /<leader>/sweep_status
        d('heatmap_topic', '/heatmap')
        d('finds_topic', '/confirmed_finds')
        d('status_topic', '/{robot}/planner_status')
        d('pose_topic', '/{robot}/pose')
        d('enable_topic', '/{robot}/planner_enable')
        d('state_topic', '/mission_state')
        d('report_topic', '/mission_report')
        d('return_topic', '/mission_return')
        # Return to find (off unless asked: it needs the datum, the sysids
        # and the layers, as "iris_2:2,iris_3:3" and "iris_2:10,iris_3:12").
        d('return_to_finds', False)
        d('sysids', '')
        d('layers', '')
        d('datum_lat', float('nan'))
        d('datum_lon', float('nan'))
        d('datum_alt', float('nan'))
        d('arrival_radius', 2.0)         # m, horizontal
        d('return_timeout', 120.0)       # wall s after the first command

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.followers = parse_names(g('followers'))
        self.leader = g('leader')
        params = MissionParams(threshold=float(g('threshold')),
                               hold=float(g('hold')),
                               timeout=float(g('timeout')))
        self.mission = Mission(self.followers, params)  # raises if none
        self.return_to_finds = bool(g('return_to_finds'))
        self.arrival_radius = float(g('arrival_radius'))
        self.return_timeout = float(g('return_timeout'))
        self.sysids, self.layers, self.datum = {}, {}, None
        if self.return_to_finds:
            self.sysids = parse_pairs(g('sysids'), int)
            self.layers = parse_pairs(g('layers'), float)
            datum = [float(g(n)) for n in ('datum_lat', 'datum_lon', 'datum_alt')]
            missing = [f for f in self.followers
                       if f not in self.sysids or f not in self.layers]
            if missing or not all(math.isfinite(v) for v in datum):
                raise ValueError(
                    "return_to_finds needs datum_lat/lon/alt and a sysid and "
                    f"a layer for every follower (missing {missing}, "
                    f"datum {datum})")
            self.datum = Datum(*datum)
        sweep_topic = g('sweep_status_topic') or f'/{self.leader}/sweep_status'
        per = {f: {k: g(k).format(robot=f)
                   for k in ('status_topic', 'pose_topic', 'enable_topic')}
               for f in self.followers}
        self.topics = {'sweep_status': sweep_topic,
                       'heatmap': g('heatmap_topic'),
                       'finds': g('finds_topic'),
                       'state': g('state_topic'),
                       'report': g('report_topic'),
                       'return': g('return_topic'),
                       'followers': per}

        # ---- State (written by callbacks, read by the timer) ------------
        self.sweep_status = None
        self.occupancy = None
        self.width = 0
        self.finds = []
        self.status = {f: None for f in self.followers}
        self.sim = SimClock()
        self.state = None
        self.reason = None
        self.logged_finds = set()
        self.reported = False
        self.poses = {}             # follower -> latest arena (x, y)
        self.resolution, self.origin = 5.0, (0.0, 0.0)
        self.return_plan = None     # follower -> (cell, x, y)
        self.return_started = None  # wall s of the first command
        self.return_status = None
        self.end_state = None

        # ---- QoS ---------------------------------------------------------
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # planner_status and pose: RELIABLE + VOLATILE depth 10, what
        # follower_planner_node and localization_node offer.
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        # planner_enable: RELIABLE depth 1, what the planner subscribes with.
        enable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ---- Subscriptions / publishers ---------------------------------
        self.create_subscription(String, sweep_topic, self._on_sweep, latched)
        self.create_subscription(
            OccupancyGrid, self.topics['heatmap'], self._on_heatmap, latched)
        self.create_subscription(
            String, self.topics['finds'], self._on_finds, latched)
        self.enable_pubs = {}
        for f, t in per.items():
            self.create_subscription(
                String, t['status_topic'],
                lambda m, n=f: self._on_status(n, m), state_qos)
            self.create_subscription(
                PoseStamped, t['pose_topic'],
                lambda m, n=f: self._on_pose(n, m), state_qos)
            self.enable_pubs[f] = self.create_publisher(
                Bool, t['enable_topic'], enable_qos)
        self.state_pub = self.create_publisher(
            String, self.topics['state'], latched)
        self.report_pub = self.create_publisher(
            String, self.topics['report'], latched)
        self.return_pub = self.create_publisher(
            String, self.topics['return'], latched)
        # Command to AP_DDS: BEST_EFFORT + VOLATILE, as the planners send it.
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.cmd_pubs = {f: self.create_publisher(
            GlobalPosition, f'/ap/v{self.sysids[f]}/cmd_gps_pose', cmd_qos)
            for f in self.followers if self.return_to_finds}

        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"mission_node up | followers={self.followers} | sweep "
            f"'{sweep_topic}' | threshold {params.threshold} hold "
            f"{params.hold:.0f} wall-s timeout {params.timeout:.0f} sim-s")

    # ------------------------------------------------------------------
    # Callbacks: store, never decide
    # ------------------------------------------------------------------
    def _wall(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_sweep(self, msg: String):
        self.sweep_status = msg.data

    def _on_heatmap(self, msg: OccupancyGrid):
        self.occupancy, self.width = list(msg.data), msg.info.width
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def _on_finds(self, msg: String):
        try:
            self.finds = parse_finds(msg.data)
        except ValueError as e:
            self.get_logger().error(f"confirmed_finds ignored: {e}")

    def _on_status(self, name, msg: String):
        self.status[name] = msg.data

    def _on_pose(self, name, msg: PoseStamped):
        self.poses[name] = (msg.pose.position.x, msg.pose.position.y)
        stamp = _secs(msg.header.stamp)
        if stamp > 0.0:     # an unstamped message carries no sim time
            self.sim.update(stamp, self._wall())

    # ------------------------------------------------------------------
    # The 1 Hz tick
    # ------------------------------------------------------------------
    def _tick(self):
        wall = self._wall()
        out = self.mission.tick(
            wall, self.sim.now(wall), self.sweep_status, self.occupancy,
            self.width, self.finds, self.status)

        for f, pub in self.enable_pubs.items():
            pub.publish(Bool(data=out.enable))

        for find in self.mission.finds.values():
            if find.cell not in self.logged_finds:
                self.logged_finds.add(find.cell)
                sim = '' if find.sim is None else f', sim +{find.sim:.1f} s'
                self.get_logger().info(
                    f"find {find.cell} by {find.drone} at +{find.wall:.1f} s{sim}")

        if out.state != self.state:
            self.get_logger().info(f"state {self.state} -> {out.state} ({out.reason})")
        # After the end the core says only 'ended'; keep the latched reason.
        ended_before = self.state in ENDED
        if (out.state, out.reason) != (self.state, self.reason) and not ended_before:
            self.state, self.reason = out.state, out.reason
            self.state_pub.publish(String(data=json.dumps(
                {'state': out.state, 'reason': out.reason})))

        if out.state in ENDED and not self.reported:
            self.reported = True
            self.end_state = out.state
            report = self.mission.report()
            self.report_pub.publish(String(data=json.dumps(report)))
            self.get_logger().info(f"report: {json.dumps(report)}")
        elif self.reported and self.return_to_finds and self.end_state == DONE:
            # A tick after the end, so each planner's hold (D29) went first.
            self._return_tick(wall)

    # ------------------------------------------------------------------
    # Return to find
    # ------------------------------------------------------------------
    def _return_tick(self, wall):
        if self.return_plan is None:
            finds = [(f.cell, f.drone) for f in sorted(
                self.mission.finds.values(), key=lambda f: f.wall)]
            cells = assign_returns(finds, self.poses, self.followers,
                                   self.resolution, self.origin)
            self.return_plan = {
                f: (cell, *cell_centre(cell, self.resolution, self.origin))
                for f, cell in cells.items()}
            self.return_started = wall
            self.get_logger().info(
                "return to find: " + (', '.join(
                    f"{f} -> {c}" for f, (c, _, _) in self.return_plan.items())
                    or 'no finds'))
        if self.return_status in (AT_FINDS, RETURN_TIMED_OUT):
            return
        stamp = self.get_clock().now().to_msg()
        arrived = True
        for f, (cell, x, y) in self.return_plan.items():
            lat, lon, _ = arena_to_lla(self.datum, x, y)
            self.cmd_pubs[f].publish(
                build_gps_command(lat, lon, self.layers[f], stamp))
            pose = self.poses.get(f)
            if pose is None or math.hypot(pose[0] - x, pose[1] - y) \
                    > self.arrival_radius:
                arrived = False
        if arrived:
            status = AT_FINDS
        elif wall - self.return_started > self.return_timeout:
            status = RETURN_TIMED_OUT
        else:
            status = RETURNING
        if status != self.return_status:
            self.return_status = status
            self.get_logger().info(f"return to find: {status}")
            self.return_pub.publish(String(data=json.dumps({
                'status': status,
                'wall_s': wall - self.return_started,
                'targets': {f: {'cell': list(c), 'x': x, 'y': y,
                                'layer': self.layers[f]}
                            for f, (c, x, y) in self.return_plan.items()}})))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
