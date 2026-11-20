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
  in   goal_topic           PointStamped per follower; its sim stamp is
                            the node's only sim time (/{robot}/goal_sector)
  out  enable_topic         Bool per follower, every tick
                            (/{robot}/planner_enable)
  out  state_topic          String JSON {state, reason}, latched, on change
                            (/mission_state)
  out  report_topic         String JSON, Mission.report(), latched, once at
                            the end (/mission_report)

Time: wall clock (use_sim_time false), so no /clock subscriber (about 45 % of
a core each, docker-notes 34). The D14 hold is in wall seconds, like the
belief map; the timeout is in sim seconds read from goal_sector stamps.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String

from uav_planning.core.mission import (
    ENDED, Mission, MissionParams, SimClock, parse_finds, parse_names)


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
        d('goal_topic', '/{robot}/goal_sector')
        d('enable_topic', '/{robot}/planner_enable')
        d('state_topic', '/mission_state')
        d('report_topic', '/mission_report')

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.followers = parse_names(g('followers'))
        self.leader = g('leader')
        params = MissionParams(threshold=float(g('threshold')),
                               hold=float(g('hold')),
                               timeout=float(g('timeout')))
        self.mission = Mission(self.followers, params)  # raises if none
        sweep_topic = g('sweep_status_topic') or f'/{self.leader}/sweep_status'
        per = {f: {k: g(k).format(robot=f)
                   for k in ('status_topic', 'goal_topic', 'enable_topic')}
               for f in self.followers}
        self.topics = {'sweep_status': sweep_topic,
                       'heatmap': g('heatmap_topic'),
                       'finds': g('finds_topic'),
                       'state': g('state_topic'),
                       'report': g('report_topic'),
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

        # ---- QoS ---------------------------------------------------------
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # planner_status and goal_sector: RELIABLE + VOLATILE depth 10, what
        # follower_planner_node offers.
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
                PointStamped, t['goal_topic'], self._on_goal, state_qos)
            self.enable_pubs[f] = self.create_publisher(
                Bool, t['enable_topic'], enable_qos)
        self.state_pub = self.create_publisher(
            String, self.topics['state'], latched)
        self.report_pub = self.create_publisher(
            String, self.topics['report'], latched)

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

    def _on_finds(self, msg: String):
        try:
            self.finds = parse_finds(msg.data)
        except ValueError as e:
            self.get_logger().error(f"confirmed_finds ignored: {e}")

    def _on_status(self, name, msg: String):
        self.status[name] = msg.data

    def _on_goal(self, msg: PointStamped):
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
            report = self.mission.report()
            self.report_pub.publish(String(data=json.dumps(report)))
            self.get_logger().info(f"report: {json.dumps(report)}")


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
