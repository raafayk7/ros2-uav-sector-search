"""ground_marker_node: keeps a marker on the ground under each drone (video).

For the arena_view option of swarm.launch.py. Each drone has a static,
visual-only model ``<prefix><robot>`` in the world (a coloured disc on the
ground). This node moves it to the drone's x and y by calling Gazebo's
``/world/<world>/set_pose`` service, which ``ros_gz_bridge`` exposes as
``ros_gz_interfaces/srv/SetEntityPose``. Seen from above through a
perspective camera, a high drone appears shifted away from the picture's
centre; the marker shows where it really is on the grid.

Nothing flies on this. The markers have no collision and no mass, and the
node only reads ``/<robot>/pose``.

Topics:

  in   /<robot>/pose   PoseStamped, arena ENU (localization_node), one per
                       robot in ``robots``

A request is sent only when the drone has moved at least ``min_move`` m since
the last one and the previous request for it has been answered, so a
hovering drone costs nothing and a slow service cannot queue up.

Timer: wall clock (use_sim_time false), so the node does not subscribe to
/clock (about 45 % of a core per subscriber, gotcha 34).
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def moved_enough(last, new, min_move):
    """True when ``new`` (x, y) is at least ``min_move`` m from ``last``.

    ``last`` is None before the first request, which always goes out."""
    if last is None:
        return True
    return math.hypot(new[0] - last[0], new[1] - last[1]) >= min_move


class GroundMarkerNode(Node):

    def __init__(self):
        super().__init__('ground_marker_node')

        self.declare_parameter('robots', ['iris_1'])
        self.declare_parameter('world', 'runway')
        self.declare_parameter('marker_prefix', 'ground_marker_')
        # Above the arena plate (0.03) and its lines (0.04), m.
        self.declare_parameter('marker_z', 0.06)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('min_move', 0.05)

        self.robots = [str(r) for r in self.get_parameter('robots').value]
        if not self.robots:
            raise ValueError('robots must not be empty')
        self.prefix = str(self.get_parameter('marker_prefix').value)
        self.z = float(self.get_parameter('marker_z').value)
        self.min_move = float(self.get_parameter('min_move').value)
        rate = float(self.get_parameter('rate_hz').value)
        if rate <= 0.0:
            raise ValueError('rate_hz must be positive')
        world = str(self.get_parameter('world').value)

        self.latest = {}    # robot -> (x, y)
        self.sent = {}      # robot -> (x, y) of the last request
        self.pending = {}   # robot -> future of the request in flight

        self.client = self.create_client(SetEntityPose,
                                         f'/world/{world}/set_pose')
        for robot in self.robots:
            self.create_subscription(
                PoseStamped, f'/{robot}/pose',
                lambda msg, r=robot: self._on_pose(r, msg),
                qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'ground_marker_node up | robots {self.robots} | '
            f'/world/{world}/set_pose | {rate:g} Hz, min move '
            f'{self.min_move:g} m')

    def _on_pose(self, robot, msg):
        p = msg.pose.position
        self.latest[robot] = (p.x, p.y)

    def _tick(self):
        if not self.client.service_is_ready():
            return
        for robot, xy in self.latest.items():
            future = self.pending.get(robot)
            if future is not None and not future.done():
                continue
            if not moved_enough(self.sent.get(robot), xy, self.min_move):
                continue
            req = SetEntityPose.Request()
            req.entity.name = self.prefix + robot
            req.entity.type = Entity.MODEL
            req.pose.position.x = xy[0]
            req.pose.position.y = xy[1]
            req.pose.position.z = self.z
            req.pose.orientation.w = 1.0
            self.pending[robot] = self.client.call_async(req)
            self.sent[robot] = xy


def main(args=None):
    rclpy.init(args=args)
    node = GroundMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
