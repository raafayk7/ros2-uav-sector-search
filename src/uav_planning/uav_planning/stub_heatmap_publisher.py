"""stub_heatmap_publisher: a static map on /heatmap for the milestones (D19).

Stands in for leader_node (and later belief_map_node) so the follower can fly
before the leader sweeps. Publishes one nav_msgs/OccupancyGrid, RELIABLE +
TRANSIENT_LOCAL depth 1: DDS keeps that one message and hands it to every
subscriber that joins later, so publishing once is enough.

Defaults are the Nov 10 / Nov 15 map: 8x8 sectors at 5 m from the arena
origin corner, three peaks at least 2 sectors apart. This is not the thesis
arena size (4x4 over 20x20 m), on purpose; leader_node keeps those defaults.

  peaks       "col:row:percent,..."   default "3:4:90,6:1:80,1:6:70"
  background  percent for every other cell (-1 = unknown, counts as 0.5)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from nav_msgs.msg import OccupancyGrid

from uav_planning.core.follower import parse_peaks, stub_map_data


class StubHeatmapPublisher(Node):

    def __init__(self):
        super().__init__('stub_heatmap_publisher')
        self.declare_parameter('heatmap_topic', '/heatmap')
        self.declare_parameter('arena_frame', 'arena')
        self.declare_parameter('width', 8)
        self.declare_parameter('height', 8)
        self.declare_parameter('resolution', 5.0)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('background', 5)
        self.declare_parameter('peaks', '3:4:90,6:1:80,1:6:70')

        g = self.get_parameter
        width, height = int(g('width').value), int(g('height').value)
        peaks = parse_peaks(g('peaks').value)

        grid = OccupancyGrid()
        grid.header.frame_id = g('arena_frame').value
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = float(g('resolution').value)
        grid.info.width, grid.info.height = width, height
        grid.info.origin.position.x = float(g('origin_x').value)
        grid.info.origin.position.y = float(g('origin_y').value)
        grid.info.origin.orientation.w = 1.0
        grid.data = stub_map_data(width, height, int(g('background').value), peaks)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        topic = g('heatmap_topic').value
        self.pub = self.create_publisher(OccupancyGrid, topic, qos)
        self.pub.publish(grid)
        self.get_logger().info(
            f"published {width}x{height} at {grid.info.resolution} m on '{topic}' "
            f"(latched), peaks (col, row, %) = {peaks}")


def main(args=None):
    rclpy.init(args=args)
    node = StubHeatmapPublisher()
    try:
        # Stay alive: the latched message lives only as long as its publisher.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
