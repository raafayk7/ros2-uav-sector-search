"""Send one /ap/v2/cmd_gps_pose (FRAME_GLOBAL_REL_ALT) and watch both drones.

Target is computed from Gazebo navsat (the frame localization_node projects),
so this also tests whether a Gazebo-derived lat/lon lands where intended.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix

R = 6378137.0


class P(Node):
    def __init__(self):
        super().__init__('goal_probe')
        self.gz = {}
        self.goal = None
        for i in (1, 2):
            self.create_subscription(NavSatFix, f'/iris_{i}/navsat',
                                     lambda m, i=i: self.gz.__setitem__(i, m), qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GeoPointStamped, '/ap/v2/goal_lla',
                                 lambda m: setattr(self, 'goal', m), latched)

    def spin_for(self, s):
        t = time.time() + s
        while time.time() < t:
            rclpy.spin_once(self, timeout_sec=0.05)


def en(m, ref):
    return (R * math.cos(math.radians(ref.latitude)) * math.radians(m.longitude - ref.longitude),
            R * math.radians(m.latitude - ref.latitude))


rclpy.init()
n = P()
n.spin_for(5)
ref = n.gz[1]
start = en(n.gz[2], ref)
print('iris_2 start EN from iris_1 (%.2f, %.2f) alt delta %.1f' % (*start, n.gz[2].altitude - ref.altitude))
print('iris_1 EN (%.2f, %.2f) alt delta %.1f' % (*en(n.gz[1], ref), n.gz[1].altitude - ref.altitude))

msg = GlobalPosition()
msg.header.frame_id = 'map'
msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
msg.type_mask = (GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | GlobalPosition.IGNORE_VZ |
                 GlobalPosition.IGNORE_AFX | GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ |
                 GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE)
msg.latitude = n.gz[2].latitude + math.degrees(8.0 / R)
msg.longitude = n.gz[2].longitude
msg.altitude = 10.0
pub = n.create_publisher(GlobalPosition, '/ap/v2/cmd_gps_pose', QoSProfile(
    depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
n.spin_for(3)
print('subscribers on cmd_gps_pose:', pub.get_subscription_count())
# A single best-effort publish: does one message stick, or must it be repeated?
pub.publish(msg)
print('published ONE message; target = 8 m north of iris_2')
t0 = time.time()
sim0 = None
while time.time() - t0 < 90:
    n.spin_for(5)
    e2, e1 = en(n.gz[2], ref), en(n.gz[1], ref)
    north = e2[1] - start[1]
    gl = ('goal_lla north-of-start %.2f' % (R * math.radians(n.goal.position.latitude - n.gz[2].latitude) + north)) if n.goal else 'goal_lla none'
    print('t+%3.0fs iris_2 moved N %.2f E %.2f alt %.1f | iris_1 EN (%.2f, %.2f) | %s' % (
        time.time() - t0, north, e2[0] - start[0], n.gz[2].altitude - ref.altitude, *e1, gl))
    if abs(north - 8.0) < 0.4:
        print('ARRIVED within 0.4 m')
        break
rclpy.shutdown()
