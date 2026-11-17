"""Live check of ground_truth_detector against leader_node (session 7).

Needs a running sim with a hovering drone, localization_node, the detector and
leader_node (docs/planner-decisions.md "Implementation notes (session 7)" has
the commands), and target_* markers spawned in the world. Prints the heatmap
next to what it should look like: cells the camera covers that hold a target
should rise above 50, covered empty cells fall below 50, uncovered cells stay
unknown (-1). Run inside the container, piped in with `python3 -`.

  python3 - ROBOT_POSE_TOPIC SECS < detector_live_check.py   (defaults below)
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo

from uav_perception.core.projection import (NADIR_QUAT, camera_pose, in_frame,
                                            project_ground_point)

POSE = '/iris_1/pose'
CINFO = '/iris_1/camera/camera_info'
SECS = 30.0


class N(Node):
    def __init__(self):
        super().__init__('detector_live_check')
        self.pose = self.k = self.grid = None
        self.create_subscription(PoseStamped, POSE, lambda m: setattr(self, 'pose', m),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo, CINFO, lambda m: setattr(self, 'k', m),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/heatmap', lambda m: setattr(self, 'grid', m),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))


def spin(n, s):
    t = time.time() + s
    while time.time() < t:
        rclpy.spin_once(n, timeout_sec=0.05)


rclpy.init()
n = N()
spin(n, SECS)
p, q = n.pose.pose.position, n.pose.pose.orientation
print('drone (%.2f, %.2f, %.2f)  yaw %.0f deg' % (
    p.x, p.y, p.z, math.degrees(2 * math.atan2(q.z, q.w))))
fx, fy, cx, cy = n.k.k[0], n.k.k[4], n.k.k[2], n.k.k[5]
cam, R = camera_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), NADIR_QUAT)
g = n.grid
res, W, H = g.info.resolution, g.info.width, g.info.height
ox, oy = g.info.origin.position.x, g.info.origin.position.y
data = np.array(g.data).reshape(H, W)

# Which cell centres does the camera cover? (centre inside the image; the
# leader uses the ground quad of the image corners, the same thing.)
print('row 0 = south. cell: value | covered by camera | target inside')
targets = [tuple(float(v) for v in a.split(',')) for a in sys.argv[1:]]
for row in range(H - 1, -1, -1):
    out = []
    for col in range(W):
        cxm, cym = ox + (col + .5) * res, oy + (row + .5) * res
        pr = project_ground_point(cxm, cym, cam, R, fx, fy, cx, cy)
        cov = pr is not None and in_frame(pr[0], pr[1], 640, 480)
        tg = any(int((x - ox) // res) == col and int((y - oy) // res) == row
                 for x, y in targets)
        out.append('%3d %s%s' % (data[row, col], 'C' if cov else '.', 'T' if tg else ' '))
    print('  '.join(out))
rclpy.shutdown()
