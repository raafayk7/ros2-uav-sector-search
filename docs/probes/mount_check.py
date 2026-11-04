"""Ground-truth test of the camera mount: known markers vs real image pixels.

Spawns coloured boxes on the ground, grabs one /camera/image frame, finds each
colour's centroid, and compares it to the pixel predicted by (a) leader_node's
default nadir mount and (b) the mount measured from tf + the SDF sensor pose.
Prints the measured mount as a leader_node mount_quat.
"""
import math
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

SDF = """<?xml version="1.0"?><sdf version="1.9"><model name="{n}"><static>true</static>
<link name="l"><visual name="v"><geometry><box><size>{s} {s} 0.1</size></box></geometry>
<material><ambient>{c}</ambient><diffuse>{c}</diffuse><emissive>{c}</emissive></material>
</visual></link></model></sdf>"""

MARKERS = {
    # name: (world x, world y, rgba)
    'red': (-12.9, 0.8, '1 0 0 1'),      # where the measured mount should centre
    'green': (0.0, 0.0, '0 1 0 1'),      # straight below: nadir's image centre
    'blue': (-8.0, 8.0, '0 0 1 1'),      # off-axis check
}


def q2r(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def r2q(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0, 0, 0, (R[k, j] - R[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return q


# Gazebo sensor frame (x=view, y=left, z=up) -> optical (x=right, y=down, z=view).
SENSOR_R_OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
PITCH_R_SENSOR = rpy(-1.57, -1.57, 0.0)
NADIR_BODY_R_CAM = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], dtype=float)


class Grab(Node):
    def __init__(self):
        super().__init__('mount_check', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.img = self.K = self.odom = None
        self.create_subscription(Image, '/camera/image', self._img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._ci, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry', self._od, qos_profile_sensor_data)

    def _img(self, m):
        self.img = m

    def _ci(self, m):
        self.K = np.array(m.k).reshape(3, 3)

    def _od(self, m):
        self.odom = m


def project(Pw, cam_pos, world_R_cam, K):
    d = world_R_cam.T @ (Pw - cam_pos)
    if d[2] <= 0:
        return None
    return K[0, 0] * d[0] / d[2] + K[0, 2], K[1, 1] * d[1] / d[2] + K[1, 2]


for name, (x, y, c) in MARKERS.items():
    path = f'/tmp/{name}.sdf'
    with open(path, 'w') as f:
        f.write(SDF.format(n=name, s=2.0, c=c))
    r = subprocess.run(
        ['gz', 'service', '-s', '/world/runway/create',
         '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req',
         f'sdf_filename: "{path}", pose: {{position: {{x: {x}, y: {y}, z: 0.05}}}}, name: "{name}"'],
        capture_output=True, text=True)
    print('spawn', name, (x, y), r.stdout.strip() or r.stderr.strip())

rclpy.init()
n = Grab()
t0 = time.time()
while time.time() - t0 < 8.0 or n.img is None or n.K is None or n.odom is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - t0 > 40:
        break

tf = n.buf.lookup_transform('base_link', 'pitch_link', rclpy.time.Time())
q = tf.transform.rotation
body_R_cam = q2r(q.x, q.y, q.z, q.w) @ PITCH_R_SENSOR @ SENSOR_R_OPTICAL
print('measured mount_quat (x,y,z,w):', [round(v, 4) for v in r2q(body_R_cam)])

p = n.odom.pose.pose.position
oq = n.odom.pose.pose.orientation
world_R_body = q2r(oq.x, oq.y, oq.z, oq.w)
cam = np.array([p.x, p.y, p.z])
yaw = math.degrees(math.atan2(world_R_body[1, 0], world_R_body[0, 0]))
print('odom pos (%.2f, %.2f, %.2f), yaw %.1f deg' % (p.x, p.y, p.z, yaw))

m = n.img
arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
print('image', m.width, m.height, m.encoding, 'mean', round(float(arr.mean()), 1))
rgb = arr[:, :, :3] if m.encoding.startswith('rgb') else arr[:, :, 2::-1]
cv2.imwrite('/tmp/cam_frame.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

masks = {
    'red': (rgb[:, :, 0] > 150) & (rgb[:, :, 1] < 80) & (rgb[:, :, 2] < 80),
    'green': (rgb[:, :, 1] > 150) & (rgb[:, :, 0] < 80) & (rgb[:, :, 2] < 80),
    'blue': (rgb[:, :, 2] > 150) & (rgb[:, :, 0] < 80) & (rgb[:, :, 1] < 80),
}
for name, (x, y, _) in MARKERS.items():
    P = np.array([x, y, 0.05])
    meas = project(P, cam, world_R_body @ body_R_cam, n.K)
    nad = project(P, cam, world_R_body @ NADIR_BODY_R_CAM, n.K)
    ys, xs = np.nonzero(masks[name])
    obs = (float(xs.mean()), float(ys.mean())) if len(xs) else None
    fmt = lambda v: 'n/a' if v is None else '(%.0f, %.0f)' % v
    err = lambda v: 'n/a' if v is None or obs is None else '%.0f px' % math.hypot(v[0] - obs[0], v[1] - obs[1])
    print(f'{name:6s} observed {fmt(obs):>12s} [{len(xs)} px] | '
          f'measured-mount pred {fmt(meas):>12s} err {err(meas):>7s} | '
          f'nadir pred {fmt(nad):>12s} err {err(nad):>7s}')
rclpy.shutdown()
