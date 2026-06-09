"""fake_detection_publisher -- synthetic Detection2DArray for testing.

Stands in for perception_node (which doesn't exist yet) so leader_node can be
developed and verified in isolation. It publishes a Detection2DArray with one
(or more) boxes at a KNOWN image pixel, at a fixed rate, so you can:

  * place the iris over a known arena point,
  * emit a detection at a known pixel,
  * and check the heatmap cell that lights up against a hand computation.

Optionally it also publishes a synthetic CameraInfo, so leader_node can run
with NO simulator at all (a pure unit test). Leave that OFF when running
against iris_runway -- the sim already publishes /camera/camera_info, and you
don't want two publishers fighting over the same topic.

Pixel convention matches vision_msgs/CameraInfo: u is the column (x, 0..width),
v is the row (y, 0..height), origin top-left.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy)
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (Detection2DArray, Detection2D,
                             ObjectHypothesisWithPose)


class FakeDetectionPublisher(Node):

    def __init__(self):
        super().__init__('fake_detection_publisher')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera')

        # Detection pixel (scalar doubles -- one box per stub instance, which
        # is all the hand-checked test needs). For multiple markers, launch a
        # second stub instance (different node name) at another pixel.
        self.declare_parameter('u', 320.0)
        self.declare_parameter('v', 240.0)
        self.declare_parameter('score', 0.9)
        self.declare_parameter('bbox_size', 40.0)     # px, cosmetic
        self.declare_parameter('class_id', 'marker')
        self.declare_parameter('publish_period', 0.5)

        # Synthetic CameraInfo (only used when publish_camera_info is true).
        self.declare_parameter('publish_camera_info', False)
        self.declare_parameter('ci_width', 640)
        self.declare_parameter('ci_height', 480)
        # Defaults ~ 1.2 rad HFOV at 640 px: fx = (640/2)/tan(0.6) ~ 467.8.
        self.declare_parameter('ci_fx', 467.8)
        self.declare_parameter('ci_fy', 467.8)
        self.declare_parameter('ci_cx', 320.0)
        self.declare_parameter('ci_cy', 240.0)

        g = self.get_parameter
        self.det_topic = g('detections_topic').value
        self.cinfo_topic = g('camera_info_topic').value
        self.frame_id = g('frame_id').value
        self.u = float(g('u').value)
        self.v = float(g('v').value)
        self.score = float(g('score').value)
        self.bbox_size = float(g('bbox_size').value)
        self.class_id = g('class_id').value
        self.publish_ci = bool(g('publish_camera_info').value)

        det_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self._det_pub = self.create_publisher(
            Detection2DArray, self.det_topic, det_qos)

        if self.publish_ci:
            self._ci = self._build_camera_info()
            self._ci_pub = self.create_publisher(
                CameraInfo, self.cinfo_topic, qos_profile_sensor_data)
        else:
            self._ci = None

        self.create_timer(float(g('publish_period').value), self._tick)
        self.get_logger().info(
            f"fake_detection_publisher up | det='{self.det_topic}' "
            f"pixel=({self.u}, {self.v}) score={self.score} "
            f"camera_info={'ON' if self.publish_ci else 'OFF'}")

    def _build_camera_info(self):
        g = self.get_parameter
        ci = CameraInfo()
        ci.header.frame_id = self.frame_id
        ci.width = int(g('ci_width').value)
        ci.height = int(g('ci_height').value)
        fx, fy = float(g('ci_fx').value), float(g('ci_fy').value)
        cx, cy = float(g('ci_cx').value), float(g('ci_cy').value)
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return ci

    def _tick(self):
        now = self.get_clock().now().to_msg()

        if self._ci is not None:
            self._ci.header.stamp = now
            self._ci_pub.publish(self._ci)

        arr = Detection2DArray()
        arr.header.stamp = now
        arr.header.frame_id = self.frame_id
        det = Detection2D()
        det.header = arr.header
        det.bbox.center.position.x = self.u
        det.bbox.center.position.y = self.v
        det.bbox.size_x = self.bbox_size
        det.bbox.size_y = self.bbox_size
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.class_id
        hyp.hypothesis.score = self.score
        det.results.append(hyp)
        arr.detections.append(det)
        self._det_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
