"""localization_node — GPS -> local ENU arena-frame pose.

This is the foundation node of the multi-agent UAV stack. Everything
downstream (heatmap, follower planner, landing) reasons about drone
positions in a single shared metric frame; this node produces that frame.

Design decisions (made consciously, see session notes):

* Arena frame = ENU (x=East, y=North, z=Up), per REP-103. ROS world
  frames are ENU by convention, Gazebo's world is ENU, and downstream
  planner math stays in standard orientation. ArduPilot is NED internally
  but we convert at the ROS boundary, not here.

* Origin = the drone's first valid GPS fix. We latch lat0/lon0/alt0 from
  the first usable /navsat message and express every later fix relative to
  it. This mirrors what a real drone must do (it only has GPS, not a
  god's-eye ground-truth frame), so the GPS->local pipeline actually gets
  built and exercised rather than shortcut via sim ground truth.

* Position comes from /navsat (honest GPS, equirectangular projection).
  Orientation comes from /odometry's quaternion (a stand-in for the real
  AHRS/IMU attitude estimate). Both inputs are used, as the task intends.

* Projection is equirectangular (flat-earth): good to well under a metre
  over an arena of a few hundred metres. Swap in pyproj if we ever need
  geodetic precision over larger areas.

Topic-naming strategy: input topics are absolute-by-default parameters
(/navsat, /odometry) so namespacing the node does NOT rewrite them — the
ros_gz_bridge publishes them globally today, and when AP_DDS comes online
you just pass navsat_topic:=/iris_1/ap/navsat/navsat0. The output topic is
*relative* ('pose'), so it auto-prefixes with the node namespace:
/pose today, /iris_1/pose once the swarm launch sets a namespace.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# WGS84 semi-major axis (metres). Earth radius used for the flat-earth
# projection. Good enough at arena scale; not a geodetic model.
WGS84_A = 6378137.0


class LocalizationNode(Node):
    """Subscribes GPS + odometry, publishes a local ENU PoseStamped."""

    def __init__(self):
        super().__init__('localization_node')

        # ---- Parameters (override from launch / CLI) -------------------
        # Inputs are absolute paths so node namespacing leaves them alone.
        self.declare_parameter('navsat_topic', '/navsat')
        self.declare_parameter('odom_topic', '/odometry')
        # Output is relative so it inherits the node namespace.
        self.declare_parameter('pose_topic', 'pose')
        # frame_id stamped on the published pose.
        self.declare_parameter('arena_frame', 'arena')
        # If the very first fix reports NO_FIX we skip it and wait; set this
        # False only if your SITL navsat never reports a fix status.
        self.declare_parameter('require_fix', True)

        navsat_topic = self.get_parameter('navsat_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.arena_frame = self.get_parameter('arena_frame').value
        self.require_fix = self.get_parameter('require_fix').value

        # ---- State -----------------------------------------------------
        # Origin datum, latched on first valid fix.
        self._lat0 = None
        self._lon0 = None
        self._alt0 = None
        self._cos_lat0 = None  # precomputed cos(lat0) for the East term
        # Latest orientation from odometry; identity until odom arrives.
        self._orientation = None

        # ---- QoS -------------------------------------------------------
        # INPUTS: SensorDataQoS is BEST_EFFORT. A BEST_EFFORT *subscriber*
        # is compatible with BOTH reliable and best-effort publishers
        # (offered reliability only has to be >= requested), so this
        # connects to whatever ros_gz_bridge offers and avoids the
        # single most common ROS 2 bug — a silent QoS mismatch.
        input_qos = qos_profile_sensor_data

        # OUTPUT: RELIABLE, VOLATILE, depth 10 (rclpy default). Pose is
        # low-rate state, not high-bandwidth sensor data, so guaranteed
        # delivery is cheap and a dropped pose could perturb a planner's
        # gradient step. Late joiners don't need history -> VOLATILE.
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Subscriptions / publisher ---------------------------------
        self.create_subscription(
            NavSatFix, navsat_topic, self._on_navsat, input_qos)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, input_qos)
        self._pose_pub = self.create_publisher(
            PoseStamped, pose_topic, pose_qos)

        self.get_logger().info(
            f"localization_node up | in: navsat='{navsat_topic}', "
            f"odom='{odom_topic}' | out: pose='{pose_topic}' "
            f"| arena_frame='{self.arena_frame}'")
        self.get_logger().info("Waiting for first GPS fix to set origin...")

    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        """Cache the latest attitude. Gazebo odometry is already in the
        world ENU frame, so its orientation is arena-aligned as-is."""
        self._orientation = msg.pose.pose.orientation

    # ------------------------------------------------------------------
    def _on_navsat(self, msg: NavSatFix):
        """Project a GPS fix into the ENU arena frame and publish pose.

        Driven by the navsat callback because position is the new
        information; orientation is whatever odometry last gave us.
        """
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude

        # Reject garbage (no fix / NaN) before it poisons the origin.
        if not (math.isfinite(lat) and math.isfinite(lon)):
            self.get_logger().warn(
                "navsat lat/lon not finite; skipping", throttle_duration_sec=5.0)
            return
        if self.require_fix and msg.status.status == NavSatStatus.STATUS_NO_FIX:
            self.get_logger().warn(
                "navsat reports NO_FIX; waiting for a fix",
                throttle_duration_sec=5.0)
            return

        # Latch the origin on the first usable fix.
        if self._lat0 is None:
            self._lat0 = lat
            self._lon0 = lon
            self._alt0 = alt
            self._cos_lat0 = math.cos(math.radians(lat))
            self.get_logger().info(
                f"Arena origin set: lat={lat:.7f} lon={lon:.7f} "
                f"alt={alt:.2f} (ENU origin at this point)")

        # Equirectangular projection: small-angle flat-earth approximation.
        #   East  = R * cos(lat0) * (lon - lon0)   [radians]
        #   North = R * (lat - lat0)               [radians]
        #   Up    = alt - alt0
        east = WGS84_A * self._cos_lat0 * math.radians(lon - self._lon0)
        north = WGS84_A * math.radians(lat - self._lat0)
        up = alt - self._alt0

        pose = PoseStamped()
        # Use the GPS message stamp so downstream time alignment is honest
        # (respects sim time when use_sim_time is set).
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.arena_frame
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = up

        if self._orientation is not None:
            pose.pose.orientation = self._orientation
        else:
            # Identity quaternion until the first odometry message lands.
            pose.pose.orientation.w = 1.0
            self.get_logger().warn(
                "No odometry yet; publishing identity orientation",
                throttle_duration_sec=5.0)

        self._pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy may already be shut down on Ctrl-C; guard the call.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
