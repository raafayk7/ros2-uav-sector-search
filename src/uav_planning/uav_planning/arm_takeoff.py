"""arm_takeoff: GUIDED, arm, take off to a layer through /ap/vN/*, then exit.

One per drone, run once at mission start (D15). It exists so that
follower_planner_node never calls a service: /ap/vN/* services hang under the
container's default CycloneDDS, so this process alone runs on Fast DDS. The
launch sets RMW_IMPLEMENTATION=rmw_fastrtps_cpp for it; we warn if it was not.

Each step is retried, because SITL rejects arming for its first ~30-60 s while
the EKF and GPS settle, and a takeoff right after arming can also be refused.
Exit code 0 once the drone has climbed to within ``tolerance`` of ``alt``;
1 on timeout.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped

COPTER_GUIDED = 4  # https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE


class ArmTakeoff(Node):

    def __init__(self):
        super().__init__('arm_takeoff')
        self.declare_parameter('sysid', 1)
        self.declare_parameter('alt', 10.0)        # metres above home
        self.declare_parameter('tolerance', 0.5)   # D26 layer tolerance
        self.declare_parameter('timeout', 180.0)   # wall seconds, whole run
        self.declare_parameter('retry_period', 2.0)

        g = self.get_parameter
        n = int(g('sysid').value)
        self.alt = float(g('alt').value)
        self.tolerance = float(g('tolerance').value)
        self.deadline = time.monotonic() + float(g('timeout').value)
        self.retry = float(g('retry_period').value)

        rmw = os.environ.get('RMW_IMPLEMENTATION', '')
        if rmw != 'rmw_fastrtps_cpp':
            self.get_logger().warn(
                f"RMW_IMPLEMENTATION='{rmw}': /ap services are known to hang "
                "unless this process runs on rmw_fastrtps_cpp")

        self.mode = self.create_client(ModeSwitch, f'/ap/v{n}/mode_switch')
        self.arm = self.create_client(ArmMotors, f'/ap/v{n}/arm_motors')
        self.takeoff = self.create_client(Takeoff, f'/ap/v{n}/experimental/takeoff')
        # Height above the EKF origin (home), ENU z up. Sensor QoS accepts
        # whatever AP_DDS offers.
        self.z = None
        self.create_subscription(
            PoseStamped, f'/ap/v{n}/pose/filtered',
            lambda m: setattr(self, 'z', m.pose.position.z),
            qos_profile_sensor_data)
        self.ns = f'/ap/v{n}'

    def _left(self):
        return self.deadline - time.monotonic()

    def _call_until(self, client, request, ok, what):
        """Call until ``ok(response)``, retrying every retry_period."""
        while self._left() > 0:
            if not client.wait_for_service(timeout_sec=min(self.retry, self._left())):
                self.get_logger().info(
                    f"waiting for {client.srv_name}", throttle_duration_sec=10.0)
                continue
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.retry * 2)
            if future.done() and future.result() is not None and ok(future.result()):
                self.get_logger().info(f"{what}: ok")
                return True
            self.get_logger().info(
                f"{what}: not yet ({future.result() if future.done() else 'no reply'})",
                throttle_duration_sec=10.0)
            self._sleep(self.retry)
        self.get_logger().error(f"{what}: timed out")
        return False

    def _sleep(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self) -> bool:
        if not self._call_until(
                self.mode, ModeSwitch.Request(mode=COPTER_GUIDED),
                lambda r: r.status and r.curr_mode == COPTER_GUIDED, 'GUIDED'):
            return False
        if not self._call_until(
                self.arm, ArmMotors.Request(arm=True), lambda r: r.result, 'arm'):
            return False
        if not self._call_until(
                self.takeoff, Takeoff.Request(alt=self.alt), lambda r: r.status,
                f'takeoff to {self.alt} m'):
            return False
        while self._left() > 0:
            self._sleep(0.5)
            if self.z is not None and abs(self.z - self.alt) <= self.tolerance:
                self.get_logger().info(
                    f"climbed: {self.ns} z={self.z:.2f} m, target {self.alt} m; exiting")
                return True
            self.get_logger().info(
                f"climbing: z={'?' if self.z is None else f'{self.z:.1f}'} m",
                throttle_duration_sec=5.0)
        self.get_logger().error(f"climb timed out at z={self.z}")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ArmTakeoff()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
