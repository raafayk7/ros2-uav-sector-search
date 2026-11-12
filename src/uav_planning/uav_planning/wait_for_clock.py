"""wait_for_clock: exit 0 once ROS /clock is flowing, 1 if it never does.

The swarm launch's gate (T5). Project nodes run on sim time, and a sim-time
timer that never gets /clock never fires, with no error (docker-notes gotcha
33). swarm.launch.py starts this first and starts the followers only when it
exits 0, so a missing clock fails loudly at launch instead of as a silent
planner.

"Flowing" means sim time has advanced by at least ``min_sim_span`` seconds,
so one stale message does not count. ``timeout`` is wall seconds.
"""

import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from rosgraph_msgs.msg import Clock


class WaitForClock(Node):

    def __init__(self):
        super().__init__('wait_for_clock')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('min_sim_span', 2.0)
        self.declare_parameter('timeout', 180.0)

        g = self.get_parameter
        self.topic = g('clock_topic').value
        self.min_span = float(g('min_sim_span').value)
        self.timeout = float(g('timeout').value)
        self.first = None       # (sim seconds, wall seconds) of first message
        self.last = None
        self.count = 0
        # BEST_EFFORT depth 1 matches any writer, like rclpy's own time source.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Clock, self.topic, self._on_clock, qos)

    def _on_clock(self, msg: Clock):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.first is None:
            self.first = (t, time.monotonic())
            self.get_logger().info(f"first {self.topic} message, sim {t:.2f} s")
        self.last = t
        self.count += 1

    def flowing(self) -> bool:
        return self.first is not None and self.last - self.first[0] >= self.min_span

    def run(self) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.flowing():
                sim_span = self.last - self.first[0]
                wall_span = time.monotonic() - self.first[1]
                self.get_logger().info(
                    f"{self.topic} flowing after {time.monotonic() - start:.1f} s "
                    f"wall: {self.count} messages, {self.count / wall_span:.0f} Hz, "
                    f"sim/wall {sim_span / wall_span:.2f}")
                return True
            self.get_logger().info(
                f"waiting for {self.topic} ({self.count} messages so far)",
                throttle_duration_sec=15.0)
        self.get_logger().error(
            f"{self.topic} not flowing after {self.timeout:.0f} s wall "
            f"({self.count} messages): sim-time nodes would never tick")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = WaitForClock()
    ok = False
    try:
        ok = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from launch / kill: rclpy has shut down already.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
