"""mission_prior_node: the operator's clue -> one starting belief (D44-D48).

Runs once at launch. It asks the provider (replay by default, so no key), turns
the answer into a starting belief with uav_mapping.core.prior.make_prior, logs
every warning, publishes the result once, and stays up so the latched message
reaches a late subscriber.

  out  /mission_prior   String, JSON {clue, provider, failed, warnings,
                        regions, start}, RELIABLE + TRANSIENT_LOCAL

Hard rule: this node publishes the prior and nothing else. It has no other
publisher, no service client and no pose or command message, and the language
model's text goes no further than uav_mapping.core.prior (a test checks this).

Whatever goes wrong (no recording, no key, invalid output twice, a world file
that does not match the grid) the node still publishes, a uniform prior marked
`failed` with the reason, so belief_map_node starts at once instead of waiting
out its timeout. A uniform map is exactly the flight without a clue.
"""

import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String

from uav_mapping.core.geometry import Grid
from uav_mapping.core.landmarks import load_world
from uav_mapping.core.live import LiveProvider
from uav_mapping.core.prior import (ALPHA, Prior, live_recording, load_replay,
                                    make_prior, prior_message, save_recording,
                                    uniform_prior)


class MissionPriorNode(Node):

    def __init__(self):
        super().__init__('mission_prior_node')
        d = self.declare_parameter
        d('clue', '')
        d('world', 'runway')
        d('worlds_dir', '/workspaces/multi-agent-uavs/worlds')
        d('provider', 'replay')              # 'replay' | 'live'
        d('recordings', '')                  # default: <worlds_dir>/<world>/recordings.json
        d('alpha', ALPHA)
        d('llm_model', '')                   # live only
        d('record_to', '')                   # live only: freeze the answer here
        d('prior_topic', '/mission_prior')
        # Grid: must match belief_map_node's (D39 defaults).
        d('resolution', 5.0)
        d('grid_width', 8)
        d('grid_height', 8)
        d('origin_x', 0.0)
        d('origin_y', 0.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.clue = str(g('clue'))
        self.grid = Grid(int(g('grid_width')), int(g('grid_height')),
                         float(g('resolution')), float(g('origin_x')),
                         float(g('origin_y')))
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(String, g('prior_topic'), latched)

        prior = self._prior(g)
        for w in prior.warnings:
            self.get_logger().warn(w)
        self.pub.publish(String(data=prior_message(prior, self.clue,
                                                   str(g('provider')))))
        raised = int((prior.start > 0.5).sum())
        self.get_logger().info(
            f"prior published | clue {self.clue!r} | provider {g('provider')} | "
            f"{'FAILED, uniform' if prior.failed else f'{len(prior.regions)} regions'}"
            f" | {raised} of {self.grid.width * self.grid.height} cells raised")

    def _prior(self, g) -> Prior:
        try:
            world_dir = Path(g('worlds_dir')) / g('world')
            world = load_world(world_dir / 'landmarks.yaml')
            kind = g('provider')
            if kind == 'replay':
                path = g('recordings') or str(world_dir / 'recordings.json')
                provider = load_replay(path)
            elif kind == 'live':
                provider = LiveProvider(g('llm_model'))
            else:
                raise ValueError(f"provider must be 'replay' or 'live', got {kind!r}")
            prior = make_prior(self.clue, world, self.grid, provider,
                               float(g('alpha')))
            if kind == 'live':
                for text, usage in zip(provider.outputs, provider.usage):
                    self.get_logger().info(
                        f"{provider.model} answered ({usage} tokens): {text}")
            if kind == 'live' and not prior.failed and g('record_to'):
                rec = live_recording(
                    world, self.clue, g('llm_model'), provider.outputs[-1],
                    datetime.date.today().isoformat())
                replaced = save_recording(g('record_to'), rec)
                self.get_logger().info(
                    f"{'replaced' if replaced else 'saved'} the live recording "
                    f"in {g('record_to')}")
            return prior
        except Exception as e:                       # noqa: BLE001 - see docstring
            return uniform_prior(self.grid, [f"{type(e).__name__}: {e}"])


def main(args=None):
    rclpy.init(args=args)
    node = MissionPriorNode()
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
