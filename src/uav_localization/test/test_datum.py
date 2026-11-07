"""localization_node datum tests.

These instantiate the real node (needs a sourced ROS 2 environment) but never
spin it: we call the navsat callback directly and capture what it would have
published.
"""

import math

import pytest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from uav_localization.localization_node import LocalizationNode, WGS84_A

# Measured from /iris_1/navsat at world (0, 0) in the runway world.
DATUM = (-35.3632621, 149.1652374, 10.19)


class _Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _datum_args(datum):
    if datum is None:
        return []
    lat, lon, alt = datum
    return ['--ros-args', '-p', f'datum_lat:={lat}', '-p', f'datum_lon:={lon}',
            '-p', f'datum_alt:={alt}']


@pytest.fixture
def make_nodes():
    """Factory: make_nodes(n, datum=None) -> n nodes sharing one context."""
    nodes = []

    def factory(n, datum=None, extra_args=()):
        rclpy.init(args=_datum_args(datum) + list(extra_args))
        for _ in range(n):
            node = LocalizationNode()
            node._pose_pub = _Recorder()
            nodes.append(node)
        return nodes

    yield factory
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _fix(lat, lon, alt):
    msg = NavSatFix()
    msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
    msg.status.status = NavSatStatus.STATUS_FIX
    return msg


def _last_xyz(node):
    p = node._pose_pub.msgs[-1].pose.position
    return p.x, p.y, p.z


def _north_offset(lat, metres):
    """Latitude `metres` north of `lat` under the node's own projection."""
    return lat + math.degrees(metres / WGS84_A)


def test_unset_datum_first_fix_is_origin(make_nodes):
    """CRITICAL regression: with no datum params, the first fix reports (0,0,0)."""
    (node,) = make_nodes(1)
    node._on_navsat(_fix(*DATUM))
    assert _last_xyz(node) == (0.0, 0.0, 0.0)
    # And a later fix is measured from that first fix, as before.
    node._on_navsat(_fix(_north_offset(DATUM[0], 3.0), DATUM[1], DATUM[2] + 2.0))
    x, y, z = _last_xyz(node)
    assert (x, y, z) == pytest.approx((0.0, 3.0, 2.0), abs=1e-6)


def test_two_nodes_one_datum_report_one_frame(make_nodes):
    """Different first fixes, one datum: the drones are 5 m apart, not both (0,0)."""
    a, b = make_nodes(2, datum=DATUM)
    lat, lon, alt = DATUM
    a._on_navsat(_fix(lat, lon, alt))
    b._on_navsat(_fix(_north_offset(lat, 5.0), lon, alt))
    xa, ya, _ = _last_xyz(a)
    xb, yb, _ = _last_xyz(b)
    assert (xa, ya) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert (xb, yb) == pytest.approx((0.0, 5.0), abs=1e-6)
    assert math.hypot(xb - xa, yb - ya) == pytest.approx(5.0, abs=1e-6)


def test_datum_applies_to_altitude(make_nodes):
    (node,) = make_nodes(1, datum=DATUM)
    node._on_navsat(_fix(DATUM[0], DATUM[1], DATUM[2] + 0.195))
    assert _last_xyz(node)[2] == pytest.approx(0.195, abs=1e-6)


def test_datum_still_waits_for_a_valid_fix(make_nodes):
    (node,) = make_nodes(1, datum=DATUM)
    bad = _fix(*DATUM)
    bad.status.status = NavSatStatus.STATUS_NO_FIX
    node._on_navsat(bad)
    assert node._pose_pub.msgs == []


def test_partial_datum_refuses_to_start():
    rclpy.init(args=['--ros-args', '-p', 'datum_lat:=-35.0'])
    try:
        with pytest.raises(ValueError, match='set together'):
            LocalizationNode()
    finally:
        rclpy.shutdown()
