"""Tests for the node's own rules (core/follower.py) and its command message.
No ROS needed, except the GlobalPosition test, which skips without it."""

import math

import pytest

from uav_planning.core.follower import (
    ARRIVED, ASCENDING, HOLDING, follower_tick, hold_command, make_peer,
    parse_peaks, parse_peers, stub_map_data)
from uav_planning.core.planner import (
    GridSpec, Peer, PlannerParams, Sector, values_from_occupancy)

G = GridSpec(8, 8, 5.0)
P = PlannerParams()
NOW = 100.0
LAYER = 10.0
PEAK = Sector(3, 4)                       # centre (17.5, 22.5)
VALUES = values_from_occupancy(stub_map_data(8, 8, 5, [(3, 4, 90)]), G)


def tick(pose, goal=None, peers=(), values=VALUES, grid=G, enabled=True):
    return follower_tick(grid, values, 1, pose, LAYER, goal, list(peers), NOW,
                         P, 0.5, enabled)


# ---- Gates: nothing is sent until everything is in place ------------------

def test_no_map_means_no_goal_and_no_command():
    out = tick((0.0, 0.0, LAYER), values=None, grid=None)
    assert out == (HOLDING, None, None, 'no-map')


def test_no_pose_holds():
    assert tick(None) == (HOLDING, None, None, 'no-pose')


def test_disabled_holds_even_with_map_and_pose():
    assert tick((0.0, 0.0, LAYER), enabled=False) == (HOLDING, None, None, 'disabled')


@pytest.mark.parametrize('z', [0.0, 9.49, 10.51, 14.0])
def test_holding_off_layer_before_first_goal(z):
    """D26: no horizontal goal until within 0.5 m of the layer, either side."""
    assert tick((0.0, 0.0, z)) == (HOLDING, None, None, 'below-layer')


@pytest.mark.parametrize('z', [9.5, 10.0, 10.5])
def test_on_layer_flies_to_best_sector(z):
    out = tick((0.0, 0.0, z))
    assert out.status == ASCENDING and out.goal == PEAK
    assert out.command == G.sector_centre(PEAK) == (17.5, 22.5)


# ---- In flight and arrival -----------------------------------------------

def test_in_flight_resends_the_same_goal():
    out = tick((8.0, 10.0, LAYER), goal=PEAK)
    assert (out.status, out.goal, out.command, out.reason) == \
        (ASCENDING, PEAK, (17.5, 22.5), 'flying')


def test_off_layer_mid_flight_keeps_goal():
    """The gate only holds back the first goal; a flying drone keeps going."""
    out = tick((8.0, 10.0, 8.0), goal=PEAK)
    assert out.status == ASCENDING and out.goal == PEAK


def test_arrived_needs_horizontal_and_altitude():
    near = (17.5 + 1.9, 22.5, LAYER)
    assert tick(near, goal=PEAK) == (ARRIVED, PEAK, None, 'hold')
    # Inside 2 m but below layer: keep commanding, not ARRIVED (D26).
    low = tick((17.5, 22.5, 9.0), goal=PEAK)
    assert low.status == ASCENDING and low.command == (17.5, 22.5)
    # On layer but outside 2 m: still flying.
    far = tick((17.5 + 2.1, 22.5, LAYER), goal=PEAK)
    assert far.status == ASCENDING


def test_arrived_sends_no_command():
    assert tick((17.5, 22.5, LAYER), goal=PEAK).command is None


def test_blocked_goal_redirects_in_flight():
    """A lower-sysid peer announcing a goal in our goal's zone moves us on."""
    values = values_from_occupancy(
        stub_map_data(8, 8, 5, [(3, 4, 90), (6, 1, 80)]), G)
    peer = Peer(0, pose=(30.0, 30.0), pose_stamp=NOW, goal=PEAK, goal_stamp=NOW)
    out = follower_tick(G, values, 1, (8.0, 10.0, LAYER), LAYER, PEAK, [peer],
                        NOW, P)
    assert out.goal == Sector(6, 1) and out.status == ASCENDING


def test_flat_map_holds_where_it_is():
    """D18: a flat map holds the sector under the drone (flies to its centre)."""
    flat = values_from_occupancy(stub_map_data(8, 8, 5, []), G)
    out = tick((1.0, 1.0, LAYER), values=flat)
    assert out.goal == Sector(0, 0) and out.reason == 'hold'


# ---- Disable ------------------------------------------------------------

def test_hold_command_is_current_xy():
    assert hold_command((12.3, -4.5, 10.2)) == (12.3, -4.5)
    assert hold_command(None) is None


# ---- Peers from raw messages ----------------------------------------------

def test_make_peer_recovers_goal_sector_from_centre():
    p = make_peer(G, 2, (1.0, 2.0), 99.0, (17.5, 22.5), 98.0)
    assert p == Peer(2, (1.0, 2.0), 99.0, PEAK, 98.0)


def test_make_peer_without_map_or_off_grid_has_no_goal():
    assert make_peer(None, 2, (1.0, 2.0), 99.0, (17.5, 22.5), 98.0).goal is None
    off = make_peer(G, 2, None, 99.0, (-3.0, 2.0), 98.0)
    assert off.goal is None and off.goal_stamp == -math.inf
    assert off.pose is None and off.pose_stamp == -math.inf


# ---- Launch strings and the stub map --------------------------------------

def test_parse_peers():
    assert parse_peers('') == []
    assert parse_peers(' iris_2:2 , iris_3:3,') == [('iris_2', 2), ('iris_3', 3)]
    with pytest.raises(ValueError):
        parse_peers('iris_2')


def test_parse_peaks_and_stub_map_layout():
    peaks = parse_peaks('3:4:90,6:1:80')
    assert peaks == [(3, 4, 90), (6, 1, 80)]
    data = stub_map_data(8, 8, 5, peaks)
    assert len(data) == 64
    assert data[4 * 8 + 3] == 90 and data[1 * 8 + 6] == 80
    assert data.count(5) == 62
    with pytest.raises(ValueError):
        stub_map_data(8, 8, 5, [(8, 0, 50)])
    with pytest.raises(ValueError):
        stub_map_data(8, 8, 101, [])


def test_default_stub_peaks_are_far_enough_apart_for_three_followers():
    """D24 zones are 3x3, so peaks must be >= 2 apart for 3 to be held."""
    from uav_planning.core.planner import chebyshev
    peaks = [Sector(c, r) for c, r, _ in parse_peaks('3:4:90,6:1:80,1:6:70')]
    assert all(chebyshev(a, b) >= 2 for i, a in enumerate(peaks)
               for b in peaks[i + 1:])


# ---- The command message (needs ROS) --------------------------------------

def test_gps_command_frame_and_altitude_mode():
    pytest.importorskip('rclpy')
    pytest.importorskip('ardupilot_msgs')
    from ardupilot_msgs.msg import GlobalPosition
    from uav_planning.follower_planner_node import build_gps_command

    msg = build_gps_command(-35.36, 149.16, 10.0)
    assert msg.header.frame_id == 'map'          # anything else is dropped
    assert msg.coordinate_frame == GlobalPosition.FRAME_GLOBAL_REL_ALT
    assert (msg.latitude, msg.longitude, msg.altitude) == (-35.36, 149.16, 10.0)
    # Position honoured, everything else ignored.
    for flag in ('IGNORE_LATITUDE', 'IGNORE_LONGITUDE', 'IGNORE_ALTITUDE'):
        assert not msg.type_mask & getattr(GlobalPosition, flag)
    for flag in ('IGNORE_VX', 'IGNORE_VY', 'IGNORE_VZ', 'IGNORE_YAW'):
        assert msg.type_mask & getattr(GlobalPosition, flag)
