"""Tests for the mission state machine (core/mission.py). No ROS needed,
except the two mission_node parameter tests at the end."""

import pytest

from uav_planning.core.mission import (
    DONE, SEARCHING, TIMED_OUT, WAITING, Mission, MissionParams, SimClock,
    assign_returns, cell_centre, open_cells, parse_finds, parse_names,
    parse_pairs)

FOLLOWERS = ['iris_2', 'iris_3', 'iris_4']
W = 8                                  # 8x8 arena (D39)
QUIET = [3] * 64                       # every empty cell at 3 %, as live
ALL_STATUS = {f: 'HOLDING' for f in FOLLOWERS}


def quiet_with(cells):
    """QUIET with some cells set: {(col, row): value}."""
    data = list(QUIET)
    for (c, r), v in cells.items():
        data[r * W + c] = v
    return data


def started(params=MissionParams()):
    """A mission ticked once with every input present, so in SEARCHING at
    wall 100 and sim 50."""
    m = Mission(FOLLOWERS, params)
    out = m.tick(100.0, 50.0, 'WAITING', QUIET, W, [], ALL_STATUS)
    assert out.state == SEARCHING
    return m


def tick(m, wall, sim=None, sweep='COMPLETE', occ=QUIET, finds=(),
         status=ALL_STATUS):
    return m.tick(wall, sim, sweep, occ, W, list(finds), status)


# ---- open_cells ------------------------------------------------------------

def test_open_cells_counts_unknown_as_half():
    occ = quiet_with({(2, 1): -1})
    assert open_cells(occ, W, []) == [(2, 1)]


def test_open_cells_threshold_is_inclusive():
    occ = quiet_with({(0, 0): 50, (1, 0): 49})
    assert open_cells(occ, W, []) == [(0, 0)]


def test_open_cells_skips_confirmed():
    occ = quiet_with({(4, 5): 97})
    assert open_cells(occ, W, [(4, 5)]) == []
    assert open_cells(occ, W, []) == [(4, 5)]


# ---- WAITING -----------------------------------------------------------------

def test_needs_a_follower():
    with pytest.raises(ValueError):
        Mission([])


def test_waits_for_every_input_with_planners_disabled():
    m = Mission(FOLLOWERS)
    out = m.tick(0.0, None, None, None, W, [], {'iris_2': 'HOLDING'})
    assert out.state == WAITING and not out.enable
    assert out.reason == 'waiting for sweep_status, heatmap, iris_3, iris_4'
    out = m.tick(1.0, None, 'WAITING', QUIET, W, [], {'iris_2': 'HOLDING'})
    assert out.state == WAITING and 'iris_3, iris_4' in out.reason


def test_starts_searching_and_enables_when_all_seen():
    m = Mission(FOLLOWERS)
    out = m.tick(100.0, 50.0, 'WAITING', QUIET, W, [], ALL_STATUS)
    assert (out.state, out.enable, out.changed) == (SEARCHING, True, True)
    assert (m.t0_wall, m.t0_sim) == (100.0, 50.0)


# ---- End rule (D14) --------------------------------------------------------

def test_done_after_hold_once_sweep_complete_and_quiet():
    m = started()
    out = tick(m, 110.0)
    assert out.state == SEARCHING and out.enable and out.reason == 'quiet 0 s'
    assert tick(m, 139.0).state == SEARCHING
    out = tick(m, 140.0)
    assert (out.state, out.enable, out.changed) == (DONE, False, True)


def test_not_done_while_sweeping_even_if_quiet():
    m = started()
    for t in range(101, 200):
        out = tick(m, float(t), sweep='SWEEPING')
        assert out.state == SEARCHING
    assert out.reason == 'sweep not complete'


def test_an_open_cell_restarts_the_hold():
    m = started()
    tick(m, 110.0)
    out = tick(m, 130.0, occ=quiet_with({(3, 3): 62}))  # a false alarm
    assert out.reason == '1 cells open'
    assert tick(m, 131.0).reason == 'quiet 0 s'
    assert tick(m, 160.0).state == SEARCHING
    assert tick(m, 161.0).state == DONE


def test_unknown_cell_blocks_the_end():
    m = started()
    occ = quiet_with({(7, 7): -1})
    for t in range(101, 300):
        assert tick(m, float(t), occ=occ).state == SEARCHING


def test_confirmed_cell_does_not_block_the_end():
    # The belief map publishes a confirmed sector as 0 (D13); even if it did
    # not, the mission's own find list keeps it out of the rule.
    m = started()
    occ = quiet_with({(4, 5): 97})
    tick(m, 110.0, occ=occ, finds=[((4, 5), 'iris_2')])
    assert tick(m, 140.0, occ=occ, finds=[((4, 5), 'iris_2')]).state == DONE


def test_missing_heatmap_counts_as_not_quiet():
    m = started()
    out = m.tick(110.0, None, 'COMPLETE', None, W, [], ALL_STATUS)
    assert (out.state, out.reason) == (SEARCHING, 'no heatmap')
    assert m.quiet_since is None


# ---- Timeout (D41) ---------------------------------------------------------

def test_times_out_on_sim_time():
    m = started(MissionParams(timeout=600.0))
    occ = quiet_with({(7, 7): 80})           # never quiet
    assert tick(m, 2000.0, sim=649.0, occ=occ).state == SEARCHING   # 599 sim-s
    out = tick(m, 2001.0, sim=650.0, occ=occ)
    assert (out.state, out.enable) == (TIMED_OUT, False)
    assert m.open_at_end == [(7, 7)]


def test_without_sim_time_wall_time_stands_in():
    m = Mission(FOLLOWERS, MissionParams(timeout=600.0))
    m.tick(100.0, None, 'WAITING', QUIET, W, [], ALL_STATUS)
    occ = quiet_with({(7, 7): 80})
    assert tick(m, 699.0, occ=occ).state == SEARCHING
    assert tick(m, 700.0, occ=occ).state == TIMED_OUT


def test_late_sim_time_counts_the_gap_as_wall_time():
    m = Mission(FOLLOWERS, MissionParams(timeout=600.0))
    m.tick(100.0, None, 'WAITING', QUIET, W, [], ALL_STATUS)
    occ = quiet_with({(7, 7): 80})
    tick(m, 110.0, sim=5.0, occ=occ)           # anchor: t0_sim = 5 - 10
    assert m.t0_sim == -5.0
    assert tick(m, 900.0, sim=594.0, occ=occ).state == SEARCHING
    assert tick(m, 901.0, sim=595.0, occ=occ).state == TIMED_OUT


def test_done_wins_over_timeout_on_the_same_tick():
    m = started(MissionParams(hold=30.0, timeout=40.0))
    tick(m, 110.0, sim=60.0)
    assert tick(m, 140.0, sim=90.0).state == DONE


def test_ended_mission_stays_ended_and_disabled():
    m = started()
    tick(m, 110.0)
    tick(m, 140.0)
    out = tick(m, 141.0, occ=quiet_with({(0, 0): 99}))
    assert (out.state, out.enable, out.changed) == (DONE, False, False)


# ---- Report ----------------------------------------------------------------

def test_report_lists_finds_in_order_with_times():
    m = started()
    tick(m, 120.0, sim=60.0, sweep='SWEEPING')
    tick(m, 125.0, sim=63.0, sweep='SWEEPING',
         finds=[((6, 1), 'iris_4')])
    tick(m, 131.0, sim=66.0, sweep='SWEEPING',
         finds=[((6, 1), 'iris_4'), ((4, 5), 'iris_2')])
    tick(m, 170.0, sim=88.0, finds=[((6, 1), 'iris_4'), ((4, 5), 'iris_2')])
    out = tick(m, 200.0, sim=104.0, status={'iris_2': 'ARRIVED',
                                           'iris_3': 'ARRIVED'})
    assert out.state == DONE
    r = m.report()
    assert r['state'] == DONE
    assert r['wall_s'] == 100.0 and r['sim_s'] == 54.0
    assert r['sweep_started_s'] == 20.0 and r['sweep_complete_s'] == 70.0
    assert r['finds'] == [
        {'cell': [6, 1], 'drone': 'iris_4', 'wall_s': 25.0, 'sim_s': 13.0},
        {'cell': [4, 5], 'drone': 'iris_2', 'wall_s': 31.0, 'sim_s': 16.0}]
    assert r['open_cells'] == []
    assert r['planner_status'] == {'iris_2': 'ARRIVED', 'iris_3': 'ARRIVED',
                                   'iris_4': None}


def test_a_find_is_logged_once_with_its_first_drone():
    m = started()
    tick(m, 110.0, sweep='SWEEPING', finds=[((1, 2), 'iris_3')])
    tick(m, 111.0, sweep='SWEEPING', finds=[((1, 2), 'iris_9')])
    assert [f['drone'] for f in m.report()['finds']] == ['iris_3']


# ---- Node helpers (parse_names, parse_finds, SimClock) ---------------------

def test_parse_names_drops_blanks_and_spaces():
    assert parse_names(' iris_2, iris_3,,iris_4 ') == ['iris_2', 'iris_3', 'iris_4']
    assert parse_names('') == []


def test_parse_finds_reads_belief_map_json():
    text = ('[{"cell": [6, 1], "drone": "iris_4", "t": 15.5},'
            ' {"cell": [4, 5], "drone": "iris_2", "t": 22.8}]')
    assert parse_finds(text) == [((6, 1), 'iris_4'), ((4, 5), 'iris_2')]
    assert parse_finds('[]') == []


@pytest.mark.parametrize('text', ['not json', '{"cell": [1, 2]}', '[{"cell": [1]}]',
                                  '[{"drone": "iris_2"}]'])
def test_parse_finds_rejects_anything_else(text):
    with pytest.raises(ValueError):
        parse_finds(text)


def test_sim_clock_carries_the_latest_stamp_forward_by_wall_time():
    c = SimClock()
    assert c.now(10.0) is None
    c.update(50.0, 10.0)
    assert c.now(12.5) == 52.5
    c.update(49.0, 13.0)            # an older stamp from another follower
    assert c.now(13.0) == 53.0
    c.update(51.0, 13.0)            # sim behind wall (RTF < 1): newest wins
    assert c.now(14.0) == 52.0


# ---- mission_node parameters (needs ROS, no sim) ---------------------------

def test_node_resolves_topics_per_follower():
    pytest.importorskip('rclpy')
    import rclpy
    from uav_planning.mission_node import MissionNode

    rclpy.init(args=['--ros-args', '-p', 'followers:=iris_2, iris_3',
                     '-p', 'leader:=iris_1', '-p', 'hold:=12.0',
                     '-p', 'timeout:=60.0'])
    try:
        node = MissionNode()
        assert node.followers == ['iris_2', 'iris_3']
        assert node.mission.params == MissionParams(0.5, 12.0, 60.0)
        assert node.topics['sweep_status'] == '/iris_1/sweep_status'
        assert node.topics['followers']['iris_3'] == {
            'status_topic': '/iris_3/planner_status',
            'pose_topic': '/iris_3/pose',
            'enable_topic': '/iris_3/planner_enable'}
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_refuses_an_empty_follower_list():
    pytest.importorskip('rclpy')
    import rclpy
    from uav_planning.mission_node import MissionNode

    rclpy.init(args=['--ros-args', '-p', 'followers:=" "'])
    try:
        with pytest.raises(ValueError):
            MissionNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


# ---- Return to find ---------------------------------------------------------

FOLLOWERS = ['iris_2', 'iris_3', 'iris_4']


def test_cell_centre_uses_resolution_and_origin():
    assert cell_centre((1, 2)) == (7.5, 12.5)
    assert cell_centre((0, 0), 2.0, (10.0, -4.0)) == (11.0, -3.0)


def test_each_finder_takes_its_own_find():
    finds = [((4, 5), 'iris_4'), ((1, 2), 'iris_3'), ((6, 1), 'iris_2')]
    poses = {f: (0.0, 0.0) for f in FOLLOWERS}
    assert assign_returns(finds, poses, FOLLOWERS) == {
        'iris_4': (4, 5), 'iris_3': (1, 2), 'iris_2': (6, 1)}


def test_a_drone_that_found_two_keeps_the_earlier():
    finds = [((1, 2), 'iris_3'), ((6, 1), 'iris_3')]
    # iris_4 is the nearer of the two others to (6, 1) = (32.5, 7.5).
    poses = {'iris_2': (0.0, 0.0), 'iris_3': (7.5, 12.5), 'iris_4': (30.0, 5.0)}
    plan = assign_returns(finds, poses, FOLLOWERS)
    assert plan == {'iris_3': (1, 2), 'iris_4': (6, 1)}


def test_more_finds_than_followers_leaves_the_extras_out():
    finds = [((0, 0), 'iris_2'), ((1, 0), 'iris_2'), ((2, 0), 'iris_2'),
             ((3, 0), 'iris_2'), ((4, 0), 'iris_2')]
    poses = {f: (0.0, 0.0) for f in FOLLOWERS}
    plan = assign_returns(finds, poses, FOLLOWERS)
    assert len(plan) == 3 and plan['iris_2'] == (0, 0)
    assert len(set(plan.values())) == 3          # no find twice


def test_a_follower_without_a_pose_is_not_sent():
    finds = [((1, 2), 'iris_3'), ((6, 1), 'iris_3')]
    assert assign_returns(finds, {'iris_3': (0.0, 0.0)}, FOLLOWERS) == {
        'iris_3': (1, 2)}


def test_a_find_by_the_leader_goes_to_the_nearest_follower():
    finds = [((6, 1), 'iris_1')]
    poses = {'iris_2': (0.0, 0.0), 'iris_3': (31.0, 8.0), 'iris_4': (20.0, 20.0)}
    assert assign_returns(finds, poses, FOLLOWERS) == {'iris_3': (6, 1)}


def test_no_finds_means_no_returns():
    assert assign_returns([], {'iris_2': (0.0, 0.0)}, FOLLOWERS) == {}


def test_parse_pairs():
    assert parse_pairs('iris_2:10, iris_3:12.5,') == {'iris_2': 10.0, 'iris_3': 12.5}
    assert parse_pairs('a:2,b:3', int) == {'a': 2, 'b': 3}
    assert parse_pairs('') == {}
    with pytest.raises(ValueError):
        parse_pairs('iris_2')


# ---- Return to find in the node ---------------------------------------------

RETURN_ARGS = ['--ros-args', '-p', 'followers:=iris_2,iris_3,iris_4',
               '-p', 'return_to_finds:=true',
               '-p', 'sysids:=iris_2:2,iris_3:3,iris_4:4',
               '-p', 'layers:=iris_2:10,iris_3:12,iris_4:14',
               '-p', 'datum_lat:=-35.3632621', '-p', 'datum_lon:=149.1652374',
               '-p', 'datum_alt:=10.195']


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def _return_node():
    from uav_planning.core.mission import Find
    from uav_planning.mission_node import MissionNode
    node = MissionNode()
    node.cmd_pubs = {f: _FakePub() for f in node.followers}
    node.return_pub = _FakePub()
    node.mission.finds = {
        (4, 5): Find((4, 5), 'iris_4', 78.0, None),
        (1, 2): Find((1, 2), 'iris_3', 85.0, None)}
    node.poses = {'iris_2': (27.5, 2.5), 'iris_3': (12.5, 7.5),
                  'iris_4': (27.5, 22.5)}
    return node


def test_return_needs_datum_sysids_and_layers():
    pytest.importorskip('rclpy')
    import rclpy
    from uav_planning.mission_node import MissionNode

    rclpy.init(args=['--ros-args', '-p', 'return_to_finds:=true'])
    try:
        with pytest.raises(ValueError, match='return_to_finds needs'):
            MissionNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_return_sends_each_finder_to_its_find_at_its_layer():
    pytest.importorskip('rclpy')
    import rclpy

    rclpy.init(args=RETURN_ARGS)
    try:
        node = _return_node()
        node._return_tick(100.0)
        assert {f: c for f, (c, _, _) in node.return_plan.items()} == {
            'iris_4': (4, 5), 'iris_3': (1, 2)}
        assert node.cmd_pubs['iris_2'].sent == []          # no find, no command
        for f, layer in (('iris_3', 12.0), ('iris_4', 14.0)):
            (msg,) = node.cmd_pubs[f].sent
            assert msg.header.frame_id == 'map' and msg.altitude == layer
        assert node.return_status == 'RETURNING'
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_return_ends_at_finds_once_all_are_within_the_radius():
    pytest.importorskip('rclpy')
    import rclpy

    rclpy.init(args=RETURN_ARGS)
    try:
        node = _return_node()
        node._return_tick(100.0)
        node.poses['iris_4'] = (22.5, 27.5)                # (4, 5) centre
        node.poses['iris_3'] = (7.5, 12.5)                 # (1, 2) centre
        node._return_tick(101.0)
        assert node.return_status == 'AT_FINDS'
        sent = len(node.cmd_pubs['iris_4'].sent)
        node._return_tick(102.0)                           # done: no more commands
        assert len(node.cmd_pubs['iris_4'].sent) == sent
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_return_gives_up_after_the_timeout():
    pytest.importorskip('rclpy')
    import rclpy

    rclpy.init(args=RETURN_ARGS + ['-p', 'return_timeout:=30.0'])
    try:
        node = _return_node()
        node._return_tick(100.0)
        node._return_tick(131.0)
        assert node.return_status == 'TIMED_OUT'
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
