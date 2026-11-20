"""Tests for the mission state machine (core/mission.py). No ROS needed."""

import pytest

from uav_planning.core.mission import (
    DONE, SEARCHING, TIMED_OUT, WAITING, Mission, MissionParams, open_cells)

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
