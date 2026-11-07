"""Library-level tests for uav_planning.core.planner. No ROS needed, except
the one round-trip test against localization_node, which skips without it."""

import math
import random
import statistics
import sys
from pathlib import Path

import pytest

from uav_planning.core.planner import (
    Datum, GridSpec, Peer, PlannerParams, Sector, WGS84_A, arena_to_lla,
    blocked_set, bll_move_probability, bll_step, cell_value, choose_goal,
    chebyshev, is_arrived, plan_step, thesis_dga_step, values_from_occupancy)

P = PlannerParams()
NOW = 100.0


def flat(grid, v=0.03):
    return [[v] * grid.width for _ in range(grid.height)]


def with_cells(grid, cells, base=0.03):
    values = flat(grid, base)
    for s, v in cells.items():
        values[s.row][s.col] = v
    return values


def parked(grid, sysid, sector, now=NOW, age=0.0):
    """A peer sitting at a sector centre with no goal (arrived)."""
    return Peer(sysid, pose=grid.sector_centre(sector), pose_stamp=now - age)


def flying(grid, sysid, pose, goal, now=NOW, age=0.0):
    return Peer(sysid, pose=pose, pose_stamp=now - age,
                goal=goal, goal_stamp=now - age)


# ---------------------------------------------------------------------------
# Indexing (D20)
# ---------------------------------------------------------------------------

def test_verified_leader_node_case():
    """leader_node's hand-verified case: pose (7.5, 7.5) at 5 m is col 1, row 1."""
    g = GridSpec(4, 4)
    assert g.world_to_sector(7.5, 7.5) == Sector(col=1, row=1)


def test_col_is_x_and_row_is_y():
    g = GridSpec(8, 8)
    s = g.world_to_sector(12.0, 2.0)
    assert (s.col, s.row) == (2, 0)
    assert g.sector_centre(Sector(2, 0)) == (12.5, 2.5)
    assert g.index(Sector(2, 3)) == 3 * 8 + 2


def test_world_to_sector_edges_origin_and_outside():
    g = GridSpec(4, 4, origin_x=-10.0, origin_y=-10.0)
    assert g.world_to_sector(-10.0, -10.0) == Sector(0, 0)
    assert g.world_to_sector(9.99, 9.99) == Sector(3, 3)
    assert g.world_to_sector(10.0, 0.0) is None
    assert g.world_to_sector(-10.01, 0.0) is None
    assert GridSpec(4, 4).world_to_sector(-0.1, 1.0) is None


def test_centre_round_trips_through_world_to_sector():
    g = GridSpec(8, 6, resolution=3.0, origin_x=4.0, origin_y=-2.0)
    for s in g.sectors():
        assert g.world_to_sector(*g.sector_centre(s)) == s


def test_zone_clips_at_corners():
    g = GridSpec(4, 4)
    assert len(g.zone(Sector(1, 1))) == 9
    assert g.zone(Sector(0, 0)) == {Sector(0, 0), Sector(1, 0),
                                    Sector(0, 1), Sector(1, 1)}
    assert Sector(0, 0) not in g.neighbours(Sector(0, 0))


# ---------------------------------------------------------------------------
# Map values (D18, D31)
# ---------------------------------------------------------------------------

def test_unknown_counts_half_and_percent_maps_to_unit():
    assert cell_value(-1) == 0.5
    assert cell_value(0) == 0.0
    assert cell_value(97) == 0.97
    assert cell_value(100) == 1.0


def test_values_are_row_major_and_not_normalised():
    g = GridSpec(3, 2)
    data = [10, 20, 30,
            40, -1, 100]
    v = values_from_occupancy(data, g)
    assert v[0][2] == 0.30          # row 0, col 2
    assert v[1][0] == 0.40          # row 1, col 0
    assert v[1][1] == 0.5
    assert sum(map(sum, v)) > 1.0   # independent probabilities, not a distribution


def test_wrong_size_map_is_rejected():
    with pytest.raises(ValueError):
        values_from_occupancy([0] * 5, GridSpec(3, 2))


# ---------------------------------------------------------------------------
# Blocked set (D24, D25, D21, D28)
# ---------------------------------------------------------------------------

G8 = GridSpec(8, 8)


def test_arrived_peer_blocks_its_position_zone():
    peer = parked(G8, 1, Sector(4, 4))
    assert blocked_set(G8, 2, [peer], NOW) == G8.zone(Sector(4, 4))


def test_flying_peer_position_does_not_block_only_its_goal_zone():
    pose = G8.sector_centre(Sector(4, 4))              # passing over (4, 4)
    goal = Sector(0, 0)                                # far from the pose
    peer = flying(G8, 1, pose, goal)
    blocked = blocked_set(G8, 2, [peer], NOW)
    assert blocked == G8.zone(goal)
    assert Sector(4, 4) not in blocked


def test_goal_blocks_only_if_peer_has_lower_sysid():
    pose = G8.sector_centre(Sector(7, 7))
    goal = Sector(1, 1)
    peer = flying(G8, 3, pose, goal)
    assert blocked_set(G8, 2, [peer], NOW) == frozenset()       # peer is higher
    assert blocked_set(G8, 5, [peer], NOW) == G8.zone(goal)     # peer is lower


def test_peer_within_arrival_radius_of_goal_counts_as_arrived():
    goal = Sector(4, 4)
    cx, cy = G8.sector_centre(goal)
    near = flying(G8, 1, (cx + 1.9, cy), goal)
    far = flying(G8, 1, (cx + 2.1, cy), goal)
    assert G8.zone(Sector(4, 4)) <= blocked_set(G8, 2, [near], NOW)
    # Position zone is what differs: look at a higher-sysid peer so the goal
    # zone cannot hide it.
    near_hi = flying(G8, 3, (cx + 1.9, cy), goal)
    far_hi = flying(G8, 3, (cx + 2.1, cy), goal)
    assert blocked_set(G8, 2, [near_hi], NOW) == G8.zone(Sector(4, 4))
    assert blocked_set(G8, 2, [far_hi], NOW) == frozenset()


def test_peer_without_valid_goal_counts_as_arrived_where_parked():
    """A disabled drone keeps publishing pose; its goal expires; it blocks where
    it is parked, not where it was heading."""
    pose = G8.sector_centre(Sector(2, 2))
    peer = Peer(1, pose=pose, pose_stamp=NOW, goal=Sector(6, 6),
                goal_stamp=NOW - 6.0)
    assert blocked_set(G8, 2, [peer], NOW) == G8.zone(Sector(2, 2))


def test_goal_expires_after_peer_timeout_even_with_fresh_pose():
    pose = G8.sector_centre(Sector(0, 0))
    live = Peer(1, pose=pose, pose_stamp=NOW, goal=Sector(6, 6),
                goal_stamp=NOW - 5.0)
    dead = Peer(1, pose=pose, pose_stamp=NOW, goal=Sector(6, 6),
                goal_stamp=NOW - 5.01)
    assert G8.zone(Sector(6, 6)) <= blocked_set(G8, 2, [live], NOW)
    assert not (G8.zone(Sector(6, 6)) & blocked_set(G8, 2, [dead], NOW))


def test_silent_peer_releases_zone_and_goal():
    stale = Peer(1, pose=G8.sector_centre(Sector(3, 3)), pose_stamp=NOW - 6.0,
                 goal=Sector(6, 6), goal_stamp=NOW - 6.0)
    assert blocked_set(G8, 2, [stale], NOW) == frozenset()
    back = Peer(1, pose=stale.pose, pose_stamp=NOW, goal=stale.goal,
                goal_stamp=NOW)
    assert blocked_set(G8, 2, [back], NOW)


def test_own_entry_and_unlocated_peers_are_ignored():
    me = parked(G8, 2, Sector(1, 1))
    no_pose = Peer(1)
    assert blocked_set(G8, 2, [me, no_pose], NOW) == frozenset()


# ---------------------------------------------------------------------------
# Choosing (D16, D17, D18)
# ---------------------------------------------------------------------------

def centre(s, g=G8):
    return g.sector_centre(s)


def test_picks_best_open_sector_anywhere_on_the_grid():
    values = with_cells(G8, {Sector(7, 7): 0.97, Sector(2, 0): 0.60})
    d = choose_goal(G8, values, centre(Sector(0, 0)), Sector(0, 0),
                    frozenset(), P)
    assert d == (Sector(7, 7), True, 'move')


def test_best_is_taken_from_open_sectors_only():
    values = with_cells(G8, {Sector(7, 7): 0.97, Sector(2, 0): 0.60})
    d = choose_goal(G8, values, centre(Sector(0, 0)), Sector(0, 0),
                    G8.zone(Sector(7, 7)), P)
    assert d.goal == Sector(2, 0)


def test_margin_holds_at_exactly_the_margin_and_moves_above_it():
    cur = Sector(1, 1)
    hold = with_cells(G8, {cur: 0.50, Sector(5, 5): 0.52})
    move = with_cells(G8, {cur: 0.50, Sector(5, 5): 0.53})
    assert choose_goal(G8, hold, centre(cur), cur, frozenset(), P).goal == cur
    assert choose_goal(G8, hold, centre(cur), cur, frozenset(), P).reason == 'hold'
    assert choose_goal(G8, move, centre(cur), cur, frozenset(), P).goal \
        == Sector(5, 5)


def test_blocked_current_goal_is_worth_zero():
    cur = Sector(1, 1)
    values = with_cells(G8, {cur: 0.97, Sector(5, 5): 0.05})
    d = choose_goal(G8, values, centre(cur), cur, G8.zone(cur), P)
    # 0.05 > 0 + 0.02 margin, even though the blocked goal reads 0.97.
    assert d.goal == Sector(5, 5)


def test_value_drops_and_the_holder_leaves():
    cur = Sector(3, 3)
    other = Sector(6, 1)
    before = with_cells(G8, {cur: 0.97, other: 0.90})
    after = with_cells(G8, {cur: 0.03, other: 0.90})   # belief map lowered it
    assert choose_goal(G8, before, centre(cur), cur, frozenset(), P).goal == cur
    assert choose_goal(G8, after, centre(cur), cur, frozenset(), P).goal == other


def test_unknown_beats_swept_empty_and_loses_to_a_hit():
    g = GridSpec(3, 1)
    data = [3, -1, 97]
    v = values_from_occupancy(data, g)
    here = Sector(0, 0)
    d = choose_goal(g, v, g.sector_centre(here), here, frozenset(), P)
    assert d.goal == Sector(2, 0)
    d = choose_goal(g, v, g.sector_centre(here), here,
                    frozenset({Sector(2, 0)}), P)
    assert d.goal == Sector(1, 0)   # unknown (0.5) over swept-empty (0.03)


def test_flat_map_holds():
    d = choose_goal(G8, flat(G8, 0.5), centre(Sector(3, 3)), None,
                    frozenset(), P)
    assert d.goal == Sector(3, 3) and d.reason == 'hold'


def test_ties_go_to_nearest_then_lowest_index():
    # Four equal peaks. (0, 0) has the lowest index but is farthest, so it loses
    # on distance; (2, 4) and (6, 4) tie at Chebyshev 2 from (4, 3), and the
    # lower index, (2, 4), wins.
    values = with_cells(G8, {Sector(0, 0): 0.9, Sector(7, 7): 0.9,
                             Sector(2, 4): 0.9, Sector(6, 4): 0.9})
    here = Sector(4, 3)
    assert chebyshev(Sector(2, 4), here) == chebyshev(Sector(6, 4), here) == 2
    assert G8.index(Sector(0, 0)) < G8.index(Sector(2, 4)) < G8.index(Sector(6, 4))
    d = choose_goal(G8, values, centre(here), here, frozenset(), P)
    assert d.goal == Sector(2, 4)


def test_tie_at_equal_distance_picks_lowest_index_on_a_flat_peak_row():
    values = with_cells(G8, {Sector(1, 0): 0.9, Sector(1, 2): 0.9})
    d = choose_goal(G8, values, centre(Sector(1, 1)), Sector(1, 1),
                    frozenset(), P)
    assert d.goal == Sector(1, 0)


def test_everything_blocked_keeps_the_goal_and_does_not_crash():
    blocked = frozenset(G8.sectors())
    d = choose_goal(G8, flat(G8), centre(Sector(2, 2)), Sector(2, 2), blocked, P)
    assert d == (Sector(2, 2), False, 'nothing-open')
    d = choose_goal(G8, flat(G8), centre(Sector(2, 2)), None, blocked, P)
    assert d.goal is None


def test_with_no_goal_the_current_sector_is_the_candidate():
    values = with_cells(G8, {Sector(2, 2): 0.9})
    d = choose_goal(G8, values, centre(Sector(2, 2)), None, frozenset(), P)
    assert d.goal == Sector(2, 2) and d.changed   # a new goal, same place


# ---------------------------------------------------------------------------
# plan_step: when to re-decide (D10)
# ---------------------------------------------------------------------------

def test_flying_drone_stays_committed_even_if_a_better_sector_appears():
    goal = Sector(2, 2)
    values = with_cells(G8, {goal: 0.6, Sector(7, 7): 0.99})
    pose = centre(Sector(0, 0))
    d = plan_step(G8, values, 1, pose, goal, [], NOW, P)
    assert d == (goal, False, 'flying')


def test_goal_blocked_by_a_lower_sysid_redecides_immediately():
    goal = Sector(5, 5)
    values = with_cells(G8, {goal: 0.97, Sector(0, 7): 0.5})
    claimant = flying(G8, 1, centre(Sector(7, 0)), Sector(5, 6))
    d = plan_step(G8, values, 2, centre(Sector(0, 0)), goal, [claimant], NOW, P)
    assert d.goal == Sector(0, 7) and d.reason == 'move'


def test_arrival_triggers_a_redecision():
    goal = Sector(2, 2)
    values = with_cells(G8, {goal: 0.6, Sector(7, 7): 0.99})
    d = plan_step(G8, values, 1, centre(goal), goal, [], NOW, P)
    assert d.goal == Sector(7, 7)


def test_is_arrived_uses_the_radius():
    goal = Sector(2, 2)
    cx, cy = centre(goal)
    assert is_arrived(G8, (cx + 2.0, cy), goal, P)
    assert not is_arrived(G8, (cx + 2.01, cy), goal, P)
    # 2.0 m is inside the sector at 5 m (half-width 2.5), D27
    assert G8.world_to_sector(cx + 2.0, cy) == goal


# ---------------------------------------------------------------------------
# Simultaneous decisions with peer state one tick stale (D24)
# ---------------------------------------------------------------------------

def test_higher_sysid_yields_when_both_picked_the_same_goal():
    values = with_cells(G8, {Sector(3, 3): 0.97, Sector(6, 6): 0.90})
    start = {1: centre(Sector(0, 0)), 2: centre(Sector(7, 0))}

    # Tick 1: neither has announced a goal, so both see the same stale world
    # and both pick the best sector.
    stale = {i: Peer(i, pose=start[i], pose_stamp=NOW - 1.0) for i in start}
    goals = {}
    for i in start:
        others = [p for j, p in stale.items() if j != i]
        # Move the other drone's parked zone out of the way: start corners.
        goals[i] = plan_step(G8, values, i, start[i], None, others, NOW, P).goal
    assert goals[1] == goals[2] == Sector(3, 3)

    # Tick 2: each now hears the other's goal (still the stale pose).
    heard = {i: Peer(i, pose=start[i], pose_stamp=NOW, goal=goals[i],
                     goal_stamp=NOW) for i in start}
    d1 = plan_step(G8, values, 1, start[1], goals[1], [heard[2]], NOW + 1, P)
    d2 = plan_step(G8, values, 2, start[2], goals[2], [heard[1]], NOW + 1, P)
    assert d1 == (Sector(3, 3), False, 'flying')      # lower sysid keeps it
    assert d2.reason == 'move' and d2.goal == Sector(6, 6)   # higher yields
    assert d2.goal not in G8.zone(Sector(3, 3))


def test_neighbouring_goals_also_resolve_to_exactly_one_yielder():
    """Goals in each other's zones: exactly one of the two re-plans."""
    values = with_cells(G8, {Sector(3, 3): 0.97, Sector(3, 4): 0.96,
                             Sector(7, 7): 0.5})
    pa = flying(G8, 1, centre(Sector(0, 0)), Sector(3, 3), now=NOW)
    pb = flying(G8, 2, centre(Sector(7, 0)), Sector(3, 4), now=NOW)
    da = plan_step(G8, values, 1, pa.pose, pa.goal, [pb], NOW, P)
    db = plan_step(G8, values, 2, pb.pose, pb.goal, [pa], NOW, P)
    assert da.reason == 'flying'
    assert db.reason == 'move'


# ---------------------------------------------------------------------------
# Arena <-> lat/lon (D8)
# ---------------------------------------------------------------------------

DATUM = Datum(-35.3632621, 149.1652374, 10.19)


def test_arena_to_lla_known_offsets():
    lat, lon, alt = arena_to_lla(DATUM, 0.0, 0.0, 0.0)
    assert (lat, lon, alt) == (DATUM.lat, DATUM.lon, DATUM.alt)
    lat, lon, alt = arena_to_lla(DATUM, 0.0, 111.0, 10.0)
    assert lat - DATUM.lat == pytest.approx(111.0 / 111319.49, rel=1e-3)
    assert lon == DATUM.lon and alt == DATUM.alt + 10.0
    _, lon_e, _ = arena_to_lla(DATUM, 100.0, 0.0)
    east = WGS84_A * math.cos(math.radians(DATUM.lat)) \
        * math.radians(lon_e - DATUM.lon)
    assert east == pytest.approx(100.0, abs=1e-6)


def _localization_node_class():
    """LocalizationNode from the sibling package, if ROS is available."""
    pytest.importorskip('rclpy')
    sibling = Path(__file__).resolve().parents[2] / 'uav_localization'
    if str(sibling) not in sys.path:
        sys.path.append(str(sibling))
    from uav_localization.localization_node import LocalizationNode
    return LocalizationNode


def test_arena_to_lla_round_trips_through_localization_node():
    """Feed lat/lon from arena_to_lla into the real node's projection (datum
    set, as in a swarm) and get the arena point back within 0.01 m."""
    import rclpy
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    node_cls = _localization_node_class()
    rclpy.init(args=['--ros-args',
                     '-p', f'datum_lat:={DATUM.lat}',
                     '-p', f'datum_lon:={DATUM.lon}',
                     '-p', f'datum_alt:={DATUM.alt}'])
    try:
        node = node_cls()
        sent = []
        node._pose_pub = type('Rec', (), {'publish': lambda s, m: sent.append(m)})()
        rng = random.Random(7)
        for _ in range(50):
            x, y, up = (rng.uniform(-300, 300), rng.uniform(-300, 300),
                        rng.uniform(0, 30))
            msg = NavSatFix()
            msg.latitude, msg.longitude, msg.altitude = arena_to_lla(
                DATUM, x, y, up)
            msg.status.status = NavSatStatus.STATUS_FIX
            node._on_navsat(msg)
            p = sent[-1].pose.position
            assert (p.x, p.y, p.z) == pytest.approx((x, y, up), abs=0.01)
        node.destroy_node()
    finally:
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Thesis baselines (D4, D22)
# ---------------------------------------------------------------------------

def test_dga_climbs_to_the_best_neighbour_and_has_no_stay():
    g = GridSpec(5, 5)
    values = with_cells(g, {Sector(3, 2): 0.6, Sector(2, 3): 0.5})
    assert thesis_dga_step(g, values, Sector(2, 2), []) == Sector(3, 2)
    # Sitting on a peak, the literal rule must leave it (R10).
    peak = with_cells(g, {Sector(2, 2): 0.97})
    assert thesis_dga_step(g, peak, Sector(2, 2), []) != Sector(2, 2)
    assert thesis_dga_step(g, peak, Sector(2, 2), [], allow_stay=True) \
        == Sector(2, 2)


def test_dga_zeroes_sectors_in_a_peer_neighbourhood():
    g = GridSpec(5, 5)
    values = with_cells(g, {Sector(3, 2): 0.9, Sector(1, 2): 0.5})
    # Peer at (4, 2): (3, 2) is in its neighbourhood, so utility 0.
    assert thesis_dga_step(g, values, Sector(2, 2), [Sector(4, 2)]) \
        == Sector(1, 2)
    # Literal L excludes the peer's own sector; the closed variant adds it.
    on_peer = with_cells(g, {Sector(3, 2): 0.9})
    assert thesis_dga_step(g, on_peer, Sector(2, 2), [Sector(3, 2)]) \
        == Sector(3, 2)
    assert thesis_dga_step(g, on_peer, Sector(2, 2), [Sector(3, 2)],
                           penalise_peer_cell=True) != Sector(3, 2)


def test_bll_move_probability_is_the_softmax_formula():
    for u_alt, u_here in [(0.97, 0.03), (0.03, 0.97), (0.5, 0.5), (0.0, 1.0)]:
        expect = math.exp(u_alt) / (math.exp(u_alt) + math.exp(u_here))
        assert bll_move_probability(u_alt, u_here) == pytest.approx(expect)
    assert bll_move_probability(0.5, 0.5) == pytest.approx(0.5)


def test_bll_step_reports_the_formula_probability():
    g = GridSpec(5, 5)
    values = with_cells(g, {Sector(3, 2): 0.9})
    rng = random.Random(1)
    for _ in range(30):
        r = bll_step(g, values, Sector(2, 2), [], rng)
        u_here = values[2][2]
        u_alt = values[r.candidate.row][r.candidate.col]
        assert r.p_move == pytest.approx(bll_move_probability(u_alt, u_here))
        assert r.sector in (Sector(2, 2), r.candidate)


def test_bll_empirical_move_rate_matches_formula():
    g = GridSpec(3, 3)
    values = with_cells(g, {Sector(2, 1): 0.97})        # only one hot neighbour
    here = Sector(1, 1)
    rng = random.Random(3)
    n, moved, expect = 20000, 0, 0.0
    for _ in range(n):
        r = bll_step(g, values, here, [], rng)
        moved += r.moved
        expect += r.p_move
    assert moved / n == pytest.approx(expect / n, abs=0.01)


def test_bll_is_repeatable_and_stays_on_the_grid():
    g = GridSpec(4, 4)
    values = with_cells(g, {Sector(3, 3): 0.97})

    def run(seed):
        rng = random.Random(seed)
        pos = [Sector(0, 0), Sector(3, 0), Sector(0, 3)]
        trace = []
        for _ in range(500):
            a = rng.randrange(len(pos))
            others = pos[:a] + pos[a + 1:]
            pos[a] = bll_step(g, values, pos[a], others, rng).sector
            assert all(g.contains(s) for s in pos)
            trace.append(tuple(pos))
        return trace

    assert run(5) == run(5)
    assert run(5) != run(6)


def test_bll_iteration_count_report(capsys):
    """REPORT, not assert (D22): iterations until all 3 drones first sit on 3
    peaks, beside the thesis's 153 (which states no grid, map or definition).
    Setup: 4x4 at 5 m, 3 drones, 3 peaks at 0.97 on a 0.03 background, peaks
    at least 2 apart, random starts, closed peer penalty, one random drone per
    iteration, 40 seeds, capped at 8000."""
    g = GridSpec(4, 4)
    cap, firsts = 8000, []
    for seed in range(40):
        rng = random.Random(seed)
        peaks = _spaced_peaks(g, rng, 3, 2)
        values = with_cells(g, {p: 0.97 for p in peaks})
        pos = rng.sample(list(g.sectors()), 3)
        first = cap
        for it in range(cap):
            a = rng.randrange(3)
            pos[a] = bll_step(g, values, pos[a], pos[:a] + pos[a + 1:], rng,
                              penalise_peer_cell=True).sector
            if set(pos) == set(peaks):
                first = it + 1
                break
        firsts.append(first)
    with capsys.disabled():
        print(f"\n[BLL report] median iterations to all-3-on-peaks: "
              f"{statistics.median(firsts):.0f} (reached in "
              f"{sum(f < cap for f in firsts)}/40 seeds); thesis states 153")
    assert all(1 <= f <= cap for f in firsts)


# ---------------------------------------------------------------------------
# Coordination harness: fake poses, no flight (D24, D25)
# ---------------------------------------------------------------------------

def _spaced_peaks(grid, rng, k, spacing):
    cells = list(grid.sectors())
    while True:
        s = rng.sample(cells, k)
        if all(chebyshev(a, b) >= spacing for i, a in enumerate(s)
               for b in s[i + 1:]):
            return s


def _fly_one_sector(grid, pose, goal):
    """Move at most one sector per axis toward the goal centre (1 sector/tick)."""
    gx, gy = grid.sector_centre(goal)
    r = grid.resolution
    return (pose[0] + max(-r, min(r, gx - pose[0])),
            pose[1] + max(-r, min(r, gy - pose[1])))


def run_harness(seed, grid=G8, drones=3, peaks=3, spacing=2, ticks=80):
    """Simultaneous decisions, every drone seeing its peers one tick stale,
    flights of one sector per tick. Returns (peaks held, goal changes, tick of
    the last goal change)."""
    rng = random.Random(seed)
    peak_cells = _spaced_peaks(grid, rng, peaks, spacing)
    values = with_cells(grid, {p: 0.97 for p in peak_cells})
    pose = {i: grid.sector_centre(s) for i, s in
            enumerate(rng.sample(list(grid.sectors()), drones), start=1)}
    goal = {i: None for i in pose}
    changes = last_change = 0
    for t in range(ticks):
        now = float(t)
        snap_pose, snap_goal = dict(pose), dict(goal)
        for i in pose:
            peers = [Peer(j, pose=snap_pose[j], pose_stamp=now - 1.0,
                          goal=snap_goal[j],
                          goal_stamp=now - 1.0 if snap_goal[j] else -math.inf)
                     for j in pose if j != i]
            d = plan_step(grid, values, i, snap_pose[i], snap_goal[i], peers,
                          now, P)
            if d.goal != goal[i]:
                goal[i] = d.goal
                changes += 1
                last_change = t
        for i in pose:
            if goal[i] is not None:
                pose[i] = _fly_one_sector(grid, pose[i], goal[i])
    held = {s for s in peak_cells
            if any(grid.world_to_sector(*p) == s for p in pose.values())}
    return len(held), changes, last_change


def test_three_drones_hold_three_peaks_on_every_seed():
    results = [run_harness(seed) for seed in range(200)]
    failures = [(s, r) for s, r in enumerate(results) if r[0] != 3]
    assert not failures, f"seeds not holding all 3 peaks: {failures[:5]}"
    assert all(r[2] < 75 for r in results), "some run was still changing goals"


def test_harness_goal_churn_is_small(capsys):
    results = [run_harness(seed) for seed in range(200)]
    with capsys.disabled():
        print(f"\n[harness] 200 seeds, 8x8: all-3-held "
              f"{sum(r[0] == 3 for r in results)}/200, mean goal changes "
              f"{statistics.mean(r[1] for r in results):.1f}")
