"""Tests for the leader sweep (core/sweep.py). No ROS needed."""

import math

import pytest

from uav_planning.core.planner import GridSpec
from uav_planning.core.sweep import (
    COMPLETE, SWEEPING, WAITING, Camera, footprint_half_extents,
    lane_half_width, lane_order_by_prior, lane_positions, lanes,
    parse_lane_order, path_length,
    plan_sweep, sweep_tick)

# The sim camera (docker-notes gotcha 47): 640x480, fx = fy = 205.47.
CAM = Camera(205.47, 205.47, 320.0, 240.0, 640, 480)
LAYER = 8.0                                # leader layer (D26)
ARENA = GridSpec(8, 8, 5.0)                # 40 x 40 m (D39)
HW = lane_half_width(CAM, LAYER)           # 0.8 x 9.34 m


# ---- Footprint and spacing -------------------------------------------------

def test_footprint_at_8_m_matches_the_sim_lens():
    half_u, half_v = footprint_half_extents(CAM, LAYER)
    assert 2 * half_u == pytest.approx(24.92, abs=0.01)
    assert 2 * half_v == pytest.approx(18.69, abs=0.01)


def test_spacing_uses_the_short_side():
    assert HW == pytest.approx(0.8 * 8.0 * 240 / 205.47)


def test_off_centre_principal_point_uses_the_nearer_edge():
    cam = Camera(205.47, 205.47, 300.0, 240.0, 640, 480)
    assert footprint_half_extents(cam, LAYER)[0] == pytest.approx(8.0 * 300 / 205.47)


def test_overlap_must_be_a_fraction():
    with pytest.raises(ValueError):
        lane_half_width(CAM, LAYER, overlap=0.0)
    with pytest.raises(ValueError):
        lane_half_width(CAM, LAYER, overlap=1.5)


def test_lane_positions_are_even_and_just_enough():
    assert lane_positions(0.0, 30.0, 5.0) == pytest.approx([5.0, 15.0, 25.0])
    # 30.01 no longer fits in three strips of 10 m.
    assert len(lane_positions(0.0, 30.01, 5.0)) == 4


def test_zero_span_is_one_lane():
    assert lane_positions(2.5, 2.5, 1.0) == [2.5]


# ---- Lanes on the arenas in play ---------------------------------------------

def test_40_m_arena_takes_three_lanes():
    xs = [a[0] for a, _ in lanes(ARENA, HW)]
    assert xs == pytest.approx([2.5 + 35 / 6, 20.0, 37.5 - 35 / 6])


def test_long_side_spacing_would_take_two():
    """The option D39 rejected: what the long side would have given."""
    long_hw = 0.8 * footprint_half_extents(CAM, LAYER)[0]
    assert len(lanes(ARENA, long_hw)) == 2


def test_20_m_arena_just_misses_one_lane():
    """Centres span 15 m and one strip is 2 x 7.47 m wide, so two lanes."""
    assert lanes(GridSpec(4, 4, 5.0), HW) == [((6.25, 2.5), (6.25, 17.5)),
                                              ((13.75, 2.5), (13.75, 17.5))]


def test_lanes_run_between_the_end_cell_centres():
    for a, b in lanes(ARENA, HW):
        assert (a[1], b[1]) == (2.5, 37.5)


def test_east_lanes_run_east_west():
    ls = lanes(ARENA, HW, axis='east')
    assert len(ls) == 3
    assert all(a[0] == 2.5 and b[0] == 37.5 and a[1] == b[1] for a, b in ls)


def test_unknown_axis_is_rejected():
    with pytest.raises(ValueError):
        lanes(ARENA, HW, axis='y')


def test_offset_origin_shifts_the_lanes():
    g = GridSpec(8, 8, 5.0, -20.0, -20.0)
    assert [a for a, _ in lanes(g, HW)][1] == pytest.approx((0.0, -17.5))


# ---- Waypoint order -----------------------------------------------------------

def test_from_home_corner_the_sweep_is_a_boustrophedon():
    wps = plan_sweep(ARENA, HW, start=(0.0, 0.0))
    x0, x1, x2 = (a[0] for a, _ in lanes(ARENA, HW))
    assert wps == pytest.approx([(x0, 2.5), (x0, 37.5), (x1, 37.5),
                                 (x1, 2.5), (x2, 2.5), (x2, 37.5)])


def test_from_the_far_corner_it_starts_on_the_far_lane():
    wps = plan_sweep(ARENA, HW, start=(40.0, 40.0))
    x2 = lanes(ARENA, HW)[2][0][0]
    assert wps[0] == pytest.approx((x2, 37.5))
    assert wps[1] == pytest.approx((x2, 2.5))


def test_lane_order_is_followed_and_each_lane_entered_at_its_near_end():
    ls = lanes(ARENA, HW)
    wps = plan_sweep(ARENA, HW, start=(0.0, 0.0), lane_order=[1, 0, 2])
    assert [p[0] for p in wps[::2]] == pytest.approx([ls[i][0][0] for i in (1, 0, 2)])
    assert [p[1] for p in wps] == [2.5, 37.5, 37.5, 2.5, 2.5, 37.5]


@pytest.mark.parametrize('order', [[0, 1], [0, 1, 1], [0, 1, 3], [0, 1, 2, 3]])
def test_lane_order_must_cover_every_lane_once(order):
    with pytest.raises(ValueError):
        plan_sweep(ARENA, HW, start=(0.0, 0.0), lane_order=order)


def test_parse_lane_order():
    assert parse_lane_order('') is None
    assert parse_lane_order(' 2,0,1 ') == [2, 0, 1]


def test_path_length_from_home():
    wps = plan_sweep(ARENA, HW, start=(0.0, 0.0))
    x0 = wps[0][0]
    expected = math.hypot(x0, 2.5) + 3 * 35.0 + 2 * (35 / 3)
    assert path_length((0.0, 0.0), wps) == pytest.approx(expected)


# ---- Coverage: the property the whole design is for ----------------------------

def _in_footprint(point, drone, yaw, half_u, half_v):
    """Is a ground point inside a nadir footprint rotated by ``yaw``?"""
    dx, dy = point[0] - drone[0], point[1] - drone[1]
    c, s = math.cos(yaw), math.sin(yaw)
    along, across = c * dx + s * dy, -s * dx + c * dy
    return abs(across) <= half_u and abs(along) <= half_v


def _samples(start, waypoints, step=0.5):
    here = start
    for p in waypoints:
        n = max(1, int(math.dist(here, p) / step))
        for k in range(n + 1):
            yield (here[0] + (p[0] - here[0]) * k / n,
                   here[1] + (p[1] - here[1]) * k / n)
        here = p


@pytest.mark.parametrize('grid', [ARENA, GridSpec(4, 4, 5.0), GridSpec(16, 16, 5.0),
                                  GridSpec(5, 11, 4.0, -7.0, 3.0)])
@pytest.mark.parametrize('yaw_deg', [0, 30, 45, 90, 137, 180])
def test_every_cell_centre_is_seen_at_any_heading(grid, yaw_deg):
    """The D39 claim: with short-side spacing, every cell centre falls inside
    the footprint somewhere on the path, whichever way the drone faces."""
    half_u, half_v = footprint_half_extents(CAM, LAYER)
    wps = plan_sweep(grid, HW, start=(grid.origin_x, grid.origin_y))
    poses = list(_samples(wps[0], wps[1:]))
    yaw = math.radians(yaw_deg)
    unseen = [s for s in grid.sectors()
              if not any(_in_footprint(grid.sector_centre(s), p, yaw, half_u, half_v)
                         for p in poses)]
    assert unseen == []


# ---- The tick -----------------------------------------------------------------

WPS = plan_sweep(ARENA, HW, start=(0.0, 0.0))


def test_no_plan_waits():
    assert sweep_tick(None, 0, (0.0, 0.0, LAYER), LAYER) == (WAITING, 0, None, 'no-plan')


def test_no_pose_waits():
    assert sweep_tick(WPS, 0, None, LAYER) == (WAITING, 0, None, 'no-pose')


def test_no_sideways_command_before_the_layer():
    """D26: climbing at home sends nothing horizontal."""
    assert sweep_tick(WPS, 0, (0.0, 0.0, 6.0), LAYER) == (WAITING, 0, None, 'climbing')


def test_at_layer_flies_to_the_first_waypoint():
    assert sweep_tick(WPS, 0, (0.0, 0.0, LAYER), LAYER) == (
        SWEEPING, 0, WPS[0], 'flying')


def test_arriving_advances_one_waypoint():
    out = sweep_tick(WPS, 2, (WPS[2][0] + 0.6, WPS[2][1], LAYER), LAYER)
    assert out == (SWEEPING, 3, WPS[3], 'advanced')


def test_outside_tolerance_keeps_the_waypoint():
    out = sweep_tick(WPS, 2, (WPS[2][0] + 1.5, WPS[2][1], LAYER), LAYER)
    assert out == (SWEEPING, 2, WPS[2], 'flying')


def test_off_layer_mid_sweep_keeps_flying_but_does_not_advance():
    out = sweep_tick(WPS, 2, (WPS[2][0], WPS[2][1], LAYER - 2), LAYER)
    assert out == (SWEEPING, 2, WPS[2], 'flying')


def test_last_waypoint_completes_and_stays_complete():
    last = len(WPS) - 1
    out = sweep_tick(WPS, last, (*WPS[last], LAYER), LAYER)
    assert out == (COMPLETE, len(WPS), None, 'done')
    assert sweep_tick(WPS, out.index, None, LAYER) == (COMPLETE, len(WPS), None, 'done')


def test_a_whole_sweep_visits_every_waypoint_in_order():
    """Teleport the drone to whatever it was told: the tick walks the list."""
    pose, index, visited = (0.0, 0.0, LAYER), 0, []
    for _ in range(3 * len(WPS)):
        out = sweep_tick(WPS, index, pose, LAYER)
        index = out.index
        if out.command is not None:
            if not visited or visited[-1] != out.command:
                visited.append(out.command)
            pose = (*out.command, LAYER)
        if out.status == COMPLETE:
            break
    assert out.status == COMPLETE
    assert visited == WPS


# ---- Lanes through the clue's cells first (D50) -----------------------------

BIG = GridSpec(24, 24)


def _raised(*cells):
    out = [[0.0] * BIG.width for _ in range(BIG.height)]
    for col, row, v in cells:
        out[row][col] = v
    return out


def _order(*cells, start=(2.5, 2.5), axis='north'):
    return lane_order_by_prior(BIG, 7.47, _raised(*cells), start, axis)


def test_no_clue_is_the_default_lane_order():
    n = len(lanes(BIG, 7.47))
    assert _order() == list(range(n))
    assert _order(start=(117.5, 2.5)) == list(range(n - 1, -1, -1))
    assert plan_sweep(BIG, 7.47, (2.5, 2.5), lane_order=_order()) == \
        plan_sweep(BIG, 7.47, (2.5, 2.5))


def test_the_clues_lane_flies_first_and_every_lane_still_flies():
    order = _order((22, 20, 0.25))
    n = len(order)
    assert order[0] == n - 1 and order[1:] == list(range(n - 2, -1, -1))
    assert sorted(order) == list(range(n))
    assert sorted(plan_sweep(BIG, 7.47, (2.5, 2.5), lane_order=order)) == \
        sorted(plan_sweep(BIG, 7.47, (2.5, 2.5)))


def test_the_highest_lane_comes_first_and_the_rest_start_at_the_nearer_end():
    order = _order((1, 5, 0.1), (13, 5, 0.1), (13, 6, 0.2), (22, 9, 0.2))
    n = len(order)
    assert order[:3] == [4, n - 1, 0]
    assert order[3:] == [i for i in range(n) if i not in order[:3]]


def test_equal_lanes_keep_the_default_order():
    assert _order((1, 5, 0.2), (22, 5, 0.2))[:2] == [0, 7]
    assert _order((1, 5, 0.2), (22, 5, 0.2), start=(117.5, 2.5))[:2] == [7, 0]


def test_east_lanes_are_scored_by_row():
    assert _order((1, 22, 0.2), axis='east')[0] == 7
    assert _order((1, 22, 0.2), axis='north')[0] == 0
