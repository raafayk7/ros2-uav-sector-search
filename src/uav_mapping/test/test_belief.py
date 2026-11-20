"""Tests for the ROS-free belief map: footprint geometry, the per-frame update,
the R29 look cap, D32 confirmation and D13 suppression (D40)."""

import math

import numpy as np
import pytest

from uav_mapping.core.belief import (BeliefMap, BeliefParams, Frame, logit,
                                     project_frame)
from uav_mapping.core.geometry import (
    NADIR_QUAT, Camera, Grid, camera_pose, cells_in_quad, footprint,
    pixel_to_ground, quat_to_rot)

GRID = Grid()                  # 8x8 at 5 m, corner origin (D39)
CAM = Camera()                 # the sim camera: 640x480, fx 205.47
LEVEL = (0.0, 0.0, 0.0, 1.0)   # nose along +x (east)
P = BeliefParams()
NO_CAP = BeliefParams(look_cap=0.0)


def yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def pct(bm, cell):
    return 100.0 * bm.probability(cell)


# ---- Geometry -------------------------------------------------------------

def test_nadir_quat_matches_leader_nodes_matrix():
    leader = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    assert np.allclose(quat_to_rot(*NADIR_QUAT), leader)


def test_image_centre_lands_below_the_drone():
    cam_pos, R = camera_pose((12.0, 7.0, 8.0), yaw(37))
    assert pixel_to_ground(CAM.cx, CAM.cy, cam_pos, R, CAM) == pytest.approx((12.0, 7.0))


def test_pixel_offsets_follow_the_nadir_mount():
    # Nose east: image up is +x (forward), image right is -y (starboard).
    h = 10.0
    cam_pos, R = camera_pose((0.0, 0.0, h), LEVEL)
    x, y = pixel_to_ground(CAM.cx + 100, CAM.cy - 50, cam_pos, R, CAM)
    assert x == pytest.approx(50 / CAM.fy * h)
    assert y == pytest.approx(-100 / CAM.fx * h)


def test_footprint_size_at_8_m():
    # D39's numbers: 24.9 x 18.7 m at 8 m with the sim lens.
    cam_pos, R = camera_pose((20.0, 20.0, 8.0), LEVEL)
    quad = footprint(cam_pos, R, CAM)
    xs, ys = [p[0] for p in quad], [p[1] for p in quad]
    assert max(xs) - min(xs) == pytest.approx(18.69, abs=0.01)   # along image v
    assert max(ys) - min(ys) == pytest.approx(24.92, abs=0.01)   # along image u


def test_no_footprint_when_on_the_ground_or_tilted_to_the_horizon():
    cam_pos, R = camera_pose((5.0, 5.0, 0.0), LEVEL)
    assert footprint(cam_pos, R, CAM) is None
    # Pitched 80 deg: the top image corners look above the horizon.
    a = math.radians(80) / 2
    cam_pos, R = camera_pose((5.0, 5.0, 8.0), (0.0, math.sin(a), 0.0, math.cos(a)))
    assert footprint(cam_pos, R, CAM) is None


def test_cells_in_quad_uses_cell_centres():
    quad = [(0.0, 0.0), (10.0, 0.0), (10.0, 7.4), (0.0, 7.4)]
    assert cells_in_quad(GRID, quad) == {(0, 0), (1, 0)}      # centres at y 2.5
    quad = [(0.0, 0.0), (10.0, 0.0), (10.0, 7.6), (0.0, 7.6)]
    assert cells_in_quad(GRID, quad) == {(0, 0), (1, 0), (0, 1), (1, 1)}


def test_cells_outside_the_grid_are_dropped():
    assert GRID.cell_of(-0.1, 3.0) is None
    assert GRID.cell_of(40.0, 3.0) is None
    assert GRID.cell_of(39.9, 39.9) == (7, 7)


# ---- project_frame ------------------------------------------------------

def test_detection_at_image_centre_hits_the_drones_cell():
    f = project_frame(GRID, CAM, (17.5, 17.5, 10.0), yaw(90),
                      [(CAM.cx, CAM.cy, 0.8)])
    assert f.hits == {(3, 3): pytest.approx(0.8)}
    assert (3, 3) in f.seen


def test_weak_and_off_grid_detections_are_dropped():
    # At the corner, the image's top-left lands outside the grid.
    f = project_frame(GRID, CAM, (1.0, 1.0, 10.0), LEVEL,
                      [(CAM.cx, CAM.cy, 0.2), (CAM.width - 1, CAM.height - 1, 0.9)])
    assert f.hits == {}


def test_two_boxes_in_one_cell_add_their_scores():
    f = project_frame(GRID, CAM, (17.5, 17.5, 10.0), LEVEL,
                      [(CAM.cx, CAM.cy, 0.6), (CAM.cx + 5, CAM.cy, 0.5)])
    assert f.hits == {(3, 3): pytest.approx(1.1)}


def test_on_the_ground_nothing_is_seen():
    f = project_frame(GRID, CAM, (17.5, 17.5, 0.0), LEVEL, [])
    assert f.seen == set()


# ---- Per-frame update (cap off: leader_node's rule) ----------------------

def empty(*cells):
    return Frame(set(cells), {})


def hit(cell, score=1.0, *seen):
    return Frame({cell, *seen}, {cell: score})


def test_unobserved_cells_publish_unknown():
    bm = BeliefMap(GRID)
    assert bm.occupancy_data() == [-1] * 64
    assert bm.probability((0, 0)) is None


def test_without_the_cap_each_frame_counts_as_in_leader_node():
    bm = BeliefMap(GRID, NO_CAP)
    for i in range(3):
        bm.update('a', i * 0.5, (0, 0, 8), hit((2, 2), 0.9, (3, 3)))
    assert bm.logodds[2, 2] == pytest.approx(3 * 0.9 * logit(0.7))
    assert bm.logodds[3, 3] == pytest.approx(3 * logit(0.45))
    for i in range(3, 6):
        bm.update('a', i * 0.5, (0, 0, 8), hit((2, 2)))
    assert pct(bm, (2, 2)) == pytest.approx(97.0)         # clamp


def test_a_hit_cell_gets_no_miss_in_the_same_frame():
    bm = BeliefMap(GRID, NO_CAP)
    bm.update('a', 0.0, (0, 0, 8), hit((2, 2), 0.5, (2, 2), (2, 3)))
    assert bm.logodds[2, 2] == pytest.approx(0.5 * logit(0.7))
    assert bm.logodds[3, 2] == pytest.approx(logit(0.45))


def test_published_values_are_percent():
    bm = BeliefMap(GRID, NO_CAP)
    bm.update('a', 0.0, (0, 0, 8), empty((0, 0)))
    data = bm.occupancy_data()
    assert data[0] == 45
    assert data[1:] == [-1] * 63


def test_row_major_from_the_origin_corner():
    bm = BeliefMap(GRID, NO_CAP)
    bm.update('a', 0.0, (0, 0, 8), empty((1, 0), (0, 1)))
    data = bm.occupancy_data()
    assert (data[1], data[8]) == (45, 45)    # (col 1, row 0) and (col 0, row 1)


# ---- The R29 look cap (D40) -----------------------------------------------

def test_one_look_moves_a_cell_at_most_the_cap():
    bm = BeliefMap(GRID)
    for i in range(20):                       # 10 s of view at 2 Hz
        bm.update('a', i * 0.5, (0, 0, 8), empty((0, 0)))
        if i == 7:                            # 3.5 s: still the first look
            assert bm.logodds[0, 0] == pytest.approx(-P.look_cap)
    # 0-3.5 s, 4-7.5 s, 8-9.5 s: three looks, the last not yet capped.
    assert bm.logodds[0, 0] == pytest.approx(-2 * P.look_cap + 4 * logit(0.45))


def test_cap_is_reached_in_about_four_empty_frames():
    bm = BeliefMap(GRID)
    for i in range(4):
        bm.update('a', i * 0.5, (0, 0, 8), empty((0, 0)))
    assert bm.logodds[0, 0] == pytest.approx(4 * logit(0.45))   # 0.80 < 0.85
    bm.update('a', 2.0, (0, 0, 8), empty((0, 0)))
    assert bm.logodds[0, 0] == pytest.approx(-P.look_cap)


def test_a_gap_out_of_view_starts_a_new_look():
    bm = BeliefMap(GRID)
    for i in range(6):
        bm.update('a', i * 0.5, (0, 0, 8), empty((0, 0)))
    assert bm.logodds[0, 0] == pytest.approx(-P.look_cap)
    bm.update('a', 2.5 + 1.0, (0, 0, 8), empty((0, 0)))         # gap exactly 1 s
    assert bm.logodds[0, 0] == pytest.approx(-P.look_cap)        # same look
    bm.update('a', 3.5 + 1.01, (0, 0, 8), empty((0, 0)))        # gap > 1 s
    assert bm.logodds[0, 0] == pytest.approx(-P.look_cap + logit(0.45))


def test_a_held_false_positive_needs_several_looks_to_look_certain():
    # R29's worry: one box held in view at score 0.9, 2 Hz.
    capped, free = BeliefMap(GRID), BeliefMap(GRID, NO_CAP)
    t_capped = t_free = None
    for i in range(60):
        t = i * 0.5
        for bm in (capped, free):
            bm.update('a', t, (0, 0, 8), hit((4, 4), 0.9))
        if t_free is None and pct(free, (4, 4)) >= 96.5:
            t_free = t
        if t_capped is None and pct(capped, (4, 4)) >= 96.5:
            t_capped = t
    assert t_free == pytest.approx(2.0)
    assert t_capped == pytest.approx(12.5)    # 2nd frame of the 4th look
    assert pct(capped, (4, 4)) == pytest.approx(97.0)


def test_two_drones_seeing_a_cell_at_once_both_count():
    bm = BeliefMap(GRID)
    for i in range(6):
        bm.update('a', i * 0.5, (0, 0, 8), empty((5, 5)))
        bm.update('b', i * 0.5, (0, 0, 10), empty((5, 5)))
    assert bm.logodds[5, 5] == pytest.approx(-2 * P.look_cap)


def test_hits_and_misses_in_one_look_share_the_cap():
    bm = BeliefMap(GRID)
    for i in range(6):
        bm.update('a', i * 0.5, (0, 0, 8), empty((1, 1)))
    bm.update('a', 3.0, (0, 0, 8), hit((1, 1), 1.0))
    # -cap, then +l_occ (0.847 = cap): back to 0 within the same look.
    assert bm.logodds[1, 1] == pytest.approx(0.0, abs=1e-12)


# ---- D32 confirmation and D13 suppression ---------------------------------

CENTRE = GRID.centre((3, 3))                  # (17.5, 17.5)
OVER = (CENTRE[0] + 1.0, CENTRE[1], 10.0)     # 1 m off centre, 10 m up


def test_three_consecutive_frames_confirm_the_sector():
    bm = BeliefMap(GRID)
    got = [bm.update('f', i * 0.5, OVER, hit((3, 3)), confirmer=True)
           for i in range(3)]
    assert got == [None, None, (3, 3)]
    assert bm.confirmed == {(3, 3): ('f', 1.0)}


def test_a_frame_without_the_detection_restarts_the_count():
    bm = BeliefMap(GRID)
    frames = [hit((3, 3)), hit((3, 3)), empty((3, 3)), hit((3, 3)), hit((3, 3))]
    got = [bm.update('f', i * 0.5, OVER, fr, confirmer=True)
           for i, fr in enumerate(frames)]
    assert got == [None] * 5
    assert bm.update('f', 2.5, OVER, hit((3, 3)), confirmer=True) == (3, 3)


def test_a_detection_in_another_sector_does_not_count():
    bm = BeliefMap(GRID)
    for i in range(5):
        assert bm.update('f', i * 0.5, OVER, hit((4, 3)), confirmer=True) is None
    assert bm.confirmed == {}


def test_only_confirmers_confirm():
    bm = BeliefMap(GRID)
    for i in range(5):
        assert bm.update('leader', i * 0.5, OVER, hit((3, 3))) is None
    assert bm.confirmed == {}


@pytest.mark.parametrize('pos', [
    (CENTRE[0] + 2.1, CENTRE[1], 10.0),       # outside the arrival radius
    (CENTRE[0], CENTRE[1], 4.9),              # below confirm_min_alt
    (-3.0, CENTRE[1], 10.0),                  # off the grid
])
def test_no_confirmation_unless_hovering_close(pos):
    bm = BeliefMap(GRID)
    for i in range(5):
        bm.update('f', i * 0.5, pos, hit((3, 3)), confirmer=True)
    assert bm.confirmed == {}


def test_leaving_the_radius_restarts_the_count():
    bm = BeliefMap(GRID)
    far = (CENTRE[0] + 2.5, CENTRE[1], 10.0)
    for i, pos in enumerate([OVER, OVER, far, OVER, OVER]):
        assert bm.update('f', i * 0.5, pos, hit((3, 3)), confirmer=True) is None
    assert bm.update('f', 2.5, OVER, hit((3, 3)), confirmer=True) == (3, 3)


def test_streaks_are_per_drone():
    bm = BeliefMap(GRID)
    for i in range(2):
        bm.update('f1', i * 0.5, OVER, hit((3, 3)), confirmer=True)
        bm.update('f2', i * 0.5, OVER, empty((3, 3)), confirmer=True)
    assert bm.update('f1', 1.0, OVER, hit((3, 3)), confirmer=True) == (3, 3)


def test_finds_list_drone_and_time_in_order_of_finding():
    bm = BeliefMap(GRID)
    other = GRID.centre((1, 1))
    over_other = (other[0], other[1], 12.0)
    for i in range(3):
        bm.update('f2', 10.0 + i * 0.5, over_other, hit((1, 1)), confirmer=True)
    for i in range(3):
        bm.update('f1', 5.0 + i * 0.5, OVER, hit((3, 3)), confirmer=True)
    assert bm.finds() == [{'cell': [3, 3], 'drone': 'f1', 't': 6.0},
                          {'cell': [1, 1], 'drone': 'f2', 't': 11.0}]
    assert BeliefMap(GRID).finds() == []


def test_a_confirmed_sector_publishes_zero_and_stays_there():
    bm = BeliefMap(GRID)
    for i in range(3):
        bm.update('f', i * 0.5, OVER, hit((3, 3)), confirmer=True)
    for i in range(3, 40):                    # more hits from anyone
        bm.update('leader', i * 0.5, OVER, hit((3, 3)))
        assert bm.update('f', i * 0.5, OVER, hit((3, 3)), confirmer=True) is None
    idx = 3 * GRID.width + 3
    assert bm.occupancy_data()[idx] == 0
    assert pct(bm, (3, 3)) == pytest.approx(97.0)   # posterior kept underneath
    assert list(bm.confirmed) == [(3, 3)]


def test_confirmation_from_projected_frames():
    # End to end with real geometry: a follower over sector (3, 3) at 10 m.
    bm = BeliefMap(GRID)
    pos = (17.5, 17.5, 10.0)
    for i in range(3):
        f = project_frame(GRID, CAM, pos, yaw(90), [(CAM.cx + 3, CAM.cy - 2, 0.7)])
        got = bm.update('f', i * 0.5, pos, f, confirmer=True)
    assert got == (3, 3)
