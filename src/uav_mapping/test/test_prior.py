"""Tests for the ROS-free language prior: the JSON contract, rasterization
(D44), the starting belief (D45), several targets (D46), the misleading-clue
rule (D47) and replay (D48)."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from uav_mapping.core import landmarks as lmk
from uav_mapping.core.belief import BeliefMap, BeliefParams, Frame
from uav_mapping.core.geometry import Grid, coverage, segment_distance
from uav_mapping.core.prior import (
    MAX_REGIONS, Capsule, Circle, NoRecording, PriorError, Recording,
    ReplayProvider, clue_key, load_replay, make_prior, overlap, parse_recordings,
    parse_regions, rasterize, starting_belief)

GRID = Grid()                  # 8x8 at 5 m, corner origin (D39)
ROOT = Path(__file__).resolve().parents[3]
WORLD_FILE = ROOT / 'worlds/runway/landmarks.yaml'
RECORDINGS = ROOT / 'worlds/runway/recordings.json'


def circle(**kw):
    return {'shape': 'circle', 'center': [20, 20], 'radius': 5, 'weight': 1.0,
            'confidence': 1.0, **kw}


def capsule(**kw):
    return {'shape': 'capsule', 'start': [5, 20], 'end': [35, 20], 'radius': 3,
            'weight': 1.0, 'confidence': 1.0, **kw}


def text(*regions):
    return json.dumps({'regions': list(regions)})


def without(d, key):
    return {k: v for k, v in d.items() if k != key}


@pytest.fixture(scope='module')
def world():
    return lmk.load_world(WORLD_FILE)


class Fake:
    """A provider that returns, or raises, one item per call."""

    def __init__(self, *items):
        self.items, self.calls = list(items), 0

    def __call__(self, clue, world):
        item = self.items[min(self.calls, len(self.items) - 1)]
        self.calls += 1
        if isinstance(item, Exception):
            raise item
        return item


# ---- The contract ---------------------------------------------------------

def test_parses_both_shapes():
    assert parse_regions(text(circle(weight=0.5), capsule(confidence=0))) == [
        Circle((20.0, 20.0), 5.0, 0.5, 1.0),
        Capsule((5.0, 20.0), (35.0, 20.0), 3.0, 1.0, 0.0)]


def test_empty_list_is_valid():
    assert parse_regions('{"regions": []}') == []


@pytest.mark.parametrize('bad', [
    '', 'the river', '[]', 'null', '{}', '{"regions": null}', '{"regions": {}}',
    '```json\n{"regions": []}\n```', '{"regions": []} Good luck!',
    '{"regions": [], "goto": [3, 4]}', '{"regions": [], "regions": []}',
    '[' * 100000,
    text('circle'), text([]),
    text(circle(shape='square')), text(circle(shape=['circle'])),
    text(without(circle(), 'weight')), text(without(circle(), 'shape')),
    text(circle(velocity=[1, 0])), text({**circle(), 'end': [1, 2]}),
    text(circle(center=[1])), text(circle(center=[1, 2, 3])),
    text(circle(center='20,20')), text(circle(center=[True, 2])),
    text(circle(radius=0)), text(circle(radius=-1)), text(circle(radius='5')),
    text(circle(radius=None)), text(circle(radius=True)),
    text(circle(weight=1.01)), text(circle(weight=-0.01)),
    text(circle(confidence=90)), text(circle(confidence=None)),
    text(circle(radius=math.nan)), text(circle(center=[math.inf, 0])),
    text(circle(radius=0)).replace('"radius": 0', '"radius": 1e999'),
    text(circle(radius=10 ** 400)),
    text(circle()).replace('"radius": 5', '"radius": 5, "radius": 6'),
    text(*[circle()] * (MAX_REGIONS + 1)),
])
def test_anything_else_is_invalid(bad):
    with pytest.raises(PriorError):
        parse_regions(bad)


@pytest.mark.parametrize('bad', [None, 7, b'\xff\xfe\xff'])
def test_not_even_text_is_invalid(bad):
    with pytest.raises(PriorError):
        parse_regions(bad)


# ---- Rasterization (D44) --------------------------------------------------

def test_segment_distance():
    a, b = (0.0, 0.0), (10.0, 0.0)
    assert segment_distance(5.0, 3.0, a, b) == pytest.approx(3.0)
    assert segment_distance(13.0, 4.0, a, b) == pytest.approx(5.0)    # past the end
    assert segment_distance(3.0, 4.0, a, a) == pytest.approx(5.0)     # a point


def test_coverage_is_indexed_row_col():
    east = coverage(GRID, lambda x, y: x >= 35.0)
    assert east[0, 7] == 1.0 and east[7, 7] == 1.0 and east[7, 0] == 0.0
    half = coverage(GRID, lambda x, y: y < 2.5)
    assert np.all(half[0] == 0.5) and np.all(half[1:] == 0.0)


def test_coverage_follows_the_grid_origin_and_shape():
    g = Grid(width=3, height=2, resolution=10.0, origin_x=-15.0, origin_y=100.0)
    cov = coverage(g, lambda x, y: (x < -5.0) & (y >= 110.0))
    assert cov.tolist() == [[0, 0, 0], [1, 0, 0]]


@pytest.mark.parametrize('region, area', [
    (Circle((20.0, 20.0), 5.0, 1, 1), math.pi * 25),
    (Circle((17.3, 22.9), 8.0, 1, 1), math.pi * 64),
    (Capsule((8.0, 8.0), (30.0, 28.0), 4.0, 1, 1),
     math.pi * 16 + 8 * math.hypot(22, 20)),
])
def test_overlap_keeps_the_true_area(region, area):
    assert overlap(GRID, region).sum() * 25 == pytest.approx(area, rel=0.01)


def test_cell_values_are_fractions_of_the_cell():
    # Centred on cell (4, 4), radius 2 m: wholly inside that 5 m cell.
    o = overlap(GRID, Circle((22.5, 22.5), 2.0, 1, 1))
    assert o[4, 4] == pytest.approx(math.pi * 4 / 25, abs=0.02)   # the lattice's error
    assert np.count_nonzero(o) == 1
    # A circle over a corner splits evenly between its four cells.
    o = overlap(GRID, Circle((20.0, 20.0), 2.0, 1, 1))
    assert o[3, 3] == o[3, 4] == o[4, 3] == o[4, 4] > 0


def test_swallowed_cells_are_full():
    assert overlap(GRID, Circle((20.0, 20.0), 9.0, 1, 1))[4, 4] == 1.0


def test_no_region_on_the_map_vanishes():
    # The rule chosen against cell-centre, which lost 64 % at 1.5 m (D44).
    rng = np.random.default_rng(0)
    for x, y in rng.uniform(0, 40, (200, 2)):
        assert overlap(GRID, Circle((x, y), 0.01, 1, 1)).any()


def test_the_targets_own_cell_is_always_marked():
    rng = np.random.default_rng(1)
    for _ in range(300):
        radius = rng.choice([1.5, 4.0, 8.0])
        c = rng.uniform(radius, 40 - radius, 2)
        a, d = rng.uniform(0, 2 * math.pi), 0.95 * radius * math.sqrt(rng.random())
        col, row = GRID.cell_of(c[0] + d * math.cos(a), c[1] + d * math.sin(a))
        assert overlap(GRID, Circle(tuple(c), radius, 1, 1))[row, col] > 0


def test_a_capsule_with_equal_ends_is_a_circle():
    p = (12.0, 31.0)
    assert np.array_equal(overlap(GRID, Capsule(p, p, 6.0, 1, 1)),
                          overlap(GRID, Circle(p, 6.0, 1, 1)))


def test_a_region_past_the_edge_is_cut_not_moved():
    o = overlap(GRID, Circle((-2.0, 20.0), 4.0, 1, 1))     # centre off the map
    assert o[:, 0].any() and not o[:, 1:].any()


# ---- Several targets (D46) ------------------------------------------------

def test_value_is_weight_times_confidence():
    assert rasterize(GRID, [Circle((20, 20), 9.0, 0.5, 0.8)]).max() == \
        pytest.approx(0.4)


def test_overlaps_take_the_maximum_and_duplicates_change_nothing():
    a, b = Circle((18, 20), 9.0, 1.0, 0.6), Circle((22, 20), 9.0, 1.0, 0.6)
    both = rasterize(GRID, [a, b])
    assert both.max() == pytest.approx(0.6)                # noisy-OR would be 0.84
    assert np.array_equal(both, np.maximum(rasterize(GRID, [a]), rasterize(GRID, [b])))
    assert np.array_equal(rasterize(GRID, [a, a, a]), rasterize(GRID, [a]))


def test_more_places_weaken_none():
    here = Circle((8, 8), 6.0, 1.0, 1.0)
    others = [Circle((32, 8), 6.0, 1.0, 1.0), Circle((20, 32), 6.0, 1.0, 1.0)]
    alone, together = rasterize(GRID, [here]), rasterize(GRID, [here, *others])
    assert together[1, 1] == alone[1, 1] == 1.0


def test_order_does_not_matter():
    rs = [Circle((8, 8), 6, 0.9, 0.5), Capsule((8, 8), (30, 30), 4, 0.7, 0.9)]
    assert np.array_equal(rasterize(GRID, rs), rasterize(GRID, rs[::-1]))


# ---- Starting belief (D45) ------------------------------------------------

def test_blend_matches_the_roadmap_formula():
    u = np.array([[0.0, 0.3, 1.0]])
    prior = 0.5 + 0.5 * u
    for alpha in (0.0, 0.5, 0.7):
        assert starting_belief(u, alpha) == \
            pytest.approx(alpha * prior + (1 - alpha) * 0.5)
    assert starting_belief(u).tolist() == [[0.5, 0.575, 0.75]]


def test_the_clue_never_lowers_a_cell():
    rng = np.random.default_rng(2)
    for _ in range(50):
        rs = [Circle(tuple(rng.uniform(0, 40, 2)), rng.uniform(1, 20),
                     rng.random(), rng.random()) for _ in range(3)]
        s = starting_belief(rasterize(GRID, rs), 0.7)
        assert s.min() >= 0.5 and s.max() <= 0.85


@pytest.mark.parametrize('alpha', [-0.1, 0.71, 1.0, math.nan])
def test_alpha_is_capped(alpha):
    with pytest.raises(ValueError):
        starting_belief(np.zeros((8, 8)), alpha)


# ---- The belief map with a start (D45) ------------------------------------

def frame(seen, hits=()):
    return Frame(set(seen), dict(hits))


def test_a_uniform_start_is_exactly_the_map_without_one():
    rng = np.random.default_rng(3)
    plain, uniform = BeliefMap(GRID), BeliefMap(GRID, start=np.full((8, 8), 0.5))
    for k in range(200):
        cells = [tuple(int(v) for v in c) for c in rng.integers(0, 6, (5, 2))]
        f = frame(cells, {cells[0]: 0.8} if rng.random() < 0.3 else {})
        for bm in (plain, uniform):
            bm.update('a', k * 0.5, (0, 0, 8), f)
        assert plain.occupancy_data() == uniform.occupancy_data()
    assert -1 in plain.occupancy_data()       # unseen cells still read unknown


def test_start_is_published_until_a_camera_looks():
    start = np.full((8, 8), 0.5)
    start[2, 5] = 0.75                                     # row 2, col 5
    bm = BeliefMap(GRID, start=start)
    data = bm.occupancy_data()
    assert data[2 * 8 + 5] == 75 and data.count(-1) == 63
    assert bm.probability((5, 2)) == pytest.approx(0.75)
    assert bm.probability((2, 5)) is None
    assert np.isnan(bm.logodds).all()         # the prior is not a look


def test_first_evidence_starts_from_the_prior():
    start = np.full((8, 8), 0.5)
    start[0, 0] = 0.75
    p = BeliefParams(look_cap=0.0)
    bm = BeliefMap(GRID, p, start)
    bm.update('a', 0.0, (0, 0, 8), frame([(0, 0), (1, 0)], {(0, 0): 1.0}))
    assert bm.logodds[0, 0] == pytest.approx(math.log(3) + p.l_occ)
    assert bm.logodds[0, 1] == pytest.approx(p.l_free)


def look(bm, n, hit=False, t0=0.0):
    """n whole looks at cell (0, 0): 8 frames at 2 Hz, 10 s apart."""
    for k in range(n):
        for j in range(8):
            bm.update('a', t0 + k * 10 + j * 0.5, (0, 0, 8),
                      frame([(0, 0)], {(0, 0): 0.8} if hit else {}))


def test_a_wrong_prior_fades_in_two_looks():
    bm = BeliefMap(GRID, start=np.full((8, 8), 0.75))
    look(bm, 1)
    assert 0.5 < bm.probability((0, 0)) < 0.6              # 75 -> 56
    look(bm, 1, t0=10.0)
    assert bm.probability((0, 0)) < 0.4                    # -> 35


def test_with_negative_evidence_off_it_never_fades():
    # The fourth eval arm (roadmap): p_miss 0.5, hits unchanged.
    off = BeliefParams(p_miss=0.5)
    bm = BeliefMap(GRID, off, np.full((8, 8), 0.75))
    look(bm, 10)
    assert bm.probability((0, 0)) == pytest.approx(0.75)
    look(bm, 1, hit=True, t0=100.0)
    assert bm.probability((0, 0)) > 0.8


def test_one_look_still_opens_a_target_nobody_mentioned():
    # Raise only: an unmentioned cell starts at 0.5, so one look at a target
    # puts it above D14's 0.5. With an outside level of 0 it would read 0.44.
    start = np.full((8, 8), 0.5)
    start[7, 7] = 0.75
    bm = BeliefMap(GRID, start=start)
    look(bm, 1, hit=True)
    assert bm.probability((0, 0)) == pytest.approx(0.70, abs=0.01)


def test_confirmed_sectors_read_0_whatever_the_start():
    bm = BeliefMap(GRID, start=np.full((8, 8), 0.75))
    bm.confirmed[(3, 1)] = ('a', 0.0)
    assert bm.occupancy_data()[1 * 8 + 3] == 0


@pytest.mark.parametrize('start', [
    np.full((8, 7), 0.5), np.full(64, 0.5), np.full((8, 8), 1.0),
    np.full((8, 8), 0.0), np.full((8, 8), np.nan)])
def test_a_bad_start_is_refused(start):
    with pytest.raises(ValueError):
        BeliefMap(GRID, start=start)


# ---- Clue to prior --------------------------------------------------------

def test_the_whole_chain(world):
    p = make_prior('c', world, GRID, Fake(text(circle(radius=9, confidence=0.8))))
    assert not p.failed and p.warnings == () and len(p.regions) == 1
    assert p.start[4, 4] == pytest.approx(0.5 + 0.25 * 0.8)
    assert p.start[0, 0] == 0.5
    BeliefMap(GRID, start=p.start)                          # accepted as it is


def test_one_retry_then_it_works(world):
    f = Fake('nonsense', text(circle()))
    p = make_prior('c', world, GRID, f)
    assert f.calls == 2 and not p.failed and 'attempt 1' in p.warnings[0]


@pytest.mark.parametrize('bad', [
    'nonsense', text(circle(center=[500, 500])), RuntimeError('network'),
    TimeoutError()])
def test_two_failures_fly_uniform(world, bad):
    f = Fake(bad)
    p = make_prior('c', world, GRID, f)
    assert f.calls == 2 and p.failed and np.all(p.start == 0.5)
    assert p.regions == () and 'uniform' in p.warnings[-1]


def test_strict_raises_instead(world):
    with pytest.raises(PriorError):
        make_prior('c', world, GRID, Fake('nonsense'), strict=True)
    with pytest.raises(NoRecording):
        make_prior('c', world, GRID, ReplayProvider([]), strict=True)


def test_an_empty_answer_is_uniform_but_not_a_failure(world):
    p = make_prior('c', world, GRID, Fake('{"regions": []}'), strict=True)
    assert not p.failed and np.all(p.start == 0.5)


def test_off_map_and_tiny_regions_are_noted(world):
    p = make_prior('c', world, GRID, Fake(
        text(circle(center=[-90, 0]), circle(radius=0.2))))
    assert len(p.regions) == 1 and not p.failed
    assert 'off the map' in p.warnings[0] and 'widened' in p.warnings[1]


def test_a_grid_the_world_does_not_describe_is_a_launch_error(world):
    with pytest.raises(ValueError, match='covers'):
        make_prior('c', world, Grid(16, 16), Fake(text(circle())))


def test_a_bad_alpha_fails_before_asking(world):
    f = Fake(text(circle()))
    with pytest.raises(ValueError):
        make_prior('c', world, GRID, f, alpha=0.9)
    assert f.calls == 0


# ---- Landmarks ------------------------------------------------------------

def test_the_committed_world_loads_and_matches_the_default_grid(world):
    world.check_grid(GRID)
    assert len(world.landmarks) == 8 and world.landmark('hut').type == 'point'
    with pytest.raises(KeyError):
        world.landmark('lake')


GOOD = """
world: w
bounds: [0, 0, 40, 40]
landmarks:
  - {name: hut, type: point, points: [[3, 4]]}
"""


def test_fingerprint_ignores_layout_and_sees_content():
    base = lmk.parse_world(GOOD).fingerprint
    assert lmk.parse_world('# note\n' + GOOD.replace('[[3, 4]]', '[[3.0,  4]]')
                           ).fingerprint == base
    for a, b in (('3, 4', '3, 5'), ('hut', 'shed'), ('40, 40', '80, 80'),
                 ('[[3, 4]]', '[[3, 4]], description: old')):
        assert lmk.parse_world(GOOD.replace(a, b)).fingerprint != base


@pytest.mark.parametrize('a, b', [
    ('world: w', 'world: 3'), ('world: w\n', ''), ('world: w', 'world: w\nx: 1'),
    ('[0, 0, 40, 40]', '[0, 0, 40]'), ('[0, 0, 40, 40]', '[40, 0, 0, 40]'),
    ('[[3, 4]]', '[[3, 44]]'), ('[[3, 4]]', '[[3]]'), ('[[3, 4]]', '[[3, .nan]]'),
    ('[[3, 4]]', '[[3, 4], [5, 6]]'), ('[[3, 4]]', '[3, 4]'),
    ('[[3, 4]]', '[[yes, 4]]'), ('type: point', 'type: lake'),
    ('type: point', 'type: polygon'), ('name: hut', 'name: 7'),
    ('type: point', 'type: point, colour: red'),
    ('  - {', '  - {name: hut, type: point, points: [[1, 1]]}\n  - {'),
])
def test_a_bad_landmark_file_is_an_error(a, b):
    assert a in GOOD
    with pytest.raises(ValueError):
        lmk.parse_world(GOOD.replace(a, b))


# ---- The misleading-clue rule (D47) ---------------------------------------

def make_world(*landmarks):
    return lmk.World('w', (0.0, 0.0, 40.0, 40.0), tuple(
        lmk.Landmark(n, t, tuple(p)) for n, t, p in landmarks))


def test_near_landmark_covers_lines_and_polygon_insides():
    line = lmk.Landmark('l', 'polyline', ((2.5, 2.5), (37.5, 2.5)))
    near = lmk.near_landmark(GRID, line, 2.0)
    assert near[0].min() > 0 and not near[1:].any()
    square = lmk.Landmark('s', 'polygon', ((10., 10.), (30., 10.), (30., 30.), (10., 30.)))
    near = lmk.near_landmark(GRID, square, 1.0)
    assert np.all(near[2:6, 2:6] == 1.0)        # the inside, not only the rim
    assert 0 < near[1, 3] < 0.3 and near[0, 3] == 0.0


def test_wrong_landmarks_keep_a_one_cell_gap():
    # Target in cell (1, 1), so cells 0-2 must stay clear: x, y < 15.
    w = make_world(('true', 'point', [(7., 7.)]),
                   ('touching', 'point', [(18.5, 7.)]),    # reaches x = 14.5
                   ('clear', 'point', [(19.5, 7.)]),       # reaches x = 15.5
                   ('line', 'polyline', [(35., 35.), (12., 12.)]),
                   ('around', 'polygon', [(1., 1.), (39., 1.), (39., 39.), (1., 39.)]),
                   ('far', 'point', [(35., 35.)]))
    assert lmk.wrong_landmarks(GRID, w, ['true'], [(7., 7.)], 4.0) == ['clear', 'far']


def test_no_targets_true_landmark_is_ever_wrong():
    w = make_world(('a', 'point', [(5., 5.)]), ('b', 'point', [(35., 35.)]),
                   ('c', 'point', [(35., 5.)]))
    # The second target strayed: b is clear of both, but it is a true landmark.
    got = lmk.wrong_landmarks(GRID, w, ['a', 'b'], [(5., 5.), (7., 5.)], 3.0)
    assert got == ['c']


def test_wrong_landmarks_refuses_typos_and_stray_targets(world):
    with pytest.raises(KeyError):
        lmk.wrong_landmarks(GRID, world, ['hutt'], [(33., 6.)], 4.0)
    with pytest.raises(ValueError):
        lmk.wrong_landmarks(GRID, world, ['hut'], [(33., -6.)], 4.0)


def test_a_misleading_prior_raises_nothing_near_a_target(world):
    target = (33.0, 6.0)
    swaps = set()
    for seed in range(30):
        swap = lmk.misleading_swap(GRID, world, ['hut'], [target], 5.0,
                                   np.random.default_rng(seed))
        swaps.add(swap['hut'])
        near = lmk.near_landmark(GRID, world.landmark(swap['hut']), 5.0)
        col, row = GRID.cell_of(*target)
        assert not near[max(0, row - 1):row + 2, col - 1:col + 2].any()
    assert len(swaps) > 1                       # drawn, not always the same one


def test_the_swap_is_seeded_distinct_and_complete():
    w = make_world(*[(f'p{i}', 'point', [(2.5 + 11 * (i % 4), 2.5 + 11 * (i // 4))])
                     for i in range(12)])
    targets = [(2.5, 2.5), (13.5, 2.5), (2.5, 3.5)]
    names = ['p0', 'p1', 'p0']                  # two targets share a landmark
    draw = lambda: lmk.misleading_swap(  # noqa: E731
        GRID, w, names, targets, 2.0, np.random.default_rng(5))
    swap = draw()
    assert swap == draw() and set(swap) == {'p0', 'p1'}
    assert len(set(swap.values())) == 2 and not set(swap.values()) & {'p0', 'p1'}


def test_too_few_wrong_landmarks_gives_none():
    w = make_world(('a', 'point', [(5., 5.)]), ('b', 'point', [(8., 5.)]))
    assert lmk.misleading_swap(GRID, w, ['a'], [(5., 5.)], 3.0,
                               np.random.default_rng(0)) is None


# ---- Replay (D48) ---------------------------------------------------------

def rec(w, **kw):
    return Recording(**{**dict(world=w.name, clue='Near the hut.',
                               landmarks=w.fingerprint, source='hand-written',
                               model='', date='', output=text(circle())), **kw})


def test_replay_ignores_case_and_spacing_only(world):
    r = ReplayProvider([rec(world)])
    assert r('  near   THE\nhut. ', world) == text(circle())
    for other in ('Near the hut', 'Not near the hut.', 'Near the huts.'):
        with pytest.raises(NoRecording):
            r(other, world)
    assert clue_key('A  b') == clue_key(' a B')


def test_replay_misses_other_worlds_and_stale_landmarks(world):
    for stale in (rec(world, world='valley'), rec(world, landmarks='0' * 16)):
        with pytest.raises(NoRecording):
            ReplayProvider([stale])('Near the hut.', world)


def test_a_miss_flies_uniform_without_a_retry(world):
    class Counting(ReplayProvider):
        calls = 0

        def __call__(self, clue, w):
            Counting.calls += 1
            return super().__call__(clue, w)

    p = make_prior('Unheard of.', world, GRID, Counting([rec(world)]))
    assert p.failed and np.all(p.start == 0.5) and Counting.calls == 1
    assert 'no recording' in p.warnings[0]


def test_two_recordings_of_one_clue_are_an_error(world):
    with pytest.raises(ValueError, match='two recordings'):
        ReplayProvider([rec(world), rec(world, clue='near the HUT.')])


def test_the_eval_refuses_hand_written_recordings(world):
    live = rec(world, clue='b', source='live', model='m', date='2026-12-10')
    ReplayProvider([live], require_live=True)
    with pytest.raises(ValueError, match='not live'):
        ReplayProvider([live, rec(world)], require_live=True)


@pytest.mark.parametrize('change', [
    {'source': 'guessed'}, {'source': 'live'}, {'model': 'm', 'date': 'd'},
    {'output': {'regions': []}}, {'extra': 'x'}])
def test_a_bad_recordings_file_is_an_error(world, change):
    item = {**rec(world)._asdict(), **change}
    with pytest.raises(ValueError):
        parse_recordings(json.dumps({'recordings': [item]}))


def test_committed_recordings_replay_against_the_committed_world(world):
    # Guards the pair: editing landmarks.yaml orphans every recording.
    recs = parse_recordings(RECORDINGS.read_text())
    replay = load_replay(RECORDINGS)
    assert recs
    for r in recs:
        p = make_prior(r.clue, world, GRID, replay, strict=True)
        assert not p.failed and p.warnings == ()


def test_a_replayed_clue_becomes_a_starting_belief(world):
    p = make_prior('One hiker, last seen near the hut.', world, GRID,
                   load_replay(RECORDINGS))
    data = BeliefMap(GRID, start=p.start).occupancy_data()
    col, row = GRID.cell_of(34, 7)
    assert data[row * 8 + col] == 74            # 0.5 + 0.25 * 0.95
    assert data.count(-1) > 40                  # the rest stays unknown


# ---- The hard rule --------------------------------------------------------

def test_the_prior_code_cannot_reach_a_drone():
    core = ROOT / 'src/uav_mapping/uav_mapping/core'
    for name in ('prior.py', 'landmarks.py', 'belief.py', 'geometry.py'):
        src = (core / name).read_text()
        for word in ('rclpy', '_msgs', 'cmd_gps_pose', 'import uav_planning',
                     'from uav_planning'):
            assert word not in src, f"{name} mentions {word}"
