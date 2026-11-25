"""The 2D eval: scenarios, the simulator, the arms, the summaries. No ROS and
no language model: priors come from a stand-in that answers by rule."""

import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from uav_eval import run as cli
from uav_eval.core import arms, scenario as sc, sim
from uav_mapping.core.geometry import Grid
from uav_mapping.core.landmarks import landmark_distance, near_landmark
from uav_mapping.core.prior import NoRecording, live_recording, save_recording
from uav_perception.core.noise import NoiseModel

ROOT = Path(__file__).resolve().parents[3]
WORLD, GRID, RECORDINGS = cli.load(str(ROOT / 'worlds'), 'valley')
CLEAN = NoiseModel(p_miss=0.0, p_false=0.0, pixel_sigma=0.0)


def answer(clue, world=WORLD, radius=8):
    """What a faultless reader of the template would return."""
    regions = []
    for lm in world.landmarks:
        if f"near the {lm.name}" in clue:
            x, y = np.mean(lm.points, axis=0)
            regions.append({'shape': 'circle', 'center': [float(x), float(y)],
                            'radius': radius, 'weight': 1.0, 'confidence': 1.0})
    return json.dumps({'regions': regions})


# ---- Scenarios --------------------------------------------------------------

def test_a_scenario_is_its_seed():
    assert sc.draw(WORLD, GRID, 7) == sc.draw(WORLD, GRID, 7)
    assert sc.draw(WORLD, GRID, 7) != sc.draw(WORLD, GRID, 8)


@pytest.mark.parametrize('seed', range(40))
def test_every_target_is_near_its_landmark_and_clear_of_the_wrong_ones(seed):
    s = sc.draw(WORLD, GRID, seed)
    assert 1 <= len(s.targets) <= 3 and len(set(s.names)) == len(s.names)
    for name, (x, y) in zip(s.names, s.targets):
        assert landmark_distance(WORLD.landmark(name), x, y) <= sc.NEAR
        assert 1 <= x <= 119 and 1 <= y <= 119
    assert set(s.swap) == set(s.names) and len(set(s.swap.values())) == len(s.names)
    assert not set(s.swap.values()) & set(s.names)
    for wrong in s.swap.values():
        near = near_landmark(GRID, WORLD.landmark(wrong), sc.CLEAR)
        for t in s.targets:
            col, row = GRID.cell_of(*t)
            assert not near[max(row - 1, 0):row + 2, max(col - 1, 0):col + 2].any()


def test_the_seeds_use_one_two_and_three_targets_and_every_landmark():
    drawn = [sc.draw(WORLD, GRID, s) for s in range(1000, 1025)]
    assert {len(s.targets) for s in drawn} == {1, 2, 3}
    assert {n for s in drawn for n in s.names} == {lm.name for lm in WORLD.landmarks}


def test_both_clues_are_one_template():
    assert sc.clue_text(['hut']) == 'One hiker is missing, last seen near the hut.'
    assert sc.clue_text(['hut', 'well', 'creek']) == \
        ('Three hikers are missing: one last seen near the hut, one near the '
         'well and one near the creek.')
    s = sc.draw(WORLD, GRID, 3)
    text = s.accurate
    for name in s.names:
        text = text.replace(f'near the {name}', f'near the {s.swap[name]}')
    assert text == s.misleading


def test_no_possible_swap_is_an_error(monkeypatch):
    monkeypatch.setattr(sc, 'misleading_swap', lambda *a: None)
    monkeypatch.setattr(sc, 'MAX_REDRAWS', 3)
    with pytest.raises(RuntimeError):
        sc.draw(WORLD, GRID, 0)


def test_a_draw_without_a_swap_is_drawn_again_and_counted(monkeypatch):
    real, calls = sc.misleading_swap, []

    def second_time(*a):
        calls.append(a)
        return None if len(calls) == 1 else real(*a)
    monkeypatch.setattr(sc, 'misleading_swap', second_time)
    assert sc.draw(WORLD, GRID, 0).redraws == 1


# ---- The simulator ----------------------------------------------------------

def test_a_run_is_its_seed():
    s = sc.draw(WORLD, GRID, 1001)
    a = sim.simulate(GRID, s.targets, None, 5)
    assert a == sim.simulate(GRID, s.targets, None, 5)
    assert a != sim.simulate(GRID, s.targets, None, 6)


def test_a_clean_uniform_run_finds_everything_covers_everything_and_ends_done():
    s = sc.draw(WORLD, GRID, 1001)
    r = sim.simulate(GRID, s.targets, None, 0, sim.SimParams(noise=CLEAN))
    assert None not in r.found and r.done and r.coverage == 1.0
    assert r.extra_confirms == 0 and r.path == []
    assert r.first == min(r.found) and r.all == max(r.found)
    assert r.end >= r.sweep_done + 30.0         # D14's hold


def test_nobody_is_found_before_anyone_could_have_seen_them():
    target = (110.0, 110.0)
    r = sim.simulate(GRID, [target], None, 0, sim.SimParams(noise=CLEAN))
    nearest = math.dist((12.5, -5.0), target) - 14.0 * math.hypot(320, 240) / 205.47
    assert r.found[0] > nearest / 5.0


def test_a_target_no_detector_reports_is_censored():
    r = sim.simulate(GRID, [(60.0, 60.0)], None, 0, sim.SimParams(
        noise=replace(CLEAN, p_miss=1.0), timeout=300.0))
    assert r.found == [None] and r.first is None and r.all is None
    assert r.confirmed == [None]


def test_a_false_alarm_is_never_a_find():
    r = sim.simulate(GRID, [(60.0, 60.0)], None, 0, sim.SimParams(
        noise=NoiseModel(p_miss=1.0, p_false=1.0), timeout=120.0, found_radius=200.0))
    assert r.found == [None]


def test_without_negative_evidence_the_mission_only_ends_at_the_timeout():
    p = sim.SimParams(timeout=240.0, negative_evidence=False)
    r = sim.simulate(GRID, [(60.0, 60.0)], None, 0, p)
    assert not r.done and r.end >= 240.0


def test_the_leader_covers_everything_whatever_the_clue_says():
    start = np.full((24, 24), 0.5)
    start[20:, 20:] = 0.75
    r = sim.simulate(GRID, [(5.0, 5.0)], start, 0,
                     sim.SimParams(noise=CLEAN, timeout=400.0, prior_lanes=True))
    assert r.path and sorted(r.path) == list(range(len(r.path)))
    assert r.path[0] == max(r.path) and r.coverage == 1.0 and None not in r.found
    off = sim.simulate(GRID, [(5.0, 5.0)], start, 0,
                       sim.SimParams(noise=CLEAN))
    assert off.path == [] and off.coverage == 1.0        # D53: off by default


# ---- Arms -------------------------------------------------------------------

def test_an_accurate_clue_covers_its_targets_and_a_misleading_one_none():
    s = sc.draw(WORLD, GRID, 1000)
    assert s.names == ('well',)
    runs = {a: arms.run_arm(a, s, WORLD, GRID, answer,
                            sim.SimParams(timeout=60.0)) for a in arms.ARMS}
    assert runs['accurate'].covered == [True]
    for a in ('uniform', 'misleading', 'fixed'):
        assert runs[a].covered == [False]
    assert runs['uniform'].regions == 0 and runs['fixed'].regions == 1


def test_the_fixed_arm_is_the_misleading_one_without_negative_evidence():
    s = sc.draw(WORLD, GRID, 1000)
    p = sim.SimParams(timeout=90.0)
    start = arms.prior_for('misleading', s, WORLD, GRID, answer).start
    assert arms.run_arm('fixed', s, WORLD, GRID, answer, p).result == sim.simulate(
        GRID, s.targets, start, s.seed, replace(p, negative_evidence=False))
    assert arms.run_arm('misleading', s, WORLD, GRID, answer, p).result == \
        sim.simulate(GRID, s.targets, start, s.seed, p)


def test_an_unusable_answer_raises_instead_of_flying_uniform():
    s = sc.draw(WORLD, GRID, 1000)
    with pytest.raises(ValueError):
        arms.run_arm('accurate', s, WORLD, GRID, lambda c, w: 'nope')
    with pytest.raises(ValueError):
        arms.run_arm('nonsense', s, WORLD, GRID, answer)


def _recordings(path, clue, **changes):
    rec = live_recording(WORLD, clue, 'gemini-x', answer(clue), '2026-11-25')
    save_recording(path, rec._replace(**changes))


def test_only_live_recordings_of_one_model_reach_a_run(tmp_path):
    s, path = sc.draw(WORLD, GRID, 1000), tmp_path / 'r.json'
    _recordings(path, s.accurate)
    assert arms.prior_for('accurate', s, WORLD, GRID, arms.strict_replay(path))
    with pytest.raises(NoRecording):
        arms.prior_for('misleading', s, WORLD, GRID, arms.strict_replay(path))
    _recordings(path, s.misleading, model='gemini-y')
    with pytest.raises(ValueError, match='several models'):
        arms.strict_replay(path)
    hand = tmp_path / 'hand.json'
    _recordings(hand, s.accurate, source='hand-written', model='', date='')
    with pytest.raises(ValueError, match='not live'):
        arms.strict_replay(hand)


# Pilot seeds on the committed recordings. A change here is a change to what the
# eval measures (planner, belief, noise, scenarios or sim): update it knowingly.
PINNED = {
    1000: {'uniform': [26.5], 'accurate': [18.5], 'misleading': [38.0]},
    1001: {'uniform': [13.5, 24.0, 38.5], 'accurate': [14.5, 17.5, 31.0],
           'misleading': [44.5, 41.5, 33.5]},
    1002: {'uniform': [20.5, 21.5], 'accurate': [15.5, 21.0],
           'misleading': [43.5, 22.5]},
}


@pytest.mark.parametrize('seed', PINNED)
def test_three_pilot_seeds_still_give_the_same_times(seed):
    provider, s = arms.strict_replay(RECORDINGS), sc.draw(WORLD, GRID, seed)
    for arm, times in PINNED[seed].items():
        run = arms.run_arm(arm, s, WORLD, GRID, provider)
        assert run.result.found == times, arm
        assert run.covered == [arm == 'accurate'] * len(s.targets)


# The reported results. A failure means the code no longer produces what was
# published: either that change is a mistake, or the results need a new run and
# a new write-up (docs/eval-results.md). Never edit the JSON to match.
RESULTS = ROOT / 'docs/results'


@pytest.mark.parametrize('name, lanes', [('eval', False), ('eval_lanes', True)])
def test_the_reported_results_still_reproduce(name, lanes):
    rows = json.loads((RESULTS / f'{name}.json').read_text())
    assert {(d['arm'], d['seed']) for d in rows} == \
        {(a, s) for a in arms.ARMS for s in range(20)}
    provider = arms.strict_replay(RECORDINGS)
    for d in (d for d in rows if d['seed'] in (0, 13) and d['arm'] != 'fixed'):
        run = arms.run_arm(d['arm'], sc.draw(WORLD, GRID, d['seed']), WORLD, GRID,
                           provider, sim.SimParams(prior_lanes=lanes))
        assert json.loads(json.dumps(cli.to_json({d['arm']: [run]})))[0] == d


@pytest.mark.parametrize('name', ['eval', 'eval_lanes'])
def test_the_published_table_is_the_one_the_json_gives(name):
    rows = json.loads((RESULTS / f'{name}.json').read_text())
    runs = {a: [arms.Run(a, SimpleNamespace(seed=d['seed']), d['covered'], d['regions'],
                         sim.Result(**{k: d[k] for k in sim.Result.__dataclass_fields__}))
                for d in rows if d['arm'] == a] for a in arms.ARMS}
    assert (RESULTS / f'{name}_table.md').read_text().startswith(cli.table(runs) + '\n')


# ---- Summaries --------------------------------------------------------------

def test_quantiles_put_censored_runs_last():
    assert arms.quantile([3.0, 1.0, 2.0, None], 0.5) == 2.0
    assert arms.quantile([1.0, None, None], 0.5) == math.inf
    assert arms.quantile([1.0, 2.0, 3.0, None], 0.75) == 3.0
    assert arms.quantile([5.0], 0.25) == 5.0


def _run(seed, *found, arm='a', by='iris_2', coverage=1.0):
    r = sim.Result(list(found), [None if t is None else by for t in found],
                   list(found), 0, 900.0, False, coverage, 206.0)
    return arms.Run(arm, SimpleNamespace(seed=seed, redraws=0),
                    [True] * len(found), 1, r)


def test_a_summary_counts_censored_runs_and_missed_targets():
    s = arms.summary([_run(0, 1.0, 2.0), _run(1, 4.0, None), _run(2, None, coverage=0.5)])
    assert s['targets'] == 5 and s['targets_missed'] == 2
    assert s['first'][1] == 4.0 and s['first_censored'] == 1
    assert s['all'][1] == math.inf and s['all_censored'] == 2
    assert s['coverage_min'] == 0.5 and s['found_by_leader'] == 0


def test_paired_differences_go_seed_by_seed():
    runs = {'a': [_run(0, 1.0), _run(1, 9.0), _run(2, None), _run(3, None)],
            'b': [_run(0, 3.0), _run(1, 5.0), _run(2, 1.0), _run(3, None)]}
    p = arms.paired(runs, 'a', 'b', 'first')
    assert (p['faster'], p['slower'], p['ties']) == (1, 2, 1)
    assert p['median_diff'] == 1.0 and p['both_found'] == 2
    with pytest.raises(AssertionError):
        arms.paired({'a': [_run(0, 1.0)], 'b': [_run(1, 1.0)]}, 'a', 'b', 'first')


def test_the_sign_test_is_the_exact_binomial():
    assert arms.sign_p(5, 0) == arms.sign_p(0, 5) == 2 / 32
    assert arms.sign_p(1, 4) == 12 / 32
    assert arms.sign_p(3, 3) == 1.0 and arms.sign_p(0, 0) == 1.0


# ---- The command line -------------------------------------------------------

def test_seed_lists():
    assert cli.parse_seeds('0-3,10') == [0, 1, 2, 3, 10]
    assert cli.parse_seeds('5') == [5]


class FakeProvider:
    model = 'gemini-x'

    def __init__(self, fail_after=99):
        self.outputs, self.usage, self._left = [], [], fail_after

    def __call__(self, clue, world):
        if not self._left:
            raise RuntimeError('quota')
        self._left -= 1
        self.outputs.append(answer(clue))
        self.usage.append({'input': 1})
        return self.outputs[-1]


def test_recording_stops_at_a_failure_and_never_asks_twice(tmp_path):
    path, wanted = tmp_path / 'r.json', cli.clues(WORLD, GRID, [1000, 1001])
    assert len(wanted) == 4
    first = FakeProvider(fail_after=3)
    with pytest.raises(RuntimeError, match='quota'):
        cli.record(WORLD, GRID, path, wanted, first, '2026-11-25')
    assert len(first.outputs) == 3
    second = FakeProvider()
    assert cli.record(WORLD, GRID, path, wanted, second, '2026-11-25') == 1
    assert cli.record(WORLD, GRID, path, wanted, FakeProvider(0), '2026-11-25') == 0
    arms.strict_replay(path)


def test_the_table_prints_a_censored_median():
    runs = {a: [_run(0, None, arm=a), _run(1, 2.0, arm=a)] for a in arms.ARMS}
    text = cli.table(runs)
    assert 'cens.' in text and all(f'| {a} |' in text for a in arms.ARMS)


def test_the_table_prints_leader_finds_and_extra_confirmations():
    r = _run(0, 1.0, 2.0, arm='uniform')
    r = r._replace(result=replace(r.result, found_by=['iris_1', 'iris_2'],
                                  extra_confirms=7))
    runs = {a: [r] for a in arms.ARMS}
    row = next(x for x in cli.table(runs).split('\n') if x.startswith('| uniform'))
    assert '| 1/2 | 7 |' in row


def test_a_results_file_compares_with_the_runs_it_came_from(tmp_path):
    def rs(*found):
        real = sc.draw(WORLD, GRID, 1001)
        return [_run(1001, *found)._replace(scenario=real)]
    old = cli.from_json(json.loads(json.dumps(cli.to_json({'uniform': rs(5.0, 9.0),
                                                           'fixed': rs(2.0, None)}))))
    text = cli.against({'uniform': rs(4.0, 12.0), 'fixed': rs(2.0, 3.0)}, old)
    rows = [x for x in text.split('\n') if 'this - that' in x]
    assert '| first | 1 | 0 | 0 | 1.000 | -1.0 (1) |' in rows[0]
    assert '| all | 0 | 1 | 0 | 1.000 | +3.0 (1) |' in rows[1]
    assert '| first | 0 | 0 | 1 |' in rows[2]
    assert '| all | 1 | 0 | 0 | 1.000 | +nan (0) |' in rows[3]


def test_the_matrix_runs_every_arm_on_every_seed(tmp_path):
    (tmp_path / 'valley').mkdir()
    for name in ('landmarks.yaml',):
        (tmp_path / 'valley' / name).write_text((ROOT / 'worlds/valley' / name).read_text())
    path = tmp_path / 'valley/recordings.json'
    cli.record(WORLD, GRID, path, cli.clues(WORLD, GRID, [1000]), FakeProvider(), 'd')
    runs = cli.matrix(str(tmp_path), 'valley', [1000],
                      sim.SimParams(timeout=30.0), 2)
    assert [len(runs[a]) for a in arms.ARMS] == [1, 1, 1, 1]
