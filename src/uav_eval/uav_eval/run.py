"""The eval's command line (no ROS). From the repo root:

  python3 -m uav_eval.run clues  --seeds 0-19     the clues the seeds need
  python3 -m uav_eval.run record --seeds 0-19 --model gemini-3.8-flash
  python3 -m uav_eval.run run    --seeds 0-19 --out results.json

`record` asks the model once per clue that has no recording yet, and stops at
the first failure, so it can be run again (a daily quota) without asking twice.
It is the only command that needs a key or the network. `run` reads recordings
and nothing else.

Final seeds are 0-19. Use 1000 and up for pilots, so that nothing is tuned on
the seeds that are reported.
"""

import argparse
import datetime
import json
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from multiprocessing import get_context
from pathlib import Path
from types import SimpleNamespace
from typing import List

from uav_mapping.core.geometry import Grid
from uav_mapping.core.landmarks import load_world
from uav_mapping.core.live import LiveProvider
from uav_mapping.core.prior import (NoRecording, live_recording, load_replay,
                                    make_prior, save_recording)

from .core.arms import ARMS, paired, run_arm, strict_replay, summary
from .core.scenario import draw
from .core.sim import SimParams

RESOLUTION = 5.0


def parse_seeds(text: str) -> List[int]:
    """``"0-3,10"`` -> [0, 1, 2, 3, 10]."""
    out: List[int] = []
    for item in text.split(','):
        lo, _, hi = item.partition('-')
        out += range(int(lo), int(hi or lo) + 1)
    return out


def load(worlds_dir: str, name: str):
    folder = Path(worlds_dir) / name
    world = load_world(folder / 'landmarks.yaml')
    x0, y0, x1, y1 = world.bounds
    grid = Grid(round((x1 - x0) / RESOLUTION), round((y1 - y0) / RESOLUTION),
                RESOLUTION, x0, y0)
    world.check_grid(grid)
    return world, grid, folder / 'recordings.json'


def clues(world, grid, seeds) -> List[str]:
    """Every distinct clue, in the order the seeds need them."""
    scenarios = [draw(world, grid, s) for s in seeds]
    return list(dict.fromkeys(c for s in scenarios for c in (s.accurate, s.misleading)))


def record(world, grid, path, wanted, provider, today: str,
           pause: float = 0.0) -> int:
    """Ask for each clue with no recording yet. Returns how many were asked."""
    have = load_replay(path, require_live=True) if path.exists() else None
    asked = 0
    for clue in wanted:
        if have is not None:
            try:
                have(clue, world)
                continue
            except NoRecording:
                pass
        if asked:
            time.sleep(pause)
        prior = make_prior(clue, world, grid, provider, strict=True)
        save_recording(path, live_recording(world, clue, provider.model,
                                            provider.outputs[-1], today))
        asked += 1
        print(f"{clue}\n    {provider.outputs[-1]}\n    {provider.usage[-1]}")
        for w in prior.warnings:
            print(f"    warning: {w}")
    return asked


def _job(args):
    arm, seed, worlds_dir, name, params = args
    world, grid, path = load(worlds_dir, name)
    provider = None if arm == 'uniform' else strict_replay(path)
    return run_arm(arm, draw(world, grid, seed), world, grid, provider, params)


def matrix(worlds_dir, name, seeds, params, workers):
    jobs = [(arm, seed, worlds_dir, name, params) for arm in ARMS for seed in seeds]
    # spawn, not fork: the caller may have threads (pytest with rclpy loaded)
    with ProcessPoolExecutor(workers, get_context('spawn')) as pool:
        runs = list(pool.map(_job, jobs))
    return {arm: [r for r in runs if r.arm == arm] for arm in ARMS}


def _t(v: float) -> str:
    return 'cens.' if math.isinf(v) else f'{v:.1f}'


def table(runs) -> str:
    out = ['| Arm | Time to first, s (q25 / median / q75) | Time to all, s | '
           'Runs not finding all | Targets missed | Targets the clue covered | '
           'Found by the leader | Extra confirmations | Coverage, lowest | '
           'Ended DONE |', '|---|---|---|---|---|---|---|---|---|---|']
    for arm, rs in runs.items():
        s = summary(rs)
        out.append(
            f"| {arm} | {' / '.join(map(_t, s['first']))} | "
            f"{' / '.join(map(_t, s['all']))} | {s['all_censored']}/{s['runs']} | "
            f"{s['targets_missed']}/{s['targets']} | "
            f"{s['targets_covered_by_clue']}/{s['targets']} | "
            f"{s['found_by_leader']}/{s['targets'] - s['targets_missed']} | "
            f"{s['extra_confirms']} | "
            f"{s['coverage_min']:.2f} | {s['done']}/{s['runs']} |")
    out += ['', *_PAIRED_HEAD]
    for a, b in (('accurate', 'uniform'), ('misleading', 'uniform'),
                 ('misleading', 'accurate'), ('fixed', 'misleading')):
        for what in ('first', 'all'):
            out.append(_paired_row(f'{a} - {b}', what, paired(runs, a, b, what)))
    return '\n'.join(out)


_PAIRED_HEAD = ('| Seed by seed | Time | Faster | Slower | Ties | Sign test p | '
                'Median difference, s (seeds where both finished) |',
                '|---|---|---|---|---|---|---|')


def _paired_row(name: str, what: str, p: dict) -> str:
    return (f"| {name} | {what} | {p['faster']} | {p['slower']} | {p['ties']} | "
            f"{p['sign_p']:.3f} | {p['median_diff']:+.1f} ({p['both_found']}) |")


def to_json(runs) -> list:
    return [{'arm': arm, **asdict(r.scenario), 'covered': r.covered,
             'regions': r.regions, **asdict(r.result)}
            for arm, rs in runs.items() for r in rs]


def from_json(data) -> dict:
    """The runs of a results file, as far as ``paired`` needs them."""
    runs = {}
    for d in data:
        times = d['found']
        runs.setdefault(d['arm'], []).append(SimpleNamespace(
            scenario=SimpleNamespace(seed=d['seed']), result=SimpleNamespace(
                first=min((t for t in times if t is not None), default=None),
                all=None if None in times else max(times))))
    return runs


def against(runs, other) -> str:
    """These runs minus another file's, arm by arm and seed by seed."""
    out = list(_PAIRED_HEAD)
    for arm in runs:
        for what in ('first', 'all'):
            out.append(_paired_row(f'{arm}, this - that', what, paired(
                {'a': runs[arm], 'b': other[arm]}, 'a', 'b', what)))
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('command', choices=('clues', 'record', 'run'))
    ap.add_argument('--seeds', default='0-19', type=parse_seeds)
    ap.add_argument('--world', default='valley')
    ap.add_argument('--worlds-dir', default='worlds')
    ap.add_argument('--model', help='record: the language model to ask')
    ap.add_argument('--pause', type=float, default=6.0,
                    help='record: s between questions (a per-minute limit)')
    ap.add_argument('--out', help='run: write every run to this JSON file')
    ap.add_argument('--against', help='run: compare with this results JSON')
    ap.add_argument('--prior-lanes', action='store_true',
                    help="run: the leader flies the clue's lanes first (D53)")
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args(argv)
    world, grid, path = load(a.worlds_dir, a.world)

    if a.command == 'clues':
        print('\n'.join(clues(world, grid, a.seeds)))
    elif a.command == 'record':
        provider = LiveProvider(a.model or '')
        try:
            record(world, grid, path, clues(world, grid, a.seeds), provider,
                   datetime.date.today().isoformat(), a.pause)
        finally:
            total = {}
            for u in provider.usage:
                for k, v in u.items():
                    total[k] = total.get(k, 0) + v
            print(f"{len(provider.usage)} answers from {provider.model}, tokens {total}")
    else:
        params = replace(SimParams(), prior_lanes=a.prior_lanes)
        t = time.monotonic()
        runs = matrix(a.worlds_dir, a.world, a.seeds, params, a.workers)
        print(table(runs))
        redraws = sum(r.scenario.redraws for r in runs['uniform'])
        print(f"\n{len(a.seeds)} seeds, {redraws} redraws, "
              f"prior lanes {'on' if params.prior_lanes else 'off'}, "
              f"{time.monotonic() - t:.0f} s")
        if a.out:
            Path(a.out).write_text(json.dumps(to_json(runs), indent=1) + '\n')
        if a.against:
            print('\n' + against(runs, from_json(json.loads(
                Path(a.against).read_text()))))


if __name__ == '__main__':
    main()
