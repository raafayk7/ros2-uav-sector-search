# What a wrong language clue costs: the 2D eval (2026-11-25)

**In short.** A correct clue made the search faster. A wrong one was not measurably worse than no clue at all, on 20 seeds. With negative evidence switched off, the same wrong clue made finding everything take more than three times as long, and the leader's full sweep did most of the finding.

This is a **kinematic 2D simulation**: drones are points at 5 m/s with no climb, acceleration, tilt, lost messages or delay. It compares arms; it does not predict flight times. The flying code (planner, sweep, belief map, mission end rule) and the detector's noise are the real ones. Nothing here was flown in Gazebo.

## Setup

- 20 seeds (0-19), 50 targets (2 seeds with one target, 6 with two, 12 with three), one 120 x 120 m world with 13 invented landmarks (`worlds/valley`). Each target is within 8 m of its own landmark. No seed needed a redraw.
- 1 leader on a fixed full sweep (about 206 s) and 3 followers on the best-open-sector rule. Detector: 20 % misses, 5 % false alarms per frame, 5 px noise (D38). Timeout 900 s.
- **Uniform:** no clue. **Accurate:** "Two hikers are missing: one last seen near the well and one near the orchard." **Misleading:** the same sentence with every landmark swapped for a wrong one, fixed before any model was asked (D47, D54). **Fixed:** the misleading clue with negative evidence off, so an empty camera view changes nothing.
- The clues were read by gemini-3.8-flash, once each, and frozen as recordings. Arms of a seed share targets and detector seed.
- *Found* = a true detection within 2 m of the target. Time starts at the mission's start.
- The rules, the table's columns and the tests were committed before any final clue was asked (`a7f70f4`), the recordings before any was flown (`31d1f45`). The matrix was run once; replays are byte-identical.

## What the model answered

40 clues: 38 asked now, 2 already recorded in the pilot (the same one-hiker sentences). 38 of 38 parsed at the first attempt, 27,649 tokens in and 20,868 out including thinking, about $0.09 at the paid price session 18 worked out.

Every region sat on the place its clue named, radii 10-18 m, confidence 0.90-0.95 everywhere: on this template the model expresses almost no doubt. The accurate regions cover **47 of 50** targets. The 3 misses are targets 4.5-7.8 m from a polygon (quarry, copse, orchard) that the model answered with one circle. The misleading ones cover 0 of 50, and none touches a target's cell or its neighbours. Checked with `docs/probes/recordings_check.py` before the matrix was run. No answer was re-asked.

## Results

Times in seconds: q25 / median / q75 (lower interpolation, censored runs last).

| Arm | First find | All found | Runs not finding all | Targets missed | Covered by the clue | Found by the leader | Extra confirmations | Ended DONE |
|---|---|---|---|---|---|---|---|---|
| uniform | 6.0 / 12.0 / 21.5 | 24.0 / 33.5 / 39.5 | 0/20 | 0/50 | 0/50 | 9/50 | 7 | 20/20 |
| accurate | 5.0 / 8.5 / 12.5 | 15.5 / 19.5 / 24.0 | 0/20 | 0/50 | 47/50 | 4/50 | 8 | 20/20 |
| misleading | 5.0 / 13.5 / 21.5 | 28.5 / 43.5 / 49.0 | 0/20 | 0/50 | 0/50 | 10/50 | 8 | 20/20 |
| fixed | 5.0 / 13.5 / 61.0 | 42.5 / 141.5 / 173.5 | 0/20 | 0/50 | 0/50 | 35/50 | 10 | 0/20 |

**No run was censored and no target was missed in any arm, and coverage was 100 % in all 80 runs.** The sweep always finishes, so coverage and misses separate no arm here.

Seed by seed (the arms share each seed's targets and noise). p is a two-sided exact sign test, **not corrected for the eight comparisons**.

| Comparison | Time | Faster | Slower | Ties | p | Median difference, s |
|---|---|---|---|---|---|---|
| accurate - uniform | first | 16 | 0 | 4 | < 0.001 | -2.5 |
| accurate - uniform | all | 19 | 0 | 1 | < 0.001 | -12.2 |
| misleading - uniform | first | 6 | 8 | 6 | 0.79 | 0.0 |
| misleading - uniform | all | 9 | 11 | 0 | 0.82 | +5.0 |
| misleading - accurate | first | 1 | 18 | 1 | < 0.001 | +3.0 |
| misleading - accurate | all | 0 | 20 | 0 | < 0.001 | +16.5 |
| fixed - misleading | first | 1 | 7 | 12 | 0.07 | 0.0 |
| fixed - misleading | all | 1 | 17 | 2 | < 0.001 | +106.5 |

Raw data: `docs/results/eval.json`; the table as printed: `docs/results/eval_table.md`.

## What it shows

1. **An accurate clue helps.** Median time to all found 19.5 s against 33.5 s without a clue, faster in 19 seeds and slower in none. First find 8.5 s against 12.0 s.
2. **On time to first find, the misleading arm is not measurably worse than uniform** (13.5 s against 12.0 s, 6 seeds faster, 8 slower, 6 ties). Three followers spread out and something is found soon whatever the clue says.
3. **On time to all found, a misleading clue is not shown to be worse either, but this is the weaker statement.** The medians are 10 s apart (43.5 against 33.5), the paired median difference is +5.0 s, and the split is 9 faster, 11 slower. The differences run from -17.5 to +45.0 s (mean +7.4): when it hurt it hurt more than it helped. Twenty seeds cannot tell a small cost from none.
4. **The clue still costs its information.** Against an accurate clue, a misleading one was slower to all found in all 20 seeds (median +16.5 s).
5. **Negative evidence is what keeps a wrong clue cheap here.** Without it the median to all found goes from 43.5 to 141.5 s (slower in 17 seeds, faster in 1). The sweep is the backstop: the leader found 35 of the 50 targets, against 10 with negative evidence on, and the slowest run still found everything at 194.5 s, before the sweep's end. That arm never ends DONE, because nothing lowers the map (by construction).

Point 5 is close to the case arXiv 2611.12317 measured (a map that never updates, where a strongly biased map did worse than a uniform one). The worlds, planners and metrics differ, so the numbers are not comparable.

## Ablation: the leader flies the clue's lanes first (D53)

Same seeds, `--prior-lanes` (`docs/results/eval_lanes.json`, `eval_lanes_table.md`). Medians to all found: accurate 21.0 (19.5 without), misleading 49.0 (43.5), fixed 181.5 (141.5). Lanes first, seed by seed against the default sweep, to all found: accurate 2 faster, 6 slower, 12 ties (p 0.29); misleading 5, 10, 5 (p 0.30); fixed 4, 15, 1 (p 0.02). To first find: misleading 0 faster, 8 slower, 12 ties (p 0.008). It helped no arm, which is why it is off.

## Extra confirmations

A follower confirmed a sector with no target in it 33 times in the 80 runs. All 33 sectors lie within 0.31 m of a target's position, just across a sector edge: pixel noise split one target over two sectors. No false alarm was confirmed away from a target. A mission report would list those targets twice (`docs/probes/eval_detail.py`, written after the table was seen).

## Does the follower rule matter? (R12, on other seeds, before the matrix)

The rule takes the highest-value sector anywhere and ignores distance except for ties. Four distance discounts were compared on 200 other seeds with a stand-in reader (`docs/probes/distance_rule_probe.py`), with the bar for adopting one written down first. None met it. With an accurate clue every discount was slower (116-166 of 200 seeds slower, 24-54 faster); with a misleading one every discount was faster in more seeds (105-132 against 51-75). **A discount is a hedge, not an improvement:** it gives up part of what a right clue is worth to lose less to a wrong one. The rule stays, so the numbers above lean towards trusting the clue. These are design numbers from a stand-in, not results.

## What it does not show

| Limit | Home |
|---|---|
| Kinematic 2D only; the same scenario was not run in Gazebo | Not scheduled |
| One clue template ("last seen near the X"), one model, one invented world, one fleet size (1 + 3), one alpha (0.5) | Not scheduled; stated here |
| Vague, negated, partly wrong or contradictory reports: here a clue is right about every target or wrong about every target | Not scheduled |
| Calibrated doubt: the model gave 0.90-0.95 on every region | Not scheduled |
| Moving targets | Not scheduled |
| Whether a wrong clue costs a little or nothing: 20 seeds cannot separate them | Not scheduled; more seeds would need new recordings |
| Time from launch: the three followers start 5 m south of the arena, so one target (seed 3) was found at 1.0 s in every arm | A fact of the setup |
| A thesis-timing arm (followers after the sweep, D42) | The user's call, open |

## Reproduce

No key and no simulator; it reads the committed recordings.

```bash
export PYTHONPATH=src/uav_mapping:src/uav_planning:src/uav_perception:src/uav_eval
python3 -m uav_eval.run run --seeds 0-19 --out /tmp/eval.json
python3 -m uav_eval.run run --seeds 0-19 --prior-lanes --against docs/results/eval.json
python3 docs/probes/recordings_check.py
python3 docs/probes/eval_detail.py
```

`src/uav_eval/test` replays two seeds of each file exactly and checks the published tables against the JSON.
