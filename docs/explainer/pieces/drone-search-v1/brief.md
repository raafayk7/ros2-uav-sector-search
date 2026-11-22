# Brief: How a drone search team works

## The idea in one sentence

One 40 x 40 m grid, and each idea in the project is a layer drawn onto it: "one shared map, three jobs: the leader looks, the followers decide, the mission manages."

## Format and spec

- Format: locked-off build, fixed stage (F2 family: the proven fixed-stage chronology, with a step counter instead of a year; no morph bridges, no hard cuts). The review's morph-chain checks that need bridges are not applicable.
- Size: 1920x1080 (16:9), 24 fps, 120 BPM, 146 s = 3504 frames (110 s in run 1; steps 10-14 were extended in run 2 at the user's request). No voice; on-screen text only, with a score composed in code.
- Style: `math`; no hero (the user removed the pi hero at the look check). Colour rules: yellow = the leader, blue = the followers, orange brightness = the chance a target is in a sector, red = a target, green = a confirmed find, grey = unknown. A key along the bottom of the card column repeats this.
- Honesty: the timing is schematic (a tag says so); "simulation" is stated; real aircraft, real image recognition and landing are marked not done or not scheduled; the language prior and the comparison are marked planned.

## Beats (approved at the story check and the storyboard check)

| # | t (s) | what happens | bridge into the next | sound |
|---|---|---|---|---|
| 0 | 0-5 | the grid draws itself; the title | the grid stays | a thin pad |
| 1 | 5-13 | arena and sector: 40 m square, 8 x 8, one sector measured | every sector gets a number | a rising wave of plucks, a chime on the lit sector |
| 2 | 13-21 | belief map: "?" then "50%" | the leader fills it | soft blips |
| 3 | 21-33 | leader, sweep, footprint: 3 lanes, a 25 m by 19 m patch | what a look does to a number | a slow tone that follows the leader |
| 4 | 33-45 | evidence: empty looks 50, 30, 16, 7, 3; hits 50, 70, 84, 93, 97 | where a hit comes from | steps down and up |
| 5 | 45-56 | detector: a stand-in, misses 1 in 5, invents 1 in 20 | followers read the map | a flicker per frame |
| 6 | 56-67 | followers and the planner; layers 10, 12, 14 m; a comparison: the original project had the followers wait for the leader to finish, this one starts them together (the leader's sweep now ends at 74 s, after the three finds, as in the real runs) | one arrives over a target | a thickening pad |
| 7 | 67-75 | confirmed find: 3 frames in a row within 2 m | the others move on | silence from 67.4, the loudest hit at 70.1 |
| 8 | 75-83 | mission: 30 s of quiet, the report, finders return | the grid, changed | a warm flat chord |
| 9 | 83-91 | where it stands | what comes next | a slow decay |
| 10 | 91-103 | language prior (planned, not built): a field report goes to a language model with a list of named places, which writes regions with weights; they become starting numbers; its only output is the map | when it is used | a plucked note per box |
| 11 | 103-115 | when it is used (planned): once, at the start, before any drone moves; the leader and followers then start together; evidence keeps correcting the map (a target the clue missed climbs 50 to 97%) | what is new | one note per row |
| 12 | 115-128 | what is new here: language-derived priors exist (LMPath, arXiv 2611.16524), so the claim is only the measurement of what a wrong clue costs; "as of a literature check, Nov 2026; to be re-run before publishing" | the evaluation | one note per row, a chime on the last |
| 13 | 128-140 | the evaluation (planned, not run): four conditions, 20 scenarios x 4 = 80 runs in a fast 2D simulator; first find, all found (unfound counted as cut off), coverage at 15 simulated minutes; the last condition separates the map's learning from the sweep | the close | held |
| 14 | 140-146 | one shared map; not scheduled: image recognition, landing | loops to the empty grid | a decay |

## Sound plan

Thin early (one pad), fuller as ideas stack (a second pad at the sweep, a third when the followers move). A hard stop at 67.4 s, silence until 70.1 s while the picture holds on the follower hovering over its target, then the single loudest hit (chord stab, sub) on the first confirmed find and a flat warm chord after it. The two later confirmations are quieter. A slow decay to the end. The user should listen: the score cannot be heard here, only measured.

## Claims and sources (from the project's own docs, not the web, as the user asked)

- Language prior design: an operator's clue text plus a per-world landmark file go to the language model; its strict JSON output is regions (circle or capsule) with weight and confidence; the clue becomes the prior at t = 0; a cell's starting belief blends the prior with 0.5; the language model's only output is the prior and nothing it produces reaches a velocity command, setpoint or mode change - `docs/roadmap.md` (Mission timing, Fusion, LLM contract). All of it is planned (Dec 10 milestone) and not built.
- What is new: "novel only in the eval"; the pipeline is made of known parts (LMPath arXiv 2605.13782, arXiv 2611.16524, Bayesian search, arXiv 2611.12317) - `docs/literature-check.md` (session 12, 2026-11-22; to be re-run before any publication). The claim is the user-accepted wording in `docs/roadmap.md` (Contribution claim). No "first" or "new pipeline" is claimed.
- Evaluation: 2D kinematic sim, 20 seeds, four conditions (uniform, accurate clue, misleading clue, misleading clue with negative evidence off), 80 runs, timeout 15 sim-minutes, unfound targets censored and counted, report time to first find, time to all found and coverage - `docs/roadmap.md` (Eval design). Planned for Dec 20, not run.

- 8 x 8 sectors of 5 m, 40 x 40 m; 3 lanes at 8 m; footprint 24.9 x 18.7 m at 8 m; lane positions 8.3, 20.0, 31.7 m - `docs/planner-decisions.md` D39 and the session 13 notes (coverage computation).
- Belief numbers: first observation p = 0.5; one look moves a sector by at most logit(0.7); belief stays within 0.03 and 0.97; an unseen sector counts as 0.5 - `src/uav_mapping/uav_mapping/core/belief.py` (BeliefParams), computed in `src/model.js` (pAfter). On screen: empty 50, 30, 16, 7, 3; target 50, 70, 84, 93, 97 (rounded from 0.500, 0.300, 0.155, 0.072, 0.033->0.03 floor; 0.700, 0.845, 0.927, 0.968).
- Detector: misses 20 %, false alarms 5 % per frame - D38 and README. The frame rows (8 of 10 seen, 1 of 20 invented) are the averages, labelled as an example.
- Followers: best open sector not claimed by a peer; layers 10, 12, 14 m - D5, D11, D24, D25, D26.
- Confirm: within 2.0 m, 3 consecutive frames, sector then reads 0 - D32, D40, D13.
- Mission end: sweep complete and every unconfirmed sector below 50 % for 30 s; report; each finder returns over its find - D14, D41 (amended 2026-11-22).
- 3 of 3 targets found in every cold run that reached the end so far - `docs/planner-decisions.md` session 11-14 notes.
- Not done: real aircraft, real image recognition (YOLO is a stretch goal), landing (in no milestone) - `docs/roadmap.md`. Planned: the language prior (Dec 10) and the four-condition comparison (Dec 20).
- Original project ran the leader first and the followers second (Phase 1, then Phase 2 in the senior-project deck); this project starts them together (D42) - `docs/planner-decisions.md` D42 and the deck.
- Schematic: the video compresses and reorders time (the real run's three finds came 15-34 s into the sweep). The tag says so.
