# Next-session prompt: showcase runs and video v2 (session 19b)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

**Session 19 (2026-11-25) did the results half of the Dec 20 row.** 40 final clues recorded, 20 seeds x 4 arms and the lanes-first ablation run once, written up in `docs/eval-results.md` and the README, literature re-checked (claim unchanged). An accurate clue helps; a misleading one was not measurably worse than none; negative evidence is what keeps it cheap. 489 tests. Not pushed unless the git log says so.

**Sessions left to Dec 20: one (19b).** Expected landing: well before Dec 20 (Dec 1 or so at the current pace). What is left is the 3 showcase runs and video v2, which need the user at the screen with OBS.

Open for the user:

- Pushing session 19's commits (then hosted CI runs; `smoke.yml` needs the self-hosted runner).
- Whether precision landing is scheduled or out of scope; the eval's optional thesis-timing arm (D42). Both are unchanged and block nothing.
- Whether to read Meng 2008 and Bertuccelli & How 2005 in a browser (TODOS). Only needed for a wider claim than language-derived priors.

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-19 (done 2026-11-07 to 11-25) | Datum through the go/no-go, video v1, CI green, clue-driven mission, live model, 2D eval and its results | Nov 10, 15, 20, 24, 30; Dec 8, 10; most of Dec 20 | mixed | see git history | ~46 h |
| **19b (this prompt)** | 3 Gazebo showcase runs, video v2 | Dec 20 | Sonnet | Runs and recording, recipes exist | ~2 h |
| later, if wanted | Thesis-timing arm, landing, the open TODOS | none | Opus for design, Sonnet for TODOS | | open |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 19b. The eval is done and written up. This session makes the three Gazebo showcase runs and video v2.

## Read first

- `docs/eval-results.md`: what may be said on screen, and what may not.
- `docs/planner-decisions.md`: session 13 notes (dress run, halos, markers), session 14 notes (OBS roughly halves sim speed), session 17 notes (the clue flights).
- `docs/docker-notes.md` 61-62 and 68.

## Task

1. **Window check** (TODOS): `swarm.launch.py gui:=true arena_view:=true` on the rebuilt image; ask before restarting a sim the user is watching.
2. **Three runs, one command each, in the 40 m runway world**, leader + 3 followers: no clue, the hut clue, the radio mast clue (the misleading twin; hut and mast recordings are live model answers). Spawn the same three targets each time. One run per clue, so they illustrate and measure nothing about time. Keep the launch lines and each `/mission_report`.
3. **Video v2**: the user records with OBS. Put on screen that it is simulation, ground-truth detection, not YOLO, and that times are not the eval's.
4. **README**: link video v2, and say the showcase is three single runs in a 40 m world, not the eval.
5. Update the roadmap's Dec 20 row, stack overview, CLAUDE.md and the skill, then `skills/pack.sh`.

## Environment

- The USB drive must be in before boot or Docker hangs (docker-notes 69). Before any sim launch: `mount | grep docker-data`, `docker ps --filter name=WinBoat`.
- `docker compose restart ros2` between runs. A cold start with no `/clock` (gotcha 33): restart and relaunch.
- Run `colcon build --symlink-install` after Python edits (gotcha 32). Tests: `python3 -m pytest src` in the container (`ROS_DOMAIN_ID=77` with a sim up). The final seeds are used: anything tuned on them needs new seeds.
- Never open, print or `cat` `docker/llm.env`. No live calls are needed here.
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

Three reports on file, video v2 in `docs/media/`, README and roadmap updated, and the Dec 20 row closed or its gap named.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D54). The contribution claim changes only with the user's yes.
- Do not push without asking.
