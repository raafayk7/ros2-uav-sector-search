# Next-session prompt: results, showcase runs, write-up (session 19)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

**Session 18 (2026-11-25) did half of the Dec 20 row.** The Gemini provider works and made the project's first real model calls (23 answers, all parsed first time, about $0.05 at paid prices). `src/uav_eval` is the 2D eval on a 120 m world, and an 8-seed pilot runs end to end on live recordings. 482 tests. The final seeds (0-19) have not been asked or run, on purpose. Pushed 2026-11-25 (`5ba1a0b`): hosted CI green (480 passed, 2 skipped, run 45533558578) and the smoke mission green on the self-hosted runner (482 tests, DONE with 3 finds, run 45533591662).

**Sessions left to Dec 20: one (19), maybe two.** The results are about an hour; the showcase runs, video v2 and the literature re-check are the rest. If it runs over, split 19b for the video.

Open for the user:

- Recording the 40 final clues: the user said next session (2026-11-25), so it is approved for the start of this one, after R12 (about $0.10 at paid prices, $0 on the free tier; the key's tier and daily limit cannot be read from the key, see AI Studio). `record` resumes where it stopped.
- Whether to compare one alternative to the best-open-sector rule on LLM-shaped maps before the table (R12, TODOS). Recommendation: yes, a distance discount, on pilot seeds only.
- Whether precision landing is scheduled or out of scope; the eval's optional thesis-timing arm (D42).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-18 (done 2026-11-07 to 11-25) | Datum through the go/no-go, video v1, CI green, clue-driven mission, live model, 2D eval and pilot | Nov 10, 15, 20, 24, 30; Dec 8, 10; half of Dec 20 | mixed | see git history | ~43 h |
| **19 (this prompt)** | Final recordings, 20 x 4 results and ablation, results write-up, literature re-check | Dec 20 | Opus | Writing what the numbers do and do not show | ~2-3 h |
| 19b, if needed | 3 Gazebo showcase runs, video v2 | Dec 20 | Sonnet | Runs and recording, recipes exist (docker-notes 61-62) | ~2 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 19. The eval harness works. This session produces the results and writes what they show.

## Read first

- `docs/planner-decisions.md`: session 18 notes (D51-D54, the pilot, "choices made while writing", the not-shown table).
- `docs/roadmap.md`: Eval design. `docs/literature-check.md`: the claim and what may be said after the eval.
- `src/uav_eval`: `core/scenario.py`, `core/sim.py`, `core/arms.py`, `run.py`.

## Task

1. **R12 first, on pilot seeds (1000+) only**: if the user wants it, compare one alternative rule. Change nothing after the final seeds have been run.
2. **Record the final clues** (with the user's yes): `docker exec --env-file docker/llm.env multi-agent-uavs bash -lc 'cd /workspaces/multi-agent-uavs; export PYTHONPATH=src/uav_mapping:src/uav_planning:src/uav_perception:src/uav_eval; /usr/bin/python3 -m uav_eval.run record --seeds 0-19 --model gemini-3.8-flash'`. Read every answer before running anything. An invalid or odd answer is part of the result: report it, do not re-ask until it looks better.
3. **Run the matrix once**: `run --seeds 0-19 --out ...`, then `--prior-lanes` for the ablation. Commit the JSON and the table.
4. **Write it up**: medians and quartiles with censor counts, paired differences, how often the clue covered each target, finds by leader against followers, extra confirmations. Coverage is 100 % everywhere (the sweep always finishes) and separates no arm: say so. Say "kinematic", one template, one model, one world, invented landmarks. If the misleading arm is no worse than uniform on time to first find (as in the pilot), say it in those words.
5. **Literature re-check** before any README claim.
6. 3 Gazebo showcase runs and video v2, or 19b.

## Environment

- The USB drive must be in before boot or Docker hangs (docker-notes 69). Before any sim launch: `mount | grep docker-data`, `docker ps --filter name=WinBoat`.
- The Gemini client is installed in the running container only; after a recreate reinstall it (docker-notes 67). Never open, print or `cat` `docker/llm.env`.
- `uav_eval` runs on the host too (numpy and yaml are enough) with the same `PYTHONPATH`; `record` needs the container.
- Run `colcon build --symlink-install` after Python edits (gotcha 32). Tests: `python3 -m pytest src` in the container (`ROS_DOMAIN_ID=77` with a sim up).
- The smoke runner was started in session 18; it is off if the machine or that session was closed (`cd ~/actions-runner && ./run.sh`, then `gh workflow run smoke.yml`).
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

A committed results table over 20 x 4 with its ablation and raw JSON, a README and roadmap that say what it shows and does not, the literature search re-run, and the showcase runs or a 19b prompt for them. Say plainly which changes are code, plan-level or documentation, and give every "not shown" gap its home.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D54). The contribution claim changes only with the user's yes.
- Do not push without asking.
