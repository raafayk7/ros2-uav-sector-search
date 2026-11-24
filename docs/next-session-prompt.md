# Next-session prompt: the 2D eval sim and a real language model (session 18)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

**Session 17 (2026-11-24) met the Dec 10 row.** `swarm.launch.py leader:=true num_followers:=3 clue:="..."` starts `mission_prior_node`; `belief_map_node` waits for its latched `/mission_prior` (D49), then builds its map from it. One cold headless run each of no clue, the hut clue and the radio mast clue (misleading) ended DONE with all 3 finds (74.7, 74.0 and 72.8 sim-s), and the clue changed the first goal sectors. 382 tests. Not shown: a real language model's output (the `live` provider was unit-tested with a fake client and never called), any time effect (one run each, 40 m arena), leader lane order (moved here, D50). Nothing has been pushed since `origin/main` unless the user did it.

**Sessions left to Dec 20: two (18 and 19).**

Open for the user, in the order session 18 needs them:

- **`LLM_API_KEY`**, set in the user's shell only, never committed, and the one `live` model to use (the roadmap allows one). The container also needs `pip install anthropic` (TODOS, docker-notes 67).
- **Confirm the temperature correction.** The roadmap said temperature 0; current Claude models reject any `temperature`, so none is sent and repeatability comes from frozen recordings. If the user wants a provider that takes a temperature, say so before the key is used.
- **Eval arena size.** Session 16's probe says a clue barely matters at 40 m and shows at 80-160 m.
- Whether precision landing is scheduled or out of scope; the eval's optional thesis-timing arm (D42).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-17 (done 2026-11-07 to 11-24) | Datum through the go/no-go, video v1, CI green, prior library, clue-driven mission | Nov 10, 15, 20, 24, 30; Dec 8, 10 | mixed | see git history | ~40 h |
| **18 (this prompt)** | 2D eval sim, arena size, the first live call and frozen recordings, leader lane order (D50), run matrix | Dec 20 | Opus | Eval design and honest reporting | ~3 h |
| 19 | 20 seeds x 4 conditions, results table, video v2, re-run the literature search | Dec 20 | Opus | Writing what the numbers do and do not show | ~2-3 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 18. A clue now drives a Gazebo mission with replay. This session builds the eval that measures what a wrong clue costs, and makes the first real language-model call.

## Read first

- `docs/planner-decisions.md`: the session 16 and 17 notes (D44-D50, the probe's limits, the three flown runs and what they do not show), D41, D42.
- `docs/roadmap.md`: Eval design, LLM contract, Mission timing. `docs/literature-check.md` for the reworded claim ("What a wrong language-derived search prior costs, measured").
- `docs/probes/prior_probe.py`: the 2D point-mass sim the eval grows from. `uav_mapping/core/prior.py`, `core/live.py`, `core/landmarks.py` (`wrong_landmarks`, `misleading_swap`).
- `uav_mapping/mission_prior_node.py` (`record_to`), `docs/probes/prior_live_check.py`.

## Task

1. **The first live call, with the user.** Ask the user to set `LLM_API_KEY` in the shell that launches the container command and to name the model. Run `pip install anthropic` in the container. Check the model takes the request as built (no temperature; the provider asks with default sampling). Then run `mission_prior_node` alone (no sim) with `provider:=live llm_model:=... record_to=worlds/runway/recordings.json` for the hut and mast clues and read what the model returned before trusting it: regions on the right places, sensible radius, weight and confidence. One call each, show the cost. Never write the key to a file or a log.
2. **Arena size**, with the user: 40 m shows little (session 16), 80-160 m shows it. The eval sim and the landmark file must match the grid (`World.check_grid`); an 80 m world means a new `worlds/<name>/` with its own landmarks and recordings.
3. **The 2D eval sim**: point-mass drones, the camera footprint model, the same planner and belief code, faster than real time (grow `prior_probe.py`). Conditions: uniform, accurate clue, misleading (D47), misleading with negative evidence off (`BeliefParams.p_miss = 0.5`). Strict mode (`make_prior(strict=True)`, `load_replay(require_live=True)`): no hand-written recording may reach a result. Report time to first find, time to all found with the censor count, coverage at timeout, and how often the regions covered each target; not time to DONE.
4. **Leader lane order** (D50): high-prior lanes first in `core/sweep.py`, with the coverage guarantee kept (every lane flown whatever the prior). Build it in the eval sim first; Gazebo only if time allows.
5. **Run matrix**: pilot a few seeds per condition to check the harness and the cost before the full 20 x 4 (session 19).
6. Docs: roadmap, decisions (session 18 notes), this file for session 19.

## Environment

- Before any sim launch: `mount | grep docker-data` and `docker ps --filter name=WinBoat` (tell the user at once if it is up). The 2D eval needs no sim.
- Run `colcon build --symlink-install` after every Python edit (gotcha 32). A cold start with no ROS `/clock` happens (docker-notes 33, 68): `docker compose restart ros2` and launch again.
- Tests: `python3 -m pytest src` in the container (`source /usr/local/bin/entrypoint.sh` first); with a sim up, `ROS_DOMAIN_ID=77`.
- `.gstack/pregate.json` has `worlds/**` and `uav_mapping/package.xml` lanes; they count only once on `origin/main`, so until a push `/verify` says "tests UNMAPPED" for them.
- The smoke runner is off between sessions (`cd ~/actions-runner && ./run.sh`, then `gh workflow run smoke.yml`).
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

A pilot of the eval runs end to end on frozen live recordings with no hand-written one anywhere in a result, the arena size is decided and recorded, and the harness cost is known. Say plainly which changes are code, which are plan-level decisions and which are documentation. For each "not shown" gap, say whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D50). The contribution claim changes only with the user's yes.
- Do not push without asking.
