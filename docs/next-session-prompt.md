# Next-session prompt: wire the language prior and fly a clue (session 17)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

**Session 16 (2026-11-24) settled the five open prior questions (D44-D48) and built the ROS-free library:** `uav_mapping/core/prior.py`, `core/landmarks.py`, `BeliefMap(start=...)`, `worlds/runway/` (8 invented landmarks, 5 hand-written recordings), 127 tests, 358 in total. Scope fully met. No sim ran, no language model was called, nothing uses the library yet. Nothing has been pushed since `origin/main` unless the user did it.

**Sessions left to Dec 10: one (session 17).** At the current pace it lands before the end of November, so Dec 10 is on track with time to spare, and Dec 20 needs sessions 18-19.

Open for the user, none blocking session 17:

- **The eval's recordings need a real language model.** Session 16's are hand-written and the eval refuses them (D48). Session 18 needs `LLM_API_KEY` set by the user, in their shell only, never committed. Choose the one `live` model then.
- **Eval arena size** (session 18): the session 16 probe says a clue barely matters at 40 m, and that 80-160 m shows it.
- Whether precision landing is scheduled or out of scope; the eval's optional thesis-timing arm (D42).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-16 (done 2026-11-07 to 11-24) | Datum through the go/no-go, video v1, CI green, prior library | Nov 10, 15, 20, 24, 30; Dec 8 | mixed | see git history | ~37 h |
| **17 (this prompt)** | `mission_prior_node`, wire the prior into `belief_map_node` and `swarm.launch.py`, the `live` provider, first mission driven by a clue; explainer v2 if time allows | **Dec 10** | Sonnet | Wiring against a settled design; one small design choice, below | ~2-3 h |
| 18 | 2D eval sim, arena size, run matrix, frozen live recordings | Dec 20 | Opus | Eval design and honest reporting | ~3 h |
| 19 | 20 seeds x 4 conditions, results table, video v2, re-run the literature search | Dec 20 | Opus | Writing what the numbers do and do not show | ~2-3 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 17. The prior library is built and tested. This session makes a typed clue change where the drones go, in Gazebo, from one command.

## Read first

- `docs/planner-decisions.md`: the session 16 notes (D44-D48, probe limits, what is not shown), D41 and D42.
- `uav_mapping/core/prior.py` (`make_prior`, `load_replay`), `core/landmarks.py` (`load_world`), `BeliefMap(start=...)` in `core/belief.py`.
- `uav_mapping/belief_map_node.py`, `uav_planning/launch/swarm.launch.py`, `ci/smoke_test.sh`.
- `docs/roadmap.md`: LLM contract, Mission timing.

## Task

1. **The wiring choice, with the user.** `BeliefMap` takes its start only when constructed, so the map must not fuse or publish before the prior is in. Recommendation to put to the user: `mission_prior_node` publishes one latched message; with a clue set, `belief_map_node` drops frames and stays silent until it arrives, then builds its map. `mission_node`'s WAITING already waits for `/heatmap` (D41), so the mission cannot start on a half-built map. An empty clue skips all of it and must be exactly today's behaviour. Decide what happens if the prior node dies (a timeout to uniform with a warning keeps "nothing here can stop a flight").
2. **`mission_prior_node`** (in `uav_mapping`): parameters clue, world directory, provider (`replay` default), alpha; calls `make_prior`, logs every warning, publishes. It publishes the prior and nothing else (hard rule; extend `test_the_prior_code_cannot_reach_a_drone`).
3. **`live` provider**: key only from `LLM_API_KEY`, temperature 0, the prompt built from the landmark file and bounds. Testable without a key; do not call it unless the user sets one. It also needs a way to save its output as a recording. Check that the chosen model still accepts a temperature setting.
4. **Launch.** `swarm.launch.py clue:="One hiker, last seen near the hut."`, replay by default so no key is needed.
5. **Fly it.** One cold run with the hut clue and one with the radio mast clue (the misleading twin), targets from `ci/spawn_targets.sh`. Show that the first goal sectors differ from a run without a clue, and that all runs end DONE with the finds. Expect little time difference at 40 m: session 16 says so, and one run each is not a measurement.
6. **Leader lane order** (roadmap: high-prior lanes first) is not built. On 40 m it cannot pay off. Ask whether it stays in scope for Dec 10 or moves to the eval sim.
7. Docs: roadmap Dec 10 row, decisions, session 17 notes, this file for session 18. Explainer v2 (`docs/explainer/pieces/drone-search-v1`, steps 10-13 become real) only if time allows.

## Environment

- Before any sim launch: `mount | grep docker-data` and `docker ps --filter name=WinBoat` (tell the user at once if it is up).
- Session 16 recreated the dev container from the rebuilt image (setuptools 68.1.2 checked). Run `colcon build --symlink-install` after every Python edit (gotcha 32). Not checked since the rebuild: the Gazebo window (TODOS); ask before any GUI run.
- Tests: `python3 -m pytest src` in the container (`source /usr/local/bin/entrypoint.sh` first); with a sim up, `ROS_DOMAIN_ID=77`.
- `.gstack/pregate.json` gained `worlds/**` and `uav_mapping/package.xml` lanes; they count only once on `origin/main`, so until a push `/verify` says "tests UNMAPPED" for them.
- One four-drone run holds about 30 DDS participants (gotcha 61); one more node is one more.
- The smoke runner is off between sessions (`cd ~/actions-runner && ./run.sh`, then `gh workflow run smoke.yml`).
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

A mission launched with a clue flies to the clue's sectors first and ends DONE from one command, with replay and no key; without a clue it behaves as today. The Dec 10 row is updated. Say plainly which changes are code, which are plan-level decisions and which are documentation. For each "not shown" gap, say whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D48). The contribution claim changes only with the user's yes.
- Do not push without asking.
