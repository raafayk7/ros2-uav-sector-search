# Next-session prompt: the language prior's design and ROS-free core (session 16)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

**Session 15 (2026-11-23) built the CI, and both jobs are green on GitHub.** `.github/workflows/ci.yml` (hosted: build + unit tests in stock `ros:jazzy`) passed on GitHub after the push (run 45134386862, 223 passed and 2 skipped, 80 s). `smoke.yml` (self-hosted: the mission, manual and on pushes to main, never pull requests) passed on the user's runner (`~/actions-runner`, label `uav-smoke`, run 45134386843: 225 tests, then mission DONE with 3 finds, about 6.5 min). By hand it had passed twice (dev container 97 s, clean checkout through `smoke.yml`'s `docker run` line 317 s), mission DONE with 3 finds both times. A first run passed with 0 finds because the arena starts empty; the script now spawns the targets and the checker needs a find. `python3 -m pytest src` in the container: 225 passed; `/verify` is honest: `gstack-checks status` reads READY_FOR_SHIP (container pytest for both the project check and the pregate stage; `.gstack/pregate.json`, `test_ci_files.py`).

Open for the user, none blocking this session: keep the runner in mind (it only runs while `./run.sh` is up on the user's machine; a push to main queues a smoke job otherwise); whether precision landing is scheduled or out of scope; the eval's optional thesis-timing arm (D42).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-15 (done 2026-11-07 to 11-23) | Datum through the go/no-go, video v1, CI green (Dec 8 met) | Nov 10, 15, 20, 24, 30; Dec 8 | mixed | see git history | ~34 h |
| **16 (this prompt)** | Settle the open prior questions, build the ROS-free prior library with tests (parse, rasterize, blend, replay provider, landmark file) | Dec 10 | Opus | Design with open questions: rasterization rule, multi-target clues, misleading-landmark rule, alpha | ~2-3 h |
| 17 | `mission_prior_node`, wire the prior into `belief_map_node`, first mission driven by a clue; explainer v2 (`docs/explainer/pieces/drone-search-v1`, steps 10-13 become real) | Dec 10 | Sonnet | Wiring against a settled design | ~2 h |
| 18-19 (estimate) | 2D eval sim, 20 seeds x 4 conditions, results table, video v2, re-run the literature search | Dec 20 | Opus | Eval design and honest reporting | ~4-5 h |

Dec 10 is tight: two sessions in about two days. If session 16 spills, say so early; the roadmap's cut order puts the LLM layer last to go, after the CI and README GIF.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 16, the language prior. The mission runs end to end, the go/no-go is a GO and the CI is built. This session decides how a clue becomes a prior and builds that as a pure Python library, with no ROS and no LLM call.

## Read first

- `docs/roadmap.md`: Fusion, LLM contract (`mission_prior_node`), Mission timing, Eval design, and "Open engineering questions" (rasterization rule, default alpha, multi-target clue mapping, misleading-landmark rule, replay lookup for unmatched clues).
- `docs/literature-check.md` (what the claim is: only the eval is novel) and the contribution claim in the roadmap. The claim changes only with the user's yes.
- `uav_mapping/core/belief.py` (the posterior, `BeliefParams`, 33 tests): where a prior enters, and the fourth eval arm's `p_miss = 0.5`.
- `docs/planner-decisions.md`: D40, D41, D42 (followers start with the sweep, as the prior needs) and the TODOS.md eval item on tied regions (R12, language priors give broad circles and capsules where every cell ties).

## Task

1. **Settle the open questions with the user** before code, one at a time: the rasterization rule (circle or capsule to the 8x8 grid: cell centre, area overlap, or max), default alpha (roadmap says a fixed 0.5), how a clue with several targets maps to regions, what "misleading" means as a rule (same template, wrong landmark), and replay lookup for an unmatched clue. State each as a recommendation, not a survey.
2. **Library.** A ROS-free module with the strict-JSON contract (circle or capsule, weight, confidence, clamped to the map; invalid -> uniform), the landmark file (`worlds/<name>/landmarks.yaml`), the rasterizer, the alpha blend to a starting belief, and the `replay` provider reading committed recorded outputs. Unit tests at the library level, in the style of `belief.py`'s. The LLM's only output is the prior: nothing it produces reaches a setpoint or mode change.
3. Docs: roadmap Dec 10 row and open questions, `docs/planner-decisions.md` (a decision number per settled question), a session 16 notes section; rewrite this file for session 17.

## Environment

- Before any sim launch: `mount | grep docker-data` and `docker ps --filter name=WinBoat` (tell the user at once if it is up). This session needs no sim; the library tests need no ROS.
- Run tests with `python3 -m pytest src` in the container (`source /usr/local/bin/entrypoint.sh` first in a non-interactive shell); with a sim up, ROS-based tests need `ROS_DOMAIN_ID=77`.
- The dev container was stopped and restarted in session 15; if `up -d` is needed after a reboot, check setuptools with `/usr/bin/python3` (gotchas 57-58, 64).
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

The open questions answered and written as decisions; a tested library that turns a replayed clue into a prior grid and a starting belief; the Dec 10 row and this file updated. Say plainly which changes are code, which are plan-level decisions and which are documentation. For each "not shown" gap, say whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D42). The contribution claim changes only with the user's yes.
- Do not push without asking.
