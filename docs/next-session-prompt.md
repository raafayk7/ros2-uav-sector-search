# Next-session prompt: the CI smoke test (session 15)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

**The Nov 30 go/no-go is a GO** (2026-11-22, sessions 13-14). A leader and three followers complete a mission in Gazebo from one command (`swarm.launch.py leader:=true num_followers:=3`): headless DONE in 142 s wall (RTF about 0.59), with the Gazebo window 203 s (RTF 0.27-0.34), with the `arena_view` look 184 s (RTF about 0.41), each with all 3 targets found and, after DONE, each finder hovering over its find. Video v1 is `docs/media/uav-v1.mp4` (72 s, 4x speed, with a poster; also in the README). `python3 -m pytest src` in the container: 225 passed; `/verify` runs it through a `gstack:verify:` line in `CLAUDE.md` (the container must be up).

Decided by the user in these sessions, all written up in `docs/planner-decisions.md`: D41 amended (return to find), D42 (followers start with the sweep, because the language prior needs it), halos and ground markers both stay, the video names the planner (best open sector, peers' claimed sectors skipped; BLL and DGA are baselines only, not run).

Open for the user, none blocking: whether precision landing is scheduled or declared out of scope (it is in no milestone; YOLO is a stretch goal), and the eval's optional thesis-timing arm (D42).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-14 (done 2026-11-07 to 11-22) | Datum through the go/no-go and video v1 | Nov 10, 15, 20, 24, 30 | mixed | see git history | ~32 h |
| **15 (this prompt)** | CI smoke test (1 leader + 1 follower, ground-truth detector, headless) | Dec 8 | Sonnet | Boilerplate workflow | ~2 h |
| 16-17 (estimate) | `mission_prior_node`, landmark file, replay provider, rasterization rule, first mission driven by a clue | Dec 10 | Opus for 16 (open questions: rasterization, multi-target clues), Sonnet for 17 | Design, then wiring; then explainer v2 (`docs/explainer/pieces/drone-search-v1`, steps 10-13 become real) | ~4 h |
| 18-19 (estimate) | 2D eval sim, 20 seeds x 4 conditions, results table, video v2, re-run the literature search | Dec 20 | Opus | Eval design and honest reporting | ~4-5 h |

At about one session every one or two days, Dec 8 is likely met (one session left). Dec 10 is tight: three sessions in about eight days. Video v2 (Dec 20) can use the re-record fixes in `TODOS.md` (grid off, 30 fps and 720p, stats overlay).

---

Continuing the multi-agent UAV ROS 2 rebuild: session 15, the CI smoke test. The mission runs end to end and the go/no-go is a GO. This session puts a headless smoke test in CI.

## Read first

- `docs/roadmap.md`: the CI section (smoke test: 1 leader + 1 follower, ground-truth detector, `replay` prior, 10-minute timeout; pass = reaches "target localized" or completes the sweep) and the Dec 8 row.
- `docs/planner-decisions.md`, session 13 and 14 notes (what the sim needs, RTF, DDS participants).
- `docs/docker-notes.md` gotchas 31-33 (login shell, `/clock`), 61 (DDS participants: a CI container needs `CYCLONEDDS_URI` or a small swarm), 58 (setuptools on a fresh image).
- `TODOS.md`: `/verify` on the host (the pregate stage).

## Task

1. **Smoke test.** A GitHub Actions workflow that builds the image (or pulls a cached layer; the image is about 25 GB, so check what a runner can hold first), builds the workspace with `colcon build --symlink-install`, runs `python3 -m pytest src`, then launches `swarm.launch.py leader:=true num_followers:=1` headless and passes on the mission reaching DONE (or the sweep completing) within the timeout. Decide with the user: a 25 GB image may not fit a hosted runner, in which case the smoke test runs against a smaller image or only the unit tests run in CI.
2. **Make `/verify` honest** if time is left: map the touched files in `.gstack/pregate.json` and route its runner through the container, or leave it (TODOS.md).
3. Update the roadmap's Dec 8 row and the docs that state status; rewrite this file for session 16.

## Environment

- Before any sim launch: `mount | grep docker-data` and `docker ps --filter name=WinBoat` (tell the user at once if it is up). After a reboot `up -d` recreates the container (gotcha 57) and then setuptools needs the check in gotcha 58.
- With a sim up, the ROS-based tests need `ROS_DOMAIN_ID=77`. `docker compose restart ros2` between sim runs. A cold start can hit the silent `/clock` flake (gotcha 33): relaunch.
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

A workflow file that runs in CI, with the result of one real run stated (or the reason it cannot run on a hosted runner and what runs instead); the Dec 8 row updated; this file rewritten for session 16. Say plainly which changes are code, which are plan-level and which are documentation. For each "not shown" gap, say whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D42). The contribution claim changes only with the user's yes.
- Do not push without asking.
