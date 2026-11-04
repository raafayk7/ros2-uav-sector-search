# Next-session prompt: planner library (session 1 of 3 to Nov 10)

Paste the block below to open the next session. Replace this file after each session.

## Session plan to the Nov 10 goal

Nov 10 goal: planner library + `follower_planner_node`, first follower flying to a stub-map sector in SITL, secret scan clean, repo public as a WIP. Times are estimates.

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| **1 (this prompt)** | Shared datum, ROS-free library, tests | T1, T2 | **Sonnet** | Every rule is pinned in `docs/planner-decisions.md`, and pure functions give fast test feedback | ~3 h |
| 2 | Node, arm helper, stub map, first flight | T3, T4 | **Opus** | Live SITL debugging across DDS, two RMWs and silent autopilot drops, plus teaching new ROS concepts | ~4-5 h |
| 3 | Secret scan, WIP README, going public | step 7 of the old plan, below | **Sonnet** | Mechanical, with one outward-facing step that needs asking first | ~1-2 h |

After Nov 10: session 4 (Opus) is the swarm launch and 3 followers (T5), for the Nov 15 trigger.

If session 1 fights the conflict rules for more than about 30 minutes, switch it to Opus rather than pushing on.

**Session 3 scope, kept here so it isn't lost:** run the secret scan over full history: `docker run --rm -v "$PWD":/repo zricethezav/gitleaks:latest detect --source /repo -v`. Confirm `docker/.env` is gitignored and was never committed (`git log --all -- docker/.env`), and rotate anything found. Write a WIP README: architecture diagram placeholder, one-command quickstart, honest status. **Ask the user before making the repo public.**

---

Continuing the multi-agent UAV ROS 2 rebuild: session 1 of 3 to the Nov 10 goal. The design review finished on 2026-11-04. No planner code exists yet. This session writes the shared datum and the ROS-free planner library with its tests, and does not touch flight.

## Read first

- `docs/planner-decisions.md`: start with "Decisions at a glance", then "Implementation Tasks". **It wins over anything else that disagrees**, including the thesis. In the ledger, a record marked SUPERSEDED points to the record to build from.
- `src/uav_localization/uav_localization/localization_node.py` and `src/uav_mapping/uav_mapping/leader_node.py`: mirror their package conventions.
- `CLAUDE.md`, and `docs/docker-notes.md` gotchas 21-30.
- Optional, as reference only: `docs/probes/algo_probe5.py`, variant C, is the conflict rule these tests must reproduce.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition, so the USB stays plugged in. Start with `docker compose -f docker/docker-compose.yml up -d`; use `stop`, never `down`.
- In a non-interactive `docker compose exec ros2 bash -lc '...'`, run `source /usr/local/bin/entrypoint.sh` first.
- Run tests in the container: `python3 -m pytest` (pytest 7.4.4). The library itself needs no ROS. The repo has no tests yet, so this session sets the pattern.
- Long commands go in the background with logs in the scratchpad. Commit after each working sub-task on `main`. Do not push.

## T1: shared datum in `localization_node`

Optional `datum_lat`, `datum_lon`, `datum_alt` params. Set: use them as the origin. Unset: today's first-fix latch, unchanged.

Tests:
- **Critical regression:** with the params unset, the first fix reports (0, 0, 0).
- Two nodes with different first fixes and one datum report one frame, 5 m apart.

For the runway world, measure the datum rather than assuming it: `/iris_1/navsat` at world (0, 0) read about lat -35.3632621, lon 149.1652374, alt 10.19 at z = 0.195.

Reuse the projection as it stands; don't restructure a verified node beyond this.

## T2: package `uav_planning`, module `core/planner.py`

ament_python, one package (D6). `core/` imports no ROS and no message types, so the 2D eval sim can use it unchanged. Suggested shape, adjust if a cleaner one appears: a small grid spec, plain data for peers, pure functions.

Behaviour to implement:

| Rule | Exactly | Ref |
|---|---|---|
| Indexing | World to sector, sector centre; pin (7.5, 7.5) to col 1, row 1 at 5 m. `leader_node` returns `(col, row)` but indexes `[row, col]`, so pick one convention and name it | D20 |
| Map values | `-1` counts as 0.5; 0-100 maps to 0-1. Each cell is an independent probability | D18, D31 |
| Blocked set | Zone = a sector + its 8 neighbours. An arrived peer's position zone blocks. A flying peer's position does not. A valid goal blocks only if that peer has the lower sysid | D24, D25 |
| Peer validity | A peer silent for `peer_timeout` (5 s) stops counting. A goal is valid only for `peer_timeout` after its last stamp, independent of pose | D21, D28 |
| Choosing | Best open sector anywhere on the grid. Move only if it beats the current goal's value by more than `move_margin` (0.02). A blocked current goal counts as value 0. Ties: nearest (Chebyshev), then lowest index. Nothing open: keep the goal, hold | D16, D17, D18 |
| Conversion | Arena to lat/lon, using the datum; inverse of `localization_node`'s projection | D8 |
| Baselines | Thesis 8-neighbour DGA step and BLL step, not used at runtime | D4, D17 |

One detail the decisions leave open, so apply this and tell the user: treat a peer as "arrived" when it has no valid goal, or is within `arrival_radius` of its goal. A disabled drone therefore still blocks where it is parked, which is the safe reading.

Tests (the value cards are in the newest `~/.gstack/projects/raafayk7-multi-agent-uavs/*-eng-review-test-plan-*.md`):
- Every row above, including everything-blocked and a value dropping so the holder leaves.
- Simultaneous decisions with peer state one tick stale: the higher sysid yields.
- Arena to lat/lon round trip against `localization_node`'s projection within 0.01 m.
- BLL: move probability matches the softmax formula, seeded runs repeat, moves stay on the grid. **Report** the iteration count beside the thesis's 153, with the setup stated; do not assert it (D22).
- Coordination harness with fake poses and no flight: 3 drones, 3 peaks on 8x8. The Python probes reached 100% of seeds, so anything lower needs an explanation, not a loosened test.

## Done means

Tests pass in the container, `core/` imports without ROS, `leader_node` is untouched, and it's committed. End by rewriting this file for session 2, whose scope is T3 and T4 in `docs/planner-decisions.md`, keeping the session 3 scope above.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a NEW concept in one paragraph of "what it is / when you care" before code. Boilerplate: write it and explain briefly.
- All design decisions here are settled, so don't reopen them without new evidence. If evidence does contradict one, stop and show the numbers; that happened more than once during this design.
- State plainly whether a change is in the plan docs or in code, and finish plan-level updates before closing out.
- Do not make the repo public without asking.
