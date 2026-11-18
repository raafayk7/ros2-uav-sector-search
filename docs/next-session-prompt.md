# Next-session prompt: four drones, then the mission state machine (session 10)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 9 is done (2026-11-18). The user decided D40: one `belief_map_node` that computes every drone's footprint itself; a per-look evidence cap of ±0.85 (R29); D32's N = 3, with a confirmed sector published as 0; and iris_1 as the corner leader in `swarm.launch.py leader:=true`. Built: `uav_mapping/core` (geometry + belief, 33 tests; `python3 -m pytest src`: 178 passed), `belief_map_node`, the swarm leader role (one detector per drone, followers spawned 5 m south of the arena), and a SIGTERM fix. Live, leader + 1 follower in two cold runs: the follower confirmed all 3 targets, 10-29 s into the sweep, with no false confirmations. **That meets the Nov 20 checkpoint (D33) two days early.** Each detector costs about 2 % RTF at 2 drones (0.99 → 0.95).

What is left: the Nov 24 mission state machine and the Nov 30 go/no-go (leader + 3 followers complete a mission in Gazebo from one command, video v1). Nothing has run with more than 2 drones and a belief map yet.

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-8 (done 2026-11-07 to 11-17) | Datum, library, node, first flight, swarm, nadir camera, sim speed, detector, sweep | Nov 10, Nov 15 | mixed | see git history | ~20 h |
| 9 (done 2026-11-18) | D40 belief map, leader role, two live runs | Nov 20 (met) | Opus | Architecture plus a new node across drones | ~2.5 h |
| **10 (this prompt)** | Leader + 3 followers live (RTF, 3 targets, crowding), a target near a sector edge; then design the mission state machine with the user and build its ROS-free core | Nov 24 | **Opus** | Live debugging at 4 drones, and architecture the user is learning | ~3-4 h |
| 11 | Mission node live end to end from one command, leader + 3 followers, D14 end rule measured | Nov 24 (likely Nov 22-24) | Opus | End-to-end integration in SITL | ~3 h |
| 12 | Go/no-go dress run and video v1 (Gazebo GUI capture) | Nov 30 | Sonnet if 11 lands clean, else Opus | Mostly running and recording | ~2 h |

At 12-15 h a week, sessions 10-11 fit before Nov 24 and session 12 before Nov 30, with slack for one bad sim day.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 10. `swarm.launch.py leader:=true` flies a corner leader (8 m sweep, D39) and followers that read one `belief_map_node` (D40). With one follower, all 3 targets were confirmed before the sweep ended (Nov 20 loop, met). This session scales it to 3 followers, then designs and starts the mission state machine (Nov 24).

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/planner-decisions.md`: **session 9 notes** (D40, the probe table, both live runs, the "not shown" table), D14 (mission end), D12/D29 (`planner_status`, `planner_enable`, hold on disable), D24 (goal conflicts), D13 and D32.
- `docs/roadmap.md`: the Mission row, "Mission timing", the Nov 24 and Nov 30 lines, and the open question on sweep time vs timeout.
- `src/uav_mapping/uav_mapping/core/belief.py` and `belief_map_node.py`; `src/uav_planning/launch/swarm.launch.py` (`_leader_actions`, `_follower_actions`).
- `docs/docker-notes.md` gotchas 54-56 (new: RTF from `/stats` deltas, `up -d` recreated the container, off-grid follower spawns).

## Task

1. **Four drones live** (verify first, about 1 h): `swarm.launch.py leader:=true num_followers:=3` cold, with `docs/probes/belief_live_check.py 240 3`. Measure RTF with `docs/probes/rtf_windows.sh` (4 detectors). Check that the three followers split the three targets without crowding one sector (D24, D25), and that every confirmation is right. If RTF falls below about 0.4 or the run fails, find out why before step 3.
2. **A target near a sector edge** (short): move one target to 0.3 m inside a sector edge. Do boxes split between two sectors, does the 3-frame streak still complete, and does a follower that first hovers over the wrong sector move on? Record what happens. Change the rule only with the user.
3. **Design the mission state machine with the user before coding** (architecture, the part they are learning). Offer a default with reasons for each:
   - Where it lives (a `mission_node` in `uav_planning` or a new package) and what it reads (`sweep_status`, `/heatmap`, `/confirmed_sectors`, `planner_status`) and writes (`planner_enable` per follower, a latched `/mission_state`).
   - States: the roadmap's launch → prior-guided dispatch while the leader sweeps → localize → report, and what each one means now that confirmation lives in the map.
   - D14's end rule: sweep COMPLETE and every unsuppressed sector below 0.5 for 30 s, or a timeout. Which clock, and what timeout in Gazebo.
   - At the end: disable planners (D29 hold), and what the leader and followers do next (hold, land, RTL). What "report" outputs (confirmed sectors, the drone and time of each find, mission time).
4. **Build** the ROS-free core with unit tests, if time allows; the node and the live end-to-end run are session 11.

Record what was built in `docs/planner-decisions.md` as "Implementation notes (session 10)", add any new trap to `docs/docker-notes.md`, and rewrite this file for session 11.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition: check `mount | grep docker-data` first. Start a stopped container with `docker compose -f docker/docker-compose.yml start` (`up -d` recreated it once, gotcha 55). Use `stop`, never `down`.
- **WinBoat must be stopped (gotchas 42, 46):** `docker ps --filter name=WinBoat`; if it is up, tell the user at once.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`. Do not put `pkill -f NAME` in a `bash -lc` string that contains NAME (gotcha 49); `pkill -f "NAM[E]"` works.
- Session-9 stack, one command from a fresh directory: `rm -rf /tmp/sim1 && mkdir -p /tmp/sim1 && cd /tmp/sim1 && ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=1 detector_seed:=1`. Once both SITLs log "JSON received" (about 10-15 s), run `python3 /workspaces/multi-agent-uavs/docs/probes/belief_live_check.py 150 1` from the same shell setup: it spawns the targets, and the sweep starts about 25 s later. Run the launch with `docker compose exec -d` and wait on its log; match only new lines (a wait loop once matched the previous run's log, session 8).
- `docker compose restart ros2` between sim runs; `pkill` leaves `arducopter`/`MicroXRCEAgent` behind.
- `colcon build --symlink-install` copies `uav_planning`'s files, so rebuild after every edit (gotcha 32).
- `wait_for_clock` caught a silent ROS `/clock` in 2 of 4 earlier cold starts at the 2 ms step (gotcha 33). Both session-9 starts were fine. If the launch shuts down with "no /clock", restart the container and run again.
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

Four-drone run measured (RTF, confirmations, how the targets were split), the edge-target behaviour recorded, the mission state machine design agreed with the user, its core built with tests if time allows, notes recorded, and this file rewritten for session 11. Say plainly which changes are plan-level and which are code, and for each "not shown" gap, whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (for example, what a state machine buys over a launch-file sequence, and why the end rule needs a hold time).
- Settled decisions stay settled (stack, D37-D40); don't reopen them without new evidence. The D14 values (0.5, 30 s) and the timeout are this session's knobs.
- Do not push without asking.
