# Next-session prompt: belief map and the leader in the swarm (session 9)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 8 is done (2026-11-17). The leader has flight code: `uav_planning/sweep_node` with a ROS-free `core/sweep.py` (55 tests; `python3 -m pytest src`: 145 passed) and `sweep.launch.py`. The user decided D39: 8x8 arena at 5 m (40 x 40 m, now `leader_node`'s default too), 5 m/s (`WP_SPD`, set by `arm_takeoff wp_speed:=5`), lane spacing from the footprint's short side x 0.8. In two cold runs the leader flew 3 north lanes at 8 m and latched `sweep_status` COMPLETE about 40 s into the sweep. The T8 verify-first check passed: at 5 m/s cruise the marker error was median 3.1-3.3 px, max 7.0. Detection hits landed median 0.2 m from the true target, with no lag misses. The map ended with targets at 96-97, every empty cell below 50 (median 3, edge columns 10-35) and no unknown cells.

What is left before Nov 24 (build order from `docs/roadmap.md`): `belief_map_node` + footprints, a leader role in the swarm launch, the Nov 20 search loop (T8, now likely Nov 21-22), then the mission state machine. At 12-15 h a week the state machine is likely to slip past Nov 24 (already expected in the roadmap).

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-7 (done 2026-11-07 to 11-17) | Datum, library, node, first flight, swarm, nadir camera, sim speed, ground-truth detector | T1-T7, part of T8 | mixed | see git history | ~17.5 h |
| 8 (done 2026-11-17) | Sweep controller (D39), pointing in motion, detection lag, live sweep against the heatmap | part of T8 | Opus | Live flight and a silent-failure check | ~2.5 h |
| **9 (this prompt)** | Design with the user, then build: `belief_map_node` (R29 evidence per frame, D32's N, footprints from every drone) and a leader role in `swarm.launch.py`; unit tests; a short live check with leader + 1 follower | T8 | **Opus** | Architecture the user is learning, plus a new node across drones | ~3-4 h |
| 10 | Nov 20 search loop live: leader sweeps, one follower observes, the map updates, it departs, it finds; one detector per drone and the `gz` query cost with several | T8 | Opus | Integration debugging in live SITL | ~3 h |
| 11+ | Mission state machine (Nov 24, may slip) | — | Opus | End-to-end debugging | ~4 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 9. One leader sweeps the 40 x 40 m arena at 8 m (`sweep.launch.py`, D39), the ground-truth detector feeds `leader_node`, and three followers fly in the swarm launch. This session designs and builds the `belief_map_node` that the roadmap makes the single owner of the posterior. It also gives the swarm launch a leader, so the Nov 20 loop can run next session.

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/roadmap.md`: the Belief map row, the cell-meaning and starting-belief lines (alpha blend, independent per-cell probabilities), the Mission row (D14's end rule), and the Nov 20 / Nov 24 lines.
- `docs/planner-decisions.md`: **session 8 notes** (D39, edge columns, the leader role gap), session 7 notes (D38, frame rate 2 Hz as a knob), R26/D31 (map meaning), R27/D32 (confirmed find, N consecutive frames), R29 (repeated frames over-count: a cell clamps in about 5 frames), R30 (empty frames must reach the map), D13 (lowering a found sector), D14 (mission end).
- `src/uav_mapping/uav_mapping/leader_node.py`: the log-odds update to extend or replace (footprint quad, `_cells_in_quad`, `l_occ`, `l_free`, clamp).
- `src/uav_planning/launch/swarm.launch.py` and `sweep.launch.py`; `uav_perception/launch/detector.launch.py` (global topic defaults, session 7 gap).
- `docs/docker-notes.md` gotchas 47-53 (new: 51 a bare `y` launch value becomes a boolean, 52 `set_parameters` saves to eeprom, 53 the 8x8 default).

## Task

1. **Design with the user before coding** (architecture, the part they are learning). Offer a default with reasons for each:
   - Where `belief_map_node` sits next to `leader_node`: replace it, or wrap its update. Which topics each drone publishes (detections plus a footprint, or detections plus pose and `camera_info` so the map computes footprints itself).
   - R29: how much one frame counts. Repeated frames of the same view are not independent, so a rate limit or a per-cell cap per pass is needed. Pick a rule and show what it does to the session-8 edge columns (10-35).
   - D32's N (provisional 3) and how a follower's confirmation lowers its sector (D13).
   - The leader in `swarm.launch.py`: which drone and sysid it is, its 8 m layer against followers at 10 + 2 x index (D5, D26), and its spawn at the arena corner. It also needs per-drone detectors with namespaced topics.
2. **Build it**: ROS-free fusion core with unit tests (as in `uav_planning/core` and `uav_perception/core`), a thin node, and the swarm-launch change.
3. **Check live, short**: leader + 1 follower in one sim, both detectors up. The map should fill from both drones' footprints and the follower should still fly to an open sector. Measure RTF with 2 detectors (the `gz` query cost, session 7 gap). The full search loop is session 10.

Record what was built in `docs/planner-decisions.md` as "Implementation notes (session 9)", add any new trap to `docs/docker-notes.md`, and rewrite this file for session 10.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition: check `mount | grep docker-data` before `docker compose up -d`. Use `stop`, never `down`.
- **WinBoat must be stopped (gotchas 42, 46):** `docker ps --filter name=WinBoat`; if it is up, tell the user at once.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`. Do not put `pkill -f NAME` in a `bash -lc` string that contains NAME (gotcha 49).
- Session-8 leader stack, from a fresh directory: `rm -rf /tmp/sim1 && mkdir -p /tmp/sim1 && cd /tmp/sim1 && ros2 launch uav_planning swarm.launch.py num_followers:=1 followers:=false camera:=true`, then `uav_mapping heatmap.launch.py pose_topic:=/iris_1/pose camera_info_topic:=/iris_1/camera/camera_info stub:=false use_sim_time:=false`, `uav_perception detector.launch.py pose_topic:=/iris_1/pose camera_info_topic:=/iris_1/camera/camera_info`, then `uav_planning sweep.launch.py`. The sweep starts about 25-30 s after `sweep.launch.py`; arming is refused for the first 10-15 s.
- `docs/probes/sweep_live_check.py [SECS]` spawns three coloured `target_*` boxes and prints pointing, detection error and the final map. When waiting on a log, first truncate it or match only new lines: a wait loop matched the previous run's log once (session 8).
- `docker compose restart ros2` between sim runs; `pkill` leaves `arducopter`/`MicroXRCEAgent` behind.
- `colcon build --symlink-install` copies `uav_planning`'s files, so rebuild after every edit (gotcha 32).
- `wait_for_clock` caught a silent ROS `/clock` in 2 of 4 swarm cold starts at the 2 ms step (gotcha 33): if the launch shuts down with "no /clock", restart the container and run again.
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

Belief-map design agreed with the user (R29 rule, N, topics, leader role), `belief_map_node` built with unit tests, the swarm launch able to start a leader with followers, a short live check of the map filling from two drones with RTF measured, notes recorded, and this file rewritten for session 10 (the Nov 20 loop). Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (for example, why repeated frames of one view are not independent evidence, and what "single owner of the posterior" buys).
- Settled decisions stay settled (stack, D37, D38, D39); don't reopen them without new evidence. The 2 Hz frame rate and D32's N are this session's knobs.
- Do not push without asking.
