# Next-session prompt: sweep controller (session 8)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 7 is done (2026-11-17). The ground-truth detector exists: package `uav_perception`, node `ground_truth_detector`, ROS-free core with 18 tests (`python3 -m pytest src`: 90 passed). It reads `target_*` model poses from Gazebo, projects them into the drone's camera, applies the noise the user agreed (D38: miss 0.20, false alarm 0.05 per frame, 5 px, 2 Hz frames), and publishes `/detections` on every frame, empty ones included. Checked live with one hovering iris against `leader_node`: the cell holding a target went to 97, covered empty cells to 3, uncovered cells stayed unknown, and removing the target dropped its cell from 97 to 3 in about 35 s. The sim camera is 640x480, fx 205.47, 10 Hz (a 31 x 23 m footprint at 10 m), not the lens the stub assumed.

What is left before the Nov 20 checkpoint and Nov 24 (build order from `docs/roadmap.md`): sweep controller, `belief_map_node` + footprints, the Nov 20 search loop (T8), then the mission state machine. At 12-15 h a week, Nov 20 is likely to slip by a few days, and the state machine may slip past Nov 24 (already expected in the roadmap).

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-6 (done 2026-11-07 to 11-16) | Datum, library, node, first flight, swarm, nadir camera, sim speed | T1-T7 | mixed | see git history | ~15 h |
| 7 (done 2026-11-17) | Ground-truth detector, noise agreed (D38), checked live against `leader_node` | part of T8 | Sonnet | Specified in the roadmap; a node and tests | ~2.5 h |
| **8 (this prompt)** | Sweep controller for the leader; marker test during sideways flight before footprints are trusted | T8 | **Opus** | Live flight, and the pointing check can fail silently | ~3 h |
| 9 | `belief_map_node` + footprints, Nov 20 search loop; decide R29 (repeated frames) and D32's N here; one detector per drone (follower detectors) and the `gz` query cost with 3-4 of them | T8 | Opus | Integration across nodes in live SITL | ~3-4 h |
| 10+ | Mission state machine (Nov 24, may slip) | — | Opus | End-to-end debugging | ~4 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 8. Three followers fly in headless SITL (`swarm.launch.py`), every camera points straight down, sim speed is understood (2 ms step, D37), and the ground-truth detector feeds `leader_node` (session 7). This session builds the leader's boustrophedon sweep controller: the first flight code the leader has, since `leader_node` only maps.

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/roadmap.md`: Leader row (sweep controller, lane order biased by prior later), the t = 0 and coverage-guarantee lines (the leader always completes the full sweep), and the Nov 24 build order.
- `docs/planner-decisions.md`: **session 7 notes** (detector, D38, what was not shown), session 6 notes (D37), R28/D33 (the checkpoint) and the T8 task with its "verify first" step.
- `src/uav_planning/uav_planning/follower_planner_node.py`, `arm_takeoff.py`, `core/follower.py`: the flight pattern to reuse (`/ap/vN/cmd_gps_pose`, `frame_id "map"`, `FRAME_GLOBAL_REL_ALT`, only when GUIDED and armed; docker-notes 27).
- `docs/docker-notes.md` gotchas 39-50 (new: 47 real camera K and footprint, 48 `heatmap.launch.py stub:=false`, 49 the `pkill -f` trap, 50 spawning and removing targets).
- `docs/probes/mount_check.py` and `docs/probes/detector_live_check.py`.

## Task

1. **Design the sweep with the user before coding** (architecture, the part they are learning): lane spacing from the 31 x 23 m footprint at the flight altitude (is the 20 x 20 m arena covered in one lane at 10 m? at what altitude and speed does a sweep take long enough to matter?), where the leader starts (corner, home), how it commands waypoints (`cmd_gps_pose` one waypoint at a time, with arrival tolerance), and what "sweep complete" means for the state machine. Offer a default with reasons.
2. **Build it**: pure lane-generation and waypoint-advance logic in a ROS-free module with unit tests (as in `uav_planning/core`), plus a thin node. A leader at its own altitude layer (D5); the sweep must not stop for detections (coverage guarantee).
3. **Verify first (carried from T6):** run `docs/probes/mount_check.py` while the drone flies sideways at sweep speed and check the marker error stays near the hover result (1-4 px). The gimbal's earth-stabilised pointing may not hold in translation. If it fails, do it at both physics steps (D37) and tell the user before building on footprints.
4. **Moving-drone check (carried from session 7):** the detector projects with the latest `/pose`, not the pose at the frame's stamp. While the leader sweeps, compare the heatmap cell of a known marker with the marker's real cell and note any lag-induced misses at sweep speed.
5. **Check live**: one leader sweeps the arena with `target_*` markers spawned (docker-notes 50), the detector and `leader_node` running (`stub:=false`); the heatmap should end with target cells high, swept empty cells low, and nothing left unknown if coverage is complete.

Record what was built in `docs/planner-decisions.md` as "Implementation notes (session 8)", add any new trap to `docs/docker-notes.md`, and rewrite this file for session 9.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition: check `mount | grep docker-data` before `docker compose up -d`. Use `stop`, never `down`.
- **After a reboot WinBoat may have started itself (gotcha 46):** `docker ps --filter name=WinBoat`; if it is up, `docker stop -t 120 WinBoat` and tell the user. It must be stopped for all sim work.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`. Do not put `pkill -f NAME` in a `bash -lc` string that contains NAME (gotcha 49): use a script file.
- Single-iris sim for the live work: `mkdir -p /tmp/sim1 && cd /tmp/sim1 && ros2 launch uav_planning swarm.launch.py num_followers:=1 followers:=false camera:=true`, then `uav_localization localization_node` (namespace `/iris_1`, shared datum), `arm_takeoff` on Fast DDS, `uav_perception detector.launch.py pose_topic:=/iris_1/pose camera_info_topic:=/iris_1/camera/camera_info`, and `heatmap.launch.py pose_topic:=/iris_1/pose camera_info_topic:=/iris_1/camera/camera_info stub:=false use_sim_time:=false`. It reached hover about 40 s after launch.
- `docker compose restart ros2` between sim runs; `pkill` leaves `arducopter`/`MicroXRCEAgent` behind.
- `colcon build --symlink-install` copies `uav_planning`'s files, so rebuild after every edit (gotcha 32); new packages need a build too.
- `wait_for_clock` caught a silent ROS `/clock` in 2 of 4 swarm cold starts at the 2 ms step (gotcha 33): if the launch shuts down with "no /clock", restart the container and run again.
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

Sweep design agreed with the user, the sweep controller built with unit tests, the sideways-flight marker check done and its result recorded, a live sweep checked against the heatmap, notes recorded, and this file rewritten for session 9 (`belief_map_node` + footprints). Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (for example, why a sweep is lanes at a spacing set by the camera footprint, and why the leader must not stop for a detection).
- Settled decisions stay settled (stack, D37, D38); don't reopen them without new evidence. The frame rate (2 Hz) and D32's N are knobs for session 9.
- Do not push without asking.
