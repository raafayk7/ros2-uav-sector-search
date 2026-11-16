# Next-session prompt: ground-truth detector (session 7)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

T7 is done (2026-11-16, the D23 timebox). With readiness confirmed, idle RTF is 0.96 at 1 iris and 0.24 at 4. The limit is Gazebo's main thread, at about 1 ms of CPU per drone per 1 ms physics step. MAVProxy, the camera and RAM each changed RTF by 10 % or less, so the camera stays. A 2 ms physics step with `SCHED_LOOP_RATE 250` doubled RTF and flew 3 and 4 followers to ARRIVED (hover RTF 0.72 and 0.53). For now it is only a probe (`docs/probes/rtf_variant.py step2`). Whether to make it the default is **D37, open**. The 4-iris gimbal check passed: all four at roll 0.1, pitch 90.0, yaw 0.0 deg.

What is left before the Nov 20 checkpoint and Nov 24 (build order from `docs/roadmap.md`): ground-truth detector, sweep controller, `belief_map_node` + footprints, the Nov 20 search loop (T8), then the mission state machine. At 12-15 h a week, Nov 20 is likely to slip by a few days, and the state machine may slip past Nov 24 (already expected in the roadmap).

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-5 (done 2026-11-07 to 11-16) | Datum, library, node, first flight, swarm, camera at nadir | T1-T6 | mixed | see git history | ~12.5 h |
| 6 (done 2026-11-16) | Sim speed timebox: Gazebo's main thread is the limit; 2 ms step doubles RTF | T7 | Opus | Diagnosis across Gazebo, SITL and DDS | ~2 h |
| **7 (this prompt)** | D37 if adopted (about 30 min), then the ground-truth detector (Gazebo target poses + noise, every frame including empty) | part of T8 | **Sonnet** | Specified in the roadmap; a node and tests, once the user picks noise values. Use Opus instead if D37 is adopted and its flight check misbehaves | ~3 h |
| 8 | Sweep controller for the leader; marker test during sideways flight before footprints are trusted | T8 | Opus | Live flight, and the pointing check can fail silently | ~3 h |
| 9 | `belief_map_node` + footprints, Nov 20 search loop | T8 | Opus | Integration across nodes in live SITL | ~3-4 h |
| 10+ | Mission state machine (Nov 24, may slip) | — | Opus | End-to-end debugging | ~4 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 7. Three followers fly in headless SITL and each hold a different stub peak (`swarm.launch.py`, T5). Every camera points straight down (T6). Sim speed is understood (T7): Gazebo's main thread is the limit, and a 2 ms physics step doubles RTF (D37 decides whether to adopt it). This session builds the ground-truth detector, the primary perception path for the Nov 30 go/no-go.

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/planner-decisions.md`: **session 6 notes** and **D37** under "Unresolved decisions"; R29/R30 (D34, D35) for the two detector-related TODOs.
- `TODOS.md`, "Belief map": **empty frames must reach the map** (apply it in this node) and **repeated frames over-count evidence** (needs this node's noise settings).
- `docs/roadmap.md`, Perception row: ground-truth detector = Gazebo target poses + noise, misses, false positives, footprint check, publishing on every frame including empty ones.
- `src/uav_mapping/uav_mapping/leader_node.py` and `fake_detection_publisher` for the `Detection2DArray` contract the detector must match (frame, stamp, pixel coordinates, camera_info).
- `docs/docker-notes.md` gotchas 39-45.

## Task

0. **D37 first.** If the user adopted the 2 ms step, add it to `swarm.launch.py` as a launch arg (for example `physics_step`, default 0.002) that writes a world copy and puts `SCHED_LOOP_RATE 250` in a `uav_planning/config` parm file, before the `dds_*.parm` files (gotcha 39). Then run one 3-follower flight (`swarm_watch.py 3 240`) to check that all ARRIVED on (3,4), (6,1), (1,6). If the user kept 1 ms, skip this step and record D23's fallback choice in `docs/planner-decisions.md`.
1. **Pick noise values with the user before coding.** Miss rate, false-positive rate, pixel noise. These are architecture inputs: the D32 confirmation N and the repeated-frames TODO both depend on them. Offer a default set with reasons; don't decide alone.
2. **Build the detector node** (package per the roadmap). Inputs: target poses from Gazebo (choose how: a `ros_gz_bridge` pose topic or `/world/runway/pose/info`), the drone's pose, `camera_info` and the nadir mount. Project each target into the image, keep those inside the frame (the footprint check), apply noise, and publish a `Detection2DArray` **on every frame, including empty ones**. Pure projection and noise logic in a ROS-free module with unit tests, as in `uav_planning/core`.
3. **Check it live against `leader_node`**: spawn the markers as in `docs/probes/mount_check.py`, hover over them, and confirm the heatmap raises the marker's sectors and lowers the covered empty ones.

Record what was built in `docs/planner-decisions.md` as "Implementation notes (session 7)", and add any new trap to `docs/docker-notes.md`.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition: check `mount | grep docker-data` before `docker compose up -d`. Use `stop`, never `down`.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`.
- Launch `iris_runway` from a fresh directory (`mkdir -p /tmp/sitl && cd /tmp/sitl`), not the repo root (gotcha 40).
- `docker compose restart ros2` between sim runs; `pkill` leaves `arducopter`/`MicroXRCEAgent` behind.
- `colcon build --symlink-install` copies `uav_planning`'s files, so rebuild after every edit (gotcha 32).
- Keep the WinBoat VM stopped for 3-4 drone runs (gotcha 42); RAM was not the limit with it stopped.
- `wait_for_clock` caught a silent ROS `/clock` once in 6 swarm cold starts (gotcha 33): if the launch shuts down with "no /clock", restart the container and run again.
- Tests without a simulator: `python3 -m pytest src` (72 passed, 2026-11-16).
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

D37 applied (or the fallback recorded), noise values agreed with the user, the detector built with unit tests, publishing on empty frames, checked live against `leader_node`, notes recorded, and `docs/next-session-prompt.md` rewritten for session 8 (sweep controller). Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (for example, how a ground-truth detector stands in for YOLO, and why empty frames are evidence).
- All design decisions are settled except D37 and the noise values, so don't reopen others without new evidence.
- Do not push without asking.
