# Next-session prompt: camera to nadir (session 5, T6)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

The Nov 15 trigger is met early (2026-11-12): `ros2 launch uav_planning swarm.launch.py` brings up 3 iris and 3 followers, which each hold a different stub peak about 102 s after a cold launch, in 2 of 2 runs. The repo is public as `raafayk7/ros2-uav-sector-search`, so do not commit private notes or career content.

What is left before the Nov 20 checkpoint and Nov 24 (build order from `docs/roadmap.md`): camera to nadir (T6, 2 h timebox), sim speed (T7, 2 h timebox), ground-truth detector, sweep controller, `belief_map_node` + footprints, the Nov 20 search loop (T8), then the mission state machine. That is more than the remaining sessions hold at 12-15 h a week, so the state machine may slip past Nov 24 (already expected in the roadmap).

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1 (done 2026-11-07) | Shared datum, ROS-free library, tests | T1, T2 | Sonnet | Rules were all pinned; pure functions | ~3 h |
| 2 (done 2026-11-08) | Node, arm helper, stub map, first flight | T3, T4 | Opus | Live SITL debugging | ~4 h |
| 3 (done 2026-11-09) | Secret scan, ignore audit, fresh-clone test, WIP README | none | Sonnet | Mechanical | ~1 h |
| 4 (done 2026-11-12) | Swarm launch, 3 followers on 3 peaks | T5 | Opus | Live multi-process SITL debugging | ~3 h |
| **5 (this prompt)** | Camera to nadir, `mount_quat` exposed, marker test passes | T6 | **Opus** | The method (ArduPilot mount params) is untried, and the evidence is a live marker test in SITL | ~2-3 h |
| 6 | Sim speed timebox: re-measure 1 and 4 iris, try the D23 candidates | T7 | Opus | Diagnosis across Gazebo, SITL and DDS, with measurements that can mislead | ~2 h |
| 7 | Ground-truth detector (Gazebo target poses + noise, every frame including empty) | part of T8 | Sonnet | Specified in the roadmap; mostly a node and tests, once noise values are picked | ~3 h |
| 8+ | Sweep controller, `belief_map_node` + footprints, Nov 20 search loop | T8 | Opus | Integration across nodes in live SITL | ~4-6 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 5. Three followers already fly in headless SITL and each hold a different stub peak (`swarm.launch.py`, T5). This session points the simulated camera straight down (T6, D30), because the search loop needs each drone to see the sector it hovers over.

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/planner-decisions.md` **R25 / D30** for the evidence and the accepted scope: at 10 m the view axis is 52.2 deg off nadir and hits ground about 13 m from the drone, the horizon is in frame, and `leader_node.py:354` then throws away the frame's negative evidence. A correct `mount_quat` alone fixes the maths, not where the camera points.
- `docs/probes/mount_check.py`: the marker test. It spawns coloured boxes, grabs one frame and compares centroids with the pixels predicted for a nadir mount and for the measured mount. It reads `/camera/image` and `/camera/camera_info`, which is the single-drone `iris_runway` topics. In the swarm they are `/iris_N/camera/*` and only exist with `camera:=true`.
- `docs/docker-notes.md` gotchas 31-37, especially 35 (no `/clock` with namespaced drones) if you test in the swarm.
- Facts that save time: `gazebo-iris-gimbal.parm` (upstream `ardupilot_gazebo` config, already in the SITL `defaults` list) sets `MNT1_TYPE 1`. A raw `/gimbal/cmd_*` publish left the joints unchanged (2026-11-04). ArduPilot mount params have not been tried.

## Task

**T6 (P1), 2 h timebox: camera to nadir and expose `mount_quat`.**

1. Point the gimbal straight down through ArduPilot mount configuration. A lead to try first, untested: the mount's default mode and its neutral or retracted angles (`MNT1_DEFLT_MODE` and `MNT1_NEUTRAL_*` / `MNT1_RETRACT_*`, check the names against this ArduPilot version), set from a project `.parm` file appended to the SITL `defaults` list. The swarm launch already builds that list in `_sim_actions`; `iris_runway` would need its `defaults` arg overridden.
2. Prove it with the marker test: the marker straight below the drone lands at the image centre and every image corner hits the ground.
3. Expose `mount_quat` as a launch arg in `src/uav_mapping/launch/heatmap.launch.py`. `leader_node.py:354` stays as it is (D30).

If nadir does not work within 2 h, stop and bring the options back to the user (D30's accepted scope). Record what was tried, with numbers, in `docs/planner-decisions.md` as "Implementation notes (session 5)", and add any new trap to `docs/docker-notes.md`.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition, so the USB stays mounted (after a reboot, check it before `docker compose up -d`; on 2026-11-12 a reboot left it unmounted and Docker hung). Use `stop`, never `down`.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`.
- `pkill` leaves `arducopter`/`MicroXRCEAgent` behind; use `docker compose restart ros2` between sim runs.
- `colcon build --symlink-install` copies `uav_planning`'s Python modules, so rebuild after every edit (gotcha 32).
- Close Chrome before 3-drone runs: with it open, the host had under 1 GB of 7.1 GB available before the sim started.
- Tests without a simulator: `python3 -m pytest src` (72 passed, 2026-11-12).
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

The camera looks straight down in SITL with the marker test as proof (or a clear "did not work in 2 h" with what was tried), `mount_quat` exposed, notes recorded, and `docs/next-session-prompt.md` rewritten for session 6 (T7, sim speed). Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (ArduPilot mount modes and how SITL loads default param files are candidates).
- All design decisions are settled, so don't reopen them without new evidence.
- Do not push without asking.
