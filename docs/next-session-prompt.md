# Next-session prompt: sim speed timebox (session 6, T7)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

T6 is done (2026-11-16): `uav_planning/config/gimbal_nadir.parm` holds the gimbal at nadir. In the marker test the marker below the drone landed 1 px from the image centre, all four corners hit the ground, and the swarm launch loads the file for every drone (3 drones checked: all gimbals at nadir, all followers ARRIVED on the session-4 peaks at 88-94 s wall). The repo is public as `raafayk7/ros2-uav-sector-search`, so do not commit private notes or career content.

What is left before the Nov 20 checkpoint and Nov 24 (build order from `docs/roadmap.md`): sim speed (T7, 2 h timebox), ground-truth detector, sweep controller, `belief_map_node` + footprints, the Nov 20 search loop (T8), then the mission state machine. That is more than four days hold at 12-15 h a week, so Nov 20 is likely to slip a little and the state machine may slip past Nov 24 (already expected in the roadmap).

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1 (done 2026-11-07) | Shared datum, ROS-free library, tests | T1, T2 | Sonnet | Rules were all pinned; pure functions | ~3 h |
| 2 (done 2026-11-08) | Node, arm helper, stub map, first flight | T3, T4 | Opus | Live SITL debugging | ~4 h |
| 3 (done 2026-11-09) | Secret scan, ignore audit, fresh-clone test, WIP README | none | Sonnet | Mechanical | ~1 h |
| 4 (done 2026-11-12) | Swarm launch, 3 followers on 3 peaks | T5 | Opus | Live multi-process SITL debugging | ~3 h |
| 5 (done 2026-11-16) | Camera to nadir, `mount_quat` exposed, marker test passes | T6 | Opus | Untried method, live evidence; found two silent param traps | ~1.5 h |
| **6 (this prompt)** | Sim speed timebox: re-measure 1 and 4 iris, try the D23 candidates | T7 | **Opus** | Diagnosis across Gazebo, SITL and DDS, with measurements that can mislead | ~2 h |
| 7 | Ground-truth detector (Gazebo target poses + noise, every frame including empty) | part of T8 | Sonnet | Specified in the roadmap; mostly a node and tests, once noise values are picked | ~3 h |
| 8+ | Sweep controller, `belief_map_node` + footprints, Nov 20 search loop; marker test during sideways flight before trusting footprints | T8 | Opus | Integration across nodes in live SITL | ~4-6 h |

---

Continuing the multi-agent UAV ROS 2 rebuild: session 6. Three followers fly in headless SITL and each hold a different stub peak (`swarm.launch.py`, T5), and every drone's camera now points straight down (T6). This session is a 2-hour timebox on simulation speed (T7, D23), because a leader plus 3 followers at about RTF 0.25 turns a 5-minute mission into 20+ minutes of wall time, which matters for the Nov 30 go/no-go.

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/planner-decisions.md` **R18 / D23** for the accepted scope, and the session 4 notes ("Speed and CPU") for the numbers so far: 3 iris idle about 0.30, swarm on the ground 0.26, hovering 0.38; `gz sim` 130 %, each `/clock` subscriber about 45 % of a core.
- `docs/docker-notes.md` gotchas 29 (RTF numbers, 1 and 4 iris unconfirmed), 34-35 (`/clock` cost and namespacing), 39-42 (new from session 5: keep a `dds_*.parm` file last in SITL's defaults, a saved `eeprom.bin` beats defaults files, backgrounded launches ignore Ctrl-C, the WinBoat VM holds about 3 GB).
- `docs/probes/rtf.sh` (the earlier measurement script) and `wait_for_clock` (it prints sim/wall).

## Task

**T7 (P2), 2 h timebox: measure and try to raise sim speed.**

1. Re-measure RTF (`gz topic -e -t /stats`) at 1 and 4 iris with a working readiness check (every SITL shows "JSON received" and `/ap/vN` topics exist) before sampling. Record free RAM and swap at the same time. In the 4-iris run, also read every drone's gimbal joints (`gz topic -e -n 1 -t /world/runway/model/iris_N/joint_state`; expect roll 0, pitch 90, yaw 0 deg): T6 was checked at 1, 2 and 3 drones only. This is a carried-over check, not part of the speed timebox.
2. Try the D23 candidates one at a time and measure each: no MAVProxy for SITL instances (upstream runs it for instances 1 and up), more free RAM (WinBoat VM stopped, gotcha 42), and a camera-free follower model. **Before trying the camera-free model, check it against the plan:** D18 and the ground-truth detector rely on followers observing what they hover over. If footprints only need the known intrinsics and the mount, a camera-free model may be fine, but that is a design question for the user, not something to decide in the timebox.
3. If still slow at the end of the timebox, stop and bring back D23's fallback choice: fewer drones in Gazebo, or longer runs cut for video.

Record what was tried, with numbers, in `docs/planner-decisions.md` as "Implementation notes (session 6)", and add any new trap to `docs/docker-notes.md`.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition, so the USB stays mounted (check before `docker compose up -d`). Use `stop`, never `down`.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`.
- Launch `iris_runway` from a fresh directory (`mkdir -p /tmp/sitl && cd /tmp/sitl`), not the repo root: the root has an old `eeprom.bin` whose saved params override defaults files (gotcha 40).
- `pkill` leaves `arducopter`/`MicroXRCEAgent` behind; use `docker compose restart ros2` between sim runs.
- `colcon build --symlink-install` copies `uav_planning`'s files, so rebuild after every edit (gotcha 32).
- Free RAM before 3-4 drone runs. On 2026-11-16 the memory hog was not Chrome but the WinBoat Windows VM (`docker stop WinBoat`, about 3 GB); with it stopped, 4.3 GB was free. Free RAM is itself one of the D23 candidates, so measure with and without.
- Tests without a simulator: `python3 -m pytest src` (72 passed, 2026-11-16).
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

RTF re-measured at 1 and 4 iris with readiness confirmed, gimbals read at 4 iris, each D23 candidate tried and measured (or a clear reason it wasn't), notes recorded, and `docs/next-session-prompt.md` rewritten for session 7 (ground-truth detector). Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (real-time factor and why SITL lock-steps to Gazebo are candidates).
- All design decisions are settled, so don't reopen them without new evidence.
- Do not push without asking.
