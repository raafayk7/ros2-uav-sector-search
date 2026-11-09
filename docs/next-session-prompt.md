# Next-session prompt: swarm launch and 3 followers (session 4, T5)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Nov 10 goal (planner library + `follower_planner_node`, one follower flying to a stub-map sector in SITL, secret scan clean, README written) is met. **The repo is public as of 2026-11-09, renamed to `raafayk7/ros2-uav-sector-search`** (local folder, container name, image tag and skill name keep `multi-agent-uavs`). Everything pushed from here on is visible to anyone, so do not commit private notes or career content.

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1 (done 2026-11-07) | Shared datum, ROS-free library, tests | T1, T2 | Sonnet | Rules were all pinned; pure functions | ~3 h |
| 2 (done 2026-11-08) | Node, arm helper, stub map, first flight | T3, T4 | Opus | Live SITL debugging | ~4 h |
| 3 (done 2026-11-09) | Secret scan, ignore audit, fresh-clone test, WIP README | none | Sonnet | Mechanical | ~1 h |
| **4 (this prompt)** | Swarm launch, 3 followers each hold a different peak | T5 | **Opus** | Live multi-process SITL debugging; has to deal with the `/clock` lead and the RTF cost | ~3-4 h |

For the Nov 15 trigger. After that: T6 camera to nadir, T7 speed timebox, T8 Nov 20 search checkpoint (see `docs/planner-decisions.md`, "Implementation Tasks").

---

Continuing the multi-agent UAV ROS 2 rebuild: session 4. One follower already flies in headless SITL (arm, climb to 10 m, fly to the best stub-map sector, ARRIVED, `planner_enable` false holds it). This session makes **three followers** do it together, each holding a different peak of the stub map on 8x8.

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/docker-notes.md` gotchas **29-34**. Two bear directly on this work:
  - **33, the `/clock` lead (unconfirmed):** in 1 run with the sim and our nodes started together, ROS `/clock` never arrived and the planner's sim-time timer never fired; in 2 runs with the sim first it ran at ~900 Hz. The swarm launch should start project nodes after the sim is up and **check `/clock` is flowing** before relying on it. Try to confirm or kill the lead rather than just work around it.
  - **34, the RTF cost:** `/clock` at ~900 Hz costs each `use_sim_time` Python node about 30-40% of a core (one sample, idle nodes). `stub_heatmap_publisher` uses sim time only for its stamp, so it could drop it. With 3 followers that is 6+ such nodes on a 6-core host, and gotcha 29 already shows RTF falling to ~0.25 with 3 iris and ~0.11 with 4. Measure before and after; this feeds T7 (D23).
- `docs/planner-decisions.md`: "Implementation notes (session 2)" for what flew; D-entries for peers (D21, D25, D28), layers (D26) and datum (D7) if the launch touches them.
- Facts that save time (CLAUDE.md, verified 2026-11-04): `multiagent.launch.py` hard-codes 4 different vehicles, so we need our own launch. Topics are `/iris_N/*` (Gazebo) and `/ap/vN/*` (autopilot, N = sysid). The command topic is `/ap/vN/cmd_gps_pose` with `frame_id "map"`; anything else is dropped silently, as is anything sent when not GUIDED and armed. `/ap/v1/*` services hang under CycloneDDS; `arm_takeoff` is the one process on Fast DDS.
- The datum in `follower.launch.py` (lat -35.3632621, lon 149.1652374, alt 10.195) was measured for the single-iris runway spawn. Re-measure it from each `/iris_N/navsat` once three drones exist, since spawn points differ.

## Task

**T5 (P1): swarm launch.** `src/uav_planning/launch/` gets a launch that brings up N followers (default 3), each with its own namespace, sysid, altitude layer (10 + 2 x index m per the roadmap), `peers` string, and a shared datum, against a sim launched by our own file (not `multiagent.launch.py`). One shared stub map. Start nodes after the sim and gate on `/clock`.

Verify (Nov 15): 3 followers each hold a **different** peak on the 8x8 stub map (default peaks (3,4), (6,1), (1,6), pairwise at least 2 apart), no two hold the same one for a sustained window, and killing one follower's goal broadcasts releases its sector for the others within `peer_timeout`. Record the numbers (time to all-ARRIVED, goal changes per drone, RTF) in `docs/planner-decisions.md` as "Implementation notes (session 4)". If three SITL instances will not run in real time on this machine, say so with the measured RTF and stop; the Nov 15 trigger is on the planner holding up, not on forcing a fix.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition, so the USB stays plugged in. `docker compose -f docker/docker-compose.yml up -d`; use `stop`, never `down`.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`. Plain `bash -c` or `bash` drops MAVProxy from PATH and SITL hangs.
- `pkill` leaves `arducopter`/`MicroXRCEAgent` behind; use `docker compose restart ros2` between sim runs.
- `colcon build --symlink-install` copies `uav_planning`'s Python modules, so rebuild after every edit (gotcha 32).
- Tests without a simulator: `python3 -m pytest src` (72 passed on a fresh clone, 2026-11-09).
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

Three followers hold three different peaks in SITL with numbers recorded, any launch/test code committed, and `docs/next-session-prompt.md` rewritten for session 5 (T6, camera to nadir; check whether Sonnet or Opus fits). Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (a launch with event handlers and conditions, `OnProcessStart`, and QoS for `/clock` are candidates).
- All design decisions are settled, so don't reopen them without new evidence.
- Do not make the repo public, or push, without asking.
