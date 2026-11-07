# Next-session prompt: node, arm helper, stub map, first flight (session 2 of 3 to Nov 10)

Paste the block below to open the next session. Replace this file after each session.

## Session plan to the Nov 10 goal

Nov 10 goal: planner library + `follower_planner_node`, first follower flying to a stub-map sector in SITL, secret scan clean, repo public as a WIP. Times are estimates.

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1 (done 2026-11-07) | Shared datum, ROS-free library, tests | T1, T2 | Sonnet | Rules were all pinned; pure functions | ~3 h |
| **2 (this prompt)** | Node, arm helper, stub map, first flight | T3, T4 | **Opus** | Live SITL debugging across DDS, two RMWs and silent autopilot drops, plus teaching new ROS concepts | ~4-5 h |
| 3 | Secret scan, WIP README, going public | step 7 of the old plan, below | **Sonnet** | Mechanical, with one outward-facing step that needs asking first | ~1-2 h |

After Nov 10: session 4 (Opus) is the swarm launch and 3 followers (T5), for the Nov 15 trigger.

**Session 3 scope, kept here so it isn't lost:** run the secret scan over full history: `docker run --rm -v "$PWD":/repo zricethezav/gitleaks:latest detect --source /repo -v`. Confirm `docker/.env` is gitignored and was never committed (`git log --all -- docker/.env`), and rotate anything found. Write a WIP README: architecture diagram placeholder, one-command quickstart, honest status. **Ask the user before making the repo public.**

---

Continuing the multi-agent UAV ROS 2 rebuild: session 2 of 3 to the Nov 10 goal. Session 1 built and committed T1 (shared datum in `localization_node`) and T2 (`uav_planning/core/planner.py`, ROS-free, 44 tests, plus a 200-seed harness at 200/200). This session wraps that library in a node and flies one follower. Scope is T3 and T4 in `docs/planner-decisions.md`.

## Read first

- `docs/planner-decisions.md`: "Decisions at a glance", then "Implementation Notes (session 1)" (readings the library chose where the record was open), then T3 and T4 under "Implementation Tasks". It wins over anything else that disagrees. The ledger entries behind T3/T4: R2 (D8 `cmd_gps_pose`), R3 (D9 static peers), R4 (D10 stepping), R6/R22/R24 (arrival 2.0 m, status/enable, hold on disable), R9 (D15 arm helper on Fast DDS), R14 (D19 8x8 stub map), R21 (D26 altitude gate), R23 (D28 goal republish).
- `src/uav_planning/uav_planning/core/planner.py`: the library. Do not re-implement any rule in the node. The node calls `plan_step` once a tick and translates.
- `src/uav_localization/uav_localization/localization_node.py` and `src/uav_mapping/uav_mapping/leader_node.py`: node conventions (absolute-default input topics, relative output topics, QoS comments).
- `CLAUDE.md` and `docs/docker-notes.md` gotchas 21-30 (non-interactive shells, services hang under CycloneDDS, silent `cmd_gps_pose` drops, `pkill` leftovers).

## What session 1 left you

Library API (all in `uav_planning.core.planner`): `GridSpec` (`world_to_sector`, `sector_centre`, `zone`), `values_from_occupancy(data, grid)`, `Peer(sysid, pose, pose_stamp, goal, goal_stamp)`, `PlannerParams(move_margin=0.02, peer_timeout=5.0, arrival_radius=2.0)`, `plan_step(grid, values, my_sysid, my_pose, current_goal, peers, now, params) -> Decision(goal, changed, reason)`, `is_arrived`, `Datum` and `arena_to_lla(datum, x, y, up) -> (lat, lon, alt)`.

Conventions: `Sector(col, row)`, `values[row][col]`, OccupancyGrid `data[row * width + col]`.

Things the library does NOT do, so the node must:

- **Altitude gate (D26):** HOLDING with no horizontal goal until altitude is within 0.5 m of the layer; ARRIVED needs `is_arrived` and altitude. The library's arrival is horizontal only.
- **Command frame:** `GlobalPosition` with `header.frame_id = "map"` and `FRAME_GLOBAL_REL_ALT`. `arena_to_lla` returns an absolute alt; for REL_ALT send the layer height instead. Any other `frame_id` is dropped silently, as is anything sent when not GUIDED and armed.
- **Goal republish (D28):** publish `/<robot>/goal_sector` at 1 Hz with a fresh stamp, RELIABLE + VOLATILE, only while the goal is real. Stamps come from the node's clock (sim time).
- **Disable (D29):** `planner_enable` false sends one goal at the current position and layer, then nothing.
- **No heatmap yet:** HOLDING, no goals.
- A re-sent goal at 1 Hz until arrival (D8); warn if `/ap/vN/goal_lla` never matches.

Run tests with `python3 -m pytest src` from the repo root inside the container (`conftest.py` files put each package on `sys.path`). The planner round-trip test needs ROS sourced and skips without it.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition, so the USB stays plugged in. Start with `docker compose -f docker/docker-compose.yml up -d`; use `stop`, never `down`.
- In a non-interactive `docker compose exec ros2 bash -lc '...'`, run `source /usr/local/bin/entrypoint.sh` first.
- Planner stays on `rmw_cyclonedds_cpp`; only `arm_takeoff` runs on `rmw_fastrtps_cpp` (set in launch for that process).
- `docker compose restart ros2` between sim runs. Headless launch: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false`.
- Long commands go in the background with logs in the scratchpad. Commit after each working sub-task on `main`. Do not push.

## T3: `follower_planner_node`

A thin ROS wrapper: params `robot`, `sysid`, `peers`, `heatmap_topic` (default `/heatmap`), layer height, the three `PlannerParams`, and the shared datum. One 1 Hz sim-time timer on the default single-threaded executor. Subscribes to its own `/<robot>/pose`, each peer's `/<robot>/pose` and `/<peer>/goal_sector`, the heatmap (RELIABLE + TRANSIENT_LOCAL) and `planner_enable`. Publishes `goal_sector`, `planner_status` (ASCENDING, ARRIVED, HOLDING) and `/ap/vN/cmd_gps_pose` (BEST_EFFORT + VOLATILE). Introduce timers/executors and QoS in one paragraph each before code. Test: pure node logic where it can be separated (message built with `frame_id "map"` and REL_ALT, HOLDING below layer, no map means no goals); otherwise SITL.

## T4: `arm_takeoff` and `stub_heatmap_publisher`

- `arm_takeoff`: sets GUIDED, arms and takes off through `/ap/vN/*` services, then exits. Fast DDS for that process only. Verify it exits after the climb.
- `stub_heatmap_publisher`: static `OccupancyGrid` on `/heatmap`, RELIABLE + TRANSIENT_LOCAL, 8x8 at 5 m, peaks and geometry as params. Verify a late subscriber receives the map.

## Done means

One follower, in headless SITL: launch, arm, climb to its layer, fly to the best stub sector and report ARRIVED (within 2 m and at altitude); `planner_enable` false mid-flight holds the drone. Tests pass; committed. If flight does not work within the time, commit what runs and write down exactly where it stops (the log line, the topic, the RMW). End by rewriting this file for session 3, keeping its scope above, and updating `docs/planner-decisions.md` task checkboxes and notes. Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a NEW concept in one paragraph of "what it is / when you care" before code. Boilerplate: write it and explain briefly.
- All design decisions are settled, so don't reopen them without new evidence. If evidence does contradict one, stop and show the numbers; that happened more than once during this design.
- Probe with bounded runs, show the numbers and their limits.
- Do not make the repo public without asking.
