# CLAUDE.md — Project context for Claude

This file is loaded automatically by Claude Code / Cowork when working in this repo. It's the canonical short-form context. For the full reasoning behind everything below, read `docs/assessment-ros2-migration.md`.

## What this project is

A ROS 2 rebuild of Raafay's LUMS senior project — a multi-agent UAV search-and-rescue system. A leader drone sweeps an area, detects ground markers and builds a probability heatmap; follower drones independently fly to the best open sectors on that map, then precision-land via color masking + PID. The original used distributed gradient ascent, kept here as a measured baseline.

The original implementation (Spring 2025) was ROS Noetic + Gazebo 11 + ArduPilot + MAVROS + YOLOv8. This rebuild targets a modern stack and aims to close out the future-work items the original team identified (true decentralization, GPS-denied follower phase, scale beyond 2 followers, more robust perception).

## User profile

Raafay graduated LUMS EE in 2025. Has high-level ROS understanding rather than deep technical fluency — explanations should lead with the *why* of each design choice so the rebuild doubles as a learning exercise. Comfortable with Python and Linux. Wants to leverage AI heavily for boilerplate so he can focus on architecture and the algorithm.

Primary machine: now Ubuntu 24.04 (was Debian Trixie); Docker data lives on a USB ext4 partition, see `docs/docker-notes.md`.

## The chosen stack (locked in — don't relitigate)

- **Host:** Ubuntu 24.04 (was Debian Trixie)
- **Containment:** Docker
- **ROS 2:** Jazzy Jalisco (LTS through May 2029)
- **Simulator:** Gazebo Harmonic
- **Drone harness:** `ArduPilot/ardupilot_gz` with `multiagent.launch.py`
- **ROS ↔ ArduPilot:** AP_DDS (native, ArduPilot 4.5+) — not MAVROS
- **Perception:** Ultralytics YOLO26
- **Algorithm:** each follower independently flies to the best open sector on the shared map, skipping sectors its peers have claimed or are parked on (D11, D24, D25; the thesis's graded overlap penalty is not used); thesis Distributed Gradient Ascent (DGA) and Bilinear Log-Linear (BLL) are library baselines (changed 2026-11-04 on measured evidence, see `docs/planner-decisions.md`)

Things explicitly NOT in the rebuild even though the original used them: MAVROS as primary bridge, MAVProxy in the runtime path, iq_sim launch files, Gazebo Classic 11, hand-rolled UDP broadcast, Ubuntu 20.04, central-planner hack ("Drone 1 computes for Drone 2"). See the assessment doc §5 and the "do not use" list in the `multi-agent-uavs` skill for the replacements.

## Layout

```
multi-agent-uavs/
├── docs/                       # thesis.pdf, legacy-implementation-guide.docx, assessment-ros2-migration.md, docker-notes.md
├── src/                        # colcon workspace src/ — ROS 2 packages live here
├── worlds/                     # landmarks.yaml + recorded clue outputs per world (runway 40 m, valley 120 m eval)
├── docker/                     # Dockerfile + compose
├── skills/                     # canonical multi-agent-uavs skill source + packaged .skill
├── .claude/skills/             # Claude Code skill discovery — symlinks into skills/
├── CLAUDE.md
└── README.md
```

## Docker stack status

The Docker stack is built and working on the system Docker daemon (NOT Docker Desktop — see `docs/docker-notes.md` for why). Image is `multi-agent-uavs:jazzy`, ~25 GB. To run:

```bash
docker context use default
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec ros2 bash -l   # -l: login shell, needed for MAVProxy on PATH
```

**FDM handshake: resolved (verified 2026-11-02).** After rebuilding the image, headless `iris_runway` brings up SITL with "JSON received" and 18 live `/ap/v1/*` topics (namespaced `/ap/v1/...` in the current ardupilot_gz), and GUIDED + arm + takeoff worked (+10 m). The old "zero packets on 9002" symptom did not reproduce; the likely culprit was a missing environment in non-interactive shells (see below), not the plugin. Gotchas that cause false failures:

- `docker compose exec ros2 bash -lc '...'` does not load `.bashrc`, so run `source /usr/local/bin/entrypoint.sh` first (it sets `GZ_SIM_RESOURCE_PATH`/`SDF_PATH`; without them `robot_state_publisher` dies on the iris SDF and the drone never spawns).
- Use `bash -lc`, never plain `bash -c`, and `bash -l` for an interactive shell (plain `bash` has the same gap): only a login shell puts MAVProxy (`~/venv-ardupilot/bin`) on PATH. Without it SITL waits forever on TCP 5760 and never reaches "JSON received", which looks like the old FDM silence (docker-notes gotcha 31, 2026-11-08).
- Start project nodes after the sim is up, not in the same breath: one run started together never got ROS `/clock` (gotcha 33, one sample).
- `colcon build --symlink-install` copies `uav_planning`'s Python files, so rebuild after every edit (gotcha 32).
- `/ap/v1/*` **services** (arm_motors, mode_switch, takeoff) hang under the container's default `rmw_cyclonedds_cpp`; export `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in the client shell. Topics work under both.
- `pkill` leaves `arducopter`/`MicroXRCEAgent` behind; use `docker compose restart ros2` between sim runs.
- Headless launch: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false`.
- The WinBoat Windows VM must be stopped for all sim work (it holds about 3 GB, more than Chrome; gotcha 42). Before a sim launch check `docker ps --filter name=WinBoat`, and if it is running tell Raafay at once rather than measuring around it.

First planner flight (2026-11-08): `ros2 launch uav_planning follower.launch.py` against a running `iris_runway` arms, climbs to 10 m, flies to the best stub-map sector and reports ARRIVED; `planner_enable` false holds it.

Swarm (2026-11-12, T5): `ros2 launch uav_planning swarm.launch.py` brings up 3 iris in headless Gazebo plus 3 followers, which each hold a different stub peak 90-103 s after a cold launch (3 runs, RTF about 0.26-0.44). Upstream's per-drone bridge puts the clock on `/iris_N/clock`, so with namespaces nothing publishes `/clock`; the swarm launch bridges `/clock` once and starts followers only after `wait_for_clock` sees it flow. Each `/clock` subscriber costs about 45% of a core, so only the planners use sim time. Details in `docs/planner-decisions.md` (session 4 notes) and `docs/docker-notes.md` gotchas 35-37.

Sim speed (2026-11-16, T7): RTF is about 1/N (0.96 idle at 1 iris, 0.24 at 4) because Gazebo's main thread spends about 1 ms per drone per physics step; MAVProxy, cameras and RAM are not the limit. A 2 ms physics step plus `SCHED_LOOP_RATE 250` doubles it and is the `swarm.launch.py` default since D37 (`physics_step`, default 0.002; `physics_step:=0.001` for upstream timing). 3 followers then ARRIVED about 48 s after launch at hover RTF about 0.8. Gotchas 43-44; ROS `/clock` stayed silent in 2 of 4 cold starts at 2 ms (gotcha 33: restart and relaunch).

Ground-truth detector (2026-11-17, session 7): `uav_perception/ground_truth_detector` publishes `/detections` from Gazebo `target_*` models with miss 0.20, false alarm 0.05 per frame, 5 px noise (D38), on every frame including empty ones. Checked live with one hovering iris against `leader_node`; run the leader with `heatmap.launch.py stub:=false`. The sim camera is 640x480 with fx 205.47 at 10 Hz (docker-notes gotchas 47-50).

Sweep controller (2026-11-17, session 8): `ros2 launch uav_planning sweep.launch.py` (after the sim with `camera:=true`) sets `WP_SPD` 5 m/s, arms, climbs to the leader's 8 m layer and flies 3 north lanes over the 40 x 40 m arena (8x8 at 5 m, now also `leader_node`'s default, D39), then latches `/iris_1/sweep_status` COMPLETE, about 40 s of sweep. It never reads detections. Camera pointing held at 5 m/s (median 3 px, max 7) and one sweep left no unknown cells; probe `docs/probes/sweep_live_check.py`. A bare `y` launch value reaches a node as a boolean (gotcha 51), and `/ap/vN/set_parameters` saves to eeprom (gotcha 52).

Belief map and leader role (2026-11-18, session 9, D40): `uav_mapping/belief_map_node` is the single owner of `/heatmap`, fusing every drone's `/<robot>/detections` and `/<robot>/pose` (it computes footprints itself). Each drone's look at a cell counts at most +-0.85 log-odds (a new look after 1 s out of view or every 4 s, R29). A follower within 2.0 m of a sector centre that sees a detection there on 3 consecutive frames confirms it: the sector then reads 0 and is listed on latched `/confirmed_sectors` (D32, D13). `ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=1` starts iris_1 as the corner leader (sweep at 8 m), followers iris_2.. spawned 5 m south of the arena, one detector per drone and the belief map; in two cold runs the follower confirmed all 3 targets before the sweep ended (the Nov 20 loop). Each detector costs about 2 % RTF at 2 drones. Measure RTF from `/stats` time deltas, not its noisy `real_time_factor` field (docker-notes gotchas 54-56).

Four drones and the mission design (2026-11-20, session 10, D41): `swarm.launch.py leader:=true num_followers:=3` found all 3 targets in two cold runs, finds 15-34 s into a 68-76 s sweep, at RTF 0.54-0.61 hovering with 4 detectors. The mission state machine is designed (D41: `mission_node` in `uav_planning`, WAITING -> SEARCHING -> DONE/TIMED_OUT, planners launch disabled and the mission enables them, D14 end rule, 600 sim-s timeout, disable = D29 hold at the end) and its ROS-free core `uav_planning/core/mission.py` is built; the node is next. `belief_map_node` also publishes latched `/confirmed_finds` (JSON: cell, drone, wall time). After a reboot, `start` can fail on a renamed Wayland `XAUTHORITY` file; `up -d` recreates the container, and then setuptools must be checked with `/usr/bin/python3` (docker-notes 57-58; not needed for a container made from the image rebuilt 2026-11-23).

Mission end to end (2026-11-20, session 11, Nov 24 milestone met): `swarm.launch.py leader:=true num_followers:=3` now also starts `mission_node`, and every planner launches disabled; the mission enables them on entering SEARCHING and disables them (one hold each) at the end. One cold run ended DONE 31.3 s after the sweep's COMPLETE, and latched `/mission_report` listed all 3 finds with the right drones; `mission_timeout:=60` gave TIMED_OUT. `mission:=false` restores session 10. A second DONE run at another seed matched it. RTF during the search 0.49-0.52. The mission reads sim time from the followers' `/<robot>/pose` stamps (D41 amended), and the report's `sim_s` matched Gazebo's `/stats` to 0.08 s. A hold sent while cruising overshoots a few metres and comes back (planner-decisions, session 11 notes). Read latched topics with `--qos-durability transient_local`, run probes with `python3 -u` (docker-notes 59-60).

Literature check (2026-11-22, session 12, no sim): `docs/literature-check.md`. The narrowed claim is **novel only in the eval**: every link of the chain has prior work (closest: LMPath 2605.13782, Language as a Sensor 2606.08666, arXiv 2611.16524), and nobody tests a misleading language clue in multi-UAV search. Raafay accepted the reworded claim ("What a wrong language-derived search prior costs, measured") and a fourth eval arm, misleading clue with negative evidence off (`BeliefParams.p_miss = 0.5`); both are in the roadmap. Put no novelty claim in the README beyond the "now" sentence there.

Go/no-go dress run (2026-11-22, session 13): the mission half of the Nov 30 go/no-go is met (the video followed in session 14: GO). `swarm.launch.py leader:=true num_followers:=3` ran cold to DONE headless (142 s wall, 74.6 sim-s, RTF about 0.59) and with `gui:=true` (203 s wall, RTF 0.27-0.34). `arena_view:=true` is a video look (plain ground with the runway's collision plane, dark-green 5 m grid plate, top-down view with origin bottom-left, yellow leader and cyan follower propellers, halos, and `ground_marker_node` markers under each drone); judge sectors by the markers, because a perspective camera pushes high drones outward (docker-notes 62). After DONE each finder flies back to hover over its find (D41 amended, latched `/mission_return`, 3 finders within 0.1 m in 19 s); followers start with the sweep, not after it, as the language prior needs (D42). A four-drone run holds about 30 DDS participants against the RMW's default 32; `docker-compose.yml` now raises it, which only a recreate applies (gotcha 61, then check setuptools, gotcha 58, unless the container comes from the image rebuilt 2026-11-23). The ROS-based tests need no sim up, or `ROS_DOMAIN_ID=77`. Open for the user: halos or markers only, and whether landing is scheduled (it is in no milestone). Details in `docs/planner-decisions.md`, session 13 notes.

CI (2026-11-23, session 15; Dec 8 row met): `.github/workflows/ci.yml` builds the workspace and runs the unit tests on a hosted runner in the stock `ros:jazzy` image (223 pass, 2 skip, 80 s). The 25 GB image cannot go on a hosted runner, so `smoke.yml` runs the mission on a self-hosted runner on the user's machine (`~/actions-runner`, label `uav-smoke`, started with `./run.sh`; manual dispatch only, with `gh workflow run smoke.yml`, never pull requests); it calls `ci/in_container.sh` (build, tests, `ci/smoke_test.sh`). The arena starts empty, so `ci/spawn_targets.sh` creates the targets and the checker needs a find. Green on the runner: DONE with 3 finds (run 45134386843, about 6.5 min); by hand 97 s in the dev container and 317 s from a clean checkout. The runner must be online, the USB mounted and no sim running, or the job queues or fails fast. Docker-notes 64-66.

Language prior library (2026-11-24, session 16, no sim): `uav_mapping/core/prior.py` and `landmarks.py` turn a clue's strict-JSON regions into a starting belief for `BeliefMap(start=...)`: area overlap (D44), raise only at alpha 0.5 so a uniform prior is exactly today's map (D45), flat region list with maximum on overlap (D46), misleading = every landmark swapped with a one-cell gap (D47), replay exact or uniform (D48, error in the eval). `worlds/runway/landmarks.yaml` is invented places; editing it orphans `recordings.json` on purpose, and hand-written recordings are never eval input. 127 tests at the time; wired in session 17, below. The probe (`docs/probes/prior_probe.py`) says a clue barely matters on the 40 m arena and alpha hardly matters anywhere; its numbers are not results.

Clue-driven missions (2026-11-24, session 17, Dec 10 row met, D49-D50): `swarm.launch.py leader:=true num_followers:=3 clue:="One hiker, last seen near the hut."` starts `mission_prior_node`, which publishes one latched `/mission_prior` (replay by default, so no key; `prior_provider:=live`, `llm_model`, `record_prior` for a live call that is saved as a recording), and `belief_map_node` drops frames and publishes no `/heatmap` until it arrives (60 wall-s, then uniform with a warning); no clue = today's map built at once. One cold headless run each of no clue, the hut clue and the radio mast clue (misleading) all ended DONE with 3 finds (74.7, 74.0, 72.8 sim-s); the clue changed the first goals (iris_2 went to the hut sector, or to the empty mast corner), and one run each says nothing about time. The three flights used hand-written recordings (hut and mast are live answers since session 18). Leader lane order moved to the eval sim (D50). Probe: `docs/probes/prior_live_check.py`. One of four cold starts had no ROS `/clock` (gotcha 33). 382 tests.

2D eval and the first model calls (2026-11-25, session 18, no sim, D51-D54): `core/live.py` picks the vendor and its one key from the model's name (`gemini-*` `GEMINI_API_KEY`, `claude-*` `ANTHROPIC_API_KEY`); keys live in the gitignored `docker/llm.env`, never read or printed, and go to one command with `docker exec --env-file docker/llm.env multi-agent-uavs ...` (docker-notes 67). The model is gemini-3.8-flash. `src/uav_eval` is the 2D kinematic eval (no ROS): `python3 -m uav_eval.run clues|record|run` with `PYTHONPATH` of the four packages, or `ros2 run uav_eval run_eval`; world `worlds/valley` (120 x 120 m), four arms (uniform, accurate, misleading, fixed = negative evidence off), live recordings of one model only. Final seeds are 0-19 and have not been asked or run; pilots use 1000 and up. The leader's sweep does not follow the clue (D53, reverses the roadmap: lanes first slowed both kinds of clue). An 8-seed pilot runs in 10 s and its numbers are not results. 482 tests.

Multi-drone and actuation facts (verified 2026-11-04, see `docs/planner-decisions.md`): `multiagent.launch.py` hard-codes 4 different vehicles, so use our own launch (`uav_planning/launch/swarm.launch.py`). Topics are `/iris_N/*` (Gazebo) and `/ap/vN/*` (autopilot, N = sysid). `/ap/vN/goal_lla` is autopilot output, not a command. The command is `/ap/vN/cmd_gps_pose` with `frame_id "map"` and `FRAME_GLOBAL_REL_ALT`; any other `frame_id` is dropped silently, as is anything sent when not GUIDED and armed. The live iris camera is on a gimbal, 52 deg off nadir with upstream params; `uav_planning/config/gimbal_nadir.parm` points it straight down (T6, 2026-11-16). `swarm.launch.py` loads it; for `iris_runway` override `defaults` and keep a `dds_*.parm` file last (docker-notes gotchas 39-40).

`build/`, `install/`, `log/` are gitignored (colcon outputs).

## Testing

Run in the container (`source /usr/local/bin/entrypoint.sh` first in a non-interactive shell): `python3 -m pytest src` from the repo root. pytest 7.4.4; each package has a `conftest.py` so tests import without installing. `uav_planning/core` needs no ROS; the datum tests and one planner round-trip test need ROS sourced.

<!-- gstack:verify: docker compose -f docker/docker-compose.yml exec -T ros2 bash -lc 'source /usr/local/bin/entrypoint.sh >/dev/null 2>&1; cd /workspaces/multi-agent-uavs && source install/setup.bash && python3 -m pytest src -q' -->
The line above is what `/verify` runs: the project's tests inside the ros2 container, which must be up (the host has neither rclpy nor pytest).

`/verify`'s per-file stage runs through the container too: `.gstack/pregate.json` (lanes for `.github`, `ci`, `docker`, `skills`, the swarm launch file, `worlds/**` and `uav_mapping/package.xml`; touching an unmapped file such as a `.parm` or `package.xml` gives PREGATE_NO_LANE, so map it) and `src/uav_planning/test/test_ci_files.py`.

Skill source-of-truth is `skills/multi-agent-uavs/SKILL.md`. `.claude/skills/multi-agent-uavs` is a symlink to that directory so Claude Code picks it up automatically when run in this project. `skills/multi-agent-uavs.skill` is the packaged ZIP bundle for installing the same skill into Cowork — **after editing `SKILL.md`, run `skills/pack.sh` before committing** so the bundle stays in sync with the source (Claude Code reads the source directly, but anyone installing the skill from this repo's `.skill` file gets whatever the bundle contains, so a stale bundle silently ships an old skill). To make local Cowork sessions pick up edits to `SKILL.md` without re-installing on every change, replace the Cowork-installed skill directory with a symlink to this repo's `skills/multi-agent-uavs/` (one-time setup — see `docs/docker-notes.md` adjacent or the project README).

## How to work with Claude on this project

- **Read `docs/roadmap.md` for the current plan** (architecture, milestones, eval, open questions) and `docs/next-session-prompt.md` for where to pick up. Career/application planning is intentionally NOT in this repo; if `~/Documents/robotics-action-plan.md` exists locally it is private context, never to be copied into committed files.
- **Load the `multi-agent-uavs` skill** at session start. It contains the full stack, node architecture, algorithm recap, command patterns, and debugging recipes. The skill is installed via `skills/multi-agent-uavs.skill` in this repo if it ever needs reinstalling.
- **Default to ROS 2 / Jazzy** for any installation or code question. The Noetic-era references in `docs/legacy-implementation-guide.docx` are historical context, not instructions to follow.
- **Don't recommend MAVROS, MAVProxy in the runtime, or iq_sim.** Use AP_DDS and `ardupilot_gz` instead — see assessment §5.
- **Check `docs/assessment-ros2-migration.md` first** when asked *why* a design choice was made. It has dedicated sections for ROS 2 setup, Gazebo, YOLO, drone harness, simulation, and enhancements.
- For boilerplate (CMakeLists, package.xml, launch files, SDF, Dockerfiles) — write it for Raafay and explain briefly. For architecture choices (which nodes exist, how they communicate) — pull him in, that's the part he's learning.
- When introducing a new ROS 2 concept (QoS, lifecycle nodes, executors), give a one-paragraph "what it is and when you care" before showing code.

## Reference repos (canonical)

- ArduPilot: <https://github.com/ArduPilot/ardupilot>
- ardupilot_gz: <https://github.com/ArduPilot/ardupilot_gz>
- pymavlink: <https://github.com/ArduPilot/pymavlink>
- MAVProxy (debugging only): <https://github.com/ArduPilot/MAVProxy>
- ros_gz bridge: <https://github.com/gazebosim/ros_gz>
- yolo_ros wrapper: <https://github.com/mgonzs13/yolo_ros>

## Reference docs (canonical)

- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- Gazebo Harmonic: <https://gazebosim.org/docs/harmonic>
- ArduPilot ROS 2: <https://ardupilot.org/dev/docs/ros2.html>
- ArduPilot ROS 2 + Gazebo: <https://ardupilot.org/dev/docs/ros2-gazebo.html>
- AP_DDS README: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md>
- ArduPilot guided-mode commands: <https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html>
- MAVLink common messages: <https://mavlink.io/en/messages/common.html>
- YOLO26 docs: <https://docs.ultralytics.com/models/yolo26>

## Skill routing

When the user's request matches an available skill, invoke it via the Skill tool. When in doubt, invoke the skill.

Key routing rules:
- Product ideas/brainstorming → invoke /office-hours
- Strategy/scope → invoke /plan-ceo-review
- Architecture → invoke /plan-eng-review
- Design system/plan review → invoke /design-consultation or /plan-design-review
- Full review pipeline → invoke /autoplan
- Bugs/errors → invoke /investigate
- QA/testing site behavior → invoke /qa or /qa-only
- Code review/diff check → invoke /review
- Visual polish → invoke /design-review
- Ship/deploy/PR → invoke /ship or /land-and-deploy
- Save progress → invoke /context-save
- Resume context → invoke /context-restore
- Author a backlog-ready spec/issue → invoke /spec
