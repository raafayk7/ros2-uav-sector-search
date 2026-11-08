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
- **Algorithm:** each follower independently flies to the best open sector on the shared map, using the thesis's overlap penalty; thesis Distributed Gradient Ascent (DGA) and Bilinear Log-Linear (BLL) are library baselines (changed 2026-11-04 on measured evidence, see `docs/planner-decisions.md`)

Things explicitly NOT in the rebuild even though the original used them: MAVROS as primary bridge, MAVProxy in the runtime path, iq_sim launch files, Gazebo Classic 11, hand-rolled UDP broadcast, Ubuntu 20.04, central-planner hack ("Drone 1 computes for Drone 2"). See the assessment doc §5 and the "do not use" list in the `multi-agent-uavs` skill for the replacements.

## Layout

```
multi-agent-uavs/
├── docs/                       # thesis.pdf, legacy-implementation-guide.docx, assessment-ros2-migration.md, docker-notes.md
├── src/                        # colcon workspace src/ — ROS 2 packages live here
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

First planner flight (2026-11-08): `ros2 launch uav_planning follower.launch.py` against a running `iris_runway` arms, climbs to 10 m, flies to the best stub-map sector and reports ARRIVED; `planner_enable` false holds it.

Multi-drone and actuation facts (verified 2026-11-04, see `docs/planner-decisions.md`): `multiagent.launch.py` hard-codes 4 different vehicles, so use our own launch. Topics are `/iris_N/*` (Gazebo) and `/ap/vN/*` (autopilot, N = sysid). `/ap/vN/goal_lla` is autopilot output, not a command. The command is `/ap/vN/cmd_gps_pose` with `frame_id "map"` and `FRAME_GLOBAL_REL_ALT`; any other `frame_id` is dropped silently, as is anything sent when not GUIDED and armed. The live iris camera is on a gimbal, 52 deg off nadir.

`build/`, `install/`, `log/` are gitignored (colcon outputs).

## Testing

Run in the container (`source /usr/local/bin/entrypoint.sh` first in a non-interactive shell): `python3 -m pytest src` from the repo root. pytest 7.4.4; each package has a `conftest.py` so tests import without installing. `uav_planning/core` needs no ROS; the datum tests and one planner round-trip test need ROS sourced.

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
