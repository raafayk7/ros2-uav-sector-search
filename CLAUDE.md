# CLAUDE.md — Project context for Claude

This file is loaded automatically by Claude Code / Cowork when working in this repo. It's the canonical short-form context. For the full reasoning behind everything below, read `docs/assessment-ros2-migration.md`.

## What this project is

A ROS 2 rebuild of Raafay's LUMS senior project — a multi-agent UAV search-and-rescue system. A leader drone sweeps an area, runs YOLO to detect ground markers, builds a probability heatmap; follower drones then run a distributed gradient ascent algorithm to converge on the marked sectors, then precision-land via color masking + PID.

The original implementation (Spring 2025) was ROS Noetic + Gazebo 11 + ArduPilot + MAVROS + YOLOv8. This rebuild targets a modern stack and aims to close out the future-work items the original team identified (true decentralization, GPS-denied follower phase, scale beyond 2 followers, more robust perception).

## User profile

Raafay graduated LUMS EE in 2025. Has high-level ROS understanding rather than deep technical fluency — explanations should lead with the *why* of each design choice so the rebuild doubles as a learning exercise. Comfortable with Python and Linux. Wants to leverage AI heavily for boilerplate so he can focus on architecture and the algorithm.

Primary machine: Debian Trixie.

## The chosen stack (locked in — don't relitigate)

- **Host:** Debian Trixie
- **Containment:** Docker
- **ROS 2:** Jazzy Jalisco (LTS through May 2029)
- **Simulator:** Gazebo Harmonic
- **Drone harness:** `ArduPilot/ardupilot_gz` with `multiagent.launch.py`
- **ROS ↔ ArduPilot:** AP_DDS (native, ArduPilot 4.5+) — not MAVROS
- **Perception:** Ultralytics YOLO26
- **Algorithm:** Distributed Gradient Ascent (DGA), with Bilinear Log-Linear (BLL) as a comparison baseline

Things explicitly NOT in the rebuild even though the original used them: MAVROS as primary bridge, MAVProxy in the runtime path, iq_sim launch files, Gazebo Classic 11, hand-rolled UDP broadcast, Ubuntu 20.04, central-planner hack ("Drone 1 computes for Drone 2"). See the assessment doc §5 and the "do not use" list in the `multi-agent-uavs` skill for the replacements.

## Layout

```
multi-agent-uavs/
├── docs/                       # thesis.pdf, legacy-implementation-guide.docx, assessment-ros2-migration.md
├── src/                        # colcon workspace src/ — ROS 2 packages live here
├── docker/                     # Dockerfile + compose
├── skills/                     # canonical multi-agent-uavs skill source + packaged .skill
├── .claude/skills/             # Claude Code skill discovery — symlinks into skills/
├── CLAUDE.md
└── README.md
```

`build/`, `install/`, `log/` are gitignored (colcon outputs).

Skill source-of-truth is `skills/multi-agent-uavs/SKILL.md`. `.claude/skills/multi-agent-uavs` is a symlink to that directory so Claude Code picks it up automatically when run in this project. `skills/multi-agent-uavs.skill` is the packaged bundle for installing the same skill into Cowork. Edits to the SKILL.md propagate to both tools without duplication.

## How to work with Claude on this project

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
