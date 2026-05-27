# Multi-Agent UAV Search & Rescue — ROS 2 Rebuild

A ROS 2 / Gazebo Harmonic reimplementation of the LUMS senior project *"Distributed Autonomous Surveillance and Rescue Architecture Integrating Computer Vision within Multi-Agent Drone Coordination"* (Anwer, Moeez, Tahir, Kazmi — Spring 2025). The original was built on ROS Noetic + Gazebo 11; this rebuild targets a modern ROS 2 stack and aims to close out the future-work items the original team identified.

## Stack

- **Host OS:** Debian Trixie
- **Containment:** Docker (so the host stays clean)
- **ROS 2:** Jazzy Jalisco (LTS through May 2029)
- **Simulator:** Gazebo Harmonic
- **Drone harness:** [`ArduPilot/ardupilot_gz`](https://github.com/ArduPilot/ardupilot_gz) with `multiagent.launch.py`
- **ROS ↔ ArduPilot:** AP_DDS (native, ArduPilot 4.5+) — not MAVROS
- **Perception:** Ultralytics YOLO26 (Jan 2026 release)
- **Algorithm:** Distributed Gradient Ascent (DGA), decomposed into per-drone ROS 2 nodes

See [`docs/assessment-ros2-migration.md`](docs/assessment-ros2-migration.md) for the full rationale.

## Layout

```
multi-agent-uavs/
├── docs/                       # thesis, legacy guide, assessment, design notes
├── src/                        # ROS 2 packages (the colcon workspace src/)
├── docker/                     # Dockerfile + compose for the dev container
├── skills/                     # canonical source of the multi-agent-uavs Claude skill
│   ├── multi-agent-uavs/       # the skill (SKILL.md + future references/scripts)
│   └── multi-agent-uavs.skill  # packaged for Cowork's "Save skill" install
├── .claude/skills/             # Claude Code project-scoped skills
│   └── multi-agent-uavs → ../../skills/multi-agent-uavs  (symlink)
├── CLAUDE.md
├── .gitignore
└── README.md
```

`build/`, `install/`, and `log/` are produced by `colcon build` inside the container and are gitignored.

## Status

Initial setup. No code committed yet — the workspace layout is in place and the design decisions are documented in `docs/`. Next steps are in §8 of the assessment.

## Source documents

- [`docs/thesis.pdf`](docs/thesis.pdf) — the original senior project thesis (49 pages)
- [`docs/legacy-implementation-guide.docx`](docs/legacy-implementation-guide.docx) — original team's command cheat sheet for the Noetic setup
- [`docs/assessment-ros2-migration.md`](docs/assessment-ros2-migration.md) — the decision document for this rebuild

## Authors

ROS 2 rebuild by Raafay Saeed Kazmi (one of the original four authors). Original senior project advised by Dr. Hassan Jaleel and Dr. Murtaza Taj, LUMS EE.
