# Replacement text for Cowork project instructions

Paste this into the Cowork project settings → Instructions field for "multi-agent-uavs". It replaces the original instructions (which were from before we decided on the stack).

---

I'm an EE grad (LUMS, 2025) rebuilding my senior project — a multi-agent UAV search-and-rescue system — on a modern ROS 2 stack. Primary machine is Debian Trixie. I have a high-level ROS understanding rather than a deep technical one, so explain the *why* of design choices when it'd help.

**Stack is decided — don't relitigate:** ROS 2 Jazzy + Gazebo Harmonic + Docker on Debian Trixie + `ArduPilot/ardupilot_gz` with `multiagent.launch.py` + AP_DDS (not MAVROS) + YOLO26. Distributed Gradient Ascent algorithm from the original thesis. Full rationale in `docs/assessment-ros2-migration.md`; short version in `CLAUDE.md` at the project root.

**Use the `multi-agent-uavs` skill** for project-specific context (node architecture, algorithm details, command patterns, debugging recipes). It's installed via `skills/multi-agent-uavs.skill` in the repo. Load it whenever working in this project.

**Workspace layout:** `docs/` for the thesis + legacy guide + assessment, `src/` is the colcon workspace src/ for ROS 2 packages, `docker/` for the dev container, `skills/` for the project skill. `build/`, `install/`, `log/` are colcon outputs and gitignored.

**Things superseded — don't pull them in by default:** MAVROS as primary bridge (use AP_DDS), MAVProxy in the runtime (only for debugging), iq_sim launch files (use ardupilot_gz), Gazebo Classic 11 (EOL).

**Useful repos:** [ardupilot_gz](https://github.com/ArduPilot/ardupilot_gz), [pymavlink](https://github.com/ArduPilot/pymavlink/), [MAVProxy](https://github.com/ArduPilot/MAVProxy/), [ros_gz](https://github.com/gazebosim/ros_gz), [yolo_ros](https://github.com/mgonzs13/yolo_ros).

**Useful docs:** [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/), [Gazebo Harmonic](https://gazebosim.org/docs/harmonic), [ArduPilot ROS 2](https://ardupilot.org/dev/docs/ros2.html), [AP_DDS README](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md), [ArduPilot guided-mode commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html), [MAVLink common messages](https://mavlink.io/en/messages/common.html), [YOLO26](https://docs.ultralytics.com/models/yolo26).

I want to leverage AI heavily for boilerplate (launch files, SDF, CMake, Dockerfiles) and use my own time on architecture and the algorithm.
