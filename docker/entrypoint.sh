#!/usr/bin/env bash
# docker/entrypoint.sh
#
# Sources every ROS 2 overlay in the right order, then execs whatever
# command the container was started with. This file is used in TWO ways:
#
#   1. As the container ENTRYPOINT (via `docker run` / `compose up`).
#      In that case we want `set -e` and a final `exec "$@"`.
#   2. Sourced from ~/.bashrc on every interactive shell, so `docker
#      compose exec ros2 bash` gives you a fully-sourced env.
#      In that case `set -e` and `exec` are wrong - we just want to
#      arrange the environment and return.
#
# We detect "am I being sourced?" with the classic bash trick:
#   `(return 0 2>/dev/null)` succeeds inside a sourced file, fails inside
#   one that was executed.
#
# Overlay order matters:
#   1. /opt/ros/jazzy/setup.bash         - ROS 2 core (underlay)
#   2. /opt/ardupilot_ws/install/...     - ardupilot_gz overlay (baked in)
#   3. /workspaces/.../install/setup.bash - your project overlay, if built
# -------------------------------------------------------------------------

# Detect sourced-vs-executed
(return 0 2>/dev/null) && _SOURCED=1 || _SOURCED=0

if [ "${_SOURCED}" = "0" ]; then
    set -e
fi

# 1. ROS 2 core
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

# 2. ardupilot_gz overlay (built into the image)
if [ -f "/opt/ardupilot_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/opt/ardupilot_ws/install/setup.bash"
fi

# 3. Project workspace overlay (only exists after `colcon build`)
if [ -f "/workspaces/multi-agent-uavs/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/workspaces/multi-agent-uavs/install/setup.bash"
fi

# 4. ArduPilot tooling on PATH (sim_vehicle.py, arducopter binary, MAVProxy)
export PATH="/opt/ardupilot/Tools/autotest:/opt/ardupilot/build/sitl/bin:${PATH}"

# Tell Gazebo and sdformat where ardupilot_gazebo + ardupilot_gz_description
# models/worlds live. The launch file copies GZ_SIM_RESOURCE_PATH to SDF_PATH
# so sdformat_urdf can resolve mesh/include paths during URDF conversion -
# critical because sdformat_urdf on Jazzy doesn't reliably resolve
# `package://` URIs via ament_index, and robot_state_publisher dies if it
# can't find iris_with_standoffs and friends.
export GZ_SIM_RESOURCE_PATH="/opt/ardupilot_ws/install/ardupilot_gazebo/share:/opt/ardupilot_ws/install/ardupilot_gz_description/share:/opt/ardupilot_ws/src/SITL_Models/Gazebo/models:/opt/ardupilot_ws/src/SITL_Models/Gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ardupilot_ws/install/ardupilot_gazebo/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export SDF_PATH="${GZ_SIM_RESOURCE_PATH}:${SDF_PATH:-}"

if [ "${_SOURCED}" = "0" ]; then
    exec "$@"
fi
