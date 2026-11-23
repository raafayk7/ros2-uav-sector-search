#!/usr/bin/env bash
# Build, unit-test and smoke-test the workspace inside the ros2 image.
# Run from the repo root inside the container (the self-hosted workflow and
# `docker compose exec` both call it):
#   bash -l ci/in_container.sh
# No -u: ROS's setup scripts read variables they never define.
set -e

source /usr/local/bin/entrypoint.sh
cd /workspaces/multi-agent-uavs

colcon build --symlink-install
source install/setup.bash

# No sim is up yet, so the ROS-based tests get the default domain to themselves.
python3 -m pytest src -q -rsf

ci/smoke_test.sh
