#!/usr/bin/env bash
# Build, unit-test and smoke-test the workspace inside the ros2 image.
# Run from the repo root inside the container (the self-hosted workflow and
# `docker compose exec` both call it):
#   bash -l ci/in_container.sh
set -eu

source /usr/local/bin/entrypoint.sh
cd /workspaces/multi-agent-uavs

# A container made from the image can hold a half-removed setuptools in
# /usr/local (78.1.0 files with an 84.2.1 record), which breaks
# `colcon build --symlink-install` with "option --uninstall not recognized"
# (docker-notes 58). Uninstalling until apt's 68.1.2 shows through is the
# documented fix; it is a no-op once that is true.
for _ in 1 2 3; do
  [ "$(/usr/bin/python3 -c 'import setuptools; print(setuptools.__version__)')" = "68.1.2" ] && break
  sudo /usr/bin/pip3 uninstall -y --break-system-packages setuptools >/dev/null 2>&1 || true
done

colcon build --symlink-install
source install/setup.bash

# No sim is up yet, so the ROS-based tests get the default domain to themselves.
python3 -m pytest src -q -rsf

ci/smoke_test.sh
