# syntax=docker/dockerfile:1.7
#
# multi-agent-uavs dev container
# -------------------------------------------------------------------------
# Layers, in order:
#   1. ROS 2 Jazzy desktop-full on Ubuntu 24.04 Noble
#   2. Gazebo Harmonic + Micro XRCE-DDS Agent + dev tooling
#   3. ArduPilot SITL (waf build, baked in) + ardupilot_gz colcon workspace
#   4. Ultralytics YOLO + Python deps
#   5. Non-root user, entrypoint that sources every overlay
#
# Image size lands around 8-10 GB. That's expected for ROS 2 desktop-full +
# Gazebo + a SITL build. We trade size for "everything works out of the box."
#
# Build:   docker compose build
# Run:     docker compose up -d && docker compose exec ros2 bash
# -------------------------------------------------------------------------

# osrf/ros:jazzy-desktop-full = Ubuntu 24.04 Noble + ROS 2 Jazzy + RViz + demos.
# 'desktop-full' (vs 'ros-base') gives us RViz, rqt, image_view, etc. - all the
# visualization we'll lean on while learning. The extra ~1 GB is worth it.
FROM osrf/ros:jazzy-desktop-full

# -------------------------------------------------------------------------
# Build args - override at build time with `--build-arg KEY=VALUE` or via
# compose.yml. UID/GID match your host user so files written from the
# container don't end up root-owned on the bind mount.
# -------------------------------------------------------------------------
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
# ArduPilot ref - pin to a master SHA, not a Copter-* release tag.
# Why a master SHA and not a release tag: ardupilot_gz's ros2_gz.repos
# pins `ardupilot: master`, and the launch files in ardupilot_gz_bringup
# (iris.launch.py et al) reference `dds_use_ns.parm` which was added to
# ardupilot in commit b17c3db3dd on 2026-03-24 - AFTER all Copter stable
# tags (4.6.3 is the latest stable and doesn't have it). So a release tag
# can't carry the file at all; only master can.
#
# Pinning to a SHA instead of `master` keeps the image reproducible across
# rebuilds. Bump this SHA when you want newer SITL behaviour.
#
# SHA below = master HEAD as of 2026-05-29, short hash ddba3b4
# ("Tools: fix setup on docker"). Use the full 40-char SHA - GitHub's
# upload-pack rejects short hashes as "couldn't find remote ref".
# Replaces previous pin: Copter-4.5.7 (which never connected to
# ArduPilotPlugin in Gazebo - FDM protocol drift + missing dds_use_ns.parm,
# see docs/docker-notes.md "Known limitation" for the diagnosis).
ARG ARDUPILOT_REF=ddba3b490eaec04dafa3a12b45b7527aef0d0341

# ardupilot_gz: pin to a specific main SHA for reproducibility.
# SHA below = main HEAD as of 2026-04-11, short hash 8df4dc1 ("Add
# namespace as launch parameter"). Co-evolves with the ardupilot SHA
# above - both need to be from the same era for the FDM handshake and
# the dds_use_ns.parm path to line up. Bump in tandem with ARDUPILOT_REF.
ARG ARDUPILOT_GZ_REF=8df4dc1726e37504e6fc8b952d02e554cfa3176f

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    # CycloneDDS is the recommended RMW for ROS 2 + AP_DDS. The default rmw
    # (Fast DDS) works too, but CycloneDDS is what most ArduPilot tutorials
    # assume, and it has more predictable multicast behaviour on Linux.
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# -------------------------------------------------------------------------
# System packages: ROS 2 Gazebo bridge, Micro XRCE-DDS Agent, common deps.
# -------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        # General dev tooling
        sudo curl wget git ca-certificates gnupg lsb-release \
        vim nano less tmux htop \
        build-essential cmake pkg-config \
        # Python tooling for colcon + rosdep + vcstool
        python3-pip python3-venv \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-rosdep python3-vcstool \
        # ros_gz brings in Gazebo Harmonic (the official pairing for Jazzy)
        # plus the ROS<->Gazebo bridge nodes. Harmonic itself is pulled in
        # transitively as a dep - no separate gz-harmonic package needed.
        ros-jazzy-ros-gz \
        # CycloneDDS RMW implementation
        ros-jazzy-rmw-cyclonedds-cpp \
        # Useful ROS 2 packages for this project
        ros-jazzy-vision-msgs \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-image-pipeline \
        ros-jazzy-tf-transformations \
        # X11 + Mesa for Gazebo rendering on Intel/AMD iGPU
        libgl1-mesa-dri mesa-utils x11-apps libglu1-mesa \
        # OpenCV runtime (cv2 in Python)
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------------------------
# Micro XRCE-DDS Agent (built from source).
#
# This is the daemon that proxies between ArduPilot's eProsima Micro
# XRCE-DDS client (running inside SITL) and the rest of the ROS 2 graph.
# Without it, /ap/* topics never appear on the ROS 2 side.
#
# It's NOT shipped as a `ros-jazzy-*` debian package - the eProsima upstream
# only publishes source + a snap. We build a pinned tag so the image stays
# reproducible. v3.0.1 is the current stable as of 2026-05 and is
# compatible with ArduPilot 4.5's Micro-XRCE-DDS-Client.
#
# UAGENT_SUPERBUILD=ON (the default) tells CMake to fetch and build the
# Fast-DDS/Fast-CDR deps inside its own build tree, so the install doesn't
# clash with ROS 2's own Fast-DDS.
# -------------------------------------------------------------------------
ARG MICRO_XRCE_AGENT_REF=v3.0.1
RUN git clone --depth 1 --branch ${MICRO_XRCE_AGENT_REF} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/microxrce-agent \
    && cd /tmp/microxrce-agent \
    && mkdir build && cd build \
    && cmake -DUAGENT_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/microxrce-agent

# -------------------------------------------------------------------------
# Non-root user. Doing this BEFORE the ArduPilot build means the build runs
# as root and lives in /opt - read-only for the user at runtime, which is
# what we want for a baked dependency.
#
# Note: Ubuntu 24.04 (Noble) base images ship with a pre-existing 'ubuntu'
# user already at UID 1000, so we delete it first before creating 'ros' at
# the same UID. The `|| true` guards make this idempotent if the base image
# ever stops shipping that user.
# -------------------------------------------------------------------------
RUN userdel -r ubuntu 2>/dev/null || true \
    && groupdel ubuntu 2>/dev/null || true \
    && groupadd --gid ${GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${UID} --gid ${GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# -------------------------------------------------------------------------
# ArduPilot SITL build (baked in).
# Lives at /opt/ardupilot. We build the 'copter' target with the 'sitl' board
# - that produces a SITL binary at /opt/ardupilot/build/sitl/bin/arducopter.
# sim_vehicle.py (the convenience launcher) is at Tools/autotest/.
#
# The clone is shallow per-branch but recurses submodules (ChibiOS, etc.) -
# the submodules are unavoidable; waf will fail without them.
# -------------------------------------------------------------------------
# Clone master shallowly, then check out the pinned SHA. Why not
# `git fetch --depth 1 origin <SHA>` directly: GitHub serves a
# precomputed pack for branch heads (fast, reliable), but has to build a
# custom pack for arbitrary-SHA requests against repos this size, and
# negotiation reliably times out on slower connections (we hit a 30-min
# RPC timeout on 2026-05-30 trying that path). Branch-then-checkout uses
# the fast path.
#
# --shallow-since (not --depth) keeps the pinned SHA in range however far master moves on; bump the date with the SHA.
# within the last few weeks of master stays reachable. If you bump
# ARDUPILOT_REF to something older and the checkout fails with "no such
# object", increase the depth or switch the clone to `--branch <SHA>`
# (only works if <SHA> resolves to a ref name) / `--unshallow`.
#
# --shallow-submodules keeps the submodule history shallow too (ChibiOS,
# mavlink, etc. are large). `git submodule update` re-runs after checkout
# to re-align the submodules with the pinned SHA's pointers.
RUN git clone --shallow-since=2026-05-28 --branch master --shallow-submodules --recurse-submodules \
        https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && git checkout ${ARDUPILOT_REF} \
    && git submodule update --init --recursive --depth 1

# install-prereqs-ubuntu.sh refuses to run as root (it edits ~/.bashrc / ~/.profile
# and the upstream maintainers don't want root-owned dotfiles). So we chown
# the tree to ros and run the build as that user. ros has NOPASSWD sudo
# (set up above), so apt-install steps inside the script still work.
RUN chown -R ${UID}:${GID} /opt/ardupilot

USER ${USERNAME}
WORKDIR /opt/ardupilot

# Patch around an upstream bug in install-prereqs-ubuntu.sh:
# for any codename != "mantic" it appends `python-argparse` to SITL_PKGS,
# but that's a Python-2-era package that no longer exists on Noble
# (argparse has been in the Python 3 stdlib for years - the line is dead
# code). We replace the token with nothing instead of deleting the line,
# because the line lives inside an `if ... fi` whose body must be non-empty.
RUN sed -i 's/python-argparse//g' \
        Tools/environment_install/install-prereqs-ubuntu.sh

# ArduPilot's bundled installer pulls every dep ArduPilot needs (cmake,
# python tooling, MAVProxy, etc). Running as ros so its .bashrc edits land
# in /home/ros instead of /root.
#   -y   - non-interactive (assume yes)
# Note: this script only accepts -y / -q (its getopts is just `yq`). Other
# flags you'll see in older blog posts (e.g. --no-gcs) don't exist on
# current ArduPilot - MAVProxy now installs by default, which is fine.
#
# Env vars we set:
#   USER, HOME                      - Docker's `USER` directive only sets the
#                                     kernel uid; env vars stay unset, and
#                                     install-prereqs runs `usermod -a -G
#                                     dialout $USER` which crashes if empty.
#   PIP_BREAK_SYSTEM_PACKAGES=1     - Noble's Python ships an
#                                     EXTERNALLY-MANAGED marker (PEP 668) so
#                                     pip refuses to install system-wide.
#                                     install-prereqs predates that change.
#                                     This env var is pip's officially
#                                     sanctioned override.
RUN USER=${USERNAME} HOME=/home/${USERNAME} \
        PIP_BREAK_SYSTEM_PACKAGES=1 \
        Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Configure for SITL + build copter. Takes ~10-15 min on a modern laptop.
# waf uses all available cores automatically.
RUN ./waf configure --board sitl \
    && ./waf copter \
    && ./waf clean

# Back to root for system-level steps (apt, rosdep, colcon install, pip).
USER root
WORKDIR /

# -------------------------------------------------------------------------
# ardupilot_gz colcon workspace (also baked in).
# Lives at /opt/ardupilot_ws. We use vcs to pull the companion repos
# (ardupilot_gazebo plugin, SITL_Models, etc.) per the official .repos file,
# then colcon-build the whole overlay.
# -------------------------------------------------------------------------
RUN mkdir -p /opt/ardupilot_ws/src
WORKDIR /opt/ardupilot_ws/src

# Same clone-branch-then-checkout pattern as for ardupilot above. See
# that block for the rationale on why we don't `git fetch <SHA>` directly.
RUN git clone --shallow-since=2026-04-10 --branch main --shallow-submodules --recurse-submodules \
        https://github.com/ArduPilot/ardupilot_gz.git /opt/ardupilot_ws/src/ardupilot_gz \
    && cd /opt/ardupilot_ws/src/ardupilot_gz \
    && git checkout ${ARDUPILOT_GZ_REF} \
    && git submodule update --init --recursive --depth 1

# ros2_gz.repos lists ardupilot itself among the companion repos, which
# would re-clone the entire ~1 GB tree we already baked in at /opt/ardupilot.
# Symlinking the existing tree into the workspace + passing --skip-existing
# to vcs makes vcs respect the directory that's already there and only
# fetch the smaller companion repos (ardupilot_gazebo, SITL_Models).
# This turns a 30+ min step into ~1 min.
RUN ln -s /opt/ardupilot /opt/ardupilot_ws/src/ardupilot

# vcs import pulls the companion repos listed in ros2_gz.repos.
# --skip-existing: don't touch repos whose directory already exists (the
#                  ardupilot symlink we just created counts).
RUN vcs import --recursive --skip-existing --input ardupilot_gz/ros2_gz.repos . \
    || echo "vcs import returned non-zero; check above for missing repos"

# Tell colcon to skip building source copies of packages we have correct
# system versions of. A COLCON_IGNORE file hides a directory from colcon's
# package discovery, so it resolves the dependency via rosdep against the
# system install instead.
#
# Why this matters: ros2_gz.repos pulls source trees of several upstream
# packages that are pinned to older Gazebo / sdformat versions. On Jazzy +
# Harmonic those don't compile (they reference ignition-math6, sdformat12,
# etc). The matching system packages installed via apt (ros-jazzy-ros-gz,
# ros-jazzy-sdformat-urdf, the sdformat14 that ships with Harmonic) are
# built correctly and resolve all rosdep deps - we just need to keep colcon
# from ALSO trying to build the stale source copies.
#
# Using find so we catch packages whether they live at the workspace root
# (sdformat_urdf) or under a sub-dir (some are inside vcs-imported repos).
#
# micro_ros_agent: this is the *micro-ROS* project's ROS 2 wrapper, pinned
# to an old Micro-XRCE-DDS-Agent (v2.4.2) that wants Fast-CDR v1. Jazzy
# ships Fast-CDR v2.2.7, so the source build fails. We already built the
# eProsima Micro-XRCE-DDS-Agent v3.0.1 directly above (the daemon at
# /usr/local/bin/MicroXRCEAgent), which is what ArduPilot SITL actually
# talks to. The ROS 2 wrapper isn't needed.
RUN cd /opt/ardupilot_ws/src \
    && for pkg in ros_gz sdformat sdformat_urdf micro_ros_agent; do \
        find . -maxdepth 3 -type d -name "$pkg" \
            -exec sh -c 'touch "$1/COLCON_IGNORE"' _ {} \; ; \
    done

WORKDIR /opt/ardupilot_ws

# rosdep was initialized by the upstream osrf image; just update + install
# the deps declared in package.xml across the workspace.
RUN rosdep update --rosdistro=${ROS_DISTRO} \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------------------------
# Micro-XRCE-DDS-Gen: the IDL code generator that pairs with the Agent we
# built above. ArduPilot's colcon `ardupilot_sitl` package invokes waf with
# DDS enabled, which calls `microxrceddsgen` to generate the C++ bindings
# between SITL (the Micro-XRCE-DDS *client*, embedded in arducopter) and
# the Agent (the daemon at /usr/local/bin/MicroXRCEAgent). The earlier
# `./waf copter` step didn't enable DDS so it didn't need this tool;
# ardupilot_sitl does.
#
# The generator is a Java tool. We install OpenJDK then run the gradle
# wrapper to build the JAR + launcher script, then symlink the launcher
# into /usr/local/bin so it's on PATH for colcon. Pinned to v2.0.2 which
# matches the Agent v3.0.1 we installed.
# -------------------------------------------------------------------------
# Use ArduPilot's *fork* of Micro-XRCE-DDS-Gen, not eProsima's upstream.
# ArduPilot's waf calls microxrceddsgen with `-default-container-prealloc-size`
# (and a few other flags) that don't exist in upstream releases - they're
# patched into the ardupilot fork specifically for this use case.
# The fork's `master` branch is what the official ArduPilot ROS 2 setup
# docs point at.
ARG MICRO_XRCE_GEN_REF=master
# Force JDK 17 for the gradle build: Micro-XRCE-DDS-Gen ships Gradle 7.6,
# which only supports Java 8-19. Noble's default-jre points at Java 21,
# which makes Gradle 7.6 explode with "Unsupported class file major version
# 65". Setting JAVA_HOME + PATH for this RUN keeps the build pinned to 17.
# Runtime invocations of microxrceddsgen are fine on any Java (the JAR is
# already-compiled bytecode, and newer JVMs are backward-compatible).
RUN apt-get update \
    && apt-get install -y --no-install-recommends openjdk-17-jdk-headless \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --depth 1 --branch ${MICRO_XRCE_GEN_REF} --recurse-submodules \
        https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git /opt/Micro-XRCE-DDS-Gen \
    && cd /opt/Micro-XRCE-DDS-Gen \
    && JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 \
       PATH=/usr/lib/jvm/java-17-openjdk-amd64/bin:$PATH \
       ./gradlew assemble

# Put the script directory directly on PATH instead of symlinking the
# launcher into /usr/local/bin. The microxrceddsgen launcher uses
# `dirname "$0"` (which doesn't follow symlinks) to compute its install
# prefix and find the JAR at <prefix>/../share/microxrcedds/...jar. A
# symlink in /usr/local/bin makes the script look for the JAR under
# /usr/local/share, where it doesn't exist. Putting the real scripts dir
# on PATH avoids that.
ENV PATH=/opt/Micro-XRCE-DDS-Gen/scripts:${PATH}

# Git refuses to operate on repos it doesn't own (CVE-2022-24765
# mitigation). The colcon build runs as root, but /opt/ardupilot is owned
# by ros (since we chowned it earlier so the waf build could run as ros).
# ardupilot_sitl's CMake calls `git rev-parse` to embed a version string in
# the binary and crashes with exit 128. `safe.directory '*'` tells git to
# trust every repo on the system - acceptable inside this container since
# nothing untrusted lives here.
RUN git config --system --add safe.directory '*'

# Install ArduPilot's Python build deps system-wide.
#
# install-prereqs-ubuntu.sh tried to install these earlier, but it checks
# for /.dockerenv to detect "am I in a container?" - which Docker Desktop's
# VM doesn't expose - so it took the non-Docker path and ran `pip install
# --user` as the ros user. That puts the packages under /home/ros/.local,
# which root (the user running colcon) can't see, so the ardupilot_sitl
# build fails with "you need to install pexpect" inside waf.
#
# Installing them system-wide here makes them visible regardless of which
# user runs Python. The list mirrors the build-time subset of
# install-prereqs's PYTHON_PKGS (skipping debugging tools like MAVProxy
# and pygame, which are heavy and not needed for the waf build itself).
RUN PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install --no-cache-dir \
        pexpect future empy==3.3.4 pymavlink ptyprocess \
        dronecan lxml intelhex pyserial

# Build the overlay. --symlink-install lets you edit Python sources inside
# /opt/ardupilot_ws without rebuilding (not that you'll be editing the
# overlay often, but it costs nothing).
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF"

# -------------------------------------------------------------------------
# Patch ardupilot_sitl's MicroRosAgentLaunch to use the eProsima
# MicroXRCEAgent binary directly instead of `Node(package="micro_ros_agent")`.
#
# Why this is needed: ardupilot_sitl's launch.py calls the micro_ros_agent
# ROS 2 wrapper package (from the micro-ROS project) which we COLCON_IGNORE'd
# above (Fast-CDR v1 vs v2 mismatch). The underlying agent is the same
# eProsima daemon we built standalone at /usr/local/bin/MicroXRCEAgent;
# we just swap how the launch invokes it. ExecuteProcess is already
# imported at the top of the file, so the patch is purely a substitution.
RUN python3 - <<'PYEOF'
from pathlib import Path
p = Path("/opt/ardupilot_ws/install/ardupilot_sitl/lib/python3.12/site-packages/ardupilot_sitl/launch.py").resolve()
text = p.read_text()
old = '''        # Create action.
        node = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            namespace=f"{micro_ros_agent_ns}",
            output="both",
            arguments=args,
        )
        return node'''
new = '''        # Create action.
        # Use eProsima MicroXRCEAgent binary directly (micro_ros_agent ROS
        # wrapper is COLCON_IGNORE\\'d - Fast-CDR v1/v2 mismatch on Jazzy).
        node = ExecuteProcess(
            cmd=["MicroXRCEAgent"] + args,
            output="both",
        )
        return node'''
if old not in text:
    raise SystemExit("MicroRosAgentLaunch marker not found - upstream ardupilot_sitl/launch.py changed; patch needs updating")
p.write_text(text.replace(old, new))
print(f"Patched: {p}")
PYEOF

# -------------------------------------------------------------------------
# Python ML deps for perception.
# Ultralytics installs torch as a dep. We pin to the CPU wheel by default
# because the Intel iGPU on a ThinkPad can't accelerate PyTorch anyway, and
# pulling the full CUDA torch (~2.5 GB) is wasted weight here.
#
# If you later add an NVIDIA GPU: rebuild with --build-arg TORCH_INDEX=cuda
# (or just `pip install --upgrade torch` inside the container against the
# CUDA index).
# -------------------------------------------------------------------------
ARG TORCH_INDEX=cpu
# --ignore-installed: Ultralytics 8.4+ requires numpy>=2.x, but Noble's
# python3-numpy is apt-installed at 1.26.4 and pip can't uninstall it (no
# RECORD file on apt-managed packages). --ignore-installed makes pip skip
# the uninstall check and write the new version to /usr/local/lib/python3.12,
# which Python's path search prefers over /usr/lib/python3 anyway. The apt
# numpy stays in place for scipy/matplotlib/opencv (which depend on it via
# apt) but Python imports resolve to the newer pip-installed one.
# Why --index-url + a constraints file (not --extra-index-url): with both
# indexes visible, pip picks PyPI's newer CUDA torch (+ ~5 GB of nvidia-*
# wheels), and the second command's --ignore-installed would re-resolve torch
# from PyPI too. The constraint pins the exact +cpu wheel for that step.
RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed \
        --default-timeout=120 --retries 10 \
        --index-url https://download.pytorch.org/whl/${TORCH_INDEX} \
        "torch>=2.3" "torchvision>=0.18" \
    && pip3 freeze | grep -iE '^(torch|torchvision)==' > /tmp/torch-constraints.txt \
    && cat /tmp/torch-constraints.txt \
    && pip3 install --break-system-packages --no-cache-dir --ignore-installed \
        --default-timeout=120 --retries 10 \
        -c /tmp/torch-constraints.txt \
        --extra-index-url https://download.pytorch.org/whl/${TORCH_INDEX} \
        "ultralytics>=8.3" \
        "opencv-python-headless" \
        "transforms3d" \
        "shapely"

# -------------------------------------------------------------------------
# Pin setuptools < 80. The torch/ultralytics install above pulls in
# setuptools >= 80, which removed the `--editable` option from the `develop`
# command (setuptools 80.0.0 defers editable installs to pip via PEP 517).
# colcon's `--symlink-install` calls `setup.py develop --editable ...` for
# ament_python packages, so any setuptools >= 80 breaks the build with
# "error: option --editable not recognized". Keep this as the LAST pip step
# touching setuptools so it wins. Ref: colcon/colcon-core#696.
# -------------------------------------------------------------------------
RUN pip3 install --break-system-packages --no-cache-dir "setuptools<80"

# -------------------------------------------------------------------------
# Entrypoint - sources every overlay so `docker compose exec ros2 bash` lands
# you in a fully-sourced shell. See entrypoint.sh for details.
# -------------------------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Fix ownership of the user's home so pip caches etc. land in the right place
RUN chown -R ${UID}:${GID} /home/${USERNAME}

USER ${USERNAME}
WORKDIR /workspaces/multi-agent-uavs

# A friendly bashrc that auto-sources the overlays for interactive shells
# (entrypoint.sh handles non-interactive 'docker exec' invocations too).
RUN echo 'source /usr/local/bin/entrypoint.sh' >> /home/${USERNAME}/.bashrc \
    && echo 'export PS1="\[\e[1;32m\]ros2\[\e[m\]:\[\e[1;34m\]\w\[\e[m\]\$ "' \
        >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
