# Docker setup notes — gotchas hit while building the image

These are non-obvious things that took iteration to figure out. Keeping them here so the next person (including future-me) doesn't rediscover them.

## Use the system Docker daemon, not Docker Desktop

Docker Desktop on Linux runs containers inside a VM. That VM blocks two things this project needs:

- `/tmp/.X11-unix` bind mount — appears inside the container as files but doesn't connect to the host's X server (Qt apps fail with "could not connect to display :0").
- `/dev/dri` device passthrough — host's iGPU isn't visible from inside Desktop's VM, so Gazebo falls back to llvmpipe software rendering (~5 FPS).

Both work natively with `docker-ce` from `download.docker.com`. The two daemons coexist cleanly via `docker context`:

```bash
docker context use default        # system daemon
docker context use desktop-linux  # Docker Desktop
```

Use Desktop for other work (e.g., Honeycomb), `default` for this project.

## Image is ~25 GB. That's normal.

Bulk: ROS 2 desktop-full (3 GB) + Gazebo Harmonic (1.5 GB) + ArduPilot tree with submodules + waf build outputs (4 GB) + colcon workspace install (2 GB) + PyTorch CPU (2.5 GB) + ARM cross-compilers ArduPilot's installer pulls in but we don't need (~500 MB) + OpenJDK 17 + Micro-XRCE-DDS-Gen + Agent + duplicated apt/pip numpy etc. Could drop to ~15 GB with a multi-stage build later; not worth the time right now.

## Upstream gotchas baked into the Dockerfile

Listed in execution order — each one cost a build iteration:

1. **No `ros-jazzy-micro-xrce-dds-agent` apt package.** Build the eProsima `Micro-XRCE-DDS-Agent` from source (pinned to `v3.0.1`).
2. **Ubuntu 24.04 base image ships a pre-existing `ubuntu` user at UID 1000.** Delete it before creating the `ros` user with the same UID.
3. **ArduPilot's `install-prereqs-ubuntu.sh` refuses to run as root.** Run it as the `ros` user (chown the source tree first; ros has `NOPASSWD` sudo for apt installs).
4. **`install-prereqs-ubuntu.sh` only accepts `-y` and `-q` flags.** No `--no-gcs` (MAVProxy installs by default, which is fine for debugging).
5. **`install-prereqs-ubuntu.sh` needs `USER` and `HOME` env vars exported.** Docker's `USER` directive sets kernel UID but not env vars, and the script's internal `usermod -a -G dialout $USER` crashes silently if empty.
6. **The script tries to `apt install python-argparse` for non-`mantic` codenames.** That's a Python-2-era package that doesn't exist on Noble. We `sed` the token out before running.
7. **PEP 668 / `EXTERNALLY-MANAGED` blocks `pip install` on Noble.** Set `PIP_BREAK_SYSTEM_PACKAGES=1` for both `install-prereqs` and the `--break-system-packages --ignore-installed` flags for our own pip installs (Ultralytics requires `numpy>=2`, which can't uninstall the apt-managed `python3-numpy` 1.26.4 — `--ignore-installed` writes the new version to `/usr/local` instead).
8. **`vcs import` of `ros2_gz.repos` re-clones the entire ArduPilot tree as a workspace package.** Pre-symlink the existing `/opt/ardupilot` into the workspace and pass `--skip-existing` to `vcs`. Cuts that step from 30 min to ~1 min.
9. **Source copies of `ros_gz`, `sdformat_urdf`, `sdformat`, and `micro_ros_agent` in the workspace fail to build on Jazzy** — they reference `ignition-math6`, `sdformat12`, or `Fast-CDR v1`, none of which match what Jazzy + Harmonic ship. `COLCON_IGNORE` markers in their src dirs make colcon defer to the apt-installed versions instead.
10. **ArduPilot's `ardupilot_sitl` colcon package re-invokes waf with DDS enabled**, which needs `microxrceddsgen` on PATH. The upstream eProsima `Micro-XRCE-DDS-Gen` doesn't accept ArduPilot-specific flags; we build the **ArduPilot fork** of Micro-XRCE-DDS-Gen (`github.com/ardupilot/Micro-XRCE-DDS-Gen`, branch `master`) which adds `-default-container-prealloc-size` and other args waf passes.
11. **Gradle 7.6 (bundled with Micro-XRCE-DDS-Gen) doesn't support Java 21.** Noble's `default-jre` is Java 21; we install `openjdk-17-jdk-headless` and pin `JAVA_HOME` + `PATH` only for the gradle command.
12. **Symlinking `microxrceddsgen` into `/usr/local/bin` breaks it** — the launcher script uses `dirname "$0"` (doesn't follow symlinks) to find its sibling JAR. Add `/opt/Micro-XRCE-DDS-Gen/scripts` to `PATH` directly instead.
13. **Git's `safe.directory` mitigation (CVE-2022-24765) blocks colcon's git calls** because `/opt/ardupilot` is owned by `ros` but colcon runs as `root`. `git config --system --add safe.directory '*'`.
14. **ArduPilot's pip deps (pexpect, future, empy, pymavlink, etc.) get installed with `--user` as ros**, invisible to root-run colcon. Install them again system-wide with `PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install ...`.
15. **`sdformat_urdf` on Jazzy doesn't reliably resolve `package://` URIs via ament_index**, so `robot_state_publisher` crashes parsing the iris SDF. Workaround: extend `GZ_SIM_RESOURCE_PATH` and `SDF_PATH` to include `/opt/ardupilot_ws/install/ardupilot_gazebo/share` and `ardupilot_gz_description/share` (baked into `entrypoint.sh`).
16. **`iris_runway.launch.py` calls `Node(package="micro_ros_agent")`** which we COLCON_IGNORE'd. Patched at image-build time to use `ExecuteProcess(cmd=["MicroXRCEAgent"] + args, ...)` instead — same args, runs the eProsima agent binary we already have. See the Python sed in the Dockerfile.

## Known limitation (unresolved as of 2026-05-28)

ArduPilotPlugin in Gazebo loads cleanly but never sends FDM packets to SITL on UDP 9002. Verified by `tcpdump` — zero packets in 10 s. SITL stays at `Waiting for connection ...` forever, so AP_DDS never initializes and `/ap/*` topics never appear.

Probably a version skew: we pinned `ARDUPILOT_REF=Copter-4.5.7` but `ardupilot_gz` is on `main`. Smoking gun: the launch references `dds_use_ns.parm` which doesn't exist anywhere on disk — newer ardupilot_gz expects newer ardupilot's default params. The JSON FDM protocol may also have changed.

Fix to try next session: bump `ARDUPILOT_REF` to Copter-4.6.x (or pin `ardupilot_gz` to a tag matching 4.5.7) and full rebuild. Or, if blocked, develop the DGA algorithm against mocked `/ap/*` topics — the rest of the simulation infrastructure is fine for visualization and ROS 2 node development.

## Useful commands once container is running

```bash
docker context use default
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec ros2 bash

# Sanity checks once inside:
glxinfo -B 2>&1 | head -10        # OpenGL renderer should be Intel, not llvmpipe
xclock                              # X11 forwarding check
which arducopter microxrceddsgen   # both should resolve
ros2 pkg list | grep ardupilot     # ardupilot_gz_* packages should be there
```
