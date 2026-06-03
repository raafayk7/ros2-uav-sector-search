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
17. **`torch`/`ultralytics` pull in `setuptools>=80`, which breaks `colcon build --symlink-install`.** (Hit 2026-06-03, building the first project package.) setuptools 80.0.0 removed the `--editable` option from the `develop` command — editable installs now defer to pip via PEP 517. colcon's `--symlink-install` runs `setup.py develop --editable …` for every `ament_python` package, so the build dies with `error: option --editable not recognized` even though the package itself is fine (a plain `colcon build` without `--symlink-install` still works, since only `develop --editable` broke). Fixed in the Dockerfile by pinning `setuptools<80` as the **last** pip step — after the torch/ultralytics block so it wins. Ref `colcon/colcon-core#696`. **For a container built before that pin (no rebuild yet), downgrade offline** — Noble's apt `python3-setuptools` (68.1.2) sits underneath the pip copy at `/usr/lib/python3/dist-packages`, so removing the `/usr/local` pip copy exposes it with no network needed: `sudo pip3 uninstall -y --break-system-packages setuptools` (the `--break-system-packages` flag is required on *uninstall* too under PEP 668, not just install), then verify with `python3 -c "import setuptools; print(setuptools.__version__)"` → should read `68.1.2`. This is container-runtime state only: it survives `docker compose stop`/`start` but **not** `docker compose down` (which rebuilds from the still-setuptools-81 image). The permanent fix is a `docker compose build` once the host is online.

## Open issue: FDM handshake silence (diagnosis 2026-05-28, fix attempted 2026-05-30, verification pending)

### Symptom

ArduPilotPlugin in Gazebo loads cleanly but never sends FDM packets to SITL on UDP 9002. Verified by `tcpdump` — zero packets in 10 s. SITL stays at `Waiting for connection ...` forever, so AP_DDS never initializes and `/ap/*` topics never appear.

### Diagnosis (2026-05-30)

Root cause is version skew between `ardupilot` and `ardupilot_gz` checkouts in the image. The previous image pinned `ARDUPILOT_REF=Copter-4.5.7` (Oct 2024) but `ardupilot_gz=main` (April 2026), and the two repos co-evolve closely.

Evidence:

1. **`ardupilot_gz/main/ros2_gz.repos` pins `ardupilot: master`**, not a release tag. The official ardupilot_gz integration tests against ardupilot master, not against Copter stable.
2. **`dds_use_ns.parm` was added to ardupilot on 2026-03-24** (commit `b17c3db3dd`, "Tools: Added DDS tests with new namespace nomenclature"). It is **not present in any Copter stable release tag**, including the latest Copter-4.6.3. The iris launch file in current ardupilot_gz_bringup references this file via `pkg_ardupilot_sitl/config/default_params/dds_use_ns.parm` — so any Copter-* tag will fail the launch silently.
3. **Recent ardupilot_gz commits explicitly touch the FDM handshake path**:
   - `1805eaa` (2026-02-16, WildThumper): "Ensure ArduPilot plugin and SITL have a consistent sim_address"
   - `e26ab1d` (2026-03-25, MultiAgent): switches micro_ros_agent transport from serial to udp4
   The JSON FDM wire format the plugin emits in ardupilot_gazebo's `ros2` branch (recent) has likely drifted from what arducopter Copter-4.5.7 (older) parses.

### Fix attempted

Bumped both refs in `docker/Dockerfile` to specific full-40-char SHAs from the same era:

- `ARDUPILOT_REF` = `ddba3b490eaec04dafa3a12b45b7527aef0d0341` (master HEAD on 2026-05-29, short `ddba3b4`, "Tools: fix setup on docker")
- `ARDUPILOT_GZ_REF` = `8df4dc1726e37504e6fc8b952d02e554cfa3176f` (main HEAD on 2026-04-11, short `8df4dc1`, "Add namespace as launch parameter")

Pinning to SHAs (rather than the floating `master` / `main` we'd otherwise track) preserves reproducibility while keeping both repos aligned to the same window.

The clone commands had to change from `git clone --depth 1 --branch <REF>` to `git init && git fetch --depth 1 origin <SHA> && git checkout FETCH_HEAD`, because `--branch` only accepts branch/tag names, not arbitrary SHAs. GitHub honours `uploadpack.allowReachableSHA1InWant=true` so the SHA fetch works as long as it's reachable from a branch.

**Gotcha hit on first rebuild attempt (2026-05-30):** the SHA must be the **full 40-character hash**. Short SHAs (`ddba3b4`) get interpreted as ref names by `git fetch`, and the upload-pack server replies `fatal: couldn't find remote ref ddba3b4`. Always pin to the full hash.

**Gotcha hit on second rebuild attempt (2026-05-30):** even with the full hash, `git fetch --depth 1 origin <SHA>` against the ardupilot repo from a typical home connection **times out after ~30 minutes** with `RPC failed; curl 56 Recv failure: Connection timed out`. Root cause: GitHub serves precomputed packs for branch heads (fast) but builds a custom pack for arbitrary-SHA requests, which for repos this size doesn't reliably complete. Switched to `git clone --depth 50 --branch master --shallow-submodules --recurse-submodules` followed by `git checkout <SHA>` — uses the fast precomputed-pack path, and `--depth 50` keeps the SHA in the shallow window. If you bump `ARDUPILOT_REF` to a SHA older than 50 commits behind master HEAD, the checkout will fail with "no such object" — increase the depth or convert the pin back to a branch/tag name.

### Outcome of the version bump (2026-05-30, after rebuild)

The bump fixed every layer except the very last one:

- **Build succeeded** after two false starts (short-SHA gotcha, then GitHub-SHA-fetch timeout). Took ~60 min total.
- **SITL boots cleanly** with the new dds_use_ns.parm in its `--defaults` list. eeprom.bin gets written. `arducopter` process is alive on the FDM listener (UDP 9002 on 127.0.0.1).
- **MicroXRCEAgent** is up on UDP 2019 in udp4 mode.
- **Gazebo** loads `runway.sdf`, spawns the `iris` model, all sensors advertise correctly (`/world/runway/model/iris/...` for imu, navsat, camera, magnetometer, air_pressure, altimeter, air_speed, battery).
- **ros_gz_bridge** publishes the Gazebo sensors to ROS 2 at `/imu`, `/navsat`, `/camera/image`, `/camera/camera_info`, `/odometry`, `/magnetometer`, `/battery`, etc.
- **ArduPilotPlugin loads successfully** — log shows `[iris] ArduPilot ready to fly. The force will be with you`, `Loaded system [ArduPilotPlugin] for entity [11]`, `Found IMU sensor with name [imu_link::imu_sensor]`, `Computed IMU topic to be: world/runway/model/iris/link/imu_link/sensor/imu_sensor/imu`. The plugin `.so` exists, `GZ_SIM_SYSTEM_PLUGIN_PATH` points at it, the IMU is found, the topic is computed correctly.

**Still broken:** the plugin emits **zero** UDP packets on port 9002. Verified with `tcpdump -i any -nn udp port 9002 -c 30` — 0 packets captured. SITL stays at "Waiting for connection ..." forever, so AP_DDS never initializes and `/ap/*` topics never appear.

Things ruled out this session (don't redo):

1. **Not a `fdm_addr` / `fdm_port_in` mismatch** — both iris SDF variants (`iris_with_ardupilot`, `iris_with_gimbal`, `iris_with_lidar`) hardcode `127.0.0.1:9002`, which matches `arducopter --sim-address=127.0.0.1 --slave 0` (default JSON physics port).
2. **Not a missing plugin `.so`** — present at `/opt/ardupilot_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo/libArduPilotPlugin.so`, loaded by Gazebo.
3. **Not a missing IMU sensor** — log shows the plugin finds it. IMU topic is alive (visible via `gz topic -l`).
4. **Not a lock_step deadlock** — tried `<lock_step>0</lock_step>` (sed across all iris model SDFs in install/), relaunched, no change. Zero packets on 9002 either way.
5. **Not a permissions issue** — `arducopter`, `MicroXRCEAgent`, and `gz sim` all run as the `ros` user and have appropriate caps.
6. **Not a DDS-side issue** — UDP 2019 has zero packets too, which makes sense because SITL only initializes AP_DDS after it gets FDM. The DDS silence is downstream of the FDM silence, not a separate fault.

### Where to look next session if you come back to this

The plugin is running its update loop (Gazebo log shows IMU topic computed in `OnPreUpdate`, simulation is stepping at full CPU, 12 postupdate worker threads alive) but its `sendto()` to 127.0.0.1:9002 is either not being called or is failing silently. Three diagnostic paths:

1. `sudo strace -f -e trace=sendto -p $(pidof gz)` on the `gz sim -v4 -s` process while the simulation runs. If no `sendto()` calls to 9002 appear, the plugin's `Update`/`PostUpdate` send-path isn't executing.
2. Build a debug version of `libArduPilotPlugin.so` with extra logging around `SendFDMUDP()` (or whatever the current ros2-branch name is) and reinstall.
3. Open an ArduPilot Discord / GitHub issue with this exact diagnosis ("plugin loads, finds IMU, emits zero FDM packets on UDP 9002, lock_step=0 doesn't change behaviour, Gazebo Harmonic + ardupilot_gz main 8df4dc1 + ardupilot master ddba3b4"). The plugin's main maintainer (`srmainwaring`) was active in late March 2026 — the FDM-handshake commit `1805eaa` is theirs — so this is a current/known-to-the-team area.

### Pivot path the project is taking

DGA development can proceed without `/ap/*` for now. The Gazebo sensors already arrive in ROS 2 via `ros_gz_bridge`:

| Project node needs | Topic available now |
|---|---|
| Drone position | `/navsat` (lat/lon/alt), `/odometry` (local frame) |
| Heatmap inputs | `/imu`, `/magnetometer` |
| YOLO frames | `/camera/image`, `/camera/camera_info` |
| Robot pose for TF | `/tf`, `/tf_static`, `/joint_states` |
| Time | `/clock` (Gazebo synthetic clock) |

What we **don't** have without `/ap/*`: a way to send commands back to arducopter's guided mode. That's the actuation half. We can develop the perception + planner + heatmap stack against the topics above, and tackle actuation when we crack the FDM handshake. If we never crack it: fallback to sending arducopter commands via pymavlink directly (MAVLink over UDP 14550, which the SITL is presumably still exposing whether or not its FDM is alive — verify next session).

## Bringing the stack up and down

```bash
# Always start with this (the desktop daemon can't render Gazebo with the iGPU)
docker context use default

# Bring up the container in the background
docker compose -f docker/docker-compose.yml up -d

# Open an interactive shell inside it (run this in every new terminal you want)
docker compose -f docker/docker-compose.yml exec ros2 bash

# Sanity checks once inside:
glxinfo -B 2>&1 | head -10        # OpenGL renderer should be Intel, not llvmpipe
xclock                              # X11 forwarding check
which arducopter microxrceddsgen   # both should resolve
ros2 pkg list | grep ardupilot     # ardupilot_gz_* packages should be there
```

When you're done working:

```bash
# Stop the container but keep it around (fast to restart later)
docker compose -f docker/docker-compose.yml stop

# Start it again next session (same state, anonymous volumes preserved)
docker compose -f docker/docker-compose.yml start

# OR: stop AND remove the container (next 'up -d' creates a fresh container
# from the same image - quick, ~5s. Use this if the container's runtime state
# got into a weird place and you want a clean slate.)
docker compose -f docker/docker-compose.yml down

# Nuclear option: remove the container AND named/anonymous volumes (wipes
# anything written outside the bind-mounted workspace, e.g. /home/ros/.gz/
# Gazebo logs, ros user's bash history, apt-installed packages added at
# runtime like the tcpdump we installed during triage)
docker compose -f docker/docker-compose.yml down -v
```

The image itself (~25 GB) is independent of any of these — it only goes away if you run `docker rmi multi-agent-uavs:jazzy` or `docker image prune -a`. Don't do that unless you want the ~60-min rebuild back.

To kill just a running `ros2 launch` (e.g., the Gazebo + SITL stack) without taking the whole container down, Ctrl-C in the launch terminal. The container keeps running; you can re-launch from any exec shell.
