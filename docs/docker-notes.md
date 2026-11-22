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

## Image is ~24 GB. That's normal.

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

## FDM handshake silence: RESOLVED (verified 2026-11-02; history below)

> **Update 2026-11-02:** on a fresh rebuild the handshake works (SITL logs "JSON received", `/ap/v1/*` topics are live, arm/takeoff succeed). The symptom below was most likely an environment problem. See "2026-11-02 rebuild notes". The rest of this section is kept as history.

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

## 2026-11-02 rebuild notes (Ubuntu 24.04 host, Docker 29)

18. **Docker 29's containerd image store ignores `data-root`.** Layers and BuildKit cache live in `/var/lib/containerd`, so moving only `data-root` left the 25 GB image on `/` and the build died at export with "no space left on device". Move containerd's root too (`root = "..."` at the top of `/etc/containerd/config.toml`) and add `RequiresMountsFor=` drop-ins for `containerd` and `docker`. A build needs about 2x the image size (cache + unpacked image), roughly 55 GB.
19. **Pinned-SHA clones need `--shallow-since`, not `--depth 50`.** Master moves past 50 commits within weeks and `git checkout <SHA>` fails with "reference is not a tree". The Dockerfile now uses `--shallow-since=<date of the pin>`. Bump the date with the SHA.
20. **CPU torch needs `--index-url`, not `--extra-index-url`.** With both indexes visible pip picks PyPI's newer CUDA torch plus ~5 GB of `nvidia-*` wheels, which also timed out after 80 minutes. The Dockerfile now installs torch/torchvision from the CPU index and pins the exact `+cpu` wheels with a constraints file for the Ultralytics step (its `--ignore-installed` would otherwise re-resolve torch from PyPI). Image is 23.8 GB.
21. **Non-interactive `exec bash -lc` skips `.bashrc`.** Source `/usr/local/bin/entrypoint.sh` explicitly, or the iris SDF cannot be resolved and the drone never spawns (see gotcha #15).
22. **`/ap/v1/*` services hang under `rmw_cyclonedds_cpp`.** Requests are sent but never answered; use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` for the client. Topics interoperate under both.
23. **`pkill` leaves SITL processes.** Stale `arducopter`/`MicroXRCEAgent` make the next launch's `arducopter` exit with code 1. `docker compose restart ros2` is the clean reset.

## 2026-11-04 multi-drone and actuation notes

Verified on the rebuilt image during the planner review. Full evidence is in `docs/planner-decisions.md`; scripts are in `docs/probes/`.

24. **`multiagent.launch.py` can't launch N iris.** Its `robots` list is hard-coded to 4 different vehicles (iris, lidar iris, rover, plane) with no launch arg. Write our own launch reusing `robots/iris.launch.py`, as `docs/probes/rtf.sh` does by patching a copy. Each instance gets `namespace`, `instance`, `sysid` and `use_instance_dir`; only instance 0 runs the DDS agent. 3 iris reach "JSON received" in about 17 s.
25. **Two namespaces, not one.** Gazebo bridge topics are `/iris_N/*`, autopilot topics and services are `/ap/vN/*` with N = sysid. Nothing is at `/iris_N/ap/...`. `ros2 topic list` right after launch can miss `/ap/` entirely while discovery settles, so don't conclude anything from one early listing.
26. **`/ap/vN/goal_lla` is output, not a command.** `AP_DDS_Topic_Table.h` has it as a `DataWriter`. It echoes the accepted target. The command is `/ap/vN/cmd_gps_pose` (`ardupilot_msgs/GlobalPosition`, BEST_EFFORT + VOLATILE).
27. **`cmd_gps_pose` fails silently in two ways.** `AP_DDS_ExternalControl.cpp` drops it unless `header.frame_id` is exactly `"map"`, and `ready_for_external_control` drops it unless GUIDED and armed. Neither replies, and the subscriber count still shows 1. Use `FRAME_GLOBAL_REL_ALT`: Gazebo navsat altitude reads 10.2 where the autopilot reads 584.2. One message is enough to fly the whole way (tested under CycloneDDS only), so stopping publication doesn't stop a flight.
28. **The iris camera is on a gimbal, not fixed.** The bridge's `/camera/*` comes from `pitch_link` on `iris_with_gimbal`. It sits 52 deg off nadir, with the horizon and the drone's own leg in frame, and stays there when armed and flying (joints -120, -30, 135 deg). Measured `mount_quat` [0.3269, 0.8365, 0.4187, -0.1346] matched spawned markers to 0-4 px; the nadir default missed by 205-265 px. Publishing to `/gimbal/cmd_*` over `gz topic` left the joints unchanged. **Fixed 2026-11-16 (T6)** with ArduPilot mount params, `uav_planning/config/gimbal_nadir.parm`: see gotchas 39-40.
29. **Real-time factor falls fast.** Headless, idle: 1 iris 0.46, 3 iris about 0.25, 4 iris 0.11 (the 1 and 4 runs had no confirmed readiness), on 6 cores and 7.1 GB RAM with swap in use. Cause not established. **Update 2026-11-16 (T7):** re-measured with `swarm.launch.py followers:=false` and readiness confirmed: 1 iris 0.96, 4 iris 0.24. The cause is gotcha 43.
30. **Each extra SITL instance writes `mav_N_1.parm` in the repo root.** All of them are gitignored since 2026-11-09 (`mav*.parm`).

## 2026-11-08 first planner flight notes

31. **Plain `exec bash -c` loses MAVProxy, and SITL then hangs before the FDM handshake.** MAVProxy is in `/home/ros/venv-ardupilot/bin`, which only a login shell puts on PATH (`~/.profile`). Under `bash -c`, `iris_runway.launch.py`'s `mavproxy.py` dies with exit code 127, and `arducopter` waits forever in `accept()` on TCP 5760 (SERIAL0). Its "Waiting for connection ...." is that port, not 9002. You get no "JSON received" and no `/ap/*`, while Gazebo runs normally (RTF 0.46, `/navsat` publishing). This looks exactly like the May "FDM silence" and was probably its cause, unproven. Failed 3 of 3 under `bash -c`; worked every time under `bash -lc`. Diagnose with `cat /proc/$(pgrep -x arducopter)/wchan`: `inet_csk_accept` means this. An interactive `exec ros2 bash` is not a login shell either and has the same gap; `bash -l` has MAVProxy and the Gazebo paths (checked). **Always `bash -lc` (or `bash -l` interactively), then `source /usr/local/bin/entrypoint.sh` in the `-lc` case.** The lasting fix is to activate the venv in `docker/entrypoint.sh`, which needs an image rebuild; not done.
32. **`colcon build --symlink-install` copies `uav_planning`'s Python modules; it does not link them** (setuptools 78.1.0 in this container; the cause is not established). An edit runs only after a rebuild. A stale install looks like a fix that didn't work: a traceback cites the new line number but the old code.
33. **Lead, unconfirmed: start our nodes after the sim, not with it.** In 1 run with `iris_runway` and `follower.launch.py` started together, Gazebo published `/clock` at ~1 kHz and the bridge said it created the clock bridge, but ROS `/clock` got nothing under either RMW. `/navsat` arrived at 30 Hz, and the planner's sim-time timer never fired. In 2 runs with the sim first ("JSON received" at +8 s, our nodes about 10 s later), ROS `/clock` ran at 859-937 Hz and the flight completed. The swarm launch (T5) should start project nodes after the sim is up and check `/clock` before relying on it. **Update 2026-11-12:** not reproduced in 3 of 3 cold starts of `swarm.launch.py`, which starts the sim and its `/clock` gate together; that launch uses a dedicated `/clock` bridge, so this is not a disproof. The gate (`wait_for_clock`) stays. **Update 2026-11-16:** seen once under `swarm.launch.py` with 4 followers (and the 2 ms step probe): all 4 SITLs had "JSON received", ROS `/clock` got 0 messages in 180 s, and the gate shut the launch down. The next cold run was fine, and there Gazebo's `/clock` was publishing; in the failed run it was not checked. 1 in 6 swarm cold starts so far. **Update 2026-11-16, later (D37 applied):** seen again on the first cold start of the new 2 ms default, 3 followers; the retry passed the gate at +10 s with Gazebo `/clock` and ROS `/clock` (266 Hz) both flowing. Swarm cold starts with followers: 2 of 4 silent at 2 ms, 0 of 3 at 1 ms. Too few to blame the step. On the next failure, run `gz topic -e -t /clock` while the gate is still waiting, to tell a Gazebo-side stall from a bridge or DDS one.
34. **`/clock` at ~900 Hz costs each `use_sim_time` Python node about 30-40% of a core** (`top`, one sample, idle nodes). `stub_heatmap_publisher` uses sim time only for its stamp, so it could drop it. With 3 followers that is 6+ such nodes, which bears on the RTF timebox (T7, D23). **Update 2026-11-12:** measured about 45% of a core per `/clock` subscriber in the 3-drone swarm (two planners and a watcher probe, one 5 s `top` sample), against 3% for `localization_node` without it. Localization and the stub no longer use sim time, so only the planners pay. `/clock` comes 1000 times per sim second, so the cost grows as RTF improves.

## 2026-11-12 swarm launch notes

35. **With namespaced drones, nothing publishes ROS `/clock`.** Upstream's `iris_bridge.yaml` bridges `clock` as a relative topic, so under namespace `iris_N` it becomes `/iris_N/clock`. `iris_runway` has no namespace, which is why single-drone runs had `/clock`. With 3 iris from `rtf.sh`, `ros2 topic hz /clock` found no publisher, and any `use_sim_time` node would never tick. `swarm.launch.py` bridges `/clock` once in the global namespace and leaves it out of the per-drone bridge.
36. **Do not wrap upstream's `robot.launch.py` in a scoped `GroupAction`.** It reads `use_gz_tf` in an `OnProcessStart` handler that fires after the group has dropped its launch configurations, and the launch dies with "launch configuration 'use_gz_tf' does not exist". Upstream's own `multiagent.launch.py` does not scope; pass every argument explicitly instead.
37. **Use a new `PythonLaunchDescriptionSource` for every include of the same file.** A source caches the parsed file, so a launch file that creates its `Node`s at parse time (like `follower.launch.py`) hands every include the same objects, and launch fails with "ExecuteLocal action ... executed more than once". Files that build nodes inside an `OpaqueFunction` (like upstream's `robot.launch.py`) are not affected.
38. **After a host reboot, check the USB data partition before starting Docker.** On 2026-11-12 the daemon (`data-root` `/mnt/docker-data/docker`) came up while the partition was not mounted (only the `/mnt/usb` autofs placeholder), and `docker compose ps` hung past 120 s. Once the USB was mounted, `docker compose up -d` worked. Check with `mount | grep docker-data` first.

## 2026-11-16 camera to nadir notes

39. **A defaults file listed last can silently drop `DDS_USE_NS`.** SITL's `--defaults` takes a comma list, and later files override earlier ones. But `AP_Param` decides whether to run its second pass, the one that applies params registered late such as AP_DDS's, from the last file alone (`done_all_default_params`, set per file). Upstream's list ends with `dds_use_ns.parm`, which triggers that pass. Append a file whose params are all known at boot (like our mount params) and `DDS_USE_NS` stays 0: topics come up as `/ap/*` instead of `/ap/v1/*`, and `arm_takeoff` times out "waiting for /ap/v1/mode_switch". **Keep a `dds_*.parm` file last.** Single-drone nadir run, from a fresh directory (gotcha 40):

    ```bash
    S=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params
    G=$(ros2 pkg prefix ardupilot_gazebo)/share/ardupilot_gazebo/config
    N=$(ros2 pkg prefix uav_planning)/share/uav_planning/config/gimbal_nadir.parm
    ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false \
      defaults:=$S/copter.parm,$G/gazebo-iris-gimbal.parm,$N,$S/dds_udp.parm,$S/dds_use_ns.parm
    ```

    `swarm.launch.py` builds this order itself.
40. **A saved `eeprom.bin` beats every defaults file.** A defaults value is applied only if the param is not already in storage. `iris_runway` writes `eeprom.bin` (and `mav.parm`, `logs/`, `terrain/`) into the working directory, so launching from the repo root reuses the old one there (gitignored). Anything ever saved to it, such as a `param set MNT1_...` in MAVProxy, silently overrides our `.parm` files. Launch from a fresh directory (`mkdir -p /tmp/sitl && cd /tmp/sitl`) or delete the old eeprom. The swarm uses per-instance directories.
41. **In a non-interactive `bash -lc`, a backgrounded `ros2 launch ... &` ignores Ctrl-C.** Background jobs in a non-interactive shell start with SIGINT ignored, so `kill -INT` does nothing and the launch keeps running. Use `timeout -s INT <secs> ros2 launch ...` or `docker compose restart ros2`.
42. **The big RAM user on this host is the WinBoat Windows VM, not Chrome.** It runs as the Docker container `WinBoat` (`ghcr.io/dockur/windows`), a qemu process showing as `windows` in `ps`, about 3 GB resident, owned by root (so `kill` from your user fails). With it up, 1.3-1.4 GB was free; after `docker stop -t 120 WinBoat` (lets Windows shut down cleanly), 4.3 GB. Check `ps -eo rss,comm --sort=-rss | head` before 3-4 drone runs; `docker start WinBoat` brings it back. **Rule (user, 2026-11-16):** WinBoat stays stopped for all sim work and the sim is never measured with it up; before a launch, `docker ps --filter name=WinBoat`, and if it is running, tell the user at once. `pgrep chrome` also matches the Claude desktop app's `chrome_crashpad_handler`, so a count of 1 does not mean Chrome is open.

## 2026-11-16 sim speed notes (T7)

43. **RTF is set by one Gazebo thread, not by cores or RAM.** At 4 iris, `top -H -p $(pgrep -d, -f "gz sim")` shows the main thread at about 80 % of a core and every other thread under 6 %, with total CPU about 215 % of 600 %. That thread spends about 0.85 ms of CPU per drone per 1 ms physics step, so RTF is about 1 / (N x 1 ms): 0.96 at 1 iris, 0.24 at 4. Removing MAVProxy, the camera sensor, or the 1 kHz `joint_states` and `imu` bridges each changed RTF by 10 % or less. Fewer steps per sim second is what helps (gotcha 44). Measure with `docs/probes/rtf_swarm.sh N`; variants with `docs/probes/rtf_variant.py`.
44. **A 2 ms physics step doubles RTF but blocks arming at ArduCopter's default loop rate.** `max_step_size` 0.002 gave 0.48 at 4 iris idle, but every drone refused to arm: `Arm: Gyro 0 rate 500Hz < loop rate*1.8 720Hz` and `Main loop slow (333Hz < 400Hz)`, repeated for minutes. The IMU samples once per physics step, and arming wants at least 1.8 x `SCHED_LOOP_RATE` (default 400). `SCHED_LOOP_RATE 250` in a defaults file before the `dds_*.parm` files (gotcha 39) fixes it: 3 and 4 followers then flew to ARRIVED, hovering at RTF 0.72 and 0.53. Only takeoff, transit and hover were tested. **Since D37 (2026-11-16) this is the `swarm.launch.py` default** (`physics_step`, with `config/loop_250hz.parm`); `physics_step:=0.001` gives upstream timing. For `iris_runway` it is not applied.
45. **`top`'s process name for `gz sim` is `ruby`** (the `gz` front end is a Ruby script that runs the server in-process). Find it with `pgrep -f "gz sim"`.

## 2026-11-17 ground-truth detector notes

46. **WinBoat can start itself after a reboot.** On 2026-11-17 it was up 56 s after the host came back, holding about 3 GB. Check `docker ps --filter name=WinBoat` after every reboot and `docker stop -t 120 WinBoat` (gotcha 42). Its restart policy was set to `no` that day, so it should stay down; the previous policy was not recorded (`docker update --restart=unless-stopped WinBoat` restores the likely original).
47. **The sim camera is 640x480, fx = fy = 205.47 (2.0 rad HFOV), 10 Hz.** Not the 1.2 rad lens (fx 467.8) that `fake_detection_publisher` and the first detector defaults assumed. At 10 m the footprint is about 31 x 23 m, not 13.7 x 10.3, so a hovering drone at home covers most of a 20 x 20 m grid. `leader_node` and the detector read K from `camera_info`, so live runs were right; only the stub's synthetic `camera_info` differs (the detector's fallback is now 205.47).
48. **`uav_mapping heatmap.launch.py` always started `fake_detection_publisher`**, which publishes a box at pixel (320, 240) at score 0.9 on `/detections` and fights any real detector: the ground cell under the drone climbs no matter what. Use `stub:=false` (added 2026-11-17).
49. **`pkill -f NAME` and `pgrep -f NAME` inside `docker exec ... bash -lc '...NAME...'` match the shell itself and kill it (exit 143).** The shell's own command line contains NAME. Put the commands in a script, `docker cp` it in and run that, or use a pattern the script text does not contain.
50. **Gazebo target poses without Python bindings:** `gz service -s /world/runway/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req ""` prints the world as protobuf text, static models included, about 1200 lines with one iris and a target. Zero fields are omitted (a model at the origin has no `x` or `y`), and nested link and model poses sit deeper, which `uav_perception/core/scene.py` handles. Spawn with `gz service -s /world/runway/create ...` (as `docs/probes/mount_check.py`) and remove with `/world/runway/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --req 'name: "target_1" type: MODEL'`; the detector sees either within 5 s.

## 2026-11-17 sweep controller notes (session 8)

51. **A bare `y` (or `n`) string parameter arrives as a boolean.** `launch_ros` writes node parameters to a temporary YAML file with PyYAML, whose YAML 1.1 booleans are `yes/no/on/off/true/false` but not `y/n`, so `y` goes out unquoted. ROS's own parameter parser then reads it as `true`, and a node declaring a string fails with `InvalidParameterTypeException ... 'True' of type 'BOOL'`. `ParameterValue(..., value_type=str)` does not help, because the quoting is lost when the file is written. Seen twice on 2026-11-17 with `sweep_node`'s `lane_axis:=y`; its values are now `north`/`east`. Avoid one-letter string values.
52. **`/ap/vN/set_parameters` saves what it sets.** AP_DDS calls `set_and_save_by_name_ifchanged` (`AP_DDS_Client.cpp`), so the `WP_SPD` that `arm_takeoff wp_speed:=5` sets lands in that SITL instance's `eeprom.bin` and beats every defaults file afterwards (gotcha 40). The swarm keeps per-instance directories under the launch directory, so a later run from the same directory could fly instance 0 at 5 m/s as a follower. Read from the source, not seen live: start each run in a fresh directory (`rm -rf /tmp/sim1`). The service needs Fast DDS, like the other `/ap` services. `WP_SPD` is in m/s (the old `WPNAV_SPEED` was cm/s); ArduPilot's default is 10.
53. **`leader_node` and `heatmap.launch.py` now default to an 8x8 grid at 5 m** (40 x 40 m, D39), not 4x4. Session-7 commands still work but publish 64 cells; `grid_width:=4 grid_height:=4` gives the old 20 x 20 m map. `sweep.launch.py` uses the same defaults; the two must match.
54. **Measure RTF from `/stats` time deltas, not its `real_time_factor` field.** Three-message samples of `real_time_factor` ranged 0.66-1.25 in one steady 2-drone hover (2026-11-18), while 30 s windows of sim time over real time read 0.99, 0.99, 0.99. Take one `/stats` message, wait 30 s, take another, divide the `sim_time` change by the `real_time` change. When parsing its text with awk, match `^ *sec:` and `^ *nsec:` separately (a bare `sec:` also matches `nsec:` lines) and stop at `step_size`, whose `nsec` follows `real_time`. `docs/probes/rtf_windows.sh` does this.
55. **`docker compose up -d` recreated an exited container** (2026-11-18: "Recreated", not "Started"; the cause was not checked, likely a compose-file change since it was created). Nothing was lost, since the stack lives in the image and the bind mount, but runtime-only state goes with it (for example gotcha 17's in-container setuptools fix). Use `docker compose start` for a stopped container, as the end of this file says.
56. **`swarm.launch.py leader:=true` spawns followers off the grid** (y = -5 by default), so a follower climbs where the leader never flies. The planner handles a drone outside the grid: it holds nothing (value 0) and takes the best open sector as soon as it reaches its layer (D26). The leader bridges only `camera_info`; follower detectors and `belief_map_node` use the sim-camera defaults (640x480, fx 205.47, gotcha 47), so a different camera needs `camera:=true` or matching parameters.
57. **After a reboot `docker compose start` can fail on the X login file, and that is what makes `up -d` recreate the container** (2026-11-20, explains gotcha 55). The compose file mounts `${XAUTHORITY}`, and Wayland (mutter) gives that file a new random name every login (`/run/user/1000/.mutter-Xwaylandauth.XXXXXX`). The stopped container still points at the old name, Docker creates an empty root-owned directory there, and `start` fails with "error mounting ... to rootfs at /home/ros/.Xauthority: not a directory". `docker compose up -d` from a shell with the current `XAUTHORITY` recreates the container with the new path and works. The stack lives in the image and the bind mount, so only runtime state is lost (gotcha 58). The old empty directory is harmless.
58. **After a recreation, check setuptools with `/usr/bin/python3`, not `python3`** (2026-11-20). In a login shell `python3` is the ArduPilot venv's (`~/venv-ardupilot`, setuptools 84.2.1), so gotcha 17's check reads 84 whatever colcon sees. colcon runs `/usr/bin/python3`. In the recreated container `colcon build --symlink-install` failed with `No module named 'setuptools.command.bdist_wheel'`: a `setuptools-84.2.1` copy was left in `/usr/local/lib/python3.12/dist-packages`. This was after one `sudo pip3 uninstall` whose output was not kept, so the state before any uninstall is not known. `sudo /usr/bin/pip3 uninstall -y --break-system-packages setuptools`, repeated until `/usr/bin/python3 -c "import setuptools; print(setuptools.__version__)"` reads 68.1.2, fixed it; the build and all tests passed, and it survived `docker compose restart`. After the next power loss (2026-11-20, session 11) it took two uninstalls: a `setuptools-78.1.0` copy sat over the 84.2.1 one, and colcon failed on `option --uninstall not recognized`. Repeat until the version reads 68.1.2, then build.

## 2026-11-20 mission node notes (session 11)

59. **A probe started with `docker compose exec -d ... > file.log` writes its log in chunks, not per line.** Python block-buffers stdout when it is redirected to a file, so `belief_live_check.py`'s log sat at its first second for about a minute while the run went on. The launch log is unaffected (ROS loggers flush). Run probes as `python3 -u ...` to watch them live; the full log is written either way by the time the probe exits.
60. **Latched topics read from the CLI need the QoS spelled out**: `ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /mission_report std_msgs/msg/String --field data`. Without `transient_local` a late `echo` waits for the next message, and `/mission_report` is sent once.
61. **A swarm run is at the DDS participant ceiling: 32 processes per domain under `rmw_cyclonedds_cpp`** (2026-11-22, session 13). `swarm.launch.py leader:=true num_followers:=3` already held about 30 participants (one per process that uses ROS). Adding `arena_view`'s `set_pose_bridge` and `ground_marker_node` made `belief_map_node` and iris_3's `follower_planner_node` die at start with "Failed to find a free participant index for domain 0". The mission then sat in WAITING (no `/heatmap`) and the followers never got a goal; the same error also made the ROS-based pytest tests fail while a sim was up (they pass with `ROS_DOMAIN_ID=77`). Counted live from `/proc/net/udp` (unicast discovery ports 7410 + 2 x index): 17 in use at 35 s, 30 at full load with the limit raised, highest index 33, past the old limit of 31. Cause: the RMW's default `MaxAutoParticipantIndex` of 32. Fix: `CYCLONEDDS_URI=<CycloneDDS><Domain id="any"><Discovery><MaxAutoParticipantIndex>119</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>`, now in `docker/docker-compose.yml`, but only a recreate (`docker compose up -d`; `restart` keeps the old environment, and a recreate needs gotcha 58's setuptools check) applies it. Applied 2026-11-22: the container was recreated, setuptools took two uninstalls (78.1.0, then 84.2.1, to 68.1.2), and a full run then had 0 participant errors with no override. In a container that predates the setting, pass it to every shell with `docker compose exec -e CYCLONEDDS_URI=...`, the launch and each probe alike: a shell without it, started while the sim holds indexes 0 to 31, cannot join. 119 is the most a domain holds (port 7410 + 2 x 119 stays below domain 1's base 7650). That is about four times the old room. The cost of one more drone was not measured (4 drones, a belief map, a mission node and the arena_view nodes came to 30), so count participants from `/proc/net/udp` as above before a larger swarm run.
62. **Gazebo window and `arena_view` facts, 2026-11-22 (session 13).** (a) The world's only ground is upstream's `runway` model: a collision plane plus the runway texture and a grass mesh. Replace the visuals, never the model, or the drones fall; `arena_view` swaps in a plain ground with the same collision plane. (b) `<transparency>` belongs on the `<visual>`; inside `<material>` Gazebo warns that it is not defined in SDF and the surface stays opaque. (c) `gz sdf -k FILE` validates a world or drone SDF (the gimbal's `gz_frame_id` warning is upstream's). A drone SDF can include a model by `file://` path, which is how `arena_view` colours the propellers per role without touching the physics. (d) A world `<gui>` block replaces Gazebo's default panels; `GzScene3D`'s `<camera_pose>` sets the start view (pitch 1.5707 looks straight down; yaw 1.5708 puts +x to the right and +y up). (e) Live view tools: `gz service -s /gui/move_to/pose` (GUICamera, quaternion for that view is x -0.5, y 0.5, z 0.5, w 0.5) and `/gui/camera/view_control` with `"orbit"` or `"ortho"`; the orthographic view's zoom could not be set from a pose, so it was dropped, and both calls last only until the window closes. (f) A static, visual-only model can be created with `/world/runway/create` and moved with `/world/runway/set_pose` (gz.msgs.Pose, by name); read its pose with `gz model -m NAME -p`. `ros_gz_bridge` exposes `set_pose` as `ros_gz_interfaces/srv/SetEntityPose` (`parameter_bridge /world/runway/set_pose@ros_gz_interfaces/srv/SetEntityPose`), and it answers under CycloneDDS (unlike `/ap/vN/*`, gotcha 22). (g) Through a perspective camera at height H a point at height h appears pushed out from the picture's centre by H / (H - h): drones at 8-14 m under a camera at 48 m look 1.2-1.4 times too far out, so judge sectors by the ground markers. (h) The container reaches the display with no `xhost` when the compose file's `XAUTHORITY` mount is current (`XOpenDisplay` from the container returned OK).

## Bringing the stack up and down

```bash
# Always start with this (the desktop daemon can't render Gazebo with the iGPU)
docker context use default

# Bring up the container in the background
docker compose -f docker/docker-compose.yml up -d

# Open an interactive shell inside it (run this in every new terminal you want).
# -l (login) is required: plain `bash` lacks MAVProxy on PATH and SITL hangs (gotcha 31).
docker compose -f docker/docker-compose.yml exec ros2 bash -l

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
