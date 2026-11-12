---
name: multi-agent-uavs
description: Project-specific knowledge for Raafay's multi-agent UAV search-and-rescue ROS 2 rebuild — the chosen stack (ROS 2 Jazzy + Gazebo Harmonic + ardupilot_gz + AP_DDS + YOLO26), node architecture, the DGA/BLL algorithms from the LUMS senior thesis, workspace layout, Docker patterns, and common ros2/colcon/sim_vehicle commands. Use aggressively whenever the user mentions drones, UAVs, ROS 2, Gazebo, ArduPilot, SITL, MAVROS, AP_DDS, multi-agent coordination, swarm robotics, YOLO for aerial detection, the LUMS senior project, boustrophedon sweep, heatmap, leader/follower drones, gradient ascent, bilinear log-linear, or anything in this project's scope — even without explicit project name. Contains decisions that take a long time to re-derive, so pull it in whenever those keywords appear.
---

# Multi-Agent UAV Project Skill

This skill loads Raafay's project context. Read it through once at the start of work, then come back to specific sections as needed.

## What this project is

Raafay was one of four authors on a LUMS EE 2025 senior project: *"Distributed Autonomous Surveillance and Rescue Architecture Integrating Computer Vision within Multi-Agent Drone Coordination."* It was selected as one of the top three EE senior projects. The system uses a **leader** drone to sweep an area, run computer vision to detect ground markers, build a probability heatmap, then deploy **follower** drones that use a distributed game-theoretic algorithm to converge on the marked sectors and precision-land via color masking + PID.

The current work is to **rebuild the simulation aspect on a modern ROS 2 stack** on Debian Trixie, and progressively extend it. Raafay self-describes as having "high-level understanding of ROS rather than a very strong technical one" — calibrate explanations accordingly. He's also expressed wanting to leverage AI heavily for boilerplate and infrastructure so he can focus on architecture and the algorithm.

## Source documents in the workspace

In `<repo>/docs/`:

- `thesis.pdf` — full 49-page thesis. Chapter 3 covers methodology + algorithms (BLL and DGA), Chapter 4 the simulation, Chapter 5 the hardware implementation, Chapter 6 cost analysis, Chapter 7 future work.
- `legacy-implementation-guide.docx` — original team's command cheat sheet from the Noetic era. Useful for context, mostly superseded.
- `assessment-ros2-migration.md` — the decision document for this rebuild. **The source of truth** for *why* the current stack was chosen. When in doubt about the rationale behind a design choice, check there first.

## The chosen stack

| Layer | Choice | Reason |
|---|---|---|
| Host OS | Ubuntu 24.04 (was Debian Trixie) | Raafay's primary machine |
| Containment | Docker | Avoid native ROS 2 install pain on Trixie; reproducible env |
| ROS 2 | Jazzy Jalisco (LTS) | Mature, paired with Gazebo Harmonic, supported through May 2029 |
| Simulator | Gazebo Harmonic | Official pairing with Jazzy; Gazebo Classic 11 is EOL since Jan 31 2025 |
| Drone harness | `ArduPilot/ardupilot_gz` | Official ArduPilot + ROS 2 + Gazebo package, ships `multiagent.launch.py` |
| ROS ↔ ArduPilot link | AP_DDS (native, ArduPilot 4.5+) | Replaces MAVROS for the main control loop; cleaner topic interface |
| Perception | Ultralytics YOLO26 | Released Jan 14 2026, small-object detection improvements over YOLOv8 |
| Vision wrapper | `mgonzs13/yolo_ros` or `Alpaca-zip/ultralytics_ros` | Standard ROS 2 nodes around Ultralytics |
| Algorithm | Each follower flies to the best open sector on the shared map, using the thesis's overlap penalty; thesis DGA and BLL are library baselines | Changed 2026-11-04 on measured evidence: the leader's spiky log-odds map has no gradient for 8-neighbour ascent to climb. See `docs/planner-decisions.md` |

**What this project explicitly does NOT use anymore**, even though the original Noetic implementation did:

- MAVROS as the primary bridge — use AP_DDS instead. Only fall back to MAVROS for specific commands AP_DDS doesn't expose yet.
- MAVProxy in the runtime path — only for debugging.
- `iq_sim` / Intelligent Quads launch files — superseded by `ardupilot_gz` (`robots/robot.launch.py` under our own `uav_planning/launch/swarm.launch.py`; `multiagent.launch.py` is hard-coded to 4 different vehicles, so it's only a template).
- `r0ch1n/ardupilot_gazebo_roscam` — superseded by ardupilot_gz.
- Gazebo Classic 11 — EOL. Use new Gazebo (Harmonic).
- Hand-rolled UDP broadcast between drones — DDS handles inter-drone discovery natively.
- Ubuntu 20.04 / Python 3.8 — old. Jazzy runs Python 3.12.
- The "Drone 1 computes for Drone 2" central-planner hack from the original hardware deployment — in simulation we do true decentralization.

If the user asks about something on the "do not use" list, gently flag that the project moved off it and explain what replaced it.

## Workspace layout

```
<repo>/
├── docs/                                   # thesis, legacy guide, assessment
│   ├── thesis.pdf
│   ├── legacy-implementation-guide.docx
│   └── assessment-ros2-migration.md
├── src/                                    # colcon workspace src/ — ROS 2 packages
├── docker/                                 # Dockerfile + compose
├── .gitignore                              # excludes build/install/log/
└── README.md
```

`build/`, `install/`, and `log/` are generated by `colcon build` inside the container. They're gitignored.

The host folder maps directly into the Docker container — typically as `/workspaces/multi-agent-uavs`. Code edits work transparently; building happens inside the container where ROS 2 lives.

## Algorithm recap

**Read this as the thesis algorithms, now baselines.** What flies is defined in `docs/planner-decisions.md`, which wins on any disagreement: best open sector anywhere on the grid, a stay rule with a 0.02 margin, and zones blocked by arrived peers and by lower-sysid goals. Read literally, the thesis's DGA has no stay action and never settles.

### Problem setup (from thesis §3.1.1.2)

- Arena: grid of `m × n` sectors, `S = {s_ij | i ≤ m, j ≤ n}`. Each sector has a probability `p_ij ∈ [0, 1]` of containing a flag/marker.
- Drones: `D = {d_1, …, d_k}`, each at position `x_a ∈ S`. Multiple drones can occupy the same sector (in theory; collision avoidance handles it in practice).
- Local neighborhood: `L(s)` is the immediate 8-connected neighborhood of sector `s`.
- Utility of sector `s_ij` for drone `d_a`:

  ```
  u(s_ij, d_a) = 0       if s_ij ∈ L(x_{-a})   (sector is in another drone's neighborhood)
               = p_ij    otherwise
  ```

  This penalizes overlap and rewards going to high-probability sectors.

### Bilinear Log-Linear (BLL) — simulation algorithm

Exploration-friendly, used in the original simulation:

1. Randomly pick a drone `d_a`
2. Pick a random alternative action `a_l ∈ A_a = L(x_a)`
3. Compute `u(a_l, d_a)` and `u(x_a, d_a)`
4. Softmax-pick between staying and moving: probability of moving = `exp(u(a_l)) / (exp(u(a_l)) + exp(u(x_a)))`

### Distributed Gradient Ascent (DGA) — hardware algorithm

Deterministic, faster convergence, used on hardware:

1. Each drone evaluates utility for all 8 neighbors of its current sector
2. Move to the highest-utility neighbor

DGA was chosen for hardware to avoid wasted flight time from BLL's exploration. In simulation either is fine; the rebuild should support both for comparison.

### Phase structure (from thesis Chapter 5)

1. **Leader Traversal:** boustrophedon sweep at fixed altitude (7m in field tests). Camera continuously running YOLO, detected marker GPS coordinates noted, heatmap built from detections.
2. **Follower Traversal:** followers launch from random positions, run DGA against the leader's heatmap to converge near their assigned markers (within 3-5m proximity is acceptable).
3. **Color Landing:** at the target, switch to HSV color masking, PID-center on the marker, descend.

## Node architecture (the ROS 2 rebuild)

Decompose the system into the following nodes. Each runs as a separate ROS 2 process; communication is via DDS topics.

| Node | Per drone? | Subscribes | Publishes |
|---|---|---|---|
| `perception_node` | one per drone | `/<robot>/camera/image_raw` | `/<robot>/detections` (`vision_msgs/Detection2DArray`) |
| `localization_node` | one per drone | `/<robot>/navsat`, `/<robot>/odometry` | `/<robot>/pose` (`geometry_msgs/PoseStamped` in arena frame) |
| `leader_node` | only on leader | own `/<robot>/detections`, own `/<robot>/pose` | `/heatmap` (`nav_msgs/OccupancyGrid`) |
| `follower_planner_node` | one per follower | `/heatmap`, peers' `/<robot>/pose` and `/<robot>/goal_sector` | `/ap/vN/cmd_gps_pose`, `/<robot>/goal_sector`, `/<robot>/planner_status`; obeys `/<robot>/planner_enable` |
| `landing_node` | one per follower | own camera, own pose | `/ap/vN/cmd_gps_pose` or `/ap/vN/cmd_vel` (after the planner is disabled) |

`<robot>` is the per-drone namespace, e.g. `iris_1`, `iris_2`, set per drone by our own swarm launch. Each drone also has a sysid N for its `/ap/vN/*` topics.

Topic naming convention:
- ArduPilot's AP_DDS publishes everything under `/ap/vN/...`, namespaced by sysid (`/iris_N/ap/...` does not exist)
- Project-level topics (`/heatmap`) are unnamespaced
- Per-drone project topics (`/iris_2/detections`) use the namespace as a prefix

**Design status (2026-11-12):** the follower planner design is settled in `docs/planner-decisions.md`, with tasks T1-T8. T1-T5 are built (T5: `swarm.launch.py`, 3 followers on 3 different peaks, 2026-11-12). T1-T4: shared datum, ROS-free `uav_planning/core/planner.py` and `core/follower.py`, `follower_planner_node`, `arm_takeoff`, `stub_heatmap_publisher` and `follower.launch.py`. One follower flew to ARRIVED in SITL on 2026-11-08 and three on 2026-11-12; T6 (camera to nadir) is next. Read that before planner work. Note: `leader_node` has no flight code, so a sweep controller is still to build.

**Build status (2026-06-10):**
- `localization_node` (`uav_localization`) — **done, verified in sim.** GPS + odometry → `/pose`, a `geometry_msgs/PoseStamped` in an ENU arena frame whose origin is latched at the first GPS fix, or set from the optional `datum_lat/lon/alt` params so a swarm shares one frame.
- `leader_node` (`uav_mapping`) — **done, verified standalone.** Projects `/detections` to the ground (pinhole via `/camera/camera_info` ∘ `/pose` ∘ a static nadir camera mount, flat ground z=0) and accumulates a Bayesian log-odds `/heatmap` (`nav_msgs/OccupancyGrid`, RELIABLE + TRANSIENT_LOCAL). One cell == one thesis sector; defaults match the thesis (5 m/cell, 4×4 over the 20×20 m arena, corner origin), all launch params. Comes with `fake_detection_publisher` (synthetic `Detection2DArray` at a known pixel) and `heatmap.launch.py`. **Checked 2026-11-04 against live `iris_runway`:** the real camera is on a gimbal, 52 deg off nadir, so `leader_node`'s nadir default is wrong and getting the gimbal to nadir is a required task (D30).
- `uav_planning/core/planner.py` — **library done, tested** (no ROS; sector indexing, blocked set, best-open-sector choice, `plan_step`, arena to lat/lon, thesis DGA and BLL baselines).
- `follower_planner_node` (`uav_planning`) — **done, flown 2026-11-08** with one follower: a thin wrapper that calls `core.follower.follower_tick` at 1 Hz on sim time. With it come `arm_takeoff` (GUIDED/arm/takeoff on Fast DDS, then exits) and `stub_heatmap_publisher` (latched 8x8 map).
- `perception_node`, `landing_node` — not built yet.

Conventions established by the two built nodes, mirror them in new ones: input topics are **absolute-default launch params** (so node namespacing doesn't rewrite them, and they flip to `/ap/*` later with no code change); project-level outputs are relative/unnamespaced; thesis-faithful numbers are defaults exposed as params.

**Shell gotcha (2026-11-08):** always `docker compose exec ros2 bash -lc` (or `bash -l` interactively), never plain `bash -c` or `bash`. Only a login shell has MAVProxy on PATH; without it SITL waits forever on TCP 5760, which looks like an FDM handshake failure. Also, `--symlink-install` copies `uav_planning` files, so rebuild after edits. See `docs/docker-notes.md` gotchas 31-34.

**Current dev reality (updated 2026-11-02):** AP_DDS now produces `/ap/v1/*` topics and services in headless `iris_runway` (the FDM handshake issue no longer reproduces; GUIDED + arm + takeoff verified). Input columns can resolve to `/ap/v1/*` or the `ros_gz_bridge` topics (`/imu`, `/navsat`, `/camera/image`, `/odometry`); keep input topic names as launch args. For `ros2 service call /ap/v1/...` use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (cyclonedds hangs), and in non-interactive `exec bash -lc` run `source /usr/local/bin/entrypoint.sh` first. See CLAUDE.md and `docs/docker-notes.md` (2026-11-02 rebuild notes).

QoS conventions:
- Sensor data (images, IMU) → `BEST_EFFORT`, `VOLATILE`, depth 1
- Detections → `RELIABLE`, depth 10
- Heatmap → `RELIABLE`, `TRANSIENT_LOCAL` (followers joining late should still get the current heatmap)
- Commands → `RELIABLE`, depth 1

If a subscriber isn't receiving messages, **first hypothesis is always QoS mismatch.** It's the most common ROS 2 bug.

## Command patterns

### Container lifecycle

```bash
# Build the image (one time)
cd <repo>
docker compose build

# Bring up the container
docker compose up -d

# Shell into it (-l: login shell, or MAVProxy is missing and SITL hangs)
docker compose exec ros2 bash -l

# Inside the container:
source /opt/ros/jazzy/setup.bash
cd /workspaces/multi-agent-uavs
source install/setup.bash   # only after first colcon build
```

### Building

```bash
# From the workspace root inside the container:
colcon build --symlink-install
# --symlink-install lets you edit Python sources without rebuilding

# Build only one package:
colcon build --symlink-install --packages-select uav_planning

# Build with verbose output:
colcon build --symlink-install --event-handlers console_direct+
```

### Running ArduPilot SITL + Gazebo + the swarm

Single drone, headless:

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false
```

`multiagent.launch.py` hard-codes 4 different vehicles with no launch arg, so N iris uses our own launch, which reuses upstream's `robots/robot.launch.py` per drone. Topics are `/iris_N/*` from Gazebo and `/ap/vN/*` from the autopilot, N = sysid. Real-time factor falls quickly: 0.46, about 0.3 and 0.11 for 1, 3 and 4 iris.

Sim plus N followers in one command (T5, flown 2026-11-12):

```bash
ros2 launch uav_planning swarm.launch.py                 # 3 iris + 3 followers
ros2 launch uav_planning swarm.launch.py sim:=false      # followers against a running sim
ros2 launch uav_planning swarm.launch.py camera:=true    # also bridge each camera
```

Drone i is `iris_{i+1}`, sysid i+1, spawned 5 m apart along east, layer 10 + 2 i m. **With namespaced drones upstream publishes no ROS `/clock`** (it lands on `/iris_N/clock`); the swarm launch bridges `/clock` once and gates the followers on `wait_for_clock`. Each `/clock` subscriber costs about 45% of a core, so only planners use sim time. Use `docker compose restart ros2` between runs.

### Talking to a drone manually

Services need Fast DDS, because they hang under the container's CycloneDDS. Mode 4 is GUIDED:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /ap/v1/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
ros2 service call /ap/v1/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
ros2 service call /ap/v1/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 10.0}"
```

Fly to a point with `/ap/vN/cmd_gps_pose` (`ardupilot_msgs/GlobalPosition`, BEST_EFFORT + VOLATILE, tested under CycloneDDS). It is **dropped silently** unless `header.frame_id` is exactly `"map"` and the vehicle is GUIDED and armed. Use `FRAME_GLOBAL_REL_ALT`, because Gazebo and autopilot altitudes differ by 574 m. `/ap/vN/goal_lla` is autopilot output, useful to confirm the accepted target, not a command. See `docs/probes/cmd_gps_pose_probe.py`.

Inspect:

```bash
ros2 topic list | grep -E "iris_1|ap/v1"
ros2 interface show ardupilot_msgs/msg/GlobalPosition
```

### YOLO training

```bash
# Inside the container, with Ultralytics installed:
yolo train \
  model=yolo26n.pt \
  data=datasets/markers/data.yaml \
  epochs=150 imgsz=640 \
  project=runs name=markers_v1

# Inference test:
yolo predict \
  model=runs/markers_v1/weights/best.pt \
  source=docs/test_images/sample.jpg
```

Roboflow workflow: collect images from headless Gazebo (screenshot the camera topic), upload to Roboflow, annotate, export as YOLO format, drop the `data.yaml` into `datasets/markers/`.

## Common debugging recipes

**Symptom: `colcon build --symlink-install` fails with `error: option --editable not recognized`.**
- Not the package — it's setuptools. Version ≥80 dropped the `--editable` option that `--symlink-install` depends on (torch/ultralytics drag it in). Check: `python3 -c "import setuptools; print(setuptools.__version__)"`.
- The image now pins `setuptools<80`. In a container built before that pin, downgrade offline with `sudo pip3 uninstall -y --break-system-packages setuptools` (exposes the apt-installed 68.1.2 underneath — no network needed). The fix is container-runtime only and lost on `docker compose down`; permanent fix is an image rebuild. Full notes: `docs/docker-notes.md` gotcha #17.

**Symptom: Gazebo shows up but the Iris doesn't appear.**
- Check `GZ_SIM_RESOURCE_PATH` is set inside the container. Look at the ardupilot_gz launch file output for the resolved path.
- Try `gz model -m iris_with_standoffs -p` to see if Gazebo can find the model definition.

**Symptom: `ros2 topic list` shows no `/ap/*` topics.**
- First check the AP_DDS agent is running: `ps aux | grep MicroXRCE`. If absent, the launch's `use_dds_agent` arg defaulted false or the patched MicroXRCEAgent invocation failed.
- Check SITL itself is running and listening on UDP 9002: `sudo ss -anup | grep ':9002'` should show `UNCONN 127.0.0.1:9002`. If absent, SITL crashed at boot.
- If both processes are alive but no /ap/*: confirm FDM packets are actually flowing from Gazebo to SITL with `sudo tcpdump -i any -nn -c 30 'udp port 9002'`. **History:** a 2026-05-30 build showed zero FDM packets; it did not reproduce on 2026-11-02. If it returns, first check that the shell sourced `/usr/local/bin/entrypoint.sh` (missing `GZ_SIM_RESOURCE_PATH` stops the iris spawning), that no stale `arducopter` is holding the port (`docker compose restart ros2`), then strace/tcpdump as above. Notes: `docs/docker-notes.md`.

**Symptom: Two nodes on the same machine can't see each other.**
- Almost always a QoS mismatch. Run `ros2 topic info <topic> --verbose` on both sides to check.
- If the publishers are in containers and subscribers are on the host, set `ROS_DOMAIN_ID` consistently and verify the network mode is `host` (not `bridge`).

**Symptom: Gazebo runs slow / RTF below 0.5.**
- First check it's using the GPU: `glxinfo | grep "OpenGL renderer"` should show your real GPU, not `llvmpipe`.
- If running in Docker, the container needs `--gpus all` (NVIDIA) or `/dev/dri` device mount (Intel/AMD).
- SITL frequency complaints: in MAVProxy, `param set ARMING_CHECK 0` reduces SITL overhead during dev.

**Symptom: YOLO node CPU-bound.**
- Use `yolo26n` (nano) for development, `yolo26s` or `yolo26m` for accuracy after the pipeline works.
- Resize images on the publisher side (compressed `sensor_msgs/Image` with 320×320) rather than the YOLO node.

## Naming and conventions

- ROS 2 package names: `uav_<area>`, e.g. `uav_perception`, `uav_planning`, `uav_bringup`, `uav_messages` (if custom message types are needed)
- Python module names: snake_case matching the package name
- Node executables: `<purpose>_node`, e.g. `follower_planner_node`
- Launch files: `<scenario>.launch.py`, e.g. `swarm.launch.py`, `single_iris.launch.py`
- Parameter files: YAML in `<package>/config/`
- Custom message types only when the standard ones (`vision_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`) don't fit

## Style and pedagogy

When explaining things to Raafay, default to:

- Explain *why* a design choice exists, not just *what* the choice is. He has high-level ROS knowledge and is learning the deeper layers, so the reasoning is the learning.
- When introducing a new ROS 2 concept (QoS, lifecycle nodes, actions vs services, executors), give a brief one-paragraph "what it is and when you care" before showing code.
- For boilerplate work (CMakeLists, package.xml, launch file scaffolding), do it for him and explain it briefly. Don't make him hand-type CMake.
- For architectural choices (which nodes exist, how they communicate), pull him in — that's the part he's trying to learn.
- Don't redo the assessment's logic from scratch. If he asks "why are we using AP_DDS instead of MAVROS again?" — it's in `docs/assessment-ros2-migration.md` §5. Point there and summarize.

## When to check the assessment doc

Always check `docs/assessment-ros2-migration.md` before:
- Recommending a different ROS 2 distro (it has a section on why Jazzy specifically)
- Recommending MAVROS over AP_DDS (the trade-offs are in §5)
- Recommending a different simulator (covered in §6)
- Picking a YOLO version (§4)
- Suggesting "future work" enhancements (§10 lists the publication-quality directions)

## Things Raafay said in scoping that affect work

- He wants to **leverage AI heavily** to speed things up. Be willing to write more code without checking in for short tasks; check in on architectural choices.
- He's interested in **learning ROS 2** during the build. So when there's a teaching moment (e.g., first time touching QoS, first time using `ros2 launch` arguments), take it.
- He's open to **ROS 2 over Noetic** — that's already the decision. Don't relitigate it.
- The original project's **future-work items in thesis §5.4.4** are aspirational targets for this rebuild: true decentralization, GPS-denied follower phase, scaling beyond 2 followers, better detection robustness. Suggestions that close those out are extra valuable.

## Quick reference: key URLs

- ardupilot_gz repo: https://github.com/ArduPilot/ardupilot_gz
- ArduPilot ROS 2 docs: https://ardupilot.org/dev/docs/ros2.html
- ArduPilot ROS 2 + Gazebo: https://ardupilot.org/dev/docs/ros2-gazebo.html
- ArduPilot AP_DDS README: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md
- ROS 2 Jazzy docs: https://docs.ros.org/en/jazzy/
- Gazebo Harmonic docs: https://gazebosim.org/docs/harmonic
- ros_gz bridge: https://github.com/gazebosim/ros_gz
- YOLO26: https://docs.ultralytics.com/models/yolo26
- yolo_ros (mgonzs13): https://github.com/mgonzs13/yolo_ros
- ArduPilot guided mode commands: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
- MAVLink common messages: https://mavlink.io/en/messages/common.html
