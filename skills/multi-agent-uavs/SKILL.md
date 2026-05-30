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

In `/home/raafayk7_cteq/Documents/Experiments/multi-agent-uavs/multi-agent-uavs/docs/`:

- `thesis.pdf` — full 49-page thesis. Chapter 3 covers methodology + algorithms (BLL and DGA), Chapter 4 the simulation, Chapter 5 the hardware implementation, Chapter 6 cost analysis, Chapter 7 future work.
- `legacy-implementation-guide.docx` — original team's command cheat sheet from the Noetic era. Useful for context, mostly superseded.
- `assessment-ros2-migration.md` — the decision document for this rebuild. **The source of truth** for *why* the current stack was chosen. When in doubt about the rationale behind a design choice, check there first.

## The chosen stack

| Layer | Choice | Reason |
|---|---|---|
| Host OS | Debian Trixie | Raafay's primary machine |
| Containment | Docker | Avoid native ROS 2 install pain on Trixie; reproducible env |
| ROS 2 | Jazzy Jalisco (LTS) | Mature, paired with Gazebo Harmonic, supported through May 2029 |
| Simulator | Gazebo Harmonic | Official pairing with Jazzy; Gazebo Classic 11 is EOL since Jan 31 2025 |
| Drone harness | `ArduPilot/ardupilot_gz` | Official ArduPilot + ROS 2 + Gazebo package, ships `multiagent.launch.py` |
| ROS ↔ ArduPilot link | AP_DDS (native, ArduPilot 4.5+) | Replaces MAVROS for the main control loop; cleaner topic interface |
| Perception | Ultralytics YOLO26 | Released Jan 14 2026, small-object detection improvements over YOLOv8 |
| Vision wrapper | `mgonzs13/yolo_ros` or `Alpaca-zip/ultralytics_ros` | Standard ROS 2 nodes around Ultralytics |
| Algorithm | Distributed Gradient Ascent (DGA), with Bilinear Log-Linear (BLL) as a comparison baseline | Same algorithms as the original thesis, but each drone runs its own planner (true decentralization) |

**What this project explicitly does NOT use anymore**, even though the original Noetic implementation did:

- MAVROS as the primary bridge — use AP_DDS instead. Only fall back to MAVROS for specific commands AP_DDS doesn't expose yet.
- MAVProxy in the runtime path — only for debugging.
- `iq_sim` / Intelligent Quads launch files — superseded by `ardupilot_gz`'s `multiagent.launch.py`.
- `r0ch1n/ardupilot_gazebo_roscam` — superseded by ardupilot_gz.
- Gazebo Classic 11 — EOL. Use new Gazebo (Harmonic).
- Hand-rolled UDP broadcast between drones — DDS handles inter-drone discovery natively.
- Ubuntu 20.04 / Python 3.8 — old. Jazzy runs Python 3.12.
- The "Drone 1 computes for Drone 2" central-planner hack from the original hardware deployment — in simulation we do true decentralization.

If the user asks about something on the "do not use" list, gently flag that the project moved off it and explain what replaced it.

## Workspace layout

```
/home/raafayk7_cteq/Documents/Experiments/multi-agent-uavs/multi-agent-uavs/
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
| `localization_node` | one per drone | `/<robot>/ap/navsat/navsat0`, `/<robot>/ap/imu` | `/<robot>/pose` (`geometry_msgs/PoseStamped` in arena frame) |
| `leader_node` | only on leader | own `/<robot>/detections`, own `/<robot>/pose` | `/heatmap` (`nav_msgs/OccupancyGrid`) |
| `follower_planner_node` | one per follower | `/heatmap`, all `/<robot>/pose` | `/<robot>/ap/cmd_vel` or `/<robot>/ap/goal_pose` |
| `landing_node` | one per follower | own camera, own pose | `/<robot>/ap/cmd_vel` (when in landing mode) |

`<robot>` is the per-drone namespace, e.g. `iris_1`, `iris_2`. Set this via the `robots` list in the launch file.

Topic naming convention:
- ArduPilot's AP_DDS publishes everything under `/ap/...` per vehicle (already namespaced when using ardupilot_gz)
- Project-level topics (`/heatmap`) are unnamespaced
- Per-drone project topics (`/iris_2/detections`) use the namespace as a prefix

**Current dev reality (2026-05-30):** AP_DDS is not yet producing `/ap/*` topics — see the open FDM-handshake issue in `docs/docker-notes.md`. Until that's resolved, the input columns above resolve to `ros_gz_bridge` topics instead: `/imu` for `/<robot>/ap/imu`, `/navsat` for `/<robot>/ap/navsat/navsat0`, `/camera/image` for `/<robot>/camera/image_raw`, `/odometry` for the position estimate. The output column (`/<robot>/ap/cmd_vel`, `/<robot>/ap/goal_pose`) has no current substitute — actuation goes via pymavlink over UDP 14550 as a fallback. The target architecture above is the goal; nodes should be written so the input topic names are a configurable launch arg, so we can flip them back to `/ap/*` without rewiring code when FDM gets fixed.

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
cd /home/raafayk7_cteq/Documents/Experiments/multi-agent-uavs/multi-agent-uavs
docker compose build

# Bring up the container
docker compose up -d

# Shell into it
docker compose exec ros2 bash

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

The canonical launch (once ardupilot_gz is set up):

```bash
ros2 launch ardupilot_gz_bringup multiagent.launch.py \
  rviz:=true use_gz_tf:=true
```

This brings up Gazebo, RViz, and N ArduPilot SITL instances with DDS. After expanding the `robots` list, each drone gets its own namespace and SITL instance.

To launch the project's own nodes alongside:

```bash
ros2 launch uav_bringup swarm.launch.py num_followers:=3
```

(Once `uav_bringup` exists.)

### Talking to a drone manually

Via AP_DDS — set a goal in the local frame:

```bash
ros2 topic pub --once /iris_1/ap/goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 0.0, z: 7.0}}}'
```

Set velocity directly:

```bash
ros2 topic pub /iris_1/ap/cmd_vel geometry_msgs/TwistStamped \
  '{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}'
```

Inspect what topics a drone exposes:

```bash
ros2 topic list | grep iris_1
ros2 topic info /iris_1/ap/navsat/navsat0
```

### Switching modes / arming

Currently AP_DDS doesn't expose mode-switching as a clean service everywhere; use MAVProxy or a small pymavlink helper for these. The fallback pattern:

```bash
# In a separate terminal, run MAVProxy against the same SITL instance
mavproxy.py --master udp:127.0.0.1:14550 --console
# Then:
mode guided
arm throttle
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

**Symptom: Gazebo shows up but the Iris doesn't appear.**
- Check `GZ_SIM_RESOURCE_PATH` is set inside the container. Look at the ardupilot_gz launch file output for the resolved path.
- Try `gz model -m iris_with_standoffs -p` to see if Gazebo can find the model definition.

**Symptom: `ros2 topic list` shows no `/ap/*` topics.**
- First check the AP_DDS agent is running: `ps aux | grep MicroXRCE`. If absent, the launch's `use_dds_agent` arg defaulted false or the patched MicroXRCEAgent invocation failed.
- Check SITL itself is running and listening on UDP 9002: `sudo ss -anup | grep ':9002'` should show `UNCONN 127.0.0.1:9002`. If absent, SITL crashed at boot.
- If both processes are alive but no /ap/*: confirm FDM packets are actually flowing from Gazebo to SITL with `sudo tcpdump -i any -nn -c 30 'udp port 9002'`. **Known open issue (2026-05-30):** ArduPilotPlugin loads cleanly, finds the IMU, computes the right topic, logs "ArduPilot ready to fly", but emits zero `sendto()` calls to 9002 with no error. lock_step=0 doesn't unblock. Diagnosis pending — strace `gz sim`'s sendto path or debug-build the plugin to localize the silent failure. Full notes in `docs/docker-notes.md`. **Workaround:** project nodes consume `/imu`, `/navsat`, `/camera/image`, `/odometry` from ros_gz_bridge directly (sufficient for DGA development; only commanding the drone needs AP_DDS, fallback is pymavlink over UDP 14550).

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
