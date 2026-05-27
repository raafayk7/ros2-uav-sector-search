# Multi-Agent UAV Senior Project — ROS 2 Migration & Enhancement Assessment

*A decision-oriented survey for Raafay, May 2026. The original LUMS senior project was built on ROS Noetic + Gazebo 11 + ArduPilot SITL + MAVROS + YOLOv8. This document assesses how to rebuild the simulation cleanly on a modern ROS 2 stack on Debian Trixie, and what's worth adding on top.*

---

## 0. Executive summary — what to actually do

If you only read one paragraph: **stand up ROS 2 Jazzy in a Docker container on your Debian Trixie host, use the official `ardupilot_gz` package (ArduPilot SITL + Gazebo Harmonic + DDS) with its built-in `multiagent.launch.py`, drive vehicles through ArduPilot's native AP_DDS interface rather than MAVROS, train YOLO26 (January 2026 release) on a Roboflow dataset just like the original team did, and wrap your DGA algorithm as plain ROS 2 Python nodes**. This collapses the original four-layer stack (ROS+SITL+MAVProxy+MAVROS+Gazebo) into a much cleaner three-layer stack (ROS 2 + AP_DDS + Gazebo) and gives you a single `ros2 launch` command to bring up the whole swarm.

The rest of the document defends and elaborates that recommendation across the five assessment areas you named.

---

## 1. Reading what the original project actually did

Before recommending anything, here is the shape of the work in your thesis and legacy doc, so subsequent design decisions can be judged against it.

**Original simulation stack (Noetic era):**

- Ubuntu 20.04 in a VM, ROS Noetic, Python 3.8
- Gazebo 11 (Classic) with a 20m × 20m gridded textured arena and custom marker models
- ArduPilot SITL running ArduCopter, four instances with `--sysid` and `-I0..3` flags
- MAVProxy as the GCS, MAVLink-router for message fanout
- MAVROS bridging MAVLink topics into ROS topics
- `iris_with_roscam.launch` and the Intelligent Quads `iq_sim` repo as the simulation skeleton
- A custom-trained YOLOv8 model (Ultralytics, 150 epochs on 40 Roboflow-annotated images, ~87% confidence) running as a Python ROS node consuming the `libgazebo_ros_camera.so` plugin output
- Algorithm: a leader drone does a boustrophedon sweep, generates a probability heatmap from YOLO detections, and follower drones run **Bilinear Log-Linear (BLL)** in simulation and **Distributed Gradient Ascent (DGA)** on hardware to converge on flag sectors. Utility function is "high probability, low overlap with neighbors."
- Communication for hardware: drones on a TP-Link router, UDP broadcast, with one drone (Drone 1) doing computation on Drone 2's behalf because of telemetry constraints — they explicitly call this out as not "truly decentralized."
- Color landing: HSV mask + PID centering + descent.

**Listed future work in §5.4.4:** true decentralization, better detection models for lighting variation, optical flow / visual odometry, scaling to 5+ drones, removing GPS for the follower phase.

A modern ROS 2 rebuild can hit every one of those bullets out of the box. That's the through-line of the recommendation below.

---

## 2. ROS 2 setup on Debian Trixie

### 2.1 What's actually current in May 2026

The ROS 2 release cadence is one distro per year on May 23rd (World Turtle Day), with even years being LTS:

- **Jazzy Jalisco** — LTS, released May 2024, EOL May 2029, paired with Gazebo Harmonic. This is the *safe, well-supported* choice.
- **Kilted Kaiju** — non-LTS, May 2025, 18-month support window, paired with Gazebo Ionic.
- **Lyrical Luth** — the new LTS, just released May 23 2026. It's paired with Gazebo Jetty. Brand new — expect a few weeks of dust to settle before community tutorials catch up.
- **Rolling Ridley** — bleeding-edge, not appropriate for a learning project.

### 2.2 The Debian Trixie wrinkle

Open Robotics does not ship official ROS 2 binaries for Debian. They ship for Ubuntu (and to some degree RHEL). Debian Trixie is *very close* to Ubuntu 24.04 / 26.04 underneath but the userland and `apt` repos don't match what the ROS apt server expects. You have three viable paths:

1. **Docker container (recommended).** Pull `osrf/ros:jazzy-desktop` or build from `ros:jazzy-ros-base`, mount your code in, run Gazebo inside the container with X11/Wayland passthrough or via `rocker --x11`. This is the path of least resistance — you keep your Debian host clean, your ROS 2 environment is reproducible, and you can throw it away and rebuild in minutes. Performance is essentially native because Docker is just a process namespace. The classic Jazzy Docker guide by Automatic Addison is a good template.
2. **Distrobox with an Ubuntu 24.04 container.** Slightly more "feels native" than Docker (your `$HOME` is shared, no special X11 dance), but the trade-off is more state leaking between the container and host, which makes "blow it away and start fresh" harder.
3. **Native ROS 2 on Debian Trixie.** A community project called `rospian` (`github.com/rospian/rospian-repo` and `rospian-buildfarm`) ships unofficial ROS 2 Jazzy debs for Raspberry Pi OS / Debian Trixie arm64. There's also a Raspberry Pi Forums thread from a developer who concluded "Docker and Conda are pain, native is the way" for embedded ARM. But this is community-supported and explicitly "for experienced users." On x86_64 it's less battle-tested.

**My recommendation: Docker on Jazzy.** You want to learn ROS 2, not become a packaging expert. Save Debian-native installs for if/when you put a real companion computer on a drone.

### 2.3 Why Jazzy and not Lyrical

Lyrical is brand new as of three days ago. The `ardupilot_gz` package, the YOLO ROS 2 wrappers, Nav2 extensions, and most of the tutorials you'll find are still on Humble/Jazzy. Lyrical will be the right answer in 3–6 months; Jazzy is the right answer today, and you can migrate later because Jazzy is LTS through 2029.

### 2.4 Setup checklist (conceptual)

1. Install Docker on Trixie (`sudo apt install docker.io`, add yourself to the `docker` group)
2. Decide on a workspace layout: host folder `~/ros2_ws/src/` mounted into `/root/ros2_ws/src/` in the container
3. Build an image based on `osrf/ros:jazzy-desktop-full` with Gazebo Harmonic packages, ArduPilot SITL build deps, and the OSRF Gazebo apt source pre-added
4. Use `rocker --x11 --user` or a `docker compose` with the appropriate DRI device mounts for GPU-accelerated Gazebo rendering
5. Inside the container, `colcon build` your workspace; `source install/setup.bash`; `ros2 launch …`

The mental model shift from ROS 1 you'll need:
- `roscore` is gone. Nodes find each other via DDS discovery on the local network.
- `catkin_ws` → `ros2_ws`; `catkin build` → `colcon build`; `rosrun pkg node` → `ros2 run pkg node`
- `.launch` (XML) → `.launch.py` (Python). Launch files are now real code, which is mostly good but takes adjustment.
- Topics are typed and QoS-policy-driven (reliable vs best-effort, transient-local vs volatile). Most "my publisher works but my subscriber sees nothing" bugs in ROS 2 are QoS mismatches, not networking bugs.

---

## 3. Gazebo setup

### 3.1 What changed since Gazebo 11

The "Gazebo Classic" lineage your team used (Gazebo 1 through 11) was discontinued in January 2025. The current product is **"Gazebo"** (formerly "Ignition Gazebo"), a complete rewrite with a more modular architecture and an `gz` CLI instead of `gazebo`. Current versions:

- **Harmonic** — LTS, EOL Sept 2028, paired with ROS 2 Humble and Jazzy
- **Ionic** — non-LTS, EOL Sept 2026 (so it's already on the way out)
- **Jetty** — LTS, EOL Sept 2030, paired with ROS 2 Lyrical

Pick **Gazebo Harmonic** to pair with ROS 2 Jazzy. The pairing is "official" in the sense that the OSRF apt repo and `ros-jazzy-ros-gz` vendor packages assume Harmonic.

### 3.2 The ROS 2 ↔ Gazebo bridge

The bridge between the two ecosystems is `ros_gz` (`github.com/gazebosim/ros_gz`). Concretely:

- `ros_gz_bridge` converts messages bidirectionally (e.g. Gazebo `gz.msgs.Image` ↔ ROS `sensor_msgs/Image`)
- `ros_gz_sim` launches Gazebo from a ROS 2 launch file
- `ros_gz_image` and `ros_gz_interfaces` are convenience wrappers

In Jazzy, the recommended path is to install the **vendor packages** (`ros-jazzy-ros-gz` from packages.ros.org) rather than mixing the OSRF apt repo for Gazebo Harmonic with the ROS apt repo, because they coordinated the soname/ABI dance for you.

### 3.3 ArduPilot + Gazebo + ROS 2 — the `ardupilot_gz` package

This is the most important single tool for your rebuild. **`github.com/ArduPilot/ardupilot_gz`** is the official ArduPilot project for "ROS 2 + Gazebo + ArduPilot SITL with DDS." It contains four sub-packages:

- `ardupilot_gz_description` — SDFormat models (Iris quadcopter with camera, Iris with lidar, etc.)
- `ardupilot_gz_gazebo` — Gazebo system plugins
- `ardupilot_gz_application` — ROS 2 nodes and config
- `ardupilot_gz_bringup` — Launch files, including `iris_runway.launch.py`, `iris_maze.launch.py`, **`multiagent.launch.py`**

The multi-agent launch is the headline feature for your project. Out of the box, `multiagent.launch.py` spawns one Iris-with-camera, one Iris-with-lidar, and one ground rover (the "wild thumper"), and the README explicitly says "can be expanded to an arbitrary number of robots by changing the `robots` list in the launch file." This replaces the `iq_sim` + `sproj_sim1_mavroslink.launch` dance you had in Noetic with a single command:

```
ros2 launch ardupilot_gz_bringup multiagent.launch.py rviz:=true use_gz_tf:=true
```

The official ArduPilot wiki page (`ardupilot.org/dev/docs/ros2-gazebo.html`) currently shows Humble as the documented prerequisite, but Jazzy works — the community has been running it for months and there's an open issue tracking the doc update. The README still says "ROS 2 humble," so when you build expect to read the launch file and tweak a couple of paths.

**Note for scaling up to your 1 leader + N followers design:** the `robots` list in `multiagent.launch.py` is just a Python list of dicts. You add entries for `iris_2`, `iris_3`, etc., give each a unique MAVLink system ID and DDS namespace, and you're done. There's also a community fork, `aau-cns/Ardupilot_Multiagent_Simulation`, that adds sensor configurability and external-odometry plumbing on top of the official package — worth a look if you want optical-flow / VIO inputs for a "remove GPS for the follower phase" experiment.

### 3.4 What you gain over the Noetic setup

- No more juggling `GAZEBO_MODEL_PATH` / `GAZEBO_PLUGIN_PATH` exports — Gazebo Harmonic uses `GZ_SIM_RESOURCE_PATH` and the ROS 2 launch system sets it for you
- No more `MAVProxy` instance per drone — DDS handles discovery
- No more "open four terminals and run `sim_vehicle.py -I0`, `-I1`, `-I2`, `-I3`" — one launch file
- The new SDF format is much more readable than the old URDF/SDFormat mix, and the Iris model is already wired up with camera, IMU, GPS, and barometer sensors

### 3.5 Worlds and assets

Two of the GitHub repos in your legacy guide are still useful:

- `leonhartyao/gazebo_models_worlds_collection` is Gazebo Classic; the models *can* be ported to new-Gazebo SDF but it's not free.
- A more current resource is the Gazebo Fuel asset library (`app.gazebosim.org/fuel`), which the new Gazebo client can pull from directly.
- For a gridded arena like your original LUMS cricket-ground simulation, the easiest path is to author the SDF by hand (it's <200 lines) or generate it with a small Python script — exactly the kind of thing you can hand to me later.

---

## 4. YOLO training

### 4.1 What's current

The Ultralytics line has moved from YOLOv8 (your original) through v9, v10, v11, v12 to **YOLO26**, released January 14 2026. The relevant headline improvements for an aerial-detection use case:

- **ProgLoss + STAL (Scale-Targeted Attention Loss)** — measurable accuracy gains on small objects, which is the regime you're in (markers seen from 7m altitude in a 640×640 image are small).
- **Better CPU inference** — important if you ever push the model onto an embedded board without GPU (Pi, Odroid, low-power Jetson Nano).
- Same Ultralytics CLI/API as YOLOv8, so the training muscle memory carries over.

For your project specifically: your team trained on **40 images and got 87% confidence**. That's a *very* small dataset and you got lucky because the marker had a distinctive texture and the Gazebo environment was deterministic. With YOLO26 + synthetic data augmentation in Gazebo (different lighting, marker rotations, different terrain textures via the new Gazebo "physics-based rendering" materials), you should be able to get to 95%+ with a dataset under 200 images.

### 4.2 Recommended training pipeline

1. **Data collection in sim** — Headless Gazebo script that spawns the marker at random poses, captures the Iris camera topic, and saves images. 200–500 images is plenty.
2. **Annotation** — Roboflow worked fine for your team; it still works fine. Alternative: CVAT or Label Studio if you'd rather self-host. For known synthetic data you can also auto-label from ground truth (Gazebo knows where the marker is).
3. **Augmentation** — Roboflow's built-in or `albumentations`; rotation, brightness, weather effects.
4. **Training** — `yolo train data=marker.yaml model=yolo26n.pt epochs=150 imgsz=640`. The `n` (nano) variant is fast enough to run real-time on a laptop CPU; `s` or `m` if you have a GPU available.
5. **Inference in ROS 2** — Wrap with one of the existing ROS 2 nodes:
   - `mgonzs13/yolo_ros` — supports YOLOv8 through v12, well-maintained, has detection / segmentation / pose / OBB modes. As of mid-2026 expect YOLO26 support either landed or in a PR.
   - `Alpaca-zip/ultralytics_ros` — Ultralytics-blessed wrapper, simpler.
   - Roll your own — for the simple "subscribe to image, publish detections" case, the Ultralytics Python API + `cv_bridge` is ~50 lines.

### 4.3 What's worth doing differently from the original

- **Tighter messages**: instead of publishing raw detections, publish a `vision_msgs/Detection2DArray` (standard ROS 2 type) with class IDs, bounding boxes, and confidence. This makes your detection node interoperable with other ROS 2 vision tooling.
- **Compress vision pipeline**: in Noetic your team ran YOLO and the algorithm in the same Python process. In ROS 2, split them — perception is its own node, behavior/algorithm is another, and they communicate via the typed topic. Easier to swap implementations and easier to scale to 5+ drones.

### 4.4 Bonus: zero-shot detection with VLMs

A trend worth knowing about even if you don't adopt it for the first pass: vision-language models (CLIP-derived, plus larger frontier models like Gemini Flash 2.5, GPT-4o, LLaMA-4) are now doing meaningful zero-shot object detection on aerial imagery. Recent 2026 papers benchmark them for UAV SAR. The relevant insight: for the *next* version of your project where the target is "any human" or "any backpack" rather than "this specific textured marker," a VLM-based detector lets you skip the entire training pipeline at the cost of latency and edge-deployability. Useful trajectory to mention in any "future work" pitch.

---

## 5. Drone harness / algorithm — the MAVROS-vs-AP_DDS decision

This is the single biggest architectural improvement available to you over the original project.

### 5.1 The three options for talking to ArduPilot from Python

| Option | What it is | When you want it |
|---|---|---|
| **pymavlink** | Pure-Python MAVLink library. You open a UDP socket, send `COMMAND_LONG` and `SET_POSITION_TARGET_LOCAL_NED` messages directly. | Quick scripts, no ROS needed, learning MAVLink. |
| **MAVROS** | ROS 2 package that runs as a node, exposes MAVLink as ROS topics/services. | Bridging to existing ROS code, multi-autopilot support (PX4 + ArduPilot). |
| **AP_DDS** (native) | ArduPilot 4.5+ has a built-in DDS client (Micro XRCE-DDS) that publishes ROS 2 topics natively. No bridge process. | ArduPilot 4.5+ on ROS 2 — the clean modern path. |

### 5.2 What AP_DDS gives you

When you compile and run ArduPilot SITL with `--enable-DDS` (or boot a hardware autopilot with `DDS_ENABLE=1`), it speaks DDS directly. Inside ROS 2 you see topics like:

```
/ap/battery/battery0
/ap/navsat/navsat0
/ap/clock
/ap/tf_static
/iris/odometry
/ap/goal_pose          (set this to command the vehicle)
/ap/cmd_vel            (velocity setpoints)
```

This is a *huge* simplification compared to MAVROS:
- No MAVProxy needed
- No `fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2 tgt_system:=2` incantations per drone
- Topics are namespaced cleanly by vehicle
- Latency is lower (no intermediate process)
- It's how the ROS 2 ecosystem expects vehicles to look (PX4 has been doing this with its uXRCE-DDS bridge for a couple of years; ArduPilot followed)

### 5.3 The trade-off

AP_DDS is newer than MAVROS. It covers the common cases (state, telemetry, position setpoints, mode changes, arm/disarm) but doesn't yet expose every MAVLink message that MAVROS does. If your algorithm only needs:

- Read pose / GPS / battery
- Send goto / velocity commands
- Switch modes (GUIDED, LAND, RTL)
- Arm / disarm

…then AP_DDS is complete. If you need something exotic (Lua scripting upload, parameter set/get for non-standard params, mission upload), you may have to drop down to MAVROS or pymavlink for that one task.

**My recommendation:** start with AP_DDS via `ardupilot_gz`. It already wires it up. If you hit a wall on a specific command, run MAVROS as a *second* bridge on the side for that one thing — they don't conflict.

### 5.4 Mapping your algorithm onto ROS 2 nodes

The Distributed Gradient Ascent algorithm in your thesis is small (a few hundred lines of Python). Here's how I'd carve it up in ROS 2:

- **`perception_node`** (one per drone) — subscribes to `/iris_N/camera`, publishes `/iris_N/detections` (Detection2DArray). Wraps YOLO26.
- **`localization_node`** (one per drone) — subscribes to `/iris_N/ap/navsat`, `/iris_N/ap/imu`, publishes `/iris_N/pose` in the arena frame.
- **`leader_node`** (one, only on the leader) — runs the boustrophedon path, subscribes to its own detections, builds a `/heatmap` topic (a `nav_msgs/OccupancyGrid` is the idiomatic type for this).
- **`follower_planner_node`** (one per follower) — subscribes to `/heatmap` and `/iris_K/pose` for K ≠ self, runs the DGA utility computation, publishes `/iris_self/goal_pose` to AP_DDS.
- **`landing_node`** (one per follower) — switches on when the follower is within X meters of a target; replaces the goal_pose stream with a color-masked centering + descent controller.

This is the architecture your thesis Section 5.4.4 wanted: each drone runs its own planner, no central "Drone 1 thinks for Drone 2" hack, decentralization by construction. The reason the original team had to centralize was hardware limits (battery, Wi-Fi range, telemetry constraints) — in simulation those constraints don't exist, so simulation is exactly the right place to prove out the truly decentralized version.

### 5.5 Inter-drone communication

In Noetic + UDP, your team built their own broadcast layer. In ROS 2, **DDS is already the broadcast layer.** If two `follower_planner_node` instances on the same network publish their pose on `/iris_N/pose`, every other follower's planner sees it automatically. No router config, no port assignments. This is one of the headline benefits of ROS 2 over ROS 1, and it's especially valuable for swarms.

The one thing to watch: in production swarms you eventually want to tune the DDS QoS (history depth, reliability) and possibly use a custom RMW like Cyclone DDS for predictability. For the simulation phase, defaults are fine.

### 5.6 Algorithm enhancements worth considering

Sticking to a "pure ROS 2 fresh design" framing, here are upgrades that compose naturally with the architecture above:

- **Behavior trees instead of state machines.** `BehaviorTree.CPP` (the engine Nav2 uses) makes the "search → detect → converge → land" sequence visualizable and tunable in XML, instead of buried in Python. Worth doing once you have the basic flow working.
- **Nav2 for the follower goto.** Nav2 was designed for ground robots but is increasingly used for drones via custom planners. Overkill for the first cut, but the framework gives you proper recovery behaviors (what to do when goto fails, when you lose the heatmap topic, etc.).
- **Coverage path planning library.** The boustrophedon sweep is fine for a square arena, but for non-convex search regions look at the `coverage_path_planning` ROS 2 packages or the algorithms in the 2026 *Drones* journal review (`doi.org/10.3390/drones10010011`).
- **PX4 as a future option.** PX4 has *tighter* ROS 2 integration than ArduPilot — you can write a flight mode in ROS 2 that's indistinguishable from a native one. ArduPilot's hardware support and ground-vehicle community are stronger, so it remains the right pick for matching your existing thesis. But keep PX4 in your back pocket for the "next senior project."

---

## 6. Simulation as a whole — choosing the simulator

You'll see Gazebo, Webots, and Isaac Sim debated online. For your specific use case:

- **Gazebo Harmonic — recommended.** Ardupilot's official ROS 2 integration is built around it. Multi-agent up to ~20 drones runs fine on a modern laptop. Photoreal-enough textures for YOLO training.
- **Webots — viable alternative.** Slightly easier multi-agent CPU profile (one independent study found Webots stays performant past 20 agents where Gazebo degrades). Less ArduPilot integration — you'd be building the SITL bridge yourself.
- **NVIDIA Isaac Sim — overkill but tempting.** Open-sourced in 2025 (Isaac Sim 5.0), photorealistic RTX ray-traced rendering, GPU-accelerated PhysX, ROS 2 bridge. Great for training detection models with truly realistic visuals and for swarms above 50 agents. Steep learning curve and high VRAM requirements (12GB+). Reserve for "phase 2."

For your project's "recreate the simulation, learn ROS 2" framing, **stay with Gazebo Harmonic** — the integration story is the strongest, the docs match what you're trying to do, and you'll spend your time on the interesting parts (the algorithm and the perception) rather than on building infrastructure.

### 6.1 Sim-to-real notes

Your thesis hit the classic sim-to-real wall: the simulation worked, the field test had GPS jitter and the follower drones landed 3.5–5 m off-marker (Table 4 in your thesis). Things that help in a rebuild:

- Train YOLO on a mix of sim images and a handful of real images (Roboflow makes this easy)
- Add noise models to the Gazebo IMU/GPS plugins to match real noise variance
- Tune the simulation's wind / drag to roughly match field conditions
- Use **EKF3's external-odometry input** + visual-inertial odometry from RealSense for the follower phase — this is exactly the "remove GPS" item from your future work, and `ardupilot_gz` has hooks for it

---

## 7. Walking through the links in your project instructions and guide

For completeness, here's what each reference URL in your instructions and legacy doc gives you and whether it's still relevant:

**From your project instructions:**

| URL | What it is | Still relevant? |
|---|---|---|
| `github.com/ArduPilot/pymavlink` | Pure-Python MAVLink lib | Yes — useful for ad-hoc scripts; in the ROS 2 design AP_DDS supersedes it for the main control loop. |
| `github.com/ArduPilot/MAVProxy` | Reference MAVLink GCS | Yes for debugging; not in the runtime path under AP_DDS. |
| `mavlink.io/en/messages/common.html` | MAVLink message dictionary | Yes — reference when you need to know what's in a `GLOBAL_POSITION_INT` etc. |
| `ardusub.com/developers/pymavlink.html` | ArduSub's pymavlink tutorial | Concepts transfer to Copter; commands differ. Skim, don't follow line-by-line. |
| `ardupilot.org/dev/docs/copter-commands-in-guided-mode.html` | The authoritative list of guided-mode commands | **Yes, essential.** This is the ground truth for what you can ask the drone to do. Whether you send these via pymavlink, MAVROS, or AP_DDS, the semantics are the same. |
| `wiki.ros.org/mavros` | MAVROS docs (ROS 1 wiki) | Outdated as your primary reference. Use `docs.ros.org/en/jazzy/p/mavros/` instead for the ROS 2 version. |

**From your legacy guide:**

| URL | What it is | Still relevant under ROS 2? |
|---|---|---|
| `github.com/r0ch1n/ardupilot_gazebo_roscam` | Noetic-era single-UAV camera plugin | No — `ardupilot_gz` replaces this entirely. |
| `github.com/monemati/multiuav-gazebo-simulation` | Noetic multi-UAV setup | No — `multiagent.launch.py` in `ardupilot_gz` replaces this. |
| `github.com/leonhartyao/gazebo_models_worlds_collection` | Gazebo Classic worlds | Partially — models need porting to new SDF format. Gazebo Fuel is the modern equivalent. |
| `github.com/Intelligent-Quads/iq_sim` | Noetic IQ tutorials | Conceptually useful for understanding the original design; not used in the ROS 2 rebuild. |
| IQ YouTube tutorials | Walkthroughs of the above | Worth watching once for context, then move on. |

---

## 8. Concrete next steps (in order)

If you want a step-by-step roadmap to actually start building:

1. **Get Docker + ROS 2 Jazzy running on Trixie.** Pull `osrf/ros:jazzy-desktop-full`, verify `ros2 topic list` works inside the container, get RViz to display on your host.
2. **Run a turtlesim demo.** Just `ros2 run turtlesim turtlesim_node` to verify the loop and X11 forwarding. Trivial but proves the stack.
3. **Install Gazebo Harmonic and the ros_gz vendor packages.** `ros2 launch ros_gz_sim gz_sim.launch.py` should bring up an empty Gazebo world.
4. **Clone `ardupilot_gz` and run `iris_runway.launch.py`.** Single drone, single command. You'll see Gazebo, RViz, the Iris, and `ros2 topic list` will show `/ap/*` topics. **This is the moment ROS 2 clicks.**
5. **Send your first guided-mode command** to the drone over AP_DDS (publish to `/ap/cmd_vel` or set a goal via `/ap/goal_pose`).
6. **Switch to `multiagent.launch.py`.** Expand the `robots` list to 1 leader + 3 followers. Verify each drone has its own namespace.
7. **Wrap your DGA Python code as a `follower_planner_node`.** Initially just have it print the action it would take. Once that's right, hook it up to publish setpoints.
8. **Build a gridded arena world.** Either port the Gazebo Classic one from your team's repo or author it fresh — ~150 lines of SDF.
9. **Train YOLO26 on screenshots from the new arena.** Same pipeline as the original team, slightly more images for safety.
10. **Wire YOLO into a `perception_node`** and connect detections to the leader's heatmap builder.
11. **Profile.** What's the perception → planner → setpoint loop latency? What's CPU usage with 4 drones? With 8?
12. **Iterate on the algorithm.** Try true decentralized DGA (no shared planner), try replacing BLLL with a more modern multi-armed-bandit approach, try behavior trees for the leader's mission structure.

Steps 1–4 are the "learn ROS 2" milestone. Steps 5–7 are "recreate the original simulation." Steps 8–12 are "extend and improve."

---

## 9. Where AI agents help — and where they don't

Since this is partly a "learn ROS 2 by leveraging AI" project, a note on what to delegate vs. own:

**Good for delegation to AI:**
- Writing the SDF world files (tedious, mechanical)
- Writing `colcon`/CMake/`setup.py` boilerplate for new ROS 2 packages
- Translating your existing Python algorithm into idiomatic ROS 2 node patterns (publishers, subscribers, services, parameters)
- Generating `launch.py` files
- Writing unit tests
- Reading Gazebo / ROS 2 stack traces and explaining what's wrong

**You should own:**
- Understanding the algorithm and why it works
- Choosing the architecture (which nodes exist, what they publish)
- Understanding the DDS / QoS model — debugging discovery problems is much faster when you actually know how it works
- Tuning ArduPilot parameters
- The ROS 2 mental model itself — there's no substitute for doing the first few tutorials by hand

A pragmatic rhythm: hand AI the "write me a `geometry_msgs/PoseStamped` subscriber that calls `update_position()`" tasks, but read every generated launch file and SDF file yourself line by line until it feels natural. ROS 2 reveals its abstractions slowly, and the only way through is to engage with them directly the first ten times.

---

## 10. Things that would deepen the project for a publication-quality outcome

If "expand upon and improve" leans toward a paper-worthy result, these are the most promising directions:

1. **True decentralization with formal communication failure modeling.** Your thesis acknowledges the Drone-1-computes-for-Drone-2 hack as a limitation. Closing that with a real distributed DGA implementation in ROS 2, then *intentionally injecting communication failures in simulation*, would directly extend the body of work cited in your literature review (Jaleel et al. on distributed control under adversarial conditions).
2. **GPS-denied follower phase.** Your section 5.4.4 calls this out. With `ardupilot_gz` and the `external_odom` integration available in the community fork, you can run the follower phase on VIO from a simulated D435i. This is also a strong fit for a workshop paper.
3. **Heterogeneous swarm.** Your project is currently homogeneous followers. A natural extension: one leader with a longer-range camera, mid-tier scouts, low-tier landers — each with different utility functions. The behavior tree framework above makes that practical.
4. **Sim-to-real domain randomization for YOLO.** Train one detector that works on both Gazebo-rendered markers and physical markers under varied lighting. The 2026 YOLO26 paper notes specific gains on this; replicating those numbers on your marker would be a clean contribution.
5. **VLM fallback detector.** Wire a Gemini-Flash-class VLM as a slow "what is this thing the YOLO confidence is borderline on?" oracle. Mostly speculative, but a hot research direction for resilient SAR perception.

---

## Sources

ROS 2 distributions and roadmap:
- [ROS 2 Distributions — Jazzy docs](https://docs.ros.org/en/jazzy/Releases.html)
- [ROS 2 Release Schedule](https://docs.ros.org/en/jazzy/The-ROS2-Project/Release-Schedule.html)
- [ROS 2 endoflife.date](https://endoflife.date/ros-2)
- [Kilted Kaiju Installation](https://docs.ros.org/en/kilted/Installation.html)

ROS 2 on Debian Trixie:
- [Ubuntu (deb packages) — Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Native ROS 2 Jazzy Debian packages for Trixie (arm64) — Open Robotics Discourse](https://discourse.openrobotics.org/t/native-ros-2-jazzy-debian-packages-for-raspberry-pi-os-debian-trixie-arm64/51965)
- [rospian-repo (GitHub)](https://github.com/rospian/rospian-repo)
- [Automatic Addison — Complete Guide to Docker for ROS 2 Jazzy](https://automaticaddison.com/the-complete-guide-to-docker-for-ros-2-jazzy-projects/)

Gazebo + ROS 2:
- [Gazebo Jetty Releases page](https://gazebosim.org/docs/latest/releases/)
- [Installing Gazebo with ROS](https://gazebosim.org/docs/latest/ros_installation/)
- [ros_gz repo](https://github.com/gazebosim/ros_gz)
- [ROS 2 Gazebo vendor packages](https://gazebosim.org/docs/latest/ros2_gz_vendor_pkgs/)

ArduPilot ROS 2 integration:
- [ardupilot_gz repository](https://github.com/ArduPilot/ardupilot_gz)
- [ardupilot_gz README](https://github.com/ArduPilot/ardupilot_gz/blob/main/README.md)
- [ROS 2 with Gazebo — ArduPilot docs](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ROS 2 — ArduPilot docs](https://ardupilot.org/dev/docs/ros2.html)
- [ROS 2 Interfaces — ArduPilot docs](https://ardupilot.org/dev/docs/ros2-interfaces.html)
- [AP_DDS README](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md)
- [ArduPilot native ROS 2 support — discuss.ardupilot.org](https://discuss.ardupilot.org/t/ardupilot-native-ros2-support/73720)
- [Ardupilot_Multiagent_Simulation (community fork)](https://github.com/aau-cns/Ardupilot_Multiagent_Simulation)

MAVROS for ROS 2:
- [mavros — Jazzy docs](https://docs.ros.org/en/jazzy/p/mavros/)
- [mavros ros2 branch README](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md)
- [MAVROS vs DDS bridge — discuss.ardupilot.org](https://discuss.ardupilot.org/t/mavros-vs-dds-ros-bridge-updates/110878)

YOLO and perception:
- [YOLO26 docs (Ultralytics)](https://docs.ultralytics.com/models/yolo26)
- [YOLO26 vs YOLO11](https://docs.ultralytics.com/compare/yolo26-vs-yolo11)
- [Deploying YOLO26 Using ROS 2 — Mike Likes Robots](https://mikelikesrobots.github.io/blog/ultralytics-yolo26-computer-vision/)
- [Ultralytics ROS quickstart](https://docs.ultralytics.com/guides/ros-quickstart/)
- [yolo_ros (mgonzs13)](https://github.com/mgonzs13/yolo_ros)
- [ultralytics_ros (Alpaca-zip)](https://github.com/Alpaca-zip/ultralytics_ros)
- [Enhanced YOLOv11n for small object detection in UAV imagery — Nature Sci Reports 2026](https://www.nature.com/articles/s41598-026-35301-2)

Swarms, planning, and related work:
- [Distributed coverage path planning for UAV swarms — ScienceDirect 2026](https://www.sciencedirect.com/science/article/pii/S0957417426012959)
- [Path-planning algorithms for multi-UAV swarms — Drones journal 2026](https://doi.org/10.3390/drones10010011)
- [ROS2swarm package paper (arXiv)](https://arxiv.org/pdf/2405.02438)
- [Looking for a Nav2-equivalent for multi-UAV — Dronecode Forum](https://discuss.px4.io/t/looking-for-a-nav2-equivalent-library-ugv-for-multi-uav-mission-management-state-machine-behavior-tree/46745)
- [VLM-Nav: Mapless UAV navigation using VLMs (PLOS One)](https://journals.plos.org/plosone/article?id=10.1371%2Fjournal.pone.0345778)
- [SkySim: ROS2-based simulation for natural-language drone swarm control (arXiv)](https://arxiv.org/pdf/2602.01226)

Simulator comparisons:
- [Gazebo vs Webots vs Isaac Sim — Partenit i-Hub](https://i.partenit.io/knowledge-base/gazebo-vs-webots-vs-isaac-sim/)
- [Robot Simulation Software: 2026 Perspective — Black Coffee Robotics](https://www.blackcoffeerobotics.com/blog/which-robot-simulation-software-to-use)

Reference docs that haven't moved:
- [MAVLink common messages](https://mavlink.io/en/messages/common.html)
- [ArduPilot Copter commands in guided mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [pymavlink repo](https://github.com/ArduPilot/pymavlink)
- [MAVProxy repo](https://github.com/ArduPilot/MAVProxy)
