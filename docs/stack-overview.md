# Stack overview & glossary

A plain-language orientation to the project: what the system does, what each
layer of the tech stack is for, and the key terms — grounded in what's
actually been built so far rather than an abstract glossary.

This is the *learner's* companion. For the full rationale behind each stack
choice (the trade-offs, the alternatives rejected), see
`assessment-ros2-migration.md` — that's the source of truth for *why*. For
build-environment gotchas, see `docker-notes.md`.

Last updated: 2026-11-04.

---

## What the system is meant to do

A **leader** drone flies a sweep over a search area, runs computer vision to
spot ground markers, and builds a probability map of where things are.
**Follower** drones then use a distributed algorithm to spread out and
converge on the high-probability sectors, and finally land precisely on a
marker using color tracking. It's the ROS 2 rebuild of the LUMS senior
project.

Everything in the system reasons about *where each drone is* in a shared
map — which is why the localization node is the foundation everything else
stacks on.

## Where the project is right now

- The simulation environment stands up fully: Gazebo + the ArduPilot flight
  controller + the ROS 2 bridge.
- **Two project nodes exist and are verified.** `uav_localization/
  localization_node` turns raw GPS into a clean local position (`/pose`) in a
  project-defined map frame. `uav_mapping/leader_node` — the first
  algorithm-relevant node — turns the leader's camera detections into a
  probability heatmap (`/heatmap`) over the thesis sector grid.
- The **follower planner is still ahead** — that's the next node
  (`follower_planner_node`, consuming `/heatmap`). Its design is settled in
  `planner-decisions.md`.
- The Gazebo↔SITL FDM handshake works as of 2026-11-02 (`/ap/v1/*` is live and
  the drone accepts arm/takeoff; see `docker-notes.md`), so actuation is no
  longer blocked. The planner can still be validated first as a coordination
  simulation without flight.

---

## The stack, layer by layer

| Layer | Choice | What it does for us |
|---|---|---|
| Host OS | Ubuntu 24.04 (was Debian Trixie) | The primary machine. Docker keeps the ROS 2 toolchain reproducible regardless of host. |
| Containment | Docker | Packages the entire ROS 2 + Gazebo + ArduPilot toolchain into one reproducible image, so we don't fight a native install on an unsupported OS. |
| Framework | ROS 2 Jazzy | The robotics framework: node model, pub/sub messaging, standard message types, build tools, launch system. Modern replacement for ROS 1 Noetic; LTS to 2029. |
| Middleware | DDS | The transport ROS 2 runs on. Handles node discovery and moving data between processes. The reason QoS settings exist. |
| Simulator | Gazebo Harmonic | Simulates the drone body, the world, gravity, and onboard sensors (GPS, IMU, camera). Official pairing for Jazzy. |
| Flight controller | ArduPilot SITL | The real autopilot firmware (`arducopter`) running as a desktop process instead of on a Pixhawk. Runs the same control loops a real drone would. |
| Harness | ardupilot_gz | Glue package that launches Gazebo and SITL together and wires them up. Ships `iris_runway.launch.py`. |
| ROS ↔ autopilot | AP_DDS | ArduPilot's native bridge: exposes the autopilot to ROS 2 as `/ap/vN/*` topics and services. Replaces the older MAVROS. |
| Sensor bridge | ros_gz_bridge | Bridges Gazebo's sensor topics into ROS 2 (`/navsat`, `/odometry`, `/camera/image`, …). |
| Perception | YOLO26 | Computer-vision marker detection. Not built yet. |
| Algorithm | Best open sector, with thesis DGA / BLL as baselines | The distributed coordination planner. Each follower picks the best open sector on the shared map. Designed, not built. |

### How the simulation pieces fit together

Gazebo and SITL are two separate programs that have to agree on reality.
They exchange **FDM** (Flight Dynamics Model) packets over UDP: Gazebo sends
the simulated physics/sensor state, SITL sends back motor commands. Once
that loop is alive, SITL starts **AP_DDS**, which publishes the autopilot's
state and accepts commands as ROS 2 `/ap/*` topics.

As of 2026-11-02 that handshake works: AP_DDS initializes and the `/ap/v1/*`
topics and services are live. **ros_gz_bridge** also publishes Gazebo's
sensors straight into ROS 2 (`/navsat`, `/odometry`, `/camera/image`, etc.),
which is what the sensing and planning nodes use as inputs today.

---

## Session 2026-06-03: the localization node

The `uav_localization` package, containing `localization_node`, which:

- subscribes to `/navsat` (GPS) and `/odometry` (Gazebo pose),
- converts GPS latitude/longitude into local East/North/Up meters,
- publishes `/pose` (`geometry_msgs/PoseStamped`) in a project-defined
  **arena frame**.

The architecture decisions made consciously (not by default):

- **Arena frame = ENU, origin at the first GPS fix.** `(0,0,0)` means
  "where the drone started." REP-103 compliant.
- **Position from GPS projection, orientation from odometry.** Position
  comes from the honest GPS pipeline (what a real drone has); orientation is
  taken from odometry as a stand-in for the real attitude estimate.
- **`/pose` published RELIABLE, depth 10.** Pose is low-rate state, so
  guaranteed delivery is cheap and a dropped pose could perturb a planner.

Verified in sim: origin latched onto ArduPilot's stock CMAC home, pose read
`(0,0,0)` at spawn, and orientation read a 90° yaw — correct, because the
drone faces North and "North" is +90° from East in an ENU frame.

A build blocker was also fixed: `setuptools>=80` (pulled in by
torch/ultralytics) breaks `colcon build --symlink-install`. Pinned to
`<80` in the Dockerfile; full notes in `docker-notes.md` gotcha #17.

---

## Session 2026-06-10: the leader heatmap node

The `uav_mapping` package, containing `leader_node` — the first
algorithm-relevant node. It is the leader's **perception-to-map** step: it
turns camera detections into a probability map the follower planners will
later consume.

What it does, per detection: take the pixel of a detected marker, back-project
it through the camera's pinhole model into a ray, rotate that ray into the
arena frame, intersect it with the flat ground (z = 0) to get a world (x, y),
find which sector that point falls in, and nudge that sector's probability.

The decisions made consciously this session:

- **Map = `nav_msgs/OccupancyGrid`, one cell == one thesis sector.** The
  message's `0–100`/`-1` cells line up exactly with the thesis's per-sector
  probability `p_ij ∈ [0,1]`, so the heatmap message *is* the probability map.
  Defaults match the thesis (5 m sectors, 4×4 over the 20×20 m arena, with
  the grid corner at home like the thesis leader's takeoff corner) and are all
  launch params, so the grid reshapes without code edits.
- **Camera pose from `/pose` ∘ a static down-facing mount**, not the
  simulator's transform tree — so the node stays inside our own localization
  pipeline and isn't coupled to sim-specific frame names. Intrinsics are read
  live from `/camera/camera_info`.
- **Probability via Bayesian log-odds.** Each detection raises a cell's
  log-odds (scaled by the detector's confidence); cells the camera looked at
  but saw nothing in decay toward empty; values are clamped so no cell ever
  pins to absolute certainty. Cells never observed stay unknown (`-1`). This
  is what makes it a real *search* map, not just a detection tally.
- **`/heatmap` published RELIABLE + TRANSIENT_LOCAL** so a follower planner
  that subscribes *after* the leader has been mapping still immediately
  receives the current map.

It ships with `fake_detection_publisher` (a stub that emits a synthetic
detection at a known pixel, so the node could be tested before the real
perception node exists) and `heatmap.launch.py`.

Verified standalone: a drone posed at `(7.5, 7.5, 7.0)` looking straight down,
fed a detection at the image center, lit up exactly the expected sector cell
`(1,1)` at world `(7.5, 7.5)` — matching a hand computation. The projection
math was also checked offline against four hand-worked cases before building.
A ROS gotcha surfaced and was fixed: `use_sim_time=true` with no simulator
running freezes every node timer (sim time never advances without `/clock`),
so the launch now forces sim time off in standalone mode. **Still pending:** a
run against the live `iris_runway` camera and confirming the real iris camera
mount (the down-facing assumption is a parameter, not code).

---

## Session 2026-11-04: warm-up and planner design review

No planner code yet; this session settled the design. What changed in your
picture of the system:

- **Followers don't climb a gradient anymore.** The leader's map has isolated
  spikes (97% next to 3%), so stepping to the best neighbour finds nothing. Each
  follower now picks the best open sector anywhere and flies straight there. The
  thesis algorithms stay as measured baselines.
- **The camera isn't pointing down.** It sits on a gimbal, 52° off vertical, so
  getting it to nadir is a required task before footprint work.
- **Commands reach the autopilot on `/ap/vN/cmd_gps_pose`**, and it silently
  ignores anything without `frame_id: "map"`.

Decisions are in `planner-decisions.md`; environment facts are gotchas 24-30 in
`docker-notes.md`.

---

## Key terminology

**Node** — one ROS 2 process that does one job. `localization_node` is the
first.

**Topic / publish / subscribe** — named channels nodes use to exchange
messages. A node never calls another directly; it emits messages anyone can
listen to. `localization_node` *subscribes* to `/navsat` and `/odometry`
and *publishes* to `/pose`.

**Message type** — the schema of what flows on a topic. Used so far:
`sensor_msgs/NavSatFix` (GPS in), `nav_msgs/Odometry` (pose+velocity in),
`geometry_msgs/PoseStamped` (position+orientation+timestamp out).

**ENU / REP-103** — the ROS coordinate convention: x=East, y=North, z=Up.
REP-103 is the standard that mandates it. ArduPilot internally uses NED
(North-East-Down); we convert at the ROS boundary so everything downstream
stays in standard orientation.

**Arena frame / origin (datum)** — the project's fixed map frame: ENU,
anchored at the drone's first GPS fix.

**Equirectangular projection** — the flat-earth math that turns GPS
lat/lon differences into local meters. Accurate to well under a meter at
arena scale; swap in a geodetic library only for much larger areas.

**QoS (Quality of Service)** — DDS's per-topic settings: *reliability*
(RELIABLE retransmits dropped messages; BEST_EFFORT doesn't) and
*history/depth* (how many messages to buffer). Key rule: a BEST_EFFORT
*subscriber* connects to any publisher, which sidesteps the silent
QoS-mismatch that is the most common ROS 2 bug.

**use_sim_time / `/clock`** — when set, nodes take time from Gazebo's
simulated clock instead of the wall clock, so timestamps line up across the
sim.

**ament_python** — the build type for a pure-Python ROS 2 package. Three
special files: `package.xml` (manifest + dependencies), `setup.py`
(installer + the entry point mapping the `localization_node` executable to
`main()`), and `setup.cfg` (puts the executable where `ros2 run` looks). The
empty `resource/` marker registers the package in the **ament index** so
ROS tools can find it.

**colcon / `--symlink-install`** — colcon is the ROS 2 build orchestrator.
`--symlink-install` symlinks Python sources into the install tree so you can
edit `.py` files without rebuilding (the feature setuptools 80 broke).

**Launch file / launch args / namespace** — a launch file starts nodes with
configured parameters. The **namespace** is how one node becomes
`/iris_1/...` vs `/iris_2/...` per drone — the reason the output topic is
*relative* (auto-namespaces) and the inputs are *absolute* (don't).

**SITL** — Software In The Loop: real firmware running as a desktop process
rather than on flight hardware.

**FDM** — Flight Dynamics Model: the physics-state packets exchanged between
Gazebo and SITL over UDP.

**AP_DDS / MAVROS / MAVLink** — AP_DDS is ArduPilot's native ROS 2 bridge
(`/ap/*` topics). MAVROS is the older bridge it replaces; MAVLink is the
underlying autopilot messaging protocol (used by MAVProxy for debugging).

**ros_gz_bridge** — bridges Gazebo transport topics to ROS 2 topics; the
current source of `/navsat`, `/odometry`, `/camera/image`, etc.

**OccupancyGrid** — ROS's standard 2D map message (`nav_msgs/OccupancyGrid`):
a flat array of cells over a rectangle of the world, with metadata for
`resolution` (m/cell), `width`/`height`, and an `origin` pose. Each cell is
`0–100` (percent probability) or `-1` (unknown). The heatmap uses it; one cell
equals one thesis sector.

**Timer / executor** — a timer calls a function at a fixed rate, and with
`use_sim_time` it follows the simulator's clock. The executor is the loop that
runs a node's callbacks; the default runs them one at a time, so a node with one
timer needs no locks.

**RMW** — the messaging library underneath ROS 2 (CycloneDDS or Fast DDS here).
Topics work across both; `/ap/*` services hang under CycloneDDS, so the arming
helper uses Fast DDS.

**Sector grid (m × n)** — the thesis's partition of the arena into square
sectors, each carrying a probability `p_ij` of holding a marker. The rebuild
maps it 1:1 onto OccupancyGrid cells: 5 m sectors, 4×4 over the 20×20 m arena.

**Detection2DArray** — `vision_msgs/Detection2DArray`, the standard message
for a frame's worth of 2D detections (bounding boxes + class/score
hypotheses). The contract between `perception_node` (producer) and
`leader_node` (consumer).

**Pinhole projection** — the camera model that maps a 3D direction to a pixel
via the intrinsics matrix `K` (`/camera/camera_info`). Run backwards, a pixel
becomes a ray; intersecting that ray with the ground plane gives the world
point a detection sits on.

**tf2** — ROS's coordinate-frame bookkeeper: nodes broadcast time-stamped
transforms between frames and tf2 answers "where is this point, expressed in
that frame?". `leader_node` deliberately does *not* use it — it composes the
camera pose from `/pose` and a fixed mount instead, to stay independent of
sim-specific frames — but it's the standard tool when you must bridge frames.

**Log-odds / inverse sensor model** — the occupancy-grid way of accumulating
evidence. Each cell stores log-odds `l` of "marker present"; `p = 1/(1+e^-l)`.
An observation adds a fixed increment (positive for a detection, negative for
an empty look), clamped so belief never saturates. Robust to repeated and
conflicting observations.

**TRANSIENT_LOCAL (QoS durability)** — a publisher setting that latches the
last message so subscribers joining *after* it was sent still receive it. Used
for `/heatmap` so a late-starting follower planner gets the current map at
once instead of waiting for the next publish.
