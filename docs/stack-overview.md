# Stack overview & glossary

A plain-language orientation to the project: what the system does, what each
layer of the tech stack is for, and the key terms — grounded in what's
actually been built so far rather than an abstract glossary.

This is the *learner's* companion. For the full rationale behind each stack
choice (the trade-offs, the alternatives rejected), see
`assessment-ros2-migration.md` — that's the source of truth for *why*. For
build-environment gotchas, see `docker-notes.md`.

Last updated: 2026-06-03.

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
- The **first project node** exists and is verified: `uav_localization/
  localization_node`, which turns raw GPS into a clean local position in a
  project-defined map frame.
- No algorithm code yet (the DGA/BLL planners are still ahead).
- One known blocker deferred: the Gazebo↔SITL FDM handshake is silent, so
  the drone can't be commanded to fly yet (details in `docker-notes.md`).
  Development proceeds against the sensor bridge in the meantime.

---

## The stack, layer by layer

| Layer | Choice | What it does for us |
|---|---|---|
| Host OS | Debian Trixie | The primary machine. ROS 2 targets Ubuntu, not Debian — which is the whole reason the next layer exists. |
| Containment | Docker | Packages the entire ROS 2 + Gazebo + ArduPilot toolchain into one reproducible image, so we don't fight a native install on an unsupported OS. |
| Framework | ROS 2 Jazzy | The robotics framework: node model, pub/sub messaging, standard message types, build tools, launch system. Modern replacement for ROS 1 Noetic; LTS to 2029. |
| Middleware | DDS | The transport ROS 2 runs on. Handles node discovery and moving data between processes. The reason QoS settings exist. |
| Simulator | Gazebo Harmonic | Simulates the drone body, the world, gravity, and onboard sensors (GPS, IMU, camera). Official pairing for Jazzy. |
| Flight controller | ArduPilot SITL | The real autopilot firmware (`arducopter`) running as a desktop process instead of on a Pixhawk. Runs the same control loops a real drone would. |
| Harness | ardupilot_gz | Glue package that launches Gazebo and SITL together and wires them up. Ships `iris_runway.launch.py`. |
| ROS ↔ autopilot | AP_DDS | ArduPilot's native bridge: exposes the autopilot to ROS 2 as `/ap/*` topics. Replaces the older MAVROS. (Offline right now — see below.) |
| Sensor bridge | ros_gz_bridge | Bridges Gazebo's sensor topics into ROS 2 (`/navsat`, `/odometry`, `/camera/image`, …). What's keeping development unblocked while AP_DDS is down. |
| Perception | YOLO26 | Computer-vision marker detection. Not built yet. |
| Algorithm | DGA / BLL | The distributed coordination planners. Not built yet. |

### How the simulation pieces fit together

Gazebo and SITL are two separate programs that have to agree on reality.
They exchange **FDM** (Flight Dynamics Model) packets over UDP: Gazebo sends
the simulated physics/sensor state, SITL sends back motor commands. Once
that loop is alive, SITL starts **AP_DDS**, which publishes the autopilot's
state and accepts commands as ROS 2 `/ap/*` topics.

Today that FDM handshake is silent (the Gazebo plugin loads but emits zero
packets), so AP_DDS never initializes. The workaround: **ros_gz_bridge**
publishes Gazebo's sensors straight into ROS 2 regardless of SITL, giving us
`/navsat`, `/odometry`, `/camera/image`, etc. — exactly the inputs the
sensing and planning nodes need. The only capability we lose meanwhile is
*commanding* the drone back; that half waits on the FDM fix (fallback:
pymavlink over UDP).

---

## What was built this session

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
