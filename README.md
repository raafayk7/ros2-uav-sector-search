# Multi-Agent UAV Search & Rescue — ROS 2 Rebuild

**Work in progress.** A leader drone sweeps an area and builds a probability map of where targets are; follower drones each fly, independently, to the best open sector on that map. This repo is a ROS 2 / Gazebo Harmonic rebuild of the LUMS senior project *"Distributed Autonomous Surveillance and Rescue Architecture Integrating Computer Vision within Multi-Agent Drone Coordination"* (Anwer, Moeez, Tahir, Kazmi — Spring 2025). The original ran on ROS Noetic + Gazebo 11; this version targets a modern stack and the future-work items the original team listed (true decentralization, scale beyond two followers, more robust perception).

What flies today: from one command, a leader sweeping the arena and **three followers** finding every target on a shared map, in simulation, with a mission that starts the search, decides when it is over and reports the finds. Read [Status](#status) before expecting more.

## How it is meant to work

```
 [ architecture diagram: placeholder ]

 leader drone ──detections──▶ shared probability map ◀──read── follower 1..N
                                  (one cell = one sector)         │
                                                                  ▼
                              each follower picks its best open sector,
                              penalising sectors its peers already hold,
                              and flies there through ArduPilot (AP_DDS)
```

- **Shared map:** a grid of per-sector probabilities, owned by one `belief_map_node`. Every drone's detections raise cells, and every camera footprint with no detection lowers them. A follower confirms a target when it hovers over a sector and sees it on 3 consecutive frames; that sector then drops out of the search.
- **Followers decide on their own.** Each one scores every sector on the map, subtracts an overlap penalty for sectors peers have claimed (peers broadcast their goal sector and pose), and flies to the best open one. There is no central planner. Followers fly at different altitude layers so transit crossings stay separated.
- **Baselines:** the thesis's Distributed Gradient Ascent (DGA) and Bilinear Log-Linear (BLL) are kept as library baselines to measure the sector planner against.
- **Library first:** the planner, the follower logic and the baselines are plain Python with no ROS import (`src/uav_planning/uav_planning/core/`). The ROS nodes are thin wrappers, so the same code can be tested and evaluated without a simulator.

Design rationale and the record of decisions: [`docs/planner-decisions.md`](docs/planner-decisions.md) · migration reasoning: [`docs/assessment-ros2-migration.md`](docs/assessment-ros2-migration.md) · plan and milestones: [`docs/roadmap.md`](docs/roadmap.md).

## Stack

| Layer | Choice |
|---|---|
| Host | Ubuntu 24.04, everything else inside Docker |
| ROS 2 | Jazzy Jalisco (LTS through May 2029) |
| Simulator | Gazebo Harmonic |
| Drone harness | [`ArduPilot/ardupilot_gz`](https://github.com/ArduPilot/ardupilot_gz), ArduPilot SITL |
| ROS ↔ ArduPilot | AP_DDS (native), not MAVROS |
| Perception | ground-truth detector with noise today; Ultralytics YOLO26 planned |

## Quickstart

Needs Docker with the system daemon (not Docker Desktop: the compose file passes through `/dev/dri`) and roughly 25 GB for the image. The `video` and `render` group IDs in `docker/docker-compose.yml` (44, 992) match Debian/Ubuntu defaults; change them if your host differs. Details and gotchas: [`docs/docker-notes.md`](docs/docker-notes.md).

```bash
cp docker/.env.example docker/.env                   # edit UID/GID if your user isn't 1000:1000
docker compose -f docker/docker-compose.yml build    # long, once
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec ros2 bash -l
```

`-l` matters: only a login shell puts MAVProxy on `PATH`, and without it SITL hangs waiting for a connection that never comes.

Inside the container, build once, then start the sim and **wait until it is up** before starting the planner:

```bash
colcon build --symlink-install && source install/setup.bash   # rebuild after every edit to uav_planning
ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false
```

In a second shell (`exec ros2 bash -l` again, then `source install/setup.bash`):

```bash
ros2 launch uav_planning follower.launch.py
```

This starts localization, a stub heatmap, the arm-and-takeoff helper and the follower planner. The drone arms, climbs to 10 m, flies to the best sector of the stub map and reports `ARRIVED`. Publishing `false` to `planner_enable` makes it hold.

Three followers, sim included, in one command (stop the single-drone sim first with `docker compose -f docker/docker-compose.yml restart ros2`):

```bash
ros2 launch uav_planning swarm.launch.py
```

Three iris spawn 5 m apart, climb to 10, 12 and 14 m, and each settles on a different peak of the stub map, about 100 s after launch on this machine.

The search itself, a leader, three followers, one detector per drone and the shared belief map:

```bash
ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=3
python3 docs/probes/belief_live_check.py 300 3    # second shell, once the SITLs are up: spawns 3 targets and logs the search
```

Tests (no simulator needed; ROS sourced for a few of them):

```bash
python3 -m pytest src
```

## Status

Verified in SITL, 2026-11-22, in cold runs from one command with a leader and three followers (`swarm.launch.py leader:=true num_followers:=3`, headless real-time factor about 0.59, 0.27-0.41 with the Gazebo window open):

- The leader flies a lawnmower sweep over the 40 x 40 m arena (8 x 8 sectors at 5 m). Followers launch with it and fly to whatever the shared map currently rates highest. The lanes cover all 64 sectors with about 1 m to spare at the edges (computed from the camera model).
- A mission state machine starts the planners, ends the run when the sweep is complete and nothing is above 50 % for 30 s (or after a timeout), and publishes a report of the finds with the drone that made each. A headless run ended DONE after 142 s of wall time (74.6 s of sim time) with all 3 targets found by 3 different drones and nothing left open.
- After the end, each finder flies to hover over its find. In a full run all three were within 0.1 m of their find 19 s later.
- Detection is a ground-truth stand-in for YOLO: Gazebo target positions with misses (20 %), false alarms (5 % per frame) and 5 px noise, on a camera held straight down.
- `gui:=true` opens the Gazebo window. `arena_view:=true` adds a video look: the arena drawn as a green plate with a line per sector, a top-down view, coloured drones and a marker on the ground under each.

Earlier milestones, still true: one follower flies to a sector and reports `ARRIVED`, and `planner_enable` false holds it (2026-11-08). Three followers each hold a different stub-map peak, and a silent follower's peak is released about 5.6 sim seconds later (2026-11-12). Two cold runs found all 3 targets before the sweep ended, including one 0.3 m inside a sector edge (2026-11-20). 225 tests pass: the planner, sweep, belief map and mission libraries, the DGA/BLL baselines, a 200-seed coordination harness, the shared-datum localization and the mission node, run with no simulator up.

**Not done yet:**

- **A recorded demo.** No video exists yet.
- **Perception.** YOLO26 is a stretch goal; the noisy ground-truth detector stands in.
- **Landing.** Precision landing (colour masking + PID in the original) is not rebuilt and is in no milestone; a find ends in a hover.
- **Scale beyond four drones** and **re-enabling after a disable** have not been run. A four-drone run holds about 30 DDS participants, close to the 32 the default RMW allows (`docs/docker-notes.md`, gotcha 61).

Known issues and measured quirks are logged in [`docs/docker-notes.md`](docs/docker-notes.md) and [`TODOS.md`](TODOS.md).

## Layout

```
multi-agent-uavs/
├── docs/        # thesis, decision record, roadmap, Docker notes, probes
├── src/         # colcon workspace: uav_localization, uav_mapping, uav_perception, uav_planning
├── docker/      # Dockerfile + compose for the dev container
├── skills/      # Claude skill describing this project (optional tooling)
├── CLAUDE.md    # context file for AI coding assistants
└── README.md
```

`build/`, `install/` and `log/` are colcon output and gitignored, as are SITL's runtime files.

## License

[MIT](LICENSE).

## Credit

Rebuilt by Raafay Saeed Kazmi, one of the four original authors. The algorithms, the sector model and the evaluation setup come from the senior project, advised by Dr. Hassan Jaleel and Dr. Murtaza Taj, LUMS Electrical Engineering.

- [`docs/thesis.pdf`](docs/thesis.pdf) — the original thesis (49 pages)
- [`docs/legacy-implementation-guide.docx`](docs/legacy-implementation-guide.docx) — the original team's command cheat sheet for the Noetic setup (historical)
