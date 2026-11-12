# Multi-Agent UAV Search & Rescue — ROS 2 Rebuild

**Work in progress.** A leader drone sweeps an area and builds a probability map of where targets are; follower drones each fly, independently, to the best open sector on that map. This repo is a ROS 2 / Gazebo Harmonic rebuild of the LUMS senior project *"Distributed Autonomous Surveillance and Rescue Architecture Integrating Computer Vision within Multi-Agent Drone Coordination"* (Anwer, Moeez, Tahir, Kazmi — Spring 2025). The original ran on ROS Noetic + Gazebo 11; this version targets a modern stack and the future-work items the original team listed (true decentralization, scale beyond two followers, more robust perception).

What flies today is **one follower**. Read [Status](#status) before expecting more.

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

- **Shared map:** a grid of per-sector probabilities. The leader's detections raise cells; (planned) camera footprints with no detection lower them.
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
| Perception (planned) | Ultralytics YOLO26 |

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

Tests (no simulator needed; ROS sourced for a few of them):

```bash
python3 -m pytest src
```

## Status

Verified in headless SITL with one iris (real-time factor about 0.46):

- Arm, take off to the 10 m layer, fly to the best stub-map sector and report `ARRIVED` about 7 s after the planner picks it (warm sim), then hold within 0.1 m of the sector centre for 40 s. From a cold start the whole sequence takes under a minute.
- `planner_enable` false mid-flight holds position.

Verified with three iris and three followers (`swarm.launch.py`, real-time factor about 0.26-0.38), 2026-11-12, in 2 cold runs:

- Each follower settles on a different peak of the stub map, all `ARRIVED` about 102 s after launch, with at most one goal change (a same-tick tie the higher sysid gives up).
- When one follower goes completely silent, its peak is released about 5.6 sim seconds later and the others move up to the better peaks.
- 72 tests pass: the planner library, the DGA/BLL baselines, a 200-seed coordination harness, the follower logic and the shared-datum localization.

**Not done yet:**

- **Scale.** Three followers have flown together; four or more have not, and the sim runs at roughly a third of real time with three.
- **Leader sweep.** The leader's detection-to-heatmap node exists, but there is no sweep controller and the leader has not flown. Followers fly against a stub map.
- **Perception.** Detections come from a fake publisher. A ground-truth detector comes first and YOLO is a stretch. The simulated camera sits on a gimbal 52° off nadir, which has to change first.
- **Landing.** Precision landing (colour masking + PID in the original) is not rebuilt.
- **Re-enabling after a disable** has not been run live.

Known issues and measured quirks are logged in [`docs/docker-notes.md`](docs/docker-notes.md) and [`TODOS.md`](TODOS.md).

## Layout

```
multi-agent-uavs/
├── docs/        # thesis, decision record, roadmap, Docker notes, probes
├── src/         # colcon workspace: uav_localization, uav_mapping, uav_planning
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
