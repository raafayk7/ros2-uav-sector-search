# Next-session prompt: follower planner (week 2)

Paste the block below to open the next session. Replace this file after the session, or delete it once the work is underway.

**Update 2026-11-04:** the warm-up and step 1 are done. `docs/planner-decisions.md` holds the decisions, and wins wherever it differs from this prompt, notably on `goal_lla`, `multiagent.launch.py`, the camera mount, gradient ascent and the BLL anchor. Start at step 2, using its Implementation Tasks.

---

Continuing the multi-agent UAV ROS 2 rebuild. This prompt supersedes the June 2026 one: the project has a plan now, the machine changed, and the biggest blocker turned out not to be one.

## Where the project stands (2026-11-03)

- Three things are done and verified: `uav_localization/localization_node` (GPS + odometry -> `/pose`, ENU arena frame, origin latched at first GPS fix, RELIABLE depth 10), `uav_mapping/leader_node` (detections -> Bayesian log-odds `nav_msgs/OccupancyGrid` on `/heatmap`, RELIABLE + TRANSIENT_LOCAL; 1 cell = 1 thesis sector, 5 m/cell, 4x4 over 20x20 m, corner origin; ships `fake_detection_publisher` and `heatmap.launch.py`), and the Docker stack.
- **Re-verified on 2026-11-02 on this machine** (Ubuntu 24.04, rebuilt image): `heatmap.launch.py standalone:=true` with a pose at (7.5, 7.5, 7.0) puts the single hot cell (value 97) at (1,1), matching the hand computation. Unknown cells are `-1`.
- **The FDM handshake works. The old "deferred, do not relitigate" note is obsolete.** Headless `iris_runway` brings up SITL ("JSON received"), 18 live `/ap/v1/*` topics (including `cmd_vel`, `cmd_gps_pose`, `goal_lla`, `geopose/filtered`, `pose/filtered`, `status`) and services (`arm_motors`, `mode_switch`, `experimental/takeoff`). GUIDED + arm + takeoff to +10 m worked. **The planner can actuate for real.** Only a single iris was tested.
- **Not yet checked:** Gazebo GUI and RViz in the container; `leader_node` against the live iris camera; `multiagent.launch.py` (exists in `ardupilot_gz_bringup`, takes a per-robot namespace from its `robots` dir; topic namespaces for several drones are unknown); `ros_gz_bridge` camera mount direction.

## The plan this session serves

Read `docs/roadmap.md` (committed, technical). The full private plan, with schedule and career context, lives outside the repo at `~/Documents/robotics-action-plan.md` and is deliberately not committed (the repo goes public as a WIP this week). If you can read it, use it for context but never copy personal content into the repo.

The project's thesis: **an LLM sets a prior heatmap, classical DGA + potential field flies the drones, the LLM never touches flight control, and the measured eval is the contribution.** Week 2 is the follower planner.

**Dates that matter:**
- by **Nov 10:** planner library + `follower_planner_node`, first follower ascending the stub heatmap **in SITL**; `gitleaks` scan clean; repo public as a WIP with a README.
- by **Nov 15 (the cut trigger):** 3 followers ascending the stub heatmap in SITL. If this slips by more than 3 days, drop to 2 followers first.
- budget about 12-15 h/week and end every weekend with a demonstrable artifact (a clip, a passing test, a plot).

## Machine and environment (read before running anything)

- Host is **Ubuntu 24.04** (CLAUDE.md predates this). Docker's data is on a **USB ext4 partition** (`/mnt/docker-data`, `/mnt/containerd-data`): the USB must stay plugged in. Do not unmount it. The vfat part of the same drive is separate and mounts at `/mnt/usb` if the user mounts it.
- Start with `docker compose -f docker/docker-compose.yml up -d`. Use `stop`, never `down`.
- In a **non-interactive** `docker compose exec ros2 bash -lc '...'`, run `source /usr/local/bin/entrypoint.sh` first, or the iris SDF will not resolve and the drone never spawns.
- Client shells that call `/ap/v1/*` **services** must export `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. Under the container's default CycloneDDS they hang silently. Topics work under both.
- Launch headless: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false`.
- Reset the sim with `docker compose restart ros2`. `pkill` leaves stray `arducopter`/`MicroXRCEAgent` that make the next launch fail.
- `docs/docker-notes.md` gotchas 18-23 have the detail. The setuptools gotcha (#17) did not bite on the new image (setuptools 84.2.1, `colcon build --symlink-install` worked), so do not apply the old workaround unless it fails.
- `build/`, `install/`, `log/`, `logs/`, `mav.parm`, `terrain/` are gitignored. The repo has a local git identity set; commit after each working sub-task, on `main`.

## Order of work

0. **Warm-up (about 15 min, folds in the old deferred item).** Start the container, `colcon build --symlink-install`, bring up headless `iris_runway`, run `leader_node` against the live camera (needs `/pose` from `localization_node` and the sim's `/camera/camera_info`), and confirm the real camera mount. Nadir is the `mount_quat` parameter, so a forward or gimbaled camera is a launch-arg change, not code. Stop the sim afterwards.

1. **Engineering review gate. Do this before writing planner code.** Run `/plan-eng-review` over the planner section of `docs/roadmap.md`. The roadmap lists the open questions; the ones that block this node are:
   - behavior on a flat or all-unknown (`-1`) posterior (the standalone heatmap is mostly unknown);
   - post-find peak suppression (after a target is found, do followers keep ascending it?);
   - termination with an unknown target count;
   - how the thesis utility game (8-neighborhood, overlap penalty) relates to the roadmap's "DGA + potential-field separation": keep the thesis one as the default, decide whether a separate potential-field layer is needed for physical separation;
   - arrival criterion and handoff to the later `landing_node`.
   Pull the user in on these; write the decisions into `docs/roadmap.md` (or a short `docs/planner-decisions.md`).

2. **Planner as a pure-Python library with no ROS imports** (suggest package `uav_planning`, ament_python, with a ROS-free module such as `uav_planning/core/`). It holds: grid indexing (pose <-> sector, same OccupancyGrid indexing as `leader_node`; decide whether to share a helper or duplicate the few lines), the utility function, DGA and BLL step functions, and any separation term. **Unit-test it with pytest at the library level** (random seeds, overlap penalty spreads drones, convergence on known high-p sectors, DGA vs BLL iteration counts with BLL near 153 as the thesis sanity anchor). This library is what the future 2D eval sim imports, so keep it free of ROS types.

3. **`follower_planner_node`: a thin ROS wrapper** around that library. One instance per follower. Subscribes to `/heatmap` (keep the input topic a launch arg: it will later point at `belief_map_node`'s fused posterior) and every follower's `/pose`; publishes a goal. `algorithm` param selects `dga` or `bll`. Defaults follow the thesis (8-neighborhood, 5 m sectors, 3-5 m arrival), as params. Mirror the `uav_localization`/`uav_mapping` package conventions (package.xml, setup.py, setup.cfg, absolute-default input topics as launch args, relative output).

4. **Actuation (now real).** Decide how a chosen sector becomes a command. `/ap/v1/cmd_gps_pose` and `/ap/v1/goal_lla` take geographic coordinates while the planner works in the ENU arena frame whose origin `localization_node` latches at the first GPS fix, so you need an arena -> lat/lon conversion (or use `cmd_vel` with a position controller). Pull the user in on this choice. First get one follower to take off in GUIDED, fly to a sector goal, and arrive.

5. **Decentralization and stepping (pull the user in; this is the learning).** How each planner discovers its peers and subscribes to every `/<robot>/pose` without a central coordinator; how asynchronous planners step on their own timers (the thesis BLL picks one random drone per iteration, ROS planners do not); avoiding two drones racing into one sector beyond what the utility penalty covers. Give the one-paragraph "what it is and when you care" on ROS timers/executors if it is load-bearing.

6. **Multi-drone in SITL.** First verify `multiagent.launch.py` actually works headless on this machine, and record the topic namespaces it produces (the single-drone `/ap/v1/...` names will be prefixed per robot). Then run 1 follower ascending a stub heatmap, then 2, then 3. This is the Nov 15 milestone. Also keep a software-only coordination harness (fake poses, no flight) as a fast test, built on the library from step 2.

7. **Repo hygiene for going public (week 2).** Run the secret scan over full history (the image is already local): `docker run --rm -v "$PWD":/repo zricethezav/gitleaks:latest detect --source /repo -v`. Confirm `docker/.env` is gitignored and was never committed (`git log --all -- docker/.env`), rotate anything found, write a WIP README (architecture diagram placeholder, one-command quickstart, honest status). **Ask the user before making the repo public**; that is an outward-facing step.

## Thesis algorithm (Chapter 3, sections 3.1.1.2-3.1.1.4), unchanged

- Arena is m x n sectors s_ij, each with probability p_ij in [0, 1]. Local neighborhood L(s) is the 8-connected neighbors.
- Utility for drone d_a at sector s_ij: `u = 0` if s_ij is in ANOTHER drone's neighborhood L(x_-a), else `u = p_ij`. Rewards high-probability sectors, penalizes piling onto the same area.
- **DGA** (deterministic, the hardware algorithm): evaluate all 8 neighbors' utility and move to the highest.
- **BLL** (stochastic, the simulation algorithm): pick a random alternative neighbor, softmax between staying and moving. About 153 iterations to converge in the original sim. Support both behind a runtime toggle.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain NEW concepts (timers/executors, discovery, QoS, any new message type) in one paragraph of "what it is / when you care" before code.
- Boilerplate (package.xml, setup.py, launch files): write it and explain briefly. Architecture choices (library boundary, decentralization, stepping, actuation path): pull the user in; that is the part they are learning.
- Default to ROS 2 Jazzy. No MAVROS, no MAVProxy in the runtime path, no iq_sim.
- Do not run long commands in the foreground; build and sim runs go in the background with logs under the scratchpad.
- Do not push or make the repo public without being asked.

## Read at session start

- `CLAUDE.md` (ground truth; its FDM section was updated 2026-11-02)
- `docs/roadmap.md` (architecture, algorithms, eval, milestones, open questions)
- `docs/docker-notes.md` (gotchas 18-23 are the newest)
- `docs/stack-overview.md` (system, stack, glossary, both built nodes)
- `docs/thesis.pdf` Chapter 3, sections 3.1.1.2-3.1.1.4
- `src/uav_localization/` and `src/uav_mapping/` (mirror their conventions; reuse the OccupancyGrid indexing from `leader_node`)
- the `multi-agent-uavs` skill auto-loads from `.claude/skills/`
