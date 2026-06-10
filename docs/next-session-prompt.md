# Next-session prompt — follower DGA planner

Paste the block below to open the next session. (This file is just a staging
spot; delete it once the session is underway if you like.)

---

Continuing the multi-agent UAV ROS 2 rebuild.

Status at end of last session (2026-06-10):
- TWO project nodes are done and verified.
  1. `src/uav_localization/localization_node` — GPS + odometry -> `/pose`
     (`geometry_msgs/PoseStamped`) in an ENU arena frame, origin latched at
     the first GPS fix. `/pose` is RELIABLE, depth 10.
  2. `src/uav_mapping/leader_node` — the first algorithm-relevant node, the
     leader's perception-to-map step. Subscribes `/pose`, `/detections`
     (`vision_msgs/Detection2DArray`), `/camera/camera_info`; projects each
     detection to the ground (pinhole via camera_info K, composed with
     `/pose` and a static nadir camera mount, flat ground z=0) and
     accumulates a Bayesian log-odds heatmap published as
     `nav_msgs/OccupancyGrid` on `/heatmap` (RELIABLE + TRANSIENT_LOCAL).
     Geometry: ONE OccupancyGrid cell == ONE thesis sector. Defaults match
     the thesis — 5 m/cell, 4x4 over the 20x20 m arena, CORNER origin (home
     at cell [0,0]'s corner) — all launch params. Ships with
     `fake_detection_publisher` (synthetic Detection2DArray at a known pixel)
     and `heatmap.launch.py`. Verified standalone: drone at (7.5,7.5,7.0)
     looking down, detection at image center -> sector cell (1,1) at world
     (7.5,7.5), matching a hand computation.
- DEFERRED from last session (small, fold in as a ~15 min warm-up): run
  `leader_node` against the live `iris_runway` camera, and confirm the real
  iris camera mount. The nadir (down-facing) assumption is a `mount_quat`
  PARAMETER, so if the sim camera is forward/gimbaled it's a launch-arg
  change, not code.
- FDM handshake still DEFERRED (do not relitigate): Gazebo<->SITL FDM is
  silent, so `/ap/*` never appears and the drone can't be COMMANDED. This
  does NOT block planner work — the planner only needs to DECIDE moves, and
  we validate convergence as a coordination simulation in software (see
  sub-task 6). Develop input topics against `ros_gz_bridge`/our `/pose` and
  `/heatmap`; the actuation OUTPUT is stubbed for now.
- Build env note (unchanged): setuptools>=80 breaks colcon --symlink-install.
  Dockerfile pins <80, but a container created before a rebuild still has
  80+. Runtime fix: `sudo pip3 uninstall -y --break-system-packages
  setuptools`. Survives `docker compose stop`/`start` but NOT `down`. Use
  `stop`, not `down`. See docs/docker-notes.md gotcha #17.

Today's goal: build `follower_planner_node` — the distributed planner that is
the algorithm core of the thesis. One instance per follower. Each subscribes
to `/heatmap` and to every follower's `/pose`, and runs the thesis utility
game to pick which sector to move toward, publishing a goal (stubbed output
since actuation is blocked). This closes the leader->follower loop and
attacks the thesis future-work item of TRUE DECENTRALIZATION (each drone runs
its own planner — no central controller, no "Drone 1 computes for Drone 2"
hack). Pull me into the design choices; that's the part I'm learning.

Thesis algorithm (Chapter 3, §3.1.1.2–3.1.1.4):
- Arena = m x n sectors s_ij, each with probability p_ij in [0,1].
- Local neighborhood L(s) = the 8-connected immediate neighbors of sector s.
- Utility for drone d_a at sector s_ij:
      u(s_ij, d_a) = 0      if s_ij is in ANOTHER drone's neighborhood L(x_-a)
                   = p_ij   otherwise
  (rewards high-probability sectors, penalizes piling onto the same area).
- DGA (deterministic, the hardware algorithm): evaluate the utility of all 8
  neighbors of the current sector, move to the highest. Quick convergence.
- BLL (stochastic, the simulation algorithm): pick a random alternative
  neighbor, softmax between staying and moving. Converged in 153 iterations
  in the original sim. Support BOTH as a runtime toggle for comparison.

Concrete sub-tasks:
1. Create the package (suggest `uav_planning`, ament_python; OccupancyGrid +
   PoseStamped are standard types, so no custom msgs). Mirror the
   uav_localization / uav_mapping conventions (package.xml, setup.py,
   setup.cfg, absolute-default input topics as launch args, relative output).
2. ARCHITECTURE — pull me in: continuous pose <-> discrete sector. How a
   follower's `/pose` (x,y) maps to a sector cell, and the inverse (chosen
   sector -> goal pose at the cell center). This is the same OccupancyGrid
   indexing `leader_node` already does — decide whether to factor a shared
   helper (a small `uav_common`?) or duplicate the few lines.
3. ARCHITECTURE — pull me in: the decentralization model. Each planner needs
   peer positions to compute L(x_-a). How do we discover the robot list and
   subscribe to all `/<robot>/pose`? How do we keep it truly decentralized
   (every drone its own planner over the shared map + peer poses) rather than
   sneaking in a central coordinator?
4. ARCHITECTURE — pull me in: the stepping & convergence model. The thesis
   BLL is a sequential "pick one random drone per iteration" process; in a
   decentralized ROS system each planner steps asynchronously on its own
   timer. How do we handle that (timer rate, avoiding two drones racing into
   the same sector beyond what the utility penalty covers)? And the
   termination/arrival criterion (thesis: within 3–5 m of a target sector,
   which later hands off to `landing_node`). Give me the one-paragraph "what
   it is / when you care" on ROS timers/executors if it's load-bearing here.
5. Implement `follower_planner_node`: subscribe `/heatmap` + all peer
   `/pose`; publish a goal on a launch-arg output topic (default a stub like
   `/<robot>/goal_sector` or `/<robot>/ap/goal_pose`, flips to `/ap/*`
   later). `algorithm` param selects `dga` or `bll`. Thesis-faithful defaults
   as params (8-neighborhood, arrival radius, etc.).
6. Build a COORDINATION SIMULATOR / test harness: since actuation is blocked,
   close the loop in software — publish a known/synthetic `/heatmap`, seed N
   follower poses at random sectors, and have the harness move each follower
   to its planner's chosen sector each step. This is how we watch convergence
   WITHOUT flying (mirrors the thesis's pure-algorithm simulation). Log the
   per-drone sector trajectories.
7. Launch file `planning.launch.py` bringing up N planners + the sim harness
   + a heatmap source, mirroring `heatmap.launch.py` arg conventions.
   Remember the `use_sim_time` footgun: standalone/no-sim runs must set sim
   time off (heatmap.launch.py now does this automatically — copy that).
8. Test: over a known heatmap with a couple of high-probability sectors, seed
   followers at random and confirm they (a) spread out via the overlap
   penalty rather than stacking, and (b) converge on the high-p sectors.
   Compare DGA vs BLL iteration counts (BLL ~153 in the thesis as a sanity
   anchor). View poses + heatmap in RViz, or read the trajectory logs.

Workflow constraints (same as prior sessions):
- I have high-level ROS understanding, not deep. Explain NEW concepts (ROS
  timers/executors, decentralized discovery, any new message type) with a
  one-paragraph "what it is / when you care" before code.
- Boilerplate (package.xml, setup.py, launch) — write it for me. Architecture
  choices (utility/stepping/decentralization model, DGA vs BLL structure) —
  pull me in; that's the learning.
- Default new-node behavior to the thesis's own numbers/behavior, kept as
  params (8-neighborhood, 5 m sectors matching the heatmap, 3–5 m arrival).
- Default to ROS 2 Jazzy. Don't recommend MAVROS / MAVProxy-in-runtime /
  iq_sim.
- Commit after each working sub-task, not at the end. Stay on main.

Files to read at session start:
- CLAUDE.md (project ground truth)
- docs/stack-overview.md (system + stack + glossary; both built nodes)
- docs/docker-notes.md (stack state, setuptools gotcha #17, FDM issue)
- docs/thesis.pdf Chapter 3, §3.1.1.2–3.1.1.4 (utility, BLL, DGA)
- src/uav_localization/ and src/uav_mapping/ (mirror conventions; reuse the
  OccupancyGrid indexing / geometry from leader_node)
- the multi-agent-uavs skill auto-loads from .claude/skills/ (now carries a
  Build status section listing what's done)
