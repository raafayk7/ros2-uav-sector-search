# Follower planner: engineering review and decisions

Review target (fixed): the follower planner section of `docs/roadmap.md`, scoped by steps 1-6 of "Order of work" in `docs/next-session-prompt.md`. Reviewed 2026-11-04 with `/plan-eng-review` at commit `7253f0e`.

## Decisions at a glance

Status: decided, nothing built yet. Pure-Python probes and SITL spikes back these; the planner itself has not flown. Full reasoning is in the ledger below.

| Topic | Decision | Ref |
|---|---|---|
| Map frame | One world datum for every drone; unset keeps today's first-fix latch | D7 |
| Flight command | `/ap/vN/cmd_gps_pose`, `frame_id "map"`, `FRAME_GLOBAL_REL_ALT`, resent at 1 Hz until arrival | D8 |
| Flying rule | Each drone flies straight to the best open sector anywhere on the grid; thesis DGA and BLL stay in the library as baselines, with no runtime toggle | D4, D17 |
| Staying put | Current sector counts; move only if the best option beats it by more than 0.02 | D16 |
| Blocked sectors | Peer goal zones (8 neighbours + centre) block; a peer's position blocks only once it has arrived | D5, D11, D25 |
| Goal conflicts | The higher sysid yields to any lower sysid's goal zone | D24 |
| Goal lifetime | Republished at 1 Hz; expires 5 s after the last repeat | D28 |
| Silent peer | Stops blocking after 5 s sim time, with a warning | D21 |
| Separation | Altitude layers (leader 8 m, followers 10 + 2 x index); no sideways flight until within 0.5 m of layer; no potential field | D5, D26 |
| Peers | Static `robot`, `sysid`, `peers` from launch | D9 |
| Stepping | 1 Hz sim-time timer; re-decide on arrival or if the goal becomes blocked | D10 |
| Arrival | Within 2.0 m horizontally and at altitude | D26, D27 |
| Handoff | `planner_status` + `planner_enable`; disabling sends one hold command | D12, D29 |
| Arming | Separate `arm_takeoff` helper on Fast DDS; planner stays on CycloneDDS | D15 |
| Unknown cells | Count as 0.5; ties go to nearest, then lowest index; flat map holds | D18 |
| Map meaning | Each cell is an independent probability of containing a target | D31 |
| Found target | Follower within arrival radius sees it in its sector for N frames; `belief_map_node` lowers that sector | D13, D32 |
| Mission end | Sweep done and all unsuppressed sectors under 0.5 for 30 s, or timeout; decided by the mission state machine | D14 |
| Milestone map | New stub publisher, 8x8 at 5 m, multiple peaks | D19 |
| Camera | Required before footprints: gimbal to nadir, proven with the marker test, 2 h timebox | D30 |
| Sim speed | 2 h timebox after Nov 15, before Nov 24 | D23 |
| Checkpoint | Around Nov 20, one follower completes a whole search loop | D33 |
| Layout | One package, `uav_planning`, with `core/planner.py` | D6 |

## Original plan (unchanged copy)

From `docs/roadmap.md`:

| Followers | `follower_planner_node`: DGA over the posterior + potential-field separation | **next** |
**Code structure rule:** DGA, potential field, fusion and region rasterization are a **pure Python library with no ROS dependency**, unit-tested at the library level. ROS nodes are thin wrappers. This is what lets the 2D eval sim reuse the code unchanged.

## Open engineering questions (settle in `/plan-eng-review` before week-2 planner code)

Follower behavior on a flat posterior; post-find peak suppression; mission termination with an unknown target count; region rasterization rule; default alpha and run matrix; sweep time vs timeout; detector noise parameters and the true-find definition; multi-target clue mapping and the misleading-landmark rule; CI vehicle model and runtime; replay lookup for unmatched clues.

From `docs/next-session-prompt.md`, Order of work:

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

## Scope record

Feature answers: D4 = BLL library only for now; D5 = thesis penalty + altitude layers, no potential field now. Structure: D6 = Smaller arrangement. Accepted scope: one package `uav_planning` with ROS-free `core/planner.py` (DGA and BLL functions), `follower_planner_node.py` (DGA only at runtime), `test/test_planner.py`, and launch files including an in-package swarm launch; about 10 files and 1 new class. BLL is implemented and unit-tested in the library, with no runtime toggle in the node. No potential-field layer; followers fly at per-drone altitudes. Pending remedies: all R-numbered choices in the ledger below until answered.

Scope Challenge result: scope reduced per recommendation.

## Scope Challenge findings

Evidence below is from bounded probes on 2026-11-04 (headless SITL, 1-4 iris) and from reading the pinned ArduPilot and ardupilot_gz sources in the image. No proposed code was built.

1. [P1] (confidence: 10/10) `ardupilot_gz_bringup/launch/multiagent.launch.py`: `robots = [` is a hard-coded list of 4 different vehicles with no launch arg. Factual correction to step 6: "verify multiagent.launch.py" becomes "write our own swarm launch reusing `robots/iris.launch.py`". A patched copy with 3 identical iris brought up all three SITL instances in 17 s. Disposition: necessary work for the Nov 15 milestone, included in accepted scope.
2. [P1] (confidence: 10/10) `AP_DDS_Topic_Table.h`: `goal_lla` has `.topic_rw = Topic_rw::DataWriter`, so it is autopilot output (`GeoPointStamped`), not a command. Publishing to it had 0 subscribers and moved nothing. Factual correction to step 4. It is useful as confirmation: it echoes the accepted target.
3. [P1] (confidence: 10/10) `AP_DDS_ExternalControl.cpp`: `if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0)` silently drops `cmd_gps_pose` unless `frame_id` is `"map"`. Verified both ways: empty frame_id moved nothing at 1 message or at 2 Hz for 70 s; `"map"` flew. Factual correction recorded for step 4.
4. [P1] (confidence: 10/10) Topic namespaces are per-source: Gazebo bridge `/iris_N/*`, autopilot `/ap/vN/*` with N = sysid. Factual correction for step 6; the skill's `/iris_1/ap/...` form does not exist on this stack.
5. [P2] (confidence: 9/10) `AP_DDS_Topic_Table.h`: `cmd_gps_pose` QoS is BEST_EFFORT + VOLATILE, depth 5. Gazebo `/iris_N/navsat` altitude reads 10.2 while `/ap/vN/navsat` reads 584.2, so absolute altitudes must not cross between them; `FRAME_GLOBAL_REL_ALT` avoids this.
6. Algorithm evidence and BLL evidence: see probe table in the ledger section.

Algorithm probe (200 seeds, 3 drones, 3 peaks at 0.97 on a 0.03 background, min 2 cells apart; pure Python, round-robin, 40 rounds):

| Variant | Grid | Peaks held | All 3 held | Moving at end | Stacked |
|---|---|---|---|---|---|
| A thesis literal: no stay, peer cell free | 4x4 | 1.32/3 | 10% | 100% | 1% |
| B + stay + peer cell penalised | 4x4 | 2.17/3 | 38% | 0% | 0% |
| C = B + gamma 0.8 slope to open peaks | 4x4 | 2.81/3 | 84% | 0% | 0% |
| A | 8x8 | 1.02/3 | 2% | 100% | 0% |
| B | 8x8 | 0.82/3 | 0% | 0% | 0% |
| C | 8x8 | 2.75/3 | 76% | 0% | 0% |
| A | 12x12 | 0.76/3 | 0% | 100% | 0% |
| B | 12x12 | 0.45/3 | 0% | 0% | 0% |
| C | 12x12 | 2.84/3 | 84% | 0% | 0% |

Other probe results: flat map with stay rule, any drone moved in 68% of runs at 4x4 and 15% at 12x12 (only drones inside a peer's neighbourhood move). Two drones zero out 63% of a 4x4 map for the third, 22% of 8x8, 11% of 12x12. Two adjacent peaks and two drones: both held 0%, one held 100%. BLL as written: median iterations to first all-3-on-peaks 408 at 4x4 and 8000 (never, within the run) at 12x12; long-run peaks held 0.66/3 and 0.14/3; P(leave a 0.97 peak for a 0.03 neighbour) = 0.28 per attempt.

## Decision ledger

### R1: Shared arena origin across drones
Finding: 1, P0, confidence 10/10, `src/uav_localization/uav_localization/localization_node.py:144`, native review.
Plan baseline: original proposal; each `localization_node` latches its own origin at its first fix (`if self._lat0 is None: self._lat0 = lat`). No approval on record.
Runtime evidence: 3 iris spawned at (0,0), (5,0), (0,5) m give distinct `/iris_N/navsat` fixes (5 m apart), so each node would latch a different origin and report itself at (0,0,0). Not yet run with 3 localization nodes. Gazebo navsat alt at spawn 10.19 with z = 0.195, so the current arena z = 0 sits 0.19 m above the true ground.
Comparison grid:
| Choice | Current | A: shared datum | B: per-drone offsets | C: leave as is |
|---|---|---|---|---|
| R1 frame shared by all drones | no, one origin per drone | yes, same lat/lon/alt params on every node | yes, each node adds its spawn offset to its own origin | no |
| Single-drone behaviour | latch at first fix | unchanged when params unset | unchanged when offset is zero | unchanged |
| Launch must know spawn positions | no | no, only the world datum | yes, duplicated per drone | no |
| Work | none | ~10 lines + test (human ~1h / CC ~5 min) | ~10 lines + launch wiring (human ~1h / CC ~5 min) | none |
Question D7:
D7 — How should all drones share one map frame?
Project/branch/task: follower planner architecture on main, before any multi-drone work.
ELI10: Right now each drone calls the spot where it first got GPS "(0,0)". One drone is fine. With three drones spawned apart, each says "I'm at (0,0)", so they all think they share a sector and the heatmap means something different to each. The fix is for every drone to measure from one agreed point.
Stakes if we pick wrong: leaving it means no multi-drone run can work, and the failure looks like a planner bug, because the numbers look valid.
Recommendation: A because the launch file already knows the world, and this needs no per-drone bookkeeping.
Note: options differ in kind, not coverage — no completeness score.
Net: one world-level constant versus per-drone spawn bookkeeping.
Header: Map frame
Options:
A) Shared world datum (recommended)
✅ One set of lat/lon/alt params passed to every drone, so frames agree regardless of where drones spawn. ✅ Unset params keep today's first-fix latch, so single-drone runs and existing launches are unchanged. ✅ Also makes the later arena-to-GPS conversion use the same reference. ❌ The datum must be set per world, though it is a single value. Effort: human ~1h / CC ~5 min.
B) Per-drone spawn offsets
✅ Keeps each node's own latch logic and only adds an offset param. ✅ Mirrors how spawn positions already appear in the launch file. ❌ Spawn positions get duplicated into localization params, and any mismatch silently shifts one drone's frame. Effort: human ~1h / CC ~5 min.
C) Leave as is
✅ No change to a verified node, and one follower still works for the Nov 10 target. ✅ Nothing to test today. ❌ Blocks the Nov 15 milestone, so the same work returns in a few days under more pressure.

State: approved
Actual answer: A) Shared world datum (D7, 2026-11-04)
Accepted scope: `localization_node` gains optional datum params (lat/lon/alt). Set: every drone uses them. Unset: today's first-fix latch is unchanged. The swarm launch passes one world datum to all drones. Includes a test that two nodes with different first fixes and the same datum report positions in one frame.
History: none

### R2: How a chosen sector becomes a flight command
Finding: 2, P1, confidence 10/10, `AP_DDS_Topic_Table.h` (`cmd_gps_pose` DataReader) and `AP_DDS_ExternalControl.cpp`, native review.
Plan baseline: original proposal; step 4 leaves the choice open between geographic goals and `cmd_vel` with a position controller.
Runtime evidence: one `GlobalPosition` on `/ap/v2/cmd_gps_pose` with `frame_id "map"`, `FRAME_GLOBAL_REL_ALT`, lat/lon derived from Gazebo navsat, under CycloneDDS: iris_2 moved 8 m north, within 0.4 m after about 10 s wall time, iris_1 stayed, `/ap/v2/goal_lla` echoed the target. `AP_ExternalControl_Copter::ready_for_external_control` requires GUIDED and armed, otherwise the command is dropped with no reply. `cmd_vel` not tested.
Comparison grid:
| Choice | Current | A: position goals | B: velocity loop |
|---|---|---|---|
| R2 command topic | none | `/ap/vN/cmd_gps_pose` | `/ap/vN/cmd_vel` |
| Who controls the path | n/a | ArduPilot's guided position controller | our node at a fixed rate |
| Conversion needed | n/a | arena to lat/lon using the R1 datum | none, velocity in arena axes |
| Lost or dropped command | n/a | resend same goal at 1 Hz until arrival; goal_lla confirms | stops when stream stops; needs a timeout |
| Work | none | ~25 lines + round-trip test (human ~2h / CC ~10 min) | ~60 lines + gain tuning at RTF 0.11-0.25 (human ~2 days / CC ~1h) |
Question D8:
D8 — How should the planner tell a drone to fly to a sector?
Project/branch/task: follower planner architecture on main; step 4 of the plan asks for your call here.
ELI10: The planner picks a sector; something must turn that into flight. Option A hands ArduPilot a GPS point and lets its own tested controller fly there. I ran this today: one message, 8 m, arrived within 0.4 m. Option B has our node stream velocity commands and steer itself, which gives finer control but makes us write and tune a controller, and potential fields are no longer needed for it now that D5 is settled.
Stakes if we pick wrong: B costs days of tuning before Nov 15; A means ArduPilot chooses the path between points, so transit is a straight line we don't shape.
Recommendation: A because it is proven today and matches the 'LLM and planner never touch low-level flight' idea.
Note: options differ in kind, not coverage — no completeness score.
Net: reuse the autopilot's controller versus own the control loop.
Header: Actuation
Options:
A) GPS position goals (recommended)
✅ Already flown today: one message arrived within 0.4 m under the default RMW. ✅ `goal_lla` echoes the target, so the node can confirm the autopilot accepted it. ✅ The arena-to-GPS function is pure maths, tested in the library. ❌ Commands are dropped silently unless GUIDED and armed, so the node resends at 1 Hz and logs if goal_lla never matches. Effort: human ~2h / CC ~10 min.
B) Velocity loop on cmd_vel
✅ Full control of the path and speed, which a future continuous separation layer would use. ✅ No lat/lon conversion at all. ❌ New controller, gains and timeout logic, not tested today, tuned at RTF 0.11-0.25. Effort: human ~2 days / CC ~1h.

State: approved
Actual answer: A) GPS position goals (D8, 2026-11-04)
Accepted scope: the node publishes `ardupilot_msgs/GlobalPosition` on `/ap/vN/cmd_gps_pose` (BEST_EFFORT, VOLATILE) with `frame_id "map"` and `FRAME_GLOBAL_REL_ALT`, resending the current goal at 1 Hz until arrival, and logs a warning if `/ap/vN/goal_lla` never matches. Arena-to-lat/lon is a pure function in `core/planner.py`, using the R1 datum, with a round-trip test against `localization_node`'s projection.
History: none

### R3: How each planner finds its peers
Finding: 3, P2, confidence 8/10, `docs/next-session-prompt.md` step 5 ("discovers its peers ... without a central coordinator"), native review.
Plan baseline: original proposal; step 5 leaves the mechanism open.
Runtime evidence: swarm probe shows `/iris_N/*` and `/ap/vN/*` are separate namespaces, so each node needs its robot name and sysid either way. Discovery timing is not instant: an early `ros2 topic list` during the probe showed no `/ap/` topics at all, and a later one showed them. Dynamic discovery was not tested.
Comparison grid:
| Choice | Current | A: static list | B: dynamic discovery |
|---|---|---|---|
| R3 how peers are known | undecided | `peers` launch param per node | node scans the ROS graph for `/*/pose` on a timer |
| Central coordinator | none | none | none |
| Drone added mid-run | n/a | not seen until relaunch | seen on next scan |
| Missing peer at startup | n/a | known by name, logged as absent | indistinguishable from no peer existing |
| Work | none | ~5 lines (human ~20 min / CC ~2 min) | ~30 lines + subscription lifecycle tests (human ~half day / CC ~20 min) |
Question D9:
D9 — How should each follower's planner know who the other followers are?
Project/branch/task: follower planner architecture on main; step 5 of the plan, decentralization.
ELI10: Each planner needs every other follower's position. ROS 2 already removes any central server: nodes find each other over DDS, the network layer underneath. So the only question is how a planner learns which names to listen to. Option A writes the list in the launch file. Option B has each planner scan the network for anything that looks like a pose topic. Both are fully decentralized at run time, because no node hands out decisions in either one.
Stakes if we pick wrong: B adds moving parts and a silent failure, because a peer that hasn't appeared yet looks like no peer at all, so two drones can pick one sector; A can't add a drone mid-mission.
Recommendation: A because the fleet is fixed per mission and an absent peer stays visible by name.
Note: options differ in kind, not coverage — no completeness score.
Net: explicit fixed fleet versus join-anytime flexibility.
Header: Peers
Options:
A) Static peer list (recommended)
✅ Each node gets `robot`, `sysid` and `peers` from the swarm launch, so a run's fleet is readable in one file. ✅ A peer that hasn't reported yet is known and can be logged, and the planner can wait for it. ✅ Scales by changing one launch arg. ❌ A drone can't join mid-mission without a relaunch, which no planned scenario needs. Effort: human ~20 min / CC ~2 min.
B) Dynamic graph discovery
✅ Drones can join or leave with no config change, which reads well for the decentralization story. ✅ No fleet list to keep in sync. ❌ A slow-to-appear peer looks like no peer, and subscriptions must be created and removed at run time. Effort: human ~half day / CC ~20 min.

State: approved
Actual answer: A) Static peer list (D9, 2026-11-04)
Accepted scope: each `follower_planner_node` takes `robot`, `sysid` and `peers` params from the swarm launch and subscribes to `/<peer>/pose` for each named peer. A named peer with no pose yet is logged as absent. No graph scanning.
History: none

### R4: When a planner decides its next sector
Finding: 4, P1, confidence 8/10, `docs/next-session-prompt.md` step 5 ("how asynchronous planners step on their own timers"), native review.
Plan baseline: original proposal; step 5 leaves it open. The thesis iterates in discrete moves and has no notion of flight time.
Runtime evidence: an 8 m move took about 10 s wall time at RTF 0.33-0.45 in SITL. Sector centres are 5 m apart. Timer behaviour under `use_sim_time` at low RTF not measured for this node; the repo already records that sim-time timers freeze when no `/clock` is published (`src/uav_mapping/launch/heatmap.launch.py:44`).
Comparison grid:
| Choice | Current | A: 1 Hz check, commit on arrival | B: fixed move tick | C: arrival event only |
|---|---|---|---|---|
| R4 when a new sector is chosen | undecided | on arrival, or at once if the current goal becomes blocked | every T seconds regardless of position | only on arrival |
| Reacts to a peer taking the goal mid-flight | n/a | yes, within 1 s | at next tick | no, until arrival |
| Reacts to a heatmap change mid-flight | n/a | only if the goal is blocked; otherwise on arrival | at next tick | on arrival |
| Risk of changing goal repeatedly in flight | n/a | low | high if T is shorter than flight time | none |
| Work | none | ~20 lines (human ~1h / CC ~5 min) | ~10 lines + T tuning (human ~1h / CC ~5 min) | ~10 lines (human ~30 min / CC ~3 min) |
Question D10:
D10 — When should a flying follower pick its next sector?
Project/branch/task: follower planner architecture on main; step 5 of the plan, stepping.
ELI10: The thesis moves drones in neat turns, but real drones take seconds to fly between sectors and each planner runs on its own clock. Option A checks once a second but only commits to a new sector when the drone has arrived, or straight away if another drone just claimed its target. B moves on a fixed beat, which can redirect a drone before it gets anywhere. C decides only on arrival, so a drone keeps flying at a sector someone else took.
Stakes if we pick wrong: B can leave drones changing their minds mid-air and never arriving; C can send two drones toward one spot for the whole flight.
Recommendation: A because it keeps flights committed and still reacts when a target is taken.
Note: options differ in kind, not coverage — no completeness score.
Net: committed flights with one exception versus simpler rules with a worse failure.
Header: Stepping
Options:
A) 1 Hz check, commit (recommended)
✅ A drone finishes its flight unless the target is taken, so it doesn't dither between sectors. ✅ One timer callback, so no threads or locks, and it runs on simulated time so it slows with the sim. ✅ The decision function stays pure and testable in the library. ❌ A hotter sector appearing mid-flight waits until arrival, a few seconds at 5 m spacing. Effort: human ~1h / CC ~5 min.
B) Fixed move tick
✅ Closest to the thesis's discrete iterations, so it is easy to explain. ✅ Fewest states in the node: no arrival check drives the decision. ❌ A tick shorter than flight time redirects drones mid-air, and the right T changes with grid spacing. Effort: human ~1h / CC ~5 min.
C) Arrival event only
✅ Simplest rule and can't change its mind in flight. ✅ No timer logic beyond checking position. ❌ A drone keeps flying at a sector a peer already holds, then has to leave on arrival. Effort: human ~30 min / CC ~3 min.

State: approved
Actual answer: A) 1 Hz check, commit (D10, 2026-11-04)
Accepted scope: one 1 Hz timer in `follower_planner_node`, on sim time, default single-threaded executor. A new sector is chosen on arrival, or immediately if the current goal becomes blocked by a peer. A heatmap change alone does not redirect a drone in flight. The decision is a pure library function.
History: none

### R5: Two drones racing for one sector
Finding: 5, P1, confidence 7/10, `docs/next-session-prompt.md` step 5 ("avoiding two drones racing into one sector beyond what the utility penalty covers"), native review.
Plan baseline: original proposal; step 5 names the problem and leaves the mechanism open. The thesis utility uses each peer's current sector `x_-a` only.
Runtime evidence: unknown in SITL. The algorithm probe steps drones one at a time, so it cannot show this race. By construction, position alone does not reveal where a peer is heading, and each planner runs on its own timer.
Comparison grid:
| Choice | Current | A: share goals, lower sysid keeps | B: positions only |
|---|---|---|---|
| R5 what peers count as occupied | current sector only | current sector and announced goal sector | current sector only |
| Same goal picked at the same moment | undecided | higher sysid re-plans at its next 1 Hz check | both fly until one enters the sector, then the other re-plans |
| New topic | none | `/<robot>/goal_sector`, `geometry_msgs/PointStamped`, RELIABLE + TRANSIENT_LOCAL | none |
| Fidelity to thesis utility | n/a | extends `x_-a` with intent | exact |
| Work | none | ~20 lines + library test (human ~2h / CC ~10 min) | none |
Question D11:
D11 — Should followers announce where they're heading, not just where they are?
Project/branch/task: follower planner architecture on main; step 5 of the plan, racing.
ELI10: The thesis rule only looks at where other drones are now. A drone 3 sectors away flying to the best sector doesn't count as near it, so a second drone can pick the same sector and both fly there, wasting a flight each. If each drone also publishes its goal sector, others treat that sector as taken. If two pick the same sector in the same second, the higher-numbered drone gives way. Each drone still decides alone; they only share what they intend.
Stakes if we pick wrong: positions-only wastes flight time and shows up in the eval as slower finds, though altitude layers keep it safe; sharing goals adds one topic and a small departure from the thesis formula.
Recommendation: A because the extra topic is small and the wasted flights would make a 3-drone run look uncoordinated.
Note: options differ in kind, not coverage — no completeness score.
Net: one extra topic versus an exact thesis formula with wasted flights.
Header: Intent
Options:
A) Share goals (recommended)
✅ A sector being flown to counts as taken, so drones split up before they fly rather than after. ✅ Each drone still decides alone, so there's still no coordinator and a lost peer doesn't stop anyone. ✅ Uses a standard message type, with no custom package. ❌ Departs slightly from the thesis's utility, so the write-up must say so. Effort: human ~2h / CC ~10 min.
B) Positions only
✅ Exactly the thesis formula, with nothing extra to explain or test. ✅ No new topic and no tie-break rule. ❌ Two drones can fly the full distance to one sector before one turns away; I have not measured how often this happens. Effort: none.

State: approved
Actual answer: A) Share goals (D11, 2026-11-04)
Accepted scope: each node publishes its goal sector centre on `/<robot>/goal_sector` (`geometry_msgs/PointStamped`, RELIABLE + TRANSIENT_LOCAL) and subscribes to each named peer's. Peers' current and goal sectors both count as occupied in the utility. If two drones hold the same goal, the higher sysid re-plans at its next 1 Hz check. Includes a library test for the tie-break. The write-up states this departs from the thesis utility.
History: none

### R6: Arrival criterion and handoff to landing_node
Finding: 6, P1, confidence 8/10, `docs/next-session-prompt.md` step 1 ("arrival criterion and handoff to the later landing_node") and step 3 ("3-5 m arrival"), native review.
Plan baseline: thesis-derived default of 3-5 m arrival as a param (step 3). The handoff mechanism is open.
Runtime evidence: `cmd_gps_pose` reached within 0.4 m of target in about 10 s wall time, so 3 m is comfortably reachable. A 5 m sector has a half-width of 2.5 m and a corner distance of 3.54 m from its centre, so a 5 m radius would count a drone as arrived while it is still in the neighbouring sector. `landing_node` does not exist yet.
Comparison grid:
| Choice | Current | A: 3 m + status and enable topics | B: 3 m + planner calls landing | C: 5 m + status and enable topics |
|---|---|---|---|---|
| R6 arrival radius (horizontal, metres) | 3-5 m, unset | 3.0 default, param | 3.0 default, param | 5.0 default, param |
| Status output | none | `/<robot>/planner_status`, `std_msgs/String`: ASCENDING, ARRIVED, HOLDING | none needed | same as A |
| How planning stops | none | `/<robot>/planner_enable`, `std_msgs/Bool`, planner stops sending goals when false | planner invokes landing itself | same as A |
| Planner knows about landing | n/a | no | yes | no |
| Work | none | ~20 lines + test (human ~1h / CC ~5 min) | ~20 lines now, more when landing exists | ~20 lines + test (human ~1h / CC ~5 min) |
Question D12:
D12 — What counts as "arrived", and how does a drone hand over to landing later?
Project/branch/task: follower planner architecture on main; step 1 blocking question on arrival and handoff.
ELI10: The planner needs a rule for "I've reached this sector", and later something else has to take over to land. A 3 m radius is the thesis's lower bound and keeps the drone inside its 5 m sector; at 5 m it can count as arrived while still over the next sector. For handoff, the planner just publishes its status and obeys an on/off switch. Whatever comes later, a mission manager or landing node, flips the switch and takes over, so the planner never needs to know landing exists.
Stakes if we pick wrong: a sloppy radius means the drone reports arriving at the wrong sector; a planner that calls landing directly must be rewritten when the mission state machine arrives on Nov 24.
Recommendation: A because it fits the sector geometry and lets the Nov 24 mission logic plug in with no planner changes.
Note: options differ in kind, not coverage — no completeness score.
Net: small clean interface now versus either a loose radius or later rewrites.
Header: Arrival
Options:
A) 3 m + status/enable (recommended)
✅ 3 m keeps an arrived drone within about its own sector and sits in the thesis's 3-5 m range. ✅ Status and enable are plain topics, so you can test handoff from the command line today. ✅ The planner stays unaware of landing, so the mission logic plugs in unchanged. ❌ Two small topics exist before anything but tests uses the enable switch. Effort: human ~1h / CC ~5 min.
B) 3 m + planner calls landing
✅ One less topic pair, and the flow is easy to follow in one file. ✅ Mirrors the thesis's traversal-then-land sequence directly. ❌ Couples the planner to a node that doesn't exist, and conflicts with mission logic deciding when to land. Effort: ~20 lines now, more when landing exists.
C) 5 m + status/enable
✅ Top of the thesis's range, so arrival triggers earlier and flights feel quicker. ✅ Same clean handoff as A. ❌ Can report arrival while still over the neighbouring sector, which breaks the sector logic. Effort: human ~1h / CC ~5 min.

State: approved
Actual answer: A) 3 m + status/enable (D12, 2026-11-04)
Accepted scope: `arrival_radius` param, default 3.0 m horizontal. The node publishes `/<robot>/planner_status` (`std_msgs/String`: ASCENDING, ARRIVED, HOLDING) and obeys `/<robot>/planner_enable` (`std_msgs/Bool`), sending no goals while false. The planner has no knowledge of landing. Covered by tests.
History: none

### R7: Post-find peak suppression
Finding: 7, P1, confidence 8/10, `docs/roadmap.md` "Open engineering questions" and architecture row ("single owner of the posterior ... followers only read it"), native review.
Plan baseline: roadmap says `belief_map_node` is the single owner of the posterior and followers only read it. Suppression itself is an open question.
Runtime evidence: `belief_map_node` does not exist yet. Under `leader_node` today, a repeatedly detected cell stays clamped at 97 (`p_clamp`, `src/uav_mapping/uav_mapping/leader_node.py:183`), so with no suppression followers keep holding a found target's sector. On the Nov 15 stub heatmap, nothing is ever found, so this has no effect until Nov 24.
Comparison grid:
| Choice | Current | A: belief map lowers it | B: found-sector mask in planner | C: each planner remembers visits |
|---|---|---|---|---|
| R7 who suppresses a found peak | nobody | `belief_map_node`, Nov 24 | planner library, from a `/found_sectors` topic | each planner privately |
| Planner code for suppression | none | none | mask input + subscriber, ~15 lines | visit memory, ~15 lines |
| Drones agree on what's found | n/a | yes, one map | yes, one topic | no, each has its own memory |
| Posterior stays a pure probability | n/a | no, published value means "worth visiting" | yes | yes |
| Planner proof this week | none | library test: peak drops, drones leave | library test: masked peak ignored | library test: visited peak ignored |
Question D13:
D13 — After a target is found, who stops other drones flying to it?
Project/branch/task: follower planner architecture on main; step 1 blocking question on peak suppression.
ELI10: Once a target is confirmed, its sector still looks like the hottest spot, so every free drone would pile in. Something must make it stop attracting them. Your roadmap already says one node owns the map and followers only read it, so the simplest answer is that the map's owner lowers that sector, and the planner just sees a lower number. The catch is that the published value then means "worth visiting" rather than strictly "probability", which matters if the eval reads it directly.
Stakes if we pick wrong: per-drone memory lets drones disagree about what's been found; a mask adds a topic the planner has to handle before anything publishes on it.
Recommendation: A because it needs no planner code and keeps the roadmap's one-owner rule, and belief_map_node can publish the true posterior separately if the eval needs it.
Note: options differ in kind, not coverage — no completeness score.
Net: one owner and no planner code versus a purer posterior with more wiring.
Header: Suppression
Options:
A) Belief map lowers it (recommended)
✅ Planner needs no suppression code, so nothing is added before the Nov 15 trigger. ✅ Keeps the roadmap's single-owner rule, so every drone sees one answer. ✅ A library test still proves drones leave when a peak drops. ❌ The published value is no longer a pure probability, so the eval may need a second topic from belief_map_node. Effort: human ~20 min / CC ~2 min this week.
B) Found-sector mask
✅ The heatmap stays a pure probability that the eval can use directly. ✅ Testable today in the library with a list of masked sectors. ❌ Adds a topic and a subscriber to the planner with no publisher until Nov 24. Effort: human ~1h / CC ~5 min.
C) Each planner remembers
✅ Works with no other node and no new topic. ✅ Also stops a drone revisiting empty sectors on a static map. ❌ Drones can disagree about what's been found, and memory is lost on restart. Effort: human ~1h / CC ~5 min.

State: approved
Actual answer: A) Belief map lowers it (D13, 2026-11-04)
Accepted scope: `belief_map_node` (Nov 24) lowers a confirmed sector in the map followers read and keeps it low against re-detections of the same target. No suppression code in the planner. This week: one library test that drones leave a sector when its value drops. If the eval needs the true posterior, `belief_map_node` publishes it on a second topic.
History: none

### R8: Termination with an unknown target count
Finding: 8, P1, confidence 8/10, `docs/roadmap.md` "Open engineering questions" and eval design ("Timeout 15 sim-minutes"), native review.
Plan baseline: open question. Roadmap fixes a 15 sim-minute eval timeout and says the leader always completes the full sweep.
Runtime evidence: none needed for the planner; the mission state machine does not exist yet. `leader_node` defines an unobserved cell's first-update prior as p = 0.5 (`src/uav_mapping/uav_mapping/leader_node.py:327`), so 0.5 is a natural "not ruled out" line. At RTF 0.11, 15 sim-minutes would be about 2.3 wall-hours in Gazebo; the eval's statistics come from the 2D sim, so this affects showcase runs only.
Comparison grid:
| Choice | Current | A: mission decides, planner never stops itself | B: planner stops itself | C: timeout only |
|---|---|---|---|---|
| R8 who ends the mission | undecided | mission state machine | each planner | mission state machine |
| End condition | none | sweep complete AND every unsuppressed sector below 0.5 for 30 s, OR timeout | every utility it can see below a threshold | timeout |
| Planner code | none | none beyond the R6 enable switch | threshold logic and self-stop | none |
| Drones could stop at different times | n/a | no | yes | no |
| Ends early when search is done | n/a | yes | yes | no |
Question D14:
D14 — How does a mission know it's finished when nobody knows how many targets exist?
Project/branch/task: follower planner architecture on main; step 1 blocking question on termination.
ELI10: With an unknown number of targets, you can't stop at "found N". A sensible rule is: the leader has finished its full sweep, and no sector still looks more likely than not to hold a target, or time is up. That's a mission-level judgement, so the planner shouldn't make it. The planner keeps doing its job until the mission flips the enable switch from the arrival decision.
Stakes if we pick wrong: planner-level stopping lets drones quit at different times on different views of the map; timeout-only makes every run take the full time, which hurts time-based results and the demo.
Recommendation: A because it needs no planner code now and matches the roadmap's coverage guarantee.
Note: options differ in kind, not coverage — no completeness score.
Net: one mission-level rule versus simpler code that ends runs badly.
Header: Termination
Options:
A) Mission decides (recommended)
✅ The planner needs no termination code, only the enable switch already approved. ✅ The rule uses the sweep the roadmap already guarantees, so coverage comes first. ✅ Clear to state in the write-up: sweep done, nothing above 50% for 30 s, or timeout. ❌ The 0.5 and 30 s values are untested until the state machine exists on Nov 24. Effort: human ~10 min / CC ~1 min this week.
B) Planner stops itself
✅ Works before the mission state machine exists. ✅ Each drone remains fully independent, which suits the decentralized story. ❌ Drones can stop at different times from different map views, and a stopped drone can't return if new evidence arrives. Effort: human ~1h / CC ~5 min.
C) Timeout only
✅ Trivial, and exactly matches the eval's stated 15 sim-minute limit. ✅ No thresholds to tune or defend. ❌ Every Gazebo run lasts the full time, about 2.3 wall-hours at RTF 0.11. Effort: human ~10 min / CC ~1 min.

State: approved
Actual answer: A) Mission decides (D14, 2026-11-04)
Accepted scope: no termination code in the planner. Recorded rule for the Nov 24 mission state machine: end when the leader's sweep is complete and every unsuppressed sector is below 0.5 for 30 s, or at timeout, then disable planners through the R6 enable switch. The 0.5 and 30 s values are provisional until tested.
History: none

### R9: Who arms the drones, and under which RMW
Finding: 9, P1, confidence 9/10, `docker/docker-compose.yml:60` (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) and `docs/docker-notes.md` gotcha 22, native review.
Plan baseline: step 4 says "get one follower to take off in GUIDED"; who does it is unstated. Gotcha 22 records that `/ap/v1/*` services hang under CycloneDDS, and clients must use Fast DDS.
Runtime evidence: today, mode, arm and takeoff services on `/ap/v1` and `/ap/v2` all returned success with a Fast DDS client. `cmd_gps_pose` and the `goal_lla` latched read both worked under CycloneDDS. The project nodes have not been run under Fast DDS on this image.
Comparison grid:
| Choice | Current | A: separate helper | B: planner calls services | C: switch default to Fast DDS |
|---|---|---|---|---|
| R9 who arms and takes off | manual CLI | `arm_takeoff` executable, one per drone, exits when airborne | planner node | `arm_takeoff` helper, same as A |
| RMW for planner | Cyclone | Cyclone, unchanged | Fast DDS, set per node | Fast DDS, project-wide |
| RMW for arming | Fast DDS | Fast DDS, set in launch for that process only | Fast DDS | Fast DDS |
| Verified nodes affected | none | none | none | localization and leader must be re-verified |
| Work | none | ~40 lines (human ~2h / CC ~10 min) | ~40 lines in node (human ~2h / CC ~10 min) | ~40 lines + one compose line + re-verify (human ~half day / CC ~20 min) |
Question D15:
D15 — What arms and launches the followers, given the service hang under the default RMW?
Project/branch/task: follower planner architecture on main; step 4 of the plan, first takeoff.
ELI10: An RMW is the messaging library underneath ROS 2, and you can pick which one. Arming and takeoff are service calls, and those hang under the container's default, CycloneDDS, but work under Fast DDS. Position goals are plain topics, which I tested today under the default. So the planner doesn't need to change RMW if something else does the arming: a tiny helper that arms, takes off and exits, and only that helper uses Fast DDS. Later the mission state machine can take over its job.
Stakes if we pick wrong: mixing arming into the planner forces it onto a different RMW and blurs its job; a project-wide switch needs re-verifying two working nodes during a tight week.
Recommendation: A because nothing that works today changes and the planner stays decision-only.
Note: options differ in kind, not coverage — no completeness score.
Net: one small extra process versus re-verifying or bloating what works.
Header: Arming
Options:
A) Separate arm helper (recommended)
✅ The planner stays decision-only and stays on the RMW its command path was tested on. ✅ Only the helper needs Fast DDS, set in launch for that one process. ✅ The Nov 24 mission state machine can absorb it. ❌ One more executable, plus two RMWs in one launch, which is the service-hang workaround made permanent for now. Effort: human ~2h / CC ~10 min.
B) Planner calls services
✅ One process per drone, so there's less to launch. ✅ The planner can re-arm itself if a drone disarms. ❌ The planner runs on Fast DDS, where its other topics are untested, and it takes on flight-mode duties. Effort: human ~2h / CC ~10 min.
C) Switch default to Fast DDS
✅ One RMW everywhere removes the gotcha for all future shells. ✅ The helper needs no special launch setting. ❌ Localization and leader nodes must be re-verified, and the Dockerfile says CycloneDDS was chosen deliberately. Effort: human ~half day / CC ~20 min.

State: approved
Actual answer: A) Separate arm helper (D15, 2026-11-04)
Accepted scope: an `arm_takeoff` executable in `uav_planning`, one per drone, sets GUIDED, arms and takes off through `/ap/vN/*` services, then exits. The swarm launch sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` for that process only. `follower_planner_node` stays on the container default and never calls services.
History: none

### R10: A stay action for DGA
Finding: 10, P1, confidence 10/10, `docs/thesis.pdf` section 3.1.1.4 ("moved to the sector with the highest utility in L(x_a)") and action set `A_a = {a_l | a_l in L(x_a)}`, native review.
Plan baseline: thesis rule, carried into `docs/next-session-prompt.md` ("evaluate all 8 neighbors' utility and move to the highest"), which has no stay action.
Runtime evidence: algorithm probe variant A: 100% of runs still moving at the end at every grid size. Trace of one drone, one peak on 8x8: (4,4), (3,4), (4,4), (3,4), (4,4), (5,5). Not run in SITL.
Comparison grid:
| Choice | Current | A: stay + margin | B: stay only | C: thesis literal |
|---|---|---|---|---|
| R10 current sector is a candidate | no | yes | yes | no |
| Margin before moving (probability units) | none | move only if best > current + 0.02, param | none | none |
| Behaviour at a peak | leaves, returns, repeats | holds | holds | leaves, returns, repeats |
| Small map noise | n/a | ignored below the margin | can trigger a flight | can trigger a flight |
| Work | none | ~5 lines + 2 tests (human ~30 min / CC ~3 min) | ~3 lines + 1 test (human ~20 min / CC ~2 min) | none |
Question D16:
D16 — Should a drone be allowed to stay put?
Project/branch/task: follower planner library on main; DGA step function.
ELI10: As written, the thesis rule always moves a drone to one of its 8 neighbours; "stay here" isn't an option. So a drone that reaches the best sector has to leave, then comes back, forever. My probe shows this directly: 100% of runs were still moving at the end. The fix is to count the current sector as an option, and only move when a neighbour is better by a small margin, so tiny map changes don't trigger a flight.
Stakes if we pick wrong: without stay, drones never settle and "arrived" never means anything; without a margin, a 1% map wobble can send a drone to the next sector.
Recommendation: A because a drone must hold a peak, and the map's values are whole percentages, so a 2-point margin filters rounding noise.
Completeness: A=10/10, B=7/10, C=3/10
Net: a small departure from the thesis text versus a controller that never settles.
Header: Stay rule
Options:
A) Stay + 2% margin (recommended)
✅ Drones hold a peak and ignore wobble smaller than the margin, so they stop wasting flights. ✅ One param, testable as a pure function in the library. ❌ A departure from the thesis text the write-up must state; a true gain under 2 points is ignored. Effort: human ~30 min / CC ~3 min.
B) Stay only
✅ Smallest fix that solves the bounce, with no extra param. ✅ Moves on any real improvement, however small. ❌ Map values changing by 1 point can trigger a flight between near-equal sectors. Effort: human ~20 min / CC ~2 min.
C) Thesis literal
✅ Exactly the written algorithm, so nothing to explain. ✅ No code beyond the base rule. ❌ Never settles: 100% of probe runs still moving, and the Nov 15 demo would show drones bouncing around peaks. Effort: none.

State: approved
Actual answer: A) Stay + 2% margin (D16, 2026-11-04)
Accepted scope: the current sector is always a candidate in the step function, and a drone moves only if the best option beats it by more than `move_margin`, default 0.02 probability, as a param. Tests cover holding a peak and ignoring a change below the margin. The write-up states this departs from the thesis text.
History: none

### R11: Peer penalty covers the peer's own sector and its neighbours
Finding: 11, P2, confidence 9/10, `docs/thesis.pdf` section 3.1.1.2 ("Multiple drones can occupy the same sector"; `u = 0` for `s_ij` in `L(x_-a)`), native review.
Plan baseline: approved. D5 keeps the thesis neighbourhood penalty as the default; D11 counts a peer's current and goal sectors as occupied.
Runtime evidence: probe with the closed neighbourhood showed 0% stacked. Two adjacent peaks with two drones: both held 0%, one held 100%. Two peers zero out 63% of a 4x4 map for the third drone, 22% of 8x8, 11% of 12x12.
State: approved
Actual answer: no new question; carried from D5 and D11 (2026-11-04)
Accepted scope: a sector is blocked if it is a peer's current or goal sector or one of their 8 neighbours. Known limit, stated in the write-up: two followers cannot hold adjacent sectors, so of two side-by-side targets only one gets a follower. At 10 m the holder's camera footprint is larger than one sector, so the neighbouring target should still be in view; this is not yet verified.
History: none

### R12: How a drone reaches a peak on a sparse map
Finding: 12, P1, confidence 8/10, `src/uav_mapping/uav_mapping/leader_node.py:183` (`p_clamp` 0.97) and `CLAUDE.md` ("Algorithm: Distributed Gradient Ascent (DGA)"), native review.
Plan baseline: approved stack in `CLAUDE.md`, marked locked: DGA, with BLL as a comparison baseline. This question challenges how DGA moves, on evidence; only the user can change it.
Runtime evidence: second probe, 100 seeds, stay + closed neighbourhood, sectors flown per drone in brackets. Slope gamma 0.8: 4x4 86% all 3 held (1.2), 8x8 78% (2.7), 12x12 80% (4.6). Slope gamma 0.5: 12x12 32% (2.7). Direct to best open sector: 4x4 98% (1.3), 8x8 100% (2.9), 12x12 100% (4.7). Thesis local steps from the first probe, with a stay action: 4x4 38%, 8x8 0%, 12x12 0%. Limits: the cause of the slope misses was not diagnosed and tuning might narrow the gap; maps were sparse only, smooth LLM-style priors not tested; pure Python, no flight dynamics; direct mode ignores distance when choosing among unequal peaks, also untested.
Comparison grid:
| Choice | Current | A: direct to best open sector | B: thesis steps + slope | C: thesis steps only |
|---|---|---|---|---|
| R12 candidates each decision | 8 neighbours (thesis) | every open sector on the grid | 8 neighbours, on a value field with slope | 8 neighbours, raw values |
| All 3 peaks held (4x4 / 8x8 / 12x12) | n/a | 98% / 100% / 100% | 86% / 78% / 80% | 38% / 0% / 0% |
| Flight between decisions | n/a | one straight flight to the target | 5 m hops | 5 m hops |
| Name in pitch and CLAUDE.md | "gradient ascent" | must change; thesis DGA kept as library baseline | stays, with a stated extension | stays |
| New param | none | none | `gamma`, default 0.8 | none |
| Library still contains | n/a | thesis DGA and BLL as baselines | BLL | BLL |
| Work | none | ~15 lines + thesis DGA as baseline + tests (human ~2h / CC ~10 min) | ~15 lines + tests (human ~2h / CC ~10 min) | ~10 lines + tests (human ~1h / CC ~5 min) |
Question D17:
D17 — How should a drone get to a peak the map shows, given there's no gradient to climb?
Project/branch/task: follower planner library on main; this challenges the 'gradient ascent' line in CLAUDE.md, which is yours to change.
ELI10: Gradient ascent means "step to whichever neighbour is higher". That needs a hill. Our heatmap has spikes: one sector at 97% and everything around it at 3%, so a drone two sectors away sees only flat ground and sits still. In my probe the thesis rule held all three peaks 0% of the time on larger grids. I tested two fixes. B adds an artificial slope around each spike so stepping works again: 78-86%. A skips stepping and has each drone pick the best open sector anywhere and fly straight there: 98-100%, with about the same distance. A is better on these numbers, but it isn't really gradient ascent anymore, which affects your pitch and thesis story. These are small sims on spiky maps, not flight tests.
Stakes if we pick wrong: C can't reach targets on the map the leader produces; B keeps the name but misses a peak about 1 run in 5; A means rewording the pitch line and CLAUDE.md, and explaining the change from the thesis.
Recommendation: A because it reaches every peak in the larger-grid runs with simpler logic, it is still distributed with the same utility, and you can report thesis DGA as a measured baseline, which is a stronger story than a tuned slope.
Completeness: A=10/10, B=7/10, C=3/10
Net: a better-performing rule that changes a headline word versus keeping the word and accepting more misses.
Header: Reach peaks
Options:
A) Direct to best open (recommended)
✅ Held all 3 peaks in 98-100% of probe runs at about the same flight distance. ✅ Simpler: no slope field or gamma to tune, one flight per decision, and still each drone decides alone from the shared map. ✅ Thesis DGA stays in the library as a measured baseline. ❌ It is no longer gradient ascent, so the pitch and CLAUDE.md need rewording; distance-blind choice among unequal peaks is untested. Effort: human ~2h / CC ~10 min.
B) Thesis steps + slope
✅ Keeps "gradient ascent" true in the pitch and stays closest to the thesis's local moves. ✅ Discounting by distance gives nearer peaks a natural preference. ❌ Held all 3 peaks in 78-86% of runs, the misses are undiagnosed, and it adds a gamma param (0.5 fell to 32% at 12x12). Effort: human ~2h / CC ~10 min.
C) Thesis steps only
✅ Exactly the thesis's local rule, so no explanation is needed. ✅ Fewest lines and no extra param. ❌ Held all 3 peaks 0% of the time on 8x8 and 12x12, so followers mostly never reach what the leader found. Effort: human ~1h / CC ~5 min.

State: approved
Actual answer: A) Direct to best open (D17, 2026-11-04)
Accepted scope: the flying rule picks the highest-value open sector on the whole grid and flies straight to it, with the R11 penalty and the R10 margin unchanged. Thesis DGA (8-neighbour steps) stays in the library beside BLL as a measured baseline. `docs/roadmap.md` and `CLAUDE.md` are reworded so they no longer say the followers fly by gradient ascent; the thesis algorithm is described as the baseline. Supersedes the "DGA only at runtime" wording in the scope record: the node runs this direct rule. The user changed the stack line marked locked in `CLAUDE.md`.
History: none

### R13: Flat or all-unknown maps
Finding: 13, P1, confidence 7/10, `src/uav_mapping/uav_mapping/leader_node.py:327` (`cur = 0.0  # first observation: prior p=0.5`) and `docs/next-session-prompt.md` step 1, native review.
Plan baseline: open question, listed as blocking. OccupancyGrid `-1` marks unknown cells, and the standalone heatmap is mostly unknown (`docs/next-session-prompt.md`).
Runtime evidence: `-1` cells verified in `/heatmap` today. With the margin rule on an all-equal map, a drone holds unless a peer's zone covers its sector; the first probe showed any drone moved in 68% of 4x4 runs and 15% of 12x12 runs. Exploration via negative evidence was not simulated, and depends on followers' footprints reaching `belief_map_node`, due Nov 24.
Comparison grid:
| Choice | Current | A: unknown = 0.5 | B: unknown = 0 | C: unknown = 0.5 + follower sweep |
|---|---|---|---|---|
| R13 value of a `-1` cell | undecided | 0.5, same as `leader_node`'s prior | 0 | 0.5 |
| Swept-empty 3% vs unknown | n/a | unknown preferred | swept-empty preferred, marginally | unknown preferred |
| All-flat map | n/a | hold until evidence changes values | hold | fly a coverage pattern |
| Ties | n/a | nearest sector, then lowest index | nearest, then lowest index | nearest, then lowest index |
| Fair uniform-prior baseline | n/a | only once follower footprints feed the map | no | yes, but duplicates the leader |
| Work | none | ~5 lines + tests (human ~30 min / CC ~3 min) | ~3 lines + tests (human ~20 min / CC ~2 min) | ~50 lines + tests (human ~1 day / CC ~30 min) |
Question D18:
D18 — What should followers do with sectors nobody has looked at, or a map with no peaks?
Project/branch/task: follower planner library on main; step 1 blocking question on flat and unknown maps.
ELI10: Many sectors start as "unknown". The planner has to give them a number. Using 50%, the same starting belief the leader uses, means a follower prefers an unexplored sector over one already swept empty, but still prefers a real detection. On a completely flat map nothing beats anything, so followers hold. They start moving once evidence arrives: when a follower's camera sees nothing below it, that sector drops and the next one wins. That last step only works once follower footprints feed the map, around Nov 24.
Stakes if we pick wrong: at 0, followers never explore and the uniform-prior baseline in your eval is unfairly weak; a follower sweep pattern is a day of work that duplicates the leader. With A, if follower footprints slip, the uniform baseline is unfair too, and I haven't simulated this yet.
Recommendation: A because it matches the leader's own prior and needs no extra behaviour, with the eval-sim fairness check listed as required.
Completeness: A=8/10, B=3/10, C=10/10
Net: simple consistent values relying on later evidence versus a full coverage behaviour now.
Header: Unknown map
Options:
A) Unknown = 50% (recommended)
✅ Same number the leader already assumes, so one consistent meaning across nodes. ✅ Followers naturally prefer unexplored sectors to swept-empty ones, with no new logic. ❌ On a flat map they hold until follower footprints arrive around Nov 24, and the baseline's fairness isn't simulated yet. Effort: human ~30 min / CC ~3 min.
B) Unknown = 0%
✅ Followers go only where something was seen, which is easy to explain. ✅ Fewest lines, with a quiet stub map. ❌ Followers never explore, and a language prior with unknown cells around it loses most of its reach. Effort: human ~20 min / CC ~2 min.
C) 50% + follower sweep
✅ Followers do something useful even on a flat map, and the uniform baseline is fair today. ✅ Doesn't depend on Nov 24 work. ❌ Adds a coverage behaviour that duplicates the leader's guaranteed sweep, before the Nov 15 trigger. Effort: human ~1 day / CC ~30 min.

State: approved
Actual answer: A) Unknown = 50% (D18, 2026-11-04)
Accepted scope: the library maps `-1` to 0.5. Ties go to the nearest sector, then the lowest index, so the choice is deterministic. On a flat map a drone holds unless a peer's zone covers its sector. Required follow-up: the 2D eval sim must check that a uniform-prior run explores once follower footprints feed the map, before any uniform-baseline result is reported.
History: none

### R14: The map the Nov 10 and Nov 15 milestones fly on
Finding: 14, P1, confidence 8/10, `src/uav_mapping/launch/heatmap.launch.py` (`grid_width` default 4), `src/uav_mapping/uav_mapping/fake_detection_publisher.py` (one synthetic pixel) and `docs/roadmap.md` milestone "3 followers ascending the stub heatmap", native review.
Plan baseline: "stub heatmap" with thesis defaults (4x4 at 5 m). No stub with multiple peaks exists: the current stub yields one hot cell, driven by a single detection through `leader_node`.
Runtime evidence: camera intrinsics fx = 205.5 at 640x480 give a nadir footprint of about 31 x 23 m at 10 m, larger than the 20 x 20 m arena. On a 4x4 grid two peers zero out 63% of the map for the third drone, against 22% at 8x8. An 8 m move took about 10 s wall time, so crossing 40 m takes roughly a minute at that RTF.
Comparison grid:
| Choice | Current | A: 8x8 stub | B: 4x4 stub | C: 12x12 stub |
|---|---|---|---|---|
| R14 milestone grid | 4x4 default | 8x8 at 5 m, 40 x 40 m | 4x4 at 5 m, 20 x 20 m | 12x12 at 5 m, 60 x 60 m |
| Map source | `leader_node` + one stub detection | new `stub_heatmap_publisher`, peaks as a param, same QoS as `/heatmap` | same new stub | same new stub |
| `leader_node` defaults | 4x4 | unchanged | unchanged | unchanged |
| Share of map blocked by 2 peers | 63% | 22% | 63% | 11% |
| Longest flight, rough wall time at today's rate | n/a | ~1 min | ~30 s | ~1.5 min |
| Work | none | ~40 lines (human ~1h / CC ~5 min) | ~40 lines (human ~1h / CC ~5 min) | ~40 lines (human ~1h / CC ~5 min) |
Question D19:
D19 — What map should the Nov 10 and Nov 15 flights use?
Project/branch/task: follower planner milestones on main; the plan says "stub heatmap" without saying which.
ELI10: To show coordination, the map needs several peaks. Today's stub makes just one hot sector, so I'd add a tiny publisher that serves a map you choose. The size matters too. The thesis's 4x4 arena is 20 x 20 m, and at 10 m altitude one camera frame already sees more than that, so three drones on it barely need to move, and two of them block 63% of the map from the third. 8x8 gives room to show real spreading without long flights.
Stakes if we pick wrong: 4x4 can pass the milestone without really testing coordination; 12x12 costs more wall time per run on a slow sim. This doesn't change leader defaults, only the milestone map.
Recommendation: A because it shows spreading clearly and keeps flights near a minute of wall time.
Note: options differ in kind, not coverage — no completeness score.
Net: a convincing demo versus thesis-exact size or longer test runs.
Header: Stub map
Options:
A) 8x8 with new stub (recommended)
✅ Enough room that three drones have real choices, with 22% of the map blocked by peers. ✅ Peaks are a launch param, so you can craft scenarios without code. ❌ Not the thesis's arena size, so state that in the write-up. Effort: human ~1h / CC ~5 min.
B) 4x4 with new stub
✅ Exactly the thesis arena, so results compare directly. ✅ Shortest flights and quickest test cycles. ❌ Two peers block 63% of the map, so it says little about coordination, and one frame covers the arena. Effort: human ~1h / CC ~5 min.
C) 12x12 with new stub
✅ Most room, closest to a realistic search area, and only 11% blocked. ✅ Best-looking video. ❌ Longest flights on a sim already running at RTF 0.11-0.25, so each test run costs more. Effort: human ~1h / CC ~5 min.

State: approved
Actual answer: A) 8x8 with new stub (D19, 2026-11-04)
Accepted scope: a new `stub_heatmap_publisher` executable in `uav_planning` publishes a static `OccupancyGrid` on `/heatmap` with RELIABLE + TRANSIENT_LOCAL, with peaks and geometry as params. The Nov 10 and Nov 15 milestones fly 8x8 at 5 m. `leader_node` defaults stay 4x4. The write-up states the milestone map is not the thesis arena size.
History: none

### R15: Sector indexing: share with leader_node or keep separate
Finding: 15, P3, confidence 9/10, `src/uav_mapping/uav_mapping/leader_node.py:315` (`col = int(math.floor((x - self.ox) / self.res))`; returns `col, row`) and `:407` (cell centre), plus proposed caller `core/planner.py`, native review.
Plan baseline: step 2 asks "decide whether to share a helper or duplicate the few lines".
Runtime evidence: `leader_node` hand-computation verified on 2026-11-02 (hot cell at (1,1) for pose (7.5, 7.5)). Shared-code rubric: one verified existing caller plus one proposed caller; about 7 implementation lines removed from `leader_node`, about 7 added in the library, so net savings about 0 and a new package dependency. `leader_node` tuples are `(col, row)` while its array is `[row, col]`, so conventions must be pinned. `belief_map_node` becomes a third caller on Nov 24.
Comparison grid:
| Choice | Current | A: separate now, share at Nov 24 | B: share now |
|---|---|---|---|
| R15 where planner indexing lives | not written | `core/planner.py` | `core/planner.py` |
| `leader_node` | own lines | untouched | imports from `uav_planning` |
| New package dependency | none | none | `uav_mapping` depends on `uav_planning` |
| Protection against drift | n/a | library test with the verified (7.5, 7.5) -> (1,1) case | single implementation |
| Revisit | n/a | when `belief_map_node` adds a third caller | no need |
| Work | none | ~7 lines + test (human ~20 min / CC ~2 min) | ~7 lines + edit + re-verify leader (human ~1h / CC ~10 min) |
Question D20:
D20 — Should the planner share sector maths with leader_node now, or keep its own copy for the moment?
Project/branch/task: follower planner library on main; step 2 asks for this choice.
ELI10: Turning a position into a sector takes about 7 lines, which `leader_node` already has and the planner also needs. Sharing avoids two copies drifting apart, but it makes a working, verified node depend on a brand-new package, for about zero lines saved. A third user, the belief map, arrives around Nov 24, and that's when sharing starts to pay off. Until then a test can pin the planner to the same case we verified by hand.
Stakes if we pick wrong: separate copies could drift, which the test covers; sharing now puts a verified node at risk during a tight week for no net saving.
Recommendation: A because the saving is about zero today and the third caller gives a natural moment to extract it.
Note: options differ in kind, not coverage — no completeness score.
Net: a small temporary duplicate versus touching verified code now.
Header: Grid maths
Options:
A) Separate until Nov 24 (recommended)
✅ `leader_node`, verified on 2026-11-02, isn't touched during the planner push. ✅ A test pins the planner to that same hand-verified case, so a mismatch is caught. ❌ About 7 duplicated lines until `belief_map_node` gives a reason to extract them. Effort: human ~20 min / CC ~2 min.
B) Share now
✅ One implementation from day one, so drift can't happen. ✅ `belief_map_node` can reuse it unchanged later. ❌ A verified node gains a dependency on a new package and must be re-verified, for no net line savings. Effort: human ~1h / CC ~10 min.

State: approved
Actual answer: A) Separate until Nov 24 (D20, 2026-11-04)
Accepted scope: the library has its own world-to-sector and sector-centre functions, with a test pinning the hand-verified (7.5, 7.5) -> (1,1) case. `leader_node` is untouched. Revisit when `belief_map_node` adds a third caller.
History: none

### R16: A peer stops reporting
Finding: 16, P2, confidence 8/10, `docs/thesis.pdf` section 3.1.1.4 ("the system was robust even when one drone momentarily lost communication") and proposed peer subscriptions from R3, code quality review.
Plan baseline: the plan does not address it. Thesis claims tolerance of a momentary loss.
Runtime evidence: unknown; no planner exists. Topics from `/iris_N/navsat` arrive at about 8.5 Hz wall time at RTF 0.25, so `/pose` is frequent and a gap of seconds is meaningful. Under sim time a timeout scales with RTF.
Comparison grid:
| Choice | Current | A: 5 s timeout | B: keep last position forever |
|---|---|---|---|
| R16 a silent peer's zone | n/a | stops blocking after 5 s sim time, param | blocks forever |
| Its goal sector | n/a | released at the same time | blocks forever |
| Logging | n/a | warning when dropped, info when back | none |
| A crashed peer on a peak | n/a | another drone can take it after 5 s | that peak is lost for the run |
| Risk | n/a | a live but silent peer's zone can be entered; altitude layers keep this safe in sim | none extra |
| Work | none | ~10 lines + test (human ~30 min / CC ~3 min) | none |
Question D21:
D21 — If another follower goes silent, should its sector stay reserved?
Project/branch/task: follower planner node on main; the plan doesn't cover this failure.
ELI10: Each drone keeps out of the sectors around other drones. If one drone crashes or stops reporting, its last position would block those sectors forever, including possibly the best one. With a timeout, after 5 seconds of silence others stop reserving its area and log a warning. The risk is a drone that is alive but silent, whose area another might enter; separate altitudes keep that safe in the sim.
Stakes if we pick wrong: no timeout can lose a peak for the whole run from one dropped node; a timeout on a real drone would need a safety check beyond this sim.
Recommendation: A because the thesis claims this tolerance, and one crashed node shouldn't cost a target.
Note: options differ in kind, not coverage — no completeness score.
Net: recovering a blocked peak versus never entering a possibly occupied zone.
Header: Silent peer
Options:
A) 5 s timeout (recommended)
✅ A crashed drone can't hold a peak forever, so the search continues. ✅ Backs up the thesis's claim with a testable behaviour and a logged warning. ❌ A live but silent peer's zone can be entered, which needs more thought off-sim. Effort: human ~30 min / CC ~3 min.
B) Keep last position forever
✅ Never enters a zone that might still be occupied, the conservative choice. ✅ No extra code or param. ❌ One crashed node can silently block a peak for the whole run, with nothing in the logs. Effort: none.

State: approved
Actual answer: A) 5 s timeout (D21, 2026-11-04)
Accepted scope: a peer silent for `peer_timeout`, default 5 s sim time, stops blocking; its goal sector is released too. The node logs a warning on drop and info on return. Library test covers release. With no heatmap received yet, the node reports HOLDING and sends no goals.
History: none

### R17: The BLL "153 iterations" test anchor
Finding: 17, P2, confidence 9/10, `docs/thesis.pdf` section 4.2 ("converge on optimal locations within just 153 iterations") and `docs/next-session-prompt.md` step 2 ("BLL near 153 as the thesis sanity anchor"), test review.
Plan baseline: step 2 proposes testing that BLL iteration counts land near 153.
Runtime evidence: the thesis states no grid size, drone count, map or convergence definition next to the 153. On sparse maps the probe gave a median of 408 iterations to first all-3-on-peaks at 4x4 and did not reach it within 8000 at 12x12; long-run peaks held 0.66/3 and 0.14/3.
Comparison grid:
| Choice | Current | A: test properties, report the number | B: assert near 153 |
|---|---|---|---|
| R17 what a BLL test asserts | near 153 | softmax probability matches the formula; seeded runs repeat; moves stay on the grid | iteration count within a tolerance of 153, on a map chosen to fit |
| Treatment of 153 | sanity anchor | reported beside our measured count, with the setup stated | pass or fail threshold |
| What a failure means | n/a | a real BLL bug | possibly just a different map |
| Work | none | 3 table rows + a printed report (human ~1h / CC ~5 min) | map search + 1 test (human ~half day / CC ~20 min) |
Question D22:
D22 — Should a test require BLL to converge in about 153 iterations?
Project/branch/task: follower planner tests on main; step 2 lists 153 as a sanity anchor.
ELI10: The thesis says BLL converged in 153 iterations, but it doesn't say on what grid, how many drones, what map, or what "converged" meant. On our maps it took 408 at best and sometimes never finished. To make a test pass near 153, I'd have to hunt for a map that gives that number, which proves nothing. Better to test what BLL must always do, like following its probability formula, and report the measured count next to the thesis's number.
Stakes if we pick wrong: a tuned-to-pass test gives false confidence and breaks on harmless changes; dropping the assert means no direct link to the thesis result, only a stated comparison.
Recommendation: A because a test should fail only for a real bug, and the thesis number lacks the setup needed to reproduce it.
Note: options differ in kind, not coverage — no completeness score.
Net: honest properties plus a reported number versus a thesis-matching threshold.
Header: BLL anchor
Options:
A) Test properties, report 153 (recommended)
✅ Fails only when BLL is actually wrong: formula, repeatability and staying on the grid. ✅ The write-up still compares our count with the thesis's, with our setup stated. ❌ No automated check ties us to the thesis's number. Effort: human ~1h / CC ~5 min.
B) Assert near 153
✅ A direct, checkable link to the thesis's published result. ✅ Follows the plan as written. ❌ Needs a map chosen to produce the number, so it proves little and can fail on harmless changes. Effort: human ~half day / CC ~20 min.

State: approved
Actual answer: A) Test properties, report 153 (D22, 2026-11-04)
Accepted scope: BLL tests assert that the move probability matches the softmax formula, that seeded runs repeat, and that moves stay on the grid. The measured iteration count is reported, not asserted, beside the thesis's 153 with our setup stated.
History: none

### R18: Simulation speed for the leader + 3 target
Finding: 18, P1, confidence 7/10, `docs/roadmap.md` milestone "Nov 30 Go/no-go: leader + 3 followers complete a mission in Gazebo", performance review.
Plan baseline: the plan assumes a mission is runnable in Gazebo; speed is not addressed.
Runtime evidence: `/stats` real-time factor, headless, idle on the ground: 1 iris 0.46, 3 iris about 0.25, 4 iris 0.11. With 2 of 3 airborne: 0.33-0.45. `gz sim` sat at about 114% CPU on a 6-core machine, with 7.1 GB RAM and 4.3-4.9 GB of swap in use; three `mavproxy.py` processes at about 9% CPU each. Disabling camera streaming on all three drones left RTF at 0.21-0.29, which does not clear cameras because streaming off may not stop rendering. Readiness check matched nothing in the 1- and 4-iris runs, so those two numbers may include SITLs that had not connected; the trend fits the 3-iris run. Cause not established. At RTF 0.11, a 5 sim-minute mission is about 45 wall-minutes.
Comparison grid:
| Choice | Current | A: timebox before Nov 24 | B: investigate now | C: accept it |
|---|---|---|---|---|
| R18 when speed is investigated | never | one 2 h timebox, after Nov 15, before Nov 24 | one 2 h timebox, before planner code | not investigated |
| Nov 15 milestone | at risk from nothing here | runs at about RTF 0.25 with 3 followers, no leader | same, with possibly better speed | same |
| Candidates to try | none | camera-free follower model, no MAVProxy, more free RAM, re-measure 1 and 4 iris | same | none |
| If still slow by Nov 24 | n/a | decide then: fewer drones in Gazebo, or longer runs cut for video | same | long runs |
| Planner code changes | none | none | none | none |
Question D23:
D23 — When should we look into why the sim slows down so much with more drones?
Project/branch/task: follower planner performance on main; this affects the Nov 30 go/no-go, not the Nov 15 trigger.
ELI10: The sim runs slower as drones are added: about half speed with one, a quarter with three, a ninth with four. Flight is still correct, because the autopilot's clock slows too, but a 5-minute mission with a leader and 3 followers could take about 45 minutes of real time. Nov 15 needs only 3 followers, which worked today. So this doesn't threaten the next milestone, but it matters for the Nov 30 demo. I don't know the cause yet: it could be cameras, memory or physics, and my 4-drone number needs re-measuring.
Stakes if we pick wrong: looking now costs planner hours this week; never looking risks finding out near Nov 30 that the demo takes an hour per run.
Recommendation: A because Nov 15 isn't at risk and the fix needs real multi-drone runs, which exist by then.
Note: options differ in kind, not coverage — no completeness score.
Net: protect this week's planner work versus reducing a later demo risk sooner.
Header: Sim speed
Options:
A) 2 h timebox before Nov 24 (recommended)
✅ Planner work comes first, and 3 followers already run at a usable speed. ✅ A fixed limit stops this eating the week, with a decision point before the go/no-go. ❌ Leaves about a week less to react than investigating today would. Effort: human ~2h / CC ~30 min.
B) Investigate now
✅ Faster runs would speed up all testing this week. ✅ Earliest warning if 4 drones can't work on this machine. ❌ Takes planner hours before the Nov 10 target, with no planner to run as a real load. Effort: human ~2h / CC ~30 min.
C) Accept it
✅ No time spent, and the 2D sim supplies the statistics anyway. ✅ Slow runs can be sped up in video editing. ❌ A leader + 3 run may take about 45 minutes or more each, which makes debugging the mission very slow. Effort: none.

State: approved
Actual answer: A) 2 h timebox before Nov 24 (D23, 2026-11-04)
Accepted scope: one 2-hour investigation after the Nov 15 milestone and before Nov 24: re-measure 1 and 4 iris with a working readiness check, then try a camera-free follower model, no MAVProxy, and more free RAM. If still slow, decide then between fewer drones in Gazebo and longer runs cut for video. No planner code changes.
History: none


## Outside voice

Provider: Codex CLI, plan review, completed 2026-11-04. Verdict: "revise before implementation because the approved contracts can pass the static-flight milestone while deadlocking coordination or failing to observe and complete a search." Eight findings, resolved below. Verified against source before any question: `leader_node.py` has no flight commands (its only publisher is the heatmap), and `leader_node.py:354` (`if len(ground) == 4:`) drops the whole footprint when any corner misses the ground.

Not reopened: Codex's suggestion to defer the BLL and thesis DGA baselines. D4 and D17 approved them, they are small pure functions, and nothing new changes that. Recorded as a disagreement.

Factual correction to `docs/roadmap.md`: the Leader row says only footprints and lane order remain, but no sweep controller exists. The row must say so.

### R19: Conflict rule for goals decided at the same time (reopens R5)
Finding: outside voice 2, P1, confidence 9/10, R5 accepted scope ("If two drones hold the same goal, the higher sysid re-plans"), Codex; reproduced by native probe.
Plan baseline: approved by D11: peers' positions and goals block; only identical goals are tie-broken.
Runtime evidence: new probe, 8x8, 3 drones, flights 1 sector per tick, simultaneous decisions, one tick stale, 200 seeds, peaks at least 2 apart (at least 3 apart in brackets). Approved rule: all 3 held 75% (70%), 49.7 goal changes, still changing at end 25% (30%). Lower sysid keeps a conflicting goal: 95% (96%), 13.3, 5% (4%). That plus "a passing drone does not block" (R20): 100% (100%), 3.9, 0% (0%). Goals always block, passing does not: 36% (41%), 110, 64% (59%). An earlier instant-flight model showed the opposite and is rejected, because it cannot represent the conflict. Pure Python, not flown.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R19 goal conflict | only identical goals tie-broken | any goal inside a lower sysid's zone: the higher sysid yields | unchanged | unchanged until a SITL test | unchanged, row unresolved |
| R20 passing drone | blocks, pending | pending | pending | pending | pending |
| Work | none | ~10 lines + a simultaneous-decision test (human ~1h / CC ~5 min) | none | one 3-drone SITL run after the node exists | none |
Question D24:
D24 — Replace the goal-conflict rule you approved in D11 with one that held up in a better test?
Project/branch/task: follower planner coordination on main; reopens D11 because of new evidence from the outside review.
ELI10: My first probe moved drones one at a time, so it couldn't show this problem. When drones decide at the same moment and flights take time, two drones can pick neighbouring sectors, each sees its goal blocked by the other, both switch, and it repeats. Under the rule you approved, 25-30% of runs never settled. The fix: when goals conflict, the lower-numbered drone always keeps its goal and the other yields. On its own that reaches 95-96% of runs holding all 3 peaks. The next question covers a second change that takes it further.
Stakes if we pick wrong: keeping D11 risks drones visibly swapping goals in the Nov 15 run. This is still a Python sim, not flight.
Recommendation: A because a rule that fails a quarter of runs is worse than a small change that nearly eliminates that failure in the same test.
Note: options differ in kind, not coverage — no completeness score.
Net: a better-tested rule versus the wording you approved earlier today.
Header: Conflicts
Options:
A) Apply this change (recommended)
✅ All-3-held rises from 70-75% to 95-96% across 400 simultaneous-decision runs. ✅ Every drone can compute it alone, so there's still no coordinator. ❌ Higher-numbered drones always give way, so they tend to fly further, and it has not been flown. Effort: human ~1h / CC ~5 min.
B) Keep this row's current value
✅ Stays exactly as approved in D11, with no rework of the decision record. ✅ No drone is favoured over another. ❌ 25-30% of probe runs never settled, which could show up in the Nov 15 flight. Effort: none.
C) Investigate before choosing
✅ Tests the rule in real SITL first, since the Python sim has no flight dynamics. ✅ Bounded to one 3-drone run once the node exists. ❌ The node gets built with a rule already shown to fail, and may need rewriting. Effort: one SITL run, no conditional fix approved.
D) Defer this proposed change only
✅ Saves about an hour now, leaving the rest of the plan intact. ✅ Can be reopened if the Nov 15 run shows swapping. ❌ This row stays unresolved at implementation, and the planner starts with the known failure. Effort: none.

State: approved
Actual answer: A) Apply this change (D24, 2026-11-04)
Accepted scope: if a drone's goal falls inside the zone of a lower sysid's goal, the higher sysid yields and re-plans; the lower sysid ignores higher sysids' goals. Replaces the identical-goals tie-break in R5. Includes a library test with simultaneous decisions and one-tick-stale peer state.
History: R5 accepted scope (D11) tie-broke identical goals only. Reopened on the outside-voice finding and the timed-flight probe: 25-30% of runs never settled.

### R20: Whether a drone flying past blocks a sector (changes R11)
Finding: outside voice 2 follow-on, P1, confidence 8/10, R11 accepted scope ("a peer's current or goal sector or one of their 8 neighbours"), native probe.
Plan baseline: approved via D5 and D11: a peer's current position blocks its zone at all times, including in transit.
Runtime evidence: with the D24 rule, 95% (96%) all 3 held, 13.3 goal changes, 5% (4%) still changing. With in-transit positions not blocking: 100% (100%), 3.9 goal changes, 0%. Same probe as R19. Physical safety in transit then rests on altitude layers (R21, pending). Not flown.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R20 what a peer's position blocks | always its zone | its zone only once arrived; in flight only its goal zone blocks | always its zone | always, until a SITL test | always, row unresolved |
| Work | none | ~3 lines + test row (human ~20 min / CC ~2 min) | none | one 3-drone SITL run | none |
Question D25:
D25 — Should a drone just flying past a sector stop others from choosing it?
Project/branch/task: follower planner coordination on main; second half of the conflict fix, changes the penalty rule from D5 and D11.
ELI10: Right now, if a drone flies past your target on the way somewhere else, your target briefly counts as blocked, and your rule says re-plan straight away. So a passing drone can knock another off course for no lasting reason. If only a drone's destination and its parked position block sectors, that stops. In the probe this took all-3-held from 95-96% to 100% and goal changes from 13 to 4.
Stakes if we pick wrong: keeping it leaves some needless re-planning and about 1 run in 20 unsettled; applying it means two drones can pass close horizontally, so altitude layers become the only separation in transit. That's the next question.
Recommendation: A because it removes the remaining failures in the probe, provided altitude layers are enforced.
Note: options differ in kind, not coverage — no completeness score.
Net: fewer pointless course changes versus keeping a sector-level buffer during transit.
Header: Passing
Options:
A) Apply this change (recommended)
✅ All-3-held reaches 100% in the probe, with goal changes down from about 13 to about 4. ✅ Three lines in one pure function, with a test row. ❌ Drones can pass close horizontally, so altitude layers must be enforced, and it has not been flown. Effort: human ~20 min / CC ~2 min.
B) Keep this row's current value
✅ Keeps a sector-sized buffer around every drone at all times. ✅ Stays closest to the thesis's penalty. ❌ About 1 run in 20 stays unsettled, with about three times the goal changes. Effort: none.
C) Investigate before choosing
✅ Checks real flight first, since the probe has no dynamics. ✅ Bounded to one 3-drone run once the node exists. ❌ Delays the choice, and the 95-96% rule gets built first. Effort: one SITL run, no conditional fix approved.
D) Defer this proposed change only
✅ No work now, with the rest of the plan unchanged. ✅ Easy to add later, since it's one condition. ❌ This row stays unresolved at implementation. Effort: none.

State: approved
Actual answer: A) Apply this change (D25, 2026-11-04)
Accepted scope: a peer's position blocks its zone only once it has arrived; in flight, only its goal zone blocks. Includes a test row. Changes the R11 blocked set.
History: R11 (D5, D11) had the position block at all times; changed on the timed-flight probe.

### R21: Altitude must be reached before horizontal flight
Finding: outside voice 4, P1, confidence 9/10, R9 accepted scope ("exits when airborne"), R6 ("3 m horizontal") and D5 altitude layers, Codex; confirmed by reading the approved scopes.
Plan baseline: D5 approved per-drone altitude layers, and D25 now relies on them in transit. Nothing requires a drone to reach its layer before horizontal flight, and arrival checks horizontal distance only. Leader altitude is unassigned.
Runtime evidence: in today's probe both drones climbed together, reaching +10 m after about 25-30 s wall time, with `GlobalPosition` altitude honoured in the relative frame. Not tested: flight with staggered layers. Two drones spawned 5 m apart climb in the same column only if commanded horizontally early.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R21 before the first horizontal goal | no check | planner reports HOLDING until own altitude is within 0.5 m of its layer | no check | no check until a SITL test | no check, row unresolved |
| R21 what arrival requires | horizontal only | horizontal and altitude | horizontal only | horizontal only | horizontal only |
| R21 layer assignment | unspecified | leader 8 m, followers 10 + 2 x (index) m, launch params | unspecified | unspecified | unspecified |
| Work | none | ~10 lines + test (human ~30 min / CC ~3 min) | none | one SITL run | none |
Question D26:
D26 — Should a follower wait until it reaches its own altitude before flying sideways?
Project/branch/task: follower planner safety on main; altitude layers are now the only separation in transit.
ELI10: Giving each drone its own height only protects them if they're actually at that height. Nothing in the plan says so: a drone could start flying sideways during takeoff, at the same height as another. The fix is a simple gate: the planner stays in HOLDING until the drone is within half a metre of its assigned height, and "arrived" checks height too, so a climb can't count as arrival. This also fixes everyone's layer, including the leader's, in the launch file.
Stakes if we pick wrong: without the gate, two drones crossing during climb can collide in the sim, which would look like a planner bug and cost debugging time.
Recommendation: A because separation by altitude means nothing unless altitude is checked, and this is about 10 lines.
Note: options differ in kind, not coverage — no completeness score.
Net: a small check versus trusting timing for the only in-flight separation.
Header: Altitude gate
Options:
A) Apply this change (recommended)
✅ No horizontal flight until the drone is at its layer, so separation holds from the first move. ✅ Arrival checks height too, so a climb can't report ARRIVED. ✅ Assigns all layers, including the leader's, in one launch file. ❌ Each follower waits for its climb before moving, a few more seconds per launch. Effort: human ~30 min / CC ~3 min.
B) Keep this row's current value
✅ No extra state or check in the node. ✅ Drones start moving as soon as the helper exits. ❌ A drone can fly sideways through another's column during climb, and layers aren't assigned. Effort: none.
C) Investigate before choosing
✅ Shows whether this happens in practice before adding code. ✅ Bounded to one 3-drone run once the node exists. ❌ The layers would be unenforced until then, and D25 relies on them. Effort: one SITL run, no conditional fix approved.
D) Defer this proposed change only
✅ Saves a little time this week, with the rest of the plan unchanged. ✅ Can be added once a run shows the need. ❌ This row stays unresolved, and D25's safety assumption stays unenforced. Effort: none.

State: approved
Actual answer: A) Apply this change (D26, 2026-11-04)
Accepted scope: the planner reports HOLDING and sends no horizontal goal until its altitude is within 0.5 m of its layer. Arrival requires horizontal and altitude criteria. Layers are launch params: leader 8 m, followers 10 + 2 x index m. Includes a test.
History: none

### R22: Arrival radius (reopens R6)
Finding: outside voice 8, P2, confidence 10/10, R6 accepted scope ("`arrival_radius` param, default 3.0 m horizontal"), Codex; native reviewer error confirmed.
Plan baseline: approved by D12 at 3.0 m. The D12 brief said 3 m "keeps the drone inside its 5 m sector". That statement was wrong: a 5 m sector's half-width is 2.5 m, so a drone 2.9 m along an axis from the centre is in the neighbouring sector.
Runtime evidence: `cmd_gps_pose` reached within 0.4 m of target in about 10 s wall time, so a tighter radius is reachable. Settling time at smaller radii not measured.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R22 `arrival_radius` default | 3.0 m | 2.0 m | 3.0 m | 3.0 m until a SITL test | 3.0 m, row unresolved |
| Can be arrived while over the next sector | yes | no, for 5 m sectors | yes | yes | yes |
| Relation to thesis 3-5 m | in range | tighter; thesis figure was proximity to a marker | in range | in range | in range |
| Work | none | one default value + test row (human ~10 min / CC ~1 min) | none | one SITL run | none |
Question D27:
D27 — My 3 m arrival radius was wrong for 5 m sectors. Change it to 2 m?
Project/branch/task: follower planner node on main; reopens D12 because I gave you an incorrect reason.
ELI10: In D12 I told you 3 m keeps a drone inside its sector. That was wrong. A 5 m sector extends only 2.5 m from its centre, so at 2.9 m a drone can count as "arrived" while actually over the neighbouring sector. Codex caught it. A 2 m radius fixes this, and today's flight got within 0.4 m, so it's easily reachable. It's tighter than the thesis's 3-5 m, but that figure was about closeness to a marker, not sector logic.
Stakes if we pick wrong: at 3 m, a drone can report ARRIVED for one sector while over another, and any later camera or landing step starts from the wrong place.
Recommendation: A because it corrects my error with one default value.
Note: options differ in kind, not coverage — no completeness score.
Net: correct sector logic versus staying inside the thesis's stated range.
Header: Arrival fix
Options:
A) Apply this change (recommended)
✅ An arrived drone is always over its target sector at the 5 m size. ✅ One default value and one test row, and it remains a param for other grids. ❌ Below the thesis's 3-5 m figure, and settling time at 2 m hasn't been measured, though today's flight reached 0.4 m. Effort: human ~10 min / CC ~1 min.
B) Keep this row's current value
✅ Stays in the thesis's stated range, as approved in D12. ✅ Arrival triggers slightly sooner. ❌ A drone can report arriving while over the neighbouring sector. Effort: none.
C) Investigate before choosing
✅ Measures settling behaviour in SITL before fixing a number. ✅ Bounded to one run once the node exists. ❌ The geometry error doesn't need a flight to confirm it. Effort: one SITL run, no conditional fix approved.
D) Defer this proposed change only
✅ No work now, with the rest of the plan unchanged. ✅ It's a param, so it's a quick change later. ❌ This row stays unresolved, and the known error ships in the default. Effort: none.

State: approved
Actual answer: A) Apply this change (D27, 2026-11-04)
Accepted scope: `arrival_radius` default is 2.0 m horizontal, with altitude also required per R21; remains a param. Includes a test row. Replaces the 3.0 m default in R6.
History: R6 (D12) set 3.0 m on an incorrect statement that it stays inside a 5 m sector. Reopened on outside-voice finding 8.

### R23: Goal reservations must renew and expire
Finding: outside voice 3a, P1, confidence 9/10, R5 accepted scope (`/<robot>/goal_sector`, RELIABLE + TRANSIENT_LOCAL) and R16 ("a peer silent for `peer_timeout`"), Codex; confirmed by reading the approved scopes.
Plan baseline: D11 latches a goal with TRANSIENT_LOCAL; D21 releases it when a peer goes silent. A disabled planner keeps publishing pose through `localization_node`, so silence never triggers and its latched goal reserves a sector indefinitely.
Runtime evidence: none; no planner exists. Follows from the approved QoS and from `localization_node` being a separate process.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R23 goal message | latched once, RELIABLE + TRANSIENT_LOCAL | republished at 1 Hz with a fresh stamp, RELIABLE + VOLATILE | latched | latched until a SITL test | latched, row unresolved |
| R23 how a goal ends | peer silence only | stops being valid `peer_timeout` after its last stamp, independent of pose | peer silence only | peer silence only | peer silence only |
| Long flight | n/a | stays valid, since it is renewed | stays valid | stays valid | stays valid |
| Late-joining peer | gets latched goal | gets the goal within 1 s | gets latched goal | gets latched goal | gets latched goal |
| Work | none | ~10 lines + test (human ~30 min / CC ~3 min) | none | one SITL run | none |
Question D28:
D28 — How should a follower's reservation on a sector end?
Project/branch/task: follower planner coordination on main; a gap between D11 and D21 found by the outside review.
ELI10: A drone announces its goal so others keep away. Under D11 that announcement sticks around after being sent. D21's timeout only fires when a drone stops reporting its position, but a disabled planner's drone keeps reporting position, since that's a different node. So a switched-off drone can reserve a sector forever. The simple fix: repeat the goal once a second while it's real, and treat it as expired 5 seconds after the last repeat. Stopping the repeats is how a goal ends, so no "cancel" message is needed.
Stakes if we pick wrong: after the Nov 24 handoff, a drone that's landing or finished could block a sector for the rest of the mission, with no error shown.
Recommendation: A because it closes the gap with less machinery: there's no latching and no cancel message.
Note: options differ in kind, not coverage — no completeness score.
Net: a repeating announcement versus a stuck reservation.
Header: Goal expiry
Options:
A) Apply this change (recommended)
✅ A goal ends by itself when repeats stop, so there's no cancel message to lose. ✅ Long flights stay reserved, because the goal is renewed every second. ✅ A late-starting peer learns the goal within 1 s. ❌ Slightly more traffic, one small message per drone per second, and a change to D11's QoS. Effort: human ~30 min / CC ~3 min.
B) Keep this row's current value
✅ One message per goal, as approved in D11. ✅ Late joiners get the goal immediately. ❌ A disabled drone can reserve a sector forever while its position keeps publishing. Effort: none.
C) Investigate before choosing
✅ Would show the stuck reservation in a real run first. ✅ Bounded to one SITL run after the node exists. ❌ The gap follows directly from the design, so a run adds little. Effort: one SITL run, no conditional fix approved.
D) Defer this proposed change only
✅ Nothing uses the enable switch until Nov 24, so nothing breaks this week. ✅ No change to the record now. ❌ This row stays unresolved, and QoS changes later touch every node's goal handling. Effort: none.

State: approved
Actual answer: A) Apply this change (D28, 2026-11-04)
Accepted scope: `/<robot>/goal_sector` is republished at 1 Hz with a fresh stamp, RELIABLE + VOLATILE, only while the goal is real. A goal stops being valid `peer_timeout` after its last stamp, independent of pose; stopping the repeats ends it. Replaces the TRANSIENT_LOCAL in R5. Includes a library test.
History: R5 (D11) latched goals with TRANSIENT_LOCAL. Reopened on outside-voice finding 3.

### R24: What a drone does when its planner is disabled
Finding: outside voice 3b, P2, confidence 9/10, R6 accepted scope ("sending no goals while false"), Codex; supported by today's probe.
Plan baseline: D12 approved that a disabled planner sends no goals.
Runtime evidence: one single `cmd_gps_pose` message flew iris_2 the whole 8 m, so stopping publication does not stop a flight already accepted. Hold at current position on disable not tested.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R24 on enable going false mid-flight | continues to the last goal | one goal at current position and layer, then silence | continues to the last goal | continues until a SITL test | continues, row unresolved |
| Next controller's starting state | drone may still be moving | drone holding | drone may still be moving | may still be moving | may still be moving |
| Work | none | ~5 lines + test (human ~20 min / CC ~2 min) | none | one SITL run | none |
Question D29:
D29 — When the planner is switched off mid-flight, should the drone stop where it is?
Project/branch/task: follower planner node on main; refines the enable switch from D12.
ELI10: Today's test showed one message is enough to fly a drone all the way to its target. So "the planner stops sending goals" doesn't stop the drone: it keeps flying to the last one. If the mission switches a planner off, the drone would carry on while whatever takes over assumes it's in control. Sending one final "stay here" position when switched off means the next controller starts from a drone that's holding still.
Stakes if we pick wrong: without it, a mission handoff can have two things steering in sequence, which is confusing to debug on Nov 24; with it, a brief mid-air stop before the next command.
Recommendation: A because a switch named "disable" should actually stop the drone, and it's about 5 lines.
Note: options differ in kind, not coverage — no completeness score.
Net: a clean handoff versus a small extra stop.
Header: Disable
Options:
A) Apply this change (recommended)
✅ Disable really stops the drone, so the next controller starts from a known state. ✅ About 5 lines, testable from the command line with the enable topic. ❌ Adds a brief hover at handoff, and holding on disable hasn't been flown yet. Effort: human ~20 min / CC ~2 min.
B) Keep this row's current value
✅ As approved in D12, with no extra command. ✅ A follower nearly at its target still completes the flight. ❌ The drone keeps flying after disable, so the next controller may not know it's moving. Effort: none.
C) Investigate before choosing
✅ Would test both behaviours in SITL with the real handoff. ✅ Bounded to one run once the node exists. ❌ The probe already shows a flight continues after commands stop. Effort: one SITL run, no conditional fix approved.
D) Defer this proposed change only
✅ Nothing uses disable until Nov 24, so this week is unaffected. ✅ Could be designed alongside the state machine. ❌ This row stays unresolved at implementation. Effort: none.

State: approved
Actual answer: A) Apply this change (D29, 2026-11-04)
Accepted scope: when `planner_enable` goes false, the node sends one goal at the drone's current position and layer, then sends nothing further. Includes a test. Extends R6.
History: none

### R25: The camera must look where the drone hovers
Finding: outside voice 5, P1, confidence 9/10, `src/uav_mapping/uav_mapping/leader_node.py:354` (`if len(ground) == 4:`) and warm-up measurement, Codex; verified in source and by measurement today.
Plan baseline: `leader_node` assumes a nadir camera (`_NADIR_QUAT`). `docs/next-session-prompt.md` treats the mount as "a launch-arg change, not code". R13 and R11 rely on follower cameras observing below and around them.
Runtime evidence: at 10 m the real view axis is 52.2 deg off nadir, hitting ground about 13 m from the drone, confirmed by markers to 0-4 px. The frame contains the horizon, so at least one image corner misses the ground, and `leader_node.py:354` then discards all negative evidence for that frame. A correct `mount_quat` fixes projection only. `heatmap.launch.py` does not expose `mount_quat`. A raw `/gimbal/cmd_*` publish left the joints unchanged. Setting the gimbal through ArduPilot mount params (`gazebo-iris-gimbal.parm` sets `MNT1_TYPE 1`) has not been tried.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R25 camera pointing | 52 deg off, leader assumes nadir | required nadir before footprints: ArduPilot mount config, proven with the marker test, in a 2 h timebox | unchanged | 1 h look at mount params now, no fix approved | unchanged, row unresolved |
| R25 launch arg | `mount_quat` not exposed | exposed in `heatmap.launch.py` | not exposed | not exposed | not exposed |
| Partial footprint rule at `leader_node.py:354` | drops the frame | unchanged, moot at nadir | unchanged | unchanged | unchanged |
| If nadir fails in the timebox | n/a | back to you, with the options | n/a | n/a | n/a |
| Nov 15 milestone | unaffected | unaffected, done before Nov 24 | unaffected | unaffected | unaffected |
| Work | none | up to 2 h (human ~2h / CC ~30 min) + 1 launch line | none | up to 1 h now | none |
Question D30:
D30 — The camera points 52° off vertical. Should getting it straight down become a required task before Nov 24?
Project/branch/task: observation model on main; doesn't affect planner code or Nov 15, but the search doesn't work without it.
ELI10: The next-session prompt called this a simple launch setting. It's not, and I measured why. The camera looks about 13 m away from the drone, so hovering over a sector doesn't mean seeing it. Correcting the setting only fixes the maths, not where the camera points. Also, because the horizon is in the frame, `leader_node` throws away that frame's "saw nothing here" evidence, and your "followers explore" answer in D18 depends on that evidence. The clean fix is pointing the gimbal straight down through ArduPilot's settings, then proving it with the same marker test I used today. I haven't tried this yet, and it may not be simple.
Stakes if we pick wrong: leaving it means the exploring behaviour you approved can't work, and we'd discover that near the Nov 30 go/no-go.
Recommendation: A because exploration depends on it, and a 2 h limit with the marker test bounds the risk.
Note: options differ in kind, not coverage — no completeness score.
Net: a required, bounded camera task versus risking the search loop later.
Header: Camera
Options:
A) Apply this change (recommended)
✅ Hover point and view match, and every image corner hits the ground, so negative evidence works with no leader change. ✅ The marker test gives a clear pass or fail. ✅ Also exposes `mount_quat` in the launch file. ❌ The method is untried, so if 2 h isn't enough it comes back to you. Effort: human ~2h / CC ~30 min, before Nov 24.
B) Keep this row's current value
✅ No time spent before Nov 24, and the planner is unaffected. ✅ Leaves verified code and config alone. ❌ Followers don't observe what they hover over, and negative evidence is dropped, so D18's exploration can't work. Effort: none.
C) Investigate before choosing
✅ Finds out now whether a settings change is enough, up to 1 h. ✅ Earlier warning before the go/no-go. ❌ Takes this week's limited hours before the Nov 10 target. Effort: up to 1 h, no conditional fix approved.
D) Defer this proposed change only
✅ Nothing this week depends on it, so the plan is unchanged. ✅ Could be looked at with the belief map work. ❌ This row stays unresolved, with nothing on the plan making sure it happens. Effort: none.

State: approved
Actual answer: A) Apply this change (D30, 2026-11-04)
Accepted scope: a required task before footprint work, after Nov 15 and before Nov 24: point the gimbal at nadir through ArduPilot mount configuration and prove it with the marker test. Timebox 2 h; if it fails, return to the user with options. Expose `mount_quat` in `heatmap.launch.py`. `leader_node.py:354` stays as it is.
History: none

### R26: What the numbers in the map mean
Finding: outside voice 6, P1, confidence 9/10, `docs/roadmap.md:45` ("posterior is proportional to ...") and `src/uav_mapping/uav_mapping/leader_node.py:50` (`p = 1 / (1 + exp(-l))`, per cell), Codex; verified in both files.
Plan baseline: the roadmap's fusion line is a normalized-looking formula; `leader_node` uses independent per-cell occupancy. D16's 0.02 margin, D14's 0.5 threshold and D18's unknown = 0.5 all assume per-cell probabilities.
Runtime evidence: under a normalized distribution a uniform 8x8 map is about 0.016 per cell, so a 0.02 margin would exceed the whole value and a 0.5 threshold would almost never apply. Under per-cell occupancy all three values work as written. `leader_node` publishes per-cell values today.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R26 map contract | unstated, two readings | each cell = probability it contains at least one target, independent; values need not sum to 1 | unstated | unstated until belief map design | unstated, row unresolved |
| Roadmap fusion line | "proportional to" | reworded: prior sets each cell's starting log-odds, blended with the 0.5 baseline by alpha | unchanged | unchanged | unchanged |
| Multiple targets | ambiguous | natural, each cell independent | ambiguous | ambiguous | ambiguous |
| Work | none | doc change only now (human ~20 min / CC ~2 min) | none | ~1 h at belief map design | none |
Question D31:
D31 — What does a number in the map mean, exactly?
Project/branch/task: fusion contract on main; the planner's thresholds depend on this.
ELI10: There are two ways to read a heatmap. One: all cells together sum to 1, like "the single target is somewhere, and this is how likely each spot is". Two: each cell separately answers "is there a target here?", so many cells can be high. Your `leader_node` already uses the second, but the roadmap's formula reads like the first. The difference is big: on an 8x8 flat map the first gives about 1.6% per cell and the second gives 50%. Your 2% margin and 50% stop rule only make sense with the second, and with an unknown number of targets the second is the natural fit.
Stakes if we pick wrong: leaving it unstated means the belief map might get built to the other meaning on Nov 24, silently breaking the margin and stop rule.
Recommendation: A because it matches the code you have and every threshold you approved, and costs one doc edit now.
Note: options differ in kind, not coverage — no completeness score.
Net: one clear definition now versus discovering a mismatch during fusion.
Header: Map meaning
Options:
A) Apply this change (recommended)
✅ Matches `leader_node` as written and every threshold you've approved. ✅ Handles an unknown number of targets naturally. ✅ A doc edit today prevents a fusion mismatch on Nov 24. ❌ The roadmap's fusion formula gets reworded, and the alpha blending has only been written, not tested. Effort: human ~20 min / CC ~2 min.
B) Keep this row's current value
✅ No edits, and the roadmap stays as agreed earlier. ✅ Leaves room to choose during belief map design. ❌ Two readings stay open, and the wrong one would silently break the margin and stop rule. Effort: none.
C) Investigate before choosing
✅ Could weigh both options against the LLM prior's real output. ✅ Bounded to about 1 h when the belief map is designed. ❌ The planner is built this week on an assumption not written down. Effort: ~1 h later, no conditional fix approved.
D) Defer this proposed change only
✅ Nothing flying this week reads a fused map. ✅ No change to the record now. ❌ This row stays unresolved, and nothing ensures the question comes back. Effort: none.

State: approved
Actual answer: A) Apply this change (D31, 2026-11-04)
Accepted scope: each map cell is the independent probability that it contains at least one target; values need not sum to 1. Stated in `docs/roadmap.md`, whose fusion line is reworded: the prior sets each cell's starting log-odds, blended with the 0.5 baseline by alpha, then updated per footprint. The blend is untested.
History: none

### R27: What counts as a confirmed find
Finding: outside voice 7, P1, confidence 8/10, `docs/roadmap.md:62` ("Found = a detection within 2 m of the true position") and R7 accepted scope ("lowers a confirmed sector"), Codex; verified in both.
Plan baseline: D13 approved that `belief_map_node` lowers a "confirmed" sector. "Confirmed" has no definition. The roadmap's 2 m rule uses true positions, which a running system does not have. D14's stop rule counts only unsuppressed sectors, so a wrong confirmation can end a search early.
Runtime evidence: none; no detector or belief map exists. `leader_node` reaches 97 after about 5 consistent frames, so repeated frames of one false positive would look like strong evidence.
Comparison grid:
| Choice | Current | A: apply | B: keep | C: investigate | D: defer |
|---|---|---|---|---|---|
| R27 definition of confirmed | none | follower hovering within arrival radius sees a detection in its own sector, N frames | none | written decision before belief map code, about 1 h, no rule chosen now | none, row unresolved |
| Suppression scope | sector | sector | sector | decided with it: sector or per target | sector |
| Second target in a suppressed sector | hidden | hidden, stated limit | hidden | decided with it | hidden |
| Planner code | none | none | none | none | none |
| Work | none | rule in docs (human ~20 min / CC ~2 min), untested | none | ~1 h at start of belief map work | none |
Question D32:
D32 — How should the system decide a target is really found?
Project/branch/task: belief map design on main; D13 and D14 rely on "confirmed", which nobody has defined.
ELI10: You approved "the map lowers a sector once a target is confirmed", but we never said what confirmed means. The eval's rule, within 2 m of the true position, only works for scoring, since a live system doesn't know the true position. It matters because lowering a sector can hide a second target there, and your stop rule ignores lowered sectors, so a false alarm could end a search early. A sound definition depends on how the detector behaves, including its noise and false positives, and that doesn't exist yet. So I'd write this decision at the start of belief map work, instead of guessing now.
Stakes if we pick wrong: guessing now may fit a detector we haven't built; leaving it open with no marker risks a search that ends on a false alarm. Planner code is the same either way.
Recommendation: C because the right rule depends on detector noise settings that are still undecided, and this makes it a scheduled decision before any belief map code.
Note: options differ in kind, not coverage — no completeness score.
Net: decide with real detector numbers versus settling a rule on paper today.
Header: Confirmation
Options:
A) Apply this change
✅ Gives the Nov 24 work a concrete rule from day one: a follower overhead sees it for N frames. ✅ Uses the close-range check the thesis's follower phase already implies. ❌ N is a guess without detector noise numbers, and the rule is untested. Effort: human ~20 min / CC ~2 min.
B) Keep this row's current value
✅ No time spent now, and D13's wording stays as approved. ✅ Leaves the belief map author free to choose. ❌ Nothing defines it, and false alarms can suppress sectors and end searches early. Effort: none.
C) Investigate before choosing (recommended)
✅ Decided with detector noise settings in hand, in one written decision of about 1 h. ✅ Placed before any belief map code, so it can't be skipped by accident. ❌ The approved suppression and stop rules stay incomplete until then, and no rule is chosen now. Effort: ~1 h at the start of belief map work.
D) Defer this proposed change only
✅ Nothing this week depends on it. ✅ No extra task on the plan. ❌ This row stays unresolved with no scheduled point to settle it. Effort: none.

State: approved
Actual answer: A) Apply this change (D32, 2026-11-04; the user chose this over the recommended C)
Accepted scope: a target is confirmed when a follower hovering within `arrival_radius` of a sector's centre sees a detection in that sector for N consecutive frames. N is a param; its value is set when detector noise is known, and the rule is untested. Suppression applies to the whole sector. Stated limit: a second target in a suppressed sector stays hidden. The eval's 2 m rule remains for scoring only. Applies to `belief_map_node`; no planner code.
History: none

### R28: A search-loop checkpoint and an explicit build order
Finding: outside voice 1, P1, confidence 8/10, `docs/roadmap.md:25` ("heatmap done; footprint publishing and prior-biased lane order still to do") and `:75` Nov 15 milestone, Codex; verified: `leader_node.py` has no flight commands.
Plan baseline: candidate not on the plan; disposition today: absent. The Nov 15 trigger is 3 followers on a static map. Sweep controller, ground-truth detector and footprints are not sequenced, and the sweep controller is not listed at all.
Runtime evidence: on a static stub nothing can be found, so Nov 15 cannot exercise observe, update, depart or find. Weekly budget is 12-15 h. Already approved for Nov 16-24: 2 h sim speed timebox (D23), 2 h camera timebox (D30).
Comparison grid:
| Choice | Current | A: include | B: defer | C: cut | D: hold |
|---|---|---|---|---|---|
| Candidate: one-follower search loop checkpoint | absent | added after Nov 15, target Nov 20: one follower observes, map updates, it departs, finds a target | moved past Nov 24 | not on the plan | stop and discuss |
| Nov 15 trigger | 3 followers on stub | unchanged | unchanged | unchanged | unchanged |
| Nov 16-24 order | unspecified | camera + speed timeboxes, detector, sweep controller, belief map + footprints, checkpoint, state machine | unspecified | unspecified | undecided |
| Sweep controller on the plan | no | yes | factual roadmap correction only | factual roadmap correction only | factual roadmap correction only |
| Extra build work | none | little: a test of parts already planned (human ~half day / CC ~30 min) | none now | none | none |
Question D33:
D33 — Should the plan add a checkpoint where one follower actually completes a search?
Project/branch/task: milestones on main; the outside review's main strategic point.
ELI10: Nov 15 proves three drones can spread out over a fixed map. Nothing can be found on a fixed map, so the real loop, look, update the map, move on, find something, first runs at the very end. Also, the leader has no flight code yet, though the roadmap reads as if the sweep exists. I'd keep Nov 15 as your trigger, then add a checkpoint around Nov 20: one follower runs the whole loop. It's mostly a test of parts already planned, and it surfaces problems ten days before the go/no-go. Codex wanted this before the three-drone milestone; I'd keep your trigger where it is.
Stakes if we pick wrong: without the checkpoint, the first full search may happen days before Nov 30. Nov 16-24 is also tight: about 15-20 hours, with two timeboxes, a detector, a sweep controller, a belief map and a state machine.
Recommendation: Include because it gives early warning at little extra build cost, and leaves the Nov 15 trigger alone.
Note: options differ in kind, not coverage — no completeness score.
Net: an early end-to-end proof versus a little less slack in a tight week.
Header: Search loop
Options:
A) Include (recommended)
✅ The real loop runs about ten days before the go/no-go, while there's still time to react. ✅ Gives Nov 16-24 a clear build order, and puts the missing sweep controller on the plan. ✅ The Nov 15 trigger stays as you set it. ❌ About half a day of test work in an already tight stretch, which could force the state machine past Nov 24. Effort: human ~half day / CC ~30 min.
B) Defer
✅ Keeps Nov 16-24 focused on building, with no extra test target. ✅ The full mission on Nov 30 still proves the loop. ❌ First end-to-end run comes at the go/no-go, with little time to fix what it finds. Effort: none now.
C) Cut
✅ Simplest plan, with the fewest milestones to track. ✅ The Nov 30 mission remains the only integration target. ❌ Same late-discovery risk as deferring, with no plan to come back to it. Effort: none.
D) Hold
✅ Stops here to discuss sequencing before the record changes. ✅ Nothing else changes while you think about it. ❌ This row and the review's close stay open until you decide. Effort: none.

State: approved
Actual answer: A) Include (D33, 2026-11-04)
Accepted scope: a checkpoint targeting Nov 20: one follower observes, the map updates, it departs and finds a target. Nov 16-24 order: camera and sim-speed timeboxes, ground-truth detector, sweep controller, belief map with footprints, checkpoint, state machine. The Nov 15 trigger is unchanged. The sweep controller goes on the roadmap. Capacity risk stated: about 15-20 h for this list may push the state machine past Nov 24.
History: none

### R29: TODO: repeated frames over-count evidence
Finding: code quality, P2, confidence 8/10, `src/uav_mapping/uav_mapping/leader_node.py:374` (`self._bump(row, col, score * self.l_occ)`), native review.
Plan baseline: not on the plan. `docs/roadmap.md` says fusion extends this update.
Runtime evidence: live run today: hit cell reached 97 and swept cells 3 within seconds. At score 0.9 and 1 Hz the clamp is reached in about 5 frames, because each frame counts as independent evidence. One false positive held in view for 5 frames looks certain. With D32, N consecutive frames has the same exposure.
Comparison grid:
| Choice | Current | A: add to TODOS | B: skip | C: build now |
|---|---|---|---|---|
| R29 evidence per frame | every frame counts fully | unchanged now; `belief_map_node` design decides | unchanged, no record | rate limit added now in `leader_node` |
| Work | none | one TODO entry (human ~5 min / CC ~1 min) | none | ~10 lines + re-verify leader (human ~1h / CC ~10 min) |
Question D34:
D34 — Track the problem of repeated camera frames counting as new evidence?
Project/branch/task: TODOS.md on main; affects belief_map_node, not this week's planner.
ELI10: What: the map treats each camera frame as fresh evidence, but frames a moment apart see the same thing. Why: one false detection held in view for about 5 frames reaches 97%, and it would pull followers. Pros of fixing: a false alarm stays weak, and your confirmation rule means more. Cons: tuning a rate limit against a detector that doesn't exist yet. Context: start at `leader_node.py:374` and the belief map's update step. Depends on: the ground-truth detector's noise settings.
Stakes if we pick wrong: forgetting it means false positives look certain in the Nov 30 mission; fixing it now spends planner hours on a node being replaced.
Recommendation: A because the fix belongs in the belief map, which is coming, and needs detector numbers first.
Note: options differ in kind, not coverage — no completeness score.
Net: record it for the right moment versus fixing code that's about to be superseded.
Header: Frame evidence
Options:
A) Add to TODOS.md (recommended)
✅ It's picked up exactly when the belief map's update is written, so it isn't lost. ✅ No change to verified code during the planner push. ❌ False positives stay over-weighted in `leader_node` until then. Effort: human ~5 min / CC ~1 min.
B) Skip
✅ Fewer items to track in a tight month. ✅ Ground-truth detector noise can be set low enough to hide it. ❌ False positives can look certain in the eval and video, with no record of why. Effort: none.
C) Build it now
✅ Fixed before anything depends on the current behaviour. ✅ `leader_node` gets more honest evidence immediately. ❌ Touches a verified node that the belief map soon replaces, with no real detector to tune against. Effort: human ~1h / CC ~10 min.

State: approved
Actual answer: A) Add to TODOS.md (D34, 2026-11-04)
Accepted scope: one TODOS.md entry; `belief_map_node` design decides how repeated frames are handled. `leader_node` unchanged.
History: none

### R30: TODO: empty frames must reach the map
Finding: code quality, P2, confidence 9/10, `src/uav_mapping/uav_mapping/leader_node.py:332` (`def _on_detections(self, msg: Detection2DArray):`, miss updates at `:378`), native review.
Plan baseline: not on the plan. Roadmap: "a footprint with no detection lowers covered cells".
Runtime evidence: miss updates run only inside the detections callback, so a detector that publishes nothing on an empty frame produces no negative evidence. `fake_detection_publisher` publishes every period, so today's run cannot show this. Upstream wrappers' behaviour was not checked. The ground-truth detector is ours to write.
Comparison grid:
| Choice | Current | A: add to TODOS | B: skip | C: build now |
|---|---|---|---|---|
| R30 empty frames | depends on the detector | detector contract: always publish, including an empty array; footprints published separately on Nov 24 | unchanged, no record | timer-driven miss updates added to `leader_node` now |
| Work | none | one TODO entry (human ~5 min / CC ~1 min) | none | ~20 lines + re-verify leader (human ~2h / CC ~15 min) |
Question D35:
D35 — Track the requirement that "saw nothing" frames must reach the map?
Project/branch/task: TODOS.md on main; affects the ground-truth detector and belief_map_node.
ELI10: What: `leader_node` only lowers a sector's value when a detection message arrives. If a detector stays quiet when it sees nothing, no "saw nothing" evidence ever arrives, and sectors never drop. Why: your exploration behaviour from D18 depends on those drops. Pros of recording: the detector you write will publish empty frames, and footprints come separately on Nov 24. Cons: one more item on a tight list. Context: `leader_node.py:332` and `:378`. Depends on: nothing; apply it when the detector is written.
Stakes if we pick wrong: forgetting it means followers park on stale values with no error, and that looks like a planner bug.
Recommendation: A because it's a one-line contract for a detector you haven't written yet.
Note: options differ in kind, not coverage — no completeness score.
Net: a recorded contract versus a silent failure later.
Header: Empty frames
Options:
A) Add to TODOS.md (recommended)
✅ The detector gets written correctly the first time, at no extra cost. ✅ No change to verified code during the planner push. ❌ `leader_node` remains fragile if someone wires in a quiet detector before footprints exist. Effort: human ~5 min / CC ~1 min.
B) Skip
✅ One less entry, and the detector author may do this anyway. ✅ Footprints on Nov 24 replace this path regardless. ❌ If forgotten, sectors never drop, and it fails with no error. Effort: none.
C) Build it now
✅ `leader_node` stops depending on detector behaviour straight away. ✅ Works with any detector, including a future YOLO wrapper. ❌ Changes a verified node just before footprints replace this path. Effort: human ~2h / CC ~15 min.

State: approved
Actual answer: A) Add to TODOS.md (D35, 2026-11-04)
Accepted scope: one TODOS.md entry: every detector publishes on every frame, including an empty array, until footprints replace this path. `leader_node` unchanged.
History: none

### R31: TODO: test the direct rule on realistic maps
Finding: test review, P2, confidence 7/10, R12 accepted scope ("picks the highest-value open sector on the whole grid"), native review.
Plan baseline: D17 approved the direct rule on evidence from spiky maps with equal peaks.
Runtime evidence: probes used three equal 0.97 peaks on 0.03 only. Untested: unequal peaks, where a distant 0.97 always beats a nearby 0.90 however far it is; and smooth region-shaped maps like a language prior produces. No flight dynamics.
Comparison grid:
| Choice | Current | A: add to TODOS | B: skip | C: build now |
|---|---|---|---|---|
| R31 maps tested | spiky, equal peaks | unequal and smooth maps in the 2D eval sim; try a distance discount only if results call for it | as is | a table-driven library test added this week |
| Work | none | one TODO entry (human ~5 min / CC ~1 min) | none | ~30 lines (human ~1h / CC ~10 min) |
Question D36:
D36 — Track testing the direct-flight rule on more realistic maps?
Project/branch/task: TODOS.md on main; follows from your D17 decision.
ELI10: What: the direct rule won on my test maps, but those all had equal, spiky peaks. Why: real maps won't. A drone will fly across the whole arena for a 97% sector while ignoring a 90% one right next to it, and a language prior produces smooth regions rather than spikes. Pros: you'd know whether the rule needs a distance discount before reporting results. Cons: it needs the 2D eval sim to be meaningful. Context: start from the rule in `core/planner.py`. Depends on: the 2D eval sim.
Stakes if we pick wrong: forgetting it could mean eval results that reflect a distance-blind rule without anyone noticing.
Recommendation: A because the eval sim is the right place, and the milestone map has equal peaks anyway.
Note: options differ in kind, not coverage — no completeness score.
Net: test at the right stage versus a quick check now on hand-made maps.
Header: Real maps
Options:
A) Add to TODOS.md (recommended)
✅ It gets checked in the eval sim, against the same map shapes the results will use. ✅ Nothing competes with the Nov 15 push. ❌ The rule's weakness on unequal peaks stays unknown until the eval sim exists in December. Effort: human ~5 min / CC ~1 min.
B) Skip
✅ One less item, and the eval will exercise realistic maps anyway. ✅ Keeps the rule simple by default. ❌ Nobody is prompted to look at distance-blind behaviour when results come in. Effort: none.
C) Build it now
✅ An early answer on whether a distance discount is needed. ✅ Cheap with the library in place. ❌ Hand-made maps may not resemble real priors, and it takes time before Nov 15. Effort: human ~1h / CC ~10 min.

State: approved
Actual answer: A) Add to TODOS.md (D36, 2026-11-04)
Accepted scope: one TODOS.md entry: test the direct rule on unequal peaks and smooth region maps in the 2D eval sim; try a distance discount only if results call for it.
History: none

Approval readiness: PASS. Checked R1-R31 against actual answers D4-D36. R11 carries D5 and D11, and is changed by D25. Reopened: R5 by D24 and D28, R6 by D27. D32 is the user's choice over the reviewer's recommendation.

## Review results

Scope Challenge: scope reduced per recommendation (D4, D5, D6).

### Section 1: Architecture, 9 findings, all resolved

R1 shared frame, R2 actuation, R3 peers, R4 stepping, R5 goal sharing (then R19, R23), R6 arrival and handoff (then R22, R24), R7 suppression, R8 termination, R9 arming. Distribution: no new artifact beyond the existing container image and colcon workspace; nothing to publish.

### Section 2: Code quality, 7 findings, all resolved

R10 stay rule, R11 penalty, R12 reaching peaks, R13 unknown and flat maps, R14 stub map, R15 indexing (shared-code rubric: net savings about 0, rejected for now), R16 silent peers. Added to TODOS: R29, R30.

### Section 3: Test review

Framework: pytest 7.4.4 in the container; `CLAUDE.md` has no Testing section; 0 test files in the repo. No LLM or prompt changes in scope, so no eval. All paths below are proposed code, so every one is a planned gap.

```
CODE PATHS (proposed)                                  USER FLOWS
[+] uav_planning/core/planner.py                       [+] Single follower (Nov 10)
  |-- world_to_sector / sector_centre                    |-- [GAP] launch, arm, climb, fly, ARRIVED [->E2E]
  |   |-- [GAP] verified (7.5, 7.5) -> (1,1)             |-- [GAP] goal never accepted: warning logged
  |   `-- [GAP] outside grid                             `-- [GAP] enable false mid-flight: drone holds
  |-- blocked_set(peers)                               [+] Three followers (Nov 15)
  |   |-- [GAP] arrived peer: position zone              |-- [GAP] each holds a different peak [->E2E]
  |   |-- [GAP] flying peer: goal zone only              `-- [GAP] peer node killed: peak released in 5 s
  |   `-- [GAP] expired goal or silent peer ignored    [+] Search checkpoint (Nov 20)
  |-- choose_goal                                        `-- [GAP] observe, update, depart, find [->E2E]
  |   |-- [GAP] -1 cells as 0.5
  |   |-- [GAP] margin holds / margin moves
  |   |-- [GAP] tie: nearest, then lowest index
  |   |-- [GAP] everything blocked: no goal
  |   |-- [GAP] value drops: holder leaves
  |   `-- [GAP] higher sysid yields, simultaneous, stale
  |-- arena_to_lla
  |   `-- [GAP] round trip with localization projection
  |-- thesis_dga_step, bll_step
  |   `-- [GAP] BLL formula, seeded repeat, stays on grid
[+] follower_planner_node
  |-- [GAP] no heatmap: HOLDING, no goals
  |-- [GAP] below layer: HOLDING, no sideways goal
  |-- [GAP] message has frame_id "map", REL_ALT
  `-- [GAP] arrival needs radius and altitude
[+] localization_node (changed)
  |-- [GAP] CRITICAL regression: datum unset keeps latch
  `-- [GAP] same datum, different fixes: one frame

COVERAGE: 0/22 planned paths have tests  |  QUALITY: none yet  |  GAPS: 22 (3 E2E, 0 eval)
Legend: [GAP] test required with the code  |  [->E2E] needs a SITL run
```

Plan issue resolved: R17 (BLL anchor). The test plan artifact was updated after the outside voice. Tests made obsolete: none.

### Section 4: Performance, 1 finding, resolved

R18 sim speed. Otherwise: no database. The direct rule is O(cells x peers) at 1 Hz on small grids, which is negligible. `core/planner.py` allocates nothing long-lived. No caching needed.

### Outside voice

Codex CLI completed: 8 findings, all resolved or recorded (R19-R28). Reviewers disagreed on two points: BLL and DGA baselines kept (Codex suggested deferring), and the search checkpoint placed after Nov 15 rather than before it. Reviewer error caught: the D12 arrival claim. Native probe limits it exposed: round-robin stepping could not show goal conflicts.

## NOT in scope

- Potential-field layer: altitude layers cover the sim (D5); revisit if a same-altitude flight is ever needed.
- BLL flying at runtime: library and eval sim only (D4).
- Gazebo-based statistics: the 2D sim supplies them, per the roadmap.
- `landing_node`: handoff interface only (D12).
- Mission state machine and `belief_map_node`: Nov 24, with decisions recorded here.
- Partial-footprint handling at `leader_node.py:354`: moot once the camera is at nadir (D30).
- Updating the `multi-agent-uavs` skill's stale topic examples and `docs/stack-overview.md`: separate doc pass, not decided here.

## What already exists

- `localization_node` equirectangular projection: reused, inverted for arena-to-lat/lon, with a round-trip test.
- `leader_node` indexing: duplicated about 7 lines deliberately (D20, rubric in R15).
- `robots/iris.launch.py` from `ardupilot_gz`: reused per drone by our swarm launch; `multiagent.launch.py` is a template to copy from.
- `/ap/vN/*` services and `cmd_gps_pose`: used as they are.
- `fake_detection_publisher`: kept for `leader_node`; not reused for the stub map, since it yields only one hot cell.

## Diagrams

```
                 /heatmap (RELIABLE, TRANSIENT_LOCAL)
 stub_heatmap_publisher ------------+---------------+
 (Nov 24: belief_map_node)          |               |
                                    v               v
 /iris_1/navsat -> localization -> /iris_1/pose -> planner iris_1 -> /ap/v1/cmd_gps_pose
                  (shared datum)        |            |  ^
                                        |            |  `- /ap/v1/goal_lla (confirm)
                                        v            v
                              planner iris_2 <- /iris_1/goal_sector (1 Hz, expires in 5 s)
                              planner iris_3
 arm_takeoff (Fast DDS, one per drone) -> /ap/vN/{mode_switch, arm_motors, takeoff} -> exits
```

```
Planner state, checked at 1 Hz:

 disabled --enable true--> HOLDING --heatmap and at layer--> choose goal
    ^                        ^                                   |
    | enable false:          | no open sector better             v
    | one hold command       | than current + margin        ASCENDING
    |                        |                               |     |
    +------------------------+-- goal blocked: re-decide <---+     | within 2 m and at layer
                             ^                                     v
                             +-- best option beats current ---- ARRIVED
```

## Failure modes

| Path | Realistic failure | Test | Handling | Visible? |
|---|---|---|---|---|
| Shared frame | A launch omits the datum on one drone | yes | none | wrong positions, visible via test only |
| Goal command | Not GUIDED or not armed, dropped by autopilot | manual | 1 Hz resend, warn if goal_lla never matches | yes, warning |
| Goal command | `frame_id` wrong, dropped by autopilot | yes | none needed once tested | test catches it |
| Peer goals | Peer crashes | yes | expires after 5 s | yes, warning |
| Altitude gate | Climb stalls below layer | yes | stays HOLDING | yes, status topic |
| Heatmap | Never arrives | yes | stays HOLDING | yes, status topic |
| Evidence | Detector stays quiet on empty frames | no | none yet; TODO records the contract | **silent: critical gap until the detector is written** |
| RMW | Helper started without Fast DDS, services hang | manual | launch sets it | helper blocks, visible in launch log |

## Worktree parallelization strategy

| Step | Modules touched | Depends on |
|------|----------------|------------|
| Planner library and tests | `src/uav_planning/core/`, its tests | none |
| Shared datum | `src/uav_localization/` | none |
| Node, helper, stub | `src/uav_planning/` | Planner library |
| Swarm launch and SITL runs | `src/uav_planning/launch/`, `.gitignore` | all above |

Parallel lanes: Lane A: library, then node, helper and stub (shared `uav_planning`). Lane B: shared datum (independent). Execution order: launch A and B together, merge both, then swarm launch. Conflict flags: none between lanes. With a 12-15 h week and one developer, running these in order on `main` is also fine.

## Implementation Tasks

Synthesized from this review's findings. Each task derives from a specific finding above. Run with Claude Code or Codex; checkbox as you ship.

- [ ] **T1 (P1, human: ~1h / CC: ~5min)** — localization — shared world datum with unset regression
  - Surfaced by: Architecture R1 (D7)
  - Files: `src/uav_localization/uav_localization/localization_node.py`, launch, test
  - Verify: pytest in container; existing single-drone launch still latches
- [ ] **T2 (P1, human: ~1 day / CC: ~40min)** — uav_planning — ROS-free library and tests
  - Surfaced by: R10, R11, R12, R13, R15, R16, R17, R19, R20, R23 (D16-D28)
  - Files: `src/uav_planning/uav_planning/core/planner.py`, `src/uav_planning/test/test_planner.py`, package boilerplate
  - Verify: pytest; coordination harness holds all peaks across seeds on 8x8
- [ ] **T3 (P1, human: ~1 day / CC: ~40min)** — uav_planning — `follower_planner_node`
  - Surfaced by: R2, R3, R4, R6, R21, R22, R24 (D8-D12, D26, D27, D29)
  - Files: `src/uav_planning/uav_planning/follower_planner_node.py`, launch
  - Verify: SITL, one follower flies to a stub sector and reports ARRIVED (Nov 10)
- [ ] **T4 (P1, human: ~3h / CC: ~15min)** — uav_planning — `arm_takeoff` and `stub_heatmap_publisher`
  - Surfaced by: R9 (D15), R14 (D19)
  - Files: `src/uav_planning/uav_planning/`
  - Verify: arm helper exits after climb under Fast DDS; latched map received by a late subscriber
- [ ] **T5 (P1, human: ~half day / CC: ~20min)** — uav_planning — swarm launch; ignore `mav_*.parm`
  - Surfaced by: Scope finding 1, finding 4
  - Files: `src/uav_planning/launch/`, `.gitignore`
  - Verify: 3 followers each hold a different peak on 8x8 (Nov 15)
- [ ] **T6 (P1, human: ~2h / CC: ~30min)** — sim — camera to nadir and expose `mount_quat`
  - Surfaced by: R25 (D30)
  - Files: a project mount params file, `src/uav_mapping/launch/heatmap.launch.py`
  - Verify: marker test places the below-drone marker at image centre
- [ ] **T7 (P2, human: ~2h / CC: ~30min)** — sim — speed timebox
  - Surfaced by: R18 (D23)
  - Files: none expected, launch settings if a fix is found
  - Verify: re-measured RTF at 1 and 4 iris with a working readiness check
- [ ] **T8 (P1, human: ~half day / CC: ~30min)** — integration — Nov 20 search checkpoint, with sweep controller and detector in the D33 order
  - Surfaced by: R28 (D33), R27 (D32)
  - Files: to be set by that work
  - Verify: one follower completes observe, update, depart, find

Effort assumes features at about 30x and tests at about 50x human-to-CC time, adjusted down for ROS integration.

## Unresolved decisions

None. N in the D32 confirmation rule and the D14 thresholds are provisional values by decision, not open questions.

## Completion summary

- Step 0: Scope Challenge — scope reduced per recommendation
- Architecture Review: 9 issues found
- Code Quality Review: 7 issues found
- Test Review: diagram produced, 22 planned paths without tests, 1 plan issue (R17)
- Performance Review: 1 issue found
- NOT in scope: written
- What already exists: written
- TODOS.md updates: 3 items proposed to user, 3 added
- Failure modes: 1 critical gap flagged
- Unresolved decisions: 0 in this review
- Outside voice: Codex, completed, 8 findings
- Parallelization: 2 lanes, 2 parallel / 1 sequential after merge
- Lake Score: 2/3 (D16 and D17 took the 10/10 option; D18 took 8/10)

## Suppressed findings

- [P3] (confidence: 4/10) `docker/Dockerfile:62` says CycloneDDS is recommended for AP_DDS, yet services hang under it. Possibly a version-specific issue; not investigated.
- [P3] (confidence: 4/10) Three `mavproxy.py` processes run in the SITL launch at about 9% CPU each, against the "no MAVProxy in the runtime path" rule. Considered in the D23 timebox.
- [P3] (confidence: 3/10) `localization_node.py:159` (`up = alt - self._alt0`): z = 0 sits 0.19 m above ground with the first-fix latch; the shared datum may remove this. Negligible at 10 m.

## GSTACK REVIEW REPORT

| Review | Trigger | Why | Runs | Status | Findings |
|--------|---------|-----|------|--------|----------|
| CEO Review | `/plan-ceo-review` | Scope & strategy | 0 | — | — |
| Outside Review | Codex CLI, plan review | Independent 2nd opinion | 1 | completed | 8 findings, all resolved or recorded |
| Eng Review | `/plan-eng-review` | Architecture & tests (required) | 1 | ISSUES OPEN | 17 issues + 1 test-plan issue, 1 critical gap |
| Design Review | `/plan-design-review` | UI/UX gaps | 0 | — | — |
| DX Review | `/plan-devex-review` | Developer experience gaps | 0 | — | — |

- **OUTSIDE COVERAGE:** Codex, plan-review phase, completed, 8 findings; 7 led to approved changes, 1 suggestion (defer baselines) recorded as a disagreement.
- **CROSS-MODEL:** Codex raised no objection to the direct flying rule or the stay rule, and found goal-conflict, reservation-expiry, altitude-gate, observation, map-meaning, confirmation and checkpoint gaps the native review missed, plus one native error (arrival radius). Model identity for Codex: as configured, not separately reported.
- **VERDICT:** No review is CLEAR. "ISSUES OPEN" here means findings mapped to approved work, with no decision pending. Eng review required before cleared status; ready to implement against the tasks above.

NO UNRESOLVED DECISIONS
