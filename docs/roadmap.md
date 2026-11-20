# Roadmap: language-prior multi-UAV search (technical)

Public-safe extract of the project plan: architecture, algorithms, eval and milestones only. Personal and career planning lives outside the repo on purpose.

## Pitch

**"The language model decides where to look. A classical distributed planner decides how to fly."**

An operator types a field report ("two hikers last seen near the river, heading north, one injured"). An LLM turns it into a structured prior probability map. Follower drones launch straight into the high-prior areas while the leader is still sweeping, and the leader's detections update the map as they arrive. Low-level flight stays classical: each follower independently picks the best open sector on the shared map and ArduPilot flies it there, with the thesis's overlap penalty and per-drone altitude layers keeping drones apart. The thesis's distributed gradient ascent (DGA) and BLL remain as measured baselines (see `docs/planner-decisions.md`). The LLM runs at mission speed (seconds), so its latency does not matter.

## Contribution claim (narrowed; verify against literature before publishing)

Never "nobody has combined LLMs with classical drone control" (false; the area is busy). The claim is the specific chain:

**natural-language incident report -> spatial probabilistic prior -> Bayesian posterior with negative evidence -> distributed multi-UAV information-seeking controller, with the LLM kept out of flight-critical control**, plus a *measured* comparison of uniform vs accurate vs misleading clues. **The eval is the contribution, not the LLM.**

Closest prior pattern to cite, not hide: "Bootstrapped LLM Semantics for Context-Aware Path Planning" (arXiv 2511.11967), where the LLM acts as a stochastic semantic sensor feeding a classical planner (single robot, per-class risk, not search). A literature check on the narrowed claim comes before any README claim.

## Architecture (ROS 2 Jazzy workspace)

| Layer | Node / piece | Status |
|---|---|---|
| Sim | Docker: Jazzy + Gazebo Harmonic + ArduPilot SITL (AP_DDS) | done, verified 2026-11-02 |
| State | `uav_localization/localization_node` (optional shared world datum, built 2026-11-07; unset keeps the first-fix latch) | done |
| Leader | `uav_mapping/leader_node`: detections + camera footprints, plus a boustrophedon sweep controller | heatmap done; sweep controller done 2026-11-17 (`uav_planning/sweep_node`, D39: 8x8 arena at 5 m, 5 m/s, 3 lanes at 8 m, COMPLETE in about 40 s, no unknown cells after one sweep); footprints done 2026-11-18 (`belief_map_node` computes them from each drone's pose, D40; no separate topic); prior-biased lane order (a `lane_order` parameter exists) still to do |
| Belief map | `belief_map_node`: **single owner of the posterior**; fuses detections + footprints from all agents; followers only read it. Evidence per drone and cell is capped at +-0.85 log-odds per look (R29, D40). A target is confirmed when a follower within arrival radius sees it in its sector for 3 consecutive frames (D40); that whole sector then publishes as 0, so a second target in it stays hidden | done 2026-11-18 (`uav_mapping/core/belief.py`, 33 tests); leader + 1 follower found all 3 targets in two cold runs (`swarm.launch.py leader:=true`) |
| Followers | `follower_planner_node`: best open sector over the posterior, thesis overlap penalty, altitude layers (leader 8 m, followers 10 + 2 x index m); thesis DGA and BLL as library baselines; `arm_takeoff` helper handles arming | library done and tested 2026-11-07; node, `arm_takeoff` and stub map built, and one follower flown to ARRIVED in SITL 2026-11-08 (`follower.launch.py`); swarm launch built and 3 followers each holding a different peak 2026-11-12 (`swarm.launch.py`, T5; Nov 15 met) |
| Perception | primary: ground-truth detector (Gazebo target poses + noise, misses, false positives, footprint check), publishing on every frame including empty ones; stretch: YOLO. Requires the camera at nadir: done 2026-11-16 (T6, `uav_planning/config/gimbal_nadir.parm`; marker below the drone 1 px from image centre, at hover). Pointing in sideways flight checked 2026-11-17 at 5 m/s: marker error median 3 px, max 7 (T8 verify-first, passed). Ground-truth detector built 2026-11-17 (`uav_perception`, noise D38, checked live against `leader_node` in a 1-iris hover); per-drone swarm wiring done 2026-11-18 (one detector per drone, about 2 % RTF each at 2 drones); YOLO still to do | detector done, YOLO stretch |
| Mission | state machine (D41): `mission_node` in `uav_planning`. WAITING -> SEARCHING -> DONE or TIMED_OUT; planners launch disabled and the mission enables them; ends when the sweep is complete and every unconfirmed sector is below 0.5 for 30 s (D14), or after 600 sim-s, then disables the planners (D29 hold) and publishes a latched `/mission_report` (finds with drone and time from `belief_map_node`'s `/confirmed_finds`) | built and flown 2026-11-20 (session 11): one cold mission from one command (`swarm.launch.py leader:=true num_followers:=3`) ended DONE 31.3 s after the sweep with all 3 finds reported by the right drones; a second DONE run at another seed, and one TIMED_OUT run (`mission_timeout:=60`). Sim time from the followers' pose stamps: the report's `sim_s` matched Gazebo's clock to 0.08 s |
| Language prior | `mission_prior_node` (contract below) | todo, after go/no-go |
| Eval | 2D kinematic sim runner reusing the planner and fusion library | todo |
| CI | GitHub Actions smoke test | todo |

**Code structure rule:** the sector planner, its DGA and BLL baselines, fusion and region rasterization are a **pure Python library with no ROS dependency**, unit-tested at the library level. ROS nodes are thin wrappers. This is what lets the 2D eval sim reuse the code unchanged.

## Mission timing

- **t = 0:** the clue becomes the prior. The leader starts its full sweep with lane order biased so high-prior lanes go first. Followers launch immediately and fly to the prior's best open sectors.
- **During the mission:** the posterior updates incrementally. Detections raise nearby cells. **Negative evidence** (a footprint with no detection) lowers covered cells, which pushes followers off a wrong prior's peak.
- **Coverage guarantee:** the leader always completes the full sweep whatever the prior says. A wrong clue can delay a find but never prevent coverage.
- **Baseline:** same timing with a flat prior. Followers hold on a flat map until evidence changes values, so the 2D sim must show that a uniform run explores, using follower footprints, before any uniform-baseline result is reported.

## Fusion

Each cell holds the independent probability that it contains at least one target; values need not sum to 1 (this handles an unknown target count and matches `leader_node`).

A cell's starting belief = alpha * prior_LLM + (1 - alpha) * 0.5, converted to log-odds, then updated per camera footprint with detections and negative evidence, extending the existing leader heatmap update. alpha is capped at 0.7; the eval uses a fixed 0.5. The 0.5 share keeps every cell above zero. This blend is untested.

## LLM contract (`mission_prior_node`)

- **Input:** clue text + a per-world landmark file (`worlds/<name>/landmarks.yaml`: points, polylines, polygons in ENU metres) + map bounds.
- **Output (strict JSON):** regions, each a circle or a capsule, with weight and confidence, clamped to the map. Invalid output -> one retry -> uniform prior with a logged warning.
- **Providers:** `live` (key only from `LLM_API_KEY`, never committed) and `replay` (committed recorded outputs). Default compose profile and CI use `replay`, so `docker compose up` works with no key.
- Temperature 0. The eval uses only frozen replay outputs.
- **Hard rule:** the LLM's only output is the prior. Nothing it produces reaches a velocity command, setpoint or mode change.

## Eval design

- Lightweight 2D kinematic sim (point-mass drones, camera footprint model, same planner/fusion library), faster than real time. Gazebo is used for 3 showcase runs for video, not for statistics.
- 20 seeds, 1-3 targets each, placed relative to landmarks in a fixed world.
- Conditions: uniform, accurate clue, misleading clue (same template, wrong landmark), alpha = 0.5.
- Found (scoring only, since a running system lacks true positions) = a detection within 2 m of the true position. Timeout 15 sim-minutes; unfound targets are censored and counted.
- Report median time-to-first-find, time-to-all-found (with censor count) and coverage at timeout.

## CI scope

Smoke test: 1 leader + 1 follower, ground-truth detector (no camera rendering, so no GPU), `replay` prior, 10-minute timeout; pass = reaches "target localized" or completes the sweep. Plus unit tests for fusion and rasterization and a 3-seed eval-sim regression.

## Milestones

| Date | Milestone |
|---|---|
| Nov 10 | Planner library + `follower_planner_node`, first follower flying to a stub-map sector in SITL; secret scan clean, repo public as a WIP |
| **Nov 15** | **3 followers each holding a different peak on the 8x8 stub map in SITL** (cut-order trigger). **Met 2026-11-12**, 3 cold runs, RTF about 0.3-0.44 |
| Nov 20 | One follower completes the search loop: observes, map updates, departs, finds a target. **Met 2026-11-18**, 2 cold runs: leader + 1 follower, all 3 targets confirmed before the sweep ended |
| Nov 24 | Mission state machine end to end. Build order from Nov 16: camera to nadir (2 h timebox, done Nov 16) and sim speed (2 h timebox, done Nov 16: a 2 ms physics step doubles RTF, now the swarm default, D37); ground-truth detector (done Nov 17); sweep controller (done Nov 17, D39); `belief_map_node` + footprints (done Nov 18, D40); Nov 20 checkpoint (met Nov 18); state machine (designed Nov 20, D41; `mission_node` built and flown end to end on Nov 20, session 11: **met**, DONE from one command with a correct report) |
| Nov 25-29 | Literature check on the narrowed contribution claim (scheduled 2026-11-18): closest prior work, whether the claim holds, rewording if needed, in `docs/literature-check.md`. Before the LLM layer and any README claim |
| **Nov 30** | **Go/no-go:** leader + 3 followers complete a mission in Gazebo (ground-truth detector) from one command; video v1 |
| Dec 8 | CI green |
| Dec 10 | Language prior drives a mission end to end |
| Dec 20 | Results table over 20 scenarios x 3 conditions, 3 Gazebo showcase runs, video v2 |

No-go on Nov 30 means weeks 6-7 finish the base system, a second check on Dec 14 applies the same criteria, and the LLM layer moves out.

## Cut order if the Nov 15 milestone slips by more than 3 days

1. Drop to 2 followers (as in the thesis hardware run).
2. Defer the CI smoke test and README GIF.
3. Drop the LLM layer and ship a polished base system.

## Out of scope

Any UI beyond RViz/Gazebo and the README; GHCR or release engineering beyond a working `docker compose`; benchmarking several LLM providers (one `live` model only); a large Gazebo statistical experiment (stats come from the 2D sim); a general-purpose swarm framework; YOLO is a stretch goal only.

## Open engineering questions (settle in `/plan-eng-review` before week-2 planner code)

Settled in `docs/planner-decisions.md`: follower behavior on a flat posterior, post-find peak suppression, mission termination with an unknown target count, the true-find definition at run time. Still open: region rasterization rule; default alpha and run matrix; sweep time vs timeout for the eval sim (fact from D39: lane flight takes about area / (15 m x 5 m/s), 40 s on the 40 x 40 m arena, so a 15-minute timeout binds only near 250 m a side, and prior-biased lane order can only pay off on an arena much larger than the default; Gazebo runs use D41's 600 sim-s, and with 4 drones the sweep took 68-76 s wall); multi-target clue mapping and the misleading-landmark rule; CI vehicle model and runtime; replay lookup for unmatched clues.
