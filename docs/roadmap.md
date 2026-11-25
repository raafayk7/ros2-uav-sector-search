# Roadmap: language-prior multi-UAV search (technical)

Public-safe extract of the project plan: architecture, algorithms, eval and milestones only. Personal and career planning lives outside the repo on purpose.

## Pitch

**"The language model decides where to look. A classical distributed planner decides how to fly."**

An operator types a field report ("two hikers last seen near the river, heading north, one injured"). An LLM turns it into a structured prior probability map. Follower drones launch straight into the high-prior areas while the leader is still sweeping, and the leader's detections update the map as they arrive. Low-level flight stays classical: each follower independently picks the best open sector on the shared map and ArduPilot flies it there, with the thesis's overlap penalty and per-drone altitude layers keeping drones apart. The thesis's distributed gradient ascent (DGA) and BLL remain as measured baselines (see `docs/planner-decisions.md`). The LLM runs at mission speed (seconds), so its latency does not matter.

## Contribution claim (reworded after the literature check, 2026-11-22; re-check before publishing)

Never "nobody has combined LLMs with classical drone control" (false; the area is busy). The claim is the measured eval on a specific system:

**What a wrong language-derived search prior costs, measured.** An LLM's only output is a prior map written from an operator's field report. Every camera view without a detection lowers the map, a leader always completes a full sweep, and each follower independently flies to the best open sector on the shared map. On that system we compare uniform, accurate and misleading clues, plus a misleading clue with negative evidence off (time to first find, time to all found with censoring, coverage at timeout). The pipeline is assembled from known parts (cite LMPath, Language as a Sensor, arXiv 2611.16524, Bourgault-style Bayesian search, 2611.12317). **The eval is the contribution, not the LLM.**

Replaced wording (session 12, kept for the record): "natural-language incident report -> spatial probabilistic prior -> Bayesian posterior with negative evidence -> distributed multi-UAV information-seeking controller". It overstated the system: fusion has one owner, so the posterior is not distributed, and the follower rule is best open sector, not information gain.

Closest prior pattern to cite, not hide: "Bootstrapped LLM Semantics for Context-Aware Path Planning" (arXiv 2511.11967), where the LLM acts as a stochastic semantic sensor feeding a classical planner (single robot, per-class risk, not search).

**Status (literature check done 2026-11-22, session 12, `docs/literature-check.md`): novel only in the eval.** No paper runs the whole chain, but every link has direct prior work, and the closest systems are from Oct-Nov 2026 (LMPath 2605.13782, Language as a Sensor 2606.08666, the LLM drone scheduler 2611.16524). No paper tests a misleading language clue against uniform and accurate ones in multi-UAV search. 2611.12317 found that a strongly biased map did about half as well as no map, for a planner whose map never updates. **The user accepted the reworded claim above and the fourth eval arm on 2026-11-22** (both from the literature check). Re-run the search before any README claim (Dec 20 write-up).

## Architecture (ROS 2 Jazzy workspace)

| Layer | Node / piece | Status |
|---|---|---|
| Sim | Docker: Jazzy + Gazebo Harmonic + ArduPilot SITL (AP_DDS) | done, verified 2026-11-02 |
| State | `uav_localization/localization_node` (optional shared world datum, built 2026-11-07; unset keeps the first-fix latch) | done |
| Leader | `uav_mapping/leader_node`: detections + camera footprints, plus a boustrophedon sweep controller | heatmap done; sweep controller done 2026-11-17 (`uav_planning/sweep_node`, D39: 8x8 arena at 5 m, 5 m/s, 3 lanes at 8 m, COMPLETE in about 40 s, no unknown cells after one sweep); footprints done 2026-11-18 (`belief_map_node` computes them from each drone's pose, D40; no separate topic); prior-biased lane order built as `core/sweep.py lane_order_by_prior` and off (D53), not wired to `sweep_node` |
| Belief map | `belief_map_node`: **single owner of the posterior**; fuses detections + footprints from all agents; followers only read it. Evidence per drone and cell is capped at +-0.85 log-odds per look (R29, D40). A target is confirmed when a follower within arrival radius sees it in its sector for 3 consecutive frames (D40); that whole sector then publishes as 0, so a second target in it stays hidden | done 2026-11-18 (`uav_mapping/core/belief.py`, 33 tests; takes an optional starting belief since 2026-11-24, D45, not yet passed by the node); leader + 1 follower found all 3 targets in two cold runs (`swarm.launch.py leader:=true`) |
| Followers | `follower_planner_node`: best open sector over the posterior, skipping sectors peers have claimed or are parked on (D11, D24, D25; no graded overlap penalty), altitude layers (leader 8 m, followers 10 + 2 x index m); thesis DGA and BLL as library baselines; `arm_takeoff` helper handles arming | library done and tested 2026-11-07; node, `arm_takeoff` and stub map built, and one follower flown to ARRIVED in SITL 2026-11-08 (`follower.launch.py`); swarm launch built and 3 followers each holding a different peak 2026-11-12 (`swarm.launch.py`, T5; Nov 15 met) |
| Perception | primary: ground-truth detector (Gazebo target poses + noise, misses, false positives, footprint check), publishing on every frame including empty ones; stretch: YOLO. Requires the camera at nadir: done 2026-11-16 (T6, `uav_planning/config/gimbal_nadir.parm`; marker below the drone 1 px from image centre, at hover). Pointing in sideways flight checked 2026-11-17 at 5 m/s: marker error median 3 px, max 7 (T8 verify-first, passed). Ground-truth detector built 2026-11-17 (`uav_perception`, noise D38, checked live against `leader_node` in a 1-iris hover); per-drone swarm wiring done 2026-11-18 (one detector per drone, about 2 % RTF each at 2 drones); YOLO still to do | detector done, YOLO stretch |
| Mission | state machine (D41): `mission_node` in `uav_planning`. WAITING -> SEARCHING -> DONE or TIMED_OUT; planners launch disabled and the mission enables them; ends when the sweep is complete and every unconfirmed sector is below 0.5 for 30 s (D14), or after 600 sim-s, then disables the planners (D29 hold) and publishes a latched `/mission_report` (finds with drone and time from `belief_map_node`'s `/confirmed_finds`) | built and flown 2026-11-20 (session 11): one cold mission from one command (`swarm.launch.py leader:=true num_followers:=3`) ended DONE 31.3 s after the sweep with all 3 finds reported by the right drones; a second DONE run at another seed, and one TIMED_OUT run (`mission_timeout:=60`). Sim time from the followers' pose stamps: the report's `sim_s` matched Gazebo's clock to 0.08 s. **Return to find** (D41 amended, built and flown 2026-11-22, session 13): after DONE each finder flies to hover over its find, all three within 0.1 m 19 s later; latched `/mission_return`. **Follower timing** (D42): followers start with the sweep, as in this row's "planners enabled in SEARCHING" |
| Language prior | `mission_prior_node` (contract below) | Built and flown 2026-11-24 (sessions 16-17, D44-D49): ROS-free `core/prior.py`, `landmarks.py`, `live.py`, `worlds/runway/`; `mission_prior_node` publishes one latched `/mission_prior` and `belief_map_node` waits for it (`clue:=` on `swarm.launch.py`, replay by default). The `live` provider called a model for the first time on 2026-11-25 (gemini-3.8-flash, D51); `worlds/runway` now holds 2 live and 3 hand-written recordings, `worlds/valley` live ones only |
| Eval | 2D kinematic sim runner reusing the planner and fusion library | built 2026-11-25 (session 18): `src/uav_eval` (`python3 -m uav_eval.run`), 120 m `worlds/valley` (D52), four arms, strict live recordings, 8-seed pilot run; the 20 x 4 results are session 19 |
| CI | GitHub Actions: `ci.yml` (hosted, build + unit tests) and `smoke.yml` (self-hosted, the mission) | built 2026-11-23 (session 15); both workflows green on GitHub the same day (`ci.yml` run 45134386862; `smoke.yml` on the self-hosted runner, run 45134386843) |

**Code structure rule:** the sector planner, its DGA and BLL baselines, fusion and region rasterization are a **pure Python library with no ROS dependency**, unit-tested at the library level. ROS nodes are thin wrappers. This is what lets the 2D eval sim reuse the code unchanged.

## Mission timing

- **t = 0:** the clue becomes the prior. The leader starts its full sweep in its default lane order (the plan was high-prior lanes first; measured in the 2D sim it slowed both kinds of clue, so it is off, D53). Followers launch immediately and fly to the prior's best open sectors.
- **During the mission:** the posterior updates incrementally. Detections raise nearby cells. **Negative evidence** (a footprint with no detection) lowers covered cells, which pushes followers off a wrong prior's peak.
- **Coverage guarantee:** the leader always completes the full sweep whatever the prior says. A wrong clue can delay a find but never prevent coverage.
- **Baseline:** same timing with a flat prior. Followers hold on a flat map until evidence changes values, so the 2D sim must show that a uniform run explores, using follower footprints, before any uniform-baseline result is reported. (It does: in the pilot's 8 uniform runs all 15 targets were found, median time to all 21.5 s against a 206 s sweep.)

## Fusion

Each cell holds the independent probability that it contains at least one target; values need not sum to 1 (this handles an unknown target count and matches `leader_node`).

A cell's starting belief = alpha * prior_LLM + (1 - alpha) * 0.5, with prior_LLM = 0.5 + 0.5 u and u = region weight x confidence x fraction of the cell inside the region (raise only: a cell in no region has u = 0 and stays unknown, D44-D46), converted to log-odds, then updated per camera footprint with detections and negative evidence, extending the existing leader heatmap update. alpha is capped at 0.7; the eval uses a fixed 0.5. The 0.5 share keeps every cell above zero. The blend is unit-tested and was probed in 2D point-mass runs (session 16 notes in `docs/planner-decisions.md`), not flown. Alpha hardly matters there, because the follower rule uses only the ranking of cells.

## LLM contract (`mission_prior_node`)

- **Input:** clue text + a per-world landmark file (`worlds/<name>/landmarks.yaml`: points, polylines, polygons in ENU metres) + map bounds.
- **Output (strict JSON):** regions, each a circle or a capsule, with weight and confidence, cut off at the map's edge (never moved). One flat list, one region per place, overlaps take the maximum (D46). Invalid output -> one retry -> uniform prior with a logged warning.
- **Providers:** `live` (the model's name picks the vendor, gemini-* or claude-*, and with it the one key it may use, `GEMINI_API_KEY` or `ANTHROPIC_API_KEY`; keys come from the environment, normally the gitignored `docker/llm.env` via `docker exec --env-file`, never committed; D51) and `replay` (committed recorded outputs, found by world, clue and landmark fingerprint exactly, else uniform with a warning; D48). The eval accepts only `live` recordings of one model (gemini-3.8-flash, the user's choice). Default compose profile and CI use `replay`, so `docker compose up` works with no key.
- No `temperature` is sent: current Claude models accept only their default sampling and answer a request that sets it with a 400 (found 2026-11-24, session 17; the plan said temperature 0). Repeatability comes from replay instead: a live answer is saved as a recording (`record_prior`, `save_recording`) and the eval uses only those frozen outputs.
- **Hard rule:** the LLM's only output is the prior. Nothing it produces reaches a velocity command, setpoint or mode change.

## Eval design

- Lightweight 2D kinematic sim (point-mass drones, camera footprint model, same planner/fusion library), faster than real time. Gazebo is used for 3 showcase runs for video, not for statistics.
- 20 seeds (0-19; pilots use 1000 and up), 1-3 targets each, each within 8 m of its own landmark in a fixed 120 x 120 m world (`worlds/valley`, D52). Every drone at 5 m/s, no climb or acceleration.
- Conditions: uniform, accurate clue, misleading clue (same template, every target's landmark swapped by seed for one at least a cell clear of every target, fixed before any LLM call; D47), and misleading clue with negative evidence off, all at alpha = 0.5. The fourth arm sets `BeliefParams.p_miss = 0.5`, so an empty look adds 0 log-odds while detections (`p_hit`) are unchanged. It is the condition 2611.12317 measured (a fixed map), and it separates how much robustness comes from negative evidence and how much from the sweep (added 2026-11-22, session 12). 20 seeds x 4 conditions = 80 runs.
- Found (scoring only, since a running system lacks true positions) = a detection within 2 m of the true position. Timeout 15 sim-minutes; unfound targets are censored and counted.
- Report median time-to-first-find, time-to-all-found (with censor count) and coverage at timeout. Not time to DONE: with negative evidence off the map never falls below 0.5, so that arm always times out (session 16 probe, 20 of 20).
- D47's swap is drawn per scenario before any LLM call; a scenario with no valid swap has its targets redrawn for all four arms, and the count is reported. A wrong landmark's surroundings count out to 20 m, above the largest radius the model gave "near" (18 m in the pilot; D54). Seed-by-seed (paired) differences are reported beside the medians, and the clue's-lanes-first sweep as an ablation (D53). Coverage is 100 % in every pilot run, because the sweep always finishes, so it separates no arm. Also reported: how often the LLM's regions covered each target, per arm.

## CI scope

Smoke test: 1 leader + 1 follower, ground-truth detector (no camera rendering, so no GPU), `replay` prior (not in the smoke test yet: nothing reads a prior until session 17), 10-minute timeout; pass = reaches "target localized" or completes the sweep. **As built (session 15):** the arena starts empty, so the smoke script spawns the three session-8 targets and the checker fails a DONE with no finds; the full image (25 GB) cannot be built or pulled on a hosted runner (about 14 GB free), so the smoke test is a separate self-hosted job (manual dispatch only, never pull requests) and the hosted job runs only build + unit tests in the stock `ros:jazzy` image. Plus unit tests for fusion and rasterization and a 3-seed eval-sim regression.

## Milestones

| Date | Milestone |
|---|---|
| Nov 10 | Planner library + `follower_planner_node`, first follower flying to a stub-map sector in SITL; secret scan clean, repo public as a WIP |
| **Nov 15** | **3 followers each holding a different peak on the 8x8 stub map in SITL** (cut-order trigger). **Met 2026-11-12**, 3 cold runs, RTF about 0.3-0.44 |
| Nov 20 | One follower completes the search loop: observes, map updates, departs, finds a target. **Met 2026-11-18**, 2 cold runs: leader + 1 follower, all 3 targets confirmed before the sweep ended |
| Nov 24 | Mission state machine end to end. Build order from Nov 16: camera to nadir (2 h timebox, done Nov 16) and sim speed (2 h timebox, done Nov 16: a 2 ms physics step doubles RTF, now the swarm default, D37); ground-truth detector (done Nov 17); sweep controller (done Nov 17, D39); `belief_map_node` + footprints (done Nov 18, D40); Nov 20 checkpoint (met Nov 18); state machine (designed Nov 20, D41; `mission_node` built and flown end to end on Nov 20, session 11: **met**, DONE from one command with a correct report) |
| Nov 25-29 | Literature check on the narrowed contribution claim (scheduled 2026-11-18): closest prior work, whether the claim holds, rewording if needed, in `docs/literature-check.md`. Before the LLM layer and any README claim. **Done 2026-11-22** (session 12): novel only in the eval; the user accepted the rewording and the fourth eval arm the same day |
| **Nov 30** | **Go/no-go:** leader + 3 followers complete a mission in Gazebo (ground-truth detector) from one command; video v1. **GO, 2026-11-22** (sessions 13-14): the mission part was met cold from one command, headless and with the Gazebo window, and video v1 exists (`docs/media/uav-v1.mp4`, 72 s, recorded with OBS from a run to DONE and AT_FINDS). Recording with OBS roughly halved the sim's speed (`docs/planner-decisions.md`, session 14 notes) |
| Dec 8 | CI green. **Met 2026-11-23 (session 15):** hosted `ci.yml` green on GitHub (run 45134386862: 223 passed, 2 skipped, 80 s); `smoke.yml` green on the self-hosted runner (run 45134386843: 225 tests, then mission DONE with 3 finds, 6 min 30 s for the job, sweep started at 91 s, DONE at 302 s of the checker's wall time). By hand before that: DONE with 3 finds in 97 s (dev container) and 317 s (clean checkout). The self-hosted runner is on the user's machine, so the smoke job only runs while it is on, with the USB mounted and no sim running |
| Dec 10 | Language prior drives a mission end to end. **Met 2026-11-24 (session 17):** `swarm.launch.py leader:=true clue:="..."` (replay, no key) flew the hut clue and the radio mast clue (the misleading twin) to DONE with all 3 finds, and a run without a clue still ends DONE; the clue changed the first goal sectors. One cold run each, not a measurement. Not shown: a real language model's output (done 2026-11-25, session 18: gemini-3.8-flash put the hut and mast clues on the right places), leader lane order (moved to the eval sim, D50; built and off, D53) |
| Dec 20 | Results table over 20 scenarios x 4 conditions, 3 Gazebo showcase runs, video v2. **Half done (session 18, 2026-11-25):** the eval sim, the world, the recording tool and an 8-seed pilot exist; the 40 final clues are not asked yet, nothing is reported |

No-go on Nov 30 means weeks 6-7 finish the base system, a second check on Dec 14 applies the same criteria, and the LLM layer moves out.

## Cut order if the Nov 15 milestone slips by more than 3 days

1. Drop to 2 followers (as in the thesis hardware run).
2. Defer the CI smoke test and README GIF.
3. Drop the LLM layer and ship a polished base system.

## Out of scope

Any UI beyond RViz/Gazebo and the README; GHCR or release engineering beyond a working `docker compose`; benchmarking several LLM providers (one `live` model only); a large Gazebo statistical experiment (stats come from the 2D sim); a general-purpose swarm framework; YOLO is a stretch goal only.

## Open engineering questions (settle in `/plan-eng-review` before week-2 planner code)

Settled in `docs/planner-decisions.md`: follower behavior on a flat posterior, post-find peak suppression, mission termination with an unknown target count, the true-find definition at run time. Settled 2026-11-24 (session 16): region rasterization rule (D44), default alpha and the outside level (D45), multi-target clue mapping (D46), misleading-landmark rule (D47), replay lookup for unmatched clues (D48). Still open: the eval's run matrix and arena size (the session 16 probe says larger than 40 m, where a clue barely matters); sweep time vs timeout for the eval sim (fact from D39: lane flight takes about area / (15 m x 5 m/s), 40 s on the 40 x 40 m arena, so a 15-minute timeout binds only near 250 m a side, and prior-biased lane order can only pay off on an arena much larger than the default; Gazebo runs use D41's 600 sim-s, and with 4 drones the sweep took 68-76 s wall); CI vehicle model and runtime (settled 2026-11-23, D43: the iris, 1 leader + 1 follower, 97-317 s wall to DONE, 600 s timeout). Also open, the user's call (raised 2026-11-22): precision landing, the thesis's Phase 3, is in no milestone and not on the out-of-scope list; schedule it or declare it out of scope. And whether the eval gets a thesis-timing arm (followers after the sweep, D42).
