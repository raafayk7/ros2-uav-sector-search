# Next-session prompt: the mission node, end to end (session 11)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 10 is done (2026-11-20). Leader + 3 followers ran live twice, cold. All 3 targets were confirmed, 15-34 s into a 68-76 s sweep, with no false confirmations. RTF while hovering with 4 detectors was 0.54-0.61. A target 0.3 m inside a sector edge was confirmed in the right sector. The user decided D41, the mission state machine: `mission_node` in `uav_planning`, WAITING -> SEARCHING -> DONE or TIMED_OUT, planners launch disabled and the mission enables them, D14's end rule, a 600 sim-s timeout, and a D29 hold at the end. Built: `uav_planning/core/mission.py` (19 tests) and latched `/confirmed_finds` on `belief_map_node` (checked live). `python3 -m pytest src`: 198 passed. From the probe's new `open` column, the end rule would have fired 30 s after the sweep. It has not run as a node yet.

What is left: the Nov 24 milestone (mission end to end) and the Nov 30 go/no-go (leader + 3 followers complete a mission in Gazebo from one command, video v1).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-9 (done 2026-11-07 to 11-18) | Datum, library, node, first flight, swarm, nadir camera, sim speed, detector, sweep, belief map | Nov 10, 15, 20 | mixed | see git history | ~22.5 h |
| 10 (done 2026-11-20) | 4 drones live, edge target, D41 design, mission core | (Nov 24 half) | Opus | Live debugging and architecture | ~2.5 h |
| **11 (this prompt)** | `mission_node`, `planner_enable` off at launch, launch wiring, live end to end from one command, D14 end rule measured | Nov 24 | **Opus** | End-to-end integration in SITL; the first node that drives the others | ~3 h |
| 12 | Literature check on the narrowed contribution claim (roadmap "Contribution claim"), in `docs/literature-check.md`. No sim | before the LLM layer (Dec 10) and any README claim | Opus | Judging novelty against papers | ~1.5-2 h |
| 13 | Go/no-go dress run and video v1 (Gazebo GUI capture) | Nov 30 | Sonnet if 11 lands clean, else Opus | Mostly running and recording | ~2 h |

Session 11 fits before Nov 24 with a day to spare. Sessions 12-13 fit before Nov 30, and 12 needs no sim, so it can swap with 13 on a bad sim day.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 11. Leader + 3 followers find all 3 targets with `swarm.launch.py leader:=true num_followers:=3`, and the mission's rules are a tested ROS-free core (`uav_planning/core/mission.py`, D41). This session wraps the core in `mission_node`, wires it into the swarm launch, and runs one whole mission from one command (Nov 24).

## Read first

- `CLAUDE.md` (Docker gotchas; always `bash -lc` / `bash -l`).
- `docs/planner-decisions.md`: **session 10 notes** (D41's table, both 4-drone runs, the D14 timing, the "not shown" table), then D14, D29 and D24.
- `src/uav_planning/uav_planning/core/mission.py` and `test/test_mission.py`; `follower_planner_node.py` (how `planner_enable` and `goal_sector` work, the `start_enabled` parameter); `sweep_node.py` (`sweep_status` QoS); `swarm.launch.py` (`_leader_actions`, `_follower_actions`).
- `docs/docker-notes.md` gotchas 57-58 (new: renamed `XAUTHORITY` after reboot; check setuptools with `/usr/bin/python3`).

## Task

1. **`mission_node`** (thin wrapper, mirror `sweep_node` and `follower_planner_node`): wall-clock 1 Hz timer calling `Mission.tick`. Subscribes `/iris_1/sweep_status` (latched), `/heatmap` (latched), `/confirmed_finds` (latched String JSON), each follower's `planner_status` and `goal_sector`. Sim time is the latest `goal_sector` header stamp, extrapolated by wall time since it arrived, or None before any. Publishes `planner_enable` to every follower each tick, latched `/mission_state` (state and reason, when either changes) and latched `/mission_report` (JSON) on the end. Log one line per state change and per new find. Parameters: `followers`, `leader`, the three D41 values, topic templates. A node test needs no sim if it only checks parameter parsing; the rules are already tested.
2. **Launch wiring**: `swarm.launch.py leader:=true` starts `mission_node` and passes `start_enabled:=false` to every follower planner (`follower_planner_node.py:110`; a disabled planner still publishes `planner_status` every tick, so the WAITING gate is met, and `arm_takeoff` still climbs it to its layer). Add `mission:=true|false` (default true with the leader) so the session-10 behaviour is one flag away. Rebuild after every edit (gotcha 32).
3. **Live, end to end**: cold, `leader:=true num_followers:=3`, targets from `belief_live_check.py 400 3` (it spawns them; it also prints `open`). Check: planners stay HOLDING until SEARCHING; DONE fires about 30 s after COMPLETE; `/mission_report` lists 3 finds with the right drones and times, and every drone holds afterwards. Take one `rtf_windows.sh` run during SEARCHING (the session-10 windows were in hover only). If time allows, one run with `timeout:=60` to see TIMED_OUT and its report.
4. **D14 values**: from those runs, say whether 0.5 and 30 s held (session 10: the worst false alarm after the sweep was 23 %). Change them only with the user.

Record what was built in `docs/planner-decisions.md` as "Implementation notes (session 11)", add any new trap to `docs/docker-notes.md`, update the roadmap's Mission row and Nov 24 line, and rewrite this file for session 12 (the literature check) or 13.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition: check `mount | grep docker-data` first; if it is missing the daemon hangs (gotcha 38) and the user must plug the USB in. Start a stopped container with `docker compose -f docker/docker-compose.yml start`. If that fails on `.Xauthority` (gotcha 57), `up -d` recreates it; then check `/usr/bin/python3 -c "import setuptools; print(setuptools.__version__)"` reads 68.1.2 before building (gotcha 58). Use `stop`, never `down`.
- **WinBoat must be stopped (gotchas 42, 46):** `docker ps --filter name=WinBoat`; if it is up, tell the user at once.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`. Do not put `pkill -f NAME` in a `bash -lc` string that contains NAME (gotcha 49).
- Session-10 stack, one command from a fresh directory: `rm -rf /tmp/sim1 && mkdir -p /tmp/sim1 && cd /tmp/sim1 && ros2 launch uav_planning swarm.launch.py leader:=true num_followers:=3 detector_seed:=1`, run with `docker compose exec -d` and its output redirected to a log. All four SITLs logged "JSON received" within about 15 s. Then `python3 /workspaces/multi-agent-uavs/docs/probes/belief_live_check.py 300 3` (add `--edge` for the edge target).
- `docker compose restart ros2` between sim runs; `pkill` leaves `arducopter`/`MicroXRCEAgent` behind.
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

`mission_node` built and wired, one live mission from one command ending in DONE with a correct report, the end rule's timing measured, notes recorded, and this file rewritten. Say plainly which changes are plan-level and which are code, and for each "not shown" gap, whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code (this session: why a command topic like `planner_enable` is resent rather than latched, and what "latched" means for a late joiner).
- Settled decisions stay settled (stack, D37-D41); don't reopen them without new evidence. The D14 values (0.5, 30 s) are this session's knobs, to change only with the user.
- Do not push without asking.
