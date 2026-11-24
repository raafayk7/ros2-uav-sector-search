# Probes from the 2026-11-04 planner review

Throwaway scripts behind the numbers in `docs/planner-decisions.md`. They are evidence, not planner code, and aren't maintained. Algorithm probes need only Python; the others run inside the container against a live headless sim, piped in with `python3 -`.

| Script | Shows | Used by |
|---|---|---|
| `algo_probe2.py` | Thesis rule never settles; stay rule fails on spiky maps; crowding; BLL on these maps | R10, R11, R17 |
| `algo_probe3.py` | Slope versus direct flight to the best open sector | R12 |
| `algo_probe5.py` | Conflict rules with simultaneous decisions and timed flights; variant C is the approved rule | R19, R20 |
| `algo_probe6.py` | Local steps versus direct flight on Gaussian-spread peaks | R12 |
| `mount_check.py` | Marker test: spawns ground markers, compares real camera pixels with nadir and measured mounts. Rerun for the camera task | R25 |
| `rtf.sh N` | Real-time factor for N iris with upstream's bridge config (no ROS `/clock`, see docker-notes 35). Its readiness check matched nothing in the 1- and 4-drone runs, but did match in a 3-drone run on 2026-11-12 (0.30-0.33) | R18 |
| `rtf_swarm.sh N [ARGS]` | RTF for N iris from `swarm.launch.py followers:=false`, sampled only once every SITL shows "JSON received" and `/ap/v1`..`/ap/vN` exist; also RAM, `top` and gimbal joints. `LAUNCH=` takes a variant from `rtf_variant.py` (2026-11-16) | T7, D23 |
| `rtf_variant.py [nomav] [nocam] [lean]` | Writes a patched copy of `swarm.launch.py` to `/tmp`: no MAVProxy, no camera sensor, or no `joint_states`/`imu` bridge. Nothing installed changes. Its 2 ms step probe became the launch default (`physics_step`, D37) (2026-11-16) | T7, D23, D37 |
| `follower_watch.py ROBOT SYSID [SECS]` | Pose, `planner_status` and `goal_sector` once a second; the evidence for the first flight and the disable hold (2026-11-08) | T3, T4 |
| `swarm_watch.py [N] [SECS]` | Status, goal sector and height for N followers once a second; summary of first all-ARRIVED, goal changes and shared goals (2026-11-12) | T5 |
| `peer_release.py [ROBOT]` | Kills one follower's planner, then its localization, and times the sector release in sim seconds (2026-11-12) | T5, D21, D25 |
| `detector_live_check.py [x,y ...]` | Prints `/heatmap` next to which cells the camera covers and which hold a target (give the target positions as `x,y` args); run inside the container with a hovering drone, `ground_truth_detector` and `leader_node` up (2026-11-17) | session 7 |
| `r29_probe.py` | Offline replay of the session-8 sweep with D38 noise: frames and looks per cell, then the map under per-frame, 1 Hz, weighted and look-cap rules, at 1, 2 and 4 frames/s; hover clearing times and D32 confirmation times and false rates (2026-11-18) | R29, D40 |
| `belief_live_check.py [SECS] [N]` | Spawns the session-8 targets, logs sweep status, follower status and goal, confirmations and known cells once a second, then the final map and RTF; run in the container after `swarm.launch.py leader:=true` (2026-11-18) | session 9, T8 |
| `rtf_windows.sh LABEL` | RTF from three 30 s `/stats` windows (sim time over real time), the reliable way (docker-notes 54) (2026-11-18) | session 9 |
| `prior_live_check.py [--followers a,b,c] [--timeout S]` | Listens only: prints the `/mission_prior` raised cells, mission state changes, each follower's goal sector (marked raised or not), confirmed finds and the report; exit 0 on DONE. Run in the container beside `swarm.launch.py leader:=true clue:=...` after `ci/spawn_targets.sh` (2026-11-24) | session 17, D49 |
| `prior_probe.py raster\|mission\|feasible` | Offline: rasterization rules on random circles; 2D point-mass missions over clue condition, alpha and outside level at 40 and 80 m (about 4.5 min); how often a wrong landmark exists. The 160 m rows came from calling `run(160, ...)` directly (2026-11-24) | D44, D45, D47 |
| `cmd_gps_pose_probe.py` | One `cmd_gps_pose` flies a drone and `goal_lla` echoes it. Needs a 3-iris swarm, with `iris_2` in GUIDED and airborne | R2 |

Limits: pure Python, no flight dynamics, equal peaks only (see `TODOS.md`).
