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
| `follower_watch.py ROBOT SYSID [SECS]` | Pose, `planner_status` and `goal_sector` once a second; the evidence for the first flight and the disable hold (2026-11-08) | T3, T4 |
| `swarm_watch.py [N] [SECS]` | Status, goal sector and height for N followers once a second; summary of first all-ARRIVED, goal changes and shared goals (2026-11-12) | T5 |
| `peer_release.py [ROBOT]` | Kills one follower's planner, then its localization, and times the sector release in sim seconds (2026-11-12) | T5, D21, D25 |
| `cmd_gps_pose_probe.py` | One `cmd_gps_pose` flies a drone and `goal_lla` echoes it. Needs a 3-iris swarm, with `iris_2` in GUIDED and airborne | R2 |

Limits: pure Python, no flight dynamics, equal peaks only (see `TODOS.md`).
