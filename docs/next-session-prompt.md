# Next-session prompt: secret scan, WIP README, going public (session 3 of 3 to Nov 10)

Paste the block below to open the next session. Replace this file after each session.

## Session plan to the Nov 10 goal

Nov 10 goal: planner library + `follower_planner_node`, first follower flying to a stub-map sector in SITL, secret scan clean, repo public as a WIP. Times are estimates.

| Session | Work | Tasks | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1 (done 2026-11-07) | Shared datum, ROS-free library, tests | T1, T2 | Sonnet | Rules were all pinned; pure functions | ~3 h |
| 2 (done 2026-11-08) | Node, arm helper, stub map, first flight | T3, T4 | Opus | Live SITL debugging; it found a shell PATH bug that looked like the old FDM silence | ~4 h |
| **3 (this prompt)** | Secret scan, WIP README, going public | below | **Sonnet** | Mechanical, with one outward-facing step that needs asking first | ~1-2 h |

After Nov 10: session 4 (**Opus**) is the swarm launch and 3 followers (T5), for the Nov 15 trigger. It is live multi-process SITL debugging, and it has to deal with the `/clock` lead in `docs/docker-notes.md` gotcha 33.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 3 of 3 to the Nov 10 goal. Sessions 1-2 built and flew T1-T4. One follower in headless SITL arms, climbs to 10 m, flies to the best stub-map sector and reports ARRIVED, and `planner_enable` false holds it. This session makes the repo safe and readable for going public, then asks before doing it.

## Read first

- `CLAUDE.md` (Docker gotchas, now including "always `bash -lc`").
- `docs/planner-decisions.md`: "Implementation notes (session 2)" has what flew and with which numbers. The README's status section should match it, no more.
- `.gitignore`: session 2 found that a bare `core` pattern had kept `uav_planning/core/` out of git. Patterns are anchored now (`/core`, `/core.*`); check that no other bare pattern hides source.

## Tasks

1. **Secret scan over full history:** `docker run --rm -v "$PWD":/repo zricethezav/gitleaks:latest detect --source /repo -v`. Confirm `docker/.env` is gitignored and was never committed (`git log --all -- docker/.env`). Rotate anything found and tell the user before any rewrite of history.
2. **Ignored files audit:** `git status --short --ignored`. Today that shows `0/ 1/ 2/ 3/`, `eeprom.bin`, `logs/`, `docker/.env` and colcon outputs. Confirm each is runtime output, and that `mav_N_1.parm` for N > 0 is ignored (docker-notes gotcha 30; that ignore was part of T5's scope, so it may still be missing).
3. **Fresh-clone check:** clone into the scratchpad and run `python3 -m pytest src` against the clone inside the container (copy it under the mounted repo's ignored dir, or mount it), so nothing needed lives only on this disk. Expect 72 passed.
4. **WIP README:** what it is (a leader maps, followers fly to the best open sectors), an architecture diagram placeholder, a one-command quickstart (container up, `exec ros2 bash -l`, sim, then `ros2 launch uav_planning follower.launch.py`), honest status (what flew, what has not: peers, leader sweep, perception, landing), and a link to the thesis credit. No career or application content.
5. **Ask the user before making the repo public.** Show the scan result and the README first.

## Environment

- Host is Ubuntu 24.04. Docker data is on a USB ext4 partition, so the USB stays plugged in. `docker compose -f docker/docker-compose.yml up -d`; use `stop`, never `down`.
- In the container, always `docker compose exec ros2 bash -lc '...'` and `source /usr/local/bin/entrypoint.sh` first; interactively, `exec ros2 bash -l`. Plain `bash -c` or `bash` drops MAVProxy from PATH and SITL hangs.
- Commit after each working sub-task on `main`. Do not push without asking.

## Done means

Scan clean (or findings rotated), README committed, fresh clone passes tests, and the user has said yes or no to going public. End by rewriting this file for session 4 (T5, Opus), with the gotcha 33 lead and the RTF cost in gotcha 34 in its brief. Say plainly which changes are plan-level and which are code.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain a new concept in one paragraph before code.
- All design decisions are settled, so don't reopen them without new evidence.
- Do not make the repo public without asking.
