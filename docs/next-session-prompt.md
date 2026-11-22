# Next-session prompt: record video v1 and make the go/no-go call (session 14)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 13 is done in part (2026-11-22): the **mission half of the Nov 30 go/no-go is met**, the video is not recorded. `swarm.launch.py leader:=true num_followers:=3` ran cold from one command to DONE headless (142 s wall, RTF about 0.59) and with the Gazebo window (203 s, RTF 0.27-0.34; with the new `arena_view` look 184 s, RTF 0.41), each time with all 3 targets found and nothing left open. Built this session, all committed: `gui:=true`, `arena_view:=true` (plain ground, 5 m grid plate, top-down view with origin bottom-left, yellow leader and cyan follower propellers, halos, no shadows), `ground_marker_node` (a marker on the ground under each drone), and return to find (D41 amended). `python3 -m pytest src`: 225 passed, with no sim up (or `ROS_DOMAIN_ID=77`). Two decisions by the user, written up: D41 amended (return to find) and D42 (followers start with the sweep, as the language prior needs; the thesis flew the leader first). The container was recreated with `CYCLONEDDS_URI` (docker-notes 61, the sim held about 30 of the RMW's 32 DDS participants).

Open for the user, none blocking: keep the airborne halos or use the ground markers alone; whether precision landing is scheduled (it is in no milestone; YOLO is a stretch goal); an optional thesis-timing eval arm (D42).

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-12 (done 2026-11-07 to 11-22) | Datum through the literature check | Nov 10, 15, 20, 24 | mixed | see git history | ~29 h |
| 13 (done in part, 2026-11-22) | GUI option, the video look, ground markers, return to find, D42, docs. Video not recorded | Nov 30 half met | Sonnet | Mostly running and looking | long: the user watched every change |
| **14 (this prompt)** | Record video v1 with OBS; the go/no-go call; settle halos and the camera | **Nov 30 go/no-go** | Sonnet | Running and recording | ~1.5 h |
| 15 (estimate) | CI smoke test (1 leader + 1 follower, ground-truth detector, headless) | Dec 8 | Sonnet | Boilerplate workflow | ~2 h |
| 16-17 (estimate) | `mission_prior_node`, landmark file, replay provider, rasterization rule, first mission driven by a clue | Dec 10 | Opus for 16 (open questions: rasterization, multi-target clues), Sonnet for 17 | Design, then wiring | ~4 h |
| 18-19 (estimate) | 2D eval sim, 20 seeds x 4 conditions, results table, video v2, re-run the literature search | Dec 20 | Opus | Eval design and honest reporting | ~4-5 h |

At about one session every one or two days, Nov 30 is likely met (one session left). Dec 10 is tight: three sessions in about eight days after the go.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 14, video v1 and the go/no-go. The mission runs end to end from one command, headless and with the Gazebo window (session 13). This session records it and decides.

## Read first

- `docs/planner-decisions.md`, session 13 notes (what `arena_view` does, D41 amended, D42, the measurements and the "not shown" table).
- `docs/roadmap.md`: the Nov 30 row (half met) and the open questions (landing).
- `docs/docker-notes.md` gotchas 31-33 (login shell, `/clock`), 38, 42/46 (drive, WinBoat), 57-58 (reboot, setuptools), 61 (DDS participants), 62 (the Gazebo window and `arena_view` facts).

## Task

1. **Preflight.** `mount | grep docker-data` and `docker ps --filter name=WinBoat` (tell the user at once if it is up); `docker compose -f docker/docker-compose.yml up -d` from a shell with the current `XAUTHORITY`; check setuptools with `/usr/bin/python3` (68.1.2) and `colcon build --symlink-install` after a recreate. The user closes Chrome and other heavy apps first: about 3 GiB is free with the sim and the window up, and OBS adds to it.
2. **OBS setup.** OBS Studio 30.0.2 is installed on GNOME Wayland with `xdg-desktop-portal-gnome`; screen or window capture goes through the portal. Check `ffmpeg` and a hardware encoder (the GPU is an AMD Vega 6) before suggesting x264, which takes CPU from the sim. Record about 20 s while a run is up and compare RTF (`docs/probes/rtf_windows.sh`) with and without OBS before the real take.
3. **Record video v1.** One cold run, `swarm.launch.py leader:=true num_followers:=3 detector_seed:=1 gui:=true arena_view:=true`, then `belief_live_check.py 500 3` to spawn the 3 targets (both in the container, in a login shell). About 90 s, sped up in post (RTF is about 0.4), showing the leader sweep, followers moving, finds, return to find and the `/mission_report` at the end (read it with `--qos-durability transient_local`). Say on the video that no language clue is in play yet (D42). Save it outside the repo (for example `~/Videos`); ask about size before committing it. The user watches the window and calls the look; do not restart a sim they are watching without asking.
4. **Settle with the user:** halos or ground markers only; a higher camera (about 80 m) or the perspective view as is (the orthographic controller's zoom could not be set, docker-notes 62).
5. **Go/no-go call.** Record the outcome in the roadmap's Nov 30 row. Go = leader + 3 followers complete a mission in Gazebo from one command (met) and video v1 exists. No-go means the Dec 14 re-check in the roadmap.
6. If time is left, ask the user about the landing question (schedule it or declare it out of scope).

Record the outcome in the roadmap and rewrite this file for session 15.

## Environment

- `docker compose restart ros2` between sim runs. A cold start can hit the silent `/clock` flake (gotcha 33): relaunch.
- With a sim up, the ROS-based tests need `ROS_DOMAIN_ID=77`.
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

OBS tried with its RTF cost measured; video v1 saved, with where it lives stated; the go/no-go outcome in the roadmap; halos and camera settled; this file rewritten for session 15. Say plainly which changes are code, which are plan-level (the go/no-go outcome, any landing decision) and which are documentation. For each "not shown" gap, say whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D42). The contribution claim changes only with the user's yes.
- Do not push without asking.
