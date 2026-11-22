# Next-session prompt: the go/no-go dress run (session 13)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 12 is done (2026-11-22, no sim): the literature check is in `docs/literature-check.md`. **Verdict: novel only in the eval.** No paper runs the whole chain, but every link has direct prior work, and the closest systems are from Oct-Nov 2026 (LMPath 2605.13782, Language as a Sensor 2606.08666, the LLM drone scheduler 2611.16524). Nobody tests a misleading language clue against uniform and accurate ones in multi-UAV search. 2611.12317 found that a strongly biased map did about half as well as no map, for a planner whose map never updates. The claim's wording also overstated the system ("distributed information-seeking": fusion has one owner, and the rule is best open sector).

**Two plan-level items await the user's yes** (both in the literature check, neither applied): (1) the reworded contribution claim, and (2) an optional fourth eval arm, misleading clue with negative evidence off (`BeliefParams.p_miss = 0.5`, 20 more 2D-sim runs). If the user answers at the start of session 13, apply the answer to the roadmap's "Contribution claim" and its eval section, then commit separately.

Session 11 met the Nov 24 milestone. Two cold headless missions ended DONE from one command (`swarm.launch.py leader:=true num_followers:=3`), and one ended TIMED_OUT. `python3 -m pytest src`: 207 passed.

What is left before Nov 30: the go/no-go (leader + 3 followers complete a mission in Gazebo from one command, video v1). The headless mission is shown twice. What is missing is a run you can *see*, and the recording.

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-11 (done 2026-11-07 to 11-20) | Datum through the mission end to end | Nov 10, 15, 20, 24 | mixed | see git history | ~27 h |
| 12 (done 2026-11-22) | Literature check, `docs/literature-check.md`; rewording proposed | before Dec 10 | Opus | Judging novelty | ~1.5 h |
| **13 (this prompt)** | GUI option for `swarm.launch.py`, a dress run with the Gazebo GUI, video v1, README and stack-overview status brought up to date | **Nov 30 go/no-go** | Sonnet | Mostly running and recording; one small launch change | ~2 h |
| 14 (estimate) | CI smoke test (1 leader + 1 follower, ground-truth detector, headless) | Dec 8 | Sonnet | Boilerplate workflow | ~2 h |
| 15-16 (estimate) | `mission_prior_node`, landmark file, replay provider, rasterization rule, first mission driven by a clue | Dec 10 | Opus for 15 (open questions: rasterization, multi-target clues), Sonnet for 16 | Design, then wiring | ~4 h |
| 17-18 (estimate) | 2D eval sim, 20 seeds x 3 (or 4) conditions, results table, video v2, re-run the literature search | Dec 20 | Opus | Eval design and honest reporting | ~4-5 h |

At about one session every one or two days, Nov 30 is likely met (one session left). Dec 10 is tight: three sessions in about eight days after the go.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 13, the go/no-go dress run. The mission runs end to end headless (session 11). Session 12's literature check is done (`docs/literature-check.md`). This session makes the run visible and records video v1 for the Nov 30 go/no-go.

## Read first

- `docs/roadmap.md`: milestones (the Nov 30 go/no-go criteria) and the "Contribution claim" status line.
- `docs/planner-decisions.md`, session 11 notes (D41, how a mission ends, the report).
- `docs/docker-notes.md` gotchas 31-33 (login shell, `/clock`), 42/46 (WinBoat), 54-60 (RTF from `/stats`, latched topics, `python3 -u`), 57 (Wayland `XAUTHORITY` after a reboot, which matters for the GUI).
- If the user has answered the two pending items from session 12, apply them first (see "Where the plan stands").

## Task

1. **Add a GUI option to `swarm.launch.py`.** Today Gazebo starts with `-s` (server only, line ~223). Add `gui:=false` (default unchanged). With `gui:=true`, drop `-s` so the Gazebo client opens. Rebuild (gotcha 32). Code change: keep it small and say so.
2. **Dress run.** `swarm.launch.py leader:=true num_followers:=3 gui:=true`, cold, from one command. Pass = mission DONE with all 3 finds on `/mission_report` (read with `--qos-durability transient_local`). Measure RTF from `/stats` time deltas, with and without the GUI. The GUI adds rendering, so expect RTF to drop below the headless 0.49-0.52. If it falls so far that a run takes more than about 10 minutes of wall time, note it and decide with the user (a headless run for the record plus a GUI run for the video is acceptable).
3. **Record video v1.** Use whatever screen recorder works on the host (Wayland: GNOME's built-in recorder or OBS; check what is installed before suggesting anything). Aim for about 90 s, sped up if the RTF is low, showing the leader sweep, followers moving to sectors, and the `/mission_report` at the end. Save it outside the repo unless the user wants it committed (it is a binary, so ask about size).
4. **Bring the status docs up to date.** `README.md` (intro line and Status still say the mission node is not built, stale since session 10) and `docs/stack-overview.md` ("Where the project is right now" names `mission_node` as next). Add the session-11 mission result and the Nov 30 outcome. Do **not** add any novelty claim to the README. If one is wanted, use only the "now" sentence from `docs/literature-check.md`.
5. **Go/no-go call.** Record the outcome in the roadmap milestone row. Go = leader + 3 followers complete a mission in Gazebo from one command (already met headless) and video v1 exists.

Record the outcome in the roadmap and rewrite this file for session 14.

## Environment

- Before any sim launch: `mount | grep docker-data` (gotcha 38) and `docker ps --filter name=WinBoat` (gotchas 42, 46). If WinBoat is up, tell the user at once.
- The GUI needs the container's X/Wayland access. After a reboot, `start` can fail on a renamed `XAUTHORITY` file. `up -d` recreates the container, and then check setuptools with `/usr/bin/python3` (gotchas 57-58).
- `docker compose restart ros2` between sim runs.
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

GUI option built; one cold DONE mission with the GUI (or a stated reason why not, plus a headless DONE run); video v1 saved, with where it lives stated; README and stack-overview current; the go/no-go outcome in the roadmap; this file rewritten for session 14. Say plainly which changes are code (the launch argument), which are plan-level (the go/no-go outcome, and the claim wording if the user answered), and which are documentation. For each "not shown" gap, say whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain new terms in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D41, including the session-11 amendments). The contribution claim's wording changes only with the user's yes.
- Do not push without asking.
