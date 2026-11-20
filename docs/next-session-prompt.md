# Next-session prompt: the literature check (session 12)

Paste the block below to open the next session. Replace this file after each session.

## Where the plan stands

Session 11 is done (2026-11-20). `mission_node` is built and wired: `swarm.launch.py leader:=true num_followers:=3` starts it and launches every planner disabled. Two cold missions from one command (detector seeds 1 and 2) ended DONE 31.3 and 31.4 s after the sweep's COMPLETE. Latched `/mission_report` listed all 3 finds with the right drones (iris_3 (6, 1), iris_2 (1, 2) and (4, 5)), every drone held afterwards, and there were no false confirmations. A `mission_timeout:=60` run ended TIMED_OUT with its report. RTF during the search was 0.49-0.52. D14 held: the worst false alarm after the sweep was 5 % (session 10: 23 %). **The Nov 24 milestone is met, four days early.** `python3 -m pytest src`: 207 passed.

The user then amended D41's clock row: the mission reads sim time from the followers' `/<robot>/pose` stamps instead of `goal_sector`. In run 3 the report's `sim_s` matched Gazebo's `/stats` clock to 0.08 s; before, it ran about 30 % high.

One question for the user is still open (planner-decisions, session 11 notes). It does not block session 12: **finds after the end.** A hold sent while cruising overshoots a few metres and comes back. After TIMED_OUT, all three targets were confirmed 3-4 s after the report had been written. This is harmless at DONE. The option is to republish the report when `/confirmed_finds` grows; the recommendation was to leave it.

What is left before Nov 30: the go/no-go (leader + 3 followers complete a mission in Gazebo from one command, video v1). The mission part is already shown once.

| Session | Work | Milestone | Model | Why that model | Time |
|---|---|---|---|---|---|
| 1-10 (done 2026-11-07 to 11-20) | Datum through 4 drones live and the mission core | Nov 10, 15, 20 | mixed | see git history | ~25 h |
| 11 (done 2026-11-20) | `mission_node`, launch wiring, two DONE runs and a TIMED_OUT live, sim time from pose stamps | Nov 24 (met) | Opus | End-to-end integration | ~2 h |
| **12 (this prompt)** | Literature check on the narrowed contribution claim, in `docs/literature-check.md`. No sim | before the LLM layer (Dec 10) and any README claim | **Opus** | Judging novelty against papers | ~1.5-2 h |
| 13 | Go/no-go dress run with the Gazebo GUI and video v1 (capture); two headless DONE runs already exist | Nov 30 | Sonnet (session 11 landed clean) | Mostly running and recording | ~2 h |

Session 12 needs no sim, so it can swap with 13 on a day the USB or the sim is unavailable.

---

Continuing the multi-agent UAV ROS 2 rebuild: session 12, the literature check. The search mission now runs end to end from one command (session 11, Nov 24 milestone met). Before the LLM layer (Dec 10) and before any claim in the README, check the narrowed contribution claim against published work.

## Read first

- `docs/roadmap.md`: **"Contribution claim"** (the chain to check, and the one paper already named, arXiv 2511.11967), then the eval section and the LLM-layer rows.
- `docs/assessment-ros2-migration.md` §10 (publication directions).
- `docs/planner-decisions.md` D40 (belief map with negative evidence) and D41 (mission), only to describe what the system actually does.
- Do not read `~/Documents/robotics-action-plan.md` into any committed file (CLAUDE.md).

## Task

1. **Search** for prior work on each link of the chain, and on the whole chain: natural-language incident report -> spatial prior; LLM as a semantic sensor or prior for search (not control); Bayesian search maps with negative evidence for multi-UAV SAR; decentralized information-seeking multi-UAV allocation; and evaluations comparing accurate vs misleading priors. Use `WebSearch` (extended mode for niche items) and read the papers, not only their abstracts, for the closest five or so.
2. **Write `docs/literature-check.md`**: per link, the closest papers (title, venue or arXiv id, year, one-line what they do, how ours differs), then a verdict on the whole chain: novel as stated, novel only in the eval, or already done. Then a suggested README sentence that the evidence supports. Quote at most one short phrase per paper; summarize the rest in your own words.
3. **Say what changes**: if the claim must narrow again, propose the new wording to the user; change the roadmap's "Contribution claim" only with their yes.

Record the outcome in the roadmap (the claim's status line) and rewrite this file for session 13.

## Environment

- No sim this session. The Docker stack is not needed; if a test run is wanted, `mount | grep docker-data` first (gotcha 38) and check `docker ps --filter name=WinBoat` (gotchas 42, 46).
- Commit on `main` after each working sub-task. Do not push without asking.

## Done means

`docs/literature-check.md` written with sources, a verdict on the narrowed claim, and a supported README sentence; roadmap status updated; this file rewritten for session 13. Say plainly which changes are plan-level (the claim's wording) and which are documentation, and for each "not shown" gap whether it was in scope and where it now lives.

## Working agreements

- The user has high-level ROS understanding, not deep. Explain any research term (for example "information-seeking control", "negative evidence") in one paragraph the first time.
- Settled decisions stay settled (stack, D37-D41). The open session-11 question above is the user's to answer; raise it at the start in one line, and do not act on it without a yes.
- Do not push without asking.
