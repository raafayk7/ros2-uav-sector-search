# TODOS

## Belief map

### Empty frames must reach the map

**What:** Every detector publishes on every frame, including an empty array.

**Why:** The map lowers sectors only when a frame arrives. A quiet detector means sectors never drop, followers park on stale values, and nothing reports an error.

**Context:** Since D40 (session 9) this is permanent, not a stopgap: `belief_map_node` treats each `Detection2DArray` as one frame and computes that frame's footprint from the drone's pose, so there is no separate footprint topic to fall back on. `ground_truth_detector` complies (2026-11-17, checked live). Binds any future detector, YOLO included. See R30 and the session 9 notes in `docs/planner-decisions.md`.

**Effort:** S
**Priority:** P2
**Depends on:** `perception_node` (YOLO stretch)

## Eval

### Test the direct rule on realistic maps

**What:** Run the best-open-sector rule on unequal peaks and LLM-shaped region maps; try a distance discount only if results call for it.

**Why:** It was validated only on equal peaks, spiky or Gaussian. It will cross the arena for a 97% sector while ignoring a 90% sector next door, and language priors produce broad circles and capsules, where every cell in a region can tie.

**Context:** Probes are described in `docs/planner-decisions.md` (R12). Do this before reporting any eval result. Start from the rule in `uav_planning/core/planner.py`.

**Effort:** S
**Priority:** P2
**Depends on:** 2D eval sim

### One target near a sector edge can be confirmed twice

**What:** Decide whether a target within about 0.2 m of a sector edge may be listed as two finds, and count it in the eval.

**Why:** Pixel noise puts its detections in both sectors, so followers confirm each. The mission report then shows more finds than targets. 7 extra confirmations in 20 runs of the session 16 probe (40 m, uniform), all 0.05-0.18 m from a target.

**Context:** `docs/probes/prior_probe.py`, D32/D40 in `uav_mapping/core/belief.py`. Never seen in a Gazebo run. Eval scoring (a detection within 2 m of the true position) is unaffected; a find count would be.

**Effort:** S
**Priority:** P3
**Depends on:** 2D eval sim

## CI and image

### Look at the Gazebo window on the rebuilt image

**What:** Run `swarm.launch.py gui:=true arena_view:=true` once on `multi-agent-uavs:jazzy` (after a dev-container recreate) and check the window opens and renders.

**Why:** The 2026-11-23 rebuild only changed setuptools and was tested headless (colcon, 231 tests, the full smoke mission), so nothing GUI-side was exercised.

**Context:** Do it at the start of the next video or showcase session; the user watches the window, so ask before restarting a sim. The old image is `multi-agent-uavs:jazzy-prev` if it ever fails.

**Effort:** S
**Priority:** P3

### Exercise the smoke retry path and explain the slow cold start

**What:** (a) Force `ci/smoke_check.py` exit 3 (for example `STALL=1 ci/smoke_test.sh`) to watch the relaunch run end to end. (b) Find why the sweep starts about 65 s later after a fresh `docker run` than in the dev container (91 s against 26 s, 3 of 3 fresh runs).

**Why:** The relaunch for the silent `/clock` start (docker-notes 33) has never fired in 5 smoke runs, so it is untested shell; the late start is inside the 600 s limit but unexplained.

**Context:** `ci/smoke_test.sh`, docker-notes 66, planner-decisions session 15 notes. Do (a) the first time a smoke run needs a retry or fails on the flake; (b) only if it starts to matter (a timeout, or a Gazebo showcase run that is slow from a fresh container).

**Effort:** S
**Priority:** P3

## Completed

### Fix the setuptools mix in the Dockerfile (done 2026-11-23)

**What:** Make the image end with only apt's setuptools 68.1.2 and nothing in `/usr/local`.

**Why:** A container from the image had setuptools 78.1.0 files with a stale 84.2.1 record, and `colcon build --symlink-install` failed with "option --uninstall not recognized" (docker-notes 58, 64).

**Context:** Dockerfile's last pip step. The build cache (24 layers) made the rebuild take minutes.

**Effort:** S
**Priority:** P3

**Done:** the step uninstalls pip's copy until 68.1.2 imports and fails the build otherwise. Fresh container: setuptools 68.1.2, `colcon build`, 231 tests, `import torch, ultralytics` all fine; `ci/in_container.sh` lost its workaround. The old image is `multi-agent-uavs:jazzy-prev` (remove with `docker rmi` once the new one has proven itself). Existing containers keep the old state until recreated.

### `/verify` on the host (done 2026-11-23)

**What:** Make `gstack-checks` green: a pregate policy and a runner that goes through the container.

**Why:** The container check passed but the pregate per-file stage reported ENV (no `pytest` on the host) and `status` stayed NOT_READY.

**Context:** `.gstack/pregate.json` (runner = `docker compose exec` pytest on `{files}`; lanes for `.github`, `ci`, `docker`, `skills` and the swarm launch file) and `src/uav_planning/test/test_ci_files.py` (6 tests). Package config files (`gimbal_nadir.parm`, `package.xml`, ...) are not mapped, so touching one gives `PREGATE_NO_LANE`: map it then. The policy is read from `origin/main`, so a change to it only counts after it lands. `gstack-checks trust` was run once for the two container commands.

**Effort:** S
**Priority:** P3

**Done:** after the push, `gstack-checks run`: verify PASS, tests PASS; `status` READY_FOR_SHIP, not_covered none. The tests check took 0.1 s because the diff against `origin/main` held only docs; a mapped change runs the lane (6 tests, 2.8 s as a head policy).

### Register the self-hosted runner and run `smoke.yml` (done 2026-11-23)

**What:** Register a self-hosted runner labelled `uav-smoke` and run `smoke.yml` once.

**Why:** Session 15 ran every step by hand; the workflow file itself had not run on a runner.

**Context:** `.github/workflows/smoke.yml`, `ci/`. `smoke.yml` must never gain a `pull_request` trigger (a self-hosted runner on a public repo runs a fork's code). The runner lives in `~/actions-runner`, outside the repo (it holds credentials).

**Effort:** S
**Priority:** P2

**Done:** run 45134386843 passed on the user's runner: 225 tests, mission DONE, 3 finds. The runner only runs while the machine is on and `./run.sh` is up; `smoke.yml` is dispatch-only (`gh workflow run smoke.yml`), so a push never queues a job nobody can take.

### Repeated frames over-count evidence (done 2026-11-18, D40)

**What:** Stop consecutive camera frames of one scene counting as independent evidence.

**Why:** One false detection held in view for about 5 frames reaches 97% and would pull followers. The D32 confirmation rule (N consecutive frames) has the same exposure.

**Context:** Detector noise is now set (session 7, D38: miss 0.20, false alarm 0.05 per frame, 2 Hz frames), so this can be decided. `src/uav_mapping/uav_mapping/leader_node.py:374` adds a full update per frame. Observed live on 2026-11-04: hit cell at 97 and swept cells at 3 within seconds. Decide when writing `belief_map_node`'s update step, using the ground-truth detector's noise settings. See R29 in `docs/planner-decisions.md`.

**Effort:** S
**Priority:** P2
**Depends on:** `belief_map_node` design (detector noise settings exist since 2026-11-17)

**Done:** `belief_map_node` caps each drone's look at a cell at +-0.85 log-odds, a new look after 1 s out of view or every 4 s (`uav_mapping/core/belief.py`, session 9 notes).

## Sim and video

### Count DDS participants per drone before a larger swarm

**What:** Measure how many DDS participants one more follower adds (count unicast discovery sockets in `/proc/net/udp`, docker-notes 61), and note where the raised limit of 120 ends.

**Why:** A leader and 3 followers with `arena_view` hold about 30 against the RMW's default 32. A swarm run that passes the limit loses nodes at start without an obvious error; the scale-beyond-two-followers goal runs into it first.

**Context:** `docker-compose.yml` sets `CYCLONEDDS_URI` (119 is the most a domain holds). Not measured: the cost per added drone.

**Effort:** S
**Priority:** P2
**Depends on:** a 5 or 6 drone launch

### Return to find: the cases only unit tests cover

**What:** Fly the return step with a TIMED_OUT mission (it is skipped by design) and with more finds than followers.

**Why:** Session 13 flew two cases: three finders with one find each, and one drone that found two targets. The other assignments are tested in `core/mission.py` only.

**Context:** `assign_returns`, `mission_node._return_tick`, D41 amended in `docs/planner-decisions.md`.

**Effort:** S
**Priority:** P3

### Video view: halos, markers and projection

**What:** For a re-record: turn the default Gazebo grid off (`<grid>false</grid>` in the world's `<scene>`, untested), try a higher camera (the orthographic controller's zoom could not be set from a pose call), and record at 30 fps and 720p with the encoder checked. Halos and markers both stay (decided 2026-11-22).

**Why:** A perspective camera at 48 m shifts a drone at 8-14 m outward by a factor of 1.2-1.4, which reads as a drone off its sector.

**Context:** docker-notes gotcha 62 (e) and (g); `arena_view` in `swarm.launch.py`. The marker trails a leader at 5 m/s by up to 1 m at the node's 5 Hz.

**Effort:** S
**Priority:** P3
