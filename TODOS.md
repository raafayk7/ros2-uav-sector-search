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

## Completed

### Repeated frames over-count evidence (done 2026-11-18, D40)

**What:** Stop consecutive camera frames of one scene counting as independent evidence.

**Why:** One false detection held in view for about 5 frames reaches 97% and would pull followers. The D32 confirmation rule (N consecutive frames) has the same exposure.

**Context:** Detector noise is now set (session 7, D38: miss 0.20, false alarm 0.05 per frame, 2 Hz frames), so this can be decided. `src/uav_mapping/uav_mapping/leader_node.py:374` adds a full update per frame. Observed live on 2026-11-04: hit cell at 97 and swept cells at 3 within seconds. Decide when writing `belief_map_node`'s update step, using the ground-truth detector's noise settings. See R29 in `docs/planner-decisions.md`.

**Effort:** S
**Priority:** P2
**Depends on:** `belief_map_node` design (detector noise settings exist since 2026-11-17)

**Done:** `belief_map_node` caps each drone's look at a cell at +-0.85 log-odds, a new look after 1 s out of view or every 4 s (`uav_mapping/core/belief.py`, session 9 notes).
