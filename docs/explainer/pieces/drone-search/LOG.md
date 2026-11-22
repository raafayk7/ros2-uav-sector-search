# Log: drone-search

## Run 1 - 2026-11-22

- Time: about 3 h with the user's three check-ins (story, look, storyboard). Outputs: `renders/final.mp4` (1920x1080, 24 fps, 110 s, 8.0 MB), `renders/share.mp4` (5.5 MB), `renders/audio.wav`, stems. Export: 2640 frames in about 30 s, audio about 80 s.
- Check-ins: story approved as written; look approved, with one change (the pi hero removed); storyboard approved, all 13 panels thumbs up.
- Review numbers (last run):
  - cuts: 0 hard cuts, none to grid-check: PASS. Flat frames: none (the grid is drawn from frame 0). Loop: diff(last, first) 6.7, the end returns to the opening grid, changed (info only).
  - story arc: PASS 3/3. Mix median / p90 dBFS: ideas -18.0 / -15.1; silence -120 / -52.4; the find -11.9 / -10.6; where next -22.7 / -19.0.
  - text: PASS, 0 cut-off or overlapping strings in 453 sampled frames.
  - sound: loudest 100 ms window at 70.1 s (-7.6 dBFS), in the payoff act; 2.0 s at or below -40 dBFS before it; -14.5 LUFS integrated, true peak -1.0 dBFS.
  - dead beats: WARN, 3 stretches just over the 6 s limit: 38.0-44.75 s, 84.25-90.75 s, 92.75-98.75 s.
- What the review caught and what fixed it:
  - story arc FAIL (the opening act louder than the payoff act; goodbye not 6 dB down): removed energy from the pads and plucks of act 1 and the goodbye, raised the find's chord. Now PASS.
  - text FAIL x3: the "leader" tag over a "?" (moved into the free band between rows), two layer labels overlapping (staggered left, right, above), the main grid fading in while the four small maps faded out (moved the stage fade-in after the maps are gone), and the step-4 cell labels over "?" (the two followed sectors start at 50%, the number an unseen sector counts as).
  - flat first frames: the grid now starts partly drawn.
  - dead beats: added live numbers in the two evidence sectors, marching dashes on every dashed line, ripples around the finds, a done / not done checklist in step 9, and a target the clue missed that the evidence corrects in step 10. The warnings stayed at 6.0-6.8 s: the metric counts big pixel changes, and these additions are small ones.
- Per-shot PASS table (contact sheets and the phone sheet against grammar/FRAME.md and the math style's checklist):

| shot | what | result | note |
|---|---|---|---|
| 1 title | grid draws, title | PASS | |
| 2 arena | 40 m square, 8 x 8, one sector measured | PASS | the "5 m" label reads at full size |
| 3 belief map | "?" then "50%" | PASS | |
| 4 leader | sweep, footprint | PASS | the footprint is drawn at one orientation (18.7 m across the lane) for the whole sweep, as an idealisation |
| 5 evidence | graph with 5 points per curve, live labels in two sectors | PASS | warn: slow stretch 38-44.75 s |
| 6 detector | two rows of frames | PASS | rows show the averages (8 of 10, 1 of 20), labelled as such |
| 7 followers | brightest unclaimed sector, layers | PASS | |
| 8 confirm | ring, 6 frames, flash | PASS | silence before it |
| 9 mission | bar, report, returns | PASS | |
| 10 now | cards plus checklist | PASS | warn: slow stretch 84.25-90.75 s |
| 11 prior | river, 65 % band, corrected target | PASS | warn: slow stretch 92.75-98.75 s |
| 12 question | four small maps | PASS | |
| 13 close | the grid again, filled | PASS | end = start, changed |

- Weakest shot: shot 5 (evidence, 33-45 s): the longest stretch with only small changes, and the densest card. Fix: split it into two beats, one per curve, or cut the card to two lines.
- Not verified: the score by ear (it can only be measured here); how the text reads on a phone (at 360 px wide the 37 px card text is about 7 px: this is a 16:9 piece for a desktop or TV); the on-screen timing against how long a first-time viewer needs to read each card (about 3.5 words per second was the budget).
- Rule to propose for craft.md: for a fixed-stage explainer with text cards, a still stretch over 6 s is not fixed by small live numbers; use a second panel or split the beat.

## Run 2 - 2026-11-22 (the user asked for the language model, when it is used, what is new, and "planned, not built" subheadings)

- Changes: steps 10-14 rebuilt. Step 10 shows the language prior as a flow (report, language model with named places, regions, numbers) and the river region; step 11 shows when it is used (once at the start, the leader and followers start together, evidence corrects it); step 12 (new) says what is known and what this project adds, with the literature-check caveat; step 13 is the evaluation with its size and measures; step 14 the close. Gold subheadings "Planned. Not built yet." on steps 10 and 11, "As of a literature check, Nov 2026" on step 12, "Planned. Not built or run yet." on step 13. 146 s now.
- Review numbers: story arc PASS 3/3 (the find: median -11.2, p90 -9.8; the ideas act p90 -14.4; goodbye median -21.4); text PASS (0 of 599 sampled frames); flat frames none; loudest window 70.1 s at -7.5 dBFS after 2.0 s of silence; -14.7 LUFS, true peak -1.0 dBFS.
- Dead beats: WARN, 4 stretches: 38.0-44.75, 84.25-90.75, 107.5-114.75 and 130.5-139.0 s. The two new ones are the text of step 11 after its bars appear, and step 13's three lines after the maps.
- Weakest shot now: step 13 (the evaluation, 8.5 s of reading with only small changes). Fix: reveal the four conditions one per beat with a run counter, or split the three measures into their own beat.
- Not verified: how long a first-time viewer needs for steps 10-13 (they are the densest cards, about 3.5 words per second); the claim wording against a fresh literature search (the check is dated 2026-11-22).

## Run 3 - 2026-11-22 (timing fix and the comparison with the original project)

- Timing: the leader's sweep now ends at 74 s (it ended at 66 s before), so the three finds (70.1, 72.3, 73.6 s) land while it is still flying, as in the real runs; follower 2 departs at 60.2 s because its target is only revealed on the third lane; the sweep's background tone stops before the silence.
- Added in step 6: two bar pairs, "Original project: the followers wait for the leader to finish" (leader bar, then follower bar) and "This project: they start together" (two parallel bars).
