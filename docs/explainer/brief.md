# Explainer video brief: how the drone search team works

For the Animate skill (`/animate`), math style, 16:9, about 110 s, **no voice**: on-screen text only. Every number below comes from this repo's docs and the runs in `docs/planner-decisions.md`.

**Look.** Black stage, white lines, one accent colour for "found". Colour code, the same as the sim video: leader yellow, followers cyan, targets red, green and blue. Each concept gets a *definition card*: the term, one plain sentence, and a "linked to" line that shows the arrow to the next idea.

**Rules for the words.** Keep the real term (sector, belief map, footprint, prior) but always say what it does in the same card. No claim that anything is the first or new. Say "simulation" wherever it applies.

| # | Scene (about 8 s each) | What is drawn | On-screen text |
|---|---|---|---|
| 1 | Title | A grid appears line by line | "A drone team that searches. Leader + three followers." |
| 2 | **Arena and sector** | A 40 x 40 m square cut into 8 x 8 squares, one square lit | "Arena: the area to search, 40 x 40 m. Sector: one square of the grid, 5 m wide. 64 sectors. Linked to: everything below is a number per sector." |
| 3 | **Belief map** | Every square grey with "50" | "Belief map: for each sector, how likely a target is there, 0 to 100 %. Unknown starts at 50. Linked to: the leader writes it, the followers read it." |
| 4 | **Leader and sweep** | A yellow dot flies three lanes, like mowing a lawn | "Leader: flies the whole area in 3 lanes, 8 m up. Sweep: the lawnmower path that leaves no sector unseen." |
| 5 | **Camera footprint** | A 24.9 x 18.7 m rectangle slides under the leader, squares inside it change | "Footprint: the patch of ground the camera sees. Linked to: every sector inside it gets updated on the map." |
| 6 | **Evidence** | A small curve: a square's number steps down on empty looks, up on a hit | "Each look nudges the number. Empty look: down. Target seen: up. One look can only move it so far." |
| 7 | **Detector** | A square flashes "target" but sometimes misses or invents | "Detector: turns what the camera sees into 'target here'. Today a stand-in: the simulator knows where targets are, misses 1 in 5 and invents one in 20 frames. Later: YOLO, a model that recognises objects (a stretch goal)." |
| 8 | **Followers and the planner** | Three cyan dots fly to the three brightest squares | "Followers: each reads the map and flies to the highest sector nobody else has claimed. Planner: that rule. No boss. Linked to: they tell each other their goals." |
| 9 | **Altitude layers** | Side view: three heights, 10, 12, 14 m | "Layers: followers fly at different heights so paths can cross safely." |
| 10 | **Confirming a find** | A cyan dot hovers over a square; three ticks; the square goes to 0 | "Confirmed: a follower within 2 m of a sector's centre sees the target in 3 frames in a row. The sector drops to 0 so the others go elsewhere." |
| 11 | **The mission** | A clock bar with the sweep, a 30 s quiet period, then a report card | "Mission: the manager. Starts everyone. Ends when the sweep is done and nothing is left above 50 % for 30 s. Then reports every find and sends each finder back over its find." |
| 12 | **Same time, not in turn** | Leader and followers move together while the map fills in | "Followers do not wait for the sweep to end. At first the map is blank so they hold; as footprints fill it in they move. Linked to: this is why an early clue can help." |
| 13 | **Where it stands** | The real top-down footage clip, small | "Today: a simulation, started with one command. 3 of 3 targets found in every cold run. Not yet: real aircraft, real image recognition." |
| 14 | **Next: the clue** | A river drawn on the grid; squares near it brighten | "Prior: what you believe before you look. A text clue such as 'last seen near the river' becomes starting numbers on the map. Evidence still updates it, so a wrong clue gets corrected." |
| 15 | **The question** | Four small maps side by side | "What does a wrong clue cost? Compared: no clue, a right clue, a wrong clue, a wrong clue where empty looks are ignored." |
| 16 | **Later, maybe** | Three faded icons | "Real image recognition. Landing near a find. More drones. Not scheduled." |
| 17 | Close | The grid again | "One shared map. The leader looks, the followers decide, the mission manages." |

**Facts to keep exact.** 8 x 8 sectors at 5 m (40 x 40 m); leader at 8 m, followers at 10, 12, 14 m; footprint 24.9 x 18.7 m at 8 m; detector misses 20 % and false alarms 5 % per frame; confirm = within 2.0 m, 3 consecutive frames; end rule = sweep complete and every unconfirmed sector below 50 % for 30 s; 3 of 3 targets found in every cold run that reached the end so far (2026-11-20 to 11-22; one cold start hit a known startup flake and was relaunched).

**Do not show.** Anything that suggests real flights, a working language model, or YOLO today. The language prior and the comparison are plans (Dec 10 and Dec 20), and landing and YOLO are not scheduled.

**For Animate's fact check.** The skill asks for a web search per on-screen claim. These claims are about this project, so source them from this repo (`docs/planner-decisions.md`, `docs/roadmap.md`, `README.md`), not the web.
