# Literature check: the narrowed contribution claim

Session 12, 2026-11-22. No sim. This checks the roadmap's "Contribution claim" against published work before the LLM layer (Dec 10) and before anything goes into the README.

**Verdict in one line: novel only in the eval.** No paper found runs the whole chain. Every link in it is established, though, and three preprints from Oct-Nov 2026 each cover three or four of them. What nobody has measured is the cost of a *wrong* language-derived prior in a multi-UAV search whose map learns from empty views and whose leader always sweeps the whole area. The claim's current wording also says more than the system does (see "Wording problems"). A reworded claim is proposed below. The user accepted it and the fourth eval arm on 2026-11-22, and both are now in the roadmap.

## How the check was done

- About 15 web searches, most in extended mode. They covered each link of the chain, the whole chain, and the papers named earlier in planning.
- **Read in full** (method and experiments, through a fetch agent with fixed questions per paper): LMPath (2605.13782), the joint LLM and optimizer drone scheduler (2611.16524), multi-agent search from uncertain vision-language priors (2610.15038), Language as a Sensor (2606.08666), BGC-Nav (2610.14702), Bootstrapped LLM Semantics (2511.11967), terrain-aware Thompson sampling (2611.14685), the random-key UAV search planner (2611.12317), LLM-CRF (2511.04042), Cognitive Guardrails (2505.23576), Guardian (2603.08954).
- **Abstract or search summary only:** Search-TTA (CoRL 2025), COMBS (2611.15964), UAV-VLRR (2503.02465), the UAE University thesis (landing page only; its PDF returned 403), the classical Bayesian-search papers, and Bertuccelli & How (2005).
- **Not reachable:** "Distributed Event-Driven Bayesian Search for Multi-UAV Systems with Spatially Correlated Targets" (Sensors 2026, doi 10.3390/s26165189). MDPI refused automated fetches (403). It looks relevant to link 4 only, which is not where the claim rests.
- Four of the closest papers appeared in October and November 2026, so this area moves within weeks. **Re-run the searches before any README claim** (scheduled with the Dec 20 write-up).

## Terms used below

- **Spatial prior:** a map of how likely each cell is to hold a target *before* any drone has looked. Here the LLM would write it from the operator's report.
- **Semantic sensor / LLM as a prior:** treating the language model like a noisy sensor whose output feeds a classical estimator or planner, instead of letting it choose actions. The opposite pattern has the LLM write waypoints or code that flies the drone.
- **Negative evidence:** a camera view that sees *nothing* still carries information. The cells it covered become less likely to hold a target. Our `belief_map_node` does this (-0.20 log-odds per empty frame, capped per look, D40). Without it, a wrong prior never fades and drones keep returning to an empty peak.
- **Information-seeking control:** a controller that moves each robot to where it expects to *learn the most*, usually measured as an expected drop in map uncertainty (entropy, mutual information). It is different from going to the cell with the highest probability. Many SAR papers use the term loosely for either.

## What our system actually does (for comparison)

Built and flown (D39-D41): a leader flies a full lawnmower sweep. One `belief_map_node` fuses every drone's detections and empty views into a log-odds grid with a per-look cap. Each follower runs the same rule on its own: fly to the highest-value open sector on that map, where sectors peers already hold are blocked, with a stay margin (`core/planner.py`, `choose_goal`). A target is confirmed after 3 consecutive frames by a follower over its sector. A mission node starts and ends the search.

Not built when this was written (since 2026-11-24 the prior drives missions, D44-D50; since 2026-11-25 a real model has been called and the 2D eval sim runs a pilot, D51-D54; no result is reported yet): the LLM prior (`mission_prior_node`), the landmark file, the 2D eval sim, and every measurement in the eval.

## Link 1: natural-language incident report -> spatial prior

| Paper | What it does | How ours differs |
|---|---|---|
| **Language as a Sensor: Calibrated Spatial Belief** (Naveen, Liu, Carlone, Bobu, MIT; arXiv 2606.08666, v2 Oct 2026) | Free-text location statements ("I left my backpack on the table") become a calibrated mixture of 3D Gaussians over a scene graph. The mixture is fused with vision in a per-voxel log-odds belief. Single Spot robot, indoors, prerecorded trajectories | Strongest precedent for turning language into a *calibrated* spatial belief that is fused like a sensor. Theirs is indoor, single robot, a trained model and no planner. Ours is an outdoor SAR report, a zero-shot LLM writing geometric regions over a landmark file, and multiple UAVs |
| **LMPath** (Diller, Cladera, Taylor, Kumar, UPenn; arXiv 2605.13782, May 2026, ICRA 2026 aerial robotics workshop) | The user gives a target label ("car"). An LLM lists likely places ("parking lot, road"), SAM segments them on satellite tiles, and the masks become a heatmap prior. An ILP or TSP then plans one fixed path. One UAV, flown on hardware | The prior comes from an object label plus imagery, not an incident report. The path is planned once, with no belief update, no negative evidence and no misleading-prompt test. Its authors call it a pipeline for "language-mediated exploration priors" for UAV search |
| **Joint LLM and Optimization Agent for Drone Scheduling in Victim Search** (Zhang et al.; arXiv 2611.16524, Nov 2026) | Free-text clues (trailhead, last sighting, shared destination) plus terrain go to an LLM. Every cycle it writes a priority score for the top 50 cells. A bottleneck-matching optimizer at the ground station then assigns 10 drones. Moving victims, simulation only | **The closest whole system.** The LLM re-scores cells every cycle instead of writing one prior, so the LLM *is* the belief: there is no probability map, and empty views enter only as prompt text. Allocation is centralized. No wrong-clue condition, and no LLM run without clues |
| **Cognitive Guardrails** (Cleland-Huang et al., Notre Dame; arXiv 2505.23576, Jun 2025) | GPT-4o rates how relevant each found clue is. A discrete Bayesian model over five lost-person strategies (trail, waterway, shelter...) then sets where 7 small UAVs search, with a coverage-based down-weighting | The belief covers *strategies*, not grid cells. Evaluation is qualitative: one narrative mission, no failure rates. Ours is a spatial grid, and the misleading clue is a measured condition |
| Guardian (Castillo & Mukkamala, arXiv 2603.08954, CAC 2026) | LLMs extract facts from missing-child case documents. Separate Monte Carlo and Markov components build the search zones | The LLM never writes the map. No UAVs |
| UAE University MSc thesis (Alsaedi, Apr 2026, abstract only) | Free-text mission description -> structured mission parameters -> PX4 coverage plan. Evaluated with a 32-person usability study | Mission configuration, not a probability prior. Single UAV as far as the abstract says |

**Link verdict: done in pieces.** Language -> calibrated spatial belief exists (2606.08666). LLM -> aerial-search heatmap exists (LMPath). Text clues -> per-cell search priorities for a drone fleet exists (2611.16524). The exact form here (a free-text SAR report turned into circles and capsules over a landmark file, rasterized into a grid prior) was not found, but the step itself is not a contribution.

## Link 2: the LLM as a sensor or prior, kept out of control

| Paper | What it does | How ours differs |
|---|---|---|
| **Bootstrapped LLM Semantics for Context-Aware Path Planning** (Amani, Beheshti, Akhavian; arXiv 2511.11967, Nov 2025) | GPT-4o-mini rates per-class danger 16 times, and a Bayesian bootstrap gives a posterior. Its CVaR scales repulsive costs for an MHA* planner. One Unitree Go1 robot, navigation | Same "LLM as a noisy sensor feeding a classical planner" pattern. The authors' own phrase is that it "turns a large language model (LLM) into a stochastic semantic sensor". Their posterior is per object class, not spatial, the task is navigation, not search, and nothing updates from the robot's own sensing |
| **BGC-Nav** (Hao et al., CAS; arXiv 2610.14702, Oct 2026) | Two UAVs in SAR. A VLM scores the high UAV's camera frames against a "last seen" prompt into a belief memory, which only reorders the low UAV's candidate actions | The belief comes from VLM image scores averaged over time, not from a prior written from the report. It does report *guidance reliability* levels, and at low reliability it does worse than its own base policy on success rate |
| Training-free multi-agent search from uncertain vision-language priors (Pu et al.; arXiv 2610.15038, Oct 2026) | MLLMs propose target-location scores and graph edges, and a centralized MILP routes 1-5 agents in Matterport3D | Same role for the LLM, which only generates hypotheses. Indoor, centralized, and visited viewpoints simply drop to zero. No wrong-prior condition |
| 2611.16524 (above) | Its LLM-Direct ablation lets the LLM assign drones: it completed 2 of 5 seeds and gave illegal output about half the time | Direct evidence for our hard rule (the LLM writes only the prior) |
| LLM-CRF (Ji et al., NWPU; arXiv 2511.04042, Nov 2025) | Voice or sketch intent -> the LLM writes executable Python for 3 AirSim UAVs, which runs after operator confirmation. User study, 400 trials | The contrasting pattern: here the LLM's output does fly the drones |
| UAV-VLRR (arXiv 2503.02465, abstract only) | A VLM plus GPT-4o pick targets and obstacles, which become references and constraints for an NMPC. One UAV | The LLM feeds a classical controller with goals, not a prior |

**Link verdict: established pattern, not a contribution.** Keeping the LLM out of flight-critical control is now the norm in this literature (2511.11967, 2610.14702, 2610.15038, 2611.16524). The 2611.16524 ablation is a good citation for *why*. State it as a design rule, not as novelty.

## Link 3: Bayesian search maps with negative evidence for multi-UAV SAR

| Work | What it does |
|---|---|
| Bourgault, Furukawa, Durrant-Whyte (2003-2006, not re-read) | Recursive Bayesian search with coordinated UAVs, with probability of detection on a grid. The standard reference |
| Tisdale et al., ACC 2008; Wang & Hussein, ACC 2011; Khan, Yanmaz, Rinner, ICRA 2014 (search summaries) | Grid-based Bayesian search fused across UAVs. A zero reading is evidence of absence (binary Bayes filter) |
| **Terrain-aware Thompson sampling** (Gagnon Souleiman & Beltrame, Polytechnique Montréal; arXiv 2611.14685, DARS 2026) | 5-20 drones, decentralized. The prior mixes five lost-person SAR models (distance ring, heading, track offset, elevation, watershed). A Gaussian posterior lets a no-detection pull cells down, weighted by forest cover. Compared against uniform, a sweep and GUTS |
| 2611.16524's Clue-Prior Bayes baseline | A clue-built prior (built once, method unspecified, not by the LLM), then negative-observation updates, for 10 drones |

**Link verdict: textbook.** A log-odds grid with negative evidence is standard (Bourgault-era work). Our version adds a per-look cap (D40), an engineering choice for frame-rate independence, not a research contribution.

## Link 4: decentralized information-seeking multi-UAV allocation

| Work | What it does |
|---|---|
| 2611.14685 (above) | Each drone samples its own posterior copy (Thompson sampling) and scores trajectories with a neighbour-repulsion term |
| GUTS, and Decentralized Uncertainty-Aware Active Search (CMU; arXiv 2304.02075, 2410.08507; search summaries) | Decentralized active search with uncertainty-aware Thompson sampling |
| The thesis's DGA and BLL (our baselines) | Distributed game-theoretic coverage |

**Link verdict: established, and ours is simpler than these.** Our follower rule is a greedy pick of the best open sector with peer blocking. It never computes expected information gain. And the posterior has one owner (`belief_map_node`), so fusion is centralized even though each follower decides alone. Claim none of this as novel.

## Link 5: measured comparison of uniform vs accurate vs misleading priors

| Paper | What it measured |
|---|---|
| **Random-key UAV search planner** (Dantas, Shiguemori, Chaves; arXiv 2611.12317, Nov 2026) | One offline planner, 1-3 UAVs, no belief update after an empty view (their conclusion lists that as future work). Fraction of victims found: exact map 52 %, uniform 38 %, mildly biased 45 %, strongly biased 20 %. Their summary is "Noisy probability maps are absorbed; biased ones are not." **A strongly biased prior did about half as well as no prior.** No language, no run counts, and the bias construction is not described |
| BGC-Nav (above) | Splits results by how highly the belief ranked the true cell. At low reliability it falls below its own base policy. Reliability is measured after the fact, not a planted misleading prompt |
| LMPath (above) | Its only prior mismatch is incidental: satellite imagery from a different date than the sim world |
| Language as a Sensor (above) | Varies the ambiguity of the statement, not its truth |
| Bertuccelli & How, CDC-ECC 2005 (not read) | Robust UAV search under imprecise probability maps. The classical reference for planning against a wrong prior |
| 2611.16524, 2611.14685, 2610.15038, Cognitive Guardrails | No wrong-prior or wrong-clue condition |

**Link verdict: the open spot.** No paper found plants a deliberately misleading *language* clue and compares it against uniform and accurate clues in a multi-UAV search, let alone one that measures whether negative evidence and a full sweep limit the damage. 2611.12317 is the sharpest point of contact. Its result (a biased map can be worse than none) comes from a planner that never updates its map, which is exactly the case our design is meant to avoid. That gives the eval a clear question.

## Verdict on the whole chain

**Novel only in the eval.**

- **Not "already done":** no single paper runs report -> spatial prior -> negative-evidence map -> multi-UAV planner with the LLM out of control. 2611.16524 is closest (text clues, LLM, 10 drones, and Bayesian baselines with negative evidence). But its LLM *is* the belief, it re-scores cells every cycle, allocation is central, and it never tests a wrong clue.
- **Not "novel as stated":** every link has direct prior work, three of the closest systems are weeks old, and a pipeline built from known parts is not a contribution by itself. A reviewer would name LMPath, 2606.08666 and 2611.16524 for the pipeline and Bourgault-era work for the map.
- **What is left:** the measured comparison, and specifically the misleading arm. The interesting result is whether negative evidence plus a guaranteed full sweep stop a wrong clue from making the search worse than no clue at all, as 2611.12317 found for a planner with a fixed map. Either answer is publishable as a README result, if it is reported honestly.

## Wording problems in the current claim

The roadmap's chain says "Bayesian posterior with negative evidence -> distributed multi-UAV information-seeking controller". Measured against the code:

1. **"Distributed"** holds for the decision (each follower picks its own sector), but not for the map: one `belief_map_node` owns the posterior. "Decentralized sector choice on a shared map" is accurate.
2. **"Information-seeking"** overstates it. The rule goes to the highest-value open sector and never computes information gain (see the term above). "Map-greedy" or "best open sector" is accurate.
3. **"Bayesian posterior"** is fair but loose: it is a log-odds occupancy grid whose updates are capped per look (D40), an approximation to Bayes. "Log-odds belief map with negative evidence" is accurate.
4. **"The eval is the contribution"** is right, and should come first, ahead of the pipeline.

## Proposed claim (plan-level; accepted by the user 2026-11-22, now in the roadmap)

> **What a wrong language-derived search prior costs, measured.** An LLM's only output is a prior map written from an operator's field report. Every camera view without a detection lowers the map, a leader always completes a full sweep, and each follower independently flies to the best open sector on the shared map. On that system we compare uniform, accurate and misleading clues (time to first find, time to all found with censoring, coverage at timeout). The pipeline is assembled from known parts (cite LMPath, Language as a Sensor, arXiv 2611.16524, Bourgault-style Bayesian search, 2611.12317). The eval is the contribution.

**A separate plan-level suggestion (accepted 2026-11-22):** add a fourth arm, *misleading clue with negative evidence turned off* (the prior never fades). This is the condition 2611.12317 measured. With it, the eval can say how much of any robustness comes from negative evidence and how much from the sweep, instead of only showing that the misleading arm did well or badly. In the 2D sim it costs 20 more runs, and turning negative evidence off is one existing parameter (`BeliefParams.p_miss = 0.5`, so an empty look adds 0 log-odds). It does not change D37-D41.

## README sentence the evidence supports

**Now** (written before the LLM layer and the eval existed; both are built since 2026-11-25, the results are not run, so this is still the sentence to use):

> An LLM will turn an operator's field report into a prior map; it never flies the drones. Language-derived search priors already exist (for example LMPath, arXiv 2605.13782, and arXiv 2611.16524), so this project's aim is narrower: to measure what such a prior costs when the clue is wrong, in a multi-drone search whose map learns from empty camera views and whose leader always sweeps the whole area.

**After the Dec 20 eval**, only with the numbers in hand: "Across 20 scenarios, a misleading clue cost X s median time-to-first-find against a uniform prior (Y censored), while an accurate one saved Z s." If the misleading arm is worse than uniform, say so in those words.

## What changes, and what is not shown

**Plan-level (accepted by the user 2026-11-22, applied the same day):** the claim's wording (proposal above) and the fourth eval arm. The roadmap's "Contribution claim" and eval conditions now carry both, and the old wording is kept there for the record.

**Documentation (done this session):** this file, the roadmap's claim status line and literature-check milestone row, and the session-13 prompt.

**Not shown, and where each now lives:**

| Gap | In this session's scope? | Where it lives now |
|---|---|---|
| Whether negative evidence and the sweep bound a misleading clue's cost | No: needs the eval | Dec 20 results (2D eval sim) |
| The LLM prior itself (`mission_prior_node`, landmarks, replay) | No | Done: Dec 10 row met 2026-11-24, first real model calls 2026-11-25 |
| Sensors 2026 distributed Bayesian search paper, UAE thesis body (both 403) | Yes, attempted; hosts refused | The user can download them in a browser if wanted. They bear on links 1 and 4, which the claim no longer rests on |
| Bertuccelli & How 2005, Search-TTA, COMBS read only at abstract level | Partly: not among the closest five | Read Bertuccelli & How before writing the eval section (Dec 20 write-up) |
| Fresh search for papers newer than 2026-11-22 | No: time-dependent | Re-run before any README claim (Dec 20 write-up) |

## Sources

- LMPath: <https://arxiv.org/abs/2605.13782>
- Joint LLM and optimization agent for drone scheduling: <https://arxiv.org/abs/2611.16524>
- Training-free multi-agent search from uncertain VL priors: <https://arxiv.org/abs/2610.15038>
- Language as a Sensor: <https://arxiv.org/abs/2606.08666>
- BGC-Nav: <https://arxiv.org/abs/2610.14702>
- Bootstrapped LLM Semantics: <https://arxiv.org/abs/2511.11967>
- Terrain-aware Thompson sampling: <https://arxiv.org/abs/2611.14685>
- Random-key UAV search planner: <https://arxiv.org/abs/2611.12317>
- LLM-CRF: <https://arxiv.org/abs/2511.04042>
- Cognitive Guardrails: <https://arxiv.org/abs/2505.23576>
- Guardian: <https://arxiv.org/abs/2603.08954>
- UAE University thesis: <https://scholarworks.uaeu.ac.ae/all_theses/1479>
- UAV-VLRR: <https://arxiv.org/abs/2503.02465>
- Search-TTA: <https://arxiv.org/abs/2505.11350>
- COMBS: <https://arxiv.org/abs/2611.15964>
- GUTS: <https://arxiv.org/abs/2304.02075>; Decentralized uncertainty-aware active search: <https://arxiv.org/abs/2410.08507>
- Agent-based lost-person priors for multi-UAV SAR (no LLM): <https://arxiv.org/abs/2411.10148>
- Distributed event-driven Bayesian search (not read, 403): <https://doi.org/10.3390/s26165189>
