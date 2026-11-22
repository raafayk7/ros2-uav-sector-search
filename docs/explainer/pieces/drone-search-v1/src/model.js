// =====================================================================
//  MODEL — drone search explainer. The stage geometry, the colour code, and the little world the scenes draw:
//  the belief numbers (from the project's real parameters), the leader's sweep, and the followers' flights.
//  Everything here is a pure function of time. Facts: belief.py BeliefParams (p_hit 0.7, look cap logit(0.7),
//  clamp 0.97, first observation p = 0.5), D39 (3 lanes, 8x8 at 5 m, footprint 24.9 x 18.7 m at 8 m),
//  D5/D26 (layers 10/12/14 m), D32/D40 (3 frames, 2 m). Timing is schematic.
// =====================================================================
// ---- the stage: the 8 x 8 grid, 20 px per metre, north up, origin bottom-left
const PXM = 20, GX0 = 150, GY0 = 150, CELLPX = 100;
const gx = (xm) => GX0 + xm * PXM;
const gy = (ym) => GY0 + (40 - ym) * PXM;
const cellX = (c) => gx(c * 5), cellY = (r) => gy((r + 1) * 5);          // top-left corner of a cell (px)
const cellMid = (c, r) => [c * 5 + 2.5, r * 5 + 2.5];                    // a cell's centre (m)
const CARDX = 1010;                                                        // left edge of the card column (px)

// ---- one meaning per colour, everywhere
const Q_LEAD = MC.yellow, Q_FOL = MC.blue, Q_BEL = MC.orange, Q_TGT = MC.red, Q_FOUND = MC.green;

// ---- the cast: three targets (cells) and the followers' plan
const TARGETS = [[1, 2], [4, 5], [6, 1]];
const isTarget = (c, r) => TARGETS.some((t) => t[0] === c && t[1] === r);
// follower i hovers at its spawn, flies to its target cell, confirms, drifts to another sector, and returns at DONE
const SPAWN = [[5, -5], [10, -5], [15, -5]];
const LAYER_M = [10, 12, 14];
const FPLAN = [
  { cell: [1, 2], depart: 56.5, arrive: 62.0, confirm: 70.1, leave: 71.5, rest: [17.5, 12.5], restAt: 74.5 },
  { cell: [6, 1], depart: 60.2, arrive: 65.7, confirm: 72.3, leave: 73.4, rest: [27.5, 2.5],  restAt: 76.0 },
  { cell: [4, 5], depart: 57.5, arrive: 65.5, confirm: 73.6, leave: 74.7, rest: [27.5, 22.5], restAt: 77.0 },
];
const DONE_T = 77.0, RETURN_T = 78.0, RETURN_END = [81.5, 81.8, 82.0];

// ---- numbers (belief.py): one look moves a cell by at most logit(0.7); belief stays in [0.03, 0.97]
const LOGIT = (p) => Math.log(p / (1 - p)), SIGM = (l) => 1 / (1 + Math.exp(-l));
const LOOK_L = LOGIT(0.7);
const pAfter = (k, up) => clamp(SIGM((up ? 1 : -1) * k * LOOK_L), 0.03, 0.97);
const pct = (p) => Math.round(p * 100);

// ---- vector keyframes, smooth between keys (absolute times)
function keysV(ks, t = TT) {
  if (t <= ks[0][0]) return ks[0][1];
  for (let i = 1; i < ks.length; i++) if (t < ks[i][0]) { const u = smoothM((t - ks[i - 1][0]) / (ks[i][0] - ks[i - 1][0])); return [lerp(ks[i - 1][1][0], ks[i][1][0], u), lerp(ks[i - 1][1][1], ks[i][1][1], u)]; }
  return ks[ks.length - 1][1];
}
function followerPos(i, t = TT) {
  const p = FPLAN[i], c = cellMid(...p.cell), s = SPAWN[i];
  return keysV([[0, s], [p.depart, s], [p.arrive, c], [p.leave, c], [p.restAt, p.rest], [RETURN_T, p.rest], [RETURN_END[i], c]], t);
}

// ---- the leader's sweep: 3 lanes, north-south, joined by short crossings (D39); a time-lapse of 52 s: it ends after the three finds, as in the real runs
const SW0 = 22.0, SW1 = 74.0;
const LP = [[8.33, 2.5], [8.33, 37.5], [20, 37.5], [20, 2.5], [31.67, 2.5], [31.67, 37.5]];
const LEG = []; let LTOT = 0;
for (let i = 1; i < LP.length; i++) { const len = dist(LP[i - 1], LP[i]); LEG.push({ a: LP[i - 1], b: LP[i], len, s0: LTOT }); LTOT += len; }
function leaderAt(t = TT) {
  const s = clamp((t - SW0) / (SW1 - SW0)) * LTOT;
  for (const g of LEG) if (s <= g.s0 + g.len + 1e-9) { const u = (s - g.s0) / g.len; return [lerp(g.a[0], g.b[0], u), lerp(g.a[1], g.b[1], u)]; }
  return LP[LP.length - 1];
}
const FOOT = [9.34, 12.46];   // half the camera footprint (m): 18.7 across the lane, 24.9 along it, at 8 m
// how long each sector has been in the leader's footprint, by frame: VT[f * 64 + c + 8 * r] (s)
const VT = new Float32Array((NFRAMES + 1) * 64);
for (let f = 1; f <= NFRAMES; f++) {
  const t = f / FPS, L = t >= SW0 ? leaderAt(t) : null;
  for (let r = 0; r < 8; r++) for (let c = 0; c < 8; c++) {
    const k = c + 8 * r, m = cellMid(c, r);
    const seen = L && Math.abs(m[0] - L[0]) <= FOOT[0] && Math.abs(m[1] - L[1]) <= FOOT[1];
    VT[f * 64 + k] = VT[(f - 1) * 64 + k] + (seen ? 1 / FPS : 0);
  }
}
const viewTime = (c, r, t = TT) => VT[clamp(Math.round(t * FPS), 0, NFRAMES) * 64 + c + 8 * r];
// the chance shown for a sector (schematic timing: one look per 2 s in view); null = never seen = unknown
function beliefAt(c, r, t = TT) {
  const f = FPLAN.find((p) => p.cell[0] === c && p.cell[1] === r);
  if (f && t >= f.confirm) return 0;                                      // a confirmed find reads 0 (D13)
  const v = viewTime(c, r, t); if (v <= 0) return null;
  return pAfter(1 + Math.floor(v / 2.0), isTarget(c, r));
}
const foundAt = (c, r, t = TT) => { const f = FPLAN.find((p) => p.cell[0] === c && p.cell[1] === r); return !!f && t >= f.confirm; };

// ---- the prior (planned): a river and the sectors near it start at 65%
const RIVER = [[0, 30], [10, 27], [18, 31], [28, 26], [40, 29]];
function nearRiver(c, r) {
  const m = cellMid(c, r); let best = 1e9;
  for (let i = 1; i < RIVER.length; i++) {
    const [a, b] = [RIVER[i - 1], RIVER[i]], dx = b[0] - a[0], dy = b[1] - a[1], u = clamp(((m[0] - a[0]) * dx + (m[1] - a[1]) * dy) / (dx * dx + dy * dy));
    best = Math.min(best, Math.hypot(m[0] - (a[0] + dx * u), m[1] - (a[1] + dy * u)));
  }
  return best;
}
const priorAt = (c, r) => (nearRiver(c, r) <= 6 ? 0.65 : 0.5);

// ---- rich text: "{y|leader}" -> a coloured segment (y yellow, b blue, o orange, r red, g green)
const RC = { y: Q_LEAD, b: Q_FOL, o: Q_BEL, r: Q_TGT, g: Q_FOUND, w: MC.white };
function rich(str, base = MC.greyA, it = false) {
  const segs = []; const re = /\{([ybgorw])\|([^}]*)\}/g; let last = 0, m;
  while ((m = re.exec(str))) { if (m.index > last) segs.push({ s: str.slice(last, m.index), c: base, it }); segs.push({ s: m[2], c: RC[m[1]], it }); last = re.lastIndex; }
  if (last < str.length) segs.push({ s: str.slice(last), c: base, it });
  return segs;
}
const segStr = (segs) => segs.map((s) => s.s).join('');
