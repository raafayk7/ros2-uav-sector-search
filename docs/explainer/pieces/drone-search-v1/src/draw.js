// =====================================================================
//  DRAW — drone search explainer. The pieces the scenes put together: the grid, the cells, the cards, the dots.
//  Every function takes its alpha, because the kit's strokes set globalAlpha themselves.
// =====================================================================
// ---- the grid: ShowCreation of 9 + 9 lines, lagged; p 0..1
function drawGrid(p, al = 1) {
  if (p <= 0 || al <= 0) return;
  for (let i = 0; i <= 8; i++) {
    const edge = i === 0 || i === 8, q = lagged(p, i, 9, 0.7), c = edge ? MC.white : MC.greyA, w = edge ? 3.5 : 2, a = al * (edge ? 0.95 : 0.55);
    segLine(GX0, GY0 + i * CELLPX, GX0 + 8 * CELLPX, GY0 + i * CELLPX, c, w, smoothM(q), a);
    segLine(GX0 + i * CELLPX, GY0, GX0 + i * CELLPX, GY0 + 8 * CELLPX, c, w, smoothM(q), a);
  }
}
// brightness of a chance: a gentle curve so 50% and 65% read as different, 3% as nearly dark, 93% as bright
const chanceA = (p) => 0.03 + 0.9 * Math.pow(p, 1.6);
// ---- a sector's fill and label. st = { p (0..1 or null = unknown), found }
function drawCell(c, r, st, al, label) {
  const x = cellX(c), y = cellY(r);
  ctx.save();
  if (st.found) { ctx.globalAlpha = 0.5 * al; ctx.fillStyle = Q_FOUND; ctx.fillRect(x + 1.5, y + 1.5, CELLPX - 3, CELLPX - 3); }
  else if (st.p != null) { ctx.globalAlpha = chanceA(st.p) * al; ctx.fillStyle = Q_BEL; ctx.fillRect(x + 1.5, y + 1.5, CELLPX - 3, CELLPX - 3); }
  ctx.restore();
  if (st.found) mText('0%', x + 50, y + 86, 32, { color: MC.white, alpha: al });
  else if (st.p == null) mText(label || '?', x + 50, y + 62, label ? 32 : 36, { color: MC.grey, alpha: 0.7 * al });
}
// ---- the cards on the right: a term, plain lines, a "linked to" line
function drawCard(cd) {
  if (TT < cd.t0 || TT > cd.t1 + 0.02) return;
  const a = clamp((cd.t1 - TT) / 0.5), ts = cd.ts || 62, termLines = Array.isArray(cd.term) ? cd.term : [cd.term];
  const p0 = runS(cd.t0 + 0.2, 0.9);
  termLines.forEach((tl, i) => writeText(tl, CARDX, 235 + i * (ts + 8), ts, lagged(p0, i, termLines.length, 0.4), { align: 'left', color: MC.white, alpha: a, bold: true }));
  let y = 235 + (termLines.length - 1) * (ts + 8);
  const tw = Math.max(...termLines.map((tl) => textW(tl, ts, { bold: true })));
  segLine(CARDX, y + 24, CARDX + tw, y + 24, MC.greyD, 2.5, smoothM(p0 * 1.3 - 0.3), a);
  if (cd.sub) { writeText(cd.sub, CARDX, y + 62, 32, runS(cd.t0 + 0.7, 0.8), { align: 'left', color: MC.gold, alpha: a, it: true, bold: true }); y += 40; }
  y += 78;
  cd.lines.forEach((l, i) => writeText(rich(l), CARDX, y + i * 52, 37, runS(cd.t0 + 1.0 + i * 0.6, 0.9), { align: 'left', alpha: a }));
  y += (cd.lines.length - 1) * 52;
  const t1 = cd.t0 + 1.0 + cd.lines.length * 0.6;
  if (cd.link) { y += 64; writeText(rich(cd.link, MC.grey, true), CARDX, y, 33, runS(t1 + 0.1, 0.9), { align: 'left', alpha: a }); }
  const notes = Array.isArray(cd.note) ? cd.note : cd.note ? [cd.note] : [];
  notes.forEach((nt, i) => { y += i ? 40 : 62; writeText(rich(nt, MC.greyC, true), CARDX, y, 31, runS(t1 + 0.9 + i * 0.5, 0.9), { align: 'left', alpha: a }); });
}
const CARDS = [
  { t0: 0.8, t1: S.arena, term: ['How a drone', 'search team works'], ts: 78, lines: ['A simulation of one {y|leader} and', 'three {b|followers} searching one area.'] },
  { t0: S.arena, t1: S.map, term: 'Arena and sector', lines: ['The area to search is 40 m square.', 'We cut it into 8 by 8 {w|sectors},', 'each one 5 m wide.'], link: '→ Everything below sits on these squares.' },
  { t0: S.map, t1: S.leader, term: 'Belief map', lines: ['For each sector, the {o|chance} a target', 'is there. A sector nobody has seen', 'yet is unknown: we count it as 50%.'], link: '→ The {y|leader} writes it. The {b|followers} read it.' },
  { t0: S.leader, t1: S.evidence, term: 'Leader, sweep, footprint', lines: ['The {y|leader} flies the whole arena in', '3 lanes, like mowing a lawn: the sweep.', 'Its camera sees a patch of ground,', 'the footprint: 25 m by 19 m.'], link: '→ Every sector inside it gets looked at.' },
  { t0: S.evidence, t1: S.detector, term: 'Evidence', lines: ['Each look nudges a sector\'s {o|chance}:', 'an empty look pushes it down, a target', 'seen pushes it up. One look can only', 'move it so far.'], link: '→ This is how the map learns.' },
  { t0: S.detector, t1: S.followers, term: 'Detector', lines: ['Turns a camera view into "target here".', 'Today it is a stand-in: the simulator', 'knows the {r|targets} and adds mistakes.'], link: '→ Later: YOLO, an object-recognition model.', note: 'Not built yet.' },
  { t0: S.followers, t1: S.confirm, term: 'Followers and the planner', lines: ['Each {b|follower} flies to the brightest', 'sector nobody else has claimed:', 'that rule is the planner. No boss.', 'They hold until the map fills in.'], link: '→ Layers: 10, 12 and 14 m up, so paths cross safely.' },
  { t0: S.confirm, t1: S.mission, term: 'Confirmed find', lines: ['A {b|follower} within 2 m of a sector\'s', 'centre must see the {r|target} in 3', 'frames in a row. Then the sector', 'drops to 0 and the others move on.'] },
  { t0: S.mission, t1: S.now, term: 'Mission', lines: ['The manager. It ends the search when', 'the sweep is done and nothing is above', '50% for 30 s, then reports the finds.'], link: '→ Then each finder returns over its find.' },
  { t0: S.now, t1: S.prior, term: 'Where it stands', lines: ['Today: simulation only, started with one', 'command. {g|3 of 3} targets found in every', 'cold run so far.'], },
  { t0: S.prior, t1: S.priorWhen, term: 'Language prior', sub: 'Planned. Not built yet.', lines: ['A prior is what you believe before you look.', 'An operator types a field report. A language', 'model reads it with a list of named places', 'and writes regions: where to look, how much.'], note: ['Its only output is the starting map.', 'It never steers a drone.'] },
  { t0: S.priorWhen, t1: S.novel, term: 'When it is used', sub: 'Planned. Not built yet.', lines: ['Once, at the start, before any drone moves:', 'the clue becomes the starting numbers.', 'Then the {y|leader} sweeps and the {b|followers} start', 'together. Evidence keeps correcting the map.'], note: ['The leader still sweeps everything:', 'a wrong clue can slow a find, not stop one.'] },
  { t0: S.novel, t1: S.question, term: 'What is new here', sub: 'As of a literature check, Nov 2026', lines: ['Language-derived priors already exist: for', 'example LMPath and arXiv 2611.16524. So the', 'claim is only a measurement: what a {r|wrong} clue', 'costs, in a team whose map learns from empty', 'looks and whose leader always sweeps.'], note: ['To be re-run before publishing.'] },
  { t0: S.close, t1: DURATION + 1, term: 'One shared map', lines: ['The {y|leader} looks.', 'The {b|followers} decide.', 'The mission manages.'], note: 'Not scheduled: image recognition, landing.' },
];
// ---- the step counter and the schematic tag
const STEPS = [['arena', 1], ['map', 2], ['leader', 3], ['evidence', 4], ['detector', 5], ['followers', 6], ['confirm', 7], ['mission', 8], ['now', 9], ['prior', 10], ['priorWhen', 11], ['novel', 12], ['question', 13], ['close', 14]];
function drawStepTag() {
  const k = STEPS.filter(([key]) => TT >= S[key] - 1e-6).pop(); if (!k) return;
  const t0 = S[k[0]], next = STEPS[STEPS.indexOf(k) + 1], t1 = next ? S[next[0]] : DURATION + 1;
  writeText('step ' + k[1] + ' of 14', CARDX, 130, 30, runS(t0 + 0.1, 0.7), { align: 'left', it: true, color: MC.grey, alpha: clamp((t1 - TT) / 0.3) });
}
function drawSchematicTag() {
  if (TT < S.leader || TT > S.question + 4) return;
  mText('schematic timing, not the real clock', 1840, 62, 24, { align: 'right', color: MC.greyC, it: true, alpha: 0.9 * clamp((TT - S.leader) / 1.0) });
}
// ---- the colour key (what each colour means), once colours start to matter
function drawKey() {
  if (TT < S.leader + 1.0 || TT > S.question) return;
  const a = clamp((TT - (S.leader + 1.0)) / 0.8) * clamp((S.question - TT) / 0.5), y = 1034;
  const items = [['dot', Q_LEAD, 'leader'], ['dot', Q_FOL, 'followers'], ['sq', Q_BEL, 'chance'], ['dot', Q_TGT, 'target'], ['sq', Q_FOUND, 'found']];
  let x = CARDX;
  items.forEach(([k, c, name]) => {
    if (k === 'dot') mDot(x + 8, y - 9, 8, c, a); else { ctx.save(); ctx.globalAlpha = 0.85 * a; ctx.fillStyle = c; ctx.fillRect(x, y - 17, 16, 16); ctx.restore(); }
    const w = mText(name, x + 26, y, 26, { align: 'left', color: MC.grey, alpha: a });
    x += 26 + w + 34;
  });
}
// a small label on a dark backing, so it reads over any cell colour
function tagBox(str, x, y, size, color, al, align = 'left') {
  if (al <= 0) return;
  const w = textW(str, size) + 16, x0 = align === 'left' ? x - 8 : align === 'right' ? x - w + 8 : x - w / 2;
  ctx.save(); ctx.globalAlpha = 0.72 * al; ctx.fillStyle = '#000'; ctx.fillRect(x0, y - size * 0.86, w, size * 1.22); ctx.restore();
  mText(str, x, y, size, { align, color, alpha: al });
}
// ---- the dots
function dotAt(xm, ym, c, al, r = 14) { trackDot(gx(xm), gy(ym), c, { al, r }); }
// the leader's footprint: a rectangle on the ground, 18.7 m across the lane and 24.9 m along it
function drawFootprint(xm, ym, al) {
  if (al <= 0) return;
  const x = gx(xm - FOOT[0]), y = gy(ym + FOOT[1]), w = FOOT[0] * 2 * PXM, h = FOOT[1] * 2 * PXM;
  ctx.save(); ctx.globalAlpha = 0.07 * al; ctx.fillStyle = Q_LEAD; ctx.fillRect(x, y, w, h);
  ctx.globalAlpha = 0.85 * al; ctx.strokeStyle = Q_LEAD; ctx.lineWidth = 3; ctx.setLineDash([14, 9]); ctx.strokeRect(x, y, w, h); ctx.restore();
}
// a cell outline (a claim, a highlight)
function cellOutline(c, r, col, w, al, dash) {
  ctx.save(); ctx.globalAlpha = al; ctx.strokeStyle = col; ctx.lineWidth = w; if (dash) ctx.setLineDash(dash);
  ctx.strokeRect(cellX(c) + 3, cellY(r) + 3, CELLPX - 6, CELLPX - 6); ctx.restore();
}
// a tiny map for the question scene: 8 x 8, orange by chance, one red target
function miniMap(x0, y0, cs, pf, al, tgt) {
  for (let r = 0; r < 8; r++) for (let c = 0; c < 8; c++) {
    const p = pf(c, r), x = x0 + c * cs, y = y0 + (7 - r) * cs;
    ctx.save(); ctx.globalAlpha = chanceA(p) * al; ctx.fillStyle = Q_BEL; ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2); ctx.restore();
  }
  for (let i = 0; i <= 8; i++) {
    const e = i === 0 || i === 8;
    segLine(x0, y0 + i * cs, x0 + 8 * cs, y0 + i * cs, MC.greyA, e ? 2.5 : 1.4, 1, al * (e ? 0.9 : 0.4));
    segLine(x0 + i * cs, y0, x0 + i * cs, y0 + 8 * cs, MC.greyA, e ? 2.5 : 1.4, 1, al * (e ? 0.9 : 0.4));
  }
  if (tgt) mDot(x0 + (tgt[0] + 0.5) * cs, y0 + (7 - tgt[1] + 0.5) * cs, cs * 0.26, Q_TGT, al);
}
