// =====================================================================
//  SCENES — drone search explainer. One stage; sceneAll() draws everything that exists at time TT.
//  Colour code: yellow leader, blue followers, orange chance, red target, green found, grey unknown.
// =====================================================================
const stageA = () => (TT < S.question - 0.2 ? 1 : TT < S.question + 0.6 ? 1 - (TT - (S.question - 0.2)) / 0.8 : TT < S.close - 0.1 ? 0 : clamp((TT - (S.close - 0.1)) / 0.8));
const entA = () => (TT < 21 ? 0 : TT < 21.6 ? (TT - 21) / 0.6 : TT < 90.4 ? 1 : TT < 91 ? 1 - (TT - 90.4) / 0.6 : TT < S.priorWhen + 3.0 ? 0 : TT < S.priorWhen + 3.6 ? (TT - S.priorWhen - 3.0) / 0.6 : TT < S.novel - 0.6 ? 1 : TT < S.novel - 0.2 ? 1 - (TT - S.novel + 0.6) / 0.4 : TT < S.close + 0.1 ? 0 : clamp((TT - (S.close + 0.1)) / 0.7));
const mainState = (c, r) => {
  const t = Math.min(TT, 83), p = beliefAt(c, r, t);
  // the two sectors followed in step 4 start at the 50% an unseen sector counts as, and carry their own label
  if (p == null && TT >= S.evidence && TT < S.detector && ((c === 3 && r === 3) || (c === 4 && r === 5))) return { p: 0.5, found: false };
  return { p, found: foundAt(c, r, t) };
};
const PRIOR_K = [[S.priorWhen + 6.0, 1], [S.priorWhen + 7.2, 2], [S.priorWhen + 8.4, 3], [S.priorWhen + 9.6, 4]];   // looks at the target the clue missed (cell 6,1)
const flatState = () => ({ p: 0.5, found: false });
const inWhen = () => TT >= S.priorWhen && TT < S.novel;
const priorState = (c, r) => {
  if (c === 6 && r === 1) { let k = 0; PRIOR_K.forEach(([tt, kk]) => { if (TT >= tt) k = kk; }); return { p: k ? pAfter(k, true) : 0.5, found: false }; }
  return { p: priorAt(c, r), found: false };
};
function drawCells(fn, al, label50) { if (al <= 0) return; for (let r = 0; r < 8; r++) for (let c = 0; c < 8; c++) { const st = fn(c, r); drawCell(c, r, st, al, label50 && st.p == null ? '50%' : undefined); } }
const px = (xm, ym) => [gx(xm), gy(ym)];

function sceneAll() {
  fillAll(MC.bg);
  ctx.lineDashOffset = -TT * 26;   // marching dashes: every dashed line moves
  const t = TT, sa = stageA(), ea = entA();

  // ---- the stage: the grid, then the sectors' numbers
  drawGrid(0.12 + 0.88 * runS(0.3, 3.0), sa);
  if (t >= S.map && sa > 0) {
    const al = clamp((t - S.map) / 0.8) * sa;
    const T1 = S.prior + 1.2, T2 = S.prior + 8.0, T3 = T2 + 1.4;
    if (t < S.prior) drawCells(mainState, al, t >= 15.4 && t < 20.2);
    else if (t < T1) { const u = smoothM((t - S.prior) / 1.2); drawCells(mainState, al * (1 - u)); drawCells(flatState, al * u); }
    else if (t < T2) drawCells(flatState, al);
    else if (t < T3) { const u = smoothM((t - T2) / 1.4); drawCells(flatState, al * (1 - u)); drawCells(priorState, al * u); }
    else if (t < S.question) drawCells(priorState, al);
    else drawCells(mainState, al);
  }

  // ---- step 1: the size of the arena and of one sector
  if (t >= S.arena && t < S.map) {
    const a = clamp((S.map - t) / 0.5), p = runS(S.arena + 0.5, 1.0), lp = runS(S.arena + 1.3, 0.6);
    mBrace(GX0, GY0 - 14, GX0 + 8 * CELLPX, GY0 - 14, { side: 1, p, al: a, label: [mn('40 m', MC.white)], size: 36, lp });
    mBrace(GX0 - 14, GY0 + 8 * CELLPX, GX0 - 14, GY0, { side: 1, p, al: a });
    writeText('40 m', GX0 - 64, GY0 + 4 * CELLPX + 12, 36, lp, { align: 'right', color: MC.white, alpha: a });
    // a wave through the sectors, then one is held lit and measured
    for (let r = 0; r < 8; r++) for (let c = 0; c < 8; c++) {
      const w = thereBack(clamp((t - (S.arena + 3.2 + (c + r) * 0.13)) / 0.6)); if (w <= 0) continue;
      ctx.save(); ctx.globalAlpha = 0.2 * w * a; ctx.fillStyle = MC.white; ctx.fillRect(cellX(c) + 1.5, cellY(r) + 1.5, CELLPX - 3, CELLPX - 3); ctx.restore();
    }
    const lit = runS(S.arena + 5.4, 0.5);
    if (lit > 0) {
      ctx.save(); ctx.globalAlpha = 0.28 * lit * a; ctx.fillStyle = MC.white; ctx.fillRect(cellX(6) + 1.5, cellY(0) + 1.5, CELLPX - 3, CELLPX - 3); ctx.restore();
      cellOutline(6, 0, MC.white, 4, lit * a);
      mBrace(cellX(6) + CELLPX, GY0 + 8 * CELLPX + 14, cellX(6), GY0 + 8 * CELLPX + 14, { side: 1, p: runS(S.arena + 5.7, 0.7), al: a, label: [mn('5 m', MC.white)], size: 34, lp: runS(S.arena + 6.2, 0.5) });
    }
    mText('64 sectors', GX0 + 8 * CELLPX, GY0 + 8 * CELLPX + 52, 32, { align: 'right', color: MC.greyA, alpha: runS(S.arena + 6.6, 0.6) * a });
  }

  // ---- the leader: its path so far, its footprint, the dot
  if (ea > 0) {
    const lp = inWhen() ? 0 : clamp((t - SW0) / (SW1 - SW0));
    if (lp > 0) showCreation(LP.map(([x, y]) => px(x, y)), lp, { c: Q_LEAD, w: 3, al: 0.3 * ea });
    const L = inWhen() ? LP[0] : leaderAt(t);
    drawFootprint(L[0], L[1], ea * clamp((t - 21.4) / 0.6) * clamp((SW1 + 1.5 - t) / 1.0));
    dotAt(L[0], L[1], Q_LEAD, ea, 15);
    if (t < S.evidence) tagBox('leader', gx(L[0]) + 24, GY0 + CELLPX * clamp(Math.floor((gy(L[1]) - GY0) / CELLPX), 0, 7) + 95, 28, Q_LEAD, ea * clamp((S.evidence - t) / 0.5));
  }
  // ---- the followers: hover, then each picks a sector nobody has claimed, confirms, drifts, returns
  if (ea > 0) {
    for (let i = 0; i < 3; i++) {
      const p = FPLAN[i], F = inWhen() ? SPAWN[i] : followerPos(i, t);
      if (t >= p.depart && t < p.confirm + 0.3) {
        const aClaim = ea * clamp((t - p.depart) / 0.5) * clamp((p.confirm + 0.3 - t) / 0.3);
        cellOutline(p.cell[0], p.cell[1], Q_FOL, 4, 0.9 * aClaim, [12, 8]);
        if (t < p.arrive) { const c = cellMid(...p.cell); ctx.save(); ctx.globalAlpha = 0.3 * aClaim; ctx.strokeStyle = Q_FOL; ctx.lineWidth = 2.5; ctx.setLineDash([8, 10]); ctx.beginPath(); ctx.moveTo(...px(...F)); ctx.lineTo(...px(...c)); ctx.stroke(); ctx.restore(); }
      }
      if (t >= RETURN_T && t < RETURN_END[i] + 0.3) {
        const c = cellMid(...p.cell), a2 = clamp((t - RETURN_T) / 0.4) * clamp((RETURN_END[i] + 0.3 - t) / 0.3);
        mArrow(...px(...p.rest), ...px(...c), Q_FOL, 3, 1, 0.35 * a2 * ea);
      }
      dotAt(F[0], F[1], Q_FOL, ea, 13);
      if (t >= S.followers && t < S.confirm) { const o = [[62, -40, 'left'], [-62, -40, 'right'], [0, -74, 'center']][i]; tagBox(LAYER_M[i] + ' m', gx(F[0]) + o[0], gy(F[1]) + o[1], 26, Q_FOL, ea * clamp((t - S.followers) / 0.6) * clamp((S.confirm - t) / 0.5), o[2]); }
    }
    if (t >= 21.6 && t < S.evidence) mText('followers: holding', gx(10), 1010, 28, { color: Q_FOL, alpha: ea * clamp((S.evidence - t) / 0.5) });
  }

  // ---- step 4: evidence for one empty sector and one with a target
  if (t >= S.evidence && t < S.detector) {
    const a = clamp((S.detector - t) / 0.5);
    cellOutline(3, 3, Q_BEL, 4, 0.9 * a * clamp((t - S.evidence) / 0.5), [10, 8]);
    cellOutline(4, 5, Q_BEL, 5, 0.95 * a * clamp((t - S.evidence) / 0.5));
    const ap = runS(S.evidence + 1.0, 0.9), AX = 1090, AY = 925, UX = 120, UY = 270, A = axes2({ at: [AX, AY], ux: UX, uy: UY, x: [0, 5], y: [0, 1], ext: 0.12, p: ap, step: 1, tick: 7, al: a, tips: false });
    const ta = a * clamp(ap * 2 - 1);
    const kk = Math.max(0, Math.min(5, Math.floor((t - (S.evidence + 2.2)) / 1.9) + 1)), kh = Math.min(kk, 4);
    tagBox(pct(kk ? pAfter(kk, false) : 0.5) + '%', cellX(3) + 50, cellY(3) + 62, 34, MC.white, a * clamp((t - S.evidence) / 0.5), 'center');
    tagBox(pct(kh ? pAfter(kh, true) : 0.5) + '%', cellX(4) + 50, cellY(5) + 62, 34, MC.white, a * clamp((t - S.evidence) / 0.5), 'center');
    for (let k = 0; k <= 5; k++) mText(String(k), A.to(k, 0)[0], AY + 38, 28, { color: MC.grey, alpha: ta });
    mText('looks', A.to(2.5, 0)[0], AY + 76, 28, { color: MC.grey, alpha: ta, it: true });
    [[0, '0%'], [0.5, '50%'], [1, '100%']].forEach(([v, s]) => mText(s, AX - 20, A.to(0, v)[1] + 10, 28, { align: 'right', color: MC.grey, alpha: ta }));
    segLine(...A.to(0, 0.5), ...A.to(5, 0.5), MC.greyD, 2, 1, 0.8 * ta);
    const T0 = S.evidence + 2.2, STEP = 1.9;
    [[false, [0, 1, 2, 3, 4, 5], [12, 8], 'empty sector'], [true, [0, 1, 2, 3, 4], null, 'sector with a target']].forEach(([up, ks, dash, name]) => {
      const pts = ks.map((k) => [k, k === 0 ? 0.5 : pAfter(k, up)]), P = pts.map(([k, p]) => A.to(k, p)), n = ks.length - 1;
      showCreation(P, clamp((t - T0) / (n * STEP)), { c: Q_BEL, w: 5, dash, al: a });
      pts.forEach(([k, p], i) => {
        const u = clamp((t - (T0 + (i - 1) * STEP)) / 0.4); if (i > 0 && u <= 0) return;
        mDot(P[i][0], P[i][1], 8, Q_BEL, a * (i === 0 ? 1 : u));
        if (i > 0) mText(pct(p) + '%', P[i][0] + (up ? -12 : 12), P[i][1] + (up ? 36 : -16), 28, { align: up ? 'right' : 'left', color: MC.white, alpha: a * u });
      });
      mText(name, up ? A.to(0.15, 1)[0] : A.to(5, 0.05)[0] + 6, up ? A.to(0, 1)[1] - 18 : A.to(5, 0.05)[1] - 70, 27, { align: 'left', color: Q_BEL, alpha: a * clamp((t - T0 - 1.0) / 0.8), it: true });
    });
  }

  // ---- step 5: what a detector does with a sector: frames, and what it says
  if (t >= S.detector + 2.2 && t < S.followers) {
    const a = clamp((S.followers - t) / 0.5), X0 = CARDX;
    const row = (y, n, size, gap, t0, label, state) => {
      mText(label, X0, y - size / 2 - 22, 30, { align: 'left', color: MC.greyA, alpha: a * clamp((t - t0) / 0.5), it: true });
      for (let i = 0; i < n; i++) {
        const u = clamp((t - (t0 + 0.3 + i * 0.14)) / 0.25); if (u <= 0) continue;
        const st = state(i), x = X0 + i * (size + gap), yy = y - size / 2;
        ctx.save(); ctx.globalAlpha = a * u;
        if (st === 'yes' || st === 'invented') { ctx.fillStyle = MC.white; ctx.fillRect(x, yy, size, size); }
        else { ctx.strokeStyle = st === 'missed' ? Q_TGT : MC.greyC; ctx.lineWidth = 3; ctx.strokeRect(x + 1.5, yy + 1.5, size - 3, size - 3); }
        ctx.restore();
      }
    };
    row(745, 10, 48, 12, S.detector + 2.2, 'A sector with a target: 10 frames', (i) => (i === 2 || i === 7 ? 'missed' : 'yes'));
    mText('saw it in 8, missed it in 2', X0 + 10 * 60 + 14, 757, 30, { align: 'left', color: MC.white, alpha: a * clamp((t - (S.detector + 4.6)) / 0.6) });
    row(885, 20, 28, 8, S.detector + 5.6, 'A sector with none: 20 frames', (i) => (i === 12 ? 'invented' : 'nothing'));
    mText('invented one in 1', X0 + 20 * 36 + 14, 892, 30, { align: 'left', color: MC.white, alpha: a * clamp((t - (S.detector + 9.0)) / 0.6) });
  }

  // ---- step 6: the original project ran the two jobs one after the other; this one runs them together
  if (t >= S.followers + 1.5 && t < S.confirm) {
    const a = clamp((S.confirm - t) / 0.5), X0 = CARDX;
    const u1 = runS(S.followers + 1.5, 0.9), u2 = runS(S.followers + 3.8, 0.9);
    mText('Original project: the followers wait for the leader to finish', X0, 706, 27, { align: 'left', color: MC.greyA, alpha: a * clamp((t - (S.followers + 1.6)) / 0.5), it: true });
    segLine(X0, 742, X0 + 400, 742, Q_LEAD, 22, u1, 0.9 * a);
    segLine(X0 + 410, 742, X0 + 810, 742, Q_FOL, 22, clamp((t - (S.followers + 2.3)) / 0.9), 0.9 * a);
    mText('This project: they start together', X0, 836, 27, { align: 'left', color: MC.white, alpha: a * clamp((t - (S.followers + 3.9)) / 0.5), it: true });
    segLine(X0, 872, X0 + 810, 872, Q_LEAD, 22, u2, 0.9 * a);
    segLine(X0, 906, X0 + 810, 906, Q_FOL, 22, u2, 0.9 * a);
  }

  // ---- step 7: the confirmation, frame by frame
  if (t >= S.confirm && t < S.mission) {
    const a = clamp((S.mission - t) / 0.5), c1 = cellMid(...FPLAN[0].cell), [rx, ry] = px(...c1);
    const ra = a * clamp((t - (S.confirm + 0.4)) / 0.5);
    ctx.save(); ctx.globalAlpha = 0.8 * ra; ctx.strokeStyle = Q_FOL; ctx.lineWidth = 3; ctx.setLineDash([10, 8]); ctx.beginPath(); ctx.arc(rx, ry, 2 * PXM, 0, TAU); ctx.stroke(); ctx.restore();
    tagBox('2 m', rx + 2 * PXM + 14, ry - 2 * PXM + 4, 26, Q_FOL, ra);
    const TK = [[67.9, 1], [68.3, 1], [68.7, 0], [69.1, 1], [69.5, 1], [69.9, 1]];
    const X0 = CARDX, Y0 = 790;
    mText('The follower\'s camera, frame by frame:', X0, Y0 - 52, 30, { align: 'left', color: MC.greyA, alpha: a * clamp((t - 67.6) / 0.5), it: true });
    let streak = 0;
    TK.forEach(([tt, ok], i) => {
      const u = clamp((t - tt) / 0.2); if (u <= 0) return;
      streak = ok ? streak + 1 : 0;
      const x = X0 + i * 76;
      ctx.save(); ctx.globalAlpha = a * u;
      if (ok) { ctx.fillStyle = MC.white; ctx.fillRect(x, Y0 - 26, 56, 56); } else { ctx.strokeStyle = Q_TGT; ctx.lineWidth = 3; ctx.strokeRect(x + 1.5, Y0 - 24.5, 53, 53); }
      ctx.restore();
    });
    const done = t >= FPLAN[0].confirm;
    writeText(done ? [{ s: '3 in a row: ', c: MC.white }, { s: 'confirmed', c: Q_FOUND }] : [{ s: 'in a row: ' + streak, c: MC.white }], X0, Y0 + 88, 40, runS(67.8, 0.5), { align: 'left', alpha: a });
    FPLAN.forEach((p) => flashAt(...px(...cellMid(...p.cell)), (t - p.confirm) / 0.5, Q_FOUND, 46, 5));
  }

  // ---- step 8: the mission's end rule, then its report
  if (t >= S.mission && t < S.now) {
    const a = clamp((S.now - t) / 0.5), X0 = CARDX, Y0 = 700;
    const p1 = runS(S.mission + 0.6, 0.9), p2 = clamp((t - 75.9) / 1.1);
    mText('leader\'s sweep: done', X0, Y0 - 24, 28, { align: 'left', color: Q_LEAD, alpha: a * p1 });
    mText('nothing above 50% for 30 s', X0 + 800, Y0 - 24, 28, { align: 'right', color: MC.grey, alpha: a * clamp((t - 75.9) / 0.6) });
    segLine(X0, Y0 + 14, X0 + 400, Y0 + 14, Q_LEAD, 22, p1, 0.85 * a);
    segLine(X0 + 408, Y0 + 14, X0 + 800, Y0 + 14, MC.greyC, 22, p2, 0.9 * a);
    const r = runS(DONE_T + 0.2, 0.8);
    if (r > 0) {
      ctx.save(); ctx.globalAlpha = 0.75 * a * smoothM(r * 2); ctx.fillStyle = '#000'; ctx.fillRect(X0 - 20, Y0 + 64, 830, 200); ctx.restore();
      writeText([{ s: 'Mission report: ', c: MC.white }, { s: 'DONE', c: Q_FOUND }], X0, Y0 + 122, 46, r, { align: 'left', alpha: a, bold: true });
      writeText(rich('{g|3 of 3} targets found; nothing left open'), X0, Y0 + 176, 34, runS(DONE_T + 0.7, 0.8), { align: 'left', alpha: a });
      writeText('each find lists the drone that made it', X0, Y0 + 224, 30, runS(DONE_T + 1.1, 0.8), { align: 'left', alpha: a, color: MC.grey, it: true });
    }
  }

  // ---- step 9: what is done in simulation, and what is not
  if (t >= S.now + 1.6 && t < S.prior) {
    const a = clamp((S.prior - t) / 0.5), X0 = CARDX;
    const rows = [[1, 'leader sweep'], [1, 'shared belief map'], [1, 'planner for the followers'], [1, 'confirming a find'], [1, 'mission start, end and report'], [0, 'real aircraft'], [0, 'real image recognition'], [0, 'landing']];
    mText('Done, in simulation', X0, 612, 30, { align: 'left', color: MC.greyA, alpha: a * clamp((t - (S.now + 1.6)) / 0.5), it: true });
    rows.forEach(([done, name], i) => {
      const y = 660 + i * 44 + (i >= 5 ? 44 : 0), u = clamp((t - (S.now + 2.0 + i * 0.6)) / 0.4);
      if (i === 5) mText('Not done', X0, 660 + 5 * 44 + 4, 30, { align: 'left', color: MC.greyA, alpha: a * clamp((t - (S.now + 2.0 + 4.6 * 0.6)) / 0.5), it: true });
      if (u <= 0) return;
      ctx.save(); ctx.globalAlpha = a * u;
      if (done) { ctx.fillStyle = Q_FOUND; ctx.fillRect(X0, y - 20, 22, 22); } else { ctx.strokeStyle = MC.greyC; ctx.lineWidth = 3; ctx.strokeRect(X0 + 1.5, y - 18.5, 19, 19); }
      ctx.restore();
      mText(name, X0 + 40, y, 32, { align: 'left', color: done ? MC.white : MC.grey, alpha: a * u });
    });
  }

  // ---- step 10: the language prior as a flow, then its region on the map
  if (t >= S.prior && t < S.priorWhen) {
    const a = clamp((S.priorWhen - t) / 0.5), X0 = CARDX;
    const rows = [['operator\'s field report: "last seen near the river"', 692], ['language model, with a list of named places', 772], ['regions on the map: circles or capsules, each with a weight', 852], ['starting numbers for every sector', 932]];
    rows.forEach(([txt, y], i) => {
      const u = clamp((t - (S.prior + 2.6 + i * 1.7)) / 0.5); if (u <= 0) return;
      ctx.save(); ctx.globalAlpha = a * u; ctx.strokeStyle = i === 1 ? MC.gold : MC.greyA; ctx.lineWidth = 3; ctx.strokeRect(X0, y - 36, 800, 56); ctx.restore();
      mText(txt, X0 + 16, y + 2, 28, { align: 'left', color: MC.white, alpha: a * u, it: i === 0 });
      if (i > 0) mArrow(X0 + 400, y - 62, X0 + 400, y - 38, MC.greyA, 3, u, a * 0.9);
    });
    // on the grid: the river, the region around it, then the numbers
    const riverP = runS(S.prior + 5.6, 1.4), capA = a * clamp((t - (S.prior + 6.8)) / 0.8);
    if (capA > 0) { ctx.save(); ctx.beginPath(); ctx.rect(GX0, GY0, 8 * CELLPX, 8 * CELLPX); ctx.clip(); ctx.globalAlpha = 0.2 * capA; ctx.strokeStyle = Q_BEL; ctx.lineWidth = 12 * PXM; ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.beginPath(); RIVER.forEach(([x, y], i) => (i ? ctx.lineTo(gx(x), gy(y)) : ctx.moveTo(gx(x), gy(y)))); ctx.stroke(); ctx.restore(); }
    showCreation(RIVER.map(([x, y]) => px(x, y)), riverP, { c: MC.greyA, w: 6, al: 0.8 * a, dash: [16, 10] });
    mText('river', gx(1), gy(31.5), 30, { align: 'left', color: MC.greyA, alpha: a * clamp((t - (S.prior + 6.2)) / 0.6), it: true });
    tagBox('one region, a capsule', gx(13), gy(37.2), 26, Q_BEL, capA);
    const la = a * clamp((t - (S.prior + 1.8)) / 0.7), v65 = t >= S.prior + 8.7;
    tagBox('50%', cellX(2) + 50, cellY(2) + 62, 34, MC.white, la, 'center');
    tagBox(v65 ? '65%' : '50%', cellX(2) + 50, cellY(5) + 62, 34, MC.white, la, 'center');
  }
  // ---- step 11: when: once, at the start; then the team starts together and evidence keeps correcting it
  if (t >= S.priorWhen && t < S.novel) {
    const a = clamp((S.novel - t) / 0.5), X0 = CARDX, T = S.priorWhen;
    const bar = (y, x0, x1, c, t0, label, dash) => {
      const u = runS(t0, 0.8); if (u <= 0) return;
      if (dash) { ctx.save(); ctx.setLineDash([14, 10]); segLine(x0, y, x1, y, c, 6, u, a); ctx.restore(); } else segLine(x0, y, x1, y, c, 24, u, 0.9 * a);
      mText(label, x0, y - 24, 27, { align: 'left', color: MC.white, alpha: a * clamp((t - t0 - 0.2) / 0.5) });
    };
    const ua = clamp((t - (T + 1.2)) / 0.5);
    segLine(X0 + 90, 668, X0 + 90, 962, MC.greyC, 2.5, runS(T + 1.2, 0.6), 0.8 * a);
    mText('start', X0 + 90, 656, 26, { color: MC.greyA, alpha: a * ua, it: true });
    bar(742, X0, X0 + 90, Q_BEL, T + 1.5, '1  the clue becomes the prior, once', false);
    bar(812, X0 + 98, X0 + 800, Q_LEAD, T + 3.0, '2  the leader sweeps everything', false);
    bar(882, X0 + 98, X0 + 800, Q_FOL, T + 3.6, '2  the followers start too', false);
    bar(952, X0 + 98, X0 + 800, Q_BEL, T + 5.0, '3  evidence keeps updating the map', true);
    // the target the clue missed: the numbers climb with each look
    const ca = a * clamp((t - (T + 5.4)) / 0.6), cm = px(...cellMid(6, 1));
    mDot(cm[0], cm[1] - 22, 12, Q_TGT, ca);
    tagBox(pct(priorState(6, 1).p) + '%', cm[0], cm[1] + 28, 34, MC.white, ca, 'center');
    tagBox('a target the clue missed', cellX(4) + 20, cellY(1) - 12, 26, Q_TGT, ca, 'left');
    ctx.save(); ctx.beginPath(); ctx.rect(GX0, GY0, 8 * CELLPX, 8 * CELLPX); ctx.clip(); ctx.globalAlpha = 0.2 * a; ctx.strokeStyle = Q_BEL; ctx.lineWidth = 12 * PXM; ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.beginPath(); RIVER.forEach(([x, y], i) => (i ? ctx.lineTo(gx(x), gy(y)) : ctx.moveTo(gx(x), gy(y)))); ctx.stroke(); ctx.restore();
    showCreation(RIVER.map(([x, y]) => px(x, y)), 1, { c: MC.greyA, w: 6, al: 0.8 * a, dash: [16, 10] });
    tagBox('65%', cellX(2) + 50, cellY(5) + 62, 34, MC.white, a, 'center');
  }

  // ---- step 12: what is known, and the one thing this project adds
  if (t >= S.novel && t < S.question) {
    const a = clamp((S.question - t) / 0.5), X0 = CARDX;
    const rows = [['field report to a prior map', 'exists: LMPath, 2611.16524', 700], ['a map that learns from empty looks', 'exists: Bayesian search', 780], ['drones that pick the best open sector', 'exists: many planners', 860], ['what a wrong clue costs, measured', 'this project', 940]];
    rows.forEach(([txt, tag, y], i) => {
      const u = clamp((t - (S.novel + 2.4 + i * 1.7)) / 0.5); if (u <= 0) return;
      const mine = i === 3;
      ctx.save(); ctx.globalAlpha = a * u; ctx.strokeStyle = mine ? Q_BEL : MC.greyC; ctx.lineWidth = mine ? 5 : 3; ctx.strokeRect(X0, y - 36, 830, 56); ctx.restore();
      mText(txt, X0 + 16, y + 2, 28, { align: 'left', color: mine ? MC.white : MC.grey, alpha: a * u });
      mText(tag, X0 + 814, y + 2, 25, { align: 'right', color: mine ? Q_BEL : MC.greyC, alpha: a * u, it: true });
    });
  }
  // ---- step 11: the question, four small maps
  if (t >= S.question && t < S.close) {
    const a = clamp((S.close - 0.6 - t) / 0.5), pa = a * clamp((t - (S.question + 0.8)) / 0.6);
    writeText('What does a wrong clue cost?', 960, 200, 68, runS(S.question + 0.15, 1.0), { alpha: a, bold: true });
    mText('Planned. Not built or run yet.', 960, 262, 32, { color: MC.gold, alpha: a * clamp((t - (S.question + 0.8)) / 0.6), it: true });
    const g = (cx, cy) => (c, r) => 0.5 + 0.35 * Math.exp(-((c - cx) ** 2 + (r - cy) ** 2) / (2 * 1.6 * 1.6));
    const maps = [() => 0.5, g(4, 5), g(1, 1), g(1, 1)], names = [['No clue', ''], ['Right clue', ''], ['Wrong clue', ''], ['Wrong clue,', 'empty looks ignored']];
    maps.forEach((pf, i) => {
      const x0 = 170 + i * 440, y0 = 330, ia = pa * clamp((t - (S.question + 0.8 + i * 0.25)) / 0.5);
      miniMap(x0, y0, 40, pf, ia, [4, 5]);
      mText(names[i][0], x0 + 160, 740, 36, { color: MC.white, alpha: ia });
      if (names[i][1]) mText(names[i][1], x0 + 160, 786, 30, { color: MC.grey, alpha: ia, it: true });
    });
    mText('(the red dot is where the target really is)', 960, 832, 28, { color: MC.greyC, alpha: pa * clamp((t - (S.question + 3.0)) / 0.6), it: true });
    mText('20 scenarios x 4 conditions = 80 runs, in a fast 2D simulator', 960, 884, 32, { color: MC.white, alpha: pa * clamp((t - (S.question + 4.5)) / 0.6) });
    mText('We time the first find, all found (unfound runs counted as cut off), and coverage at 15 simulated minutes', 960, 932, 28, { color: MC.greyA, alpha: pa * clamp((t - (S.question + 6.5)) / 0.6) });
    mText('The last condition shows how much the map\'s learning from empty looks adds, apart from the sweep', 960, 978, 28, { color: MC.greyA, alpha: pa * clamp((t - (S.question + 8.5)) / 0.6), it: true });
  }

  // ---- a slow ripple around each find while the picture rests (steps 9 and 12)
  if (sa > 0 && ((t >= S.now && t < S.prior - 0.6) || t >= S.close + 0.6)) {
    const ra = sa * (t < S.prior - 0.6 ? clamp((t - S.now) / 0.6) * clamp((S.prior - 0.6 - t) / 0.6) : clamp((t - S.close - 0.6) / 0.6));
    FPLAN.forEach((p, i) => { const u = ((t + i * 0.8) % 2.4) / 2.4, [x, y] = px(...cellMid(...p.cell)); ctx.save(); ctx.globalAlpha = 0.55 * (1 - u) * ra; ctx.strokeStyle = Q_FOUND; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(x, y, 34 + 46 * u, 0, TAU); ctx.stroke(); ctx.restore(); });
  }
  // ---- the cards, the counter, the key
  CARDS.forEach(drawCard);
  drawStepTag();
  drawSchematicTag();
  drawKey();
}
