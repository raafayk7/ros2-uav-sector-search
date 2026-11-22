  // SCORE BODY — drone search explainer (inside buildScore(); see kit/score-head.js).
  // Thin early (one pad), fuller as ideas stack; a hard stop at 67.4, silence until the first confirmed find at 70.1,
  // the loudest hit there, then a flat warm chord and a slow decay. Every cue time comes from the scene code.
  const T = TIMELINE, cu = T.cues, SIL0 = cu.silence0, HIT = cu.hit;
  to = 'm';
  // --- the bed: one thin pad, then a second voice when the leader starts, a third when the followers move
  pad(0.0, 21.5, ['D3', 'A3', 'Fs4'], 0.016, { type: 'triangle', cut: 900, att: 1.2, rel: 0.8 });
  pad(21.0, SIL0, ['D3', 'A3', 'E4', 'Fs4'], 0.02, { type: 'triangle', cut: 1200, att: 1.0, rel: 0.4, send: 0.5 });
  pad(S.followers - 0.5, SIL0, ['E3', 'B3', 'Fs4', 'A4'], 0.018, { type: 'triangle', cut: 1500, att: 1.4, rel: 0.3, send: 0.5 });
  // --- step 1: a rising wave through the sectors, a chime on the lit one
  for (let k = 0; k < 14; k++) pluck(S.arena + 3.2 + k * 0.13, nz(['D4', 'Fs4', 'A4', 'B4', 'D5', 'Fs5', 'A5'][k % 7]), 0.05, -0.4 + (k % 7) * 0.13, 0.4, 2600, 0.35);
  chime(S.arena + 5.4, [nz('D5'), nz('A5')], 0.05);
  // --- step 2: the numbers arrive, soft blips as "?" becomes "50%"
  to = 's';
  for (let i = 0; i < 6; i++) blip(15.4 + i * 0.12, 880 + i * 90, 760 + i * 80, 0.09, 0.02, -0.3 + i * 0.12);
  // --- step 3: the sweep, a slow tone that follows the leader; a tick at each lane end
  to = 'm';
  drone(SW0 - 0.3, Math.min(SW1 + 1.0, SIL0), nz('D3'), 0.012, { att: 1.0, rel: 1.0 });   // stops before the silence
  to = 's';
  LEG.forEach((g, i) => blip(SW0 + (g.s0 + g.len) / LTOT * (SW1 - SW0), 700 + i * 90, 540 + i * 70, 0.12, 0.018, -0.2 + i * 0.1));
  // --- step 4: the curves step down and up
  to = 'm';
  const E0n = S.evidence + 2.2;
  ['A4', 'Fs4', 'E4', 'D4', 'A3'].forEach((n, k) => pluck(E0n + k * 1.9 + 0.2, nz(n), 0.055, -0.25, 0.5, 2200, 0.35));
  ['D5', 'Fs5', 'A5', 'B5'].forEach((n, k) => pluck(E0n + k * 1.9 + 0.55, nz(n), 0.05, 0.25, 0.5, 3000, 0.4));
  // --- step 5: a flicker per frame square
  to = 's';
  for (let i = 0; i < 10; i++) blip(S.detector + 2.2 + 0.3 + i * 0.14, 1500 + (i % 3) * 120, 1300, 0.04, 0.012, 0.3);
  for (let i = 0; i < 20; i++) blip(S.detector + 5.6 + 0.3 + i * 0.14, 1100 + (i % 4) * 60, 1000, 0.03, 0.008, -0.3);
  // --- step 6: the comparison with the original project, then each follower sets off and arrives
  pluck(S.followers + 1.5, nz('D4'), 0.04, -0.2, 0.5, 2400, 0.4);
  pluck(S.followers + 3.8, nz('A4'), 0.045, 0.2, 0.5, 2600, 0.4);
  to = 'm';
  FPLAN.forEach((p, i) => { pluck(p.depart, nz(['D4', 'Fs4', 'A4'][i]), 0.08, -0.4 + i * 0.4, 0.5, 2400, 0.4); chime(p.arrive, [nz(['A4', 'B4', 'D5'][i]), nz('Fs5')], 0.03, -0.3 + i * 0.3); });
  // --- the silence: nothing between SIL0 and the hit. Then the payoff: a short chord stab and a sub, a flat chord after it
  to = 's';
  sub(HIT, 0.55);
  to = 'm';
  chime(HIT, [nz('D5'), nz('Fs5'), nz('A5'), nz('D6')], 0.11);
  pad(HIT, 83.5, ['D3', 'A3', 'D4', 'Fs4'], 0.04, { type: 'triangle', cut: 1100, att: 0.5, rel: 1.4, send: 0.6 });
  FPLAN.slice(1).forEach((p) => chime(p.confirm, [nz('A4'), nz('D5'), nz('Fs5')], 0.045));
  // --- step 8: the report lands softly
  chime(DONE_T + 0.2, [nz('A4'), nz('E5')], 0.04);
  // --- steps 9-12: a slow decay, a rising line for the clue, a held question, a last chime that loops back to the empty grid
  pad(83.0, 90.5, ['D3', 'A3', 'Fs4'], 0.014, { type: 'triangle', cut: 900, att: 1.5, rel: 1.5 });
  ['D4', 'Fs4', 'A4', 'D5'].forEach((n, k) => pluck(S.prior + 2.6 + k * 1.7, nz(n), 0.05, -0.2 + k * 0.12, 0.6, 2400, 0.4));
  chime(S.prior + 6.8, [nz('A4'), nz('E5')], 0.03);
  [[1.5, 'D4'], [3.0, 'A4'], [3.6, 'B4'], [5.0, 'Fs5']].forEach(([dt, n], k) => pluck(S.priorWhen + dt, nz(n), 0.045, -0.2 + k * 0.12, 0.5, 2400, 0.4));
  [[6.0, 'D5'], [7.2, 'Fs5'], [8.4, 'A5'], [9.6, 'B5']].forEach(([dt, n]) => pluck(S.priorWhen + dt, nz(n), 0.04, 0.25, 0.5, 3000, 0.4));
  ['D4', 'E4', 'Fs4', 'A4'].forEach((n, k) => pluck(S.novel + 2.4 + k * 1.7, nz(n), k === 3 ? 0.06 : 0.04, -0.15 + k * 0.1, 0.6, 2400, 0.4));
  chime(S.novel + 2.4 + 3 * 1.7, [nz('D5'), nz('A5')], 0.03);
  [4.5, 6.5, 8.5].forEach((dt, k) => pluck(S.question + dt, nz(['A4', 'B4', 'D5'][k]), 0.04, 0.1 * k, 0.5, 2600, 0.4));
  pad(S.prior, S.close, ['D3', 'A3', 'E4'], 0.014, { type: 'triangle', cut: 1000, att: 1.2, rel: 1.0 });
  drone(S.question, S.close + 0.5, nz('A3'), 0.014, { att: 0.8, rel: 0.8 });
  pad(S.close - 0.5, DURATION + 0.4, ['D3', 'A3', 'D4', 'Fs4'], 0.018, { type: 'triangle', cut: 1000, att: 1.2, rel: 3.0, send: 0.6 });
  chime(DURATION - 4.2, [nz('D5'), nz('A5'), nz('Fs6')], 0.05);
