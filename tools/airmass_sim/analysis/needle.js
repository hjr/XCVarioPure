// The cockpit needle: how much it moves and how right it is, for AirMass variants and cockpit dampings, on the real
// flights. Per variant: needle movement within 1 s (rms, all / straight / circling by the bank of the moment), the
// delay of the shown value to the gust-insensitive GNSS reference, the 10 s error of the shown value at that delay, and
// its own error without the reference's (three-cornered hat with TE netto and the reference), 10 and 30 s; all errors
// over the same windows.
//   node needle.js <web dir> log1.nmea.gz ...
//   REALTIME=1: errors without the delay shift (as the pilot sees the needle); PHASE_LAG=1: the errors with each
//   series shifted by its delay in that phase
//   SERIES='[["name","settings",dampAir], ...]' (dampAir: the cockpit damping in s, default 2 = half the 4 s TE damping;
//   [straight, circling]: blended between 15° and 25° bank)
const path = require("path");
const { loadPage, cat, lagOf, freeMask } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { runReplay, loadFlight } = loadPage(web);
const HZ = 10;

// the cockpit damping dAir: a number, or [straight, circling] blended between 15° and 25° bank
const dampOf = (D, dAir) => {
  if (!Array.isArray(dAir)) return () => dAir;
  const [d0, d1] = dAir, roll = D.rep.cols.roll_deg;
  return (k) => d0 + (d1 - d0) * Math.min(1, Math.max(0, (Math.abs(roll[D.ai[k]]) - 15) / 10));
};

(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const variants = JSON.parse(process.env.SERIES || '[["now","",2],["blend at any bank","blend_bank=0",2],["1st tuning","kinetic_tau=10,blend_bank=0",2]]');
  const runs = [];
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = loadFlight(log);
    const reps = {}, ser = {};
    let D0 = null;
    for (const [nm, st, dAir] of variants) {
      if (!reps[st]) { D = build(conv, await runReplay(M, conv.rec, { settings: st })); derive(D); reps[st] = D; }
      D0 ||= reps[st];
      ser[nm] = cockpitSeries(reps[st], dampOf(reps[st], dAir ?? 2));
    }
    D = D0;
    const r = D0.rep.cols, c = D0.tr, n = D0.n;
    const inert = D0.gnssNetInert;
    const { free } = freeMask(D0, (k) => Object.values(reps).every((v) => v.amOk[k]) && c.ias[k] > 60 / 3.6 && Number.isFinite(inert[k]) && Number.isFinite(D0.netto[k]));
    ser["TE netto"] = D0.netto;
    const roll = Float32Array.from({ length: n }, (_, k) => (D0.ai[k] >= 0 ? r.roll_deg[D0.ai[k]] : NaN));
    runs.push({ n, free, roll, ref: inert, ser });
    process.stderr.write(fileName + "\n");
  }
  const names = Object.keys(runs[0].ser);
  const ok = cat(runs, (a, k) => a.free[k]), ref = cat(runs, (a, k) => a.ref[k]);
  const all = Object.fromEntries(names.map((s) => [s, cat(runs, (a, k) => a.ser[s][k])]));
  // the delays: over the whole flight, straight (bank under 10°), circling (over 25°)
  const okS = cat(runs, (a, k) => a.free[k] && Math.abs(a.roll[k]) < 10), okC = cat(runs, (a, k) => a.free[k] && Math.abs(a.roll[k]) > 25);
  const lags = {}, lagsS = {}, lagsC = {};
  for (const s of names) { lags[s] = lagOf(all[s], ref, ok); lagsS[s] = lagOf(all[s], ref, okS); lagsC[s] = lagOf(all[s], ref, okC); }
  const phase = (b) => (b < 10 ? "straight" : b > 25 ? "circling" : "mixed");
  // the shift of a series against the reference: its delay over the whole flight; REALTIME=1: none, the needle as the
  // pilot sees it (a late needle pays for it); PHASE_LAG=1: its delay in that phase (the lift changes around each
  // circle, so a delay that does not fit the circling shows as an error there)
  const shiftFor = (s, p) => (process.env.REALTIME ? 0 : !process.env.PHASE_LAG ? lags[s] : p === "circling" ? lagsC[s] : p === "straight" ? lagsS[s] : lags[s]);
  const maxShift = Math.max(...names.flatMap((s) => [lags[s], lagsS[s], lagsC[s]]));
  const W = 10 * HZ;
  // the errors over 10 and 30 s windows, each series against the reference, and its own error: that minus the
  // reference's own, the latter by the three-cornered hat of the first series, TE netto and the reference (taken
  // independent): σref² = (ms(s - ref) + ms(TE - ref) - ms(s - TE)) / 2. Not the hat per series: a series that leans on
  // the energy balance like TE netto shares more of its errors, which the hat would count to the reference
  const first = names.find((s) => s !== "TE netto");
  const raw = {}, own = {}; // [W][phase][series]: mean square error against the reference, own error variance
  for (const Wh of [W, 3 * W]) {
    const ms = {}, add = (p, key, v) => { const x = ((ms[p] ||= {})[key] ||= { n: 0, s2: 0 }); x.n++; x.s2 += v * v; };
    for (const a of runs) for (let k0 = 0; k0 + Wh + 60 < a.n; k0 += Wh) {
      let okw = true, absr = 0;
      for (let k = k0; k < k0 + Wh + maxShift; k++) {
        if (!a.free[k]) { okw = false; break; }
        if (k < k0 + Wh) absr += Math.abs(a.roll[k]);
      }
      if (!okw) continue;
      const p = phase(absr / Wh);
      const err = {};
      for (const s of names) { const sh = shiftFor(s, p); let e = 0; for (let k = k0; k < k0 + Wh; k++) e += a.ser[s][k + sh] - a.ref[k]; err[s] = e / Wh; }
      for (const q of [p, "all"]) { add(q, "first-TE", err[first] - err["TE netto"]); for (const s of names) add(q, s, err[s]); }
    }
    raw[Wh] = {}; own[Wh] = {};
    for (const p in ms) {
      const m = (key) => ms[p][key].s2 / ms[p][key].n;
      const ref2 = (m(first) + m("TE netto") - m("first-TE")) / 2;
      raw[Wh][p] = Object.fromEntries(names.map((s) => [s, m(s)]));
      own[Wh][p] = Object.fromEntries(names.map((s) => [s, m(s) - ref2]));
      own[Wh][p]["the reference"] = ref2;
    }
  }
  const rms = (v) => (v > 0 ? Math.sqrt(v).toFixed(3) : "  –  ");
  console.log("series                               needle 1 s (all / straight / circling)   delay (circling)   10 s error (all / straight / circling)   own error: 10 s (straight / circling)  30 s");
  for (const s of names) {
    const mv = {}, add = (p, v) => { const x = (mv[p] ||= { n: 0, s2: 0 }); x.n++; x.s2 += v * v; };
    for (const a of runs) for (let k = 0; k + 10 < a.n; k += 5) {
      if (!a.free[k] || !a.free[k + 10]) continue;
      const d = a.ser[s][k + 10] - a.ser[s][k];
      if (Number.isFinite(d)) { add("all", d); add(phase(Math.abs(a.roll[k])), d); }
    }
    const m = (p) => rms(mv[p] ? mv[p].s2 / mv[p].n : NaN);
    console.log(`${s.padEnd(36)} ${m("all")} / ${m("straight")} / ${m("circling")}            ${(lags[s] / HZ).toFixed(1)} s (${(lagsC[s] / HZ).toFixed(1)} s)        ${rms(raw[W].all[s])} / ${rms(raw[W].straight[s])} / ${rms(raw[W].circling[s])}` +
      `              ${rms(own[W].straight[s])} / ${rms(own[W].circling[s])}     ${rms(own[3 * W].straight[s])} / ${rms(own[3 * W].circling[s])}`);
  }
  console.log(`the reference's own error: 10 s ${rms(own[W].straight["the reference"])} / ${rms(own[W].circling["the reference"])}, 30 s ${rms(own[3 * W].straight["the reference"])} / ${rms(own[3 * W].circling["the reference"])} (from ${first}, TE netto and the reference)`);
})().catch((e) => { console.error(e); process.exit(1); });
