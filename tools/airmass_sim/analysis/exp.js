// Two references: the GNSS netto of the page (the speed term from pitot TAS: a horizontal gust counts as air movement)
// and an inertial one (the speed term from the GNSS ground velocity relative to a slowly varying wind: a horizontal
// gust does not move it). AirMass (blend on and off) and TE netto against both, each shifted by its own delay.
//   node exp.js <web dir> log1.nmea.gz ...   (SERIES='[["name","settings"], ...]', REF_SINK=airmass)
const path = require("path");
const { loadPage, cat, lagOf, dampSeries, freeMask } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { runReplay, loadFlight } = loadPage(web);
const HZ = 10;
async function flight(M, log, settings) {
  fileName = path.basename(log);
  const conv = loadFlight(log);
  D = build(conv, await runReplay(M, conv.rec, { settings }));
  derive(D);
  return { D, am: dampSeries(D.amRaw, D.amOk) };
}
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const runs = [];
  for (const log of process.argv.slice(3)) {
    // the AirMass variants: SERIES='[["name","settings"], ...]' (default: blend on, off, faded above 20°)
    const variants = JSON.parse(process.env.SERIES || '[["blend on",""],["blend off","gust_acc=1000"],["fade >20°","blend_bank=20"]]');
    const vs = [];
    for (const [nm, st] of variants) vs.push([nm, await flight(M, log, st)]);
    const on = vs[0][1];
    const D0 = on.D, r = D0.rep.cols, c = D0.tr, n = D0.n;
    const inert = D0.gnssNetInert;
    const { free } = freeMask(D0, (k) => vs.every(([, v]) => v.D.amOk[k]) && c.ias[k] > 60 / 3.6 && Number.isFinite(D0.gnssNetPitot[k]) &&
      Number.isFinite(inert[k]) && Number.isFinite(D0.netto[k]));
    const roll = Float32Array.from({ length: n }, (_, k) => (D0.ai[k] >= 0 ? r.roll_deg[D0.ai[k]] : NaN));
    runs.push({ n, free, roll, refs: { pitot: D0.gnssNetPitot, inertial: inert }, ser: Object.assign(Object.fromEntries(vs.map(([nm, v]) => [nm, v.am])), { "TE netto": D0.netto }) });
    process.stderr.write(path.basename(log) + "\n");
  }
  const ok = cat(runs, (a, k) => a.free[k]);
  for (const refName of ["pitot", "inertial"]) {
    const ref = cat(runs, (a, k) => a.refs[refName][k]);
    console.log(`\n=== reference with the speed term from ${refName === "pitot" ? "pitot TAS (the page's GNSS netto)" : "GNSS ground velocity minus a 2 min wind (gust-insensitive)"}`);
    const lags = {};
    for (const s in runs[0].ser) lags[s] = lagOf(cat(runs, (a, k) => a.ser[s][k]), ref, ok);
    console.log("  delays: " + Object.entries(lags).map(([s, l]) => `${s} ${l / HZ} s`).join(", "));
    for (const W of [10, 30]) {
      const acc = {};
      for (const a of runs) for (let k0 = 0; k0 + W * HZ + 60 < a.n; k0 += W * HZ) {
        let okw = true, absr = 0; const sum = {}; for (const s in a.ser) sum[s] = 0; let rs = 0;
        for (let k = k0; k < k0 + W * HZ; k++) {
          if (!a.free[k] || Object.values(lags).some((l) => !a.free[k + l])) { okw = false; break; }
          rs += a.refs[refName][k]; absr += Math.abs(a.roll[k]);
          for (const s in a.ser) sum[s] += a.ser[s][k + lags[s]];
        }
        if (!okw) continue;
        absr /= W * HZ;
        const ph = absr > 25 ? "circling" : absr < 10 ? "straight" : "mixed";
        for (const p of [ph, "all"]) for (const s in a.ser) { const e = (sum[s] - rs) / (W * HZ); const x = ((acc[p] ||= {})[s] ||= { n: 0, s2: 0, s: 0 }); x.n++; x.s2 += e * e; x.s += e; }
      }
      console.log(`  ${W} s windows, rms (mean) m/s:`);
      for (const p of ["all", "straight", "circling"]) {
        console.log(`    ${p.padEnd(9)} ` + Object.entries(acc[p]).map(([s, x]) => `${s} ${Math.sqrt(x.s2 / x.n).toFixed(3)} (${(x.s / x.n).toFixed(2)})`).join("   ") + `   [${Object.values(acc[p])[0].n} windows]`);
      }
    }
  }
})().catch((e) => { console.error(e); process.exit(1); });
