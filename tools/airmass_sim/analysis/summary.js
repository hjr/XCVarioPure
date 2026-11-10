// Every series against the two GNSS references on the real flights, with the page's own code (build, derive):
// AirMass now, after the first real-flight tuning, before any real-flight tuning (a833f536), TE netto. (The GNSS/INS
// netto after the Larus design in RESULTS-2026-11-06.md came from the page, which no longer has it.)
//   node summary.js <web dir> <old airmass.js> log1.nmea.gz ...
const path = require("path");
const { loadPage, cat, lagOf, dampSeries, freeMask } = require("./page.js");
const web = path.resolve(process.argv[2]), oldModule = path.resolve(process.argv[3]);
const { runReplay, loadFlight } = loadPage(web);
const HZ = 10;

(async () => {
  const Mnow = await require(path.join(web, "airmass.js"))(), Mold = await require(oldModule)();
  const variants = [["AirMass now", Mnow, ""], ["AirMass, 1st real-flight tuning", Mnow, "kinetic_tau=10,blend_bank=0"], ["AirMass before real-flight tuning", Mold, ""]];
  const runs = [];
  let hours = 0;
  for (const log of process.argv.slice(4)) {
    fileName = path.basename(log);
    const conv = loadFlight(log);
    const vs = [];
    for (const [nm, M, st] of variants) {
      D = build(conv, await runReplay(M, conv.rec, { settings: st }));
      derive(D);
      vs.push([nm, D]);
    }
    D = vs[0][1]; // the references and TE from the current run (same converter, same track)
    const D0 = D, r = D0.rep.cols, c = D0.tr, n = D0.n;
    const inert = D0.gnssNetInert;
    const { free } = freeMask(D0, (k) => vs.every(([, v]) => v.amOk[k]) && c.ias[k] > 60 / 3.6 && Number.isFinite(D0.gnssNetPitot[k]) &&
      Number.isFinite(inert[k]) && Number.isFinite(D0.netto[k]));
    for (let k = 0; k < n; k++) hours += free[k] / 36000;
    const ser = {}, shown = {};
    for (const [nm, v] of vs) { ser[nm] = dampSeries(v.amRaw, free); shown[nm] = v.am; }
    ser["TE netto"] = D0.netto; shown["TE netto"] = D0.netto;
    runs.push({ n, free, roll: Float32Array.from({ length: n }, (_, k) => D0.ai[k] >= 0 ? r.roll_deg[D0.ai[k]] : NaN), refs: { pitot: D0.gnssNetPitot, inertial: inert }, ser, shown });
    process.stderr.write(fileName + "\n");
  }
  const names = Object.keys(runs[0].ser);
  const ok = cat(runs, (a, k) => a.free[k]);
  console.log(`${process.argv.length - 4} flights, ${hours.toFixed(1)} h free flight; AirMass with the TE vario's 4 s damping, every series shifted by its own delay`);
  for (const refName of ["inertial", "pitot"]) {
    const ref = cat(runs, (a, k) => a.refs[refName][k]);
    console.log(`\n=== against the GNSS netto with the speed term from ${refName === "pitot" ? "pitot TAS (gusts count as air movement)" : "GNSS ground speed minus a 2 min wind (gust-insensitive)"}`);
    const lags = {}; for (const s of names) lags[s] = lagOf(cat(runs, (a, k) => a.ser[s][k]), ref, ok);
    console.log("  delays: " + names.map((s) => `${s} ${lags[s] / HZ} s`).join(", "));
    for (const W of [10, 30, 300]) {
      const acc = {};
      for (const a of runs) for (let k0 = 0; k0 + W * HZ + 60 < a.n; k0 += W * HZ) {
        let okw = true, absr = 0, rs = 0; const sum = {}; for (const s of names) sum[s] = 0;
        for (let k = k0; k < k0 + W * HZ; k++) {
          if (!a.free[k] || names.some((s) => !a.free[k + lags[s]])) { okw = false; break; }
          rs += a.refs[refName][k]; absr += Math.abs(a.roll[k]); for (const s of names) sum[s] += a.ser[s][k + lags[s]];
        }
        if (!okw) continue;
        absr /= W * HZ;
        const phs = W >= 300 ? ["all"] : ["all", absr > 25 ? "circling" : absr < 10 ? "straight" : "mixed"];
        for (const p of phs) for (const s of names) { const e = (sum[s] - rs) / (W * HZ); const x = ((acc[p] ||= {})[s] ||= { n: 0, s2: 0, s: 0 }); x.n++; x.s2 += e * e; x.s += e; }
      }
      console.log(`  ${W} s windows, rms m/s (${W >= 300 ? "all" : "all / straight / circling"}; mean all):`);
      for (const s of names) {
        const v = (p) => (acc[p] && acc[p][s] ? Math.sqrt(acc[p][s].s2 / acc[p][s].n).toFixed(3) : "–");
        console.log(`    ${s.padEnd(36)} ${W >= 300 ? v("all") : `${v("all")} / ${v("straight")} / ${v("circling")}`}   (${(acc.all[s].s / acc.all[s].n).toFixed(3)})   [${acc.all[s].n}]`);
      }
    }
  }
  console.log("\nas shown in the cockpit: needle movement within 1 s, rms m/s");
  for (const s of names) {
    let s2 = 0, m = 0;
    for (const a of runs) for (let k = 0; k + 10 < a.n; k += 5) { if (!a.free[k] || !a.free[k + 10]) continue; const d = a.shown[s][k + 10] - a.shown[s][k]; if (Number.isFinite(d)) { s2 += d * d; m++; } }
    console.log(`    ${s.padEnd(36)} ${Math.sqrt(s2 / m).toFixed(3)}`);
  }
})().catch((e) => { console.error(e); process.exit(1); });
