// faster? the delay that lines each series up best with the GNSS netto reference (which sits at the true time),
// from the cross-correlation at 2 Hz. Less noisy? how much the needle moves within a second (rms of the 1 s change),
// and how much of the series is not in the reference (rms of the difference at that best delay, 2 s means).
//   node lag.js <web dir> log1.nmea.gz ...
const path = require("path");
const { loadPage, dampSeries, freeMask } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { runReplay, loadFlight } = loadPage(web);
const LAGS = []; for (let l = -2; l <= 10.001; l += 0.25) LAGS.push(+l.toFixed(2));
const SER = ["AirMass (raw)", "AirMass cockpit (half damping)", "AirMass with TE's damping", "TE netto (device)"];
const xc = {}; for (const s of SER) xc[s] = LAGS.map(() => ({ n: 0, sx: 0, sy: 0, sxx: 0, syy: 0, sxy: 0 }));
const jit = {}; // phase -> series -> {n, s2}
const addJ = (ph, s, d) => { const a = ((jit[ph] ||= {})[s] ||= { n: 0, s2: 0 }); a.n++; a.s2 += d * d; };
const resid = {}; // series -> lag -> {n, s2} on 2 s means
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = loadFlight(log);
    D = build(conv, await runReplay(M, conv.rec, {}));
    derive(D);
    const r = D.rep.cols, c = D.tr, n = D.n;
    // AirMass with the TE vario's damping (the device's vario damping, 4 s), for an equal comparison
    const amTe = dampSeries(D.amRaw, D.amOk);
    const arr = { "AirMass (raw)": D.amRaw, "AirMass cockpit (half damping)": D.am, "AirMass with TE's damping": amTe, "TE netto (device)": D.netto };
    const { free } = freeMask(D, (k) => D.amOk[k] && c.ias[k] > 60 / 3.6 && Number.isFinite(D.gnssNetPitot[k]));
    // 2 Hz samples
    const step = 5;
    for (let k = 0; k < n; k += step) {
      if (!free[k]) continue;
      const ref = D.gnssNetPitot[k];
      LAGS.forEach((lag, li) => {
        const kk = k + Math.round(lag * 10);
        if (kk < 0 || kk >= n || !free[kk]) return;
        for (const s of SER) {
          const x = arr[s][kk];
          if (!Number.isFinite(x)) continue;
          const a = xc[s][li]; a.n++; a.sx += x; a.sy += ref; a.sxx += x * x; a.syy += ref * ref; a.sxy += x * ref;
        }
      });
      // 1 s change
      const k1 = k + 10;
      if (k1 < n && free[k1]) {
        const ph = Math.abs(r.roll_deg[D.ai[k]]) > 20 ? "circling" : "straight";
        for (const s of SER) { const d = arr[s][k1] - arr[s][k]; if (Number.isFinite(d)) { addJ(ph, s, d); addJ("all", s, d); } }
        const d = D.gnssNetPitot[k1] - D.gnssNetPitot[k]; addJ("all", "GNSS netto (reference)", d); addJ(Math.abs(r.roll_deg[D.ai[k]]) > 20 ? "circling" : "straight", "GNSS netto (reference)", d);
      }
    }
    process.stderr.write(fileName + "\n");
  }
  console.log("Delay to the GNSS netto reference (best cross-correlation), and the correlation there");
  const best = {};
  for (const s of SER) {
    let bi = 0, bc = -2;
    const corr = xc[s].map((a) => (a.n * a.sxy - a.sx * a.sy) / Math.sqrt((a.n * a.sxx - a.sx * a.sx) * (a.n * a.syy - a.sy * a.sy)));
    corr.forEach((v, i) => { if (v > bc) { bc = v; bi = i; } });
    best[s] = LAGS[bi];
    console.log(`  ${s.padEnd(32)} delay ${LAGS[bi].toFixed(2).padStart(5)} s   correlation ${bc.toFixed(3)}   (at 0 s: ${corr[LAGS.indexOf(0)].toFixed(3)})`);
  }
  console.log("\nNeedle movement within 1 s, rms m/s (less = calmer)");
  for (const ph of ["all", "straight", "circling"]) {
    console.log(`  ${ph}`);
    for (const s of [...SER, "GNSS netto (reference)"]) { const a = jit[ph][s]; console.log(`    ${s.padEnd(32)} ${Math.sqrt(a.s2 / a.n).toFixed(3)}`); }
  }
})().catch((e) => { console.error(e); process.exit(1); });
