// AirMass's along-track wind bias in straight flight without a reference: straight legs (bank < 10°, track within ±8°,
// >= 40 s) paired with a leg within 6 min whose track differs by > 120°. A bias b along the track gives
// (W1 − W2)·t1 = b (1 − t1·t2); real wind changes average out over the pairs.
//   node opposite_legs.js LOG...   (SETTINGS="name=value,...": AirMass settings)
const fs = require("fs"), path = require("path");
const A = __dirname;
const { loadPage, freeMask } = require(path.join(A, "page.js"));
const web = path.resolve(A, "../web");
const { runReplay, loadFlight } = loadPage(web);
const D2R = Math.PI / 180;
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const all = [];
  for (const log of process.argv.slice(2)) {
    globalThis.fileName = path.basename(log);
    const conv = loadFlight(log);
    const rep = await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" });
    if (!conv.track.n || !rep.n) continue;
    globalThis.D = build(conv, rep); derive(D);
    const c = D.tr, r = D.rep.cols, n = D.n, { free } = freeMask(D, (k) => c.ias[k] > 60 / 3.6);
    // legs
    const legs = []; let s = -1;
    const close = (k) => {
      if (s < 0) return;
      const ks = []; for (let j = s; j < k; j++) ks.push(j);
      s = -1;
      if (D.t[k - 1] - D.t[ks[0]] < 40) return;
      let sn = 0, se = 0, wn = 0, we = 0, an = 0, ae = 0, tas = 0, m = 0;
      for (const j of ks) { const i = D.ai[j]; if (i < 0 || !(r.status[i] & 18)) continue;
        sn += Math.cos(c.gnss_track[j] * D2R); se += Math.sin(c.gnss_track[j] * D2R);
        wn += r.wind_n[i]; we += r.wind_e[i]; an += r.avg_wind_n[i]; ae += r.avg_wind_e[i]; tas += c.tas[j]; m++; }
      if (m < 100) return;
      const h = Math.hypot(sn, se);
      legs.push({ t0: D.t[ks[0]], t1: D.t[k - 1], tn: sn / h, te: se / h, wn: wn / m, we: we / m, an: an / m, ae: ae / m, tas: tas / m });
    };
    let mt = NaN;
    for (let k = 0; k < n; k++) {
      const i = D.ai[k];
      const okk = free[k] && i >= 0 && Math.abs(r.roll_deg[i]) < 10 && Number.isFinite(c.gnss_track[k]);
      if (!okk) { close(k); continue; }
      if (s < 0) { s = k; mt = c.gnss_track[k]; continue; }
      const d = Math.abs(((c.gnss_track[k] - mt) % 360 + 540) % 360 - 180);
      if (d > 8) { close(k); s = k; mt = c.gnss_track[k]; }
    }
    close(n);
    for (let a = 0; a < legs.length; a++) for (let b = a + 1; b < legs.length; b++) {
      const L1 = legs[a], L2 = legs[b];
      if (L2.t0 - L1.t1 > 360) break;
      const dot = L1.tn * L2.tn + L1.te * L2.te;
      if (dot > -0.5) continue;
      // symmetric: bias along each leg's own track
      const d = (x1n, x1e, x2n, x2e) => ((x1n - x2n) * L1.tn + (x1e - x2e) * L1.te - (x1n - x2n) * L2.tn - (x1e - x2e) * L2.te) / (2 * (1 - dot));
      all.push({ f: fileName, inst: d(L1.wn, L1.we, L2.wn, L2.we), avg: d(L1.an, L1.ae, L2.an, L2.ae), tas: (L1.tas + L2.tas) / 2, gap: L2.t0 - L1.t1 });
    }
    process.stderr.write(`${fileName} legs ${legs.length}\n`);
  }
  const mean = (a) => a.reduce((s, x) => s + x, 0) / a.length, sd = (a) => Math.sqrt(mean(a.map((x) => (x - mean(a)) ** 2)));
  const med = (a) => { a = [...a].sort((x, y) => x - y); return a[a.length >> 1]; };
  for (const key of ["inst", "avg"]) {
    const v = all.map((x) => x[key]);
    console.log(`${key}: pairs ${v.length}, mean bias along track ${mean(v).toFixed(2)} ± ${(sd(v) / Math.sqrt(v.length)).toFixed(2)} (se), median ${med(v).toFixed(2)}`);
  }
  for (const [lo, hi] of [[0, 30], [30, 38], [38, 45], [45, 80]]) {
    const v = all.filter((x) => x.tas >= lo && x.tas < hi).map((x) => x.inst);
    if (v.length) console.log(`  TAS ${lo}-${hi} m/s: pairs ${v.length}, inst bias ${mean(v).toFixed(2)} ± ${(sd(v) / Math.sqrt(v.length)).toFixed(2)}`);
  }
  const files = [...new Set(all.map((x) => x.f))];
  console.log("per flight inst bias: " + files.map((f) => { const v = all.filter((x) => x.f === f).map((x) => x.inst); return `${f.slice(0, 16)} ${mean(v).toFixed(2)} (${v.length})`; }).join(", "));
})().catch((e) => { console.error(e); process.exit(1); });
