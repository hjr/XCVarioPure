// Which flight conditions do the errors follow? AirMass (raw, with the TE vario's 4 s damping) and TE netto against
// the GNSS netto reference, each shifted by its own delay to the reference, in 10 s window means; the conditions per
// window; binned means and a least squares fit. Same definitions as cond_sim.py.
//   MASS=430 SETTINGS=... node cond.js <web dir> log1.nmea.gz ...
const fs = require("fs"), zlib = require("zlib"), path = require("path");
const { loadPage, lagOf, dampSeries, freeMask, inertialRef } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { convert, runReplay } = loadPage(web);
const WIN = 10, HZ = 10;
const rows = []; // one per window
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const all = [];
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = convert(zlib.gunzipSync(fs.readFileSync(log)).toString("latin1"), { glider: "ASG 29/18", mass: +(process.env.MASS || 0) });
    D = build(conv, await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" }));
    derive(D);
    const r = D.rep.cols, c = D.tr, n = D.n;
    const am = dampSeries(D.amRaw, D.amOk);
    const { free, since } = freeMask(D, (k) => D.amOk[k] && c.ias[k] > 60 / 3.6 && Number.isFinite(D.gnssNetRaw[k]) && Number.isFinite(D.netto[k]));
    const roll = Float32Array.from({ length: n }, (_, k) => (D.ai[k] >= 0 ? r.roll_deg[D.ai[k]] : NaN));
    all.push({ name: fileName, n, am, te: D.netto, ref: D.gnssNetRaw, free, since, roll, c, r, ai: D.ai, t: D.t });
    process.stderr.write(fileName + "\n");
  }
  // delays over all flights
  const cat = (f) => { const out = []; for (const a of all) for (let k = 0; k < a.n; k++) out.push(f(a, k)); return out; };
  const ok = cat((a, k) => a.free[k]), ref = cat((a, k) => a.ref[k]);
  const lagAm = lagOf(cat((a, k) => a.am[k]), ref, ok), lagTe = lagOf(cat((a, k) => a.te[k]), ref, ok);
  console.log(`delays to the reference: AirMass (4 s damping) ${lagAm / HZ} s, TE netto ${lagTe / HZ} s`);
  for (const a of all) {
    const fi = all.indexOf(a);
    for (let k0 = 0; k0 + WIN * HZ + 60 < a.n; k0 += WIN * HZ) {
      let okw = true, eA = 0, eT = 0, cnt = 0, rollS = 0, rollAbs = 0, ias = 0, rr2 = 0, dv2 = 0, since = 0, nlf = 0, trk = [];
      for (let k = k0; k < k0 + WIN * HZ; k++) {
        if (!a.free[k] || !a.free[k + lagAm] || !a.free[k + lagTe]) { okw = false; break; }
        eA += a.am[k + lagAm] - a.ref[k]; eT += a.te[k + lagTe] - a.ref[k]; cnt++;
        rollS += a.roll[k]; rollAbs += Math.abs(a.roll[k]); ias += a.c.ias[k] * 3.6; since += a.since[k];
        nlf += 1 / Math.max(Math.cos(a.roll[k] * Math.PI / 180), 0.3) - 1;
        if (k > k0) { const d = (a.roll[k] - a.roll[k - 1]) * HZ; rr2 += d * d; const dv = (a.c.tas[k] - a.c.tas[k - 1]) * HZ; dv2 += dv * dv; }
      }
      if (!okw) continue;
      rows.push({ f: fi, eA: eA / cnt, eT: eT / cnt, roll: rollS / cnt, absRoll: rollAbs / cnt, ias: ias / cnt, rr: Math.sqrt(rr2 / cnt), dv: Math.sqrt(dv2 / cnt), since: since / cnt / 60, nm1: nlf / cnt });
    }
  }
  const rmsAll = (f) => Math.sqrt(rows.reduce((s, x) => s + f(x) ** 2, 0) / rows.length);
  console.log(`SUMMARY settings "${process.env.SETTINGS || ""}": ${rows.length} windows, rms AirMass ${rmsAll((x) => x.eA).toFixed(3)}, TE ${rmsAll((x) => x.eT).toFixed(3)}; calm (TAS change < 0.6) AirMass ${Math.sqrt(rows.filter((x) => x.dv < 0.6).reduce((s, x) => s + x.eA ** 2, 0) / rows.filter((x) => x.dv < 0.6).length).toFixed(3)}; circling AirMass ${Math.sqrt(rows.filter((x) => x.absRoll > 25).reduce((s, x) => s + x.eA ** 2, 0) / rows.filter((x) => x.absRoll > 25).length).toFixed(3)}, straight ${Math.sqrt(rows.filter((x) => x.absRoll < 10).reduce((s, x) => s + x.eA ** 2, 0) / rows.filter((x) => x.absRoll < 10).length).toFixed(3)}`);
  if (!process.env.SETTINGS) report(rows, all.map((a) => a.name));
})().catch((e) => { console.error(e); process.exit(1); });

function report(rows, names) {
  console.log(`${rows.length} windows of ${WIN} s; mean error against the GNSS netto reference, m/s (± standard error)`);
  const se = (v) => { const m = v.reduce((a, b) => a + b, 0) / v.length; const s = Math.sqrt(v.reduce((a, b) => a + (b - m) ** 2, 0) / (v.length - 1)); return [m, s / Math.sqrt(v.length)]; };
  const rmsOf = (v) => Math.sqrt(v.reduce((a, b) => a + b * b, 0) / v.length);
  const fmt = ([m, s]) => `${m >= 0 ? "+" : ""}${m.toFixed(3)}±${s.toFixed(3)}`;
  const table = (title, key, edges, unit) => {
    console.log(`\n${title}`);
    console.log("  bin".padEnd(20) + "windows".padStart(8) + "AirMass".padStart(16) + "TE netto".padStart(16) + "AirMass−TE".padStart(16) + "rms AM".padStart(9) + "rms TE".padStart(9));
    for (let b = 0; b + 1 < edges.length; b++) {
      const sel = rows.filter((x) => x[key] >= edges[b] && x[key] < edges[b + 1]);
      if (sel.length < 20) continue;
      const lab = `${edges[b] === -Infinity ? "<" + edges[b + 1] : edges[b + 1] === Infinity ? "≥" + edges[b] : edges[b] + "…" + edges[b + 1]} ${unit}`;
      console.log(("  " + lab).padEnd(20) + String(sel.length).padStart(8) + fmt(se(sel.map((x) => x.eA))).padStart(16) +
        fmt(se(sel.map((x) => x.eT))).padStart(16) + fmt(se(sel.map((x) => x.eA - x.eT))).padStart(16) +
        rmsOf(sel.map((x) => x.eA)).toFixed(3).padStart(9) + rmsOf(sel.map((x) => x.eT)).toFixed(3).padStart(9));
    }
  };
  table("bank angle, signed (left negative)", "roll", [-Infinity, -40, -25, -10, 10, 25, 40, Infinity], "°");
  table("IAS", "ias", [0, 85, 100, 120, 150, 180, Infinity], "km/h");
  table("roll rate, rms in the window", "rr", [0, 2, 5, 10, 20, Infinity], "°/s");
  table("TAS change, rms in the window", "dv", [0, 0.3, 0.6, 1, 2, Infinity], "m/s²");
  table("time since the start", "since", [0, 15, 30, 60, 120, 240, Infinity], "min");
  // least squares
  const X = rows.map((x) => [1, x.nm1, x.roll / 45, (x.ias - 110) / 30, x.rr / 10, x.dv, x.since / 60]);
  const lbl = ["constant", "load factor − 1", "bank, signed /45°", "(IAS − 110)/30 km/h", "roll rate /10°/s", "TAS change m/s²", "hours since start"];
  const fit = (y) => {
    const p = X[0].length, A = Array.from({ length: p }, () => new Array(p + 1).fill(0));
    X.forEach((xr, i) => { for (let a = 0; a < p; a++) { for (let b = 0; b < p; b++) A[a][b] += xr[a] * xr[b]; A[a][p] += xr[a] * y[i]; } });
    for (let i = 0; i < p; i++) { let m = i; for (let j = i + 1; j < p; j++) if (Math.abs(A[j][i]) > Math.abs(A[m][i])) m = j; [A[i], A[m]] = [A[m], A[i]];
      for (let j = 0; j < p; j++) if (j !== i) { const f = A[j][i] / A[i][i]; for (let q = i; q <= p; q++) A[j][q] -= f * A[i][q]; } }
    const beta = A.map((row, i) => row[p] / row[i]);
    const res = y.map((v, i) => v - X[i].reduce((s, x, j) => s + x * beta[j], 0));
    const r2 = 1 - res.reduce((s, v) => s + v * v, 0) / y.reduce((s, v) => s + (v - y.reduce((a, b) => a + b, 0) / y.length) ** 2, 0);
    return { beta, r2 };
  };
  const fa = fit(rows.map((x) => x.eA)), ft = fit(rows.map((x) => x.eT)), fd = fit(rows.map((x) => x.eA - x.eT));
  console.log(`\nleast squares fit of the window error, m/s per unit (R² AirMass ${fa.r2.toFixed(3)}, TE ${ft.r2.toFixed(3)}, difference ${fd.r2.toFixed(3)})`);
  console.log("  term".padEnd(26) + "AirMass".padStart(10) + "TE netto".padStart(10) + "AirMass−TE".padStart(12));
  lbl.forEach((l, i) => console.log(("  " + l).padEnd(26) + fa.beta[i].toFixed(3).padStart(10) + ft.beta[i].toFixed(3).padStart(10) + fd.beta[i].toFixed(3).padStart(12)));
  // per flight: circling left against right
  console.log("\nper flight, circling (|bank| > 25°): mean AirMass−TE, left / right, and AirMass error left / right");
  names.forEach((nm, fi) => {
    const L = rows.filter((x) => x.f === fi && x.roll < -25), R = rows.filter((x) => x.f === fi && x.roll > 25);
    if (L.length < 5 || R.length < 5) return;
    const m = (v) => (v.reduce((a, b) => a + b, 0) / v.length);
    console.log(`  ${nm.padEnd(26)} n ${String(L.length).padStart(4)}/${String(R.length).padEnd(4)} diff ${m(L.map((x) => x.eA - x.eT)).toFixed(2).padStart(6)} / ${m(R.map((x) => x.eA - x.eT)).toFixed(2).padStart(6)}   AirMass ${m(L.map((x) => x.eA)).toFixed(2).padStart(6)} / ${m(R.map((x) => x.eA)).toFixed(2).padStart(6)}`);
  });
}
