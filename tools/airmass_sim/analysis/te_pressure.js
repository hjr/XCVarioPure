// AirMass netto, the TE netto the device logged ($PXCV) and the TE netto replayed from the logged TE pressure (the
// page's TeVario) against the GNSS references, per flight and pooled.
//   node te_pressure.js OUTPUT.json LOG...      (EXCLUDE=file,...: logs left out of the pools)
// All three at the same 4 s damping (AirMass and TE pressure with the page's Damp; the logged TE as the device damped
// it, its setting is not in the log, 4 s by default) and each shifted by its fitted delay, like batch.js; the delays
// and the 1 s needle movement as shown. Own errors by the three-cornered hat, once with each TE: TE and TE pressure
// share the probe and its errors, so they never go into one hat together.
const fs = require("fs"), path = require("path");
const { loadPage, lagOf, dampSeries, freeMask } = require("./page.js");
const web = path.resolve(__dirname, "../web");
const { runReplay, loadFlight } = loadPage(web);
const SERIES = ["AirMass", "TE", "TEp"], REFS = ["inertial", "pitot"], WINDOWS = [10, 30], PHASES = ["all", "straight", "circling"];
const exclude = new Set((process.env.EXCLUDE || "").split(",").filter(Boolean));

async function evaluate(M, file) {
  const conv = loadFlight(file, { glider: "log" });
  if (!conv.track.n || !conv.track.cols.te_alt.some(Number.isFinite)) return { file: path.basename(file), skipped: "no TE pressure" };
  const rep = await runReplay(M, conv.rec, {});
  if (!rep.n) return { file: path.basename(file), skipped: "no replay" };
  globalThis.fileName = path.basename(file);
  globalThis.D = build(conv, rep);
  derive(D); // deriveDamping at page.js's damping of 4 s
  const c = D.tr, r = D.rep.cols, n = D.n;
  const { free } = freeMask(D, (k) => c.ias[k] > 60 / 3.6 && c.gnss_speed[k] > 10);
  const ok = Uint8Array.from(free, (v, k) => v && D.amOk[k] && Number.isFinite(D.netto[k]) && Number.isFinite(D.tep[k]) && Number.isFinite(D.gnssNetInert[k]));
  const source = { AirMass: dampSeries(D.amRaw, ok), TE: D.netto, TEp: D.tep, inertial: D.gnssNetInert, pitot: D.gnssNetPitot };
  // a 10 Hz grid, interpolating only over short measured intervals (batch.js)
  const N = Math.floor((D.t[n - 1] - D.t[0]) * 10) + 1, g = {};
  for (const s in source) g[s] = new Float32Array(N).fill(NaN);
  const gOk = new Uint8Array(N), roll = new Float32Array(N);
  for (let j = 0, a = 0; j < N; j++) {
    const t = D.t[0] + j / 10;
    while (a + 1 < n && D.t[a + 1] <= t) a++;
    if (a + 1 >= n || D.t[a + 1] - D.t[a] > 0.3 || !ok[a] || !ok[a + 1]) continue;
    const f = (t - D.t[a]) / (D.t[a + 1] - D.t[a]);
    for (const s in source) g[s][j] = source[s][a] + f * (source[s][a + 1] - source[s][a]);
    gOk[j] = 1; roll[j] = Math.abs(r.roll_deg[D.ai[a]]);
  }
  const out = { file: path.basename(file), glider: conv.glider, hours: gOk.reduce((s, v) => s + v, 0) / 36000, refs: {} };
  // the 1 s needle movement as shown, rms m/s
  out.motion = {};
  for (const s of SERIES) {
    let s2 = 0, m = 0;
    for (let k = 0; k + 10 < N; k += 5) if (gOk[k] && gOk[k + 10]) { s2 += (g[s][k + 10] - g[s][k]) ** 2; m++; }
    out.motion[s] = Math.sqrt(s2 / m);
  }
  for (const ref of REFS) {
    const R = g[ref], mask = Uint8Array.from(gOk, (v, k) => v && Number.isFinite(R[k]));
    const lag = Object.fromEntries(SERIES.map((s) => [s, lagOf(g[s], R, mask)]));
    const res = (out.refs[ref] = { delay: Object.fromEntries(SERIES.map((s) => [s, lag[s] / 10])), sums: {} });
    // per window: each series' mean minus the reference's; sums of squares per phase, and of the pairwise differences
    for (const W of WINDOWS) {
      const L = W * 10;
      for (let k = 0; k + L + 60 < N; k += L) {
        let good = true, bank = 0, sr = 0;
        const sum = { AirMass: 0, TE: 0, TEp: 0 };
        for (let j = k; j < k + L && good; j++) {
          if (!mask[j] || SERIES.some((s) => !mask[j + lag[s]])) { good = false; break; }
          bank += roll[j]; sr += R[j];
          for (const s of SERIES) sum[s] += g[s][j + lag[s]];
        }
        if (!good) continue;
        bank /= L;
        const e = Object.fromEntries(SERIES.map((s) => [s, (sum[s] - sr) / L]));
        const terms = { ...e, "AirMass-TE": e.AirMass - e.TE, "AirMass-TEp": e.AirMass - e.TEp, "TE-TEp": e.TE - e.TEp };
        for (const ph of ["all", bank < 10 ? "straight" : bank > 25 ? "circling" : null]) {
          if (!ph) continue;
          const acc = ((res.sums[W] ||= {})[ph] ||= { n: 0 });
          acc.n++;
          for (const [name, v] of Object.entries(terms)) { acc[name] = (acc[name] || 0) + v * v; acc["bias " + name] = (acc["bias " + name] || 0) + v; }
        }
      }
    }
  }
  return out;
}

// rms of a term over pooled sums, the three-cornered hat's own errors
const rms = (a, name) => (a && a.n ? Math.sqrt(a[name] / a.n) : NaN);
const hat = (a, x, y) => {
  // x, y and the reference: own variance of x = (V(x-ref) + V(x-y) - V(y-ref)) / 2, of the reference = (V(x-ref) + V(y-ref) - V(x-y)) / 2
  const v = (name) => a[name] / a.n, d = v(`${x}-${y}`) ?? NaN;
  const sq = (u) => (u > 0 ? Math.sqrt(u) : -Math.sqrt(-u));
  return { [x]: sq((v(x) + d - v(y)) / 2), [y]: sq((v(y) + d - v(x)) / 2), ref: sq((v(x) + v(y) - d) / 2) };
};
function pool(rows) {
  const P = {};
  for (const row of rows) for (const ref of REFS) for (const W of WINDOWS) for (const ph of PHASES) {
    const a = row.refs[ref].sums[W]?.[ph];
    if (!a) continue;
    const p = (((P[ref] ||= {})[W] ||= {})[ph] ||= { n: 0 });
    for (const k in a) p[k] = (p[k] || 0) + a[k];
  }
  return P;
}
const f3 = (v) => (Number.isFinite(v) ? v.toFixed(3) : "–");
const median = (v) => { const a = v.filter(Number.isFinite).sort((x, y) => x - y); return a.length ? a[Math.floor((a.length - 1) / 2)] : NaN; };

function report(rows) {
  const scored = rows.filter((r) => !r.skipped);
  console.log("\nper flight, gust-insensitive reference, 10 s windows, rms m/s (TE-TEp: the two TE varios against each other)");
  console.log("flight".padEnd(30) + "glider".padEnd(18) + "h     AirMass  TE      TEp     TE-TEp  delay AM/TE/TEp s  1 s motion AM/TE/TEp");
  for (const r of scored) {
    const a = r.refs.inertial.sums[10]?.all, d = r.refs.inertial.delay;
    console.log((exclude.has(r.file) ? "(" + r.file + ")" : r.file).padEnd(30) + String(r.glider).padEnd(18) + r.hours.toFixed(1).padEnd(6) +
      ["AirMass", "TE", "TEp", "TE-TEp"].map((s) => f3(rms(a, s)).padEnd(8)).join("") +
      `${d.AirMass.toFixed(1)}/${d.TE.toFixed(1)}/${d.TEp.toFixed(1)}`.padEnd(19) + SERIES.map((s) => f3(r.motion[s])).join("/"));
  }
  for (const r of rows.filter((r) => r.skipped)) console.log(`${r.file}: ${r.skipped}`);
  const pooled = scored.filter((r) => !exclude.has(r.file));
  const P = pool(pooled);
  console.log(`\npooled over ${pooled.length} flights (${pooled.reduce((s, r) => s + r.hours, 0).toFixed(1)} h scored)` + (exclude.size ? `, without ${[...exclude].join(", ")}` : ""));
  for (const ref of REFS) {
    console.log(`\n${ref === "inertial" ? "gust-insensitive" : "pitot"} reference: rms (bias) m/s against it; own error by the three-cornered hat with AirMass`);
    for (const W of WINDOWS) for (const ph of PHASES) {
      const a = P[ref]?.[W]?.[ph];
      if (!a) continue;
      const h1 = hat(a, "AirMass", "TE"), h2 = hat(a, "AirMass", "TEp");
      console.log(`  ${String(W).padStart(2)} s ${ph.padEnd(8)} [${String(a.n).padStart(5)}]  ` +
        SERIES.map((s) => `${s} ${f3(rms(a, s))} (${(a["bias " + s] / a.n >= 0 ? "+" : "") + (a["bias " + s] / a.n).toFixed(2)})`).join("  ") +
        `  TE-TEp ${f3(rms(a, "TE-TEp"))}  |  own: AirMass ${f3(h1.AirMass)}/${f3(h2.AirMass)}, TE ${f3(h1.TE)}, TEp ${f3(h2.TEp)}, ref ${f3(h1.ref)}/${f3(h2.ref)}`);
    }
  }
  console.log("\nmedian delay to the gust-insensitive reference, s: " + SERIES.map((s) => `${s} ${median(pooled.map((r) => r.refs.inertial.delay[s])).toFixed(1)}`).join(", ") +
    "\nmedian 1 s needle movement, m/s: " + SERIES.map((s) => `${s} ${f3(median(pooled.map((r) => r.motion[s])))}`).join(", "));
}

(async () => {
  const [outFile, ...logs] = process.argv.slice(2);
  if (outFile.endsWith(".json") && fs.existsSync(outFile) && !logs.length) return report(JSON.parse(fs.readFileSync(outFile, "utf8")).flights);
  const M = await require(path.join(web, "airmass.js"))();
  const output = { created: new Date().toISOString(), flights: [] };
  for (const log of logs) {
    const row = await evaluate(M, log);
    output.flights.push(row);
    fs.writeFileSync(outFile, JSON.stringify(output) + "\n");
    console.error(row.file, row.skipped || `${row.hours.toFixed(1)} h`);
  }
  report(output.flights);
})().catch((e) => { console.error(e); process.exitCode = 1; });
