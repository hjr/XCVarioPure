// The TE compensation (te_comp_adjust) for eComp and for AirMass (its static_comp) against the gust-insensitive GNSS
// reference, per flight and pooled.
//   node te_adjust.js OUTPUT.json LOG...      (EXCLUDE=file,...: logs left out of the pools; OUTPUT.json alone re-reports)
// AIRMASS=100,96,... and ECOMP=84,88,... set the values (the first AIRMASS one gives the reference).
// AirMass replayed with each AIRMASS value as its static_comp (the setup's $PXCVC setting 2·value − 100, as the
// pipeline takes half the setting's deviation since 2026-11-21), eComp (the page's formula through TeVario) with each
// ECOMP value; all through Damp(4), shifted by the fitted delay. Windows of 10 s, all, straight, circling and "speed
// change" (the kinetic energy height TAS²/2g changing by more than 15 m over the window, where a wrong compensation
// shows); false climbs of 2 m/s for 2 s in straight flight as batch.js counts them.
const fs = require("fs"), path = require("path");
const { loadPage, lagOf, dampSeries, freeMask, withSetting } = require("./page.js");
const web = path.resolve(__dirname, "../web");
const { runReplay, loadFlight } = loadPage(web);
const list = (text, dflt) => (text ? text.split(",").map(Number) : dflt);
const AIRMASS = list(process.env.AIRMASS, [100, 96, 90]), ECOMP = list(process.env.ECOMP, [84, 88, 90, 92, 96, 100, 104]), PHASES = ["all", "straight", "circling", "speed change"];
const exclude = new Set((process.env.EXCLUDE || "").split(",").filter(Boolean));
const G = 9.80665, isa = (p) => (p > 0 ? 44330.76923 * (1 - Math.pow(p / 101325, 1 / 5.25588)) : NaN);

// the recording with the setup's setting giving AirMass this static_comp [%]
const withAdjust = (rec, staticComp) => withSetting(rec, 2 * staticComp - 100);

async function evaluate(M, file) {
  const conv = loadFlight(file, { glider: "log" });
  if (!conv.track.n) return { file: path.basename(file), skipped: "no samples" };
  const series = {}, runs = {};
  // AIRMASS[0] last: its D gives the reference (AirMass's averaged wind is in it) for every run
  for (const a of [...AIRMASS.slice(1), AIRMASS[0]]) {
    const rep = await runReplay(M, withAdjust(conv.rec, a), {});
    if (!rep.n) return { file: path.basename(file), skipped: "no replay" };
    globalThis.D = build(conv, rep);
    derive(D);
    const r = D.rep.cols, valid = r.status.filter((st) => st & STATUS.WIND_VALID).length;
    runs[a] = { amRaw: D.amRaw, amOk: D.amOk, windValid: valid / r.status.length };
  }
  // D of the last run (AIRMASS[0]): the reference, the sink and the track for every run
  const c = D.tr, n = D.n, ref = D.gnssNetInert, roll = Float32Array.from({ length: n }, (_, k) => (D.ai[k] >= 0 ? Math.abs(D.rep.cols.roll_deg[D.ai[k]]) : NaN));
  const { free } = freeMask(D, (k) => c.ias[k] > 60 / 3.6 && c.gnss_speed[k] > 10);
  const ok = Uint8Array.from(free, (v, k) => v && AIRMASS.every((a) => runs[a].amOk[k]) && Math.abs(ref[k]) < 20); // a GNSS altitude glitch: not a reference
  for (const a of AIRMASS) series[`AirMass ${a}`] = dampSeries(runs[a].amRaw, ok);
  for (const e of ECOMP) {
    const te = new TeVario(4), adj = e / 100, out = new Float32Array(n).fill(NaN);
    for (let k = 0; k < n; k++) {
      const p = c.p_static[k], q = c.q_dyn[k], v = c.tas[k];
      out[k] = te.at(D.t[k], q >= 0 ? (isa(p) + v * v / (2 * G) * adj + isa(p - q * adj)) / 2 : NaN) - D.sinkUp[k];
    }
    series[`eComp ${e}`] = out;
  }
  const names = Object.keys(series);
  for (let k = 0; k < n; k++) if (ok[k] && !names.every((s) => Number.isFinite(series[s][k]))) ok[k] = 0;
  const out = { file: path.basename(file), glider: conv.glider, sums: {}, delay: {}, falseClimbs: { straightHours: 0 },
    windValid: Object.fromEntries(AIRMASS.map((a) => [a, runs[a].windValid])) };
  const lag = Object.fromEntries(names.map((s) => [s, lagOf(series[s], ref, ok)]));
  for (const s of names) out.delay[s] = lag[s] / 10;
  const L = 100;
  for (let k = 0; k + L + 60 < n; k += L) {
    let good = true, bank = 0, sr = 0;
    const sum = Object.fromEntries(names.map((s) => [s, 0]));
    for (let j = k; j < k + L; j++) {
      if (!ok[j] || D.t[j + 1] - D.t[j] > 0.3 || names.some((s) => !ok[j + lag[s]] || Math.abs(D.t[j + lag[s]] - D.t[j] - lag[s] / 10) > 0.15)) { good = false; break; }
      bank += roll[j]; sr += ref[j];
      for (const s of names) sum[s] += series[s][j + lag[s]];
    }
    if (!good) continue;
    bank /= L;
    const ekin = (j) => c.tas[j] * c.tas[j] / (2 * G);
    const phases = ["all", bank < 10 ? "straight" : bank > 25 ? "circling" : null, Math.abs(ekin(k + L) - ekin(k)) > 15 ? "speed change" : null];
    for (const ph of phases) {
      if (!ph) continue;
      const acc = (out.sums[ph] ||= { n: 0 });
      acc.n++;
      for (const s of names) acc[s] = (acc[s] || 0) + ((sum[s] - sr) / L) ** 2;
    }
  }
  // false climbs, 2 m/s for 2 s, in straight flight (batch.js)
  const refMean = new Float32Array(n).fill(NaN);
  for (let k = 10; k + 10 < n; k++) {
    let s = 0, m = 0;
    for (let j = k - 10; j <= k + 10; j++) if (ok[j]) { s += ref[j]; m++; }
    if (m >= 15) refMean[k] = s / m;
  }
  for (const s of names) {
    let events = 0, above = 0, counted = false, level = 0, seconds = 0;
    for (let k = 0; k < n; k++) {
      level = ok[k] && roll[k] < 10 ? level + 1 : 0;
      const err = series[s][k] - refMean[k];
      if (level < 80 || !Number.isFinite(err)) { above = 0; counted = false; continue; }
      seconds += 0.1;
      if (err > 2) { above += 0.1; if (above >= 2 - 1e-6 && !counted) { events++; counted = true; } }
      else if (err < 1) { above = 0; counted = false; }
    }
    out.falseClimbs[s] = events;
    out.falseClimbs.straightHours = seconds / 3600;
  }
  return out;
}

function report(rows) {
  const scored = rows.filter((r) => !r.skipped && r.sums.all), pooled = scored.filter((r) => !exclude.has(r.file));
  const names = Object.keys(pooled[0].delay), rms = (a, s) => (a && a.n ? Math.sqrt(a[s] / a.n) : NaN);
  console.log("per flight, 10 s rms all / speed change, m/s: best AirMass, best eComp adjust");
  for (const r of scored) {
    const best = (prefix, ph) => names.filter((s) => s.startsWith(prefix)).reduce((b, s) => (rms(r.sums[ph], s) < rms(r.sums[ph], b) ? s : b)).split(" ")[1];
    console.log((exclude.has(r.file) ? `(${r.file})` : r.file).padEnd(30) + String(r.glider).padEnd(18) +
      `AirMass ${best("AirMass", "all")} / ${r.sums["speed change"] ? best("AirMass", "speed change") : "–"}   eComp ${best("eComp", "all")} / ${r.sums["speed change"] ? best("eComp", "speed change") : "–"}`);
  }
  const P = {};
  for (const r of pooled) for (const ph of PHASES) { const a = r.sums[ph]; if (!a) continue; const p = (P[ph] ||= { n: 0 }); for (const k in a) p[k] = (p[k] || 0) + a[k]; }
  const hours = pooled.reduce((h, r) => h + r.falseClimbs.straightHours, 0);
  console.log(`\npooled over ${pooled.length} flights, gust-insensitive reference, 10 s windows, rms m/s; false climbs 2 m/s 2 s per h straight (${hours.toFixed(1)} h)\n`);
  console.log("| | " + PHASES.map((ph) => `${ph} [${P[ph]?.n ?? 0}]`).join(" | ") + " | false climbs | delay, s |\n|---|" + PHASES.map(() => "---|").join("") + "---|---|");
  for (const s of names) {
    const fc = pooled.reduce((m, r) => m + r.falseClimbs[s], 0) / hours;
    const delays = pooled.map((r) => r.delay[s]).sort((a, b) => a - b);
    console.log(`| ${s} | ` + PHASES.map((ph) => rms(P[ph], s).toFixed(3)).join(" | ") + ` | ${fc.toFixed(1)} | ${delays[Math.floor((delays.length - 1) / 2)].toFixed(1)} |`);
  }
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
    console.error(row.file, row.skipped || "");
  }
  report(output.flights);
})().catch((e) => { console.error(e); process.exitCode = 1; });
