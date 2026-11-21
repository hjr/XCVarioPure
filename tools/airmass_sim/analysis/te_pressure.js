// AirMass netto and the TE netto of the compensation methods replayed from the $SENS data, against the GNSS
// references, per flight and pooled: eComp (the page's, VarioFilter::postProcess with electronic compensation,
// adjust 92 %), TEp (the TE probe pressure through TeVario), TEK (the page's ImuTek, XCVarioProModules) and, as a
// control for eComp, TEK0 (ImuTek without its IMU term). The vario logged in $PXCV is not scored: it is whatever
// method the device showed.
//   node te_pressure.js OUTPUT.json LOG...      (EXCLUDE=file,...: logs left out of the pools)
// COMP: the TE compensation setting of eComp, the IMU TEK and AirMass's setup (half its deviation): "tuned" each
// installation's (TUNED_SETTING in page.js), a number for all; unset eComp and the IMU TEK at 92 and AirMass at the
// setup's (the device's, 100 for $SENS logs without one).
// All at the same 4 s damping (the page's Damp, not the single 4 s low pass the IMU TEK's firmware damps with) and
// each shifted by its fitted delay, like batch.js; the delays and the 1 s needle movement as shown. Own errors by the
// three-cornered hat of AirMass, each other vario and the reference. Windows by phase: straight (bank under 10°),
// circling (over 25°) and speed change (the kinetic energy height TAS²/2g changing by more than 15 m over the
// window, where the TE compensation matters most).
const fs = require("fs"), path = require("path");
const { loadPage, lagOf, dampSeries, freeMask, withSetting, installation, TUNED_SETTING } = require("./page.js");
const web = path.resolve(__dirname, "../web");
const { runReplay, loadFlight } = loadPage(web);
const SERIES = ["AirMass", "eComp", "TEp", "TEK", "TEK0"], FALSE_CLIMBS = [[1, 1], [2, 2]], OTHERS = SERIES.slice(1), REFS = ["inertial", "pitot"], WINDOWS = [10, 30], PHASES = ["all", "straight", "circling", "speed change"], SPEED_CHANGE = 15;
const exclude = new Set((process.env.EXCLUDE || "").split(",").filter(Boolean));

async function evaluate(M, file) {
  const conv = loadFlight(file, { glider: "log" });
  if (!conv.track.n || !conv.track.cols.te_alt.some(Number.isFinite)) return { file: path.basename(file), skipped: "no TE pressure" };
  const comp = process.env.COMP, name = path.basename(file);
  const setting = comp === "tuned" ? TUNED_SETTING[installation(name, conv.glider)] : comp ? +comp : NaN;
  if (comp && !Number.isFinite(setting)) return { file: name, skipped: `no setting for ${installation(name, conv.glider)}` };
  const rep = await runReplay(M, Number.isFinite(setting) ? withSetting(conv.rec, setting) : conv.rec, {});
  if (!rep.n) return { file: name, skipped: "no replay" };
  globalThis.fileName = name;
  globalThis.D = build(conv, rep);
  const page$ = globalThis.$, adjust = Number.isFinite(setting) ? setting : 92;
  globalThis.$ = (id) => (id === "ecomp" ? { value: String(adjust) } : page$(id));
  derive(D); // deriveDamping at page.js's damping of 4 s
  globalThis.$ = page$;
  const c = D.tr, r = D.rep.cols, n = D.n;
  const tekNetto = (imu) => {
    const f = new ImuTek(0, adjust / 100), damp = new Damp(4), out = new Float32Array(n).fill(NaN);
    for (let k = 0; k < n; k++) {
      const pitch = imu && D.ai[k] >= 0 ? r.pitch_deg[D.ai[k]] * Math.PI / 180 : NaN;
      const raw = f.at(D.t[k], { p: c.p_static[k], q: c.q_dyn[k], ax: c.acc_x[k], pitch });
      if (!Number.isFinite(raw)) { damp.reset(NaN); continue; }
      out[k] = damp.at(D.t[k], raw) - D.sinkUp[k];
    }
    return out;
  };
  const tek = tekNetto(true), tek0 = tekNetto(false);
  const ecomp = D.netto;
  const { free } = freeMask(D, (k) => c.ias[k] > 60 / 3.6 && c.gnss_speed[k] > 10);
  const ok = Uint8Array.from(free, (v, k) => v && D.amOk[k] && Number.isFinite(D.tep[k]) && Number.isFinite(tek[k]) && Number.isFinite(ecomp[k]) && Number.isFinite(D.gnssNetInert[k]));
  const source = { AirMass: dampSeries(D.amRaw, ok), eComp: ecomp, TEp: D.tep, TEK: tek, TEK0: tek0, inertial: D.gnssNetInert, pitot: D.gnssNetPitot };
  // a 10 Hz grid, interpolating only over short measured intervals (batch.js)
  const N = Math.floor((D.t[n - 1] - D.t[0]) * 10) + 1, g = {};
  for (const s in source) g[s] = new Float32Array(N).fill(NaN);
  const gOk = new Uint8Array(N), roll = new Float32Array(N), ekin = new Float32Array(N).fill(NaN);
  for (let j = 0, a = 0; j < N; j++) {
    const t = D.t[0] + j / 10;
    while (a + 1 < n && D.t[a + 1] <= t) a++;
    if (a + 1 >= n || D.t[a + 1] - D.t[a] > 0.3 || !ok[a] || !ok[a + 1]) continue;
    const f = (t - D.t[a]) / (D.t[a + 1] - D.t[a]);
    for (const s in source) g[s][j] = source[s][a] + f * (source[s][a + 1] - source[s][a]);
    gOk[j] = 1; roll[j] = Math.abs(r.roll_deg[D.ai[a]]);
    const tas = c.tas[a] + f * (c.tas[a + 1] - c.tas[a]);
    ekin[j] = tas * tas / (2 * 9.80665);
  }
  const out = { file: path.basename(file), glider: conv.glider, setting: Number.isFinite(setting) ? setting : null, hours: gOk.reduce((s, v) => s + v, 0) / 36000, refs: {} };
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
    // false climbs in straight flight as batch.js counts them: bank under 10° for 8 s, the needle as shown (not
    // shifted) a threshold or more above the reference's centred 2 s mean for a time, again only after it came back
    // within half the threshold; false sinks the same below it, for what the reference and the air alone give
    const refMean = new Float32Array(N).fill(NaN);
    for (let k = 10; k + 10 < N; k++) {
      let s = 0, m = 0;
      for (let j = k - 10; j <= k + 10; j++) if (mask[j]) { s += R[j]; m++; }
      if (m >= 15) refMean[k] = s / m;
    }
    const fc = (res.falseClimbs = { straightHours: 0 });
    for (const [thr, dur] of FALSE_CLIMBS) for (const sign of [1, -1]) for (const s of SERIES) {
      let events = 0, above = 0, counted = false, level = 0, seconds = 0;
      for (let k = 0; k < N; k++) {
        level = mask[k] && roll[k] < 10 ? level + 1 : 0;
        const err = sign * (g[s][k] - refMean[k]);
        if (level < 80 || !Number.isFinite(err)) { above = 0; counted = false; continue; }
        seconds += 0.1;
        if (err > thr) { above += 0.1; if (above >= dur - 1e-6 && !counted) { events++; counted = true; } }
        else if (err < thr / 2) { above = 0; counted = false; }
      }
      fc[`${sign > 0 ? "climb" : "sink"}${thr} ${s}`] = events;
      fc.straightHours = seconds / 3600;
    }
    // per window: each series' mean minus the reference's; sums of squares per phase, and of the pairwise differences
    for (const W of WINDOWS) {
      const L = W * 10;
      for (let k = 0; k + L + 60 < N; k += L) {
        let good = true, bank = 0, sr = 0;
        const sum = Object.fromEntries(SERIES.map((s) => [s, 0]));
        for (let j = k; j < k + L && good; j++) {
          if (!mask[j] || SERIES.some((s) => !mask[j + lag[s]])) { good = false; break; }
          bank += roll[j]; sr += R[j];
          for (const s of SERIES) sum[s] += g[s][j + lag[s]];
        }
        if (!good) continue;
        bank /= L;
        const e = Object.fromEntries(SERIES.map((s) => [s, (sum[s] - sr) / L]));
        const terms = { ...e };
        for (let i = 0; i < SERIES.length; i++) for (let j = i + 1; j < SERIES.length; j++) terms[`${SERIES[i]}-${SERIES[j]}`] = e[SERIES[i]] - e[SERIES[j]];
        const speedChange = Math.abs(ekin[k + L - 1] - ekin[k]) > SPEED_CHANGE ? "speed change" : null;
        for (const ph of ["all", bank < 10 ? "straight" : bank > 25 ? "circling" : null, speedChange]) {
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
  console.log("\nper flight, gust-insensitive reference, 10 s windows, rms m/s (TEp: TE probe; TEK: IMU TEK, TEK0: without its IMU term, the control eComp-TEK0)");
  console.log("flight".padEnd(30) + "glider".padEnd(18) + "h     " + [...SERIES, "eComp-TEK0"].map((s) => s.padEnd(11)).join("") + `delay ${SERIES.join("/")} s`.padEnd(32) + `1 s motion ${SERIES.join("/")}`.padEnd(48) + `false climbs 2 m/s /h ${SERIES.join("/")}`);
  for (const r of scored) {
    const a = r.refs.inertial.sums[10]?.all, d = r.refs.inertial.delay;
    console.log((exclude.has(r.file) ? "(" + r.file + ")" : r.file).padEnd(30) + String(r.glider).padEnd(18) + r.hours.toFixed(1).padEnd(6) +
      [...SERIES, "eComp-TEK0"].map((s) => f3(rms(a, s)).padEnd(11)).join("") +
      SERIES.map((s) => d[s].toFixed(1)).join("/").padEnd(32) + SERIES.map((s) => f3(r.motion[s])).join("/").padEnd(48) +
      SERIES.map((s) => (r.refs.inertial.falseClimbs[`climb2 ${s}`] / r.refs.inertial.falseClimbs.straightHours).toFixed(1)).join("/"));
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
      const hats = OTHERS.map((s) => hat(a, "AirMass", s));
      console.log(`  ${String(W).padStart(2)} s ${ph.padEnd(8)} [${String(a.n).padStart(5)}]  ` +
        SERIES.map((s) => `${s} ${f3(rms(a, s))} (${(a["bias " + s] / a.n >= 0 ? "+" : "") + (a["bias " + s] / a.n).toFixed(2)})`).join("  ") +
        `  eComp-TEK0 ${f3(rms(a, "eComp-TEK0"))}  eComp-TEp ${f3(rms(a, "eComp-TEp"))}  TEK-TEK0 ${f3(rms(a, "TEK-TEK0"))}` +
        `\n${" ".repeat(25)}own: AirMass ${hats.map((h) => f3(h.AirMass)).join("/")}, ` + OTHERS.map((s, i) => `${s} ${f3(hats[i][s])}`).join(", ") + `, ref ${hats.map((h) => f3(h.ref)).join("/")}`);
    }
  }
  for (const ref of REFS) {
    const hours = pooled.reduce((h, r) => h + r.refs[ref].falseClimbs.straightHours, 0);
    console.log(`\nfalse climbs (sinks) per hour of straight flight, ${hours.toFixed(1)} h, ${ref === "inertial" ? "gust-insensitive" : "pitot"} reference:`);
    for (const [thr, dur] of FALSE_CLIMBS) {
      const rate = (kind, s) => (pooled.reduce((n, r) => n + r.refs[ref].falseClimbs[`${kind}${thr} ${s}`], 0) / hours).toFixed(1);
      console.log(`  ${thr} m/s ${dur} s  ` + SERIES.map((s) => `${s} ${rate("climb", s)} (${rate("sink", s)})`).join("  "));
    }
  }
  console.log("\nmedian delay to the gust-insensitive reference, s: " + SERIES.map((s) => `${s} ${median(pooled.map((r) => r.refs.inertial.delay[s])).toFixed(1)}`).join(", ") +
    "\nmedian 1 s needle movement, m/s: " + SERIES.map((s) => `${s} ${f3(median(pooled.map((r) => r.motion[s])))}`).join(", "));
  combined(pooled, P);
}

// all of the above in one markdown table, gust-insensitive reference; the best of each row in bold (TEK0, the
// control, not ranked). AirMass's own error is a range, one per hat with each other vario (ranked by its top); a
// negative variance from the hat shows as ≈0.
function combined(pooled, P) {
  const NAMES = { AirMass: "AirMass", eComp: "eComp", TEp: "TE pressure", TEK: "IMU TEK", TEK0: "TEK0 (control)" }, RANKED = SERIES.slice(0, -1);
  const fc = (kind, thr) => {
    const hours = pooled.reduce((h, r) => h + r.refs.inertial.falseClimbs.straightHours, 0);
    return Object.fromEntries(SERIES.map((s) => [s, pooled.reduce((n, r) => n + r.refs.inertial.falseClimbs[`${kind}${thr} ${s}`], 0) / hours]));
  };
  const own = (W, ph) => {
    const a = P.inertial[W][ph], hats = OTHERS.map((s) => hat(a, "AirMass", s)), am = hats.map((h) => h.AirMass);
    return { ...Object.fromEntries(OTHERS.map((s, i) => [s, hats[i][s]])), AirMass: [Math.min(...am), Math.max(...am)] };
  };
  const wins = Object.fromEntries(SERIES.map((s) => [s, 0]));
  for (const r of pooled) {
    const a = r.refs.inertial.sums[10]?.all;
    if (!a) continue;
    const best = Math.min(...RANKED.map((s) => rms(a, s)));
    for (const s of RANKED) if (rms(a, s) === best) wins[s]++;
  }
  const rows = [];
  const row = (label, values, fmt, lowBest = true) => {
    const key = (v) => (Array.isArray(v) ? v[1] : v), ranked = RANKED.map((s) => key(values[s])).filter(Number.isFinite);
    const best = lowBest ? Math.min(...ranked) : Math.max(...ranked);
    rows.push(`| ${label} | ` + SERIES.map((s) => { const t = fmt(values[s]); return RANKED.includes(s) && key(values[s]) === best ? `**${t}**` : t; }).join(" | ") + " |");
  };
  for (const W of WINDOWS) for (const ph of PHASES) {
    const a = P.inertial[W][ph];
    row(`rms ${W} s ${ph}, m/s`, Object.fromEntries(SERIES.map((s) => [s, rms(a, s)])), f3);
  }
  for (const ph of PHASES) {
    const o = own(10, ph);
    row(`own error 10 s ${ph} (hat), m/s`, o, (v) => { const g = (x) => (x < 0 ? "≈0" : f3(x)); return Array.isArray(v) ? `${g(v[0])}–${g(v[1])}` : g(v); });
  }
  row("median delay, s", Object.fromEntries(SERIES.map((s) => [s, median(pooled.map((r) => r.refs.inertial.delay[s]))])), (v) => v.toFixed(1));
  row("median needle movement in 1 s, m/s", Object.fromEntries(SERIES.map((s) => [s, median(pooled.map((r) => r.motion[s]))])), f3);
  for (const [thr, dur] of FALSE_CLIMBS.slice().reverse()) {
    row(`false climbs ${thr} m/s ${dur} s, per h straight`, fc("climb", thr), (v) => v.toFixed(1));
    row(`false sinks ${thr} m/s ${dur} s, per h straight`, fc("sink", thr), (v) => v.toFixed(1));
  }
  row("flights best at 10 s", wins, String, false);
  const hours = pooled.reduce((s, r) => s + r.hours, 0), straight = pooled.reduce((h, r) => h + r.refs.inertial.falseClimbs.straightHours, 0);
  console.log(`\ncombined, ${pooled.length} flights, ${hours.toFixed(1)} h scored (${straight.toFixed(1)} h straight for the false climbs), gust-insensitive reference\n`);
  console.log("| | " + SERIES.map((s) => NAMES[s]).join(" | ") + " |\n|---|" + SERIES.map(() => "---|").join(""));
  console.log(rows.join("\n"));
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
