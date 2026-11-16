// AirMass's wind against XCSoar's zigzag wind (WindEKF with its WindEKFGlue and the circling detection it depends on,
// ported from XCSoar master, GPL-2.0-or-later, src/Computer/Wind and src/Computer/CirclingComputer.cpp), both against
// the wind from GNSS alone (page.js gnssCircleWind: a circle fit per full turn, neither heading nor airspeed).
// While circling: each estimate at the end of each turn against that turn's fit. In straight flight: against the
// fits interpolated in time between the thermal before and the one after, by the time since leaving the thermal.
// XCSoar is fed what XCVario gives it: each GNSS fix (track and ground speed) at its arrival with the TAS of that moment
// and the load factor of the bank angle (XCVario's $PXCV acceleration).
//   node xcsoar_wind.js OUT.json log1.nmea.gz ...   (SETTINGS="name=value,...": AirMass settings; OUT.json alone re-reports)
const fs = require("fs"), path = require("path");
const { loadPage, freeMask, gnssCircleWind } = require("./page.js");
const web = path.resolve(__dirname, "../web");
const { runReplay, loadFlight } = loadPage(web);
const D2R = Math.PI / 180;

class WindEKF {
  init() { this.k = 1e-2 * 4; this.X = [0, 0, 1]; }
  // gps east, north [m/s]; X: the air mass velocity east, north and the airspeed scale
  update(airspeed, ge, gn) {
    const X = this.X, dx = ge - X[0], dy = gn - X[1], mag = Math.hypot(dx, dy);
    const K = [-X[2] * dx / mag * this.k, -X[2] * dy / mag * this.k, mag * 1e-5];
    this.k += 0.01 * (1e-2 - this.k);
    const err = airspeed - X[2] * mag;
    for (let i = 0; i < 3; i++) X[i] += K[i] * err;
    X[2] = Math.min(1.5, Math.max(0.5, X[2]));
  }
}

// XCSoar's wind computer with an airspeed: per GNSS fix, the turn rate and circling mode (CirclingComputer, default
// 15 s / 10 s switch thresholds, no flight mode switch), WindEKFGlue and its output every 10th sample
class XCSoarWind {
  constructor(vTakeoff) {
    this.vTakeoff = vTakeoff; this.ekf = new WindEKF();
    this.resetPending = true; this.i = 0; this.blackout = -Infinity;
    this.lastTrack = NaN; this.lastTime = NaN; this.turnRate = 0; this.smoothed = 0;
    this.mode = "cruise"; this.modeStart = 0;
    this.out = null;
  }
  get circling() { return this.mode === "climb" || this.mode === "possible cruise"; }
  // utc: GNSS time [s], clock: device time [s], track [deg], speed [m/s], tas [m/s], g: load factor
  fix(utc, clock, track, speed, tas, g) {
    this.turning(utc, clock, track);
    if (!Number.isFinite(track) || !Number.isFinite(speed) || !(tas >= 1)) { this.blackout = -Infinity; return; }
    if (!(tas > this.vTakeoff)) return;
    if (this.circling) this.i = 0;
    if (Math.abs(this.turnRate) > 20 || Math.abs(g - 1) > 0.3) { this.blackout = clock + 3; return; }
    if (clock < this.blackout) return;
    this.blackout = -Infinity;
    if (this.resetPending) { this.resetPending = false; this.ekf.init(); }
    this.ekf.update(tas, speed * Math.sin(track * D2R), speed * Math.cos(track * D2R));
    if (++this.i % 10 !== 0) return;
    this.out = { n: this.ekf.X[1], e: this.ekf.X[0] };
  }
  // the mode timers on the device clock, which does not wrap at midnight
  turning(utc, clock, track) {
    if (!Number.isFinite(this.lastTime) || !Number.isFinite(this.lastTrack)) { this.lastTime = utc; this.lastTrack = track; return; }
    let dt = utc - this.lastTime;
    if (dt < -80000) dt += 86400;
    if (dt < 0 || dt > 10) { this.turnRate = this.smoothed = 0; this.lastTime = utc; this.lastTrack = track; return; }
    if (dt < 1 / 3 || !Number.isFinite(track)) return;
    this.turnRate = ((((track - this.lastTrack) % 360) + 540) % 360 - 180) / dt;
    this.smoothed += (Math.min(50, Math.max(-50, this.turnRate)) - this.smoothed) * 0.3;
    this.lastTime = utc; this.lastTrack = track;
    const turning = Math.abs(this.smoothed) >= 4;
    if (this.mode === "cruise" && turning) { this.mode = "possible climb"; this.modeStart = clock; }
    else if (this.mode === "possible climb") {
      if (!turning) this.mode = "cruise";
      else if (clock - this.modeStart > 15) this.mode = "climb";
    } else if (this.mode === "climb" && !turning) { this.mode = "possible cruise"; this.modeStart = clock; }
    else if (this.mode === "possible cruise") {
      if (turning) this.mode = "climb";
      else if (clock - this.modeStart > 10) this.mode = "cruise";
    }
  }
}

const SERIES = ["AirMass nav", "AirMass instant", "AirMass 100 s", "XCSoar", "held thermal wind"];
const SI = Object.fromEntries(SERIES.map((s, i) => [s, i]));
// a sample's error [along, across] of a series; stored per series index, to the cm/s
const E = (x, s) => x.e[SI[s]];
const r2 = (v) => Math.round(v * 100) / 100;
const BINS = [[0.5, 2], [2, 5], [5, 10], [10, 20]]; // minutes since the last turn of the thermal before
const MAX_GAP = 20 * 60; // s between the thermals the straight reference is interpolated from

function evaluate(D) {
  const c = D.tr, r = D.rep.cols, n = D.n;
  const est = Object.fromEntries(SERIES.map((s) => [s, { n: new Float32Array(n).fill(NaN), e: new Float32Array(n).fill(NaN) }]));
  const xcs = new XCSoarWind(10);
  let j = 0, lp = null;
  for (let k = 0; k < n; k++) {
    for (; j < D.fixes.length && D.fixes[j].k <= k; j++) {
      const f = D.fixes[j], i = D.ai[f.k], roll = i >= 0 ? r.roll_deg[i] : 0;
      xcs.fix(f.u, D.t[f.k], f.track, f.speed, c.tas[f.k], 1 / Math.max(Math.cos(roll * D2R), 0.3));
    }
    if (xcs.out) { est.XCSoar.n[k] = xcs.out.n; est.XCSoar.e[k] = xcs.out.e; }
    const i = D.ai[k];
    if (i >= 0 && r.status[i] & (STATUS.WIND_VALID | STATUS.WIND_DEGRADED)) {
      est["AirMass nav"].n[k] = r.avg_wind_n[i]; est["AirMass nav"].e[k] = r.avg_wind_e[i];
      est["AirMass instant"].n[k] = r.wind_n[i]; est["AirMass instant"].e[k] = r.wind_e[i];
      // the time constant of XCSoar's wind at 1 Hz fixes (gain 0.01 per fix)
      if (!lp) lp = [r.wind_n[i], r.wind_e[i]];
      const a = Math.min(1, (D.t[k] - D.t[Math.max(0, k - 1)]) / 100);
      lp[0] += (r.wind_n[i] - lp[0]) * a; lp[1] += (r.wind_e[i] - lp[1]) * a;
      est["AirMass 100 s"].n[k] = lp[0]; est["AirMass 100 s"].e[k] = lp[1];
    } else lp = null;
  }
  const { free } = freeMask(D, (k) => c.ias[k] > 60 / 3.6);
  const raw = gnssCircleWind(D), ref = gnssCircleWind(D, { smooth: 1 });
  const turns = raw.turns, out = { circling: [], straight: [], moves: {}, xcsScale: xcs.ekf.X ? xcs.ekf.X[2] : NaN,
    amKtas: D.rep.summary.k_tas, turns: turns.length, fixInterval: medianFixInterval(D) };

  // circling: at the end of each turn, its index within the thermal
  let idx = 0;
  turns.forEach((t, m) => {
    idx = m && turns[m - 1].thermal === t.thermal ? idx + 1 : 0;
    const k = indexAt(t.t1);
    if (!free[k]) return;
    out.circling.push({ idx, e: SERIES.map((s) => [r2(est[s].n[k] - t.n), r2(est[s].e[k] - t.e)]) });
  });

  // straight flight between two thermals, against the interpolated per-turn winds
  const T = ref.turns;
  let p = -1;
  for (let k = 0; k < n; k++) {
    const t = D.t[k];
    while (p + 1 < T.length && T[p + 1].t1 <= t) p++;
    if (p < 0 || p + 1 >= T.length || !free[k]) continue;
    const a = T[p], b = T[p + 1];
    if (a.thermal === b.thermal || b.t0 - a.t1 > MAX_GAP || b.t0 <= t) continue;
    const i = D.ai[k];
    if (Math.abs(r.roll_deg[i]) > 10 || !Number.isFinite(c.gnss_track[k])) continue;
    const since = (t - a.t1) / 60, until = (b.t0 - t) / 60, bin = BINS.findIndex(([lo, hi]) => since >= lo && since < hi);
    if (bin < 0 || k % 5) continue; // 2 Hz is plenty for errors that change over minutes
    const ct = Math.cos(c.gnss_track[k] * D2R), st = Math.sin(c.gnss_track[k] * D2R);
    est["held thermal wind"].n[k] = a.n; est["held thermal wind"].e[k] = a.e;
    // along the track: + is more tailwind than the reference
    const e = SERIES.map((s) => {
      const dn = est[s].n[k] - ref.wn[k], de = est[s].e[k] - ref.we[k];
      return [r2(dn * ct + de * st), r2(-dn * st + de * ct)];
    });
    out.straight.push({ bin, until, valid: !!(r.status[i] & STATUS.WIND_VALID), e });
  }

  // how much each series moves within 10 s in straight flight (not air: the reference cannot resolve it)
  for (const s of SERIES.filter((s) => s !== "held thermal wind")) {
    let s2 = 0, m = 0, k2 = 0;
    for (let k = 0; k < n; k += 10) {
      while (k2 < n && D.t[k2] < D.t[k] + 10) k2++;
      const i = D.ai[k];
      if (k2 >= n || D.t[k2] - D.t[k] > 10.5 || !free[k] || !free[k2] || i < 0 || Math.abs(r.roll_deg[i]) > 10) continue;
      const d = Math.hypot(est[s].n[k2] - est[s].n[k], est[s].e[k2] - est[s].e[k]);
      if (Number.isFinite(d)) { s2 += d * d; m++; }
    }
    out.moves[s] = m ? Math.sqrt(s2 / m) : NaN;
  }

  // the reference's own changes: from the last turn of one thermal to the first of the next, and turn to turn within one
  const jumps = [], within = [];
  for (let m = 1; m < turns.length; m++) {
    const a = turns[m - 1], b = turns[m], d = Math.hypot(b.n - a.n, b.e - a.e);
    if (a.thermal === b.thermal) within.push(d);
    else if (b.t0 - a.t1 <= MAX_GAP) jumps.push([(b.t0 - a.t1) / 60, d]);
  }
  out.refJumps = jumps; out.refWithin = within;
  return out;
}

function medianFixInterval(D) {
  const d = [];
  for (let j = 1; j < D.fixes.length; j++) d.push(D.t[D.fixes[j].k] - D.t[D.fixes[j - 1].k]);
  d.sort((x, y) => x - y);
  return d.length ? d[d.length >> 1] : NaN;
}

const OK = (e) => e.every(Number.isFinite);
const rms = (a) => Math.sqrt(a.reduce((s, x) => s + x * x, 0) / a.length);
const mean = (a) => a.reduce((s, x) => s + x, 0) / a.length;
const fx = (x, d) => (Number.isFinite(x) ? x.toFixed(d) : "–").padStart(d + 3);
const f2 = (x) => fx(x, 2);

function report(flights) {
  const lines = [];
  const pool = (key) => flights.flatMap((f) => f.res[key]);
  const table = (rows, label) => {
    const all = rows.filter((x) => SERIES.every((s) => OK(E(x, s))));
    lines.push(`${label}: ${rows.length} samples, all have a wind at ${all.length} (${(100 * all.length / Math.max(1, rows.length)).toFixed(0)} %)`);
    lines.push("  series              has wind   rms error   rms along   rms across   mean along   mean across   (on the common samples)");
    for (const s of SERIES) {
      const have = rows.filter((x) => OK(E(x, s))).length;
      const al = all.map((x) => E(x, s)[0]), cr = all.map((x) => E(x, s)[1]);
      const tot = all.map((x) => Math.hypot(...E(x, s)));
      lines.push(`  ${s.padEnd(18)}    ${(100 * have / Math.max(1, rows.length)).toFixed(0).padStart(3)} %     ${f2(rms(tot))}       ${f2(rms(al))}       ${f2(rms(cr))}        ${f2(mean(al))}        ${f2(mean(cr))}`);
    }
  };
  lines.push(`${flights.length} flights, ${flights.reduce((s, f) => s + f.res.turns, 0)} full turns fitted; m/s`);
  lines.push("\n=== straight flight (bank < 10°) between two thermals, against the per-turn GNSS winds interpolated between them");
  lines.push("    (along: + more tailwind along the track than the reference; across: + wind from the left of the track)\n");
  const st = pool("straight");
  table(st.filter((x) => x.bin <= 2), "0.5 to 10 min after the thermal");
  BINS.forEach(([lo, hi], b) => { lines.push(""); table(st.filter((x) => x.bin === b), `${lo} to ${hi} min after the thermal`); });
  lines.push("\n(held thermal wind: the per-turn GNSS wind of the thermal before, held; the reference starts from it, so it is favoured\n early in the glide)");
  lines.push("\n--- by the time until the next thermal (0.5 to 20 min after the one before)\n");
  for (const [lo, hi] of [[0, 1], [1, 3], [3, 10]]) { table(st.filter((x) => x.until >= lo && x.until < hi), `${lo} to ${hi} min before the next thermal`); lines.push(""); }
  lines.push("--- by AirMass's wind status, 0.5 to 20 min after the thermal\n");
  table(st.filter((x) => x.valid), "AirMass wind valid"); lines.push("");
  table(st.filter((x) => !x.valid), "AirMass wind degraded");
  lines.push("\n=== circling, at the end of each full turn against that turn's GNSS circle fit (rms error, each series where it has a wind)");
  const ci = pool("circling");
  for (const [lab, f] of [["all turns", () => true], ["1st turn", (x) => x.idx === 0], ["2nd turn", (x) => x.idx === 1], ["3rd turn on", (x) => x.idx >= 2]]) {
    const rows = ci.filter(f);
    lines.push(`  ${lab.padEnd(12)} [${String(rows.length).padStart(4)}]  ` + SERIES.map((s) => {
      const v = rows.filter((x) => OK(E(x, s))).map((x) => Math.hypot(...E(x, s)));
      return `${s} ${f2(rms(v)).trim()} (${(100 * v.length / Math.max(1, rows.length)).toFixed(0)} %)`;
    }).join(",  "));
  }
  lines.push("\n=== how much the wind moves within 10 s in straight flight (rms m/s, mean over the flights)");
  lines.push("  " + Object.keys(flights[0].res.moves).map((s) => `${s} ${f2(mean(flights.map((f) => f.res.moves[s]).filter(Number.isFinite))).trim()}`).join(",  "));
  const jumps = flights.flatMap((f) => f.res.refJumps), within = flights.flatMap((f) => f.res.refWithin);
  lines.push("\n=== the reference itself");
  lines.push(`  turn to turn within a thermal: rms ${f2(rms(within)).trim()} m/s [${within.length}] (a single turn's fit error is about this / √2)`);
  for (const [lo, hi] of [[0, 5], [5, 10], [10, 20]]) {
    const v = jumps.filter(([g]) => g >= lo && g < hi).map(([, d]) => d);
    lines.push(`  last turn of one thermal to the first of the next, ${lo}–${hi} min apart: rms ${f2(rms(v)).trim()} m/s [${v.length}]`);
  }
  lines.push("\n=== per flight, straight 0.5–10 min after the thermal, rms error on the common samples; XCSoar's learned airspeed scale, AirMass's k_tas");
  lines.push("  flight                        fix s   samples   AirMass nav  100 s    XCSoar    held      XCSoar scale  AirMass k_tas");
  for (const f of flights) {
    const rows = f.res.straight.filter((x) => x.bin <= 2 && SERIES.every((s) => OK(E(x, s))));
    const v = (s) => f2(rows.length ? rms(rows.map((x) => Math.hypot(...E(x, s)))) : NaN);
    lines.push(`  ${f.file.padEnd(28)}  ${fx(f.res.fixInterval, 2).trim().padStart(4)}   ${String(rows.length).padStart(6)}      ${v("AirMass nav")}     ${v("AirMass 100 s")}    ${v("XCSoar")}    ${v("held thermal wind")}        ${fx(f.res.xcsScale, 3)}        ${fx(f.res.amKtas, 3)}`);
  }
  return lines.join("\n");
}

(async () => {
  const [outFile, ...logs] = process.argv.slice(2);
  if (outFile && !outFile.endsWith(".json")) throw new Error("usage: node xcsoar_wind.js OUT.json LOG...");
  if (!logs.length) { console.log(report(JSON.parse(fs.readFileSync(outFile, "utf8")))); return; }
  const M = await require(path.join(web, "airmass.js"))();
  const flights = [];
  for (const log of logs) {
    globalThis.fileName = path.basename(log);
    const conv = loadFlight(log);
    const rep = await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" });
    if (!conv.track.n || !rep.n) { process.stderr.write(`${fileName}: not replayable\n`); continue; }
    globalThis.D = build(conv, rep);
    derive(D);
    flights.push({ file: fileName, res: evaluate(D) });
    process.stderr.write(fileName + "\n");
  }
  fs.writeFileSync(outFile, JSON.stringify(flights));
  console.log(report(flights));
})().catch((e) => { console.error(e); process.exit(1); });
