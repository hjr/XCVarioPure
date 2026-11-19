// The GNSS velocity's age against the IMU, comparable to the estimator's latency state: fixes at arrival minus
// GnssTiming's transport delay. Track: Vg * track change between consecutive fixes against the integral of the horizontal
// lateral acceleration g (fy cos(roll) - fz sin(roll)). Speed: speed change in level flight against the horizontal
// acceleration along the heading g (fx cos(pitch) + fz sin(pitch)). One JSON line per log, ages in ms: track and speed
// over the flight, track per quarter, the estimator's latency (median, end) and the transport delay.
//   node imu_latency.js LOG...   (SETTINGS="name=value,...": AirMass settings)
const path = require("path"), A = __dirname;
const { loadPage } = require(A + "/page.js"); const web = path.resolve(A, "../web");
const { runReplay, loadFlight } = loadPage(web);
const G = 9.81;
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  for (const f of process.argv.slice(2)) {
    try {
    const conv = loadFlight(f, { glider: "log" });
    const fr = new Map(); const gn = [];
    for (const l of conv.rec) {
      if (l.startsWith("$PXCVR,")) { const x = l.split("*")[0].split(","); fr.set(+x[1], [+x[2], +x[3], +x[4], +x[9]]); }
      else if (l.startsWith("$PXCVG,")) { const x = l.split("*")[0].split(","); gn.push({ t: +x[1], vn: +x[2], ve: +x[3], u: x.length > 9 && x[9] !== "" ? +x[9] : NaN }); }
    }
    let ga = null, ge = null, gea = 0, gd = 0; const fx = [];
    for (const q of gn) { if (ga !== null && q.t < ga) continue; if (Number.isFinite(q.u)) { if (ge !== null) { let sd = q.u - ge; if (sd < -43200000) sd += 86400000; if (sd > 43200000) sd -= 86400000; if (sd <= 0) continue; gd = Math.max(0, gd + (q.t - gea) - sd); } ge = q.u; gea = q.t; } ga = q.t; fx.push({ t: q.t - gd, vn: q.vn, ve: q.ve, d: gd }); }
    const rep = await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" });
    const r = rep.cols, t = rep.t_ms, n = rep.n;
    const ay = new Float64Array(n).fill(NaN), ah = new Float64Array(n).fill(NaN);
    for (let i = 0; i < n; i++) {
      const g = fr.get(t[i]); if (!g || !(r.status[i] & 8) || !(g[3] > 15)) continue;
      const ph = r.roll_deg[i] * Math.PI / 180, th = r.pitch_deg[i] * Math.PI / 180;
      if (Math.abs(r.roll_deg[i]) < 70) ay[i] = G * (g[1] * Math.cos(ph) - g[2] * Math.sin(ph));
      if (Math.abs(r.roll_deg[i]) < 8) ah[i] = G * (g[0] * Math.cos(th) + g[2] * Math.sin(th));
    }
    // integral of v dt up to each sample (a sample covers the interval before its time), NaN-breaking
    const integ = (v) => { const s = new Float64Array(n), bad = new Uint32Array(n + 1); for (let i = 1; i < n; i++) { const dt = (t[i] - t[i - 1]) / 1000; const ok = Number.isFinite(v[i]) && dt > 0 && dt < 0.3; s[i] = s[i - 1] + (ok ? v[i] * dt : 0); bad[i + 1] = bad[i] + (ok ? 0 : 1); } return { s, bad }; };
    const idx = (ms) => { let lo = 0, hi = n; while (lo < hi) { const md = (lo + hi) >> 1; if (t[md] < ms) lo = md + 1; else hi = md; } return lo; };
    const cum = (c, ms) => { const j = idx(ms); if (j <= 0 || j >= n) return NaN; return c.s[j - 1] + (c.s[j] - c.s[j - 1]) * (ms - t[j - 1]) / (t[j] - t[j - 1]); };
    const over = (c, a, b) => { const i = idx(a), j = idx(b); if (i <= 0 || j >= n || c.bad[j + 1] - c.bad[i] > 0) return NaN; return cum(c, b) - cum(c, a); };
    const cy = integ(ay), ch = integ(ah);
    const sp = [], tr = [];
    for (let j = 1; j < fx.length; j++) {
      const p = fx[j - 1], q = fx[j]; const dt = (q.t - p.t) / 1000; if (!(dt > 0.5 && dt < 1.5)) continue;
      const s0 = Math.hypot(p.vn, p.ve), s1 = Math.hypot(q.vn, q.ve); if (s0 < 15 || s1 < 15) continue;
      sp.push({ a: p.t, b: q.t, v: s1 - s0 });
      let d = Math.atan2(q.ve, q.vn) - Math.atan2(p.ve, p.vn); d = ((d % (2 * Math.PI)) + 3 * Math.PI) % (2 * Math.PI) - Math.PI;
      if (Math.abs(d / dt) < 0.6) tr.push({ a: p.t, b: q.t, v: d * 0.5 * (s0 + s1) });
    }
    const fit = (obs, c, demean, minM, part) => {
      let best = null; const cur = [];
      for (let L = -500; L <= 2500; L += 20) { let s2 = 0, sb = 0, m = 0; for (let k = 0; k < obs.length; k++) { if (part && !part(k)) continue; const o = obs[k]; const p = over(c, o.a - L, o.b - L); if (!Number.isFinite(p)) continue; const e = o.v - p; s2 += e * e; sb += e; m++; } const v = demean ? s2 / m - (sb / m) ** 2 : s2 / m; cur.push([L, v]); if (m > minM && (!best || v < best.v)) best = { L, v, m }; }
      if (!best) return null;
      const k = cur.findIndex((c) => c[0] === best.L); if (k > 0 && k < cur.length - 1) { const y0 = cur[k - 1][1], y1 = cur[k][1], y2 = cur[k + 1][1]; const den = y0 - 2 * y1 + y2; if (den > 0) best.L = Math.round(best.L + 20 * 0.5 * (y0 - y2) / den); }
      return best;
    };
    const T = fit(tr, cy, false, 500), S = fit(sp, ch, true, 800);
    // the track age over the flight's quarters
    const parts = []; for (let q = 0; q < 4; q++) { const lo = Math.floor(tr.length * q / 4), hi = Math.floor(tr.length * (q + 1) / 4); const x = fit(tr, cy, false, 200, (k) => k >= lo && k < hi); parts.push(x ? x.L : NaN); }
    let lat = []; for (let i = 0; i < n; i++) if (r.status[i] & 8) lat.push(r.latency_ms[i]); const last = lat.length ? lat[lat.length - 1] : NaN; lat.sort((a, b) => a - b);
    const ds = fx.map((x) => x.d).sort((a, b) => a - b);
    console.log(JSON.stringify({ file: path.basename(f).slice(0, 16), track: T ? T.L : null, track_rms: T ? +Math.sqrt(T.v).toFixed(3) : null, quarters: parts, speed: S ? S.L : null, ekf_med: lat.length ? Math.round(lat[lat.length >> 1]) : null, ekf_end: Math.round(last), transport_med: ds[ds.length >> 1], transport_mean: Math.round(ds.reduce((a, b) => a + b, 0) / ds.length) }));
    } catch (e) { console.log(path.basename(f), "ERR", e.message); }
  }
})();
