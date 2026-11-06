// Shared by the analysis scripts: the browser replay's own code (index.html: build, atFixes, Damp, derive, indexAt)
// loaded into node, and the helpers the scripts have in common. The page functions read the globals D (the flight
// being worked on), fileName and $ (the settings), so a script assigns D = build(...) without declaring it.
const fs = require("fs"), path = require("path");

// REF_SINK=airmass (or { refSink: "airmass" }): the GNSS netto reference subtracts AirMass's polar sink (calibrated
// airspeed, its load factor) instead of the device's (measured IAS polar in TAS terms), the one TE netto uses
function loadPage(web, { refSink = process.env.REF_SINK } = {}) {
  web = path.resolve(web); // a relative path would be taken for a package name by require()
  const script = fs.readFileSync(path.join(web, "index.html"), "utf8").split("<script>")[1].split("</script>")[0];
  const grab = (name) => {
    const i = script.indexOf(name);
    if (i < 0) throw new Error("index.html has no " + name);
    let j = script.indexOf("{", i), depth = 0;
    for (; j < script.length; j++) { if (script[j] === "{") depth++; else if (script[j] === "}" && --depth === 0) break; }
    return script.slice(i, j + 1);
  };
  let src = ["function build(conv, rep)", "function atFixes(D, get, maxHold)", "class Damp", "function derive(D)", "function indexAt(t)"].map(grab).join("\n");
  if (refSink === "airmass") {
    const patched = src.replace("if (Number.isFinite(sink)) sinkUp[k] = sink;", "if (Number.isFinite(r.polar_sink[i])) sinkUp[k] = -r.polar_sink[i];");
    if (patched === src) throw new Error("REF_SINK patch failed: derive() changed");
    src = patched;
  }
  globalThis.$ = (id) => ({ damping: { value: "4" }, samesmooth: { checked: false } })[id];
  globalThis.D = null;
  globalThis.fileName = "";
  (0, eval)(src + "\nglobalThis.build = build; globalThis.atFixes = atFixes; globalThis.derive = derive; globalThis.Damp = Damp; globalThis.indexAt = indexAt;");
  return { convert: require(path.join(web, "convert.js")).convert, runReplay: require(path.join(web, "replay.js")).runReplay };
}

// the delay (in 10 Hz samples, 0..6 s) at which x matches y best (cross-correlation), on the samples where ok
function lagOf(x, y, ok) {
  let best = -2, bl = 0;
  for (let l = 0; l <= 60; l += 2) {
    let n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    for (let k = 0; k + l < x.length; k += 3) {
      if (!ok[k] || !ok[k + l]) continue;
      const a = x[k + l], b = y[k];
      if (!Number.isFinite(a) || !Number.isFinite(b)) continue;
      n++; sx += a; sy += b; sxx += a * a; syy += b * b; sxy += a * b;
    }
    const c = (n * sxy - sx * sy) / Math.sqrt((n * sxx - sx * sx) * (n * syy - sy * sy));
    if (c > best) { best = c; bl = l; }
  }
  return bl;
}

// x with the device's vario damping (4 s), restarted where ok is false
function dampSeries(x, ok, d = 4) {
  const y = new Float32Array(x.length).fill(NaN);
  let f = null;
  for (let k = 0; k < x.length; k++) {
    if (!ok[k] || !Number.isFinite(x[k])) { f = null; continue; }
    f ||= new Damp(d);
    y[k] = f.f(x[k]);
  }
  return y;
}

// free flight: AirMass running, after the tow (until 10 s banked over 30°, at most 15 min), where extra(k) holds.
// Returns { free, since } (since: seconds since the start of the flight)
function freeMask(D, extra) {
  const r = D.rep.cols, n = D.n, free = new Uint8Array(n), since = new Float32Array(n).fill(NaN);
  let start = null, banked = 0, towEnd = null;
  for (let k = 0; k < n; k++) {
    const i = D.ai[k];
    if (i < 0) { start = null; towEnd = null; banked = 0; continue; }
    if (start === null) start = D.t[k];
    since[k] = D.t[k] - start;
    if (towEnd === null) {
      banked = Math.abs(r.roll_deg[i]) > 30 ? banked + 1 : 0;
      if (banked >= 100 || D.t[k] - start > 900) towEnd = D.t[k];
      continue;
    }
    free[k] = extra(k) ? 1 : 0;
  }
  return { free, since };
}

// The gust-insensitive reference: the energy height of the fixes from GNSS altitude and the ground velocity relative to
// AirMass's averaged wind (smoothed over 2 min), its change over the fixes before and after each fix, minus the mean
// polar sink; at the fixes' measurement times, interpolated. Uses the global D (indexAt).
function inertialRef(D) {
  const G = 9.80665, r = D.rep.cols, c = D.tr, n = D.n, F = D.fixes;
  const wn = new Float64Array(n).fill(NaN), we = new Float64Array(n).fill(NaN), half = 600;
  const cn = new Float64Array(n + 1), ce = new Float64Array(n + 1), cc = new Float64Array(n + 1);
  for (let k = 0; k < n; k++) {
    const i = D.ai[k], ok = i >= 0 && (r.status[i] & 2);
    cn[k + 1] = cn[k] + (ok ? r.avg_wind_n[i] : 0); ce[k + 1] = ce[k] + (ok ? r.avg_wind_e[i] : 0); cc[k + 1] = cc[k] + (ok ? 1 : 0);
  }
  for (let k = 0; k < n; k++) {
    const a = Math.max(0, k - half), b = Math.min(n, k + half), m = cc[b] - cc[a];
    if (m > 300) { wn[k] = (cn[b] - cn[a]) / m; we[k] = (ce[b] - ce[a]) / m; }
  }
  const sens = D.kind.startsWith("XCVario sensor");
  const cs = new Float64Array(n + 1), cm = new Float64Array(n + 1);
  for (let k = 0; k < n; k++) {
    const i = D.ai[k];
    const s = i < 0 ? NaN : sens ? c.polar_sink[k] * c.tas[k] / Math.max(c.ias[k], 10) : -r.polar_sink[i];
    const ok = Number.isFinite(s);
    cs[k + 1] = cs[k] + (ok ? s : 0); cm[k + 1] = cm[k] + (ok ? 1 : 0);
  }
  const E = F.map((f) => {
    const k = f.k, spd = c.gnss_speed[k], trk = c.gnss_track[k] * Math.PI / 180, km = indexAt(f.tm);
    if (!Number.isFinite(spd) || !Number.isFinite(trk) || !Number.isFinite(wn[km])) return NaN;
    const vn = spd * Math.cos(trk) - wn[km], ve = spd * Math.sin(trk) - we[km];
    return f.alt + (vn * vn + ve * ve) / (2 * G);
  });
  const ref = [];
  for (let j = 1; j + 1 < F.length; j++) {
    const p = F[j - 1], q = F[j + 1], dt = q.u - p.u;
    if (!(dt > 0.5 && dt < 5) || !Number.isFinite(E[j - 1]) || !Number.isFinite(E[j + 1])) continue;
    const kp = indexAt(p.tm), kq = indexAt(q.tm), m = cm[kq + 1] - cm[kp];
    if (!(m > 0)) continue;
    ref.push({ t: F[j].tm, v: (E[j + 1] - E[j - 1]) / dt - (cs[kq + 1] - cs[kp]) / m });
  }
  const out = new Float32Array(n).fill(NaN);
  for (let r0 = 0, k = 0; k < n; k++) {
    const t = D.t[k];
    while (r0 + 1 < ref.length && ref[r0 + 1].t <= t) r0++;
    const a = ref[r0], b = ref[r0 + 1];
    if (!a || !b || t < a.t || b.t - a.t > 3) continue;
    out[k] = a.v + (b.v - a.v) * (t - a.t) / (b.t - a.t);
  }
  return out;
}

module.exports = { loadPage, lagOf, dampSeries, freeMask, inertialRef };
