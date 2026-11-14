// Shared by the analysis scripts: the browser replay's own code (index.html: build, atFixes, Damp, derive, indexAt)
// loaded into node, and the helpers the scripts have in common. derive() gives the two GNSS references: D.gnssNetInert
// (gust-insensitive: GNSS ground velocity minus a 2 min wind) and D.gnssNetPitot (pitot TAS). The page functions read
// the globals D (the flight being worked on), fileName, $ (the settings) and STATUS (the status bits, replay.js), so a
// script assigns D = build(...) without declaring it.
const fs = require("fs"), path = require("path"), zlib = require("zlib");

// the page's script (index.html in the web dir), and one of its functions or classes in it: from its head (the text
// it starts with) to the matching brace. Also for the page's tests.
const pageScript = (web) => fs.readFileSync(path.join(web, "index.html"), "utf8").split("<script>")[1].split("</script>")[0];
function grab(script, name) {
  const i = script.indexOf(name);
  if (i < 0) throw new Error("index.html has no " + name);
  let j = script.indexOf("{", i), depth = 0;
  for (; j < script.length; j++) { if (script[j] === "{") depth++; else if (script[j] === "}" && --depth === 0) break; }
  return script.slice(i, j + 1);
}

// REF_SINK=airmass (or { refSink: "airmass" }): the GNSS netto references subtract AirMass's polar sink (calibrated
// airspeed, its load factor) instead of the device's (measured IAS polar in TAS terms), the one TE netto uses.
// Returns the web dir's convert and runReplay, and loadFlight(log, opts = logOptions()): a log (.gz or plain) converted.
function loadPage(web, { refSink = process.env.REF_SINK } = {}) {
  web = path.resolve(web); // a relative path would be taken for a package name by require()
  const script = pageScript(web);
  const names = ["build", "gnssAltDelay", "gnssSpeedLag", "atFixes", "Damp", "TeVario", "derive", "deriveDamping", "indexAt", "runningSums", "cockpitSeries", "gnssNetFromEnergy"];
  let src = ["function build(conv, rep)", "function gnssAltDelay(fixes, t, baro, running)", "function gnssSpeedLag(fixes, t)", "function atFixes(D, get, maxHold)", "class Damp", "class TeVario", "function derive(D)", "function deriveDamping(D)", "function indexAt(t)",
    "function runningSums(n, val)", "function cockpitSeries(D, dampAt)", "function gnssNetFromEnergy(D, sinkUp, energyOf)"].map((name) => grab(script, name)).join("\n");
  if (refSink === "airmass") {
    const patched = src.replace("if (Number.isFinite(sink)) sinkUp[k] = sink;", "if (Number.isFinite(r.polar_sink[i])) sinkUp[k] = -r.polar_sink[i];");
    if (patched === src) throw new Error("REF_SINK patch failed: derive() changed");
    src = patched;
  }
  const { runReplay, STATUS } = require(path.join(web, "replay.js"));
  globalThis.$ = (id) => ({ damping: { value: "4" }, samesmooth: { checked: false } })[id];
  globalThis.D = null;
  globalThis.fileName = "";
  globalThis.STATUS = STATUS;
  (0, eval)(src + "\n" + names.map((x) => `globalThis.${x} = ${x};`).join(" "));
  const { convert } = require(path.join(web, "convert.js"));
  const loadFlight = (log, opts = logOptions()) => {
    const bytes = fs.readFileSync(log);
    return convert((log.endsWith(".gz") ? zlib.gunzipSync(bytes) : bytes).toString("latin1"), opts);
  };
  return { convert, runReplay, loadFlight };
}

// f(run, k) of every sample k of all runs ({ n, ... }), in one array
function cat(runs, f) {
  const out = [];
  for (const a of runs) for (let k = 0; k < a.n; k++) out.push(f(a, k));
  return out;
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

// x with the device's vario damping (4 s) in its 10 Hz cycles at the sample times t [s], restarted where ok is false
function dampSeries(x, ok, d = 4, t = D.t) {
  const y = new Float32Array(x.length).fill(NaN);
  let f = null;
  for (let k = 0; k < x.length; k++) {
    if (!ok[k] || !Number.isFinite(x[k])) { f = null; continue; }
    f ||= new Damp(d);
    y[k] = f.at(t[k], x[k]);
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

// least squares: the coefficients x minimising |X x - y|² (rows of X), by the normal equations and Gaussian
// elimination; null if singular
function lsq(X, y) {
  const m = X[0].length, A = Array.from({ length: m }, () => new Float64Array(m)), b = new Float64Array(m);
  X.forEach((row, i) => { for (let p = 0; p < m; p++) { b[p] += row[p] * y[i]; for (let q = 0; q < m; q++) A[p][q] += row[p] * row[q]; } });
  for (let p = 0; p < m; p++) {
    let piv = p;
    for (let r = p + 1; r < m; r++) if (Math.abs(A[r][p]) > Math.abs(A[piv][p])) piv = r;
    [A[p], A[piv]] = [A[piv], A[p]]; [b[p], b[piv]] = [b[piv], b[p]];
    if (!(Math.abs(A[p][p]) > 1e-12)) return null;
    for (let r = p + 1; r < m; r++) { const f = A[r][p] / A[p][p]; for (let q = p; q < m; q++) A[r][q] -= f * A[p][q]; b[r] -= f * b[p]; }
  }
  const x = new Float64Array(m);
  for (let p = m - 1; p >= 0; p--) { let s = b[p]; for (let q = p + 1; q < m; q++) s -= A[p][q] * x[q]; x[p] = s / A[p][p]; }
  return x;
}

// The wind from GNSS alone, per full turn while circling: the ground velocities of the fixes of one turn (360° of
// track change, the track turning faster than 6°/s the same way) lie on a circle around the wind, its radius the
// airspeed (taken constant over the turn); a least squares circle fit (Kåsa). Uses neither heading nor airspeed nor
// AirMass. Returns the turns [{ t (device time of the turn's middle, s), n, e (m/s), tas (the radius), res (rms of
// the points off the circle) }] and the wind per sample (wn, we), interpolated in time between turns, held before the
// first and after the last.
// opts: tas (true: fit |ground velocity - wind| to the measured TAS of each fix times a scale, Gauss-Newton from the
// circle fit, instead of a constant airspeed: the size of the airspeed only, still no heading and no AirMass), smooth
// (the number of turns on either side averaged with each turn's wind, within the same circling, default 0)
function gnssCircleWind(D, opts = {}) {
  const F = D.fixes, c = D.tr, turns = [];
  let seg = [], cum = 0, sign = 0, thermalId = 0;
  const fit = (pts) => {
    // x² + y² + a x + b y + d = 0, linear in a, b, d
    const sol = lsq(pts.map(([x, y]) => [x, y, 1]), pts.map(([x, y]) => -(x * x + y * y)));
    if (!sol) return null;
    const cx = -sol[0] / 2, cy = -sol[1] / 2, r = Math.sqrt(cx * cx + cy * cy - sol[2]);
    let res = 0;
    for (const [x, y] of pts) res += (Math.hypot(x - cx, y - cy) - r) ** 2;
    return { n: cx, e: cy, tas: r, res: Math.sqrt(res / pts.length) };
  };
  for (let j = 1; j < F.length; j++) {
    const p = F[j - 1], q = F[j], dt = q.u - p.u;
    const tp = c.gnss_track[p.k], tq = c.gnss_track[q.k], sp = c.gnss_speed[q.k];
    let rate = NaN;
    if (dt > 0.5 && dt < 3 && Number.isFinite(tp) && Number.isFinite(tq)) rate = ((((tq - tp) % 360) + 540) % 360 - 180) / dt;
    if (!(Math.abs(rate) > 6) || (sign && Math.sign(rate) !== sign) || !Number.isFinite(sp)) { if (sign) thermalId++; seg = []; cum = 0; sign = 0; continue; }
    sign = Math.sign(rate);
    if (!seg.length) seg.push(p);
    seg.push(q);
    cum += Math.abs(rate) * dt;
    if (cum >= 360) {
      const pts = seg.map((f) => [c.gnss_speed[f.k] * Math.cos(c.gnss_track[f.k] * Math.PI / 180), c.gnss_speed[f.k] * Math.sin(c.gnss_track[f.k] * Math.PI / 180)]);
      let w = fit(pts);
      if (w && opts.tas) {
        // minimise the sum of (|v_i - W| - s TAS_i)² over W and s
        const tas = seg.map((f) => c.tas[indexAt(f.tm)]), tasMean = tas.reduce((u, v) => u + v, 0) / tas.length;
        let x = [w.n, w.e, w.tas / tasMean];
        for (let it = 0; it < 10; it++) {
          // Gauss-Newton: the step solves J step = residuals in least squares
          const J = [], res = [];
          pts.forEach(([px, py], i) => {
            const dx = px - x[0], dy = py - x[1], d = Math.hypot(dx, dy) || 1e-6;
            J.push([-dx / d, -dy / d, -tas[i]]); res.push(d - x[2] * tas[i]);
          });
          const step = lsq(J, res);
          if (!step) break;
          x = x.map((v, i) => v - step[i]);
        }
        let res = 0;
        pts.forEach(([px, py], i) => { res += (Math.hypot(px - x[0], py - x[1]) - x[2] * tas[i]) ** 2; });
        w = { n: x[0], e: x[1], tas: x[2] * tasMean, res: Math.sqrt(res / pts.length) };
      }
      if (w && w.tas > 15 && w.tas < 70 && w.res < 2.5) turns.push({ t: (seg[0].tm + seg[seg.length - 1].tm) / 2, thermal: thermalId, ...w });
      seg = [q]; cum = 0;
    }
  }
  if (opts.smooth > 0) {
    const raw = turns.map((t) => [t.n, t.e]);
    turns.forEach((t, i) => {
      let sn = 0, se = 0, m = 0;
      for (let j = i - opts.smooth; j <= i + opts.smooth; j++) if (turns[j] && turns[j].thermal === t.thermal) { sn += raw[j][0]; se += raw[j][1]; m++; }
      t.n = sn / m; t.e = se / m;
    });
  }
  const wn = new Float32Array(D.n).fill(NaN), we = new Float32Array(D.n).fill(NaN);
  for (let k = 0, i = 0; k < D.n && turns.length; k++) {
    const t = D.t[k];
    while (i + 1 < turns.length && turns[i + 1].t <= t) i++;
    const a = turns[i], b = turns[i + 1];
    const f = !b || t <= a.t ? 0 : Math.min(1, (t - a.t) / (b.t - a.t));
    wn[k] = a.n + ((b ? b.n : a.n) - a.n) * f;
    we[k] = a.e + ((b ? b.e : a.e) - a.e) * f;
  }
  return { turns, wn, we };
}

// The gust-insensitive GNSS netto as derive() computes it (gnssNetFromEnergy, its polar sink D.sinkUp), with the given
// wind per sample (wn, we) instead of AirMass's averaged one. Uses the global D (indexAt).
function gnssNetWithWind(D, wn, we) {
  const G = 9.80665, c = D.tr;
  return gnssNetFromEnergy(D, D.sinkUp, (f) => {
    const spd = c.gnss_speed[f.k], trk = c.gnss_track[f.k] * Math.PI / 180, km = indexAt(f.tm);
    const vn = spd * Math.cos(trk) - wn[km], ve = spd * Math.sin(trk) - we[km];
    return f.alt + (vn * vn + ve * ve) / (2 * G);
  });
}

// The converter options of the real-flight scripts: the glider and the weight the log names (CA1, else the ASG 29/18
// and the logged ballast factor, as the page does), or GLIDER=name, MASS=kg
const logOptions = () => ({ glider: process.env.GLIDER || "log", mass: +(process.env.MASS || 0) });

module.exports = { loadPage, pageScript, grab, cat, lagOf, dampSeries, freeMask, lsq, gnssCircleWind, gnssNetWithWind, logOptions };
