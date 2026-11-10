// Does a changing wind (climbing through a wind gradient) make AirMass's circling error? The attitude's gravity
// reference takes the flight path acceleration from the airspeed alone (constant wind, AirMassAttitude.h), so a
// steady wind change dW/dt tilts it by dW/dt / g, fixed to the ground: a pitch error that follows the heading, and in
// the vario ~ V * (dW/dt . u) / g. Per thermal (90 s or more circling): dW/dt from the GNSS-only per-turn winds
// (page.js gnssCircleWind), and the once-per-circle part of the vario error against the GNSS-only reference,
// err = c + a cos(psi) + b sin(psi). Across thermals: (a, b) against dW/dt, for AirMass and for TE netto (no attitude).
//   node shear.js <web dir> log1.nmea.gz ...   (SETTINGS="name=value,...")
const path = require("path");
const { loadPage, lagOf, freeMask, lsq, gnssCircleWind, gnssNetWithWind } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { runReplay, loadFlight } = loadPage(web);
const HZ = 10, MIN_S = 90;

(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const thermals = [];
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = loadFlight(log);
    D = build(conv, await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" }));
    derive(D);
    const r = D.rep.cols, c = D.tr, n = D.n;
    const gw = gnssCircleWind(D, { tas: true });
    const ref = gnssNetWithWind(D, gw.wn, gw.we);
    const { free } = freeMask(D, (k) => D.amOk[k] && Number.isFinite(ref[k]) && Number.isFinite(D.netto[k]));
    const lagA = lagOf(D.amRaw, ref, free), lagT = lagOf(D.netto, ref, free);
    const bank = (k) => (D.ai[k] >= 0 ? Math.abs(r.roll_deg[D.ai[k]]) : NaN);
    for (let k = 0; k < n; k++) {
      if (!(bank(k) > 25) || !free[k]) continue;
      let e = k; while (e < n && bank(e) > 15 && free[e]) e++;
      if ((e - k) / HZ >= MIN_S) {
        // dW/dt: regression of the turn winds over the thermal's time
        const ts = gw.turns.filter((t) => t.t >= D.t[k] && t.t <= D.t[e - 1]);
        if (ts.length >= 4) {
          const t0 = ts[0].t, X = ts.map((t) => [1, t.t - t0]);
          const gn = lsq(X, ts.map((t) => t.n))[1], ge = lsq(X, ts.map((t) => t.e))[1];
          const fit = (ser, lag) => {
            const X2 = [], y = [];
            for (let j = k; j < e - lag; j++) {
              const v = ser[j + lag] - ref[j], psi = r.heading_deg[D.ai[j]] * Math.PI / 180;
              if (Number.isFinite(v) && Number.isFinite(psi)) { X2.push([1, Math.cos(psi), Math.sin(psi)]); y.push(v); }
            }
            const x = lsq(X2, y);
            return [x[1], x[2]];
          };
          let climb = 0; for (let j = k; j < e; j++) climb += ref[j] / (e - k);
          const wn = ts.reduce((s2, t) => s2 + t.n, 0) / ts.length, we = ts.reduce((s2, t) => s2 + t.e, 0) / ts.length;
          thermals.push({ flight: fileName.slice(0, 16), s: (e - k) / HZ, climb, gn, ge, wn, we, am: fit(D.amRaw, lagA), te: fit(D.netto, lagT), tas: c.tas[k] });
        }
      }
      k = e;
    }
    process.stderr.write(fileName + "\n");
  }
  // across thermals: the error harmonic (a, b) against K * (gn, ge) (a: north, b: east), one K for both components
  const K = (key) => {
    let sxy = 0, sxx = 0, syy = 0;
    for (const t of thermals) { const [a, b] = t[key]; sxy += a * t.gn + b * t.ge; sxx += t.gn * t.gn + t.ge * t.ge; syy += a * a + b * b; }
    return { K: sxy / sxx, corr: sxy / Math.sqrt(sxx * syy) };
  };
  const rms = (key) => Math.sqrt(thermals.reduce((s, t) => s + t[key][0] ** 2 + t[key][1] ** 2, 0) / thermals.length);
  const g = Math.sqrt(thermals.reduce((s, t) => s + t.gn ** 2 + t.ge ** 2, 0) / thermals.length);
  console.log(`${thermals.length} thermals (${MIN_S} s or more circling); wind change rate rms ${g.toFixed(3)} m/s² (${(g * 60).toFixed(1)} m/s per minute)`);
  for (const key of ["am", "te"]) {
    const k = K(key);
    console.log(`  ${key === "am" ? "AirMass" : "TE netto"}: once-per-circle error amplitude rms ${rms(key).toFixed(3)} m/s; against dW/dt: K ${k.K.toFixed(2)} s, correlation ${k.corr.toFixed(2)}` +
      `${key === "am" ? " (the attitude hypothesis: K about -V/g = -3 s)" : ""}`);
  }
  // the same against the wind itself (the heading relative to the wind)
  for (const key of ["am", "te"]) {
    let sxy = 0, sxx = 0, syy = 0, cxy = 0;
    for (const t of thermals) { const [a, b] = t[key]; sxy += a * t.wn + b * t.we; cxy += a * t.we - b * t.wn; sxx += t.wn * t.wn + t.we * t.we; syy += a * a + b * b; }
    console.log(`  ${key === "am" ? "AirMass" : "TE netto"} error harmonic against the wind: along K ${(sxy / sxx).toFixed(3)} s⁻¹·s (corr ${(sxy / Math.sqrt(sxx * syy)).toFixed(2)}), across corr ${(cxy / Math.sqrt(sxx * syy)).toFixed(2)}`);
  }
  // the mean direction of dW/dt and of AirMass's harmonic
  const mean = (f) => thermals.reduce((s, t) => s + f(t), 0) / thermals.length;
  const dir = (n, e) => ((Math.atan2(e, n) * 180 / Math.PI) + 360) % 360;
  console.log(`  mean dW/dt toward ${dir(mean((t) => t.gn), mean((t) => t.ge)).toFixed(0)}° (${Math.hypot(mean((t) => t.gn), mean((t) => t.ge)).toFixed(3)} m/s²); ` +
    `AirMass error harmonic peaks at heading ${dir(mean((t) => t.am[0]), mean((t) => t.am[1])).toFixed(0)}° (${Math.hypot(mean((t) => t.am[0]), mean((t) => t.am[1])).toFixed(3)} m/s)`);
})().catch((e) => { console.error(e); process.exit(1); });
