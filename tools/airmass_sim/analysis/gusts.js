// Horizontal gusts on the real flights, to look at in the web replay: straight flight with a steady pitch (not a pull-up)
// and steady vertical air (the gust-insensitive GNSS reference) over 9 s, where IAS jumps (1 s means 1 s apart) and TE
// netto shows it as lift or sink against the reference. Per moment: the flight, UTC (the replay's clock), the IAS
// change, and the largest deviation from the reference within 0 .. +4 s of TE netto and the cockpit AirMass vario, each
// at its delay. Sorted by how much less AirMass deviates.
//   node gusts.js <web dir> log1.nmea.gz ...   (MIN_DIAS km/h, default 8; MIN_TE m/s, default 1.5; TOP, default 15)
const path = require("path");
const { loadPage, cat, lagOf, freeMask } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { runReplay, loadFlight } = loadPage(web);
const HZ = 10, MIN_DIAS = +(process.env.MIN_DIAS || 8), MIN_TE = +(process.env.MIN_TE || 1.5), TOP = +(process.env.TOP || 15);
const hms = (s) => [s / 3600, (s / 60) % 60, s % 60].map((x) => String(Math.floor(x)).padStart(2, "0")).join(":");

(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const runs = [];
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = loadFlight(log);
    D = build(conv, await runReplay(M, conv.rec, {}));
    derive(D);
    const r = D.rep.cols, c = D.tr;
    const ref = D.gnssNetInert;
    const { free } = freeMask(D, (k) => D.amOk[k] && c.ias[k] > 60 / 3.6 && Number.isFinite(ref[k]) && Number.isFinite(D.netto[k]));
    const at = (col, k) => (D.ai[k] >= 0 ? col[D.ai[k]] : NaN);
    runs.push({ name: fileName, n: D.n, free, ref, am: D.am, te: D.netto, ias: c.ias, utc: c.utc,
      roll: Float32Array.from({ length: D.n }, (_, k) => at(r.roll_deg, k)), pitch: Float32Array.from({ length: D.n }, (_, k) => at(r.pitch_deg, k)) });
    process.stderr.write(fileName + "\n");
  }
  const ok = cat(runs, (a, k) => a.free[k]), ref = cat(runs, (a, k) => a.ref[k]);
  const lag = { am: lagOf(cat(runs, (a, k) => a.am[k]), ref, ok), te: lagOf(cat(runs, (a, k) => a.te[k]), ref, ok) };
  const found = [];
  for (const a of runs) {
    let lastT = -1e9;
    for (let k = 60; k + 80 < a.n; k++) {
      // the IAS jump: the mean over 1 .. 2 s after minus the mean over the second before
      let i0 = 0, i1 = 0;
      for (let j = 0; j < 10; j++) { i0 += a.ias[k - 10 + j]; i1 += a.ias[k + 10 + j]; }
      const dIas = (i1 - i0) / 10 * 3.6;
      if (!(Math.abs(dIas) >= MIN_DIAS)) continue;
      // straight with a steady pitch (no pull-up), and the air's vertical movement steady (the reference), from 3 s
      // before to 6 s after
      let steady = true, pmin = Infinity, pmax = -Infinity, rmin = Infinity, rmax = -Infinity;
      for (let j = k - 30; j <= k + 60; j++) {
        if (!a.free[j] || !(Math.abs(a.roll[j]) < 10)) { steady = false; break; }
        pmin = Math.min(pmin, a.pitch[j]); pmax = Math.max(pmax, a.pitch[j]);
        rmin = Math.min(rmin, a.ref[j]); rmax = Math.max(rmax, a.ref[j]);
      }
      if (!steady || pmax - pmin > 2 || rmax - rmin > 1.5) continue;
      // the largest deviations from the reference within 0 .. +4 s, each series at its delay
      let te = 0, am = 0;
      for (let j = k; j <= k + 40; j++) {
        const dt = a.te[j + lag.te] - a.ref[j], da = a.am[j + lag.am] - a.ref[j];
        if (Math.abs(dt) > Math.abs(te)) te = dt;
        if (Math.abs(da) > Math.abs(am)) am = da;
      }
      if (!(Math.abs(te) >= MIN_TE)) continue;
      const t = a.utc[k];
      if (t - lastT < 60) continue; // one per minute
      lastT = t;
      found.push({ flight: a.name.replace(".nmea.gz", ""), utc: hms(t), dIas, te, am });
    }
  }
  found.sort((x, y) => (Math.abs(y.te) - Math.abs(y.am)) - (Math.abs(x.te) - Math.abs(x.am)));
  console.log(`${found.length} horizontal gusts (straight, pitch within 2° and the reference within 1.5 m/s over 9 s, IAS ±${MIN_DIAS} km/h, TE netto off the reference by ${MIN_TE} m/s or more); the top ${TOP}:`);
  console.log("flight             UTC        IAS change   TE netto off   AirMass off   (delays: AirMass " + lag.am / HZ + " s, TE " + lag.te / HZ + " s)");
  for (const f of found.slice(0, TOP)) {
    console.log(`${f.flight.padEnd(18)} ${f.utc}   ${(f.dIas >= 0 ? "+" : "") + f.dIas.toFixed(0)} km/h`.padEnd(45) +
      `${(f.te >= 0 ? "+" : "") + f.te.toFixed(1)} m/s`.padEnd(15) + `${(f.am >= 0 ? "+" : "") + f.am.toFixed(1)} m/s`);
  }
  const worse = found.filter((f) => Math.abs(f.am) >= Math.abs(f.te)).length;
  console.log(`AirMass deviated as much or more than TE netto in ${worse} of ${found.length}`);
})().catch((e) => { console.error(e); process.exit(1); });
