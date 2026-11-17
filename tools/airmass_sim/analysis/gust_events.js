// The gust detector (AirMassGust.h) on the real flights: events per hour of straight flight (|roll| < 10°, vario valid),
// split into head wind increase and drop, and for each event the IAS change over it and the pitch range around it
// (a pull-up changes the airspeed with the pitch, a gust without).
//   node gust_events.js <web dir> log1.nmea.gz ...   (LIST=1: every event)
const path = require("path");
const { loadPage, hms } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { runReplay, loadFlight } = loadPage(web);

(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  let hours = 0, up = 0, down = 0, pullups = 0;
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = loadFlight(log);
    const rep = await runReplay(M, conv.rec, {});
    const D = build(conv, rep);
    const r = rep.cols, t = rep.t_ms, n = rep.n;
    const ias = new Float32Array(n).fill(NaN), utc = new Float64Array(n).fill(NaN);
    for (let k = 0; k < D.n; k++) if (D.ai[k] >= 0) { ias[D.ai[k]] = D.tr.ias[k]; utc[D.ai[k]] = D.tr.utc[k]; }
    const flying = (i) => (r.status[i] & STATUS.VARIO_VALID) && ias[i] > 60 / 3.6;
    for (let i = 1; i < n; i++) if (!Number.isFinite(ias[i])) ias[i] = ias[i - 1];
    let straight = 0;
    for (let i = 1; i < n; i++) {
      if (flying(i) && Math.abs(r.roll_deg[i]) < 10) straight += Math.min(t[i] - t[i - 1], 300);
    }
    hours += straight / 3.6e6;
    const at = (i, dt) => { let j = i; while (j > 0 && j < n - 1 && Math.abs(t[j] - t[i]) < Math.abs(dt)) j += Math.sign(dt); return j; };
    let fUp = 0, fDown = 0;
    for (let i = 0; i < n; i++) {
      const e = r.gust_event[i];
      if (!e) continue;
      e > 0 ? (++up, ++fUp) : (++down, ++fDown);
      let pmin = Infinity, pmax = -Infinity;
      for (let j = at(i, -4000); j <= at(i, 2000); j++) { pmin = Math.min(pmin, r.pitch_deg[j]); pmax = Math.max(pmax, r.pitch_deg[j]); }
      const dIas = (ias[at(i, 1000)] - ias[at(i, -3000)]) * 3.6;
      if (pmax - pmin > 5) ++pullups;
      if (process.env.LIST) {
        console.log(`  ${fileName.padEnd(26)} ${Number.isFinite(utc[i]) ? hms(utc[i]) : "--:--:--"}  ${e > 0 ? "+" : "-"}  IAS ${dIas >= 0 ? "+" : ""}${dIas.toFixed(0)} km/h  pitch range ${(pmax - pmin).toFixed(1)}°  roll ${r.roll_deg[i].toFixed(0)}°`);
      }
    }
    console.log(`${fileName.padEnd(26)} straight ${(straight / 3.6e6).toFixed(2)} h, gusts +${fUp} -${fDown}`);
  }
  console.log(`all: straight ${hours.toFixed(2)} h, head wind increase ${up} (${(up / hours).toFixed(1)}/h), drop ${down} (${(down / hours).toFixed(1)}/h), ` +
    `with a pitch range over 5° (pull-up?) ${pullups}`);
})().catch((e) => { console.error(e); process.exit(1); });
