// The gust-insensitive GNSS reference with a wind from GNSS alone (a circle fit per full turn, page.js gnssCircleWind)
// instead of AirMass's averaged wind, so that it does not depend on AirMass: how the two references differ, AirMass's
// averaged wind against the GNSS-only one, and the cockpit AirMass vario and TE netto against both references.
//   node circle_ref.js <web dir> log1.nmea.gz ...   (CIRCLE='{"tas":true,"smooth":1}': gnssCircleWind options)
const fs = require("fs"), zlib = require("zlib"), path = require("path");
const { loadPage, lagOf, freeMask, gnssCircleWind, gnssNetWithWind, logOptions } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { convert, runReplay } = loadPage(web);
const HZ = 10;

(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const runs = [], wind = { n: 0, d2: 0, dn: 0, de: 0, tasRatio: [] }, turnRes = [];
  let turns = 0;
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = convert(zlib.gunzipSync(fs.readFileSync(log)).toString("latin1"), logOptions());
    D = build(conv, await runReplay(M, conv.rec, {}));
    derive(D);
    const r = D.rep.cols, c = D.tr, n = D.n;
    // the GNSS-only wind and reference
    const gw = gnssCircleWind(D, JSON.parse(process.env.CIRCLE || "{}"));
    turns += gw.turns.length;
    for (const t of gw.turns) {
      turnRes.push(t.res);
      const k = indexAt(t.t), i = D.ai[k];
      if (i >= 0 && r.status[i] & STATUS.WIND_VALID) {
        const dn = r.avg_wind_n[i] - t.n, de = r.avg_wind_e[i] - t.e;
        wind.n++; wind.d2 += dn * dn + de * de; wind.dn += dn; wind.de += de;
        wind.tasRatio.push(t.tas / (r.k_tas[i] * c.tas[k]));
      }
    }
    const refG = gnssNetWithWind(D, gw.wn, gw.we);
    const { free } = freeMask(D, (k) => D.amOk[k] && c.ias[k] > 60 / 3.6 && Number.isFinite(D.gnssNetInert[k]) && Number.isFinite(refG[k]) && Number.isFinite(D.netto[k]));
    runs.push({ n, free, roll: Float32Array.from(D.ai, (i) => (i >= 0 ? r.roll_deg[i] : NaN)), refs: { "AirMass wind": D.gnssNetInert, "GNSS-only wind": refG },
      ser: { "AirMass (cockpit)": D.am, "TE netto": D.netto } });
    process.stderr.write(fileName + "\n");
  }
  const med = (a) => a.slice().sort((x, y) => x - y)[a.length >> 1];
  console.log(`GNSS-only wind: ${turns} full turns fitted (median rms off the circle ${med(turnRes).toFixed(2)} m/s)`);
  console.log(`AirMass's averaged wind minus the GNSS-only one at ${wind.n} turns: rms ${Math.sqrt(wind.d2 / wind.n).toFixed(2)} m/s, mean north ${(wind.dn / wind.n).toFixed(2)}, east ${(wind.de / wind.n).toFixed(2)}`);
  console.log(`  the turns' radius (airspeed from GNSS alone) over AirMass's calibrated TAS: median ${med(wind.tasRatio).toFixed(3)}`);
  const cat = (f) => { const o = []; for (const a of runs) for (let k = 0; k < a.n; k++) o.push(f(a, k)); return o; };
  const ok = cat((a, k) => a.free[k]);
  for (const refName of Object.keys(runs[0].refs)) {
    const ref = cat((a, k) => a.refs[refName][k]);
    const lags = {}; for (const s in runs[0].ser) lags[s] = lagOf(cat((a, k) => a.ser[s][k]), ref, ok);
    const lagList = Object.values(lags), [sa, st] = Object.keys(runs[0].ser);
    console.log(`\n=== reference with ${refName} (delays: ${Object.entries(lags).map(([s, l]) => `${s} ${l / HZ} s`).join(", ")})`);
    for (const W of [10, 30]) {
      const acc = {};
      for (const a of runs) for (let k0 = 0; k0 + W * HZ + 60 < a.n; k0 += W * HZ) {
        let good = true, br = 0; const sum = {}; let rs = 0, other = 0;
        for (const s in a.ser) sum[s] = 0;
        for (let k = k0; k < k0 + W * HZ; k++) {
          if (!a.free[k] || lagList.some((l) => !a.free[k + l])) { good = false; break; }
          rs += a.refs[refName][k]; br += Math.abs(a.roll[k]);
          other += a.refs["AirMass wind"][k] - a.refs["GNSS-only wind"][k];
          for (const s in a.ser) sum[s] += a.ser[s][k + lags[s]];
        }
        if (!good) continue;
        br /= W * HZ;
        const ph = br < 10 ? "straight" : br > 25 ? "circling" : null;
        if (!ph) continue;
        for (const s in a.ser) { const e = (sum[s] - rs) / (W * HZ); const x = ((acc[ph] ||= {})[s] ||= { n: 0, s2: 0 }); x.n++; x.s2 += e * e; }
        // AirMass minus TE netto, for the three-cornered hat
        const dat = (sum[sa] - sum[st]) / (W * HZ), z = ((acc[ph] ||= {})["A-T"] ||= { n: 0, s2: 0 }); z.n++; z.s2 += dat * dat;
        const d = other / (W * HZ), y = ((acc[ph] ||= {})["refs differ"] ||= { n: 0, s2: 0 }); y.n++; y.s2 += d * d;
      }
      const ms = (p, s) => acc[p][s].s2 / acc[p][s].n;
      console.log(`  ${W} s windows, rms m/s:  ` + ["straight", "circling"].map((p) => `${p}: ` + Object.entries(acc[p]).filter(([s]) => s !== "A-T").map(([s, x]) => `${s} ${Math.sqrt(x.s2 / x.n).toFixed(3)}`).join(", ") + ` [${acc[p]["TE netto"].n}]`).join("   |   "));
      // three-cornered hat: AirMass, TE netto and the reference as three measurements with independent errors
      const hat = (p) => {
        const r2 = (ms(p, sa) + ms(p, st) - ms(p, "A-T")) / 2;
        const f = (v) => (v > 0 ? Math.sqrt(v).toFixed(3) : `(${v.toFixed(3)})`);
        return `${p}: AirMass ${f(ms(p, sa) - r2)}, TE netto ${f(ms(p, st) - r2)}, the reference itself ${f(r2)}`;
      };
      console.log(`    own errors (three-cornered hat):  ${hat("straight")}   |   ${hat("circling")}`);
    }
  }
})().catch((e) => { console.error(e); process.exit(1); });
