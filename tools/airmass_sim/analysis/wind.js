// The instant wind on the real flights: how much it moves in straight flight and while circling, and how much of its
// movement while circling turns with the glider (an estimation error, not air). The deviation of the instant wind from
// its 60 s centred mean, in the glider's heading frame: a constant part along the heading is an airspeed scale error
// (Δk ≈ along / TAS), across it a heading or sideslip offset (same sign in left and right circles) or a GNSS timing
// error (sign flips with the turn direction: V·ω·Δt).
//   node wind.js <web dir> log1.nmea.gz ...   (SETTINGS="name=value,...")
const fs = require("fs"), zlib = require("zlib"), path = require("path");
const { loadPage, freeMask, logOptions } = require("./page.js");
const web = path.resolve(process.argv[2]);
const { convert, runReplay } = loadPage(web);
const HZ = 10, HALF = 30 * HZ;

(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  const acc = {}; // per phase: sums
  const get = (p) => (acc[p] ||= { n: 0, dev2: 0, al: 0, cr: 0, al2: 0, cr2: 0, mv2: 0, mvn: 0, m22: 0, m2n: 0 });
  const perFlight = [], hb = {};
  for (const log of process.argv.slice(3)) {
    fileName = path.basename(log);
    const conv = convert(zlib.gunzipSync(fs.readFileSync(log)).toString("latin1"), logOptions());
    D = build(conv, await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" }));
    derive(D);
    const r = D.rep.cols, c = D.tr, n = D.n;
    const ok = (k) => D.ai[k] >= 0 && (r.status[D.ai[k]] & STATUS.WIND_VALID) && c.ias[k] > 60 / 3.6;
    const { free } = freeMask(D, ok);
    const wn = (k) => r.wind_n[D.ai[k]], we = (k) => r.wind_e[D.ai[k]];
    // centred 60 s means over the free samples
    const sn = new Float64Array(n + 1), se = new Float64Array(n + 1), sc = new Float64Array(n + 1);
    for (let k = 0; k < n; k++) { const f = free[k]; sn[k + 1] = sn[k] + (f ? wn(k) : 0); se[k + 1] = se[k] + (f ? we(k) : 0); sc[k + 1] = sc[k] + f; }
    // first pass: the means along / across per phase and turn direction, for the residual in the second
    const samples = [];
    for (let k = 0; k < n; k++) {
      if (!free[k]) continue;
      const a = Math.max(0, k - HALF), b = Math.min(n, k + HALF), m = sc[b] - sc[a];
      if (m < HALF) continue;
      const dn = wn(k) - (sn[b] - sn[a]) / m, de = we(k) - (se[b] - se[a]) / m;
      const i = D.ai[k], psi = r.heading_deg[i] * Math.PI / 180, roll = r.roll_deg[i];
      const al = dn * Math.cos(psi) + de * Math.sin(psi), cr = -dn * Math.sin(psi) + de * Math.cos(psi);
      const bank = Math.abs(roll);
      const ph = bank < 10 ? "straight" : bank > 25 ? (roll > 0 ? "circling right" : "circling left") : null;
      if (!ph) continue;
      const k10 = k + 10 * HZ, mv = k10 < n && free[k10] ? Math.hypot(wn(k10) - wn(k), we(k10) - we(k)) : NaN;
      const k2 = k + 2 * HZ, mv2 = k2 < n && free[k2] ? Math.hypot(wn(k2) - wn(k), we(k2) - we(k)) : NaN;
      // the heading relative to the 60 s mean wind (0: into the wind)
      const wdir = Math.atan2(-(se[b] - se[a]) / m, -(sn[b] - sn[a]) / m); // where the wind comes from
      samples.push({ ph, al, cr, dn, de, psi, rel: psi - wdir, mv, mv2 });
    }
    const fl = {};
    // heading locked part while circling: the mean deviation per 30° of heading (absolute, and relative to the wind),
    // per flight; its share of the deviation's variance
    for (const key of ["psi", "rel"]) {
      const bins = Array.from({ length: 12 }, () => ({ n: 0, dn: 0, de: 0 }));
      for (const s of samples) {
        if (!s.ph.startsWith("circling")) continue;
        const j = Math.floor((((s[key] * 180 / Math.PI) % 360) + 360) % 360 / 30);
        bins[j].n++; bins[j].dn += s.dn; bins[j].de += s.de;
      }
      const h = (hb[key] ||= { expl: 0 });
      for (const bn of bins) if (bn.n) h.expl += (bn.dn * bn.dn + bn.de * bn.de) / bn.n;
    }
    for (const s of samples) {
      for (const p of [s.ph, s.ph.startsWith("circling") ? "circling" : null]) {
        if (!p) continue;
        const x = get(p); x.n++; x.dev2 += s.al * s.al + s.cr * s.cr; x.al += s.al; x.cr += s.cr; x.al2 += s.al * s.al; x.cr2 += s.cr * s.cr;
        if (Number.isFinite(s.mv)) { x.mv2 += s.mv * s.mv; x.mvn++; }
        if (Number.isFinite(s.mv2)) { x.m22 += s.mv2 * s.mv2; x.m2n++; }
        const y = (fl[p] ||= { n: 0, al: 0, cr: 0 }); y.n++; y.al += s.al; y.cr += s.cr;
      }
    }
    perFlight.push([fileName, fl, D.rep.summary]);
    process.stderr.write(fileName + "\n");
  }
  console.log("instant wind minus its 60 s centred mean, in the heading frame (along: + = more tailwind than the mean along the heading)");
  console.log("phase            samples  rms dev  moves in 10 s   mean along  mean across   rms along  rms across   left after removing the means");
  for (const p of ["straight", "circling", "circling left", "circling right"]) {
    const x = acc[p]; if (!x) continue;
    const ma = x.al / x.n, mc = x.cr / x.n;
    const ra = Math.sqrt(x.al2 / x.n), rc = Math.sqrt(x.cr2 / x.n);
    const left = Math.sqrt((x.al2 / x.n - ma * ma) + (x.cr2 / x.n - mc * mc));
    console.log(`${p.padEnd(16)} ${String(x.n).padStart(7)}   ${Math.sqrt(x.dev2 / x.n).toFixed(2)}     ${Math.sqrt(x.mv2 / x.mvn).toFixed(2)}          ${ma.toFixed(2).padStart(6)}      ${mc.toFixed(2).padStart(6)}       ${ra.toFixed(2)}       ${rc.toFixed(2)}        ${left.toFixed(2)}`);
  }
  console.log("\nmoves within 2 s: " + ["straight", "circling"].map((p) => `${p} ${Math.sqrt(acc[p].m22 / acc[p].m2n).toFixed(2)}`).join(", "));
  const cv = acc.circling.dev2;
  console.log(`circling, share of the deviation's variance locked to the heading (30° bins per flight): absolute ${(100 * hb.psi.expl / cv).toFixed(0)} %, relative to the wind ${(100 * hb.rel.expl / cv).toFixed(0)} %` +
    ` (rms ${Math.sqrt(hb.psi.expl / acc.circling.n).toFixed(2)} / ${Math.sqrt(hb.rel.expl / acc.circling.n).toFixed(2)} m/s)`);
  console.log("\nper flight, circling left / right: mean along, across (m/s); learned k_tas, GNSS latency");
  for (const [f, fl, s] of perFlight) {
    const v = (p) => (fl[p] ? `${(fl[p].al / fl[p].n).toFixed(2).padStart(5)} ${(fl[p].cr / fl[p].n).toFixed(2).padStart(5)}` : "   –     –  ");
    console.log(`${f.padEnd(28)} L ${v("circling left")}   R ${v("circling right")}   k_tas ${s.k_tas.toFixed(3)}  latency ${s.latency_ms} ms`);
  }
})().catch((e) => { console.error(e); process.exit(1); });
