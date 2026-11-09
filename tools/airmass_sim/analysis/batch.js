// Current browser replay on plain or gzipped logs; per-flight comparisons and data quality.
// node batch.js OUTPUT.json LOG...
const fs = require('fs'), path = require('path'), zlib = require('zlib');
const crypto = require('crypto');
const { loadPage, lagOf, dampSeries, freeMask, gnssCircleWind } = require('./page.js');
const web = path.resolve(__dirname, '../web');
const { convert, runReplay } = loadPage(web);
const quantile = (v, p) => {
  const a = v.filter(Number.isFinite).sort((x, y) => x - y);
  return a.length ? a[Math.floor((a.length - 1) * p)] : null;
};
const clock = (u) => Number.isFinite(u) ? [Math.floor(u / 3600), Math.floor(u / 60) % 60, Math.floor(u) % 60].map(x => String(x).padStart(2, '0')).join(':') : null;
async function evaluate(M, file) {
  const bytes = fs.readFileSync(file);
  const conv = convert((file.endsWith('.gz') ? zlib.gunzipSync(bytes) : bytes).toString('latin1'), { glider: 'log' });
  const replay = await runReplay(M, conv.rec, {});
  if (!conv.track.n || !replay.n) return { file: path.basename(file), bytes: bytes.length,
    sha256: crypto.createHash('sha256').update(bytes).digest('hex'), replayable: false,
    reason: conv.track.n ? 'No usable airborne initialization' : 'No sensor samples', summary: replay.summary };
  globalThis.fileName = path.basename(file);
  globalThis.D = build(conv, replay);
  derive(D);
  const c = D.tr, r = D.rep.cols, n = D.n;
  const { free } = freeMask(D, k => c.ias[k] > 60 / 3.6 && c.gnss_speed[k] > 10);
  const airborne = freeMask(D, k => c.ias[k] > 60 / 3.6).free; // availability without a GNSS prerequisite
  const ok = Uint8Array.from(free, (v, k) => v && D.amOk[k] && Number.isFinite(D.netto[k]) && Number.isFinite(D.gnssNetInert[k]));
  const shown = { AirMass: D.am, TE: D.netto }, equal = { AirMass: dampSeries(D.amRaw, ok), TE: D.netto };
  // Score on a time grid: 100 sensor rows are not necessarily ten seconds in these logs.
  // Interpolate only across short measured intervals, never across a sensor outage.
  const gridN = Math.floor((D.t[n-1] - D.t[0]) * 10) + 1;
  const regular = {};
  const source = { ...equal, cockpitAirMass: D.am, inertial: D.gnssNetInert, pitot: D.gnssNetPitot };
  for (const name of Object.keys(source)) regular[name] = new Float32Array(gridN).fill(NaN);
  const gridOk = new Uint8Array(gridN), gridRoll = new Float32Array(gridN), gridUtc = new Float64Array(gridN);
  for (let j = 0, a = 0; j < gridN; j++) {
    const t = D.t[0] + j / 10;
    while (a+1 < n && D.t[a+1] <= t) a++;
    gridUtc[j] = c.utc[a];
    if (a+1 >= n || D.t[a+1]-D.t[a] > 0.3 || !ok[a] || !ok[a+1]) continue;
    const f = (t-D.t[a])/(D.t[a+1]-D.t[a]);
    for (const [name, s] of Object.entries(source)) regular[name][j] = s[a] + f*(s[a+1]-s[a]);
    gridOk[j] = 1; gridRoll[j] = r.roll_deg[D.ai[a]];
  }
  const results = {};
  const score = (mask, ref, series, lags) => {
    const scores = {};
    for (const W of [10, 30, 300]) {
      const bins = { all: [], straight: [], circling: [] };
      for (let k = 0; k + W * 10 + 60 < gridN; k += W * 10) {
        let good = true, bank = 0, sr = 0, sa = 0, st = 0;
        for (let j = k; j < k + W * 10; j++) {
          if (!mask[j] || !mask[j + lags.AirMass] || !mask[j + lags.TE]) { good = false; break; }
          bank += Math.abs(gridRoll[j]); sr += ref[j];
          sa += series.AirMass[j + lags.AirMass]; st += series.TE[j + lags.TE];
        }
        if (!good) continue;
        const a = { t: clock(gridUtc[k]), AirMass: (sa - sr) / (W * 10), TE: (st - sr) / (W * 10) };
        bins.all.push(a);
        bank /= W * 10;
        if (bank < 10) bins.straight.push(a); else if (bank > 25) bins.circling.push(a);
      }
      scores[W] = Object.fromEntries(Object.entries(bins).map(([phase, b]) => [phase, {
        windows: b.length,
        ...Object.fromEntries(['AirMass', 'TE'].map(s => [s, { rms: b.length ? Math.sqrt(b.reduce((v,x) => v + x[s] ** 2,0) / b.length) : null,
          bias: b.length ? b.reduce((v,x) => v + x[s],0) / b.length : null }]))
      }]));
      if (W === 10) scores.worst = bins.all.sort((a,b) => Math.abs(b.AirMass) - Math.abs(a.AirMass)).slice(0,5);
    }
    return scores;
  };
  for (const refName of ['inertial', 'pitot']) {
    const ref = regular[refName];
    const mask = Uint8Array.from(gridOk, (v,k) => v && Number.isFinite(ref[k]) && Number.isFinite(regular.AirMass[k]) && Number.isFinite(regular.TE[k]));
    const lags = Object.fromEntries(['AirMass', 'TE'].map(name => [name, lagOf(regular[name], ref, mask)]));
    const cockpit = { AirMass: regular.cockpitAirMass, TE: regular.TE };
    const cockpitDelay = {};
    for (const phase of ['all', 'straight', 'circling']) {
      const phaseMask = Uint8Array.from(mask, (v,k) => v && (phase === 'all' ||
        (phase === 'straight' ? Math.abs(gridRoll[k]) < 10 : Math.abs(gridRoll[k]) > 25)));
      cockpitDelay[phase] = { samples: phaseMask.reduce((s,v) => s + v, 0),
        ...Object.fromEntries(Object.entries(cockpit).map(([name, series]) => [name, lagOf(series, ref, phaseMask) / 10])) };
    }
    results[refName] = { delay: Object.fromEntries(Object.entries(lags).map(([s,l]) => [s,l / 10])),
      scores: score(mask, ref, regular, lags),
      cockpitDelay,
      cockpitUnshifted: score(mask, ref, cockpit, { AirMass: 0, TE: 0 }) };
  }
  const status = { samples: 0, vario: 0, wind: 0, degraded: 0, fresh: 0, aligned: 0 }, windSpeed = [], beta = [];
  const airborneStatus = { samples: 0, vario: 0, wind: 0, degraded: 0, fresh: 0, aligned: 0 };
  let freeSeconds = 0, scoredSeconds = 0, airborneSeconds = 0;
  for (let k = 0; k < n; k++) {
    const dt = k ? Math.min(0.5, Math.max(0, D.t[k] - D.t[k-1])) : 0;
    freeSeconds += free[k] * dt; scoredSeconds += ok[k] * dt;
    if (airborne[k] && D.ai[k] >= 0) {
      const s = r.status[D.ai[k]];
      airborneSeconds += dt;
      airborneStatus.samples++;
      for (const [name, bit] of Object.entries({ vario: 1, wind: 2, fresh: 4, aligned: 8, degraded: 16 })) if (s & bit) airborneStatus[name]++;
    }
    if (!free[k] || D.ai[k] < 0) continue;
    const i = D.ai[k], s = r.status[i]; status.samples++;
    for (const [name, bit] of Object.entries({ vario: 1, wind: 2, fresh: 4, aligned: 8, degraded: 16 })) if (s & bit) status[name]++;
    if (s & 2) windSpeed.push(Math.hypot(r.wind_n[i], r.wind_e[i]));
    beta.push(Math.abs(r.slip_beta_deg[i]));
  }
  const motion = {};
  for (const [name,s] of Object.entries(shown)) {
    const gridSeries = name === 'AirMass' ? regular.cockpitAirMass : regular.TE;
    let sum = 0, count = 0;
    for (let k = 0; k + 10 < gridN; k += 5) if (gridOk[k] && gridOk[k+10] && Number.isFinite(gridSeries[k]) && Number.isFinite(gridSeries[k+10])) {
      sum += (gridSeries[k+10]-gridSeries[k])**2; count++;
    }
    motion[name] = count ? Math.sqrt(sum/count) : null;
  }
  const circles = gnssCircleWind(D), errors = [], avgErrors = [];
  for (const turn of circles.turns) {
    const k = indexAt(turn.t), i = D.ai[k];
    if (i < 0 || !free[k] || !(r.status[i]&2) || turn.res > 1) continue;
    errors.push(Math.hypot(turn.n - r.wind_n[i], turn.e - r.wind_e[i]));
    avgErrors.push(Math.hypot(turn.n - r.avg_wind_n[i], turn.e - r.avg_wind_e[i]));
  }
  let gaps = 0, gapSeconds = 0, maxGap = 0, altitudeJumps = 0;
  for (let j = 1; j < D.fixes.length; j++) {
    const a = D.fixes[j-1], b = D.fixes[j], dt = b.u-a.u;
    if (dt > 3) { gaps++; gapSeconds += dt; maxGap = Math.max(maxGap,dt); }
    if (dt > 0 && dt <= 3 && Math.abs(b.alt-a.alt)/dt > 30) altitudeJumps++;
  }
  let sensorGaps = 0, rejectedGnss = 0;
  const extremes = { raw: null, valid: null, displayed: null };
  for (let k = 0; k < n; k++) {
    if (k && D.t[k] - D.t[k-1] > 0.30001) sensorGaps++;
    const i = D.ai[k];
    if (i < 0) continue;
    if (i) rejectedGnss += Math.max(0, r.gnss_rejects[i] - r.gnss_rejects[i-1]);
    for (const [name, value] of Object.entries({ raw: r.vario[i], valid: D.amOk[k] ? r.vario[i] : NaN, displayed: D.am[k] })) {
      if (Number.isFinite(value) && (!extremes[name] || Math.abs(value) > Math.abs(extremes[name].value))) extremes[name] = {
        utc: clock(c.utc[k]), elapsedSeconds: D.t[k] - D.tStart, value, status: r.status[i], iasKmh: c.ias[k] * 3.6,
        tasKmh: c.tas[k] * 3.6, rollDeg: r.roll_deg[i], pitchDeg: r.pitch_deg[i] };
    }
  }
  return { file: path.basename(file), bytes: bytes.length, sha256: crypto.createHash('sha256').update(bytes).digest('hex'),
    replayable: true, glider: conv.glider, gliderFrom: conv.gliderFrom, mass: conv.mass,
    samples: n, hours: (D.t[n-1]-D.t[0])/3600, freeHours: freeSeconds/3600, scoredHours: scoredSeconds/3600,
    temperatureEstimatedSamples: conv.temperatureEstimatedSamples, temperatureKelvinSamples: conv.temperatureKelvinSamples,
    axesCorrected: conv.axesCorrected, axesInferred: conv.axesInferred,
    sensorGapsOver300ms: sensorGaps, rejectedGnss, extremes, status, airborneStatus, airborneHours: airborneSeconds/3600, summary: D.rep.summary,
    gnss: { fixes: D.fixes.length, gapsOver3s: gaps, gapSeconds, maxGap, altitudeJumpsOver30mps: altitudeJumps },
    vario: results, needleMotion1s: motion,
    wind: { validSpeedMedian: quantile(windSpeed,0.5), validSpeedP95: quantile(windSpeed,0.95), independentTurns: errors.length,
      vectorErrorMedian: quantile(errors,0.5), vectorErrorP90: quantile(errors,0.9), avgVectorErrorMedian: quantile(avgErrors,0.5) },
    absoluteSideslipMedian: quantile(beta,0.5), absoluteSideslipP95: quantile(beta,0.95) };
}
(async () => {
  const M = await require(path.join(web,'airmass.js'))();
  const implementation = Object.fromEntries(['convert.js', 'airmass.js', 'index.html', 'replay.js', '../analysis/page.js', '../analysis/batch.js'].map(name => [name,
    crypto.createHash('sha256').update(fs.readFileSync(path.join(web, name))).digest('hex')]));
  const output = { created: new Date().toISOString(), implementation,
    notes: 'Default AirMass, Mahony off, logged glider/weight, validated/inferred axes, Kelvin corrections and temperature fallback. Same 4 s damping for main scores; each series shifted by its fitted 0–6 s delay. CockpitUnshifted uses displayed AirMass damping without time shifts. Reference is not truth. Free-flight mask is heuristic. status retains IAS/GS eligibility for historical comparisons; airborneStatus removes the GNSS requirement. Vario scoring requires AirMass-valid reference overlap. NeedleMotion1s now uses the regular time grid, not row offsets.', flights: [] };
  for (const file of process.argv.slice(3)) {
    const row = await evaluate(M,file); output.flights.push(row);
    fs.writeFileSync(process.argv[2],JSON.stringify(output,null,2)+'\n');
    console.log(row.file, row.replayable ? JSON.stringify({glider: row.glider, hours: row.scoredHours, error10: row.vario.inertial.scores[10].all, wind: row.wind}) : row.reason);
  }
})().catch(e => { console.error(e); process.exitCode = 1; });
