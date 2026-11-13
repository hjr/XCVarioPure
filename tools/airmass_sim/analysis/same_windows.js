// Two builds of the estimator on the same scored windows: node same_windows.js OUT.json A.js B.js LOG...
// A window counts when both replays, their references and TE netto are usable in it, at every fitted delay.
// apart: rms between the two replays' series, sample by sample. The inertial GNSS reference takes the wind of the
// replay it is compared with: REF=pitot scores against the pitot reference, REF=first both against the first replay's.
const fs = require('fs'), path = require('path');
const { loadPage, lagOf, dampSeries, freeMask } = require('./page.js');
const web = path.resolve(__dirname, '../web');
const { runReplay, loadFlight } = loadPage(web);
const NAMES = ['AirMass', 'TE', 'cockpit', 'ref'];

async function grid(M, conv) {
  const replay = await runReplay(M, conv.rec, {});
  if (!conv.track.n || !replay.n) return null;
  globalThis.D = build(conv, replay);
  derive(D);
  const c = D.tr, r = D.rep.cols, n = D.n;
  const { free } = freeMask(D, k => c.ias[k] > 60 / 3.6 && c.gnss_speed[k] > 10);
  const ok = Uint8Array.from(free, (v, k) => v && D.amOk[k] && Number.isFinite(D.netto[k]) && Number.isFinite(D.gnssNetInert[k]));
  const source = { AirMass: dampSeries(D.amRaw, ok), TE: D.netto, cockpit: D.am, ref: process.env.REF === 'pitot' ? D.gnssNetPitot : D.gnssNetInert };
  const N = Math.floor((D.t[n - 1] - D.t[0]) * 10) + 1;
  const g = { N, mask: new Uint8Array(N), roll: new Float32Array(N) };
  for (const name of NAMES) g[name] = new Float32Array(N).fill(NaN);
  for (let j = 0, a = 0; j < N; j++) {
    const t = D.t[0] + j / 10;
    while (a + 1 < n && D.t[a + 1] <= t) a++;
    if (a + 1 >= n || D.t[a + 1] - D.t[a] > 0.3 || !ok[a] || !ok[a + 1]) continue;
    const f = (t - D.t[a]) / (D.t[a + 1] - D.t[a]);
    for (const name of NAMES) g[name][j] = source[name][a] + f * (source[name][a + 1] - source[name][a]);
    g.mask[j] = NAMES.every(name => Number.isFinite(g[name][j]));
    g.roll[j] = r.roll_deg[D.ai[a]];
  }
  return g;
}

function score(runs, mask, W, shifted) {
  const N = mask.length, bins = { all: [], straight: [], circling: [] };
  const lags = runs.map(g => shifted ? { AirMass: lagOf(g.AirMass, g.ref, mask), TE: lagOf(g.TE, g.ref, mask) } : { AirMass: 0, TE: 0 });
  const offsets = [...new Set([0, ...lags.flatMap(l => [l.AirMass, l.TE])])];
  for (let k = 0; k + W * 10 + 60 < N; k += W * 10) {
    let good = true, bank = 0;
    const sums = runs.map(() => [0, 0]);
    for (let j = k; j < k + W * 10 && good; j++) {
      good = offsets.every(o => mask[j + o]);
      if (!good) break;
      bank += Math.abs(runs[0].roll[j]);
      runs.forEach((g, i) => {
        sums[i][0] += g[shifted ? 'AirMass' : 'cockpit'][j + lags[i].AirMass] - g.ref[j];
        sums[i][1] += g.TE[j + lags[i].TE] - g.ref[j];
      });
    }
    if (!good) continue;
    const e = sums.map(s => s.map(v => v / (W * 10)));
    bins.all.push(e);
    bank /= W * 10;
    if (bank < 10) bins.straight.push(e); else if (bank > 25) bins.circling.push(e);
  }
  return Object.fromEntries(Object.entries(bins).map(([phase, b]) => [phase, { windows: b.length,
    sq: runs.map((_, i) => [0, 1].map(s => b.reduce((v, e) => v + e[i][s] ** 2, 0))) }]));
}

(async () => {
  const [out, a, b, ...logs] = process.argv.slice(2);
  const modules = [await require(path.resolve(a))(), await require(path.resolve(b))()];
  const flights = [];
  for (const file of logs) {
    const conv = loadFlight(file, { glider: 'log' });
    const runs = [];
    for (const M of modules) runs.push(await grid(M, conv));
    if (runs.some(g => !g)) continue;
    const apartRef = runs.map(g => g.ref);
    if (process.env.REF === 'first') runs[1].ref = runs[0].ref;
    const mask = Uint8Array.from(runs[0].mask, (v, k) => v && runs[1].mask[k]);
    const own = runs.map(g => g.mask.reduce((s, v) => s + v, 0) / 36000);
    const apart = Object.fromEntries(NAMES.map(name => {
      let n = 0, sq = 0;
      for (let k = 0; k < mask.length; k++) if (mask[k]) { n++; sq += ((name === 'ref' ? apartRef[0] : runs[0][name])[k] - (name === 'ref' ? apartRef[1] : runs[1][name])[k]) ** 2; }
      return [name, n ? Math.sqrt(sq / n) : null];
    }));
    const row = { file: path.basename(file), ownHours: own, commonHours: mask.reduce((s, v) => s + v, 0) / 36000, apart,
      equal10: score(runs, mask, 10, true), equal30: score(runs, mask, 30, true), cockpit10: score(runs, mask, 10, false) };
    flights.push(row);
    fs.writeFileSync(out, JSON.stringify({ a, b, flights }, null, 1));
    const rms = (s, i, j) => s.all.windows ? Math.sqrt(s.all.sq[i][j] / s.all.windows).toFixed(3) : '-';
    console.log(row.file.padEnd(26), own.map(h => h.toFixed(2)).join('/'), row.commonHours.toFixed(2).padStart(5), String(row.equal10.all.windows).padStart(5),
      `A ${rms(row.equal10, 0, 0)}/${rms(row.equal10, 0, 1)}  B ${rms(row.equal10, 1, 0)}/${rms(row.equal10, 1, 1)}`,
      ' apart ' + NAMES.map(name => `${name} ${apart[name] == null ? '-' : apart[name].toFixed(3)}`).join(' '));
  }
  for (const key of ['equal10', 'equal30', 'cockpit10']) {
    console.log(key.padEnd(10), ['all', 'straight', 'circling'].map(phase => {
      let n = 0; const sq = [[0, 0], [0, 0]];
      for (const f of flights) { n += f[key][phase].windows; f[key][phase].sq.forEach((r, i) => r.forEach((v, j) => sq[i][j] += v)); }
      const v = (i, j) => Math.sqrt(sq[i][j] / n).toFixed(3);
      return `${phase} n=${n} A ${v(0, 0)}/${v(0, 1)} B ${v(1, 0)}/${v(1, 1)}`;
    }).join(' | '));
  }
})().catch(e => { console.error(e); process.exitCode = 1; });
