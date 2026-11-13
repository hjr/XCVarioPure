// A restart in flight (as an instrument replay or a seek does it) against the uninterrupted estimator, by time since
// the restart: node restart.js LOG... (MODULE=other airmass.js, SETTINGS)
// A: the attitude from the accelerometer alone (what a just reset AHRS gives), B: the true attitude of the moment,
// C: the accelerometer with the turn's centripetal acceleration taken out (airmass::downFromMotion, without the AoA).
const path = require('path');
const R = path.resolve(__dirname, '..');
const { loadPage } = require('./page.js');
const { runReplay, loadFlight } = loadPage(R + '/web');
const { checksum } = require(R + '/web/convert.js');
const { STATUS } = require(R + '/web/replay.js');
const bins = [5, 10, 15, 20, 30, 45, 60, 90, 120, 180];
const acc = () => ({ sq: bins.map(() => 0), n: bins.map(() => 0), invalid: bins.map(() => 0), max: bins.map(() => 0) });
(async () => {
  const M = await require(process.env.MODULE || R + '/web/airmass.js')();
  const settings = process.env.SETTINGS || '';
  const stats = { A: acc(), B: acc(), C: acc() }, bank = { A: acc(), B: acc(), C: acc() };
  let starts = 0;
  for (const file of process.argv.slice(2)) {
    const rec = loadFlight(file, { glider: 'log' }).rec;
    const base = await runReplay(M, rec, { settings });
    const row = new Map(); for (let i = 0; i < base.n; i++) row.set(base.t_ms[i], i);
    const frames = []; rec.forEach((l, i) => { if (l.startsWith('$PXCVR,')) frames.push(i); });
    for (let f = 12000; f + 2000 < frames.length; f += 6000) {
      const li = frames[f], p = rec[li].split('*')[0].split(','), t = +p[1], i0 = row.get(t);
      if (i0 === undefined || !(base.cols.status[i0] & STATUS.VARIO_VALID)) continue;
      const c = base.cols, fx = [+p[2], +p[3], +p[4]], fn = Math.hypot(...fx);
      const roll = c.roll_deg[i0] * Math.PI / 180, pitch = c.pitch_deg[i0] * Math.PI / 180;
      const down = { A: fx.map(v => -v / fn), B: [-Math.sin(pitch), Math.sin(roll) * Math.cos(pitch), Math.cos(roll) * Math.cos(pitch)] };
      { const G = 9.80665, V = +p[9], w = [+p[5], +p[6], +p[7]], g = [0 - fx[0] * G, w[2] * V - fx[1] * G, -w[1] * V - fx[2] * G], n = Math.hypot(...g); down.C = g.map(x => x / n); }
      const warm = [c.k_tas[i0].toFixed(5), (c.pitch_ofs_deg[i0] * Math.PI / 180).toFixed(6), (c.latency_ms[i0] / 1000).toFixed(4), '1.00000', ...Array(7).fill('1.000000'), '2'];
      starts++;
      for (const v of ['A', 'B', 'C']) {
        const st = checksum(['$PXCVS', t, p[10], ...down[v].map(x => x.toFixed(6)), p[5], p[6], p[7], ...warm].join(','));
        const part = [rec[0], ...rec.slice(1, li).filter(l => l.startsWith('$PXCVC')).slice(-1), st, ...rec.slice(li, frames[f + 1900])];
        const rep = await runReplay(M, part, { settings });
        const circling = Math.abs(c.roll_deg[i0]) > 20 ? bank : stats;
        for (let i = 0; i < rep.n; i++) {
          const s = (rep.t_ms[i] - t) / 1000, j = row.get(rep.t_ms[i]), b = bins.findIndex(x => s <= x);
          if (b < 0 || j === undefined || !(c.status[j] & STATUS.VARIO_VALID)) continue;
          if (!(rep.cols.status[i] & STATUS.VARIO_VALID)) { circling[v].invalid[b]++; continue; }
          const d = rep.cols.vario[i] - c.vario[j];
          circling[v].sq[b] += d * d; circling[v].n[b]++; circling[v].max[b] = Math.max(circling[v].max[b], Math.abs(d));
        }
      }
    }
  }
  console.log('restarts', starts, settings);
  for (const [name, g] of [['restart in straight flight', stats], ['restart while banked over 20 deg', bank]]) {
    console.log(name + '\nuntil s   A rms   max  invalid   B rms   max  invalid   C rms   max  invalid');
    bins.forEach((b, i) => console.log(String(b).padStart(7), ...['A', 'B', 'C'].flatMap(v => [(g[v].n[i] ? Math.sqrt(g[v].sq[i] / g[v].n[i]) : NaN).toFixed(2).padStart(7),
      g[v].max[i].toFixed(1).padStart(5), ((100 * g[v].invalid[i] / Math.max(1, g[v].invalid[i] + g[v].n[i])).toFixed(0) + '%').padStart(7)])));
  }
})();
