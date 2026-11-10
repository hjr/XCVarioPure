// Per-flight table and pooled scores from batch.js outputs: node table.js OUT.json... (EXCLUDE=file,file: left out of the pools)
const fs = require('fs');
const rows = [];
for (const f of process.argv.slice(2)) for (const r of JSON.parse(fs.readFileSync(f)).flights) rows.push(r);
rows.sort((a, b) => a.file.localeCompare(b.file));
const f2 = (x) => x == null ? '   -  ' : x.toFixed(3).padStart(6);
const pct = (s, k) => s.samples ? (100 * s[k] / s.samples).toFixed(0).padStart(3) + '%' : '  - ';
console.log('file                      glider          hrs  N10   eq10 AM/TE  str AM/TE   circ AM/TE   eq30 AM/TE   cock10 AM/TE  biasS/biasC(AM)  vario wind  gaps  windErr');
let pool = { all: [0, 0, 0], straight: [0, 0, 0], circling: [0, 0, 0] }, pool30 = { all: [0, 0, 0], straight: [0, 0, 0], circling: [0, 0, 0] }, poolC = { all: [0, 0, 0], straight: [0, 0, 0], circling: [0, 0, 0] };
const exclude = new Set((process.env.EXCLUDE || '').split(',').filter(Boolean));
for (const r of rows) {
  if (!r.replayable) { console.log(r.file.padEnd(26), r.reason); continue; }
  const s = r.vario.inertial.scores, c = r.vario.inertial.cockpitUnshifted;
  const g = (sc, w, p) => sc[w][p];
  const line = [r.file.padEnd(26), (r.glider || '').padEnd(15), r.scoredHours.toFixed(2).padStart(5), String(g(s, 10, 'all').windows).padStart(5),
    f2(g(s, 10, 'all').AirMass.rms) + '/' + f2(g(s, 10, 'all').TE.rms), f2(g(s, 10, 'straight').AirMass.rms) + '/' + f2(g(s, 10, 'straight').TE.rms),
    f2(g(s, 10, 'circling').AirMass.rms) + '/' + f2(g(s, 10, 'circling').TE.rms), f2(g(s, 30, 'all').AirMass.rms) + '/' + f2(g(s, 30, 'all').TE.rms),
    f2(g(c, 10, 'all').AirMass.rms) + '/' + f2(g(c, 10, 'all').TE.rms),
    f2(g(s, 10, 'straight').AirMass.bias) + '/' + f2(g(s, 10, 'circling').AirMass.bias),
    pct(r.status, 'vario'), pct(r.status, 'wind'), String(r.sensorGapsOver300ms).padStart(5), f2(r.wind.vectorErrorMedian)].join(' ');
  console.log(line);
  if (exclude.has(r.file)) continue;
  for (const p of ['all', 'straight', 'circling']) {
    for (const [P, sc, w] of [[pool, s, 10], [pool30, s, 30], [poolC, c, 10]]) {
      const b = g(sc, w, p); P[p][0] += b.windows; P[p][1] += b.windows * b.AirMass.rms ** 2; P[p][2] += b.windows * b.TE.rms ** 2;
    }
  }
}
for (const [name, P] of [['equal damping 10 s', pool], ['equal damping 30 s', pool30], ['cockpit 10 s', poolC]])
  console.log(name.padEnd(20), Object.entries(P).map(([p, [n, a, t]]) => `${p} n=${n} AM ${Math.sqrt(a / n).toFixed(3)} TE ${Math.sqrt(t / n).toFixed(3)}`).join(' | '));
