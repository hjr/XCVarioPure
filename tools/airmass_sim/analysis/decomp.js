// Bias decomposition: per-sample means of the vario series and the estimator's internal paths against the references,
// binned by bank and by time since circling. node decomp.js OUT.json LOG...   (then node decomp.js --print OUT*.json)
const fs = require('fs'), path = require('path');
const BANK = [[0, 10], [10, 25], [25, 35], [35, 45], [45, 90]];
const SINCE = [[0, 10], [10, 30], [30, 60], [60, 120], [120, 300], [300, 1e9]];
const Q = ['am', 'amCk', 'te', 'refI', 'refP', 'eWd', 'ePlain', 'kin', 'sinkAM', 'sinkDev', 'vd', 'gnssClimb', 'baroClimb'];
function newBins() { const b = {}; for (const [a] of BANK) { b['bank' + a] = {}; b['left' + a] = {}; b['right' + a] = {}; } for (const [a] of SINCE) b['since' + a] = {}; return b; }
function add(bin, vals, keys) { // sums over samples where all keys finite
  for (const k of keys) if (!Number.isFinite(vals[k])) return;
  const tag = keys.join('+');
  const o = bin[tag] ||= { n: 0, s: Object.fromEntries(Q.map(q => [q, 0])) };
  o.n++; for (const q of Q) if (Number.isFinite(vals[q])) o.s[q] += vals[q];
}
if (process.argv[2] === '--print') {
  const merged = newBins();
  for (const f of process.argv.slice(3)) for (const fl of JSON.parse(fs.readFileSync(f)).flights) for (const [b, tags] of Object.entries(fl.bins)) for (const [tag, o] of Object.entries(tags)) {
    const m = merged[b][tag] ||= { n: 0, s: Object.fromEntries(Q.map(q => [q, 0])) }; m.n += o.n; for (const q of Q) m.s[q] += o.s[q];
  }
  const show = (tag, cols) => {
    console.log('\n== samples with ' + tag + ' ; means of differences, m/s');
    console.log('bin'.padEnd(10) + 'n'.padStart(8) + cols.map(c => c[0].padStart(13)).join(''));
    for (const [b, tags] of Object.entries(merged)) { const o = tags[tag]; if (!o || o.n < 100) continue; const m = Object.fromEntries(Q.map(q => [q, o.s[q] / o.n]));
      console.log(b.padEnd(10) + String(o.n).padStart(8) + cols.map(c => c[1](m).toFixed(3).padStart(13)).join('')); }
  };
  show('am+te+refI', [['am-ref', m => m.am - m.refI], ['te-ref', m => m.te - m.refI], ['am-te', m => m.am - m.te], ['eWd-ref', m => m.eWd - m.refI], ['ePlain-ref', m => m.ePlain - m.refI], ['kin-ref', m => m.kin - m.refI], ['sinkAM-Dev', m => m.sinkAM - m.sinkDev], ['-vd-gnssCl', m => -m.vd - m.gnssClimb], ['baro-gnssCl', m => m.baroClimb - m.gnssClimb], ['amCk-te', m => m.amCk - m.te]]);
  show('am+te+refP', [['am-refP', m => m.am - m.refP], ['te-refP', m => m.te - m.refP], ['am-te', m => m.am - m.te], ['eWd-refP', m => m.eWd - m.refP], ['ePlain-refP', m => m.ePlain - m.refP], ['kin-refP', m => m.kin - m.refP]]);
  show('am+te', [['am-te', m => m.am - m.te], ['eWd-te', m => m.eWd - m.te], ['ePlain-te', m => m.ePlain - m.te], ['kin-te', m => m.kin - m.te], ['sinkAM-Dev', m => m.sinkAM - m.sinkDev], ['amCk-te', m => m.amCk - m.te]]);
  process.exit(0);
}
const { loadPage, freeMask } = require('./page.js');
const web = path.resolve(__dirname, '../web');
const { runReplay, loadFlight } = loadPage(web);
(async () => {
  const M = await require(path.join(web, 'airmass.js'))();
  const out = { flights: [] };
  for (const file of process.argv.slice(3)) {
    const conv = loadFlight(file, { glider: 'log' });
    const rep = await runReplay(M, conv.rec, { settings: process.env.SETTINGS || '' });
    globalThis.fileName = path.basename(file);
    globalThis.D = build(conv, rep); derive(D);
    const r = D.rep.cols, c = D.tr, n = D.n;
    const { free } = freeMask(D, k => c.ias[k] > 60 / 3.6 && c.gnss_speed[k] > 10);
    // GNSS and baro climb over +-5 s centred (crude), for the vertical-speed path
    const gnssClimb = new Float32Array(n).fill(NaN), baroClimb = new Float32Array(n).fill(NaN);
    for (let j = 1; j + 1 < D.fixes.length; j++) { const a = D.fixes[j - 1], b = D.fixes[j + 1]; if (b.u - a.u > 0 && b.u - a.u < 5) { const k = D.fixes[j].k; gnssClimb[k] = (b.alt - a.alt) / (b.u - a.u); } }
    for (let k = 0; k < n; k++) { let a = k, b = k; while (a > 0 && D.t[k] - D.t[a] < 1) a--; while (b + 1 < n && D.t[b] - D.t[k] < 1) b++; if (D.t[b] - D.t[a] > 1.5 && D.t[b] - D.t[a] < 2.5) baroClimb[k] = (c.baro_alt[b] - c.baro_alt[a]) / (D.t[b] - D.t[a]); }
    const bins = newBins();
    let lastCircling = -1e9;
    for (let k = 0; k < n; k++) {
      const i = D.ai[k]; if (i < 0) continue;
      const roll = r.roll_deg[i], ab = Math.abs(roll);
      if (ab > 25) lastCircling = D.t[k];
      if (!free[k] || !D.amOk[k]) continue;
      const vals = { am: D.amRaw[k], amCk: D.am[k], te: D.netto[k], refI: D.gnssNetInert[k], refP: D.gnssNetPitot[k], eWd: -r.energy_wd[i], ePlain: -r.energy_wd_plain[i], kin: -r.kinematic_wd[i],
        sinkAM: -r.polar_sink[i], sinkDev: D.sinkUp[k], vd: r.vd[i], gnssClimb: gnssClimb[k], baroClimb: baroClimb[k] };
      const targets = [];
      for (const [a, b] of BANK) if (ab >= a && ab < b) { targets.push('bank' + a); targets.push((roll < 0 ? 'left' : 'right') + a); }
      if (ab < 10) for (const [a, b] of SINCE) { const s = D.t[k] - lastCircling; if (s >= a && s < b) targets.push('since' + a); }
      for (const t of targets) for (const keys of [['am', 'te', 'refI'], ['am', 'te', 'refP'], ['am', 'te']]) add(bins[t], vals, keys);
    }
    out.flights.push({ file: path.basename(file), bins });
    fs.writeFileSync(process.argv[2], JSON.stringify(out));
    console.error('done', path.basename(file));
  }
})().catch(e => { console.error(e); process.exit(1); });
