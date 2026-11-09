// Pool per-flight batch metrics by complete-window count, retaining matched-cohort and outlier sensitivity.
// node aggregate.js BATCH.json OUTPUT.json PREVIOUS-AGGREGATE.json
const fs = require('fs');
const [input, output, previous] = process.argv.slice(2);
if (!input || !output || !previous) throw new Error('usage: aggregate.js BATCH.json OUTPUT.json PREVIOUS-AGGREGATE.json');
const batch = JSON.parse(fs.readFileSync(input)), baseline = JSON.parse(fs.readFileSync(previous));
const flights = batch.flights;
const median = values => {
  const a = values.filter(Number.isFinite).sort((x, y) => x - y), n = a.length;
  return n ? (a[Math.floor((n - 1) / 2)] + a[Math.floor(n / 2)]) / 2 : null;
};
function pool(rows, ref, mode, window, phase) {
  const bins = rows.map(f => f.vario[ref][mode][window][phase]).filter(b => b.windows);
  const n = bins.reduce((sum, b) => sum + b.windows, 0);
  const result = { windows: n };
  for (const series of ['AirMass', 'TE']) result[series] = {
    rms: n ? Math.sqrt(bins.reduce((sum, b) => sum + b.windows * b[series].rms ** 2, 0) / n) : null,
    bias: n ? bins.reduce((sum, b) => sum + b.windows * b[series].bias, 0) / n : null,
  };
  result.rmsChangePercent = n && result.TE.rms > 0 ? (result.AirMass.rms / result.TE.rms - 1) * 100 : null;
  return result;
}
function group(rows) {
  const aggregate = {};
  for (const ref of ['inertial', 'pitot']) {
    aggregate[ref] = {};
    for (const mode of ['scores', 'cockpitUnshifted']) aggregate[ref][mode] = Object.fromEntries(
      [10, 30, 300].map(w => [w, Object.fromEntries(['all', 'straight', 'circling'].map(p => [p, pool(rows, ref, mode, w, p)]))]));
  }
  const responseDelay = {};
  for (const phase of ['all', 'straight', 'circling']) {
    const a = rows.map(f => f.vario.inertial.cockpitDelay[phase]).filter(x => x.samples >= 100);
    const leads = a.map(x => x.TE - x.AirMass);
    responseDelay[phase] = { flights: a.length, AirMassMedianSeconds: median(a.map(x => x.AirMass)),
      TEMedianSeconds: median(a.map(x => x.TE)), pairedLeadMedianSeconds: median(leads),
      pairedLeadMinSeconds: a.length ? Math.min(...leads) : null, pairedLeadMaxSeconds: a.length ? Math.max(...leads) : null };
  }
  const medians = rows.map(f => f.wind.vectorErrorMedian).filter(Number.isFinite);
  return { included: rows.map(f => f.file), referenceOverlapHours: rows.reduce((s, f) => s + f.scoredHours, 0), aggregate, responseDelay,
    flightsWithLowerRms10: rows.filter(f => {
      const b = f.vario.inertial.scores[10].all;
      return b.windows && b.AirMass.rms < b.TE.rms;
    }).length,
    flightsWithLowerCockpitRms10: rows.filter(f => {
      const b = f.vario.inertial.cockpitUnshifted[10].all;
      return b.windows && b.AirMass.rms < b.TE.rms;
    }).length,
    wind: { independentTurns: rows.reduce((s, f) => s + f.wind.independentTurns, 0),
      minPerFlightMedian: medians.length ? Math.min(...medians) : null,
      maxPerFlightMedian: medians.length ? Math.max(...medians) : null } };
}
const comparable = flights.filter(f => f.replayable && f.file !== '2025-05-13_08-19.nmea' && f.vario.inertial.scores[10].all.windows >= 30);
const may2026 = f => /^2026-05-(24|25)_/.test(f.file);
const result = { created: new Date().toISOString(), implementation: batch.implementation,
  selection: 'At least 30 complete 10-second main-reference windows, excluding known pressure-corrupted May 13. Every excluded log remains in availability and failure reporting. May 24/25 remain in the expanded cohort; show a sensitivity cohort without them because of unusually large TE disagreement.',
  recordedHours: flights.reduce((s, f) => s + (f.hours || 0), 0), logs: flights.length,
  excluded: flights.filter(f => !comparable.includes(f)).map(f => ({ file: f.file,
    reason: !f.replayable ? f.reason : f.file === '2025-05-13_08-19.nmea' ? 'Known gross pressure corruption' :
      !f.summary.gnss ? 'No recorded GNSS velocity/reference' : `Only ${f.vario.inertial.scores[10].all.windows} complete 10-second windows` })),
  groups: {
    expanded: group(comparable),
    previousMatched: group(comparable.filter(f => baseline.included.includes(f.file))),
    withoutMay2026: group(comparable.filter(f => !may2026(f))),
    newMay2026: group(comparable.filter(may2026)),
    newCompressed: group(comparable.filter(f => f.file.endsWith('.gz'))),
  } };
fs.writeFileSync(output, JSON.stringify(result, null, 2) + '\n');
for (const [name, g] of Object.entries(result.groups)) console.log(name, JSON.stringify({ flights: g.included.length,
  overlapHours: g.referenceOverlapHours, rms10: g.aggregate.inertial.scores[10].all,
  rms30: g.aggregate.inertial.scores[30].all, circling30: g.aggregate.inertial.scores[30].circling,
  responseDelay: g.responseDelay, wind: g.wind }));
