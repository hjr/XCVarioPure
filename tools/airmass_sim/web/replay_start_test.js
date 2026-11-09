// End-to-end converter -> WebAssembly -> page data for logs opened after takeoff.
// Run after build.sh: node tools/airmass_sim/web/replay_start_test.js
const assert = require("node:assert/strict");
const { loadPage } = require("../analysis/page.js");
const { convert, runReplay } = loadPage(__dirname);
function sentence(body) {
  let sum = 0;
  for (const c of body.slice(1)) sum ^= c.charCodeAt(0);
  return `${body}*${sum.toString(16).padStart(2, "0")}`;
}

(async () => {
  const M = await require("./airmass.js")();
  const sensor = [];
  for (let i = 0; i < 240; ++i) {
    sensor.push("$GPRMC,120000.00,A,4800.000,N,01100.000,E,60.0,90.0,091126,,,A");
    sensor.push(sentence("$PXCV,0,1,0,1,1,15,1013,1000,0,25,3"));
    sensor.push(`$SENS,${(300 + i * 0.1).toFixed(3)},0,900,900,500,15,0,0,-1,0,0,0`);
  }
  const sens = convert(sensor.join("\n"), { glider: "log" });
  assert.equal(sens.starts, 1);
  assert.equal(sens.frames, 141);
  assert.equal(sens.track.cols.running.findIndex(v => v), 99);
  const full = await runReplay(M, sens.rec);
  assert.equal(full.n, 141);
  assert.equal(full.summary.recovered_starts, 0);

  // RR-style logs carry sensors/attitude but no navigation sentences at all.
  const noGnss = convert(sensor.filter(line => !line.startsWith("$GPRMC,")).join("\n"), { glider: "log" });
  assert.equal(noGnss.frames, 141);
  assert.equal(noGnss.fixes, 0);
  for (const mahony of [false, true]) {
    const rep = await runReplay(M, noGnss.rec, { mahony });
    assert.equal(rep.n, 141);
    assert(rep.cols.vario.every(Number.isFinite));
    assert(rep.cols.status.every(s => !(s & (2 | 4 | 8))), "missing GNSS cannot invent wind, fixes or heading alignment");
    assert(rep.cols.status.some(s => s & 1), "vario can settle without GNSS");
    const data = globalThis.build(noGnss, rep);
    globalThis.derive(data);
    assert(data.lat.every(v => !Number.isFinite(v)), "no GNSS positions means no invented track");
  }
  const lowGroundSpeed = convert(sensor.map(line => line.startsWith("$GPRMC,") ? line.replace(",60.0,", ",3.0,") : line).join("\n"), { glider: "log" });
  assert.equal(lowGroundSpeed.frames, 141, "a headwind must not block replay startup");
  const groundOnly = convert(sensor.map(line => line.startsWith("$SENS,") ? line.replace(",500,", ",0,") : line).join("\n"), { glider: "log" });
  assert.equal(groundOnly.frames, 0, "low IAS still prevents flight startup");

  const recording = sens.rec.flatMap(line => line.startsWith("$PXCVR,") ? [
    sentence("$PXCV,0,1,0,1,1,15,1013,1000,0,25,3"), line,
  ] : [line]);
  const missingStart = recording.filter(line => !line.startsWith("$PXCVS,"));
  for (const mahony of [false, true]) {
    const conv = convert(missingStart.join("\n"));
    assert.equal(conv.kind, "AirMass recording");
    assert(conv.rec.some(line => line.startsWith("$PXCV,")), "preserve device attitude for shared replay");
    const rep = await runReplay(M, conv.rec, { mahony });
    assert.equal(rep.n, 141);
    assert.equal(rep.summary.recovered_starts, 1);
    assert.equal(rep.summary.estimated_attitude_starts, 0);
    assert(Math.abs(rep.cols.roll_deg[0]) > 10, "initial roll comes from the logged attitude");
    const data = globalThis.build(conv, rep);
    assert(data.tr.running.every(v => v === 1), "page uses recovered frames for its running mask");
    assert(data.ai.every(v => v >= 0), "recovered output is available to the instruments and chart");
    assert.equal(data.tStart, rep.t_ms[0] / 1000);
  }
  const withoutAttitude = missingStart.filter(line => !line.startsWith("$PXCV,"));
  const estimated = await runReplay(M, convert(withoutAttitude.join("\n")).rec);
  assert.equal(estimated.n, 141);
  assert.equal(estimated.summary.estimated_attitude_starts, 1);
  const noSetup = await runReplay(M, missingStart.filter(line => !line.startsWith("$PXCVC,")));
  assert.equal(noSetup.n, 0, "missing setup is not silently replaced by an invented polar");
  assert.equal(noSetup.summary.skipped, 141);

  // New device recordings include source timestamps: 20Hz polls with physical readings at 10Hz.
  const timed = [sens.rec.find(line => line.startsWith("$PXCVC,")),
    sentence("$PXCVS,0,1000,0,0,1,0,0,0,1,0,0.75,1,1,1,1,1,1,1,1,2")];
  for (let t = 50; t <= 12000; t += 50) {
    const stamp = Math.floor(t / 100) * 100;
    const held = t > 50 && t % 100 !== 0;
    // Different values with unchanged source timestamps must be ignored, including by Mahony.
    timed.push(sentence(`$PXCVR,${t},0,0,-1,${held ? 2 : 0},0,0,${held ? 90 : 30},${held ? 90 : 30},${held ? 2000 : 1000},,${stamp},${stamp},${stamp},${stamp},15`));
  }
  for (const mahony of [false, true]) {
    const rep = await runReplay(M, convert(timed.join("\n")).rec, { mahony });
    assert.equal(rep.n, 240);
    assert.equal(rep.summary.seconds, 12, "reported duration is elapsed time, independent of frame rate");
    assert(rep.cols.roll_deg.every(v => Math.abs(v) < 0.001), "held-stamp gyro changes cannot tilt either attitude filter");
    assert(rep.cols.status.slice(0, 99).every(s => !(s & 1)), "20Hz still needs five seconds of settling");
    assert(rep.cols.status[99] & 1);
    const invalid = [...timed, sentence("$PXCVR,12050,,,,,,,,,,,12050,12050,12050,12050,0")];
    const bad = await runReplay(M, convert(invalid.join("\n")).rec, { mahony });
    assert(!(bad.cols.status[bad.n - 1] & 1), "explicit invalid-source metadata forces fallback");
    assert(bad.cols.vario.every(Number.isFinite));
  }
  console.log("PASS: no-GNSS/low-ground-speed startup, missing starts with both filters, attitude estimation, page integration, missing setup");
})().catch(error => { console.error(error); process.exitCode = 1; });
