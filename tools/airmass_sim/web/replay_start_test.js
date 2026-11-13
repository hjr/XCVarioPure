// End-to-end converter -> WebAssembly -> page data for logs opened after takeoff.
// Run after build.sh: node tools/airmass_sim/web/replay_start_test.js
const assert = require("node:assert/strict");
const { loadPage } = require("../analysis/page.js");
const { STATUS } = require("./replay.js");
const { checksum: sentence } = require("./convert.js");
const { convert, runReplay } = loadPage(__dirname);

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
  assert.equal(full.summary.prediction_ms, 100);
  assert(full.t_ms.every((t, i) => !i || t - full.t_ms[i - 1] === 100), "10Hz sensor logs replay at their own rate");
  assert.equal(full.summary.recovered_starts, 0);

  // A confirmed sensor outage must reach the estimator as a gap, without held-input predictions filling it.
  const gapped = [];
  for (let i = 0; i < 240; i++) {
    const elapsed = i * .1 + (i >= 180 ? 113.573 : 0), utc = 43200 + elapsed;
    const time = [Math.floor(utc / 3600), Math.floor(utc / 60) % 60].map(v => String(v).padStart(2, "0")).join("") +
      (utc % 60).toFixed(3).padStart(6, "0");
    gapped.push(`$GPRMC,${time},A,4800.000,N,01100.000,E,60.0,90.0,091126,,,A`);
    gapped.push(`$SENS,${(300 + elapsed).toFixed(3)},0,900,900,500,15,0,0,-1,0,0,0`);
  }
  const gapConv = convert(gapped.join("\n"), { glider: "log" });
  assert.equal(gapConv.track.cols.t_ms[180] - gapConv.track.cols.t_ms[179], 113673);
  for (const mahony of [false, true]) {
    const gapReplay = await runReplay(M, gapConv.rec, { mahony });
    const after = gapReplay.t_ms.findIndex((t, i, times) => i && t - times[i - 1] > 10000);
    assert(after > 0, "the recorded gap reaches WASM");
    assert.equal(gapReplay.t_ms[after] - gapReplay.t_ms[after - 1], 113673);
    assert(gapReplay.n <= full.n, "no prediction ticks are fabricated through the outage");
    assert(gapReplay.cols.status[after - 1] & STATUS.VARIO_VALID, "the vario settled before the outage");
    assert(!(gapReplay.cols.status[after] & STATUS.VARIO_VALID), "the estimator must settle again after the gap");
  }

  // RR-style logs carry sensors/attitude but no navigation sentences at all.
  const noGnss = convert(sensor.filter(line => !line.startsWith("$GPRMC,")).join("\n"), { glider: "log" });
  assert.equal(noGnss.frames, 141);
  assert.equal(noGnss.fixes, 0);
  let frozenFrame = 0;
  const frozen = sensor.filter(line => !line.startsWith("$GPRMC,")).map(line => {
    if (!line.startsWith("$SENS,")) return line;
    const fields = line.split(",");
    if (frozenFrame > 180 && frozenFrame <= 190) fields[1] = "318.000";
    fields[2] = String(1700000 + frozenFrame++ * 100);
    return fields.join(",");
  });
  const recoveredClock = convert(frozen.join("\n"), { glider: "log" });
  assert.equal(recoveredClock.timestampRecoveredSamples, 10);
  assert.deepEqual(recoveredClock.track.cols.t_ms, noGnss.track.cols.t_ms);
  for (const mahony of [false, true]) {
    const rep = await runReplay(M, noGnss.rec, { mahony });
    assert.equal(rep.n, 141);
    assert(rep.cols.vario.every(Number.isFinite));
    assert(rep.cols.status.every(s => !(s & (STATUS.WIND_VALID | STATUS.GNSS_FRESH | STATUS.ALIGNED))), "missing GNSS cannot invent wind, fixes or heading alignment");
    assert(rep.cols.status.some(s => s & STATUS.VARIO_VALID), "vario can settle without GNSS");
    const recovered = await runReplay(M, recoveredClock.rec, { mahony });
    assert.deepEqual(recovered.t_ms, rep.t_ms, "recovered clock preserves measured/predicted replay cadence");
    assert.deepEqual(recovered.cols, rep.cols, "recovered clock yields the same estimator output as correctly timestamped input");
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
    assert(rep.cols.status.slice(0, 99).every(s => !(s & STATUS.VARIO_VALID)), "20Hz still needs five seconds of settling");
    assert(rep.cols.status[99] & STATUS.VARIO_VALID);
    // One invalid frame is bridged by the held readings; validity ends when they are older than 300 ms.
    const invalid = [...timed];
    for (let t = 12050; t <= 12350; t += 50) invalid.push(sentence(`$PXCVR,${t},,,,,,,,,,,${t},${t},${t},${t},0`));
    const bad = await runReplay(M, convert(invalid.join("\n")).rec, { mahony });
    assert.equal(bad.n, 247);
    assert(bad.cols.status[240] & STATUS.VARIO_VALID, "one invalid frame is bridged");
    assert(!(bad.cols.status[bad.n - 1] & STATUS.VARIO_VALID), "explicit invalid-source metadata forces fallback");
    assert(bad.cols.vario.every(Number.isFinite));

    assert(rep.t_ms.every((t, i) => t === (i + 1) * 50), "recorded 20Hz polls are kept");

    const slow = timed.slice(0, 2);
    for (let t = 0; t <= 12000; t += 200) slow.push(sentence(`$PXCVR,${t},0,0,-1,0,0,0,30,30,1000,`));
    const legacy = await runReplay(M, convert(slow.join("\n")).rec, { mahony });
    assert.equal(legacy.n, 121);
    assert(legacy.t_ms.every((t, i) => t === i * 100), "5Hz legacy recordings predict every 100ms");
    const data = globalThis.build(convert(slow.join("\n")), legacy);
    assert(data.ai.every(i => i >= 0), "page still maps every measured frame to its replay output");
  }
  console.log("PASS: airborne/no-GNSS recovery, confirmed sensor gaps, frozen-clock recovery, both attitude filters, 10Hz predictions from 5Hz logs, source freshness and page integration");
})().catch(error => { console.error(error); process.exitCode = 1; });
