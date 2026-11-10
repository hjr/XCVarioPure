// node --test tools/airmass_sim/web/settings_test.js
const { test } = require("node:test");
const assert = require("node:assert/strict");
const vm = require("node:vm");
const page = require("../analysis/page.js");
const script = page.pageScript(__dirname);
const grab = name => page.grab(script, name);
function elements() {
  const values = { averager: "30", damping: "4", speed: "5", glider: "log", mass: "-1", bugs: "-1", w0: "1", w1: "0", w2: "1", wlat: ".75" };
  const all = {};
  return id => all[id] ||= { value: values[id] || "", checked: false, options: [{}, {}], listeners: {}, style: {},
    addEventListener(name, fn) { (this.listeners[name] ||= []).push(fn); },
    checkValidity() { return this.value !== "" && Number.isFinite(+this.value); },
    fire(name) { for (const fn of this.listeners[name] || []) fn(); } };
}
function harness({ connected = false, playing = true } = {}) {
  const $ = elements(), timeline = { events: [{ t: 300 }, { t: 400 }], start: 300, end: 400 };
  const calls = { pause: 0, play: 0, draw: 0, derive: 0, damping: 0, labels: 0, timeline: 0, trackFit: [] };
  const context = vm.createContext({ $, D: { sourceGeneration: 3 }, T: 320, view: [310, 330], playing,
    fileLog: "flight source", fileName: "flight", loadGeneration: 3, loading: false,
    serial: { connected, port: connected ? {} : null, ready: true, speed: 5, timeline },
    localStorage: { getItem: () => "{}", setItem() {} }, performance: { now: () => 100 },
    modulePromise: Promise.resolve({}), tick: async () => {},
    setStatus() {}, serialControls() {}, drawTrack(fit) { calls.trackFit.push(fit); }, invalidateCharts() {},
    drawAll() { calls.draw++; }, deriveDamping() { calls.damping++; }, updateVarioLabels() { calls.labels++; },
    logDate: () => ({ day: NaN }), console: { error() {} },
    AirMassConvert: { convert: () => ({ frames: 2, kind: "XCVario sensor log", track: { n: 2 }, fixes: 0 }),
      serialTimeline: () => { throw new Error("Same-file settings must retain the serial timeline"); } },
    runReplay: async () => ({ n: 2, t_ms: [300000, 400000], summary: { seconds: 100, starts: 1, cycles: 2,
      gnss: 0, compass: 0, k_tas: 1, pitch_ofs_deg: 0, aoa_scale: 1, latency_ms: 750, gyro_z_bias_dps: 0 } }),
  });
  context.pause = () => { calls.pause++; context.playing = false; };
  context.play = () => { calls.play++; context.playing = true; };
  context.derive = data => { calls.derive++; data.altDelay = 1.6; };
  context.build = (_, rep) => ({ t: [300, 400], tStart: 309.9, rep });
  context.displayTime = t => { context.T = t; };
  context.serial.setTimeline = async () => { calls.timeline++; };
  vm.runInContext("let replaying = false, replayAgain = false;\n" + grab("async function run()") + "\n" + grab("async function replayOnce()"), context);
  return { context, $, calls, timeline };
}

test("display settings apply without pausing/replaying; model settings apply automatically", () => {
  const h = harness({ connected: true });
  let runs = 0;
  h.context.run = () => { runs++; };
  vm.runInContext(grab("function averagingSeconds()") + "\n" + script.slice(script.indexOf("const SETTINGS_KEY ="), script.indexOf("// progress and errors")), h.context);
  h.$("averager").value = "20"; h.$("averager").fire("input"); h.$("averager").fire("change");
  h.$("damping").value = "6"; h.$("damping").fire("input"); h.$("damping").fire("change");
  h.$("samesmooth").fire("change");
  assert.equal(h.calls.labels, 2); assert.equal(h.calls.damping, 3);
  assert.equal(h.calls.derive, 0); assert.equal(h.calls.pause, 0); assert.equal(runs, 0);
  assert.equal(h.context.T, 320); assert.equal(h.context.playing, true);
  assert.equal(h.context.serial.timeline, h.timeline); assert.equal(h.context.serial.speed, 5);
  for (const id of ["glider", "mass", "bugs", "sets", "noflaps", "mahony", "learn", "warm"]) h.$(id).fire("change");
  assert.equal(runs, 8);
  h.$("w0").fire("change"); assert.equal(runs, 8, "inactive warm-start values do not recalculate");
  h.$("warm").checked = true; h.$("w0").fire("change"); assert.equal(runs, 9);
});

test("successful recalculation preserves position, chart view, speed and running/paused state", async () => {
  for (const connected of [false, true]) for (const playing of [false, true]) {
    const h = harness({ connected, playing });
    await h.context.run();
    assert.equal(h.calls.pause, 1);
    assert.equal(h.context.T, 320);
    assert.deepEqual([...h.context.view], [310, 330]);
    assert.equal(h.context.playing, playing);
    assert.equal(h.calls.play, Number(playing));
    assert.equal(h.$("speed").value, "5"); assert.equal(h.context.serial.speed, 5);
    assert.equal(h.context.serial.timeline, h.timeline); assert.equal(h.calls.timeline, 0);
    assert.deepEqual(h.calls.trackFit, [false], "the map must not zoom back to the whole flight");
  }
});

test("failed estimator updates retain prior data and remain paused", async () => {
  const h = harness(), previous = h.context.D;
  h.context.runReplay = async () => { throw new Error("Invalid estimator setting"); };
  await h.context.run();
  assert.equal(h.context.D, previous); assert.equal(h.context.playing, false);
  assert.equal(h.calls.play, 0); assert.equal(h.context.T, 320);
});

test("new file or a lost/replaced serial port prevents automatic resume", async () => {
  for (const mutate of [
    c => { c.loadGeneration++; c.fileLog = "another flight"; },
    c => { c.serial.connected = false; c.serial.ready = false; c.serial.port = null; },
    c => { c.serial.port = {}; },
  ]) {
    const h = harness({ connected: true });
    let release;
    h.context.tick = () => new Promise(resolve => { release = resolve; });
    const pending = h.context.run();
    mutate(h.context); release(); await pending;
    assert.equal(h.calls.play, 0); assert.equal(h.context.playing, false);
  }
});

test("rapid estimator edits serialize and resume only after the latest recalculation", async () => {
  const h = harness();
  let release, passes = 0;
  h.context.replayOnce = async () => {
    h.context.pause();
    if (++passes === 1) await new Promise(resolve => { release = resolve; });
    return true;
  };
  const pending = h.context.run();
  await h.context.run(); await h.context.run();
  assert.equal(passes, 1); release(); await pending;
  assert.equal(passes, 2); assert.equal(h.calls.play, 1); assert.equal(h.context.playing, true);
});

test("live damping updates cached display series while retaining estimator/reference/average data", () => {
  const $ = elements(), n = 200;
  const data = { n, t: Float64Array.from({ length: n }, (_, i) => i / 10), ai: Int32Array.from({ length: n }, (_, i) => i), amOk: new Uint8Array(n).fill(1),
    amRaw: Float32Array.from({ length: n }, (_, i) => i < 100 ? 0 : 4), netto: new Float32Array(n).fill(2),
    sinkUp: new Float32Array(n).fill(-1), gnssNet: Float32Array.from({ length: n }, (_, i) => i < 100 ? 0 : 3),
    gnssGrossRaw: Float32Array.from({ length: n }, (_, i) => i < 100 ? 0 : 5),
    tr: { te_vario: new Float32Array(n).fill(2) }, rep: {}, cum: {} };
  const retained = [data.rep, data.tr, data.cum, data.amRaw, data.gnssNet, data.gnssGrossRaw];
  data.amOk[90] = 0;
  const context = vm.createContext({ $ });
  vm.runInContext(["class Damp", "function cockpitSeries(D, dampAt)", "function deriveDamping(D)"].map(grab).join("\n"), context);
  $("samesmooth").checked = true; $("damping").value = "2";
  context.deriveDamping(data);
  const fast = data.am[120], fastGnss = data.gnss[120];
  $("damping").value = "8"; context.deriveDamping(data);
  assert(data.am[120] < fast); assert(data.gnss[120] < fastGnss);
  assert.equal(data.am[90], 2); assert.equal(data.amGross[90], 2, "gross fallback remains logged TE");
  $("samesmooth").checked = false; context.deriveDamping(data);
  assert.equal(data.gnss[120], 3); assert.equal(data.gnssGross[120], 5);
  assert.deepEqual([data.rep, data.tr, data.cum, data.amRaw, data.gnssNet, data.gnssGrossRaw], retained);
});

test("the needle damping runs the device's 10 Hz cycles whatever the frame rate", () => {
  const context = vm.createContext({});
  // the damping before, one device cycle per frame
  const perFrame = `function perFrame(D, dampAt) {
    const out = new Float32Array(D.n).fill(NaN), f = new Damp(1);
    for (let k = 0; k < D.n; k++) {
      if (D.ai[k] < 0) { f.reset(NaN); continue; }
      f.d = dampAt(k);
      if (D.amOk[k]) {
        if (f.y === null) f.reset(Number.isFinite(D.netto[k]) ? D.netto[k] : D.amRaw[k]);
        out[k] = f.f(D.amRaw[k]);
      } else { f.reset(D.netto[k]); out[k] = D.netto[k]; }
    }
    return out;
  }`;
  vm.runInContext(["class Damp", "function cockpitSeries(D, dampAt)"].map(grab).join("\n") + "\n" + perFrame, context);
  // netto until 1 s, then a step of the air mass vario from 0 to 4 m/s at 3 s; 10 Hz frames 10 ms early and late in
  // turn, as in the logs
  const flight = hz => {
    const n = 20 * hz, nominal = i => i / hz;
    return { n, nominal: Float64Array.from({ length: n }, (_, i) => nominal(i)),
      t: Float64Array.from({ length: n }, (_, i) => nominal(i) + (hz === 10 ? (i % 2 ? 0.01 : -0.01) : 0)),
      ai: Int32Array.from({ length: n }, (_, i) => i), amOk: Uint8Array.from({ length: n }, (_, i) => nominal(i) >= 1 ? 1 : 0),
      amRaw: Float32Array.from({ length: n }, (_, i) => nominal(i) >= 3 ? 4 : 0), netto: new Float32Array(n).fill(1) };
  };
  const damp = () => 2, at = (D, s, time) => s[D.nominal.findIndex(x => Math.abs(x - time) < 1e-9)];
  const runs = [5, 10, 20].map(hz => { const D = flight(hz); return { D, s: context.cockpitSeries(D, damp), before: context.perFrame(D, damp) }; });
  const [r5, r10, r20] = runs;
  assert.deepEqual(r10.s, r10.before, "10 Hz frames are damped as before");
  let worst5 = 0, worst20 = 0;
  for (let i = 0; i < 100; i++) {
    const time = i / 5, ref = at(r10.D, r10.s, time);
    assert(Math.abs(at(r5.D, r5.s, time) - ref) < 1e-6, `5 Hz at ${time} s`);
    assert(Math.abs(at(r20.D, r20.s, time) - ref) < 1e-6, `20 Hz at ${time} s`);
    worst5 = Math.max(worst5, Math.abs(at(r5.D, r5.before, time) - ref));
    worst20 = Math.max(worst20, Math.abs(at(r20.D, r20.before, time) - ref));
  }
  assert(worst5 > 0.5 && worst20 > 0.5, "a cycle per frame damped 5 and 20 Hz frames differently");
  for (let k = 1; k < r20.D.n; k += 2) assert.equal(r20.s[k], r20.s[k - 1], "the needle holds between cycles");
  assert(r20.s[r20.D.n - 1] > 3.9, "the needle settles on the step");
});
