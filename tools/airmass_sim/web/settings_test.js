// node --test tools/airmass_sim/web/settings_test.js
const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const script = fs.readFileSync(__dirname + "/index.html", "utf8").split("<script>")[1].split("</script>")[0];
function grab(name) {
  const start = script.indexOf(name);
  assert(start >= 0, name);
  let i = script.indexOf("{", start), depth = 0;
  for (; i < script.length; i++) {
    if (script[i] === "{") depth++;
    else if (script[i] === "}" && --depth === 0) break;
  }
  return script.slice(start, i + 1);
}
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
    fileText: "flight source", fileName: "flight", loadGeneration: 3, loading: false,
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
    c => { c.loadGeneration++; c.fileText = "another flight"; },
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
  const data = { n, ai: Int32Array.from({ length: n }, (_, i) => i), amOk: new Uint8Array(n).fill(1),
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
