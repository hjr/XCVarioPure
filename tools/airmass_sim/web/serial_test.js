// Deterministic protocol/transport tests: no serial device or browser permission needed.
// node --test tools/airmass_sim/web/serial_test.js
const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const { serialTimeline, convert, prepare, gliderFromLog } = require("./convert.js");
const { SerialReplay, command, allowedSpeeds, stateAt } = require("./serial.js");
const { pageScript, grab } = require("../analysis/page.js");
const sens = (t, extra = "") => `$SENS,${t.toFixed(3)},0,900,899,500,15,0,0,-1,0,0,0${extra}`;
const rmc = t => {
  const u = ((t % 86400) + 86400) % 86400;
  const time = [Math.floor(u / 3600), Math.floor(u / 60) % 60].map(v => String(v).padStart(2, "0")).join("") +
    (u % 60).toFixed(3).padStart(6, "0");
  return `$GPRMC,${time},A,4800,N,01100,E,60,90,091126,,,A`;
};
const ca1 = "!XCV,CA1,2305,324,100,0,0,102000";
const log = [ca1, sens(300), "$GPRMC,120000,A,4800,N,01100,E,60,90,091126,,,A", sens(300.1),
  "!XCV,FLP,33", "!xcv,bal-water,20", sens(300.2), sens(300.3), sens(300.4)].join("\n");
const deferred = () => { let resolve, reject; const promise = new Promise((a, b) => { resolve = a; reject = b; }); return { promise, resolve, reject }; };
async function flush() { for (let i = 0; i < 80; i++) await Promise.resolve(); }
class Clock {
  constructor() { this.t = 0; this.id = 0; this.timers = new Map(); }
  now = () => this.t;
  setTimer = (fn, ms) => { const id = ++this.id; this.timers.set(id, { fn, t: this.t + ms }); return id; };
  clearTimer = id => this.timers.delete(id);
  async advance(ms) {
    const end = this.t + ms;
    await flush();
    for (;;) {
      const next = [...this.timers].filter(([, v]) => v.t <= end).sort((a, b) => a[1].t - b[1].t || a[0] - b[0])[0];
      if (!next) break;
      this.t = Math.max(this.t, next[1].t); this.timers.delete(next[0]); next[1].fn(); await flush();
    }
    this.t = Math.max(this.t, end); await flush();
  }
}
function harness(text = log) {
  const clock = new Clock(), sent = [], calls = [], listeners = {};
  const writer = { ready: Promise.resolve(), write: async data => { sent.push({ t: clock.t, line: new TextDecoder().decode(data).trim() }); },
    abort: async () => { calls.push("abort"); }, releaseLock: () => calls.push("release") };
  const port = { writable: { getWriter: () => writer }, open: async opts => calls.push(opts), close: async () => calls.push("close") };
  const api = { requestPort: async () => { calls.push("chooser"); return port; }, addEventListener: (event, fn) => { listeners[event] = fn; } };
  const changes = [];
  const replay = new SerialReplay({ serial: api, now: clock.now, setTimer: clock.setTimer, clearTimer: clock.clearTimer,
    onChange: s => changes.push(s.message) });
  const timeline = serialTimeline(text);
  replay.setTimeline(timeline);
  return { clock, sent, calls, writer, port, api, replay, timeline, changes, listeners,
    connect: async () => { const p = replay.connect(); await clock.advance(1000); await p; } };
}

test("timeline preserves original cadence, navigation, TE/magnetometer and repaired time", () => {
  const text = ["!XCV,FMT,SENS,7,1,-1,-1,1,-1,-1,1,-1,-1", "!XCV,CMN,2305,324,100,0,0",
    "$SENS;1000;0;900;899;500;84.9;0;0;1;2;3;4;5;6;7", "$GNGGA,120000,4800,N,01100,E,1,9,1,1000,M,0,M,,",
    sens(1.1), sens(500), sens(1)].join("\n");
  const timeline = serialTimeline(text), offline = convert(text);
  const sensors = timeline.events.filter(e => e.kind === "sensor");
  assert.deepEqual(sensors.map(e => e.t * 1000), [...offline.track.cols.t_ms]);
  assert.deepEqual(sensors.map(e => e.t), [1, 1.1, 1.2, 1.3]);
  assert.match(sensors[0].line, /,900,899,500,[\d.]+,0,0,-1,2,-3,-4,5,-6,-7\*/);
  assert(Math.abs(+sensors[0].line.split(",")[6] - 18.57474971364874) < 1e-10);
  assert(timeline.events.some(e => e.line.startsWith("$GNGGA,")));
  assert(timeline.events.every((e, i, arr) => !i || e.t >= arr[i - 1].t));
  assert.match(timeline.initialConfig, /^!XCV,CA1,2305,324,100,0,0,10/);
  assert.equal(sensors.length, 4, "no prediction samples injected");
  assert.match(serialTimeline("$PXCVR,1000,0,0,-1").reason, /lack the complete/);
  const malformed = serialTimeline(sens(1).replace(",-1,", ",NaN,"));
  assert.equal(malformed.skippedSensors, 1);
  assert.match(malformed.reason, /malformed \$SENS lines/);
});

test("GNSS-confirmed long sensor gaps spread navigation traffic and retain serial deadlines", async () => {
  const start = 42550.388, end = 42663.961;
  const navigation = Array.from({ length: 235 }, (_, i) => rmc(start + (end - start) * (i + 1) / 236));
  const text = [rmc(start), sens(start), ...navigation, sens(end)].join("\n");
  const h = harness(text), offline = convert(text);
  const sensors = h.timeline.events.filter(e => e.kind === "sensor");
  assert.deepEqual(sensors.map(e => e.t * 1000), [...offline.track.cols.t_ms]);
  assert(Math.abs(sensors[1].t - sensors[0].t - 113.573) < 1e-9);
  const during = h.timeline.events.filter(e => e.t > start && e.t < end);
  assert.equal(during.length, 235);
  assert(during.at(-1).t - during[0].t > 112, "navigation is not squeezed into a sensor cycle");
  assert(allowedSpeeds(h.timeline, 115200).includes(1));
  await h.connect(); await h.replay.play(); await h.clock.advance(1000);
  assert.equal(h.sent.filter(e => e.line.startsWith("$SENS,")).length, 1);
  await h.clock.advance(112574);
  const sent = h.sent.filter(e => e.line.startsWith("$SENS,"));
  assert.equal(sent.length, 2);
  assert(Math.abs(sent[1].t - sent[0].t - 113573) < 1e-6);
  assert.equal(h.replay.running, false);
});

test("confirmed gaps survive UTC midnight while clock synchronization and reboot jumps are repaired", () => {
  const cases = [
    { lines: [rmc(86399.9), sens(86399.9), rmc(86513.473), sens(86513.473)], expected: [86399900, 86513473] },
    { lines: [rmc(43200), sens(10), rmc(43200.1), sens(43200.1), sens(1)], expected: [10000, 10100, 10200] },
    { lines: [sens(10), rmc(43200), sens(43200)], expected: [10000, 10100] },
    { lines: [rmc(43200), sens(10), rmc(43200), sens(43200)], expected: [10000, 10100] },
  ];
  for (const { lines, expected } of cases) {
    const text = lines.join("\n"), offline = convert(text);
    assert.deepEqual([...offline.track.cols.t_ms], expected);
    assert.deepEqual(serialTimeline(text).events.filter(e => e.kind === "sensor").map(e => Math.round(e.t * 1000)), expected);
  }
});

test("navigation outside sensor coverage cannot create edge bursts; setup and all sensor samples remain", async () => {
  const prefix = Array.from({ length: 1200 }, (_, i) => rmc(i));
  const tail = Array.from({ length: 2000 }, (_, i) => rmc(1200 + i));
  const text = [ca1, ...prefix, "!XCV,FLP,33", sens(300), "$PFLAA,0,1,2,3,1,ABCDEF,0,,0,0,1", sens(300.1), ...tail].join("\n");
  const h = harness(text), offline = convert(text);
  assert.equal(h.timeline.excludedNavigation, 3200);
  assert.match(h.timeline.warning, /3200 navigation messages outside the sensor recording ignored/);
  assert.equal(h.timeline.start, 300);
  assert.equal(h.timeline.end, 300.1);
  assert.deepEqual(h.timeline.events.filter(e => e.kind === "sensor").map(e => Math.round(e.t * 1000)), [...offline.track.cols.t_ms]);
  assert.equal(h.timeline.events.filter(e => e.kind === "navigation").length, 1);
  assert(h.timeline.events.some(e => e.kind === "config"));
  assert(h.timeline.events.some(e => e.kind === "flap"));
  assert(allowedSpeeds(h.timeline, 9600).includes(1));
  await h.connect(); await h.replay.play(); await h.clock.advance(101);
  assert.equal(h.sent.filter(e => e.line.startsWith("$SENS,")).length, 2);
  assert.equal(h.sent.filter(e => e.line.startsWith("$PFLAA,")).length, 1);
  assert(!h.sent.some(e => e.line.startsWith("$GPRMC,")));
});

test("advancing stale age counter recovers frozen timestamps without duplicating time when the clock resumes", async () => {
  const sample = (t, age) => sens(t).replace(",0,", `,${age},`);
  const text = [sample(1027, 1700259), sample(1027, 1700359), sample(1027, 1700559), sample(1027.4, 1700659)].join("\n");
  const h = harness(text), offline = convert(text);
  const expected = [1027000, 1027100, 1027300, 1027400];
  assert.deepEqual([...offline.track.cols.t_ms], expected);
  assert.deepEqual(h.timeline.events.map(e => Math.round(e.t * 1000)), expected);
  assert.equal(offline.timestampRecoveredSamples, 2);
  assert.equal(h.timeline.timestampRecoveredSamples, 2);
  assert.match(h.timeline.warning, /Timing recovered for 2 samples with frozen timestamps/);
  await h.connect(); await h.replay.play(); await h.clock.advance(401);
  assert.deepEqual(h.sent.filter(e => e.line.startsWith("$SENS,")).map(e => Math.round(e.t)), [1000, 1100, 1300, 1400]);
  assert.equal(h.replay.running, false);
  const reboot = convert([sample(1027, 1700259), sample(1027, 1700359), sample(7.682, 7590)].join("\n"));
  assert.deepEqual([...reboot.track.cols.t_ms], [1027000, 1027100, 1027200]);
});

test("coincident timestamps remain unchanged when the age counter cannot establish elapsed time", () => {
  for (const ages of [[0, 100], [1700000, 1700000], [1700000, 1699900], [1700000, 1710000]]) {
    const text = ages.map(age => sens(1027).replace(",0,", `,${age},`)).join("\n");
    const timeline = serialTimeline(text), offline = convert(text);
    assert.equal(timeline.timestampRecoveredSamples, 0);
    assert.equal(offline.timestampRecoveredSamples, 0);
    assert.deepEqual(timeline.events.map(e => e.t), [1027, 1027]);
  }
});

test("invalid pressure skips only affected samples and reports logs with no usable sensors", () => {
  for (const [field, values] of [[3, ["", " ", "NaN", "Infinity", "0", "-1"]],
    [4, ["", " ", "NaN", "Infinity", "0", "-1"]], [5, ["", " ", "NaN", "Infinity"]]]) {
    for (const value of values) {
      const fields = sens(300.1).split(","); fields[field] = value;
      const bad = fields.join(",");
      const timeline = serialTimeline([sens(300), bad, sens(300.2)].join("\n"));
      assert.equal(timeline.reason, undefined, `${field}: ${value}`);
      assert.equal(timeline.skippedPressureSensors, 1, `${field}: ${value}`);
      assert.match(timeline.warning, /1 \$SENS sample with invalid pressure skipped/);
      assert.deepEqual(timeline.events.map(e => e.t), [300, 300.2]);
      assert(timeline.events.every(e => e.line.startsWith("$SENS,")));
      const unusable = serialTimeline(bad);
      assert.equal(unusable.events.length, 0);
      assert.equal(unusable.skippedPressureSensors, 1);
      assert.match(unusable.reason, /No complete \$SENS samples.*invalid pressure/);
    }
  }
  // A signed differential-pressure reading near zero is valid on the ground.
  const signed = serialTimeline(sens(300).replace(",500,", ",-0.5,"));
  assert.equal(signed.skippedPressureSensors, 0);
  assert.match(signed.events[0].line, /,900,899,-0.5,/);
});

test("rejected pressure samples retain clock repairs, navigation timing and timeline bounds", () => {
  const bad = t => sens(t).replace(",899,", ",NaN,");
  const text = [bad(300), sens(300.1), "$GPRMC,120000,A,4800,N,01100,E,60,90,091126,,,A",
    bad(300.2), "!XCV,FLP,33", sens(300.3), bad(300.4)].join("\n");
  const timeline = serialTimeline(text), offline = convert(text);
  assert.equal(timeline.skippedPressureSensors, 3);
  assert.equal(timeline.start * 1000, offline.track.cols.t_ms[0]);
  assert.equal(timeline.end * 1000, offline.track.cols.t_ms.at(-1));
  assert.deepEqual(timeline.events.filter(e => e.kind === "sensor").map(e => e.t), [300.1, 300.3]);
  assert(Math.abs(timeline.events.find(e => e.kind === "navigation").t - 300.15) < 1e-9);
  assert.equal(timeline.events.find(e => e.kind === "flap").t, 300.25);
  assert(timeline.events.every(e => e.line && !e.line.includes("NaN")));
  const repaired = serialTimeline([bad(300), sens(900), bad(1)].join("\n"));
  assert.equal(repaired.start, 300);
  assert.equal(repaired.end, 300.2);
  assert.equal(repaired.events[0].t, 300.1, "rejected samples still anchor clock jumps");
});

test("serial streams remaining valid pressure samples at their original deadlines", async () => {
  const h = harness([ca1, sens(300).replace(",899,", ",NaN,"), sens(300.1),
    sens(300.2).replace(",900,", ",0,"), sens(300.3), sens(300.4).replace(",899,", ",0,")].join("\n"));
  await h.connect(); await h.replay.play(); await h.clock.advance(99);
  assert.equal(h.sent.filter(s => s.line.startsWith("$SENS,")).length, 0);
  await h.clock.advance(202);
  const sent = h.sent.filter(s => s.line.startsWith("$SENS,"));
  assert.deepEqual(sent.map(s => s.line.split(",")[1]), ["300.100", "300.300"]);
  assert(Math.abs(sent[0].t - 1100) < 1e-6);
  assert(Math.abs(sent[1].t - 1300) < 1e-6);
  assert.equal(h.replay.running, true, "the trailing rejected sample must not finish the clock early");
  assert(Math.abs(h.replay.time() - 300.301) < 1e-9);
  await h.clock.advance(100);
  assert.equal(h.replay.running, false);
  assert.equal(h.replay.time(), 300.4);
  assert(h.sent.every(s => !s.line.includes("NaN")));
});

test("serial sensors share inferred/corrected axes and Kelvin evidence with offline import", () => {
  for (const declared of [true, false]) {
    const lines = declared ? ["!XCV,FMT,SENS,7,1,-1,-1,1,-1,-1"] : [];
    const roll = -5 * Math.PI / 180;
    let expected;
    for (let i = 0; i < 220; i++) {
      const t = i * .2, pitch = (8 + 2 * Math.sin(t / 2)) * Math.PI / 180;
      const force = [Math.sin(pitch), -Math.sin(roll) * Math.cos(pitch), -Math.cos(roll) * Math.cos(pitch)];
      const gyro = [0, Math.cos(t / 2) * Math.cos(roll), -Math.cos(t / 2) * Math.sin(roll)];
      lines.push("$SENS," + [t.toFixed(3), 0, 1000, 999, 0, 285.59, ...force.map(x => -x), ...gyro, 4, 5, 6].join(","));
      lines.push(`$PXCV,0,1,0,1,1,12.44,1013,1000,0,-5,${pitch * 180 / Math.PI}`);
      if (!i) expected = [...force, ...gyro];
    }
    const text = lines.join("\n"), offline = convert(text);
    assert(declared ? offline.axesCorrected : offline.axesInferred);
    assert.equal(offline.temperatureKelvinSamples, 220);
    const f = serialTimeline(text).events[0].line.split("*")[0].split(",");
    assert(Math.abs(+f[6] - 12.44) < 1e-10);
    for (let i = 0; i < 6; i++) assert(Math.abs(+f[i + 7] - expected[i]) < 1e-10);
    assert.deepEqual(f.slice(13), ["4", "5", "6"]);
  }
});

test("configuration/flap restoration and modern weight updates are chronological", () => {
  const timeline = serialTimeline(log);
  const early = stateAt(timeline, 300);
  assert.equal(early.index, 0);
  assert.match(early.lines[0], /,100,0,0,102000\*/);
  const late = stateAt(timeline, 300.2);
  assert.match(late.lines[0], /,100,20,0,102000\*/);
  assert.match(late.lines[1], /!XCV,FLP,33\*/);
  assert.equal(timeline.events[late.index].kind, "sensor");
  assert.equal(serialTimeline(sens(1)).warning.startsWith("No logged"), true);
});

test("a prepared log converts like its text; the initial setup is the first complete CA1, else CMN", () => {
  const prepared = prepare(log);
  assert.deepEqual(serialTimeline(prepared), serialTimeline(log));
  assert.deepEqual(convert(prepared), convert(log));
  assert.equal(gliderFromLog(prepared), gliderFromLog(log));
  const incomplete = "!XCV,CA1,2305,324,100,0,0,", cmn = "!XCV,CMN,2305,300,90,0,0";
  assert.match(serialTimeline([incomplete, sens(300), cmn, ca1, sens(300.1)].join("\n")).initialConfig, /,324,100,0,0,102000\*/);
  assert.match(serialTimeline([incomplete, sens(300), cmn, sens(300.1)].join("\n")).initialConfig, /,300,90,0,0,\d+\*/);
  // the UTC date of the first dated fix, for the page's clock
  assert.equal(convert(log).utcDay, Date.UTC(2026, 10, 9));
  assert(Number.isNaN(convert(sens(300)).utcDay));
  assert.equal(convert(["$GPRMC,120000,A,4800,N,01100,E,60,90,311226,,,A", "$PXCVR,1000,0,0,-1"].join("\n")).utcDay, Date.UTC(2026, 11, 31));
});

test("initialization waits one second, config precedes samples and starts paused at 1x", async () => {
  const h = harness();
  assert.equal(h.calls.length, 0, "no automatic permission request");
  const connected = h.replay.connect();
  await h.clock.advance(999);
  assert.deepEqual(h.sent.map(s => s.line), ["$PFLAX,S*3c"]);
  assert.equal(h.replay.ready, false);
  await h.clock.advance(1); await connected;
  assert.equal(h.calls[1].baudRate, 115200);
  assert.equal(h.replay.speed, 1);
  assert.equal(h.replay.running, false);
  assert.match(h.sent[1].line, /^!XCV,CA1/);
  assert.equal(h.sent[2].line, command(0));
  assert.equal(h.sent.some(s => s.line.startsWith("$SENS")), false);
  await h.replay.play(); await h.clock.advance(0);
  assert.equal(h.replay.time(), 300);
  assert.equal(h.sent.filter(s => s.line.startsWith("$SENS")).length, 1);
  await h.clock.advance(99);
  assert.equal(h.sent.filter(s => s.line.startsWith("$SENS")).length, 1);
  await h.clock.advance(1.001);
  assert.equal(h.sent.filter(s => s.line.startsWith("$SENS")).length, 2);
});

test("pause/resume freezes the shared clock and preserves every sentence exactly once", async () => {
  const h = harness(); await h.connect(); await h.replay.play(); await h.clock.advance(75);
  await h.replay.pause(); const t = h.replay.time(), count = h.sent.length;
  await h.clock.advance(10000);
  assert.equal(h.replay.time(), t); assert.equal(h.sent.length, count);
  await h.replay.play(); await h.clock.advance(500);
  assert.deepEqual(h.sent.filter(s => !s.line.startsWith("$PFLAX")).slice(1).map(s => s.line), h.timeline.events.map(e => e.line));
  assert.equal(h.replay.running, false);
  assert.equal(h.replay.time(), h.timeline.end);
});

test("seek invalidates stale work waiting for backpressure and restores state before resuming", async () => {
  const h = harness(); await h.connect(); await h.replay.play(); await h.clock.advance(0);
  const gate = deferred(); h.writer.ready = gate.promise;
  await h.clock.advance(50); // navigation waits for ready
  const pending = h.replay.seek(300.2);
  gate.resolve(); await flush(); await pending; await h.clock.advance(0);
  assert(!h.sent.some(s => s.line.startsWith("$GPRMC")), "stale navigation never starts");
  const reset = h.sent.map(s => s.line).lastIndexOf(command(0));
  assert.match(h.sent[reset + 1].line, /CA1,2305,324,100,20,0,102000/);
  assert.match(h.sent[reset + 2].line, /FLP,33/);
  assert.equal(h.sent[reset + 3].line, command(1));
  assert.match(h.sent[reset + 4].line, /SENS,300\.200/);
  assert.equal(h.replay.time(), 300.2);
});

test("pause during an in-flight write never retransmits the handed-off sample", async () => {
  const h = harness(); await h.connect();
  const gate = deferred(), write = h.writer.write;
  h.writer.write = data => { write(data); return new TextDecoder().decode(data).startsWith("$SENS") ? gate.promise : Promise.resolve(); };
  await h.replay.play(); await h.clock.advance(0);
  const paused = h.replay.pause(); gate.resolve(); await paused;
  h.writer.write = write;
  await h.replay.play(); await h.clock.advance(500);
  assert.equal(h.sent.filter(s => s.line.startsWith("$SENS,300.000")).length, 1);
});

test("speed changes keep position continuous and honor baud and target limits", async () => {
  const h = harness(); await h.connect(); await h.replay.play(); await h.clock.advance(25);
  const t = h.replay.time(); await h.replay.setSpeed(2);
  assert.equal(h.replay.time(), t);
  await h.clock.advance(25); assert.equal(h.replay.time(), t + .05);
  assert(h.sent.some(s => s.line === command(2)));
  assert.throws(() => h.replay.setSpeed(300), /limit/);
  assert.deepEqual(allowedSpeeds(h.timeline, 4800), []);
  assert(allowedSpeeds(h.timeline, 115200).includes(1));
  assert(!allowedSpeeds(h.timeline, 1e9).includes(120));
});

test("event-loop lag and slow writes pause without catch-up bursts or sample loss", async () => {
  for (const slowWrite of [false, true]) {
    const h = harness(); await h.connect(); await h.replay.play();
    const write = h.writer.write;
    if (slowWrite) h.writer.write = data => {
      write(data);
      return new TextDecoder().decode(data).startsWith("$SENS") ? new Promise(resolve => h.clock.setTimer(resolve, 300)) : Promise.resolve();
    };
    else h.clock.t += 300; // timers were suspended, like a blocked/hidden page
    await h.clock.advance(300);
    assert.equal(h.replay.running, false);
    assert.match(h.replay.message, /lag/);
    assert(h.sent.filter(s => s.line.startsWith("$SENS")).length <= 1);
    h.writer.write = write;
    await h.replay.play(); await h.clock.advance(1000);
    assert.equal(h.sent.filter(s => s.line.startsWith("$SENS")).length, 5);
  }
});

test("lag watchdog freezes the clock while backpressure is blocked, including at the last sample", async () => {
  const h = harness(); await h.connect(); await h.replay.seek(300.4); await h.replay.play();
  const gate = deferred(); h.writer.ready = gate.promise;
  await h.clock.advance(251);
  assert.equal(h.replay.running, false);
  assert.match(h.replay.message, /lag/);
  assert(!h.sent.some(s => s.line.startsWith("$SENS,300.400")));
  gate.resolve(); await flush(); await h.replay.play(); await h.clock.advance(0);
  assert.equal(h.sent.filter(s => s.line.startsWith("$SENS,300.400")).length, 1);
});

test("chooser cancel, write failure, timeout, unplug and reconnect release resources", async () => {
  const canceled = harness();
  canceled.api.requestPort = async () => { throw Object.assign(new Error("cancel"), { name: "NotFoundError" }); };
  await canceled.replay.connect(); assert.equal(canceled.replay.connected, false);
  assert.match(canceled.replay.message, /canceled/);
  for (const failure of ["write", "timeout", "unplug"]) {
    const h = harness(); await h.connect();
    if (failure === "unplug") h.listeners.disconnect({ target: h.port });
    else {
      h.writer.write = () => failure === "write" ? Promise.reject(new Error("gone")) : new Promise(() => {});
      const p = h.replay.play(); await h.clock.advance(2000); await p;
    }
    await flush();
    assert.equal(h.replay.connected, false); assert.equal(h.replay.running, false);
    assert(h.calls.includes("release")); assert(h.calls.includes("close"));
    h.writer.write = async data => { h.sent.push({ t: h.clock.t, line: new TextDecoder().decode(data).trim() }); };
    await h.connect(); assert.equal(h.replay.ready, true);
    await h.replay.disconnect(); assert.equal(h.replay.connected, false);
  }
});

test("new log/rerun cancel old data and latest seek wins during initialization", async () => {
  const h = harness(); const connecting = h.replay.connect(); await h.clock.advance(100);
  h.replay.seek(300.1); const seek = h.replay.seek(300.2);
  await h.clock.advance(2000); await connecting; await seek;
  assert.equal(h.replay.ready, true); assert.equal(h.replay.time(), 300.2);
  await h.replay.play(); await h.clock.advance(0);
  const other = serialTimeline([ca1, sens(400), sens(400.1)].join("\n"));
  const before = h.sent.length;
  await h.replay.setTimeline(other); await h.clock.advance(200);
  assert.equal(h.replay.running, false);
  await h.replay.play(); await h.clock.advance(500);
  assert(h.sent.slice(before).filter(s => s.line.startsWith("$SENS")).every(s => s.line.startsWith("$SENS,400.")));
  await h.replay.setTimeline(serialTimeline("$PXCVR,0"));
  assert.equal(h.replay.connected, false);
});

test("disconnect cannot be canceled by play/seek or a simultaneous log replacement", async () => {
  const h = harness(); await h.connect(); await h.replay.play(); await h.clock.advance(0);
  const gate = deferred(); h.writer.ready = gate.promise;
  const disconnect = h.replay.disconnect();
  h.replay.play(); h.replay.seek(300.3);
  const replace = h.replay.setTimeline(serialTimeline([ca1, sens(500), sens(500.1)].join("\n")));
  gate.resolve(); await disconnect; await replace;
  assert.equal(h.replay.connected, false);
  assert.equal(h.replay.timeline.start, 500);
  assert.equal(h.calls.filter(c => c === "close").length, 1);
});

test("coincident sensor timestamps exceeding wire capacity are rejected before the chooser", async () => {
  const h = harness(Array.from({ length: 2000 }, () => sens(300)).join("\n"));
  assert.deepEqual(allowedSpeeds(h.timeline, 921600), []);
  await assert.rejects(h.replay.connect(), /exceeds/);
  assert.equal(h.calls.length, 0);
});

test("page controls connect at the selected offline time, render from serial time and pause when hidden", async () => {
  const h = harness(), events = {}, elements = {};
  const element = id => elements[id] ||= { value: "", checked: false, options: [], addEventListener() {}, setAttribute() {} };
  element("baud").value = "115200";
  element("speed").value = "10";
  element("speed").options = [1, 2, 5, 10, 30, 60, 120, 300].map(value => ({ value: String(value), disabled: false }));
  const script = pageScript(__dirname);
  const setup = script.slice(script.indexOf("let serialWasConnected"), script.indexOf("// ---------- loading ----------"));
  const playback = script.slice(script.indexOf("function seek("), script.indexOf("// the settings panel:"));
  const context = vm.createContext({
    $: element, AirMassSerial: { ...require("./serial.js"), SerialReplay: class {
      constructor(opts) { h.replay.onChange = opts.onChange; return h.replay; }
    } }, navigator: { serial: h.api }, window: { isSecureContext: true, addEventListener() {} },
    document: { hidden: false, addEventListener: (name, fn) => { events[name] = fn; } },
    D: { t: [300, 300.4], n: 2, tStart: 300 }, T: 300.2, view: [300, 300.4], playing: false, loading: false, replaying: false,
    audio: { ctx: null }, performance: { now: h.clock.now }, playbackFrame: null,
    requestAnimationFrame: () => 1, cancelAnimationFrame() {}, scheduleDraw() {}, soundOff() {}, soundOn: async () => {},
    drawAll() {}, invalidateCharts() {}, setStatus() {},
  });
  vm.runInContext(setup + playback + "\nserialControls();", context);
  const connect = element("serialbtn").onclick(); await h.clock.advance(1000); await connect;
  assert.equal(h.replay.time(), 300.2);
  assert.equal(element("speed").value, "1");
  assert.equal(element("speed").options.at(-1).disabled, true);
  vm.runInContext("play();", context); await h.clock.advance(50);
  const generation = h.replay.generation;
  vm.runInContext("frame(performance.now());", context);
  assert.equal(context.T, h.replay.time());
  assert.equal(h.replay.generation, generation, "animation never seeks the hardware");
  // dragging the slider moves the page only; the instrument seeks once, where the slider is let go
  for (const value of ["300.3", "300.35"]) { element("slider").value = value; element("slider").oninput(); }
  assert.equal(h.replay.generation, generation, "dragging does not seek the hardware");
  assert.equal(context.T, 300.35);
  await h.clock.advance(20); vm.runInContext("frame(performance.now());", context);
  assert.equal(context.T, 300.35, "serial time does not move a dragged slider");
  element("slider").onchange();
  assert.equal(h.replay.generation, generation + 1);
  assert.equal(h.replay.position, 300.35);
  vm.runInContext("endScrub();", context);
  assert.equal(h.replay.generation, generation + 1, "letting go after the change does not seek again");
  await h.clock.advance(50); vm.runInContext("frame(performance.now());", context);
  assert.equal(context.T, h.replay.time(), "the page follows the instrument again");
  context.document.hidden = true; events.visibilitychange(); await flush();
  assert.equal(h.replay.running, false); assert.equal(context.playing, false);
  assert.match(element("serialstatus").textContent, /hidden/);
});

test("page hooks keep serial time separate from offline frame advancement; assets include serial", () => {
  const script = pageScript(__dirname);
  new vm.Script(script);
  const functionText = name => grab(script, `function ${name}(`);
  assert.match(functionText("frame"), /T = serial.time\(\)/);
  assert(!functionText("frame").split("const dt")[0].includes("seek(T)"), "drawing does not issue a hardware seek");
  assert.match(script, /visibilitychange[\s\S]*?document.hidden && serial.connected\) serial.pause/);
  assert.match(script, /option && !option.disabled/);
  assert.match(fs.readFileSync(__dirname + "/deploy.sh", "utf8"), /SCRIPTS="[^"]*serial.js/);
});
