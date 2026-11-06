// Flight log to AirMass recording, in the browser (index.html) and in node (check.js).
//
// Two kinds of log:
//  * an AirMass recording ("NMEA Output" Recording, main/airmass/AirMassRecord.h): its $PXCV? sentences pass through
//  * an XCVario "$SENS" sensor log: converted like tools/airmass_sim/sens2rec.py (keep the two in step)
// Either way the result also has one track sample per sensor cycle: position and GNSS altitude of the last fix, the
// static pressure altitude, the device's TE vario ($PXCV), IAS, TAS and the polar sink, for the map and the charts.
(function (root) {
  "use strict";
  const RHO0 = 1.225, R_AIR = 287.058, G0 = 9.80665, P0 = 101325.0, T0DIVL = 44330.76923, G0DIVRXL = 5.25588;
  const KNOT = 0.514444;

  // the firmware's glider types (components/glider/PolarTable.txt and FlapTable.txt, generated into gliders.js by
  // make_gliders.py, with the device's default Vne); the ASG 29 as sens2rec.py has it, so both convert alike
  const TABLE = typeof module !== "undefined" && module.exports ? require("./gliders.js") : root.GLIDER_TABLE || {};
  const GLIDERS = Object.assign({}, TABLE, {
    "ASG 29/18": { wingload: 42.9, polar: [[82, -0.47], [132, -0.80], [184, -1.80]], area: 10.5, vne: 280.0, flaps: [180, 142, 110, 94, 57] },
    "ASG 29/15": { wingload: 47.7, polar: [[86, -0.58], [127, -0.80], [182, -1.80]], area: 9.22, vne: 285.0, flaps: [198, 156, 121, 104, 60] },
  });

  function checksum(body) {
    let cs = 0;
    for (let i = 1; i < body.length; i++) cs ^= body.charCodeAt(i);
    return body + "*" + cs.toString(16).padStart(2, "0");
  }
  const fx = (v, d) => (Number.isFinite(v) ? v.toFixed(d) : "");
  const fg = (v, p) => (Number.isFinite(v) ? String(parseFloat(v.toPrecision(p))) : "");
  const sentence = (name, fields) => checksum("$" + name + (fields.length ? "," + fields.join(",") : ""));

  // S2F::recalculatePolar and getSink
  function makePolar(g, ballastFactor, bugs) {
    const [[v1, w1], [v2, w2], [v3, w3]] = g.polar.map(([v, w]) => [v / 3.6, w]);
    const d = v1 * v1 * (v2 - v3) + v2 * v2 * (v3 - v1) + v3 * v3 * (v1 - v2);
    const a2 = ((v2 - v3) * (w1 - w3) + (v3 - v1) * (w2 - w3)) / d;
    const a1 = (w2 - w3 - a2 * (v2 * v2 - v3 * v3)) / (v2 - v3);
    const a0 = w3 - a2 * v3 * v3 - a1 * v3;
    const lf = Math.sqrt(ballastFactor);
    const b = (bugs + 100.0) / 100.0;
    const p = { a0: a0 * lf * b, a1: a1 * b, a2: a2 / lf * b };
    p.vStall = Math.sqrt(2.0 * g.wingload * G0 / (RHO0 * 1.4)) * 1.05 * Math.sqrt(lf);
    p.sink = (v, n) => {
      if (v < p.vStall * 0.9) return 0.0;
      const sqn = Math.sqrt(Math.max(n, 0.0));
      return p.a0 * n * sqn + p.a1 * v * n + p.a2 * v * v * sqn;
    };
    return p;
  }

  // Flap::prepLevels and getOptimum: the recommended lever position
  function makeFlaps(g, ballastFactor) {
    if (!g.flaps || !g.flaps.length) return null;
    const prep = g.flaps.map((s) => s / 3.6 * Math.sqrt(ballastFactor));
    const delta = [];
    let prev = g.vne / 3.6;
    for (const p of prep) {
      delta.push(Math.min(p - prev, -1.0));
      prev = p;
    }
    return (ias, load) => {
      load = Math.max(load, 0.3);
      const v = ias / Math.sqrt(load);
      let i = 0;
      while (i < prep.length && v <= prep[i]) i++;
      i = Math.min(i, prep.length - 1);
      if (v < 30.0 / 3.6) return 0.0;
      return Math.min(Math.max(i + 0.5 + (v - prep[i]) / delta[i], -0.1), prep.length - 1);
    };
  }

  function fields(line) {
    const star = line.indexOf("*");
    return (star > 0 ? line.slice(1, star) : line.slice(1)).split(",");
  }

  // a growable column store for the track samples
  class Track {
    constructor() {
      // te_new: 1 on the first sample after a $PXCV (the device's TE vario, 5 Hz in the log), for the interpolation
      this.names = ["t_ms", "utc", "lat", "lon", "gnss_alt", "gnss_speed", "gnss_track", "baro_alt", "te_vario", "te_new", "ias", "tas", "polar_sink", "running"];
      this.n = 0;
      this.cap = 1 << 16;
      this.cols = {};
      for (const k of this.names) this.cols[k] = new Float64Array(this.cap);
    }
    push(o) {
      if (this.n === this.cap) {
        this.cap *= 2;
        for (const k of this.names) {
          const a = new Float64Array(this.cap);
          a.set(this.cols[k]);
          this.cols[k] = a;
        }
      }
      for (const k of this.names) this.cols[k][this.n] = o[k] ?? NaN;
      this.n++;
    }
    done() {
      for (const k of this.names) this.cols[k] = this.cols[k].slice(0, this.n);
      return this;
    }
  }

  // opts: glider (a GLIDERS name, or an object like them), mass [kg] (0: the logged ballast), bugs [%] (<0: logged),
  // noFlaps. Returns { kind, rec: [recording lines], track, starts, frames, fixes }.
  function convert(text, opts = {}) {
    const isRecording = /\$PXCVR,/.test(text.slice(0, 4e6)) || (!/\$SENS,/.test(text.slice(0, 4e6)) && text.includes("$PXCVR,"));
    return isRecording ? passRecording(text) : convertSens(text, opts);
  }

  function parseRmc(f) {
    if (f.length > 8 && f[2] === "A" && f[7]) {
      const hh = +f[1].slice(0, 2), mm = +f[1].slice(2, 4), ss = +f[1].slice(4);
      const lat = (+f[3].slice(0, 2) + +f[3].slice(2) / 60.0) * (f[4] === "N" ? 1 : -1);
      const lon = (+f[5].slice(0, 3) + +f[5].slice(3) / 60.0) * (f[6] === "E" ? 1 : -1);
      const speed = +f[7] * KNOT;
      const track = f[8] ? +f[8] : NaN;
      if ([hh, mm, ss, lat, lon, speed].every(Number.isFinite)) return { utc: hh * 3600 + mm * 60 + ss, speed, track, lat, lon };
    }
    return null;
  }

  // an AirMass recording: our sentences pass through, the track from the frames and the navigation sentences
  function passRecording(text) {
    const rec = [];
    const track = new Track();
    let fix = null, gnssAlt = NaN, te = NaN, teNew = 0, running = false, starts = 0, frames = 0, fixes = 0;
    for (const raw of text.split("\n")) {
      const line = raw.trim();
      if (line.startsWith("$PXCV") && line[5] !== ",") {
        rec.push(line);
        const kind = line[5];
        if (kind === "S") { running = true; starts++; }
        else if (kind === "X") running = false;
        else if (kind === "G") fixes++;
        else if (kind === "R") {
          const f = fields(line);
          frames++;
          track.push({ t_ms: +f[1], utc: fix ? fix.utc : NaN, lat: fix ? fix.lat : NaN, lon: fix ? fix.lon : NaN, gnss_alt: gnssAlt,
            gnss_speed: fix ? fix.speed : NaN, gnss_track: fix ? fix.track : NaN, baro_alt: f[10] === "" ? NaN : +f[10], te_vario: te,
            te_new: teNew, ias: +f[8], tas: +f[9], polar_sink: NaN, running: running ? 1 : 0 });
          teNew = 0;
        }
      } else if (line.startsWith("$PXCV,")) {
        const v = parseFloat(fields(line)[1]);
        if (Number.isFinite(v)) { te = v; teNew = 1; }
      } else if (line.startsWith("$GPRMC,") || line.startsWith("$GNRMC,")) {
        fix = parseRmc(fields(line)) || fix;
      } else if (line.startsWith("$GPGGA,") || line.startsWith("$GNGGA,")) {
        const v = parseFloat(fields(line)[9]);
        if (Number.isFinite(v)) gnssAlt = v;
      }
    }
    return { kind: "AirMass recording", rec, track: track.done(), starts, frames, fixes };
  }

  // sens2rec.py
  function convertSens(text, opts) {
    const g = typeof opts.glider === "object" ? opts.glider : GLIDERS[opts.glider || "ASG 29/18"];
    const mass = opts.mass || 0, bugsOpt = opts.bugs ?? -1, lever = opts.leverArm || 0;
    const rec = [];
    const track = new Track();
    const pxcv = { vario: NaN, bugs: 0.0, ballast: 1.0, roll: 0.0, pitch: 0.0 };
    let teNew = 0;
    let fix = null, gnssAlt = NaN, pendingFix = null, running = false, above = 0, below = 0;
    let setupLine = null, setupCount = 0, starts = 0, frames = 0, fixes = 0, polar = null, flaps = null, tMs = 0;

    const setup = () => {
      const area = g.area || 10.0;
      const bf = mass > 0 ? mass / (g.wingload * area) : (pxcv.ballast > 0.5 ? pxcv.ballast : 1.0);
      polar = makePolar(g, bf, bugsOpt >= 0 ? bugsOpt : pxcv.bugs);
      flaps = opts.noFlaps ? null : makeFlaps(g, bf);
      const m = bf * g.wingload * area;
      return sentence("PXCVC", [fx(lever, 4), fx(m, 2), fx(area, 4), fg(polar.a0, 8), fg(polar.a1, 8), fg(polar.a2, 8), fx(polar.vStall, 4),
        "1", "1.000", "1.000", "100.000"]);
    };

    for (const raw of text.split("\n")) {
      if (raw.startsWith("$PXCV,")) {
        const f = fields(raw.trim());
        const num = (s) => (s === undefined || s === "" ? NaN : +s);
        if (Number.isFinite(num(f[1]))) { pxcv.vario = num(f[1]); teNew = 1; }
        if (Number.isFinite(num(f[3]))) pxcv.bugs = num(f[3]);
        else if (f[3] === "") pxcv.bugs = 0.0;
        if (Number.isFinite(num(f[4]))) pxcv.ballast = num(f[4]);
        if (f.length > 11 && Number.isFinite(num(f[10]))) pxcv.roll = num(f[10]);
        if (f.length > 11 && Number.isFinite(num(f[11]))) pxcv.pitch = num(f[11]);
      } else if (raw.startsWith("$GPRMC,") || raw.startsWith("$GNRMC,")) {
        const r = parseRmc(fields(raw.trim()));
        if (r) { fix = r; pendingFix = r; }
      } else if (raw.startsWith("$GPGGA,") || raw.startsWith("$GNGGA,")) {
        const v = parseFloat(fields(raw.trim())[9]);
        if (Number.isFinite(v)) gnssAlt = v;
      } else if (raw.startsWith("$SENS,")) {
        const f = fields(raw.trim());
        if (f.length < 13) continue;
        const t = Math.round(+f[1] * 1000.0), age = parseInt(f[2], 10);
        const p = +f[3] * 100.0, q = +f[5], oat = +f[6];
        const acc = [+f[7], +f[8], +f[9]];
        const gyro = [+f[10], +f[11], +f[12]].map((x) => x * Math.PI / 180);
        if (![t, age, p, q, oat, ...acc, ...gyro].every(Number.isFinite)) continue;
        tMs = t;
        const ias = q > 0.0 ? Math.sqrt(2.0 * q / RHO0) : 0.0;
        const tas = p > 0 ? ias * Math.sqrt(RHO0 * R_AIR * (oat + 273.15) / p) : 0.0;
        const alt = p > 0 ? T0DIVL * (1.0 - Math.pow(p / P0, 1.0 / G0DIVRXL)) : NaN;
        const gs = fix ? fix.speed : 0.0;

        // airborne: flying speed and moving over ground for 10 s; landed: 30 s slow
        if (!running) {
          above = ias > 20.0 && gs > 15.0 ? above + 1 : 0;
          if (above >= 100) {
            running = true;
            starts++;
            setupLine = setup();
            rec.push(setupLine);
            const rr = pxcv.roll * Math.PI / 180, pp = pxcv.pitch * Math.PI / 180;
            const down = [-Math.sin(pp), Math.sin(rr) * Math.cos(pp), Math.cos(rr) * Math.cos(pp)];
            const warm = ["1.00000", "0.000000", "0.0000", "1.00000"].concat(Array(7).fill("1.000000")); // cold, flap offsets unknown
            rec.push(sentence("PXCVS", [String(t), fx(alt, 2)].concat(down.map((x) => fx(x, 6)), gyro.map((x) => fx(x, 6)), warm)));
            setupCount = 0;
          }
        } else {
          below = ias < 10.0 ? below + 1 : 0;
          if (below >= 300) {
            running = false;
            below = above = 0;
            rec.push(sentence("PXCVX", [String(t)]));
          }
        }
        let sink = NaN;
        if (running) {
          const load = 1.0 / Math.max(Math.cos(pxcv.roll * Math.PI / 180), 0.3);
          const flap = flaps ? Math.round(flaps(ias, load)) : NaN;
          sink = polar.sink(ias, Math.max(-acc[2], 0.3));
          setupCount++;
          if (setupCount % 100 === 0) rec.push(setupLine); // like the device, every 10 s
          rec.push(sentence("PXCVR", [String(t)].concat(acc.map((x) => fx(x, 5)), gyro.map((x) => fx(x, 6)), [fx(ias, 3), fx(tas, 3), fx(alt, 2), fx(flap, 3)])));
          frames++;
          // the fix arrived age ms before this cycle; the estimator gets it in this cycle
          if (pendingFix && pendingFix.speed > 1.0 && Number.isFinite(pendingFix.track)) {
            const trk = pendingFix.track * Math.PI / 180;
            rec.push(sentence("PXCVG", [String(t - age), fx(pendingFix.speed * Math.cos(trk), 3), fx(pendingFix.speed * Math.sin(trk), 3),
              "0.500", "750", "0", "0.000", "0.000"]));
            fixes++;
          }
          pendingFix = null;
        }
        track.push({ t_ms: t, utc: fix ? fix.utc : NaN, lat: fix ? fix.lat : NaN, lon: fix ? fix.lon : NaN, gnss_alt: gnssAlt,
          gnss_speed: fix ? fix.speed : NaN, gnss_track: fix ? fix.track : NaN, baro_alt: alt, te_vario: pxcv.vario, te_new: teNew, ias, tas,
          polar_sink: sink, running: running ? 1 : 0 });
        teNew = 0;
      }
    }
    if (running) rec.push(sentence("PXCVX", [String(tMs)]));
    return { kind: "XCVario sensor log", rec, track: track.done(), starts, frames, fixes };
  }

  const api = { convert, GLIDERS };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.AirMassConvert = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
