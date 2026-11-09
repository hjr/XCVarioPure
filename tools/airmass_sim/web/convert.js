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

  // Bad/missing OAT (some logs are stuck at 84.9°C): 25°C at sea level, standard 6.5°C/km lapse rate.
  // Use altitude above sea level, falling back to pressure altitude when GNSS altitude is unavailable.
  const plausibleTemperature = (oat) => Number.isFinite(oat) && oat >= -60 && oat <= 50;
  const estimatedTemperature = (alt) => 25 - 0.0065 * (Number.isFinite(alt) ? alt : 0);

  // the firmware's glider types (components/glider/PolarTable.txt and FlapTable.txt, generated into gliders.js by
  // make_gliders.py, with the device's default Vne); the ASG 29 with its own Vne, as sens2rec.py has it, so both
  // convert alike
  const TABLE = typeof module !== "undefined" && module.exports ? require("./gliders.js") : root.GLIDER_TABLE || {};
  const GLIDERS = Object.assign({}, TABLE, {
    "ASG 29/18": { ...TABLE["ASG 29/18"], vne: 280.0 },
    "ASG 29/15": { ...TABLE["ASG 29/15"], vne: 285.0 },
  });

  // the glider of a device glider type (the CA1 telegram), or undefined
  const gliderById = (id) => Object.keys(GLIDERS).find((name) => GLIDERS[name].id === id);
  const DEFAULT_GLIDER = "ASG 29/18";

  // What the device logs besides the sensors (main/protocol/nmea/XCVarioMsg.cpp): "!XCV,CA1,glider type,empty weight,
  // crew weight,ballast,speed cal,QNH", since protocol version 2 "!xcv,empty-weight|crew-weight|bal-water,kg" on
  // changes, and in a sensor log "!XCV,FLP,lever x 10" when the flap lever moves (FlapsBox: the flap sensor's position
  // to 0.5, or without a sensor the recommendation). Updates logged (kg, lever) from the line; returns "flap", "setup"
  // (glider or weights), or false if it is none
  function parseXcv(line, logged) {
    const f = fields(line.trim());
    if (!/^xcv$/i.test(f[0]) || f.length < 3) return false;
    const num = (x) => (x === "" ? NaN : +x);
    if (f[1] === "FLP") {
      if (Number.isFinite(num(f[2]))) logged.flap = num(f[2]) / 10;
      return "flap";
    } else if (f[1] === "CA1") {
      const v = f.slice(2, 6).map(num);
      if (v.length === 4 && v.every(Number.isFinite)) [logged.type, logged.empty, logged.crew, logged.ballast] = v;
    } else {
      const key = { "empty-weight": "empty", "crew-weight": "crew", "bal-water": "ballast" }[f[1]];
      if (!key) return false;
      if (Number.isFinite(num(f[2]))) logged[key] = num(f[2]);
    }
    return "setup";
  }

  // a logger's text before the sentence, and the ';' separators of old $SENS logs
  function cleanLine(raw) {
    const line = raw[0] === "$" || raw[0] === "!" ? raw : raw.slice(Math.max(raw.lastIndexOf("$"), raw.lastIndexOf("!")));
    return line.startsWith("$SENS;") ? line.replace(/;/g, ",") : line;
  }

  // Check legacy metadata against independent PXCV attitude/temperature readings. Some logs declare
  // an ENU conversion although their acceleration is gravity (not specific force) and gyros are body rates.
  class SensorConventions {
    constructor() {
      this.sensor = this.prev = null;
      this.kelvin = this.temperaturePairs = this.ground = this.rates = 0;
      this.gravityDeclared = this.gravityCorrected = this.rateDeclared = this.rateCorrected = 0;
    }
    sens(f) {
      const num = (i) => f[i] === undefined || f[i] === "" ? NaN : +f[i];
      const t = f[1]?.includes(".") ? num(1) : num(1) / 1000;
      const a = [7, 8, 9].map(num), g = [10, 11, 12].map(i => num(i) * Math.PI / 180);
      this.sensor = [t, num(5), ...a, ...g].every(Number.isFinite) ? { t, q: num(5), oat: num(6), a, g } : null;
    }
    pxcv(f) {
      const s = this.sensor;
      if (!s) return;
      const temp = f[6] === "" ? NaN : +f[6];
      if (plausibleTemperature(s.oat - 273.15) && plausibleTemperature(temp)) {
        this.temperaturePairs++;
        if (Math.abs(s.oat - 273.15 - temp) < 2) this.kelvin++;
      }
      const roll = f[10] === "" ? NaN : +f[10] * Math.PI / 180;
      const pitch = f[11] === "" ? NaN : +f[11] * Math.PI / 180;
      if (![roll, pitch].every(Number.isFinite)) return;
      const norm = Math.hypot(...s.a), rate = Math.hypot(...s.g);
      if (s.q < 50 && norm > 0.85 && norm < 1.15 && rate < 3 * Math.PI / 180 &&
          Math.abs(roll) < Math.PI / 6 && Math.abs(pitch) < Math.PI / 7 && Math.abs(Math.sin(pitch)) > 0.05) {
        const force = [Math.sin(pitch), -Math.sin(roll) * Math.cos(pitch), -Math.cos(roll) * Math.cos(pitch)];
        for (let j = 0; j < 3; j++) {
          this.gravityDeclared += (s.a[j] / norm * (j === 0 ? 1 : -1) - force[j]) ** 2;
          this.gravityCorrected += (-s.a[j] / norm - force[j]) ** 2;
        }
        this.ground++;
      }
      const dt = this.prev ? s.t - this.prev.t : 0;
      if (dt > 0.15 && dt < 0.3 && Math.abs(roll) < Math.PI / 3 && Math.abs(pitch) < Math.PI / 6 && rate < 1.4) {
        const measured = (pitch - this.prev.pitch) / dt;
        const predicted = s.g[1] * Math.cos(roll) - s.g[2] * Math.sin(roll);
        if (Math.abs(measured) < Math.PI / 6) {
          this.rateDeclared += (-predicted - measured) ** 2;
          this.rateCorrected += (predicted - measured) ** 2;
          this.rates++;
        }
      }
      this.prev = { t: s.t, pitch };
    }
    apply(out) {
      out.temperatureKelvin = this.kelvin >= 20 && this.kelvin >= this.temperaturePairs * 0.8;
      const f = out.fmt;
      if (f?.start === 7 && f.factors.join(",") === "1,-1,-1,1,-1,-1" && this.ground >= 50 && this.rates >= 100 &&
          this.gravityCorrected / this.ground < 0.01 && this.gravityCorrected < this.gravityDeclared * 0.5 &&
          this.rateCorrected < this.rateDeclared * 0.5) {
        out.fmt = { start: 7, factors: [-1, -1, -1, 1, 1, 1] };
        out.axesCorrected = true;
      }
    }
  }

  // One pass through the log before the conversion, for what applies to all of it wherever it stands:
  //  "!XCV,FMT,SENS,start,factors...": the $SENS fields from index start on (SENS itself is 0) are multiplied by the
  //    factors (logs before 2025: east-north-up axes)
  //  "!XCV,CMN,glider type,empty weight,crew weight,ballast,speed cal": CA1 without the QNH; a CA1 in the log wins
  //  the QNH [hPa]: of CA1, else of $PXCV, else from the GNSS altitude and static pressure of the first fixes on the ground
  function scanLog(lines) {
    const out = { fmt: null, cmn: null, ca1: null, qnh: NaN, qnhFrom: "" };
    const pairs = [];
    const conventions = new SensorConventions();
    let p = NaN, q = NaN, pxcv = NaN;
    for (const raw of lines) {
      const line = cleanLine(raw.trim());
      if (line.startsWith("!XCV,")) {
        const f = fields(line);
        if (f[1] === "FMT" && f[2] === "SENS") {
          const factors = f.slice(4).map(Number);
          if (Number.isInteger(+f[3]) && factors.length && factors.every(Number.isFinite)) out.fmt = { start: +f[3], factors };
        } else if (f[1] === "CMN") {
          const v = f.slice(2, 6).map(Number);
          if (v.length === 4 && v.every(Number.isFinite)) out.cmn = v;
        } else if (f[1] === "CA1") {
          const v = f.slice(2, 6).map(x => x === "" ? NaN : Number(x));
          if (!out.ca1 && v.length === 4 && v.every(Number.isFinite)) out.ca1 = v;
          const hpa = +f[7] > 8e4 ? +f[7] / 100 : +f[7]; // Pa, or hPa
          if (hpa > 800 && hpa < 1100) [out.qnh, out.qnhFrom] = [hpa, "CA1"];
        }
      } else if (line.startsWith("$PXCV,")) {
        conventions.pxcv(fields(line));
        if (!Number.isFinite(pxcv) && +fields(line)[7] > 800) pxcv = +fields(line)[7];
      } else if (line.startsWith("$SENS,")) {
        const f = fields(line);
        conventions.sens(f);
        p = +f[3];
        q = +f[5];
      } else if (pairs.length < 20 && (line.startsWith("$GPGGA,") || line.startsWith("$GNGGA,"))) {
        const f = fields(line), h = parseFloat(f[9]);
        // on the ground only: high up the air is too far from ISA (a log started at 3500 m came out 24 hPa high)
        if (+f[6] > 0 && Number.isFinite(h) && p > 300 && q < 150) pairs.push([p, h]);
      }
    }
    conventions.apply(out);
    if (out.qnhFrom) return out;
    if (Number.isFinite(pxcv)) return Object.assign(out, { qnh: pxcv, qnhFrom: "$PXCV" });
    if (pairs.length) {
      const median = (a) => a.sort((x, y) => x - y)[a.length >> 1];
      const isa = Math.pow(1.0 - median(pairs.map((x) => x[1])) / T0DIVL, G0DIVRXL);
      Object.assign(out, { qnh: median(pairs.map((x) => x[0])) / isa, qnhFrom: "GNSS altitude and pressure" });
    }
    return out;
  }

  // Detect the initial logged type even when the configuration telegram occurs later in the file.
  function gliderFromLog(text) {
    const scan = scanLog(text.split("\n"));
    const setup = scan.ca1 || scan.cmn;
    return setup ? gliderById(setup[0]) : undefined;
  }

  // the $SENS time stamp [ms] made monotone: a clock jump (the UTC synchronisation, midnight, a reboot) continues at the
  // sensor cycle; shorter holes are lost log lines (up to 2 s in flight), the time goes on across them. The estimator
  // runs through a reboot in flight: a real log showed it recovers within 30 s, a cold restart here took 2 min
  function stampRepair() {
    let last = NaN, correction = 0;
    return (raw) => {
      const d = raw - correction - last;
      if (d < 0 || d > 5000) correction = raw - last - 100;
      return (last = raw - correction);
    };
  }

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

  // opts: glider (a GLIDERS name, "log": the glider type the log names (CA1), else the ASG 29/18, or an object like
  // them), mass [kg] (0: from the log: its empty, crew and ballast weights (CA1), else the ballast factor of $PXCV),
  // bugs [%] (<0: logged), noFlaps. Returns { kind, rec: [recording lines], track, starts, frames, fixes, glider (the
  // name used), gliderFrom ("set", "log", "default": the log names none), mass [kg, of the last setup], massFrom ("set",
  // "logged weights", "logged ballast factor"), flapsFrom ("logged": the lever (FLP) from its first telegram on, "the
  // recommendation", "": none), temperatureEstimatedSamples (sensor samples using the OAT estimate) }.
  function convert(text, opts = {}) {
    const isRecording = /\$PXCVR,/.test(text.slice(0, 4e6)) || (!/\$SENS[,;]/.test(text.slice(0, 4e6)) && text.includes("$PXCVR,"));
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
    const mass = opts.mass || 0, bugsOpt = opts.bugs ?? -1, lever = opts.leverArm || 0;
    const logged = { type: NaN, empty: NaN, crew: NaN, ballast: NaN, flap: NaN };
    let g = null, gliderName = "", gliderFrom = "", massUsed = NaN, massFrom = "", loggedFlapFrames = 0;
    const rec = [];
    const track = new Track();
    const pxcv = { vario: NaN, bugs: 0.0, ballast: 1.0, roll: 0.0, pitch: 0.0 };
    let teNew = 0;
    let fix = null, gnssAlt = NaN, pendingFix = null, running = false, above = 0, below = 0;
    let setupLine = null, setupCount = 0, starts = 0, frames = 0, fixes = 0, polar = null, flaps = null, tMs = 0;
    let temperatureEstimatedSamples = 0, temperatureKelvinSamples = 0;
    const lines = text.split("\n");
    const scan = scanLog(lines), monotone = stampRepair();
    const initialSetup = scan.ca1 || scan.cmn;
    if (initialSetup) [logged.type, logged.empty, logged.crew, logged.ballast] = initialSetup;

    const setup = () => {
      // the glider and the weights as logged so far
      if (typeof opts.glider === "object") { g = opts.glider; gliderName = "custom"; gliderFrom = "set"; }
      else {
        const named = opts.glider === "log" ? gliderById(logged.type) : opts.glider;
        gliderName = named || DEFAULT_GLIDER;
        gliderFrom = opts.glider !== "log" ? "set" : named ? "log" : "default";
        g = GLIDERS[gliderName];
      }
      const area = g.area || 10.0, ref = g.wingload * area;
      const loggedMass = logged.empty > 0 ? logged.empty + (logged.crew || 0) + (logged.ballast || 0) : NaN;
      let bf;
      [bf, massFrom] = mass > 0 ? [mass / ref, "set"] : loggedMass > 0 ? [loggedMass / ref, "logged weights"] : [pxcv.ballast > 0.5 ? pxcv.ballast : 1.0, "logged ballast factor"];
      massUsed = bf * ref;
      polar = makePolar(g, bf, bugsOpt >= 0 ? bugsOpt : pxcv.bugs);
      flaps = opts.noFlaps ? null : makeFlaps(g, bf);
      return sentence("PXCVC", [fx(lever, 4), fx(massUsed, 2), fx(area, 4), fg(polar.a0, 8), fg(polar.a1, 8), fg(polar.a2, 8), fx(polar.vStall, 4),
        "1", "1.000", "1.000", "100.000"]);
    };

    for (const original of lines) {
      const raw = cleanLine(original);
      const xcv = raw[0] === "!" && parseXcv(raw, logged);
      if (xcv) {
        // a new glider or weight in flight (water ballast dumped): the setup the device sends from now on
        if (xcv === "setup" && running) setupLine = setup();
        continue;
      }
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
        if (f.length < 13 || f.slice(1, 13).some((v, i) => i !== 5 && v === "")) continue; // OAT may be missing
        if (scan.fmt) scan.fmt.factors.forEach((c, i) => { if (f[scan.fmt.start + i]) f[scan.fmt.start + i] *= c; });
        const stamp = Math.round(f[1].includes(".") ? +f[1] * 1000.0 : +f[1]); // old logs: integer milliseconds
        // the time since the last fix; old logs also hold other values here, the fix then counts as just arrived
        const ageField = parseInt(f[2], 10), age = ageField >= 0 && ageField <= 2000 ? ageField : Number.isFinite(ageField) ? 0 : NaN;
        const p = +f[3] * 100.0, q = +f[5];
        let oat = String(f[6]).trim() === "" ? NaN : Number(f[6]);
        if (scan.temperatureKelvin && plausibleTemperature(oat - 273.15)) { oat -= 273.15; temperatureKelvinSamples++; }
        const acc = [+f[7], +f[8], +f[9]];
        const gyro = [+f[10], +f[11], +f[12]].map((x) => x * Math.PI / 180);
        if (![stamp, age, p, q, ...acc, ...gyro].every(Number.isFinite)) continue;
        const t = monotone(stamp);
        tMs = t;
        const ias = q > 0.0 ? Math.sqrt(2.0 * q / RHO0) : 0.0;
        const alt = p > 0 ? T0DIVL * (1.0 - Math.pow(p / P0, 1.0 / G0DIVRXL)) : NaN;
        if (!plausibleTemperature(oat)) {
          oat = estimatedTemperature(Number.isFinite(gnssAlt) ? gnssAlt : alt);
          temperatureEstimatedSamples++;
        }
        const tas = p > 0 ? ias * Math.sqrt(RHO0 * R_AIR * (oat + 273.15) / p) : 0.0;
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
            const warm = ["1.00000", "0.000000", "0.7500", "1.00000"].concat(Array(7).fill("1.000000"), "2"); // cold, flap offsets unknown, format 2
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
          // the lever as logged (FLP), else the recommendation
          const flap = opts.noFlaps ? NaN : Number.isFinite(logged.flap) ? (loggedFlapFrames++, logged.flap) : flaps ? Math.round(flaps(ias, load)) : NaN;
          sink = polar.sink(ias, Math.max(-acc[2], 0.3));
          setupCount++;
          if (setupCount % 100 === 0) rec.push(setupLine); // like the device, every 10 s
          rec.push(sentence("PXCVR", [String(t)].concat(acc.map((x) => fx(x, 5)), gyro.map((x) => fx(x, 6)), [fx(ias, 3), fx(tas, 3), fx(alt, 2), fx(flap, 3)])));
          frames++;
          // the fix arrived age ms before this cycle; the estimator gets it in this cycle
          if (pendingFix && pendingFix.speed > 1.0 && Number.isFinite(pendingFix.track)) {
            const trk = pendingFix.track * Math.PI / 180;
            rec.push(sentence("PXCVG", [String(t - age), fx(pendingFix.speed * Math.cos(trk), 3), fx(pendingFix.speed * Math.sin(trk), 3),
              "0.500", "750", "0", "0.000", "0.000", String(Math.round(pendingFix.utc * 1000) % 86400000)]));
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
    return { kind: "XCVario sensor log", rec, track: track.done(), starts, frames, fixes, glider: gliderName, gliderFrom, mass: massUsed, massFrom,
      qnh: scan.qnh, qnhFrom: scan.qnhFrom, temperatureEstimatedSamples, temperatureKelvinSamples, axesCorrected: !!scan.axesCorrected,
      flapsFrom: opts.noFlaps ? "" : loggedFlapFrames ? "logged" : flaps ? "the recommendation" : "" };
  }

  const api = { convert, GLIDERS, gliderFromLog };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.AirMassConvert = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
