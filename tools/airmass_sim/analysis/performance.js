// The standard performance run of the vario methods (AirMass, eComp, TE pressure, IMU TEK, TEK0) on the real flights
// and the simulator: te_pressure.js on every log in parallel jobs, the simulator's tests and scoreboard in the ESP-IDF
// Docker image, flights left out by fixed rules, and a RESULTS file in a fixed layout, with the change against a
// baseline run on the flights both share.
//   node performance.js [--comp tuned|logged|N] [--logs DIR] [--jobs N] [--baseline RUN.json] [--tag NAME] [--sim off] [--reuse RUN.json]
// --comp: the TE compensation setting (te_pressure.js COMP): tuned (the default) each installation's (TUNED_SETTING in
// page.js, 92 for one not in it, listed in the report), N for all, logged the logs' own (eComp and the IMU TEK at 92,
// AirMass at the device's setup, 100 for $SENS logs without one). --logs: a directory of logs (~/Downloads/NMEAlogs),
// its .nmea and .nmea.gz files, a plain log left out where its .gz twin is there. --baseline: an earlier run's JSON
// (raw/perf-*.json) for the change. --sim off: no simulator run. --reuse: report a run's JSON again without replaying.
// Writes raw/perf-DATE[-TAG]-COMP.json and RESULTS-DATE-performance[-TAG]-COMP.md (an earlier one of the same name is
// kept: -2, -3, ...), prints the verdict and the short table.
const fs = require("fs"), path = require("path"), os = require("os"), { spawn, execSync } = require("child_process");
const { SERIES, NAMES, RANKED, metrics, markdown, top, rms, f3 } = require("./te_pressure.js");
const { installation } = require("./page.js");
const ROOT = path.resolve(__dirname, "../../..");

// logs that never count: serial-garbled $SENS (RESULTS-2026-11-10), no GNSS
const KNOWN_BAD = {
  "2025-05-13_08-19.nmea": "garbled serial data",
  "2025-05-18_08-17.nmea": "garbled serial data",
  "2026-03-12_10-42_RR.nmea": "no GNSS",
};
// rules on a flight's 10 s rms (gust-insensitive reference) and scored hours
const MIN_HOURS = 0.2, BROKEN_REF = 1.5, FAULTY_PROBE = 1.3;
// the needle lead the AirMass needle must have over TE (the user's requirement of 2026-11-06), s
const LEAD_REQUIRED = 1;
// a GNSS latency that moves more within a flight, or between an installation's flights, is flagged, ms
const LATENCY_SPREAD = 300;

function verdict(r) {
  if (KNOWN_BAD[r.file]) return { pool: "none", reason: KNOWN_BAD[r.file] };
  if (r.skipped) return { pool: "none", reason: r.skipped };
  if (!(r.hours >= MIN_HOURS)) return { pool: "none", reason: `${r.hours.toFixed(2)} h scored (sensor gaps)` };
  const a = r.refs.inertial.sums[10]?.all, v = Object.fromEntries(SERIES.map((s) => [s, rms(a, s)]));
  if (!(v.AirMass < BROKEN_REF) || SERIES.some((s) => !(v[s] < 3)))
    return { pool: "none", reason: `broken reference or data (AirMass ${f3(v.AirMass)}, worst ${f3(Math.max(...SERIES.map((s) => v[s])))} m/s at 10 s)` };
  if (v.TEp > FAULTY_PROBE * v.eComp) return { pool: "all", reason: `faulty TE probe (TE pressure ${f3(v.TEp)} vs eComp ${f3(v.eComp)} m/s at 10 s)` };
  return { pool: "main" };
}

function args() {
  const o = { comp: "tuned", logs: path.join(os.homedir(), "Downloads/NMEAlogs"), jobs: Math.max(1, Math.min(4, os.cpus().length - 2)), tag: "", sim: "on" };
  const a = process.argv.slice(2);
  for (let i = 0; i < a.length; i += 2) {
    const k = a[i].replace(/^--/, "");
    if (!(k in o) && k !== "baseline" && k !== "reuse") throw new Error(`unknown option ${a[i]}`);
    o[k] = k === "jobs" ? +a[i + 1] : a[i + 1];
  }
  return o;
}

function logList(dir) {
  const names = fs.readdirSync(dir).filter((f) => /\.nmea(\.gz)?$/.test(f) && !/\(\d+\)/.test(f));
  return names.filter((f) => !(f.endsWith(".nmea") && names.includes(f + ".gz"))).map((f) => path.join(dir, f)).sort();
}

// the replay module older than the code it is built from
function staleModule() {
  const module = path.resolve(__dirname, "../web/airmass.js");
  if (!fs.existsSync(module)) return "airmass.js is missing";
  const built = fs.statSync(module).mtimeMs;
  const sources = ["main/airmass", "components/util/math", "tools/airmass_sim/web/airmass_wasm.cpp", "tools/airmass_sim/replay_core.h", "tools/airmass_sim/replay_settings.h", "tools/airmass_sim/mahony_source.h"];
  for (const s of sources) {
    const p = path.join(ROOT, s), files = fs.statSync(p).isDirectory() ? fs.readdirSync(p).map((f) => path.join(p, f)) : [p];
    const newer = files.find((f) => fs.statSync(f).mtimeMs > built);
    if (newer) return `airmass.js is older than ${path.relative(ROOT, newer)}`;
  }
  return null;
}

// te_pressure.js on the logs in parallel jobs of about equal size; the flights
async function realFlights(logs, o) {
  const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "perf-")), weight = (f) => fs.statSync(f).size * (f.endsWith(".gz") ? 8 : 1);
  const parts = Array.from({ length: o.jobs }, () => ({ logs: [], load: 0 }));
  for (const f of logs.slice().sort((a, b) => weight(b) - weight(a))) {
    const p = parts.reduce((m, q) => (q.load < m.load ? q : m));
    p.logs.push(f); p.load += weight(f);
  }
  const env = { ...process.env, COMP: o.comp === "logged" ? "" : o.comp };
  let done = 0;
  await Promise.all(parts.filter((p) => p.logs.length).map((p, i) => new Promise((resolve, reject) => {
    const child = spawn(process.execPath, [path.join(__dirname, "te_pressure.js"), path.join(tmp, `${i}.json`), ...p.logs], { env, stdio: ["ignore", "ignore", "pipe"] });
    child.stderr.on("data", (d) => { for (const line of String(d).split("\n").filter(Boolean)) console.error(`[${++done}/${logs.length}] ${line}`); });
    child.on("exit", (code) => (code ? reject(new Error(`job ${i} failed (${code})`)) : resolve()));
  })));
  const flights = parts.flatMap((p, i) => (p.logs.length ? JSON.parse(fs.readFileSync(path.join(tmp, `${i}.json`), "utf8")).flights : []));
  fs.rmSync(tmp, { recursive: true });
  return flights;
}

// the simulator built and tested (ctest), its scoreboard on the development and the held-out days
function synthetic() {
  const script = ". $IDF_PATH/export.sh > /dev/null 2>&1; cmake -S tools/airmass_sim -B build_sim -DCMAKE_BUILD_TYPE=Release > /dev/null && " +
    "cmake --build build_sim -j > /dev/null && (ctest --test-dir build_sim -j8 | tail -3); echo @@DEV; python3 -I tools/airmass_sim/scoreboard.py --json; " +
    "echo @@HOLDOUT; python3 -I tools/airmass_sim/scoreboard.py --holdout --json";
  return new Promise((resolve) => {
    const child = spawn("docker", ["run", "--rm", "--entrypoint", "", "-v", `${ROOT}:/project`, "-w", "/project", "espressif/idf:release-v5.4", "bash", "-c", script]);
    let out = "", err = "";
    child.stdout.on("data", (d) => (out += d));
    child.stderr.on("data", (d) => (err += d));
    child.on("error", (e) => resolve({ error: e.message }));
    child.on("exit", (code) => {
      const t = out.match(/(\d+)% tests passed, (\d+) tests failed out of (\d+)/), dev = out.split("@@DEV")[1]?.split("@@HOLDOUT")[0], hold = out.split("@@HOLDOUT")[1];
      try {
        resolve({ ctest: t ? { failed: +t[2], total: +t[3], failures: [...out.matchAll(/^\s*\d+ - (\S+) \(Failed\)/gm)].map((m) => m[1]) } : null,
          dev: JSON.parse(dev), holdout: JSON.parse(hold) });
      } catch { resolve({ error: `simulator run failed (${code}): ${(err || out).trim().split("\n").slice(-3).join(" ")}` }); }
    });
  });
}

const median = (v) => { const a = v.filter(Number.isFinite).sort((x, y) => x - y); return a.length ? a[Math.floor((a.length - 1) / 2)] : NaN; };
const quantile = (v, q) => { const a = v.filter(Number.isFinite).sort((x, y) => x - y); return a.length ? a[Math.floor((a.length - 1) * q)] : NaN; };
const f1 = (v) => (Number.isFinite(v) ? v.toFixed(1) : "–"), f0 = (v) => (Number.isFinite(v) ? v.toFixed(0) : "–"), pc = (v) => (Number.isFinite(v) ? `${(100 * v).toFixed(0)} %` : "–");
const sign = (v, d) => { const x = +v.toFixed(d); return (x > 0 ? "+" : x < 0 ? "−" : "±") + Math.abs(x).toFixed(d); };
const decimals = (label) => (label.includes("flights best") ? 0 : label.includes("delay") || label.includes("per h") ? 1 : 3);

// the cockpit needle rows (te_pressure.js extras): its unshifted 10 s rms and its delays by phase, AirMass, eComp and
// TE pressure only
function cockpitRows(pool) {
  const shown = ["AirMass", "eComp", "TEp"], sums = { n: 0 };
  for (const r of pool) for (const k in r.cockpit?.sums ?? {}) sums[k] = (sums[k] || 0) + r.cockpit.sums[k];
  const values = (f) => Object.fromEntries(SERIES.map((s) => [s, shown.includes(s) ? f(s) : NaN]));
  const rows = [{ label: "rms 10 s all as shown, m/s", values: values((s) => rms(sums, s)), fmt: f3, lowBest: true }];
  for (const ph of ["all", "straight", "circling"])
    rows.push({ label: `needle delay as shown, ${ph}, s`, values: values((s) => median(pool.map((r) => r.cockpit?.delay[ph][s]))), fmt: f1, lowBest: true });
  return rows;
}
// the AirMass needle's lead over eComp and TE pressure by phase: the median over the flights of the difference
function leads(pool) {
  return Object.fromEntries(["all", "straight", "circling"].map((ph) => [ph, Object.fromEntries(["eComp", "TEp"].map((s) =>
    [s, median(pool.map((r) => r.cockpit ? r.cockpit.delay[ph][s] - r.cockpit.delay[ph].AirMass : NaN))]))]));
}
// all rows of a pool: te_pressure.js's and the cockpit's
function allRows(pool) {
  const m = metrics(pool);
  return { ...m, rows: [...m.rows, ...cockpitRows(pool)] };
}
const SHORT = ["rms 10 s all, m/s", "rms 10 s straight, m/s", "rms 10 s circling, m/s", "rms 10 s speed change, m/s", "rms 30 s all, m/s",
  "rms 30 s straight, m/s", "rms 30 s circling, m/s", "rms 10 s all as shown, m/s", "false climbs 2 m/s 2 s, per h straight",
  "needle delay as shown, all, s", "needle delay as shown, straight, s", "needle delay as shown, circling, s", "flights best at 10 s"];
const pick = (m, labels) => ({ ...m, rows: labels.map((l) => m.rows.find((r) => r.label === l)).filter(Boolean) });

// new against baseline per row and series: the value and its change, better or worse by the row's direction
function deltaTable(m, b) {
  const lines = ["| | " + SERIES.map((s) => NAMES[s]).join(" | ") + " |", "|---|" + SERIES.map(() => "---|").join("")];
  for (const row of m.rows) {
    const old = b.rows.find((x) => x.label === row.label);
    if (!old) continue;
    const d = decimals(row.label);
    lines.push(`| ${row.label} | ` + SERIES.map((s) => {
      const n = top(row.values[s]), p = top(old.values[s]), dv = n - p;
      if (!Number.isFinite(n) || !Number.isFinite(p)) return "–";
      const worse = row.lowBest ? dv > 0 : dv < 0, mark = Math.abs(dv) < 0.5 * 10 ** -d ? "" : worse ? " ▲" : " ▼";
      return `${row.fmt(row.values[s])} (${sign(dv, d)}${mark})`;
    }).join(" | ") + " |");
  }
  return lines.join("\n");
}

// AirMass's wind and availability over a pool (flights with a faulty TE probe included: AirMass does not use the probe)
function windStats(pool) {
  const sum = (f) => pool.reduce((s, r) => s + (f(r) || 0), 0), all = (k) => pool.flatMap((r) => r.wind?.[k] ?? []);
  const secs = sum((r) => r.availability?.seconds), gustHours = sum((r) => r.gusts?.straightHours);
  return {
    hours: secs / 3600, vario: sum((r) => r.availability?.vario) / secs, wind: sum((r) => r.availability?.wind) / secs,
    shown: sum((r) => (r.availability?.wind ?? 0) + (r.availability?.degraded ?? 0)) / secs,
    firstWind: median(pool.map((r) => r.availability?.firstWind ?? NaN)), never: pool.filter((r) => r.availability && r.availability.firstWind === null).length,
    starts: sum((r) => r.availability?.starts), gaps: sum((r) => r.availability?.sensorGaps),
    valid: all("valid"), recovered: all("recovered"), avg: all("avg"), gusts: sum((r) => r.gusts?.events) / gustHours,
  };
}
function windTable(w, b) {
  const rows = [
    ["flying, h", (x) => x.hours, f1],
    ["AirMass vario valid", (x) => x.vario, pc],
    ["wind valid", (x) => x.wind, pc],
    ["wind valid or degraded (shown)", (x) => x.shown, pc],
    ["time to the first valid wind, median, s", (x) => x.firstWind, f0],
    ["flights without a valid wind", (x) => x.never, f0],
    ["instant wind vs circle fit while valid, median / p90, m/s", (x) => [median(x.valid), quantile(x.valid, 0.9)], (v) => `${f3(v[0])} / ${f3(v[1])}`],
    ["averaged wind vs circle fit while valid, median, m/s", (x) => median(x.avg), f3],
    ["circle fits while valid", (x) => x.valid.length, f0],
    ["degraded wind vs the wind it becomes valid at, median / p90, m/s", (x) => [median(x.recovered), quantile(x.recovered, 0.9)], (v) => `${f3(v[0])} / ${f3(v[1])}`],
    ["degraded samples compared (every 10 s)", (x) => x.recovered.length, f0],
    ["estimator starts / sensor gaps over 0.3 s", (x) => [x.starts, x.gaps], (v) => `${v[0]} / ${v[1]}`],
    ["gust alerts per h straight", (x) => x.gusts, f1],
  ];
  return [`| | now${b ? " | baseline" : ""} |`, `|---|---|${b ? "---|" : ""}`, ...rows.map(([l, f, fmt]) => `| ${l} | ${fmt(f(w))}${b ? ` | ${fmt(f(b))}` : ""} |`)].join("\n");
}

// the learned calibration per installation; flagged where the GNSS latency moves too much
function calibrationTable(pool) {
  const by = {};
  for (const r of pool) if (r.calibration) (by[installation(r.file, r.glider)] ||= []).push(r);
  const range = (v, d) => { const a = v.filter(Number.isFinite); return a.length ? `${Math.min(...a).toFixed(d)}–${Math.max(...a).toFixed(d)}` : "–"; };
  const lines = ["| installation (flights) | k_tas | GNSS latency at the end, ms | latency moving within a flight, max, ms | pitch offset, ° | IMU roll, ° | |",
    "|---|---|---|---|---|---|---|"];
  for (const [name, rs] of Object.entries(by).sort()) {
    const c = rs.map((r) => r.calibration), ends = c.map((x) => x.latency_ms), within = c.map((x) => x.latency_max - x.latency_min);
    const across = Math.max(...ends) - Math.min(...ends), worst = Math.max(...within.filter(Number.isFinite));
    const flag = [across > LATENCY_SPREAD && `latency differs by ${across.toFixed(0)} ms between flights`, worst > LATENCY_SPREAD && `latency moves ${worst.toFixed(0)} ms in a flight`].filter(Boolean).join("; ");
    lines.push(`| ${name} (${rs.length}) | ${range(c.map((x) => x.k_tas), 3)} | ${range(ends, 0)} | ${f0(worst)} | ${range(c.map((x) => x.pitch_ofs_deg), 1)} | ${range(c.map((x) => x.imu_roll_deg), 1)} | ${flag ? "⚠ " + flag : ""} |`);
  }
  return lines.join("\n");
}

// the simulator scoreboard: ours, TE, reference, LOSS, and ours against the baseline
function simTable(rows, base) {
  const f = (x) => (x == null ? "–" : x.toFixed(3)), lines = ["| metric | AirMass | TE | reference | | AirMass change |", "|---|---|---|---|---|---|"];
  let losses = 0;
  for (const [name, o, te, rf] of rows) {
    const lose = [te, rf].some((x) => x != null && o > x + 1e-9);
    losses += lose;
    const b = base?.find((x) => x[0] === name), dv = b ? o - b[1] : NaN, d = Math.abs(dv) < 0.0005 ? "±0.000" : `${sign(dv, 3)} ${dv > 0 ? "▲" : "▼"}`;
    lines.push(`| ${name} | ${f(o)} | ${f(te)} | ${f(rf)} | ${lose ? "LOSS" : ""} | ${b ? d : "–"} |`);
  }
  return { table: lines.join("\n"), losses, total: rows.length };
}

const DEFINITIONS = `Rows:

- rms W s PHASE: the rms over windows of W seconds of the method's mean minus the reference's mean, each method shifted by
  its fitted delay (the error of the reading, not of its timing). Straight: bank under 10° on average, circling: over 25°,
  speed change: TAS²/2g changing by more than 15 m over the window (pull-ups, push-overs, speed changes).
- rms 10 s all as shown: the same without the shift, the needle as the pilot sees it (AirMass with its own damping).
- own error (hat): the three-cornered hat with AirMass and the reference: the method's own error with the reference's
  taken out. AirMass's is a range, one per hat with each other method; ≈0 is a negative variance.
- median delay: each method's delay to the reference at the same damping. Needle delay as shown: with the damping the
  cockpit shows (AirMass 0.6 × the vario damping, the TE methods the vario damping); the lead is TE's minus AirMass's.
- needle movement in 1 s: the rms of the needle's change over 1 s, how busy it looks.
- false climbs / sinks X m/s Y s: in straight flight (bank under 10° for 8 s), the needle as shown X m/s above (below) the
  reference's centred 2 s mean for Y seconds, counted again only after it came back within X/2. About as many sinks as
  climbs means noise, not a lift bias; the 1 m/s rows are mostly the reference's own noise.
- flights best at 10 s: the flights where the method has the lowest 10 s rms (TEK0 not ranked).`;

(async () => {
  const o = args(), date = new Date().toISOString().slice(0, 10), base = o.baseline ? JSON.parse(fs.readFileSync(o.baseline, "utf8")) : null;
  let name = `${date}${o.tag ? "-" + o.tag : ""}-${o.comp}`;
  if (!o.reuse) for (let i = 2, stem = name; fs.existsSync(path.join(__dirname, "raw", `perf-${name}.json`)); i++) name = `${stem}-${i}`;
  else name = path.basename(o.reuse, ".json").replace(/^perf-/, "");
  let result;
  if (o.reuse) result = JSON.parse(fs.readFileSync(o.reuse, "utf8"));
  else {
    const stale = staleModule();
    if (stale) throw new Error(`${stale}: run tools/airmass_sim/web/build.sh first`);
    const logs = logList(o.logs);
    console.error(`${logs.length} logs from ${o.logs}, ${o.jobs} jobs, compensation ${o.comp}${o.sim === "off" ? "" : ", simulator in Docker"}`);
    const [flights, sim] = await Promise.all([realFlights(logs, o), o.sim === "off" ? null : synthetic()]);
    const git = (c) => { try { return execSync(`git ${c}`, { cwd: __dirname }).toString().trim(); } catch { return ""; } };
    result = { created: new Date().toISOString(), comp: o.comp, commit: git("rev-parse --short HEAD"),
      dirty: git("status --porcelain -- ../../../main ../../../components ../web ../*.h ../*.cpp") !== "",
      module: fs.statSync(path.resolve(__dirname, "../web/airmass.js")).mtime.toISOString(), synthetic: sim, flights };
    fs.writeFileSync(path.join(__dirname, "raw", `perf-${name}.json`), JSON.stringify(result) + "\n");
  }

  const rows = result.flights.map((r) => ({ r, ...verdict(r) }));
  const main = rows.filter((x) => x.pool === "main").map((x) => x.r), all = rows.filter((x) => x.pool !== "none").map((x) => x.r);
  const m = allRows(main), short = pick(m, SHORT), lead = leads(main);
  const when = (r) => [r.commit && `commit ${r.commit}${r.dirty ? " with uncommitted changes" : ""}`, r.module && `replay module built ${r.module.slice(0, 16).replace("T", " ")}`,
    `compensation ${r.comp ?? "logged"}`].filter(Boolean).join(", ");
  // the baseline on the flights both runs have in the main pool
  const baseMainFiles = new Set(base ? base.flights.filter((r) => verdict(r).pool === "main").map((r) => r.file) : []);
  const common = main.filter((r) => baseMainFiles.has(r.file)).map((r) => r.file), commonSet = new Set(common);
  const mNow = base && allRows(main.filter((r) => commonSet.has(r.file))), mBase = base && allRows(base.flights.filter((r) => commonSet.has(r.file)));
  const baseFiles = new Set(base ? base.flights.map((r) => r.file) : []), newFlights = base ? rows.filter((x) => !baseFiles.has(x.r.file)) : [];
  const onlyBase = base ? [...baseMainFiles].filter((f) => !main.some((r) => r.file === f)) : [];

  // the verdict
  const value = (label, s) => top(m.rows.find((r) => r.label === label)?.values[s]);
  const won = short.rows.filter((r) => { const v = RANKED.map((s) => top(r.values[s])).filter(Number.isFinite); const b = r.lowBest ? Math.min(...v) : Math.max(...v); return top(r.values.AirMass) === b; });
  const lost = short.rows.filter((r) => !won.includes(r) && Number.isFinite(top(r.values.AirMass)));
  const better = (label, s) => `${(100 * (1 - value(label, "AirMass") / value(label, s))).toFixed(0)} %`;
  const leadText = ["all", "straight", "circling"].map((ph) => `${ph} ${f1(lead[ph].eComp)} / ${f1(lead[ph].TEp)} s`).join(", ");
  const leadFails = ["all", "straight", "circling"].filter((ph) => !(Math.min(lead[ph].eComp, lead[ph].TEp) >= LEAD_REQUIRED));
  // flights whose own lead over eComp is short of the requirement, by phase
  const short1 = ["all", "straight", "circling"].map((ph) => {
    const l = main.filter((r) => r.cockpit).map((r) => ({ f: r.file, v: r.cockpit.delay[ph].eComp - r.cockpit.delay[ph].AirMass }));
    const below = l.filter((x) => x.v < LEAD_REQUIRED - 1e-9);
    return `${ph} ${below.length} of ${l.length}` + (ph === "circling" && below.length ? ` (${below.sort((a, b) => a.v - b.v).slice(0, 4).map((x) => `${x.f.slice(0, 10)} ${f1(x.v)} s`).join(", ")}${below.length > 4 ? ", ..." : ""})` : "");
  }).join(", ");
  const v = [];
  v.push(`- AirMass is best in ${won.length} of ${won.length + lost.length} rows of the short table${lost.length ? `; not in ${lost.map((r) => r.label.replace(/, (m\/s|s)$/, "")).join(", ")}` : ""}.`);
  v.push(`- 10 s rms below eComp by ${better("rms 10 s all, m/s", "eComp")} and TE pressure by ${better("rms 10 s all, m/s", "TEp")} (all windows); in speed changes by ${better("rms 10 s speed change, m/s", "eComp")} and ${better("rms 10 s speed change, m/s", "TEp")}.`);
  v.push(`- Needle lead as shown over eComp / TE pressure, median over the flights: ${leadText}; the required ${LEAD_REQUIRED} s ${leadFails.length ? `is missed ${leadFails.join(", ")}` : "is met by the median in every phase"}. Flights below ${LEAD_REQUIRED} s against eComp: ${short1}.`);
  const sim = result.synthetic, simDev = sim?.dev && simTable(sim.dev, base?.synthetic?.dev), simHold = sim?.holdout && simTable(sim.holdout, base?.synthetic?.holdout);
  if (sim?.error) v.push(`- Simulator: ${sim.error}.`);
  else if (sim) v.push(`- Simulator: ${sim.ctest ? `${sim.ctest.total - sim.ctest.failed} of ${sim.ctest.total} tests pass${sim.ctest.failures.length ? ` (failing: ${sim.ctest.failures.join(", ")})` : ""}` : "tests not run"}; scoreboard losses ${simDev.losses} of ${simDev.total} (development), ${simHold.losses} of ${simHold.total} (held-out)` +
    (base?.synthetic?.dev ? ` against ${simTable(base.synthetic.dev).losses} and ${simTable(base.synthetic.holdout).losses} in the baseline` : "") + ".");
  if (base) {
    const d = mNow.rows.filter((r) => SHORT.includes(r.label)).map((r) => ({ r, b: mBase.rows.find((x) => x.label === r.label) }))
      .map(({ r, b }) => ({ label: r.label, dv: top(r.values.AirMass) - top(b.values.AirMass), lowBest: r.lowBest, dec: decimals(r.label) }))
      .filter((x) => Number.isFinite(x.dv) && Math.abs(x.dv) >= 0.5 * 10 ** -x.dec);
    const up = d.filter((x) => (x.lowBest ? x.dv < 0 : x.dv > 0)), down = d.filter((x) => !(x.lowBest ? x.dv < 0 : x.dv > 0));
    v.push(`- Against the baseline on the ${common.length} flights both share: AirMass better in ${up.length} and worse in ${down.length} rows of the short table` +
      (down.length ? ` (worse: ${down.map((x) => `${x.label.replace(/, (m\/s|s)$/, "")} ${sign(x.dv, x.dec)}`).join(", ")})` : "") + `${newFlights.length ? `; ${newFlights.length} new flights` : ""}.`);
  }

  const md = [`# Vario methods performance, ${date}${o.tag ? ", " + o.tag : ""}, compensation ${result.comp}`, "",
    `\`performance.js\` on ${result.flights.length} logs and the simulator: ${when(result)}. Real flights: each method replayed from the`,
    "`$SENS` data and scored against the gust-insensitive GNSS reference (GNSS altitude and ground velocity minus AirMass's",
    "2 min wind; the one to rank by). Damping: the scores at the same 4 s for every method (the page's `Damp`), the rows",
    "\"as shown\" with the cockpit's (AirMass 0.6 × 4 s, eComp and TE pressure 4 s). TEK0 is the IMU TEK without its IMU",
    "term, a control, not ranked. Rows are defined at the end.", "",
    "## Verdict", "", ...v, "",
    `## Main pool: ${m.flights} flights, ${m.hours.toFixed(1)} h (${m.straight.toFixed(1)} h straight)`, "", markdown(short), ""];
  if (base) {
    md.push("## Change against the baseline", "", `Baseline: ${path.basename(o.baseline)}, ${when(base)}. On the ${common.length} flights in both main pools` +
      `${onlyBase.length ? `; only in the baseline's: ${onlyBase.join(", ")}` : ""}. ▲ worse, ▼ better.`, "", deltaTable(pick(mNow, SHORT), pick(mBase, SHORT)), "");
  }
  if (sim && !sim.error) {
    md.push("## Simulator", "", `${sim.ctest ? `ctest: ${sim.ctest.total - sim.ctest.failed} of ${sim.ctest.total} pass${sim.ctest.failures.length ? `, failing ${sim.ctest.failures.join(", ")}` : ""}.` : ""} ` +
      "`scoreboard.py`: the synthetic flights over 6 noise seeds against TE netto and the GNSS/INS reference; LOSS where either does better.", "",
      `Development set, ${simDev.losses} losses of ${simDev.total}:`, "", simDev.table, "", `Held-out days, ${simHold.losses} losses of ${simHold.total}:`, "", simHold.table, "");
  }
  const w = windStats(all), wb = base && windStats(base.flights.filter((r) => verdict(r).pool !== "none" && r.wind));
  md.push(`## Wind and availability`, "", `AirMass on ${all.length} flights (those with a faulty TE probe included), while flying (IAS over 60 km/h). The wind while`,
    "valid against the GNSS circle fits of single turns (gnssCircleWind); degraded wind is straight flight, without circle",
    "fits, so it is compared with the wind it becomes valid at (every 10 s of a degraded stretch against the mean valid wind",
    "of the 30 s after).", "", windTable(w, wb?.hours ? wb : null), "");
  md.push("## Learned calibration", "", `Per installation, at the end of each flight; flagged where the GNSS latency moves more than ${LATENCY_SPREAD} ms in a flight`,
    "(the last two thirds) or differs that much between flights: one device should have one latency.", "", calibrationTable(all), "");
  if (newFlights.length) {
    md.push("## New flights since the baseline", "", "| flight | glider | h | " + SERIES.map((s) => NAMES[s]).join(" | ") + " | pool |", "|---|---|---|" + SERIES.map(() => "---|").join("") + "---|");
    for (const { r, pool, reason } of newFlights) {
      const a = r.refs?.inertial.sums[10]?.all, sc = r.refs?.inertial.sums[10]?.["speed change"];
      md.push(`| ${r.file} | ${r.glider ?? "–"} | ${r.hours ? r.hours.toFixed(1) : "–"} | ` + SERIES.map((s) => (a ? `${f3(rms(a, s))} / ${sc ? f3(rms(sc, s)) : "–"}` : "–")).join(" | ") + ` | ${pool}${reason ? ": " + reason : ""} |`);
    }
    md.push("", "10 s rms, all windows / speed change, m/s.", "");
  }
  const mAll = allRows(all);
  md.push(`## Full table, main pool`, "", markdown(m), "",
    `With the flights of a faulty TE probe (${mAll.flights} flights, ${mAll.hours.toFixed(1)} h):`, "",
    markdown(pick(mAll, ["rms 10 s all, m/s", "rms 10 s speed change, m/s", "rms 30 s all, m/s", "false climbs 2 m/s 2 s, per h straight", "flights best at 10 s"])), "", DEFINITIONS, "");
  md.push("## Per flight", "", "10 s rms, all windows / speed change, m/s; setting: the compensation used (– the log's own); AirMass vario valid and wind shown (valid or degraded) while flying.", "",
    "| flight | glider | h | setting | " + SERIES.map((s) => NAMES[s]).join(" | ") + " | vario valid | wind shown | pool |", "|---|---|---|---|" + SERIES.map(() => "---|").join("") + "---|---|---|");
  for (const { r, pool, reason } of rows.slice().sort((a, b) => a.r.file.localeCompare(b.r.file))) {
    const a = r.refs?.inertial.sums[10]?.all, sc = r.refs?.inertial.sums[10]?.["speed change"], av = r.availability;
    const cell = (s) => (a ? `${f3(rms(a, s))} / ${sc ? f3(rms(sc, s)) : "–"}` : "–");
    md.push(`| ${r.file} | ${r.glider ?? "–"} | ${r.hours ? r.hours.toFixed(1) : "–"} | ${r.setting ?? "–"} | ` + SERIES.map(cell).join(" | ") +
      ` | ${av ? pc(av.vario / av.seconds) : "–"} | ${av ? pc((av.wind + av.degraded) / av.seconds) : "–"} | ${pool}${reason ? ": " + reason : ""} |`);
  }
  md.push("", `Pools: main without the flights marked; "all" ones (a faulty TE probe: TE pressure over ${FAULTY_PROBE}× eComp) only in the`,
    `tables that say so and in wind and calibration; "none": known bad logs, under ${MIN_HOURS} h scored, or a broken reference (AirMass`,
    `over ${BROKEN_REF} m/s or any method over 3 m/s at 10 s).`, "");
  const untuned = [...new Set(result.flights.map((r) => r.untuned).filter(Boolean))];
  if (untuned.length) md.push(`Not in TUNED_SETTING, run at the default 92: ${untuned.join(", ")}; sweep them with te_adjust.js and add them.`, "");

  const file = path.join(__dirname, `RESULTS-${name.replace(/^(\d{4}-\d\d-\d\d)/, "$1-performance")}.md`);
  fs.writeFileSync(file, md.join("\n"));
  console.log(`${v.join("\n")}\n\n${markdown(short)}\n`);
  console.log(`left out: ${rows.filter((x) => x.pool !== "main").map((x) => `${x.r.file} (${x.reason})`).join("; ")}`);
  if (untuned.length) console.log(`not in TUNED_SETTING (run at 92): ${untuned.join(", ")}`);
  console.log(`\nwrote ${path.relative(process.cwd(), file)}${o.reuse ? "" : ` and raw/perf-${name}.json`}`);
})().catch((e) => { console.error(e.stack || e); process.exitCode = 1; });
