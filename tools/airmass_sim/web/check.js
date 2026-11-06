// Check the browser replay against the host tools on one flight:
//   node tools/airmass_sim/web/check.js log.nmea.gz [sens2rec.rec] [airmass_replay.out.csv] [mass kg]
// the recording against sens2rec.py's, the replayed vario and wind against airmass_replay --csv.
const fs = require("fs");
const zlib = require("zlib");
const path = require("path");
const { convert } = require("./convert.js");
const { runReplay } = require("./replay.js");

(async () => {
  const [log, pyRec, nativeCsv, mass] = process.argv.slice(2);
  let buf = fs.readFileSync(log);
  if (log.endsWith(".gz")) buf = zlib.gunzipSync(buf);
  let t0 = Date.now();
  const c = convert(buf.toString("latin1"), { glider: "ASG 29/18", mass: +(mass || 0) });
  console.log(`${c.kind}: ${c.starts} start(s), ${c.frames} cycles, ${c.fixes} fixes, ${c.track.n} track samples (${Date.now() - t0} ms)`);
  if (pyRec) {
    const py = fs.readFileSync(pyRec, "latin1").split(/\r?\n/).filter((l) => l);
    let diff = 0, first = -1;
    for (let i = 0; i < Math.max(py.length, c.rec.length); i++) {
      if (py[i] !== c.rec[i]) { diff++; if (first < 0) first = i; }
    }
    console.log(`recording: ${c.rec.length} lines, sens2rec.py ${py.length}, ${diff} differ` + (first >= 0 ? `, first ${first}:\n  js ${c.rec[first]}\n  py ${py[first]}` : ""));
  }
  const M = await require(path.join(__dirname, "airmass.js"))();
  t0 = Date.now();
  const r = await runReplay(M, c.rec, {});
  console.log(`replay: ${r.n} cycles in ${Date.now() - t0} ms`, r.summary);
  if (nativeCsv) {
    const lines = fs.readFileSync(nativeCsv, "utf8").trim().split("\n");
    const hdr = lines[0].split(",");
    const iv = hdr.indexOf("vario"), iwn = hdr.indexOf("wind_n"), iwe = hdr.indexOf("wind_e"), ist = hdr.indexOf("status");
    let n = 0, dv = 0, dw = 0, ds = 0, maxv = 0;
    for (let i = 1; i < lines.length && i - 1 < r.n; i++) {
      const f = lines[i].split(",");
      const k = i - 1;
      const ev = r.cols.vario[k] - +f[iv];
      dv += ev * ev;
      maxv = Math.max(maxv, Math.abs(ev));
      dw += (r.cols.wind_n[k] - +f[iwn]) ** 2 + (r.cols.wind_e[k] - +f[iwe]) ** 2;
      ds += r.cols.status[k] !== +f[ist];
      n++;
    }
    console.log(`against airmass_replay (${lines.length - 1} rows): vario rms ${Math.sqrt(dv / n).toFixed(4)} max ${maxv.toFixed(3)} m/s, ` +
      `wind rms ${Math.sqrt(dw / n).toFixed(4)} m/s, status differs ${ds} times`);
  }
})().catch((e) => { console.error(e); process.exit(1); });
