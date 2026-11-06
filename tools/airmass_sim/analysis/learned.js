// the calibration AirMass learns on each flight (cold start), at 430 kg
const fs = require("fs"), zlib = require("zlib"), path = require("path");
const web = path.resolve(process.argv[2]);
const { convert } = require(path.join(web, "convert.js"));
const { runReplay } = require(path.join(web, "replay.js"));
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  for (const log of process.argv.slice(3)) {
    const conv = convert(zlib.gunzipSync(fs.readFileSync(log)).toString("latin1"), { glider: "ASG 29/18", mass: 430 });
    const r = await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" });
    const s = r.summary;
    console.log(`${path.basename(log).padEnd(26)} k_tas ${s.k_tas.toFixed(3)}  pitch ofs ${s.pitch_ofs_deg.toFixed(2)}°  AoA scale ${s.aoa_scale.toFixed(3)}  latency ${s.latency_ms} ms  gyro z bias ${s.gyro_z_bias_dps.toFixed(3)}°/s`);
  }
})();
