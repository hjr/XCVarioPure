// the calibration AirMass learns on each flight (cold start), at the logged weight (GLIDER=name, MASS=kg to set them)
const path = require("path");
const web = path.resolve(process.argv[2]);
const { loadPage } = require("./page.js");
const { runReplay, loadFlight } = loadPage(web);
(async () => {
  const M = await require(path.join(web, "airmass.js"))();
  for (const log of process.argv.slice(3)) {
    const conv = loadFlight(log);
    const r = await runReplay(M, conv.rec, { settings: process.env.SETTINGS || "" });
    const s = r.summary;
    console.log(`${path.basename(log).padEnd(26)} ${conv.glider} ${Math.round(conv.mass)} kg (${conv.massFrom})  k_tas ${s.k_tas.toFixed(3)}  pitch ofs ${s.pitch_ofs_deg.toFixed(2)}°  AoA scale ${s.aoa_scale.toFixed(3)}  latency ${s.latency_ms} ms  gyro z bias ${s.gyro_z_bias_dps.toFixed(3)}°/s`);
  }
})();
