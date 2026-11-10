// Run a recording (convert.js) through the WebAssembly replay (airmass.js), in chunks so the page stays responsive.
(function (root) {
  "use strict";
  // opts: settings ("name=value, ..."), learn (default true), warm: [k_tas, pitch_ofs_deg, aoa_scale, latency_s] or null.
  // mahony: use the device's extended Mahony AHRS for the estimator attitude (default false).
  // Resolves to { t_ms: Uint32Array, cols: {name: Float32Array}, summary } or rejects with the replay's error.
  async function runReplay(M, rec, opts = {}, onProgress = null) {
    const withString = (s, fn) => {
      const n = M.lengthBytesUTF8(s) + 1;
      const p = M._malloc(n);
      M.stringToUTF8(s, p, n);
      try { return fn(p); } finally { M._free(p); }
    };
    const w = opts.warm;
    const ok = withString(opts.settings || "", (p) => M._am_begin(p, opts.learn === false ? 0 : 1, w ? 1 : 0, w ? w[0] : 1, w ? w[1] : 0, w ? w[2] : 1, w ? w[3] : 0.75, opts.mahony ? 1 : 0));
    if (!ok) throw new Error(M.UTF8ToString(M._am_error()));
    // Older cached modules ignore the new argument: do not silently replay with the wrong attitude filter.
    if (opts.mahony && !JSON.parse(M.UTF8ToString(M._am_summary())).mahony) {
      throw new Error("Mahony AHRS needs the updated replay module. Rebuild airmass.js and reload the page.");
    }
    const CHUNK = 20000;
    for (let i = 0; i < rec.length; i += CHUNK) {
      if (!withString(rec.slice(i, i + CHUNK).join("\n"), (p) => M._am_feed(p))) throw new Error(M.UTF8ToString(M._am_error()));
      if (onProgress) await onProgress(Math.min(1, (i + CHUNK) / rec.length));
    }
    M._am_finish();
    const n = M._am_nrows(), nc = M._am_ncols();
    const names = M.UTF8ToString(M._am_columns()).split(",");
    const rows = new Float32Array(M.HEAPF32.buffer, M._am_rows(), n * nc);
    const t_ms = new Uint32Array(M.HEAPU32.buffer, M._am_times(), n).slice();
    const cols = {};
    names.forEach((name, c) => {
      const a = new Float32Array(n);
      for (let i = 0; i < n; i++) a[i] = rows[i * nc + c];
      cols[name] = a;
    });
    return { n, t_ms, cols, summary: JSON.parse(M.UTF8ToString(M._am_summary())) };
  }
  // the bits of the replay's status column: AirMassEKF::ST_* (main/airmass/AirMassEKF.h)
  const STATUS = Object.freeze({ VARIO_VALID: 1, WIND_VALID: 2, GNSS_FRESH: 4, ALIGNED: 8, WIND_DEGRADED: 16 });
  if (typeof module !== "undefined" && module.exports) module.exports = { runReplay, STATUS };
  else Object.assign(root, { runReplay, STATUS });
})(typeof globalThis !== "undefined" ? globalThis : this);
