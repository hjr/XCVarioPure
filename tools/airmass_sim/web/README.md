# AirMass Replay in the browser

`index.html` replays a flight log through the device's AirMass code, compiled to WebAssembly, and plays it back:
the glider on a map with the winds as arrows (track coloured by the climb), the vario as the cockpit shows it in Air Mass
mode against TE netto and a GNSS reference, the attitude, and the vario tone. Nothing is uploaded; the replay runs in
the page.

    tools/airmass_sim/web/build.sh                          # airmass.js, in the emscripten/emsdk Docker image
    python3 -m http.server 8765 -d tools/airmass_sim/web    # then open http://127.0.0.1:8765
    tools/airmass_sim/web/deploy.sh                         # publish to Cloudflare (airmass-replay.<subdomain>.workers.dev)

It reads an XCVario sensor log (`$SENS`, converted like `sens2rec.py`) or an AirMass recording ("NMEA Output" →
Recording), plain or `.gz`. Settings: the glider polar for sensor logs, the mass, bugs, the vario damping, estimator
settings as in `airmass_replay --set`, and the start calibration.

Older sensor logs: `!XCV,FMT,SENS,start,factors...` multiplies the `$SENS` fields from index `start` on (`SENS` is 0; logs
before 2025 are east-north-up), `!XCV,CMN,type,empty,crew,ballast,speedcal` stands in for `CA1`, and `;` separators and
times in integer milliseconds are accepted. The QNH comes from `CA1`, else `$PXCV`, else the GNSS altitude and static
pressure on the ground; it only feeds the QNH altitude shown, AirMass uses the ISA pressure altitude.

Files: `airmass_wasm.cpp` (the module, on `replay_core.h` like `airmass_replay`), `convert.js` (log → recording;
keep in step with `sens2rec.py`), `replay.js` (runs the module), `make_gliders.py` (the glider list from the firmware's tables), `deploy.sh` (publishes only the page files), `check.js` (node: the converter against
`sens2rec.py`, the replay against `airmass_replay --csv`).
