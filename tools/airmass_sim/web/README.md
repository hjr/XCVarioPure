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

Opening a new log automatically selects its glider from `CA1` or older `CMN` metadata, and shows the type in the
glider selector. A saved manual choice from a previous log is replaced; you can override the type for the current
log. Configuration can occur later in the file. Logs with no recognized type use the ASG 29/18 fallback.

Logs can begin airborne. Sensor logs start after 100 consecutive samples above 20 m/s IAS (10 seconds at 10 Hz).
GNSS is optional: logs without GNSS can replay the instruments and AirMass vario, while wind and the GNSS vario
reference remain unavailable. A low ground speed in a headwind also does not prevent startup.
AirMass recordings normally use their `PXCVS` initialization. When it is missing,
replay waits for a recorded `PXCVC` setup and usable flying-speed sensor data, then starts with default calibration
(or the selected warm-start override). It uses a recent logged `$PXCV` roll/pitch when available; otherwise it
estimates gravity after accounting for the coordinated-turn acceleration from gyro rates and TAS. Implausible
motion references are skipped until usable data arrives. The summary identifies recovered starts and estimated
attitudes. Early output needs settling and wind still needs heading alignment; recovery cannot reproduce the
missing estimator history exactly. An explicit `PXCVX` stop requires a new start record.

The device sends a fresh setup and `PXCVS` snapshot if Recording is enabled while AirMass is already running.
The snapshot carries its current attitude, calibration and flap offsets without resetting the live estimator.
A logger joining an existing recording stream can still miss the snapshot and use the recovery path above.

The target polls AirMass every 50 ms (20 Hz). Each tick predicts the state; barometer, kinematic and energy
corrections consume only new source timestamps. IMU and pressure/pitot freshness are tracked separately, so a
barometer can update while other readings are held. Held IMU data carry increasing prediction uncertainty;
stale/invalid data cannot maintain vario validity. Settling remains five seconds and delayed GNSS support 6.4
seconds, independent of poll rate. Kinematic/energy model-constraint noise is normalized by correction interval.
Output synchronization stays at 5 Hz, wind at 2 Hz, average wind at 1 Hz, and setup/compass/save periods use
elapsed time. This changes the estimator scheduler only; it does not increase physical sensor sampling rates.

New `PXCVR` frames append acceleration, gyro, barometer and airspeed source timestamps, then a valid-source
bitmask (1/2/4/8). Native and web replay preserve these fields, including prediction-only ticks. The web replay
also inserts 50 ms prediction ticks between frames of older recordings and converted `$SENS` logs. Inserted
ticks hold the previous readings and their original source timestamps; corrections still run only for new
measurements, including with Mahony enabled. Already recorded 20 Hz ticks are preserved. A new flight waits
for its first measured frame, and gaps longer than ten seconds use the existing dynamics reset instead of
fabricated history. The native recording checker keeps the original recorded cadence for exact comparisons.
Displayed replay duration is computed from timestamps. Map/chart sensor data keep their logged cadence;
estimator output includes the additional prediction ticks. Rebuild `airmass.js` to use the updated core.
`sensor_freshness_20hz` covers independent updates, held values, outages, scheduling, history duration, rollover,
device/replay equivalence and upsampling equivalence. The WASM integration test covers 5/10 Hz logs at 20 Hz.
The native simulator accepts `--set predict_hz=20` to insert held-input prediction ticks between the same 10 Hz
physical measurements (default: 10). CTest also runs straight, thermal, gust and cross-country scenarios and an
exact recording/replay check at this mixed rate.

Sensor-log temperatures outside −60…+50°C, missing or non-finite values use an estimate: **25°C at sea level,
decreasing by 6.5°C per kilometre**. Altitude comes from GNSS, falling back to ISA pressure altitude. This estimate
feeds TAS and the replay; plausible logged temperatures remain unchanged. The replay summary reports how many
samples used the estimate. `sens2rec.py` uses the same rule. AirMass recordings already contain TAS and pass through.

GNSS timing uses each RMC fix's source UTC as well as its arrival time. New recordings retain source UTC as an
optional ninth `PXCVG` field; older recordings remain readable. Repeated/backward source epochs and invalid
velocities are rejected. Source/arrival intervals distinguish irregular fix cadence from extra transport delay,
including queued bursts and UTC midnight. The constant receiver latency is still learned: source UTC alone cannot
establish the absolute delay of the first fix. Without source epochs, arrival ordering and innovation checks still
apply, with additional allowance for timing uncertainty.

Delayed GNSS fusion requires measured sensor history bracketing its observation time; the history holds about
6.4 seconds at 10 Hz. It never clamps an old fix to a history endpoint or interpolates through an IMU gap. A rejected
fix does not refresh accepted-GNSS age, and the entire horizontal observation is gated before any state changes.

Sensor gaps over 0.3 seconds are propagated in small steps holding the preceding measurement, with increased
uncertainty and inferred history excluded from GNSS fusion. Gaps over 10 seconds restart the dynamics while keeping
calibration. After a gap, vertical validity settles for about 5 seconds (TE fallback), and the inertial gust
correction regains full trust over 10 seconds of continuous data. Pitch/AoA kinematic fusion and GNSS calibration
learning wait for adequate input quality; wind-work correction fades with accepted-fix age and GNSS quality.
Sparse logs can consequently spend much of the replay on TE fallback. Exported native/WASM columns
`gnss_age_ms`, `gnss_received_age_ms`, `gnss_rejects`, and `inertial_trust` expose this behavior; GNSS age is the age
of the last accepted observation, received age is the time since the last fusion attempt, and `UINT32_MAX` means none.

`sensor_gnss_robustness` in the host CTest suite covers delayed/irregular fixes, transport jitter, out-of-order and
duplicate epochs, outages, outliers, midnight/clock wrap, unsupported history, whole-fix rejection and sensor gaps.

**Gross mode for all varios** switches the three needles, chart lines, 30 s averages and selected sound source
together. AirMass gross includes the polar sink and falls back to TE gross; TE gross is the logged TE vario;
GNSS gross is the total-energy climb before subtracting the polar sink. The GNSS damping option applies to both
modes. The switch takes effect immediately and is saved in the browser; the map's named colour sources keep their
own meanings.

Below the flying-speed cutoff (80% of the polar's stall-speed reference, with a 10 m/s minimum), AirMass invalidates
its vario and stops using the wing air-vector model. The dashed blue needle follows TE through landing and ground
roll; it returns to AirMass after five seconds of supported flying-speed samples. This policy runs in the shared
device/replay pipeline, including a separate 10 m/s TAS floor for direct EKF inputs.

**Use Mahony AHRS** switches the estimator and attitude display to the device's extended Mahony filter, including
its coordinated-turn reference. It is off by default; the usual AirMass air-data-aided attitude remains the default.
Changing the checkbox replays the loaded flight and saves the choice in this browser. Each flight starts from the
recorded attitude. The native replay has the same option as `airmass_replay --mahony`. Rebuild `airmass.js` after
updating the sources so the browser can run the new filter.

Older sensor logs: `!XCV,FMT,SENS,start,factors...` multiplies the `$SENS` fields from index `start` on (`SENS` is 0; logs
before 2025 are east-north-up), `!XCV,CMN,type,empty,crew,ballast,speedcal` stands in for `CA1`, and `;` separators and
times in integer milliseconds are accepted. The QNH comes from `CA1`, else `$PXCV`, else the GNSS altitude and static
pressure on the ground; it only feeds the QNH altitude shown, AirMass uses the ISA pressure altitude.

The converters validate the legacy `FMT,SENS,7,1,-1,-1,1,-1,-1` declaration against the logged `$PXCV` attitude.
They repair it to gravity-vector acceleration (negated on all axes) and unchanged body gyros only when both
stationary tilted samples and pitch-rate comparisons provide sufficient evidence. Valid declarations and logs
without that evidence retain their declared mapping. Kelvin OAT is recognized by agreement with `$PXCV`'s Celsius
readout; other implausible/missing temperatures retain the altitude-based fallback. Applied corrections appear in
the replay summary. These conversions do not modify the flight log or the live estimator.

If `FMT` is absent, gravity-vector acceleration can also be identified from at least 50 quiet low-speed tilted
attitudes (at least 80% must agree) and at least 100 pitch-rate comparisons supporting unchanged body gyros.
This covers older RR logs such as `2026-03-12_10-42_RR.nmea`. Missing metadata is left alone when those checks fail;
the summary reports inferred axes separately from corrections to an existing declaration.

Run `python3 tools/airmass_sim/conversion_test.py` for the shared JavaScript/Python convention checks.
`replay_airborne_start` in CTest covers snapshots, banked recovery with both attitude filters, configuration order,
invalid inputs and stop markers. After rebuilding the web module, run
`node tools/airmass_sim/web/replay_start_test.js` for converter/WASM/page integration.

Files: `airmass_wasm.cpp` (the module, on `replay_core.h` like `airmass_replay`), `convert.js` (log → recording;
keep in step with `sens2rec.py`), `replay.js` (runs the module), `make_gliders.py` (the glider list from the firmware's tables), `deploy.sh` (publishes only the page files), `check.js` (node: the converter against
`sens2rec.py`, the replay against `airmass_replay --csv`).
