# AirMass Replay in the browser

`index.html` replays a flight log through the device's AirMass code, compiled to WebAssembly, and plays it back:
the glider on a map with the winds as arrows (track coloured by the climb), the vario as the cockpit shows it in Air Mass
mode against TE netto and a GNSS reference, the attitude, and the vario tone. Nothing is uploaded; the replay runs in
the page.

    tools/airmass_sim/web/build.sh                          # airmass.js, in the emscripten/emsdk Docker image
    python3 -m http.server 8765 -d tools/airmass_sim/web    # then open http://127.0.0.1:8765
    tools/airmass_sim/web/deploy.sh                         # publish to Cloudflare (airmass-replay.<subdomain>.workers.dev)

It reads an XCVario sensor log (`$SENS`, converted like `sens2rec.py`) or an AirMass recording (System > Logging →
All Sensor Data), plain or `.gz`. Settings: the glider polar for sensor logs, the mass, bugs, the vario damping and averager, estimator
settings as in `airmass_replay --set`, and the start calibration.

Opening a new log automatically selects its glider from `CA1` or older `CMN` metadata, and shows the type in the
glider selector, replacing a manual choice from a previous log; you can override the type for the current log.
Configuration can occur later in the file. A log with no recognized type keeps the glider picked by hand before
(also saved across visits); without one the selector shows "Select glider" and the replay uses the ASG 29/18 fallback.

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

The target polls AirMass every 100 ms (10 Hz), with its sensors. Each tick predicts the state; barometer, kinematic and energy
corrections consume only new source timestamps. IMU and pressure/pitot freshness are tracked separately, so a
barometer can update while other readings are held. Held IMU data carry increasing prediction uncertainty;
stale/invalid data cannot maintain vario validity. Settling remains five seconds and delayed GNSS support 6.4
seconds, independent of poll rate. Kinematic/energy model-constraint noise is normalized by correction interval.
Output synchronization stays at 5 Hz, wind at 2 Hz, average wind at 1 Hz, and setup/compass/save periods use
elapsed time. This changes the estimator scheduler only; it does not increase physical sensor sampling rates.

New `PXCVR` frames append acceleration, gyro, barometer and airspeed source timestamps, then a valid-source
bitmask (1/2/4/8). Native and web replay preserve these fields, including prediction-only ticks. The web replay
also inserts 100 ms prediction ticks between frames of slower recordings and converted `$SENS` logs. Inserted
ticks hold the previous readings and their original source timestamps; corrections still run only for new
measurements, including with Mahony enabled. Already recorded 20 Hz ticks are preserved. A new flight waits
for its first measured frame, and gaps longer than ten seconds use the existing dynamics reset instead of
fabricated history. The native recording checker keeps the original recorded cadence for exact comparisons.
Displayed replay duration is computed from timestamps. Map/chart sensor data keep their logged cadence;
estimator output includes the additional prediction ticks. Rebuild `airmass.js` to use the updated core.
`sensor_freshness_20hz` covers independent updates, held values, outages, scheduling, history duration, rollover,
device/replay equivalence and upsampling equivalence. The WASM integration test covers 5 Hz logs at 10 Hz and recorded 20 Hz polls.
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

The angle of attack scale (`aoa_scale` in the summary, `kcl_min`/`kcl_max`) stays at 1 unless the settings give it a
range: learned, it is not observable beside the per-flap pitch offsets and ran to its limit.

Implausible readings are held out like a missing sample (`Plausibility` in `AirMassPipeline.h`): an airspeed above
150 m/s, a pressure altitude outside −1000..15000 m, more than 8 g or 8 rad/s, or a step of more than 15 m/s, 25 m,
4 g or 5 rad/s between consecutive samples (the limit scales with the sample interval up to three times; real logs stay
below half of it). A reading that steps and stays is taken over at the third agreeing sample. The held-out counts per
channel are in the native simulator's summary (`held out`). The vertical constraints also reject innovations beyond
20 m/s and the kinetic disagreement is clamped at 20 m/s², so a garbled `$SENS` line (the May 2025 logs) can no longer
drive the vario to hundreds of m/s while marked valid.

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

**Main vario**, the selector in the header, picks AirMass, TE or TE pressure as the main vario: it has the big
needle (in its own colour), the tone, and its average on the gauge's centre plate instead of a momentary value; its
average leaves the readouts below the gauge. Where the log has no TE pressure, TE pressure falls back to TE. The
choice is saved in this browser.

**Averager**, beside Damping in **Settings → Vario**, controls the trailing averaging window for the main vario's
average on the gauge and the other varios' numeric readouts. It defaults to 30 seconds and accepts 7–50 seconds, matching the device's range.
The averages are over the damped varios, as the device averages what its needle shows.
Changes update the readouts and their period label immediately and are saved in this browser.
The vario damping runs the device's 10 Hz cycles on the frame timestamps, each cycle reading the latest frame, so
5 Hz logs and 20 Hz recordings are damped like 10 Hz ones; between cycles the needle holds.
Display, damping and sound settings apply during playback using the cached flight output. Glider, loading,
flap, calibration and estimator settings recalculate automatically while retaining the current position,
chart view, playback speed and running/paused state. Instrument transmission pauses during recalculation;
the same log keeps its serial timeline and resumes at the selected speed after a successful update.

**TE pressure** (green) is the device's TE vario replayed from the TE pressure in `$SENS` logs: the ISA altitude of
the pressure through `VarioFilter::postProcess` with a TEK probe (its altitude filter, the rate per 10 Hz cycle and the
vario damping set here), netto minus the polar sink like TE netto. The TE line is what the device logged in `$PXCV`
(damped with its own setting, 0.1 m/s steps, 5 Hz). On real flights the two agree to about 0.3 m/s RMS at 4 s damping,
so a difference shows a TE probe or tube problem in the log, or electronic compensation, which this replay does
not model. It has its own needle, average, chart line, tooltip value, main-vario option and CSV columns
(`te_pressure_vario`, `te_pressure_netto`); AirMass recordings carry no TE pressure, so it is hidden for them.

**Gross mode for all varios** switches the vario needles, chart lines, numeric averages and the main vario's tone
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

To replay into an XCVario, open a sensor log, then open **Settings → Instrument replay → Connect serial…**.
Choose the baud rate used by the instrument's NMEA input (115200 by default) and select its port in the browser
chooser. This requires [Web Serial in desktop Chrome](https://developer.chrome.com/docs/capabilities/serial)
over HTTPS or localhost. Permission is requested only when you click Connect. The connection status and
Disconnect button are in Settings. `$PXCVR` recordings cannot supply the full hardware sensor payload;
serial playback currently requires `$SENS` logs.

Connection enters simulation mode with `$PFLAX,S,0`, waits one second, restores the logged glider/loading
configuration and flap position at the selected time, and stays paused. Play starts at 1×. The normal Play/Pause,
speed selector and all seek controls drive the instrument and display from one clock. Untimed navigation and
configuration sentences are interpolated in file order between the repaired `$SENS` timestamps. TE pressure and
optional magnetometer data are retained; sensor conventions and temperature corrections use the same normalization
as offline import. Offline estimator prediction ticks are never transmitted as additional sensor readings.
Forward sensor gaps longer than five seconds keep their duration when elapsed RMC UTC agrees within two seconds
(including UTC midnight). Navigation/setup messages remain spread across the gap. Browser and command-line
conversion use the same rule; clock synchronization jumps and resets retain the existing timestamp repair.
Navigation messages before the first or after the last imported sensor sample are excluded from serial playback
and counted in the connection hint. Their minutes of GPS/traffic data cannot be replayed with sensor state and
must not be collapsed onto the first/last sensor timestamp. Logged setup at the start is still restored.
Malformed sensor lines filtered by the shared importer are counted in the connection hint. Serial replay also
skips samples with missing/nonfinite pressure values or nonpositive static/TE pressure, and reports their count
in the hint. Valid samples keep their logged pressures and timing; no replacement pressures are invented.
Rejected samples accepted by the offline importer still anchor timestamp repairs and intervening navigation/setup
sentences. A log with no usable sensor samples remains unavailable for serial replay.
Every sentence is sent with a new checksum, so navigation and setup lines a logger damaged (a failed logged
checksum, or another sentence starting inside one without a checksum) are not sent; the hint counts them.
The device drops a sentence longer than its 128 character frame (`ProtocolItf::MAX_LEN`), and a logged `$SENS`
with magnetometer data plus a checksum often is. `$SENS` values go without trailing zeros, values the
normalization changed get the decimals the device logs, and the time stamp stays as logged; a sample still too long
gets the magnetometer with 1 decimal. Two samples merged into one line are not sent; the hint counts them.
Legacy CMN becomes CA1 with the importer's QNH estimate (101325 Pa if unavailable), and modern weight updates
become complete CA1 messages. When no glider/loading configuration was logged, Settings explains that the
instrument's current setup will be used. Browser polar, mass, flap and estimator overrides affect the offline
display; the serial replay sends logged setup and flap changes.

Serial speeds are restricted to 1×, 2×, 5× and 10×, further limited by the log's peak bytes in a rolling second
and 70% of the selected 8N1 port capacity. The 10× ceiling is conservative, not a claim of hardware validation;
reduce speed if the instrument cannot keep up. Offline replay retains its full speed selection. Connection is
blocked if the sensor replay interval cannot fit at 1×. Frozen timestamps are recovered only when the logged
GNSS-age counter is stale (over two seconds) and continues advancing by at most two seconds per sample. Its
measured increments supply the elapsed time; browser and Python converters report the recovered sample count.
Without that evidence, coincident samples are retained and can still exceed every baud rate.
Writes are serialized and await stream backpressure. More than 250 ms of scheduling/write lag pauses playback without
dropping samples or starting a catch-up burst; a two-second write/backpressure timeout closes the connection.
Serial replay goes on while the tab is hidden; its timers run in a worker, which the browser does not slow down. A write already handed to the OS can
finish after Pause/Seek, but pending stale sentences are canceled. Loading another log or rerunning the estimator
pauses transmission; replacing the timeline restores setup and resets serial speed to 1×.

Seeks notify the instrument of a disruptive jump and restore the applicable configuration and flap position.
The firmware also resets vario/AHRS settling on pause, resume and speed commands. The instrument must settle
again; it cannot receive the offline estimator's accumulated history. Disconnect freezes the simulation clock;
restart the instrument to leave simulation mode. Device replies are not used as acknowledgments, so connection
status describes the browser transport, not proof that the instrument accepted the replay.

Run `node --test tools/airmass_sim/web/serial_test.js` for deterministic fake-clock/fake-writer tests of protocol
ordering, timing, initialization, pause/resume, seeks, speed limits, lag, backpressure, failures and reconnects.
These tests do not replace validation with a connected instrument.

Files: `airmass_wasm.cpp` (the module, on `replay_core.h` like `airmass_replay`), `convert.js` (log → recording;
keep in step with `sens2rec.py`), `serial.js` (serial clock/transport), `replay.js` (runs the module), `make_gliders.py` (the glider list from the firmware's tables), `deploy.sh` (publishes only the page files), `check.js` (node: the converter against
`sens2rec.py`, the replay against `airmass_replay --csv`).
