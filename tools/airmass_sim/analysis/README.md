# AirMass comparisons on real flights and in the simulator

Results: [RESULTS-2026-11-06.md](RESULTS-2026-11-06.md), raw script outputs in `raw/`.

Latest: [2026-11-10, all 34 logs of ~/Downloads/NMEAlogs](RESULTS-2026-11-10.md): new defaults `sigma_pitch` 0.03 and
`kinetic_tau` 5 s (AirMass now beats TE netto over 30 s too), plausibility guards against garbled readings, the covariance
fixes behind two blow-ups, and the bias decomposition by bank (`decomp.js`). `table.js` tabulates `batch.js` outputs.

Latest full batch: [2026-11-09 reevaluation after conversion fixes](RESULTS-2026-11-09-REEVALUATION.md),
15 logs, including equal-damping scores, cockpit display timing, wind cross-checks and robustness failures.

Downloads/NMEAlogs batch: [2026-11-08 results](RESULTS-2026-11-08-NMEAlogs.md), including sensor-gap availability and corrupted-input failures. `batch.js OUTPUT.json LOG...` accepts plain or gzipped logs and scores current AirMass versus TE on a regular time grid.

New September 15 log: [2026-11-09 results](RESULTS-2026-11-09-NMEAlogs.md), with one RMC solution per epoch and continued sensor-gap recovery problems.

## Data

The 11 real flights (XCVario `$SENS` sensor logs, ASG 29E, Southern French Alps, from XCVario discussion #451) are in
`flightlogs/discussion451/` at the repo root (git-ignored, 84 MB). The scripts replay them with the glider and weight the
log names, as the page does (`logOptions` in `page.js`): these logs have no CA1 telegram, so the ASG 29/18 at the logged
ballast factor, 441 kg (450 kg on two flights). `GLIDER=name` and `MASS=kg` set them.

## Scripts

All real-flight scripts run the browser replay's own code (`tools/airmass_sim/web`: the converter, the WebAssembly
estimator, the page's `build`/`derive`), so they stay in step with the site. Build `airmass.js` first
(`tools/airmass_sim/web/build.sh`).

| Script | What it does |
|---|---|
| `summary.js` | every series (AirMass now, after the 1st real-flight tuning, before any, TE netto; the GNSS/INS reference of the results is no longer in the page) against both GNSS references, 10 s / 30 s / 5 min windows, delays, needle movement |
| `exp.js` | AirMass variants (`SERIES='[["name","settings"], ...]'`) and TE netto against both GNSS references |
| `cond.js` | errors binned by bank, IAS, roll rate, TAS change, time since start; least squares fit (`MASS`, `SETTINGS`) |
| `lag.js` | delay to the reference and needle movement |
| `needle.js` | (`REALTIME=1`: no delay shift, as the pilot sees it; `PHASE_LAG=1`: each series shifted by its delay in straight flight and circling, fairer for circling) the cockpit needle per variant (`SERIES='[["name","settings",damping], ...]'`, damping `[straight, circling]` for a bank-dependent one): movement within 1 s and 10 s error, straight / circling, and the delay while circling |
| `wind.js` | the instant wind: deviation from its 60 s mean and movement, straight / circling, the part turning with the glider (heading frame, left / right circles) and locked to the heading |
| `sim_wind.py` | the same in simulator CSVs, against the true wind at the glider: how much of its movement the estimate follows |
| `circle_ref.js` | the gust-insensitive reference with a wind from GNSS alone (a circle fit per full turn, `CIRCLE='{"tas":true}'`: fitted to the measured airspeed) against the one with AirMass's wind; AirMass's wind against the GNSS-only one; each series' own error by the three-cornered hat |
| `shear.js` | per thermal: the wind change rate (GNSS-only per-turn winds) and the once-per-circle part of the vario error against the GNSS-only reference, for AirMass and TE netto; whether the error follows the wind change or the wind |
| `gusts.js` | horizontal gusts to look at in the web replay: straight flight, steady pitch and vertical air, an IAS jump that TE netto shows as lift or sink; flight, UTC, TE netto and AirMass deviation (`MIN_DIAS`, `MIN_TE`, `TOP`) |
| `learned.js` | the calibration AirMass learns per flight (`SETTINGS`) |
| `phase.py` | the IMU acceleration vs airspeed change disagreement and AirMass vs AHRS pitch around the circle (native replays: `airmass_replay --csv` and `sens2rec.py --aux` outputs) |
| `sim_agree.py` | simulator CSVs (`airmass_sim nl ... --csv`): AirMass, TE netto, GNSS/INS against the truth and a GNSS-style reference |
| `table.js` | per-flight table and pooled 10 s / 30 s / cockpit scores from `batch.js` outputs (`EXCLUDE=file,...` leaves logs out of the pools) |
| `decomp.js` | per-sample means of AirMass, its energy, plain-energy and pitch paths, TE netto and both references, binned by bank (left / right) and by time since circling; `--print` merges outputs |
| `te_pressure.js` | AirMass netto, the logged TE netto and the TE netto replayed from the logged TE pressure (the page's `TeVario`) at equal 4 s damping against both references, per flight and pooled (`EXCLUDE`), with the three-cornered hat once per TE (the two share the probe), delays and 1 s needle movement; `node te_pressure.js OUT.json` alone re-reports a saved run |
| `build_old.sh` | builds the estimator before real-flight tuning (a833f536, with the replay tools of 323f5a9c) as `build_sim/old/tools/airmass_sim/web/airmass_old.js` |

```sh
W=tools/airmass_sim/web; L=flightlogs/discussion451/*.nmea.gz
tools/airmass_sim/analysis/build_old.sh
node tools/airmass_sim/analysis/summary.js $W build_sim/old/tools/airmass_sim/web/airmass_old.js $L
SERIES='[["blend on","blend_bank=0"],["blend off","gust_acc=1000"]]' node tools/airmass_sim/analysis/exp.js $W $L
MASS=430 node tools/airmass_sim/analysis/cond.js $W $L   # at a set weight instead of the logged one
```

Errors against a reference include the reference's own: over 10 s it is the larger part (`circle_ref.js`, three-cornered
hat), so compare series against the same reference, or use the hat's own errors.

The two GNSS references: GNSS netto as on the page (energy height from GNSS altitude and **pitot TAS**, minus the
polar sink: a horizontal gust counts as air movement, as for TE), and the gust-insensitive one (GNSS altitude and the
**GNSS ground velocity minus a 2 min wind**). The pitot one favours TE netto; trust the gust-insensitive one more.

Simulator runs for `sim_agree.py` (CSV names `<day>_g<variant>.csv`; the development day 6 seeds and the 6 held-out
days of `scoreboard.py --holdout`), e.g. in the ESP-IDF image:

```sh
for s in 1 2 3 4 5 6; do build_sim/airmass_sim nl --seed $s --set gyro_gate=0.5 --csv out/dev_s${s}_gnow.csv; done
python3 tools/airmass_sim/analysis/sim_agree.py out now
```
