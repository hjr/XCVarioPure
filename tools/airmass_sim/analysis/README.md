# AirMass comparisons on real flights and in the simulator

Results: [RESULTS-2026-11-06.md](RESULTS-2026-11-06.md), raw script outputs in `raw/`.

## Data

The 11 real flights (XCVario `$SENS` sensor logs, ASG 29E, Southern French Alps, from XCVario discussion #451) are in
`flightlogs/discussion451/` at the repo root (git-ignored, 84 MB). Flying mass 430 kg (`mass: 430`).

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
| `learned.js` | the calibration AirMass learns per flight (`SETTINGS`) |
| `phase.py` | the IMU acceleration vs airspeed change disagreement and AirMass vs AHRS pitch around the circle (native replays: `airmass_replay --csv` and `sens2rec.py --aux` outputs) |
| `sim_agree.py` | simulator CSVs (`airmass_sim nl ... --csv`): AirMass, TE netto, GNSS/INS against the truth and a GNSS-style reference |
| `build_old.sh` | builds the estimator before real-flight tuning (a833f536) as `build_sim/old/tools/airmass_sim/web/airmass_old.js` |

```sh
W=tools/airmass_sim/web; L=flightlogs/discussion451/*.nmea.gz
tools/airmass_sim/analysis/build_old.sh
node tools/airmass_sim/analysis/summary.js $W build_sim/old/tools/airmass_sim/web/airmass_old.js $L
SERIES='[["blend on","blend_bank=0"],["blend off","gust_acc=1000"]]' node tools/airmass_sim/analysis/exp.js $W $L
MASS=430 node tools/airmass_sim/analysis/cond.js $W $L
```

The two GNSS references: GNSS netto as on the page (energy height from GNSS altitude and **pitot TAS**, minus the
polar sink: a horizontal gust counts as air movement, as for TE), and the gust-insensitive one (GNSS altitude and the
**GNSS ground velocity minus a 2 min wind**). The pitot one favours TE netto; trust the gust-insensitive one more.

Simulator runs for `sim_agree.py` (CSV names `<day>_g<variant>.csv`; the development day 6 seeds and the 6 held-out
days of `scoreboard.py --holdout`), e.g. in the ESP-IDF image:

```sh
for s in 1 2 3 4 5 6; do build_sim/airmass_sim nl --seed $s --set gyro_gate=0.5 --csv out/dev_s${s}_gnow.csv; done
python3 tools/airmass_sim/analysis/sim_agree.py out now
```
