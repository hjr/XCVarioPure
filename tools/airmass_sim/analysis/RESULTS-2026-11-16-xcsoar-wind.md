# AirMass wind against XCSoar's zigzag wind, 2026-11-16

`xcsoar_wind.js` runs XCSoar's wind computer, ported from XCSoar master (`WindEKF`, `WindEKFGlue`, the circling detection
of `CirclingComputer`). It is fed what XCVario gives XCSoar: each GNSS fix at its arrival, the TAS of that moment, and the
load factor of the bank angle. Both winds are scored against the wind from GNSS alone (`gnssCircleWind`: one circle fit per
full turn, using neither heading nor airspeed). 27 logs: the 25 clean ones and the two Arcus flights, whose broken TE vario
does not matter for wind. 1 Hz GNSS throughout, 2016 turns. Raw output: `raw/xcsoar-wind-2026-11-16.txt`.

What XCSoar does with an airspeed: a fixed-gain update (gain 0.04 decaying to 0.01 per fix, so a ~100 s time constant at
1 Hz) of the wind and an airspeed scale, from |ground velocity − wind| = scale · TAS. It has no heading. Fixes are skipped
above 20°/s turn rate or 0.3 g off 1 g, plus 3 s after. Its displayed wind is updated every 10th fix, and not at all while
circling: the counter restarts on every fix in circling mode. The circling wind only goes to the analysis store once the
EKF has produced a wind.

## Results (rms vector error, m/s)

| | AirMass nav (30 s, what XCSoar receives) | AirMass instant | AirMass, 100 s low pass | XCSoar | held thermal wind |
|---|---:|---:|---:|---:|---:|
| circling, all turns | 1.65 | **1.55** | 1.86 | 2.66 | – |
| circling, 1st turn | 1.88 | **1.69** | 2.13 | 2.84 | – |
| straight, 0.5–2 min after the thermal | 1.56 | 2.29 | **1.53** | 1.97 | (0.98) |
| straight, 2–5 min | 2.36 | 2.83 | 2.04 | **1.87** | (1.62) |
| straight, 5–10 min | 2.81 | 3.06 | 2.53 | **2.04** | 2.27 |
| straight, 10–20 min | 3.10 | 3.24 | 2.97 | **2.48** | 3.03 |
| straight, last minute before the next thermal | 2.50 | 2.67 | 2.43 | **2.36** | 2.78 |
| straight, AirMass wind valid | 2.21 | 2.69 | 2.10 | 2.08 | 1.86 |
| straight, AirMass wind degraded | 2.64 | 2.97 | 2.35 | **2.01** | 2.06 |

Per flight: AirMass nav beats XCSoar while circling on 24 of 24 flights, but in straight flight (0.5–10 min after the
thermal) only on 2 of 24 (5 of 24 with the 100 s low pass).

The reference is noisy: consecutive turns in the same thermal differ by 1.86 m/s rms, so one turn's fit is off by about
1.3 m/s. The fits of two thermals 0–5 min apart differ by 2.7 m/s. That noise is the same for every series, so the ranking
holds even though the absolute values are inflated. In straight flight the reference is the per-turn winds interpolated
between the thermal before and the one after. That favours a series that holds the thermal's wind: "held thermal wind"
starts from the reference's own value (in brackets where that dominates). The last minute before the next thermal is the
fairest straight-flight bin: there the reference is close to the wind actually met. Both estimators beat holding the old
wind there (2.36 and 2.50 vs 2.78), so both track real changes, XCSoar a little better.

## Why AirMass loses in straight flight

- **Smoothing, about half the gap.** AirMass's 30 s nav wind moves 0.32 m/s within 10 s in straight flight, XCSoar's 0.19.
  The same instant wind with a 100 s low pass moves 0.16 and scores 2.13 against XCSoar's 1.96 (nav 2.38). While its wind
  is valid, AirMass at 100 s equals XCSoar (2.10 vs 2.08).
- **Crosswind drift while degraded.** AirMass's across-track error grows with the time since the thermal (1.10, 1.68,
  2.09, 2.52) faster than XCSoar's (1.40, 1.33, 1.47, 1.96). In straight flight heading and crosswind cannot be told
  apart, and AirMass's wind is degraded for 62 % of the straight samples. XCSoar has no heading to drift: it barely moves
  the crosswind in straight flight, it holds it.
- **Along-track bias at cruise speed.** AirMass shows +0.3 m/s more tailwind than the reference in straight flight (+0.1
  in the first 2 min, +0.4 later). XCSoar shows −0.05. XCSoar's learned scale (0.93–0.99 on the ASG flights, true TAS =
  measured / scale) puts the true airspeed 2–7 % above the measured one. AirMass's `k_tas` (1.00–1.03) is learned mostly
  in turns, at circling speed, where it matches the circle radius (`circle_ref.js`, median 1.000). The pitot/static
  error at cruise speed seems to differ from the one while circling.

Tuning does not close the gap (straight 0.5–10 min, AirMass nav): `q_wind_h` 0.04 → 0.01 gives 2.38 → 2.25 (circling
1.65 → 1.72); `q_psi` 3e-5 → 3e-4 gives 2.37; both together 2.27. Not adopted; the vario was not checked.

## Crosswind hold

The idea: hold the crosswind, and let only the headwind follow GNSS. First tested as post-processing of the replay output
(since removed from `xcsoar_wind.js`), so the vario is untouched. While held, only the component of the wind along the heading follows the instant
wind: W += dt/τ · ((W_instant − W)·ĥ) ĥ. Along the heading the instant wind does not depend on a heading error to first
order, across it does. When the heading changes in straight flight, other components get updated, as in XCSoar's
zigzag. Held either while the wind is degraded, or in straight flight (from 10 s below 10° bank until 25°).

| straight flight, rms m/s | total | along | across | mean along | flights beating XCSoar |
|---|---:|---:|---:|---:|---:|
| AirMass nav (30 s, before) | 2.38 | 1.63 | 1.73 | +0.31 | 2 of 24 |
| hold while degraded, τ 30 s | 2.24 | 1.62 | 1.55 | +0.28 | 2 of 24 |
| hold while degraded, τ 100 s | 2.14 | 1.50 | 1.52 | +0.23 | – |
| hold in straight flight, τ 30 s | 2.24 | 1.64 | 1.52 | +0.31 | 3 of 24 |
| hold in straight flight, τ 100 s | 2.03 | 1.44 | 1.43 | +0.22 | 9 of 24 |
| XCSoar | 1.96 | 1.37 | 1.40 | −0.04 | |

The hold does what it should: the across error falls from 1.73 to 1.52 at the same 30 s, and to 1.43 with 100 s, equal to
XCSoar's 1.40. Holding only while degraded is too late: the crosswind has already drifted by then.

## In the firmware

`WindAverage` (`AirMassWindAverage.h`): in straight flight the along-heading update with `ALONG_TAU` 100 s; turning (the
low-passed turn rate above the 4°/s circling threshold, outside the circling state) the plain 30 s low pass as before,
since turning makes the crosswind observable; circling the heading sectors, unchanged. Device and replays share the
header. The wind sent to the navigation computer, the wind rose and the status page "avg" change; the instant wind and
the vario do not.

Three variants were scored before choosing (straight flight 0.5–10 min after the thermal, rms m/s; flights beating
XCSoar; simulator scenario "straight", averaged wind against the true wind, before 0.20):

| | real flights | beating XCSoar | sim "straight" | sim "straight" +300 ms latency |
|---|---:|---:|---:|---:|
| 30 s low pass (before) | 2.38 | 2 of 24 | 0.20 | 0.25 |
| hold outside the circling state | 1.90 | 13 of 24 | 0.48 | 0.74 |
| the same, the crosswind following while the heading uncertainty shrinks | 1.94 | – | 0.47 | 0.73 |
| **hold in straight flight, 30 s low pass turning (chosen)** | **1.95** | **12 of 24** | **0.43** | **0.65** |
| XCSoar | 1.96 | | | |

Chosen on principle (hold only where the crosswind cannot be observed), equal on real flights, a little better in the
simulator. Real flights with it: by time since the thermal 1.42, 1.84, 2.33, 2.76 (XCSoar 1.97, 1.87, 2.04, 2.48); last
minute before the next thermal 2.35 (XCSoar 2.36); while AirMass's wind is valid 1.99 (XCSoar 2.08), degraded 2.12
(2.01); circling 1.64 (before 1.65, XCSoar 2.66); moves 0.14 m/s within 10 s in straight flight (before 0.32, XCSoar 0.19).

The cost is in the simulator's straight scenario: its glider circles briefly just after the wind gets valid, and the
average holds the crosswind it had then (0.5 m/s off) while the instant wind settles to 0.2. The simulated gyro drifts far
less than the real one, so there holding costs more than it saves; on real flights the opposite. All 38 host tests pass
(thresholds unchanged). Other scenarios' averaged wind rms (before → after): thermal 0.18 → 0.07, pullup 0.28 → 0.03,
slip 0.28 → 0.17, gust 1.06 → 1.04, endurance 0.09 → 0.09; NL day off the mean wind 0.85 / 0.80 → 0.80 / 0.77 (straight
/ circling), against the 30 s averaged truth 0.45 → 0.52 (a 100 s filter against a 30 s truth).

The vario and the instant wind are bit-identical before and after (replays of two flights). `batch.js` on the 27 logs moves
by the reference only, whose gust-insensitive energy subtracts a 2 min mean of the averaged wind: equal damping 10 s
AirMass 0.534 → 0.538 (TE netto 0.829 → 0.831), 30 s 0.242 → 0.247 (0.388 → 0.390), cockpit 10 s 0.631 → 0.635; false
climbs 2 m/s 2 s 11.3 /h unchanged. (TE netto's pool includes the two Arcus flights with the broken TE vario.)

### Faster first circle: not adopted

A sector's first visit in a climb weighted fully instead of `VISIT_WEIGHT` 0.5 (the wind from before the thermal then
drops out after one circle) speeds up the first circle but costs the glide after it. Straight flight starts from the
climb's mean and holds its crosswind, so a noisier mean stays for the glide. Real-flight rms (m/s):

| first visit weight | 1st circle | all circles | straight 0.5–10 min after |
|---|---:|---:|---:|
| 0.5 (kept) | 1.86 | 1.64 | 1.95 |
| 0.75 | 1.81 | 1.62 | 1.98 |
| 1 | 1.80 | 1.61 | 2.01 |
| XCSoar | 2.84 | 2.66 | 1.96 |

## Caveats

- The load factor is 1/cos(bank) from AirMass's attitude, not XCVario's accelerometer, so turbulence does not trigger
  XCSoar's g blackout here. Real XCSoar would skip a few more fixes in straight flight.
- XCSoar's circling wind and its wind store are not modelled. With airspeed, XCSoar displays the EKF wind only.
- 11 of the logs are mountain flights (French Alps, ASG 29) with strongly varying wind. The other gliders' logs (DG 800,
  Arcus, Ventus) show the same ranking per flight.

## Leads

- An airspeed scale that depends on speed (or `k_tas` learned separately at cruise speed), for the along-track bias.
