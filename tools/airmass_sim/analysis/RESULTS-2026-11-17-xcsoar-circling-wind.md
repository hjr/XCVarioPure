# AirMass wind against XCSoar's circling wind, 2026-11-17

`xcsoar_wind.js` now also runs XCSoar's circling wind, ported from XCSoar master (`CirclingWind`, `WindStore`,
`WindMeasurementList`). It is fed what XCVario gives XCSoar: GNSS track and ground speed per fix, the TAS, no gyroscope.
A wind comes out after each full circle that is round enough (turn rate deviation, cosine fit of ground speed − TAS
against track, 0.25 s GNSS latency correction), then every quarter circle. The store averages all measurements weighted
by quality, age (1 h) and altitude (1000 m). This is what XCSoar displays with auto wind set to "Circling". With
"Circling & ZigZag" and an airspeed it displays the EKF wind instead (series "XCSoar", see
[the 2026-11-16 results](RESULTS-2026-11-16-xcsoar-wind.md)). Same 27 logs, 2029 turns. Raw output:
`raw/xcsoar-circling-wind-2026-11-17.txt`.

Series: "XCSoar circling" is the store's average (the displayed wind), "XCSoar circle" the latest single measurement.

## Results (rms vector error, m/s)

The circling wind uses the same GNSS fixes as the per-turn reference fit, so against the same turn it is favoured. The
fairer circling score is against the next turn's fit in the same thermal.

| | AirMass nav | AirMass instant | XCSoar EKF | XCSoar circling (store) | XCSoar last circle |
|---|---:|---:|---:|---:|---:|
| circling, against the next turn, all | **1.69** | 1.64 | 2.56 | 2.14 | 2.12 |
| circling, against the next turn, after the 1st turn | **1.63** | 1.61 | 2.51 | 1.89 | 1.76 |
| circling, against the next turn, 1st turn | **1.83** | 1.72 | 2.69 | 2.64 | 2.78 |
| circling, against the same turn (favours XCSoar circling) | 1.64 | 1.55 | 2.67 | 2.20 | 2.10 |
| straight, 0.5–2 min after the thermal | **1.42** | 2.29 | 1.96 | 1.70 | 1.69 |
| straight, 2–5 min | **1.84** | 2.83 | 1.87 | 1.94 | 2.08 |
| straight, 5–10 min | 2.33 | 3.06 | **2.04** | 2.21 | 2.48 |
| straight, 10–20 min | 2.76 | 3.24 | **2.48** | 3.05 | 3.20 |
| straight, last minute before the next thermal | 2.35 | 2.67 | 2.36 | 2.67 | 2.82 |
| straight, 0.5–10 min | **1.95** | 2.80 | 1.96 | 1.99 | 2.15 |

Per flight, AirMass nav beats the XCSoar circling wind while circling on 24 of 24 flights (next-turn reference), and in
straight flight 0.5–10 min after the thermal on 18 of 24.

- **Circling.** AirMass is 0.45 m/s better on all turns, 0.26 after the first turn. On the first turn the circling wind
  has at most one fresh measurement, mixed with the old ones in the store, and is barely better than the EKF. Each
  single circle measurement is noisy (the GNSS-only circle fits of consecutive turns differ by 1.86 m/s).
- **Straight flight.** The circling wind holds the last thermals' wind, like "held thermal wind" of the earlier
  results, but averaged over the last hour, so it is worse than the plain held wind early (1.70 vs 0.98, where the
  reference favours the held wind) and drifts behind later (10–20 min 3.05 vs AirMass 2.76). It has no along-track bias
  (−0.07 mean, AirMass +0.19).
- It hardly moves: 0.06 m/s within 10 s in straight flight (AirMass 0.14, EKF 0.19).

## Against XCSoar's default "Circling & ZigZag"

Series "XCSoar both": the store's wind until the EKF's first wind, then the EKF's only (`WindComputer::Compute`: once
the EKF is active the store is no longer read for the display). The EKF has a wind 10 fixes after takeoff (median 0.1
min; AirMass's averaged wind 1.9 min, the circling store's 5.1 min), so the circling wind is never shown with an airspeed
and "both" equals the zigzag wind. The EKF starts from zero wind, which only the scored windows after the tow hide.

| rms m/s | samples | AirMass avg | XCSoar circling + zigzag | difference |
|---|---:|---:|---:|---:|
| circling, against the next turn, all | 1290 | **1.69** | 2.56 | −0.87 |
| circling, 1st turn | 400 | **1.83** | 2.69 | −0.85 |
| circling, 2nd turn on | 890 | **1.63** | 2.51 | −0.88 |
| straight, 0.5–2 min after the thermal | 44808 | **1.42** | 1.96 | −0.54 |
| straight, 2–5 min | 76633 | 1.84 | 1.87 | −0.03 |
| straight, 5–10 min | 67657 | 2.33 | **2.04** | +0.28 |
| straight, 10–20 min | 29656 | 2.76 | **2.48** | +0.28 |
| straight, last minute before the next thermal | 24185 | 2.35 | 2.36 | 0.00 |
| straight, 0.5–10 min | 189098 | 1.95 | 1.96 | −0.01 |

Flights won: circling 24 of 24, straight 0.5–10 min 12 of 24. Straight 0.5–10 min along / across rms 1.38 / 1.37 vs
1.37 / 1.40, mean along +0.19 vs −0.03; moves within 10 s 0.14 vs 0.19.

## The tailwind bias

AirMass's straight-flight wind has more tailwind along the track than the reference: averaged +0.19 m/s, instant +0.48;
XCSoar −0.04. It is AirMass's own and not the reference's:

- Reference-free (`opposite_legs.js`, 97 pairs of opposite straight legs within 6 min): instant +0.66 ± 0.14 m/s
  (median +0.38), averaged +0.32 ± 0.11.
- It does not depend on the wind: along error = a + b · (reference tailwind) gives a = +0.48 for the instant wind, b
  about equal for every series (−0.02 to −0.08, the reference noise). Thermals drifting slower than the wind would
  show in b, not a.
- It hardly depends on speed: instant +0.67 at 80–90 km/h IAS, +0.37 to +0.59 up to 150, +0.42 at 150–180. So it is
  not a cruise-speed airspeed scale: it is there at circling speed too. In straight flight the true airspeed is about
  0.5 m/s above what `k_tas` learned while circling makes of it, at all speeds. XCSoar's single scale, learned in
  straight flight, fits it at medium speed and overcorrects fast (−0.30 at 150–180 km/h, −0.66 above).
- The air vector model cannot make it: the flight path angle costs < 0.1 %.

What differs between circling and straight flight at the same IAS is the angle of attack and sideslip, which move the
static and pitot errors. Removing the bias would take a second calibration learned in straight flight from heading
changes (an airspeed offset, or a scale depending on the lift coefficient), as XCSoar learns its scale.

What it costs: removing the mean gives straight 0.5–10 min 1.946 → 1.937 (instant 2.80 → 2.76). It is a small part of
the rms, but it does not average out over a glide.

A learned straight-flight airspeed offset was tried on 2026-11-18 and dropped: pinned at 0.5 m/s it removes the bias,
but the filter learns only +0.13 on real flights. Parked until more real flights; the results are in issue #4 on
XCVarioPurePrivate.

## Caveats

- No gyroscope: XCVario's XCSoar driver passes attitude and load factor only, so the turn rate comes from GNSS track.
- Altitude for the store's weighting is the GNSS altitude; XCSoar uses the navigation altitude (baro when available).
  Only the 100 m recalculation trigger and the 1000 m altitude weighting see it.
- The reference noise is the one of the earlier results (about 1.3 m/s per turn); only the ranking is meaningful.
