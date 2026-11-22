# Vario methods performance, 2026-11-22, compensation tuned

`performance.js` on 39 logs and the simulator: commit 813da8b3 with uncommitted changes, replay module built 2026-11-22 19:29, compensation tuned. Real flights: each method replayed from the
`$SENS` data and scored against the gust-insensitive GNSS reference (GNSS altitude and ground velocity minus AirMass's
2 min wind; the one to rank by). Damping: the scores at the same 4 s for every method (the page's `Damp`), the rows
"as shown" with the cockpit's (AirMass 0.6 × 4 s, eComp and TE pressure 4 s). TEK0 is the IMU TEK without its IMU
term, a control, not ranked. Rows are defined at the end.

## Verdict

- AirMass is best in 11 of 13 rows of the short table; not in rms 10 s circling, rms 30 s circling.
- 10 s rms below eComp by 12 % and TE pressure by 13 % (all windows); in speed changes by 17 % and 20 %.
- Needle lead as shown over eComp / TE pressure, median over the flights: all 1.2 / 1.4 s, straight 1.2 / 1.6 s, circling 1.0 / 1.2 s; the required 1 s is met by the median in every phase. Flights below 1 s against eComp: all 1 of 24, straight 2 of 24, circling 10 of 24 (2026-03-03 0.0 s, 2025-04-16 0.2 s, 2025-07-08 0.2 s, 2025-05-16 0.2 s, ...).
- Simulator: 52 of 52 tests pass; scoreboard losses 8 of 34 (development), 4 of 14 (held-out) against 8 and 4 in the baseline.
- Against the baseline on the 24 flights both share: AirMass better in 0 and worse in 0 rows of the short table; 1 new flights.

## Main pool: 24 flights, 86.9 h (34.2 h straight)

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | **0.542** | 0.613 | 0.621 | 0.642 | 0.640 |
| rms 10 s straight, m/s | **0.474** | 0.595 | 0.608 | 0.632 | 0.631 |
| rms 10 s circling, m/s | 0.654 | **0.648** | 0.651 | 0.667 | 0.659 |
| rms 10 s speed change, m/s | **0.582** | 0.705 | 0.724 | 0.753 | 0.751 |
| rms 30 s all, m/s | **0.251** | 0.267 | 0.264 | 0.282 | 0.282 |
| rms 30 s straight, m/s | **0.247** | 0.280 | 0.278 | 0.300 | 0.301 |
| rms 30 s circling, m/s | 0.214 | 0.205 | **0.203** | 0.213 | 0.211 |
| rms 10 s all as shown, m/s | **0.626** | 0.761 | 0.787 | – | – |
| false climbs 2 m/s 2 s, per h straight | **11.7** | 32.4 | 35.1 | 39.8 | 37.4 |
| needle delay as shown, all, s | **1.8** | 3.0 | 3.2 | – | – |
| needle delay as shown, straight, s | **1.6** | 2.8 | 3.2 | – | – |
| needle delay as shown, circling, s | **2.2** | 3.2 | 3.4 | – | – |
| flights best at 10 s | **24** | 0 | 0 | 0 | 0 |

## Change against the baseline

Baseline: perf-2026-11-22-align-wind-tuned.json, commit d8587e71 with uncommitted changes, replay module built 2026-11-22 09:48, compensation tuned. On the 24 flights in both main pools. ▲ worse, ▼ better.

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | 0.542 (±0.000) | 0.613 (±0.000) | 0.621 (±0.000) | 0.642 (±0.000) | 0.640 (±0.000) |
| rms 10 s straight, m/s | 0.474 (±0.000) | 0.595 (±0.000) | 0.608 (±0.000) | 0.632 (±0.000) | 0.631 (±0.000) |
| rms 10 s circling, m/s | 0.654 (±0.000) | 0.648 (±0.000) | 0.651 (±0.000) | 0.667 (±0.000) | 0.659 (±0.000) |
| rms 10 s speed change, m/s | 0.582 (±0.000) | 0.705 (±0.000) | 0.724 (±0.000) | 0.753 (±0.000) | 0.751 (±0.000) |
| rms 30 s all, m/s | 0.251 (±0.000) | 0.267 (±0.000) | 0.264 (±0.000) | 0.282 (±0.000) | 0.282 (±0.000) |
| rms 30 s straight, m/s | 0.247 (±0.000) | 0.280 (±0.000) | 0.278 (±0.000) | 0.300 (±0.000) | 0.301 (±0.000) |
| rms 30 s circling, m/s | 0.214 (±0.000) | 0.205 (±0.000) | 0.203 (±0.000) | 0.213 (±0.000) | 0.211 (±0.000) |
| rms 10 s all as shown, m/s | 0.626 (±0.000) | 0.761 (±0.000) | 0.787 (±0.000) | – | – |
| false climbs 2 m/s 2 s, per h straight | 11.7 (±0.0) | 32.4 (±0.0) | 35.1 (±0.0) | 39.8 (±0.0) | 37.4 (±0.0) |
| needle delay as shown, all, s | 1.8 (±0.0) | 3.0 (±0.0) | 3.2 (±0.0) | – | – |
| needle delay as shown, straight, s | 1.6 (±0.0) | 2.8 (±0.0) | 3.2 (±0.0) | – | – |
| needle delay as shown, circling, s | 2.2 (±0.0) | 3.2 (±0.0) | 3.4 (±0.0) | – | – |
| flights best at 10 s | 24 (±0) | 0 (±0) | 0 (±0) | 0 (±0) | 0 (±0) |

## Simulator

ctest: 52 of 52 pass. `scoreboard.py`: the synthetic flights over 6 noise seeds against TE netto and the GNSS/INS reference; LOSS where either does better.

Development set, 8 losses of 34:

| metric | AirMass | TE | reference | | AirMass change |
|---|---|---|---|---|---|
| nl: cruise rms | 0.323 | 0.440 | 0.338 |  | ±0.000 |
| nl: false lift /h | 2.667 | 8.167 | 1.167 | LOSS | ±0.000 |
| nl: entry rms | 0.498 | 0.590 | 1.393 |  | ±0.000 |
| nl: entry max | 1.752 | 1.730 | 3.302 | LOSS | ±0.000 |
| nl: circling rms | 0.342 | 0.420 | 0.530 |  | ±0.000 |
| nl: climb per thermal | 0.042 | 0.030 | 0.063 | LOSS | ±0.000 |
| nl: display lag s | 1.400 | 2.300 | 0.567 | LOSS | ±0.000 |
| nl: wind instant (valid) | 0.568 | – | 1.075 |  | ±0.000 |
| nl: wind instant (all) | 0.782 | – | 1.257 |  | ±0.000 |
| nl: wind shown (valid+degraded) | 0.780 | – | 1.255 |  | +0.002 ▲ |
| nl: wind not shown % | 0.667 | – | – |  | ±0.000 |
| nl: wind 30s avg | 0.523 | – | 1.033 |  | ±0.000 |
| nl: avg vs mean, straight | 0.802 | – | 2.040 |  | ±0.000 |
| nl: avg vs mean, circling | 0.775 | – | 1.347 |  | ±0.000 |
| straight: wind rms | 0.273 | – | 1.643 |  | ±0.000 |
| straight: averaged wind rms | 0.430 | – | 1.602 |  | ±0.000 |
| straight: steady vario rms | 0.105 | 0.097 | 0.398 | LOSS | ±0.000 |
| thermal: wind rms | 0.188 | – | 1.088 |  | ±0.000 |
| thermal: averaged wind rms | 0.050 | – | 1.042 |  | ±0.000 |
| thermal: steady vario rms | 0.088 | 0.080 | 0.172 | LOSS | ±0.000 |
| gust: wind rms | 0.685 | – | 1.658 |  | ±0.000 |
| gust: averaged wind rms | 1.028 | – | 1.865 |  | ±0.000 |
| gust: steady vario rms | 0.102 | 0.082 | 0.190 | LOSS | ±0.000 |
| gust: spike max | 2.012 | 3.973 | 1.085 | LOSS | ±0.000 |
| pullup: wind rms | 0.320 | – | 1.768 |  | ±0.000 |
| pullup: averaged wind rms | 0.090 | – | 1.767 |  | ±0.000 |
| pullup: steady vario rms | 0.125 | 0.188 | 0.277 |  | ±0.000 |
| pullup: spike max | 0.617 | 0.842 | 1.767 |  | ±0.000 |
| slip: wind rms | 0.622 | – | 2.745 |  | ±0.000 |
| slip: averaged wind rms | 0.178 | – | 2.523 |  | ±0.000 |
| slip: steady vario rms | 0.220 | 0.253 | 0.490 |  | ±0.000 |
| endurance: wind rms | 0.182 | – | 1.242 |  | ±0.000 |
| endurance: averaged wind rms | 0.092 | – | 1.055 |  | ±0.000 |
| endurance: steady vario rms | 0.150 | 0.160 | 0.440 |  | ±0.000 |

Held-out days, 4 losses of 14:

| metric | AirMass | TE | reference | | AirMass change |
|---|---|---|---|---|---|
| held-out nl: cruise rms | 0.342 | 0.453 | 0.338 | LOSS | ±0.000 |
| held-out nl: false lift /h | 3.667 | 10.167 | 1.333 | LOSS | ±0.000 |
| held-out nl: entry rms | 0.492 | 0.575 | 1.228 |  | ±0.000 |
| held-out nl: entry max | 1.432 | 1.558 | 3.013 |  | ±0.000 |
| held-out nl: circling rms | 0.360 | 0.442 | 0.540 |  | ±0.000 |
| held-out nl: climb per thermal | 0.075 | 0.068 | 0.102 | LOSS | ±0.000 |
| held-out nl: display lag s | 1.417 | 2.400 | 0.367 | LOSS | ±0.000 |
| held-out nl: wind instant (valid) | 0.527 | – | 1.007 |  | ±0.000 |
| held-out nl: wind instant (all) | 0.675 | – | 1.205 |  | ±0.000 |
| held-out nl: wind shown (valid+degraded) | 0.673 | – | 1.203 |  | ±0.000 |
| held-out nl: wind not shown % | 0.333 | – | – |  | ±0.000 |
| held-out nl: wind 30s avg | 0.453 | – | 0.963 |  | ±0.000 |
| held-out nl: avg vs mean, straight | 0.830 | – | 1.755 |  | ±0.000 |
| held-out nl: avg vs mean, circling | 0.845 | – | 1.343 |  | ±0.000 |

## Wind and availability

AirMass on 28 flights (those with a faulty TE probe included), while flying (IAS over 60 km/h). The wind while
valid against the GNSS circle fits of single turns (gnssCircleWind); degraded wind is straight flight, without circle
fits, so it is compared with the wind it becomes valid at (every 10 s of a degraded stretch against the mean valid wind
of the 30 s after).

| | now | baseline |
|---|---|---|
| flying, h | 131.2 | 124.6 |
| AirMass vario valid | 99 % | 99 % |
| wind valid | 67 % | 68 % |
| wind valid or degraded (shown) | 97 % | 97 % |
| time to the first valid wind, median, s | 80 | 80 |
| flights without a valid wind | 0 | 0 |
| instant wind vs circle fit while valid, median / p90, m/s | 1.160 / 2.280 | 1.180 / 2.310 |
| averaged wind vs circle fit while valid, median, m/s | 1.230 | 1.240 |
| circle fits while valid | 1371 | 1307 |
| degraded wind vs the wind it becomes valid at, median / p90, m/s | 2.250 / 5.010 | 2.240 / 4.990 |
| degraded samples compared (every 10 s) | 14816 | 13747 |
| estimator starts / sensor gaps over 0.3 s | 75 / 1022 | 72 / 920 |
| gust alerts per h straight | 8.5 | 8.5 |

## Learned calibration

Per installation, at the end of each flight; flagged where the GNSS latency moves more than 300 ms in a flight
(the last two thirds) or differs that much between flights: one device should have one latency.

| installation (flights) | k_tas | GNSS latency at the end, ms | latency moving within a flight, max, ms | pitch offset, ° | IMU roll, ° | |
|---|---|---|---|---|---|---|
| ASG 29/18 (12) | 0.992–1.026 | 745–861 | 142 | -8.4–2.1 | 0.1–1.4 |  |
| Arcus 2025-07 (5) | 0.986–1.005 | 0–99 | 76 | -1.9–1.8 | -0.1–0.2 |  |
| Arcus 2026-05 (2) | 1.007–1.010 | 34–61 | 65 | 8.3–8.9 | -0.9–-0.8 |  |
| Arcus 2026-11 (4) | 0.990–1.011 | 131–205 | 138 | 5.3–12.3 | -3.7–-3.3 |  |
| DG 800B/18 (5) | 0.928–0.981 | 422–790 | 365 | -1.8–5.2 | -0.7–0.5 | ⚠ latency differs by 368 ms between flights; latency moves 365 ms in a flight |

## New flights since the baseline

| flight | glider | h | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) | pool |
|---|---|---|---|---|---|---|---|---|
| 2026-11-22_08-27.nmea | Arcus | 3.6 | 0.571 / 0.573 | 0.681 / 0.741 | 1.359 / 2.264 | 0.697 / 0.760 | 0.703 / 0.770 | all: faulty TE probe (TE pressure 1.359 vs eComp 0.681 m/s at 10 s) |

10 s rms, all windows / speed change, m/s.

## Full table, main pool

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | **0.542** | 0.613 | 0.621 | 0.642 | 0.640 |
| rms 10 s straight, m/s | **0.474** | 0.595 | 0.608 | 0.632 | 0.631 |
| rms 10 s circling, m/s | 0.654 | **0.648** | 0.651 | 0.667 | 0.659 |
| rms 10 s speed change, m/s | **0.582** | 0.705 | 0.724 | 0.753 | 0.751 |
| rms 30 s all, m/s | **0.251** | 0.267 | 0.264 | 0.282 | 0.282 |
| rms 30 s straight, m/s | **0.247** | 0.280 | 0.278 | 0.300 | 0.301 |
| rms 30 s circling, m/s | 0.214 | 0.205 | **0.203** | 0.213 | 0.211 |
| rms 30 s speed change, m/s | **0.276** | 0.309 | 0.301 | 0.337 | 0.339 |
| own error 10 s all (hat), m/s | **0.120–0.165** | 0.317 | 0.327 | 0.382 | 0.373 |
| own error 10 s straight (hat), m/s | **≈0–0.085** | 0.358 | 0.372 | 0.426 | 0.419 |
| own error 10 s circling (hat), m/s | 0.223–0.253 | **0.210** | 0.213 | 0.285 | 0.264 |
| own error 10 s speed change (hat), m/s | **0.102–0.179** | 0.418 | 0.443 | 0.509 | 0.499 |
| median delay, s | **2.4** | 3.0 | 3.2 | 2.8 | 2.6 |
| median needle movement in 1 s, m/s | **0.467** | 0.656 | 0.565 | 1.258 | 1.249 |
| false climbs 2 m/s 2 s, per h straight | **11.7** | 32.4 | 35.1 | 39.8 | 37.4 |
| false sinks 2 m/s 2 s, per h straight | **15.7** | 31.8 | 34.7 | 43.3 | 40.9 |
| false climbs 1 m/s 1 s, per h straight | **118.1** | 173.4 | 164.1 | 247.9 | 247.3 |
| false sinks 1 m/s 1 s, per h straight | **115.7** | 164.5 | 155.5 | 262.8 | 257.9 |
| flights best at 10 s | **24** | 0 | 0 | 0 | 0 |
| rms 10 s all as shown, m/s | **0.626** | 0.761 | 0.787 | – | – |
| needle delay as shown, all, s | **1.8** | 3.0 | 3.2 | – | – |
| needle delay as shown, straight, s | **1.6** | 2.8 | 3.2 | – | – |
| needle delay as shown, circling, s | **2.2** | 3.2 | 3.4 | – | – |

With the flights of a faulty TE probe (28 flights, 101.6 h):

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | **0.546** | 0.626 | 0.771 | 0.654 | 0.652 |
| rms 10 s speed change, m/s | **0.590** | 0.726 | 1.157 | 0.769 | 0.769 |
| rms 30 s all, m/s | **0.254** | 0.275 | 0.344 | 0.288 | 0.289 |
| false climbs 2 m/s 2 s, per h straight | **13.5** | 36.7 | 40.9 | 43.9 | 41.8 |
| flights best at 10 s | **28** | 0 | 0 | 0 | 0 |

Rows:

- rms W s PHASE: the rms over windows of W seconds of the method's mean minus the reference's mean, each method shifted by
  its fitted delay (the error of the reading, not of its timing). Straight: bank under 10° on average, circling: over 25°,
  speed change: TAS²/2g changing by more than 15 m over the window (pull-ups, push-overs, speed changes).
- rms 10 s all as shown: the same without the shift, the needle as the pilot sees it (AirMass with its own damping).
- own error (hat): the three-cornered hat with AirMass and the reference: the method's own error with the reference's
  taken out. AirMass's is a range, one per hat with each other method; ≈0 is a negative variance.
- median delay: each method's delay to the reference at the same damping. Needle delay as shown: with the damping the
  cockpit shows (AirMass 0.6 × the vario damping, the TE methods the vario damping); the lead is TE's minus AirMass's.
- needle movement in 1 s: the rms of the needle's change over 1 s, how busy it looks.
- false climbs / sinks X m/s Y s: in straight flight (bank under 10° for 8 s), the needle as shown X m/s above (below) the
  reference's centred 2 s mean for Y seconds, counted again only after it came back within X/2. About as many sinks as
  climbs means noise, not a lift bias; the 1 m/s rows are mostly the reference's own noise.
- flights best at 10 s: the flights where the method has the lowest 10 s rms (TEK0 not ranked).

## Per flight

10 s rms, all windows / speed change, m/s; setting: the compensation used (– the log's own); AirMass vario valid and wind shown (valid or degraded) while flying.

| flight | glider | h | setting | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) | vario valid | wind shown | pool |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 2024-09-21_10-38.nmea | DG 800B/18 | 1.4 | 76 | 0.517 / 0.566 | 0.518 / 0.597 | 0.520 / 0.578 | 0.540 / 0.606 | 0.525 / 0.575 | 95 % | 90 % | main |
| 2025-04-16_06-02.nmea | DG 800B/18 | 2.6 | 76 | 0.850 / 1.012 | 0.938 / 1.139 | 0.922 / 1.111 | 0.975 / 1.154 | 0.983 / 1.177 | 99 % | 90 % | main |
| 2025-05-13_08-19.nmea | DG 800B/18 | 2.5 | 76 | 20.061 / 2.844 | 20.087 / 2.918 | 20.072 / 2.967 | 20.747 / 9.965 | 20.095 / 2.932 | 96 % | 36 % | none: garbled serial data |
| 2025-05-16_11-09.nmea | DG 800B/18 | 1.9 | 76 | 0.587 / 0.631 | 0.618 / 0.675 | 0.611 / 0.652 | 0.639 / 0.682 | 0.647 / 0.712 | 100 % | 96 % | main |
| 2025-05-18_08-17.nmea | Ventus CM 17.6 | 2.1 | 92 | 0.951 / 1.032 | 0.942 / 0.874 | 1.031 / 1.056 | 15.154 / 0.805 | 0.889 / 0.761 | 96 % | 28 % | none: garbled serial data |
| 2025-06-09_09-40.nmea | DG 800B/18 | 1.6 | 76 | 0.632 / 0.622 | 0.658 / 0.669 | 0.650 / 0.660 | 0.680 / 0.737 | 0.676 / 0.725 | 99 % | 98 % | main |
| 2025-07-04_10-36.nmea | Arcus | 4.2 | 96 | 0.971 / 0.593 | 1.045 / 0.741 | 1.032 / 0.727 | 1.065 / 0.767 | 1.069 / 0.783 | 100 % | 100 % | main |
| 2025-07-05_10-13.nmea | Arcus | 2.4 | 96 | 0.492 / 0.583 | 0.623 / 0.777 | 0.623 / 0.782 | 0.651 / 0.812 | 0.649 / 0.817 | 100 % | 100 % | main |
| 2025-07-08_06-46.nmea | Arcus | 3.7 | 96 | 0.577 / 0.635 | 0.678 / 0.829 | 0.680 / 0.821 | 0.696 / 0.872 | 0.698 / 0.879 | 100 % | 95 % | main |
| 2025-07-10_10-04.nmea | Arcus | 5.6 | 96 | 0.478 / 0.517 | 0.592 / 0.654 | 0.591 / 0.658 | 0.624 / 0.703 | 0.626 / 0.709 | 100 % | 100 % | main |
| 2025-07-11_09-24.nmea | Arcus | 4.1 | 96 | 0.520 / 0.611 | 0.651 / 0.758 | 0.646 / 0.754 | 0.692 / 0.806 | 0.688 / 0.801 | 100 % | 100 % | main |
| 2025-07-12_09-05.nmea | Arcus | 4.4 | 96 | 5.032 / 0.528 | 5.039 / 0.675 | 5.038 / 0.684 | 5.042 / 0.708 | 5.042 / 0.726 | 100 % | 100 % | none: broken reference or data (AirMass 5.032, worst 5.042 m/s at 10 s) |
| 2026-03-03_12-49.nmea | DG 800B/18 | 1.5 | 76 | 0.490 / 0.514 | 0.523 / 0.535 | 0.507 / 0.524 | 0.553 / 0.583 | 0.551 / 0.582 | 99 % | 88 % | main |
| 2026-03-12_10-42_RR.nmea | Ventus CM 17.6 | – | 92 | – | – | – | – | – | 0 % | 0 % | none: no GNSS |
| 2026-05-24_11-20.nmea | Arcus | 2.5 | 100 | 0.432 / 0.416 | 0.449 / 0.483 | 0.448 / 0.445 | 0.575 / 0.761 | 0.510 / 0.620 | 100 % | 96 % | main |
| 2026-05-25_12-12.nmea | Arcus | 2.9 | 100 | 0.435 / 0.547 | 0.488 / 0.616 | 0.492 / 0.621 | 0.559 / 0.772 | 0.531 / 0.701 | 100 % | 93 % | main |
| 2026-08-22_09-38.nmea | ASG 29/18 | 4.5 | 90 | 0.451 / 0.529 | 0.490 / 0.615 | 0.504 / 0.641 | 0.513 / 0.656 | 0.510 / 0.656 | 100 % | 100 % | main |
| 2026-08-24_12-01.nmea.gz | ASG 29/18 | 4.7 | 90 | 0.550 / 0.610 | 0.636 / 0.762 | 0.653 / 0.804 | 0.666 / 0.820 | 0.664 / 0.825 | 100 % | 100 % | main |
| 2026-08-25_12-10.nmea.gz | ASG 29/18 | 3.9 | 90 | 0.448 / 0.474 | 0.516 / 0.614 | 0.536 / 0.652 | 0.543 / 0.654 | 0.542 / 0.656 | 100 % | 99 % | main |
| 2026-08-30_10-03.nmea.gz | ASG 29/18 | 6.5 | 90 | 0.426 / 0.521 | 0.482 / 0.636 | 0.490 / 0.658 | 0.504 / 0.675 | 0.501 / 0.674 | 100 % | 100 % | main |
| 2026-08-31_09-35.nmea.gz | ASG 29/18 | 6.4 | 90 | 0.547 / 0.594 | 0.647 / 0.778 | 0.677 / 0.842 | 0.671 / 0.814 | 0.678 / 0.824 | 100 % | 100 % | main |
| 2026-09-01_09-30.nmea.gz | ASG 29/18 | 4.5 | 90 | 0.424 / 0.495 | 0.459 / 0.571 | 0.477 / 0.583 | 0.481 / 0.612 | 0.476 / 0.625 | 100 % | 100 % | main |
| 2026-09-02_09-52.nmea.gz | ASG 29/18 | 1.1 | 90 | 0.381 / 0.449 | 0.396 / 0.495 | 0.386 / 0.451 | 0.424 / 0.551 | 0.411 / 0.532 | 99 % | 97 % | main |
| 2026-09-02_10-15.nmea | ASG 29/18 | 0.1 | 90 | 0.399 / 0.121 | 0.284 / 0.204 | 0.302 / 0.233 | 0.284 / 0.101 | 0.281 / 0.172 | 95 % | 86 % | none: 0.12 h scored (sensor gaps) |
| 2026-09-02_11-01.nmea | ASG 29/18 | 4.0 | 90 | 0.463 / 0.528 | 0.529 / 0.634 | 0.543 / 0.671 | 0.561 / 0.664 | 0.554 / 0.660 | 100 % | 100 % | main |
| 2026-09-02_12-05.nmea.gz | ASG 29/18 | 3.0 | 90 | 0.479 / 0.491 | 0.563 / 0.610 | 0.577 / 0.650 | 0.599 / 0.648 | 0.597 / 0.640 | 100 % | 99 % | main |
| 2026-09-05_10-50.nmea.gz | ASG 29/18 | 4.3 | 90 | 0.599 / 0.702 | 0.663 / 0.835 | 0.692 / 0.904 | 0.666 / 0.849 | 0.672 / 0.862 | 100 % | 100 % | main |
| 2026-09-05_16-16.nmea.gz | ASG 29/18 | 0.1 | 90 | 0.523 / 0.355 | 0.530 / 0.487 | 0.483 / 0.418 | 0.509 / 0.403 | 0.489 / 0.400 | 98 % | 14 % | none: 0.07 h scored (sensor gaps) |
| 2026-09-06_11-02.nmea.gz | ASG 29/18 | 5.4 | 90 | 0.488 / 0.521 | 0.548 / 0.607 | 0.569 / 0.665 | 0.567 / 0.638 | 0.565 / 0.638 | 100 % | 100 % | main |
| 2026-09-07_11-17.nmea.gz | ASG 29/18 | 4.2 | 90 | 0.482 / 0.525 | 0.553 / 0.652 | 0.568 / 0.692 | 0.585 / 0.705 | 0.581 / 0.706 | 100 % | 100 % | main |
| 2026-09-15_10-39.nmea | Ventus CM 17.6 | – | 92 | – | – | – | – | – | 0 % | 82 % | none: 0.00 h scored (sensor gaps) |
| 2026-09-17_09-11.nmea | Ventus CM 17.6 | – | 92 | – | – | – | – | – | 0 % | 45 % | none: 0.00 h scored (sensor gaps) |
| 2026-10-05_10-33.nmea | Ventus CM 17.6 | – | 92 | – | – | – | – | – | 0 % | 94 % | none: 0.00 h scored (sensor gaps) |
| 2026-10-12_10-21.nmea | Ventus CM 17.6 | 0.1 | 92 | 0.792 / – | 0.682 / – | 0.610 / – | 0.721 / – | 0.850 / – | 7 % | 81 % | none: 0.13 h scored (sensor gaps) |
| 2026-11-13_11-34.nmea | Ventus CM 17.6 | – | 92 | – | – | – | – | – | 0 % | 0 % | none: 0.00 h scored (sensor gaps) |
| 2026-11-18_10-02+11-33.nmea | Arcus | 2.7 | 90 | 0.552 / 0.678 | 0.698 / 0.864 | 1.134 / 1.827 | 0.698 / 0.852 | 0.708 / 0.869 | 96 % | 72 % | all: faulty TE probe (TE pressure 1.134 vs eComp 0.698 m/s at 10 s) |
| 2026-11-19_08-02.nmea | Arcus | 3.9 | 90 | 0.518 / 0.591 | 0.647 / 0.746 | 1.379 / 2.317 | 0.661 / 0.762 | 0.663 / 0.769 | 96 % | 97 % | all: faulty TE probe (TE pressure 1.379 vs eComp 0.647 m/s at 10 s) |
| 2026-11-21_09-04.nmea | Arcus | 4.5 | 90 | 0.617 / 0.658 | 0.769 / 0.888 | 1.558 / 2.436 | 0.806 / 0.942 | 0.800 / 0.930 | 98 % | 98 % | all: faulty TE probe (TE pressure 1.558 vs eComp 0.769 m/s at 10 s) |
| 2026-11-22_08-27.nmea | Arcus | 3.6 | 90 | 0.571 / 0.573 | 0.681 / 0.741 | 1.359 / 2.264 | 0.697 / 0.760 | 0.703 / 0.770 | 98 % | 88 % | all: faulty TE probe (TE pressure 1.359 vs eComp 0.681 m/s at 10 s) |

Pools: main without the flights marked; "all" ones (a faulty TE probe: TE pressure over 1.3× eComp) only in the
tables that say so and in wind and calibration; "none": known bad logs, under 0.2 h scored, or a broken reference (AirMass
over 1.5 m/s or any method over 3 m/s at 10 s).

Not in TUNED_SETTING, run at the default 92: Ventus CM 17.6; sweep them with te_adjust.js and add them.
