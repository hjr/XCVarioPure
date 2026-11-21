# All vario methods with the TE compensation applied (2026-11-21)

`te_pressure.js` as in RESULTS-2026-11-20-methods.md, on the 29 clean flights, with the TE compensation setting
applied (`COMP`): eComp and the IMU TEK use it, AirMass gets it in its setup and takes half its deviation (the half
rule, RESULTS-2026-11-21-te-adjust.md). TE pressure does not depend on it. Two runs:

- tuned (`COMP=tuned`, raw/methods-2026-11-21-tuned.*): each installation at eComp's optimum from the fine sweep,
  ASG 29 90, DG 800 76, Arcus 2025-07 96, Arcus 2026-05 100, Arcus 2026-11 90. Chosen on these flights for eComp, so
  close to eComp's best case.
- default (`COMP=92`, raw/methods-2026-11-21-default92.*): every device at the default 92, eComp 92, AirMass 96.

The tables leave out the three November Arcus flights (2026-11-18, -19, -21): their TE probe is faulty (TE pressure
1.13 / 1.38 / 1.56 at 10 s). 26 flights, 87.1 h, gust-insensitive reference, best of each row in bold, TEK0 not ranked.

## Tuned

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | **0.542** | 0.613 | 0.621 | 0.642 | 0.639 |
| rms 10 s straight, m/s | **0.474** | 0.595 | 0.607 | 0.631 | 0.631 |
| rms 10 s circling, m/s | 0.654 | **0.647** | 0.650 | 0.667 | 0.659 |
| rms 10 s speed change, m/s | **0.582** | 0.705 | 0.724 | 0.752 | 0.751 |
| rms 30 s all, m/s | **0.251** | 0.267 | 0.264 | 0.282 | 0.282 |
| rms 30 s straight, m/s | **0.248** | 0.280 | 0.278 | 0.300 | 0.301 |
| rms 30 s circling, m/s | 0.214 | 0.205 | **0.203** | 0.213 | 0.211 |
| rms 30 s speed change, m/s | **0.277** | 0.309 | 0.301 | 0.337 | 0.339 |
| own error 10 s all (hat), m/s | **0.120–0.165** | 0.316 | 0.326 | 0.381 | 0.373 |
| own error 10 s straight (hat), m/s | **≈0–0.085** | 0.357 | 0.372 | 0.425 | 0.419 |
| own error 10 s circling (hat), m/s | 0.223–0.253 | **0.209** | 0.213 | 0.284 | 0.263 |
| own error 10 s speed change (hat), m/s | **0.102–0.179** | 0.418 | 0.442 | 0.509 | 0.498 |
| median delay, s | **2.4** | 3.0 | 3.2 | 2.8 | 2.6 |
| median needle movement in 1 s, m/s | **0.467** | 0.656 | 0.565 | 1.258 | 1.249 |
| false climbs 2 m/s 2 s, per h straight | **11.7** | 32.4 | 35.0 | 39.8 | 37.4 |
| false sinks 2 m/s 2 s, per h straight | **15.7** | 31.8 | 34.7 | 43.4 | 40.9 |
| false climbs 1 m/s 1 s, per h straight | **118.0** | 173.3 | 164.1 | 247.8 | 247.2 |
| false sinks 1 m/s 1 s, per h straight | **115.9** | 164.5 | 155.7 | 262.9 | 258.0 |
| flights best at 10 s | **24** | 0 | 1 | 1 | 0 |

## Default 92

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | **0.543** | 0.621 | 0.621 | 0.648 | 0.645 |
| rms 10 s straight, m/s | **0.476** | 0.607 | 0.607 | 0.640 | 0.639 |
| rms 10 s circling, m/s | 0.654 | **0.650** | 0.650 | 0.669 | 0.661 |
| rms 10 s speed change, m/s | **0.586** | 0.736 | 0.724 | 0.775 | 0.773 |
| rms 30 s all, m/s | **0.251** | 0.271 | 0.264 | 0.284 | 0.284 |
| rms 30 s straight, m/s | **0.247** | 0.288 | 0.278 | 0.305 | 0.305 |
| rms 30 s circling, m/s | 0.214 | 0.206 | **0.203** | 0.213 | 0.212 |
| rms 30 s speed change, m/s | **0.276** | 0.319 | 0.301 | 0.343 | 0.345 |
| own error 10 s all (hat), m/s | **0.122–0.161** | 0.327 | 0.325 | 0.389 | 0.379 |
| own error 10 s straight (hat), m/s | **≈0–0.074** | 0.371 | 0.370 | 0.434 | 0.427 |
| own error 10 s circling (hat), m/s | 0.222–0.252 | 0.215 | **0.211** | 0.290 | 0.267 |
| own error 10 s speed change (hat), m/s | **0.077–0.159** | 0.451 | 0.439 | 0.531 | 0.519 |
| median delay, s | **2.4** | 3.0 | 3.2 | 2.8 | 2.6 |
| median needle movement in 1 s, m/s | **0.471** | 0.677 | 0.565 | 1.338 | 1.328 |
| false climbs 2 m/s 2 s, per h straight | **11.8** | 33.6 | 35.0 | 40.5 | 38.6 |
| false sinks 2 m/s 2 s, per h straight | **15.7** | 32.4 | 34.7 | 44.5 | 42.2 |
| false climbs 1 m/s 1 s, per h straight | **118.7** | 175.8 | 164.1 | 252.0 | 251.2 |
| false sinks 1 m/s 1 s, per h straight | **116.8** | 165.9 | 155.6 | 266.6 | 259.7 |
| flights best at 10 s | **23** | 0 | 2 | 1 | 0 |

## All 29 flights (with the November Arcus)

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 |
|---|---|---|---|---|---|
| tuned: rms 10 s all | **0.544** | 0.624 | 0.742 | 0.652 | 0.649 |
| tuned: rms 10 s speed change | **0.590** | 0.725 | 1.084 | 0.769 | 0.768 |
| tuned: false climbs 2 m/s per h | **13.2** | 35.7 | 39.2 | 42.9 | 40.7 |
| tuned: flights best at 10 s | **27** | 0 | 1 | 1 | 0 |
| default: rms 10 s all | **0.546** | 0.631 | 0.742 | 0.657 | 0.654 |
| default: rms 10 s speed change | **0.594** | 0.751 | 1.085 | 0.789 | 0.786 |
| default: false climbs 2 m/s per h | **13.2** | 36.9 | 39.2 | 43.7 | 41.8 |

## Findings

- AirMass leads in every row but circling, tuned or not; circling the three TE methods are level with it.
- AirMass hardly needs tuning: at the default (96) it is within 0.004 of its tuned score in speed changes (0.586 vs
  0.582) and 0.001 overall. Before the half rule it ran at 100 in these replays: speed change 0.600
  (RESULTS-2026-11-20-methods.md).
- eComp gains most from tuning: speed change 0.736 → 0.705, all 0.621 → 0.613; tuned it is slightly ahead of TE
  pressure at 10 s, not at 30 s.
- The IMU TEK stays last with either setting (speed change 0.752 tuned); its IMU term still does not help (TEK0 0.751).
