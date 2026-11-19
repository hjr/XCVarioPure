# 2026-11-19: the GNSS latency from the lateral acceleration

Follows [RESULTS-2026-11-18-gnss-latency.md](RESULTS-2026-11-18-gnss-latency.md): the learned latency was not one value
per installation even with no random walk. Found with a separate investigation (its scripts in `imu_latency.js`):

1. The "air-data-free" reference used there, the lag of the track rate behind g·tan(roll)/TAS, was itself wrong: the bank
   leads the centripetal acceleration, in the simulator it gives 1170 ms for a true 750 ms. A sound reference uses the
   IMU only: ground speed × track change between consecutive fixes against the integral of the horizontal lateral
   acceleration g·(f_y cos φ − f_z sin φ) over the same span, the latency earlier (`imu_latency.js`). Simulator 754–763 ms
   for a true 750 ms on 7 days; real flights, age at arrival (latency + median transport delay): ASG 29 844–891 ms over
   10 flights, DG 800 798–856 ms, Arcus 89–265 ms. Track and speed give the same age (ASG 2026-09-07 783 / 774 ms), the
   earlier "speed younger than course" measurement was an artefact.
2. Learned from the air vector, one latency had to fit the heading's timing in turns and the airspeed's in speed
   changes: per flight 81 ms (ASG) and 133 ms (DG) rms off the IMU reference, which is why it differed per flight and why
   pinning it to the bank-rate value (250 ms too high) had made things worse.
3. GnssTiming's transport delay counts from the earliest arrival seen since the start, the latency takes up the rest; on
   the Arcus logs converted from `$SENS` the delay reference steps (2025-07-04 by 880 ms), the latency then sits at its
   0 floor. Not changed here.

Change: the latency is learned only from that relation (`fuseInertialLatency`: a scalar update per fix pair, at most
20 ms per pair, σ 0.8 m/s of the cross-track velocity change); the air vector's GNSS fusion no longer moves it.
`q_tau` 1e-6 → 1e-7 s²/s; the first-learning phase (`q_tau_start` 1e-5 for an hour) stays.

## Real flights (32 logs, clean pool as before; `raw/inertial_latency_2026-11-19_*`)

Vario on the same windows (`same_windows.js`, `REF=first`: both against the reference of the build before), 10 s:

| | before | after |
|---|---|---|
| all (21806 windows) | 0.5441 | 0.5443 |
| ASG / Arcus / DG | 0.5099 / 0.5428 / 0.6970 | 0.5095 / 0.5427 / 0.6994 |

Wind (`batch.js`): circling wind error, mean of the per-flight medians, 1.118 → 1.138 m/s (ASG 1.250 → 1.255, Arcus
1.070 → 1.073, DG 0.920 → 0.992); wind valid / degraded 53.8 / 43.4 % → 53.9 / 43.3 %.

Latency (`imu_latency.js`), per glider over its flights:

| | latency − IMU reference, mean (sd) | latency + delay, range |
|---|---|---|
| ASG, before / after | +34 (81) / +6 (10) ms | 736–1020 / 853–919 ms (reference 844–891) |
| DG, before / after | −121 (133) / −3 (21) ms | 493–876 / 776–860 ms (reference 798–856) |
| Arcus, before / after | −8 (86) / +30 (60) ms | 41–441 / 107–440 ms (reference 89–265) |

## Simulator (`raw/inertial_latency_2026-11-19_sb_*`)

Losses 8 of 34 and held-out 4 of 14 as before, vario within ±0.005 (nl cruise 0.330 / 0.330, circling 0.355 / 0.352).
Wind: nl instant (valid) 0.610 → 0.580, thermal 0.203 → 0.200, gust 0.740 → 0.733; straight 0.210 → 0.243, pull-up
0.288 → 0.305, endurance 0.162 → 0.175 (without turns the latency is not observed, the air vector's latency could take up
the simulator's 0.3 s pitot lag). Latency of nl: 450 / 750 / 1050 ms true, 456 / 757 / 1057 estimated (it was 50 ms
short); new tests `nl_lat-300`, `nl_lat300` (`--max-latency-err 30`).

Open: the Arcus's latency stays wide with GnssTiming's delay reference (point 3); taking the airspeed 0.6–0.8 s later in
the GNSS fusion still improves the vario on all gliders (0.5454 → 0.5412 with this change in the investigation's batch
table, not same windows), not a pitot lag, unexplained.
