# 2026-11-18: one GNSS latency per installation

The test pilot of the Arcus flight ([RESULTS-2026-11-18-arcus-ridge.md](RESULTS-2026-11-18-arcus-ridge.md)) saw the
"GNSS lag" of the status page go up and down; there is one latency per device. On that flight the device's learned
latency ranged 0–776 ms (5–95 % 68–409 ms), the largest swing 364 ms within 10 min.

Two causes: the random walk `q_tau` 1e-5 s²/s, raised from 1e-6 on 2026-11-11 to follow per-flight latency
differences that turned out to be a replay bug (2026-11-14), and every start (power-up, restart) opening the
latency's uncertainty to 0.3 s even when it started from a learned value (after this flight's restarts at 11:42–11:47
the latency went 132 → 4 → 186 ms at the next alignment).

Changes: `q_tau` 1e-6; a start from a learned latency keeps its uncertainty (`Warm::latency_sigma`: the pipeline's own
restarts take the running estimate's, the device's starts 0.1 s, the uncertainty below which it saves the latency, once
one is saved); recorded in `$PXCVS` format 3 (`...,fmt,lat_sigma`), format 2 recordings start at 0.3 s as before. A start without a
learned latency learns it with the old random walk for its first hour of aligned flight (`q_tau_start` 1e-5,
`tau_start_time` 3600 s), and the device saves the latency only after that phase.

First flight of the Arcus (from 0.75 s), minutes from the first alignment until the latency stays within 50 ms of its
last hour's median, q_tau 1e-6 with the first phase of 0 / 1800 / 3600 / 5400 s: 2025-07-04 121 / 121 / 60 / 60,
07-05 125 / 111 / 55 / 55, 07-08 106 / 75 / 65 / 63, 07-10 146 / 52 / 63 / 63, 07-11 157 / 32 / 32 / 32, 07-12
110 / 70 / 45 / 45 (old firmware 34–62). Flights after it start from the learned latency: within 50 ms right away on 5
of 7 Arcus flights. Pooled scores at every phase length within 0.001 (10 s 0.544, cockpit 0.643–0.644); after 2 aligned
hours the latency swings by a median 57 ms per 10 min (q_tau 1e-5 throughout: 167 ms).

## The q_tau sweep, real flights (`batch.js`, 32 logs, cold starts; `raw/gnss_latency_2026-11-18_qtau_*`)

Pools without the corrupt, TE-broken and gap-heavy logs (as in RESULTS-2026-11-18-lateral-offset.md); the latency over
the aligned flight after its first 30 aligned minutes, median over 21 clean flights:

| q_tau | 10 s all | cockpit 10 s | false climbs 2 m/s /h | latency p5–p95 range | 10 min swing (max) |
|---|---|---|---|---|---|
| 1e-5 (before) | 0.544 | 0.643 | 11.0 | 209 ms | 167 ms (414) |
| 1e-6 | 0.545 | 0.644 | 11.1 | 121 ms | 97 ms (256) |
| 1e-7 | 0.545 | 0.646 | 11.3 | 68 ms | 30 ms (101) |
| 0 | 0.546 | 0.647 | 11.5 | 26 ms | 11 ms (40) |

Wind valid 44.2–44.3 %, degraded 37.8–37.9 % at every setting. Even a frozen latency (q_tau 0) lands on different values
per flight of the same glider (ASG 29: 630–1030 ms): it is weakly observable and catches what it learned early.
In the simulator 3e-7 and 1e-7 fail `nl_asw20_bent` (a flapped glider with a bending lift curve, day 0): the latency
settles 106 ms off (644 against 750 ms) and the shown wind is 0.98 against the reference's 0.97 m/s; 1e-6 passes it on
the days 0, 101–103 like 1e-5.

## After (q_tau 1e-6 and the learned start; `raw/gnss_latency_2026-11-18_after.txt`, `..._sb_*_after.txt`)

Simulator: 44 tests pass; development and held-out scoreboards within ±0.005 of before (losses 8 of 34, 4 of 14).
Real flights: as the 1e-6 row above (equal damping 10 s 0.545, 30 s 0.249, cockpit 0.644, false climbs 11.1 /h).

The Arcus flight, learned latency per 10 min after the re-alignment at 12:50 (min–max):

- old firmware, the log as recorded: 0–221 ms, often to 0 and back (e.g. 13:00 0–198, 15:00 86–221, 15:20 0–57)
- new firmware, the log as the device now records it (pitch adjustments as `$PXCVO`, starts with the learned
  uncertainty): 103–221 ms, 15–50 ms within each 10 min

The replays start cold (the logs before have no learned starts), so the first flight with the new firmware still learns
from 0.75 s; later flights and restarts start from what it learned.

## Why the latency moves (root-cause checks, scratch scripts not kept)

- An air-data-free latency per flight: the lag of the GNSS track rate (consecutive 1 Hz fixes at their arrival) behind
  the bank turn rate g·tan(roll)/TAS. One value per glider: ASG 29 1000–1220 ms over 10 flights, Arcus 200–320 ms,
  DG 800 920–1220 ms. Against the gyro yaw rate instead it is 0.3–0.6 s longer (the body turns before the path). The
  estimator's latency plus GnssTiming's transport delay is 100–160 ms (ASG), ~100 ms (Arcus), ~400 ms (DG) below the
  bank value and spreads 2–3 times as much over the flights.
- Where it moves (q_tau 1e-5, 10 s windows): rolling in and out / S-turns 59 % of the movement, straight 27–31 %,
  circling 10–13 %. Per thermal it changes by only 25–42 ms rms, with no left/right difference and no correlation with
  the measured sideslip or the bank.
- The simulator, whose true latency is constant, wanders 51–116 ms per 10 min (real flights 167 ms) and lands 620–770 ms
  for a true 750 ms at q_tau 1e-6: much of the movement is the estimation noise of a weakly observable state, so a
  nearly constant latency is the right model; its value is the open problem.
- Refuted: the sideslip of rolling in the air vector (`slip_roll` 0.1–0.45 s per rad/s: wander and offset unchanged,
  real and simulated); a younger GNSS ground speed than course (`gnss_speed_lead` 0.25–0.7 s: vario 10 s 0.544 →
  0.554, wander up); pitot lag: the airspeed rate follows the IMU's acceleration with 0–100 ms on most installations
  (300–400 ms on the DG 2024-09 and the Arcus 2026-11-18), the 0.2 s modelled are about right. Taking the airspeed
  0.8 s later in the GNSS fusion (`tas_lead`) improved the vario 0.5438 → 0.5402 and halved the ASG's offset to the
  bank latency, without a physical explanation (the simulator, with a 0.3 s pitot lag, is best at 0.3 s and loses it at
  0.6 s); not adopted. All three experiments were removed from the code.
