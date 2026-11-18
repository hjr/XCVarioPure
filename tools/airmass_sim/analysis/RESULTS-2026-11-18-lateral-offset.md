# 2026-11-18: the IMU's lateral offset and in-flight IMU reference changes

Follows [the Arcus ridge flight](RESULTS-2026-11-18-arcus-ridge.md): the wind was valid 2.4 % of that flight. Causes, in
order: the IMU rolled about 3.2° against the wings (+0.055 g lateral specific force in wings-level flight; earlier
Arcus flights +0.007 g), read as 4-10° of sideslip, whose uncertainty inflated every GNSS velocity's variance and kept
turn samples out of the heading alignment (circling: wind valid 15 %, other flights 91-97 %); a pitch adjustment on
the horizon page (in flight, twice), every knob step a full restart, then 68 min of straight ridge flight without a turn
to align on; little circling. Three changes:

1. `Pipeline` learns the lateral specific force per g while not rolling (`LAT_TAU` 120 s, roll rate below 0.05 rad/s,
   ratio below 0.15) and removes it before the side force becomes sideslip. Straight flight and steady turns give the
   same ratio on real flights (this flight 0.054 / 0.052, Arcus 2025-07-04 0.007 / 0.010, ASG 2026-09-07 −0.006 /
   −0.006), so it learns while circling too. Kept over restarts.
2. A new IMU reference in flight (`glider_ground_aa`, `imu_reference`, `imu_facref`; same glider and airspeed
   calibration) no longer restarts AirMass: `AirMassSensor::turnBody` takes the rotation between the old and the new
   reference (`ImuSensor::getRefRot`), `Pipeline::rotateBody` turns the attitude and its gyro bias and moves the pitch
   offset and all learned flap offsets by the turn of the body x axis; recorded as `$PXCVO,t,rx,ry,rz` (rotation
   vector) and replayed by `airmass_replay` and the browser replay.
3. `adjust_gaa()` (`main/setup/SetupNG.cpp`) built the IMU frame from the default reference plus the ground angle,
   dropping a calibrated `imu_reference` until the next reboot; it now uses the stored one, as `MpuImu::setup` does.

## Simulator (`scoreboard.py`, 6 seeds; `raw/lateral_2026-11-18_sb_*`)

Development and held-out sets unchanged within noise: losses 8 of 34 / 4 of 14 before and after; nl cruise 0.330 /
0.330, false lift 3.0 / 2.8 per h, wind instant valid 0.613 / 0.610, held-out cruise 0.348 / 0.348.

With the IMU rolled 3° (`--set mount_roll=3`), wind rms m/s (no roll in brackets):

| scenario | before | after |
|---|---|---|
| thermal | 5.00, never valid | 0.17 (0.20) |
| gust | 4.71 | 0.91 (0.74) |
| slip | 3.39 | 0.67 (0.62) |
| pullup | 1.63 | 0.29 (0.29) |
| straight | 1.51 | 0.25 (0.21) |
| nl instant (valid) | 0.90 | 0.63 (0.61) |
| nl wind not shown | 2.8 % | 1.0 % (1.0 %) |

losses 13 → 9 of 34, vario unchanged or better (nl cruise 0.350 → 0.340, thermal steady 0.107 → 0.082).

An 8° IMU reference step at 1800 s of nl (`mount_step`, `mount_step_t`, flapped): the run passes like the one without
a step (shown wind 98 %, instant 0.80 m/s both); the old restart failed the run (numerical health restart, no vario
score). New tests: `nl_mount_roll3`, `thermal_mount_roll3`, `record_mount_step` / `replay_mount_step` (the replay
follows the recorded `$PXCVO` within 0.02 m/s), the `$PXCVO` and rotation vector round trip in `record_format_test`.

## Real flights (`batch.js`, 32 logs; `raw/lateral_2026-11-18_table_*`)

Pools without the corrupt, TE-broken and gap-heavy logs (EXCLUDE of RESULTS-2026-11-10 plus the five Ventus logs of
2026-09/10, 2026-03-12_RR and 2026-11-13), gust-insensitive reference, rms m/s before → after:

| | before | after |
|---|---|---|
| equal damping 10 s all / straight / circling | 0.549 / 0.411 / 0.674 | 0.544 / 0.408 / 0.671 |
| equal damping 30 s all | 0.249 | 0.249 |
| cockpit 10 s all | 0.649 | 0.643 |
| false climbs / sinks 2 m/s 2 s per h straight | 11.6 / 12.7 | 11.0 / 12.6 |

The reference needs a valid wind, so the pools are not on the same windows: the Arcus 2026-11-18 flight scores 1.26 h
instead of 0.21 h. Per flight (22 clean ones), the 10 s vario is better on 17, the same on 2, worse on 3 (by 0.001–0.002,
and the Arcus 2026-11-18 flight on its six times longer windows); wind valid up on nearly all (Arcus 2026-05-24
56 → 64 %), median |sideslip| about 1.5 → 1.1°. Learned ratios: 0.007 on the Arcus in 2025, −0.028 on the Ventus of
2025-05 (steady over the flight, a real installation roll of −1.6°). Arcus 2026-11-18: wind valid / degraded 3 / 35 →
17 / 46 %, turns with a valid wind 1 → 12, median wind error against the turn's GNSS fit 2.32 → 1.08 m/s, sideslip
5.2 → 2.4°.

Costs on the excluded logs: the two serial-garbled Ventus logs (2025-05-13/18, two GNSS receivers interleaved with
out-of-order epochs) stay aligned 33 / 40 % of the time instead of 62 / 60 %: without the inflated sideslip variance
their bad fixes are rejected more often, ten in a row drop the alignment. Their vario is unchanged against TE netto
(the logged TE moves with the reference windows: 2025-05-18 AirMass 0.708 → 0.883, TE 0.705 → 0.904), extremes
unchanged. The gap-heavy Ventus 2026-09-17 (vario never valid in either run) lost its 10 % of wind (alignment comes and
goes with its gaps); 2026-09-15 and 2026-10-05 gained.

The real flight's restart bursts rewritten as `$PXCVO` (steps estimated from the accelerometer, −2.1, −7.7, +20.7,
−2.4, −2.0°): 43 → 2 starts, vario valid 96.6 → 97.3 %, 10 s 0.560 → 0.558. The 68 min without wind cannot be shown
on this log: the replay already lost the alignment at the logger's 50 min gap before 11:42.
