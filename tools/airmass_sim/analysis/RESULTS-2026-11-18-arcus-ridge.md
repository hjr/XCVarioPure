# 2026-11-18: Arcus ridge flight, South Africa, AirMass on the device

Logs: `~/Downloads/20261118/2026-11-18_10-02.nmea` (power-up, aerotow, first 30 min, until 10:43 UTC) and
`2026-11-18_11-33.nmea` (11:33–16:35 UTC, landing; the logger was off from 10:43 to 11:33, the device was not). Arcus,
795 kg (CA1 in the first log only: the second needs it prepended, otherwise the page takes the ASG 29/18 polar),
`$SENS` at 10 Hz, Flarm-style 1 Hz GNSS, the device's `$PXCVS/C/A` records, 90 sensor gaps over 300 ms in 5.4 h.
Photos from the flight (phone clock 1 h ahead of UTC): XCVario in a bracket in the rear cockpit, ridge soaring along the
Cape ranges, the status page at 16:08:3x UTC (pitch 8.09°, lag 87 ms: matches `$PXCVA` to the second).

The flight was ridge soaring: 7 full turns in 4.7 h, bank over 25° 3 % of the time.

## Vario

Replay against the device: corr 0.994, rms 0.18 m/s (4 s damping), so the replay scores are what the pilot saw.

The usual gust-insensitive reference uses AirMass's wind only while WIND_VALID, 2 % of this flight: `batch.js` scores
10 min of it. Scored here instead (scratch script, not kept) against the same reference with AirMass's averaged wind
wherever ALIGNED (3.5 h, 12:55–16:35) and against the pitot reference (4.55 h). Equal 4 s damping for AirMass, each
series shifted by its fitted delay, logged TE as the device damped it. TE pressure (the page's `TeVario`) is much worse
than the logged TE on this log (10 s 1.15 straight) and is left out.

| reference, window | AirMass | logged TE | hat own AirMass / TE |
|---|---|---|---|
| aligned wind, 10 s straight (869) | **0.565** | 0.654 | 0.08 / 0.34 |
| aligned wind, 30 s straight (258) | **0.359** | 0.383 | 0.07 / 0.15 |
| pitot, 10 s straight (1158) | 0.640 | **0.596** | 0.33 / 0.23 |
| pitot, 10 s circling (64) | **0.435** | 0.610 | 0.00 / 0.38 |
| pitot, 30 s straight (343) | 0.293 | **0.240** | 0.18 / 0.05 |

The aligned-wind reference's circling windows are dominated by its own error (1.75), not scored. The pitot reference
shows horizontal gusts as air movement like TE, which in 25–45 km/h wind along a ridge favours TE (as on all earlier
flights). Delays (aligned wind): AirMass 2.4 s, logged TE 2.8 s. 1 s needle movement: AirMass 0.49, logged TE 1.55 m/s.

## What went wrong on the device

1. **The IMU pitch reference was changed in flight, twice.** Every `$PXCVS` burst (10:16:07–09 on tow: 8 starts;
   11:42:19–11:47:14: 25 starts) coincides with a step of the body-frame x specific force in straight flight
   (10:16 +0.06 → +0.12 g; 11:42–11:47 +0.03 → +0.17 → −0.21 → −0.16 → −0.12 g). Each step changes
   `pitchSignature()`, so `postProcess()` restarts the estimator with no learned pitch or flap offsets. No NaN stop
   involved. The learned pitch offset went from about 0° to 8–12° for the rest of the flight.
2. **No alignment for 68 min after the last restart (11:47–12:55).** Turn alignment needs 180° of heading within
   180 s; along the ridge the largest heading span in 180 s stayed under 100°, except for one 240° turn at 12:10 that
   was not accepted. Without alignment no wind, no GNSS fusion, and the energy-only vario.
3. **The wind was hardly ever valid: 2.4 % of the airborne time, degraded 36 %.** On the status page at 16:08
   "Wind: --". Part of it is the few turns. Part is the installation: the lateral specific force is +0.055 g in
   wings-level flight all day (IMU rolled about 3.2° in the bracket). AirMass reads it as 4–9° sideslip (median),
   above `SLIP_ALIGN_MAX` (0.1 rad) much of the time and inflating the GNSS velocity variance. The same log with the
   IMU rotated back by 3.15° about x (accel, gyro, `$PXCVS` vectors): sideslip median 5.0 → 2.3°, wind valid
   2.4 → 14.6 %, degraded 35.7 → 51.8 %, vario unchanged within the reference noise (aligned 10 s straight
   0.565 → 0.531, pitot 0.640 → 0.636), alignment times unchanged.
4. The device AHRS pitch in `$PXCV` reads −20° to −50° in straight flight for long stretches (12:00–12:40), the photo at
   14:40 shows −15.3° in cruise. Not AirMass's (it runs its own attitude), but it is what the pilot sees in the AHRS
   screen.

Other: learned k_tas 0.99–1.006, GNSS latency 4–335 ms wandering (Arcus earlier: 0–0.2 s), 24–26 rejected fixes,
flap sensor jittering in 0.1 steps (28k `!XCV,FLP` lines in 5 h).

## Wind

Only 7 full turns (`xcsoar_wind.js`): against the next turn's GNSS fit (4 turns) AirMass nav 0.96, instant 0.77,
XCSoar zigzag 1.11, XCSoar circling 1.12 m/s; straight 0.5–2 min after a thermal (36 samples) AirMass nav 0.59, XCSoar
1.39, XCSoar circling 0.85. Too few samples to rank. `opposite_legs.js` finds no opposite leg pairs with a wind (and
crashes on the empty set).

## Both logs joined

`2026-11-18_10-02+11-33.nmea` (the two logs, the CA1 now at the start): 5.33 h airborne, 25 full turns, vario valid
96.6 %, wind valid 2.9 %, degraded 35.5 %, replay against the device rms 0.21 m/s (corr 0.992). Aligned-wind reference
(3.8 h): 10 s straight AirMass 0.569 / logged TE 0.655 (hat 0.12 / 0.34), 30 s straight 0.341 / 0.364, 10 s circling
1.48 / 1.47 (reference 1.47 of it). False climbs ≥ 2 m/s for 2 s in straight flight (1.74 h): AirMass 23/h, TE 48/h;
false sinks 12/h vs 38/h. Wind against the next turn's GNSS fit (19 turns): AirMass nav 1.37, XCSoar zigzag 1.45,
XCSoar circling 2.04 m/s.

The log-2 sensors before its first RMC made the converter collapse the 50 min logging gap (its forward-gap check only
had the last RMC's time), the replay then skipped 32690 cycles from 11:42 on. Fixed: the check now takes the newest fix
time of RMC or GGA (`laterUtc` in convert.js, `later_utc` in sens2rec.py). Of 57 real logs only
`NMEAlogs/2026-10-12_10-21.nmea` converts differently (a 28 s gap at 13:26 UTC, resumed with a GGA, now kept): its
replay restarts there and the vario never settles again (30 s in-flight settle against the logger's frequent short
drops), 0.31 → 0.10 h scored; the short-gap recovery item again.

## Possible changes

- Restart on a pitch-signature change only once the setting has been stable for a few seconds, or keep running and
  shift the pitch offset by the known rotation: one knob turn gave 8–11 restarts here.
- Ridge flights: a heading alignment from the 180° reversals at the ridge ends (wider than 180 s, or from shorter
  partial turns) would have shortened the 68 min without wind.
- Learn or remove a constant lateral specific-force offset (installation roll) before the side force becomes a
  sideslip, or tell the pilot to level the IMU reference in roll.
