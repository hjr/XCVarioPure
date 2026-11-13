# The vario after a start in flight, 2026-11-13

On the instrument the AirMass needle swung for the first minute of a replay. A replay jump (`$PFLAX,S`) resets the AHRS,
and the estimator restarted from that attitude: the accelerometer direction alone, level in every turn. The vario was
declared valid after 5 s.

## Restarts in real flights

`restart.js`: 137 restarts, one every ten minutes of four ASG 29 logs (`flightlogs/discussion451`, 2026-08-30, 08-31,
09-01, 09-07), with the calibration of the moment. rms difference of the vario to the uninterrupted estimator in m/s,
by time since the restart. About 0.3 remains in every column: the restart also loses the learned flap offsets.

Before (valid after 5 s), by start attitude:

| until s | accelerometer, straight | true, straight | turn-corrected, straight | accelerometer, banked | true, banked | turn-corrected, banked |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 0.81 | 0.39 | 0.65 | 4.61 | 0.32 | 0.91 |
| 20 | 0.56 | 0.35 | 0.61 | 2.96 | 0.27 | 0.86 |
| 30 | 0.68 | 0.30 | 0.42 | 4.11 | 0.29 | 0.73 |
| 45 | 0.48 | 0.32 | 0.36 | 3.79 | 0.25 | 0.51 |
| 60 | 0.40 | 0.33 | 0.36 | 2.95 | 0.28 | 0.49 |
| 90 | 0.35 | 0.32 | 0.34 | 2.81 | 0.23 | 0.31 |
| 120 | 0.32 | 0.31 | 0.32 | 2.39 | 0.21 | 0.25 |
| 180 | 0.28 | 0.28 | 0.28 | 1.78 | 0.26 | 0.27 |

With the true attitude there is no transient: it is the start attitude, not the estimator's filters. Banked more than
20 degrees at the restart, the accelerometer attitude leaves the vario wrong by metres per second for minutes.

## Change

- A restart after a replay jump takes the down axis from the accelerometer with the air path's centripetal acceleration
  (rate of turn times airspeed) taken out, `airmass::downFromMotion`, the estimate the replay already used for logs that
  begin airborne. Another start (takeoff, estimator restart) keeps the converged AHRS attitude.
- The first validity after a start at flying speed takes 30 s of supported flight instead of 5 s
  (`SETTLE_IN_FLIGHT_MS`); the vario shows the TE fallback meanwhile. An interruption after that settles in 5 s as before.

After, the same restarts (`raw/restart-2026-11-13.txt`): no valid vario before 30 s, then

| until s | turn-corrected, straight | turn-corrected, banked |
|---:|---:|---:|
| 45 | 0.36 | 0.51 |
| 60 | 0.36 | 0.49 |
| 90 | 0.34 | 0.31 |
| 120 | 0.32 | 0.25 |

## Whole flights

`REF=first same_windows.js` on the 43 replayable logs before and after (`raw/same-windows-settle-2026-11-13.txt`): where
both are valid the vario is the same sample by sample (largest difference 0.0000 m/s), pooled scores unchanged (10 s
equal damping 0.692 / 0.877). Scored hours 107.75 before, 107.10 after: the first 30 s of each flight and of each
restart after a sensor outage of more than 10 s now show the TE fallback.

Simulator (`airmass_sim nl`, `thermal`, `gust`, `straight`): unchanged, cruise / thermal entry / circling rms 0.34 / 0.51 /
0.36, thermal vario rms 0.09, gust 0.10 (spike 1.98), straight 0.11.

Not measured: the instrument itself. The replay's start estimate uses the angle of attack model as well; column
"turn-corrected" is without it.
