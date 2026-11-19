# 2026-11-19: the GNSS velocity carried to now (`gnss_bridge`)

Follows the open point of [RESULTS-2026-11-19-inertial-latency.md](RESULTS-2026-11-19-inertial-latency.md): taking the
airspeed 0.6–0.8 s later in the GNSS fusion improved the vario on all gliders.

## The airspeed is not late

- Logged TAS rate against the estimator's `acc_long` (both low passed 0.5 s, only |acceleration| > 1 or 2 m/s²): the
  airspeed's lag is ASG −0.28..−0.05 s, Arcus −0.18..+0.19 s, DG −0.07..+0.28 s over 14 flights. The same method gives
  0.31–0.34 s in the simulator for its true 0.3 s pitot lag.
- TAS change against the GNSS speed change between fixes is no reference: in a horizontal gust the airspeed changes at
  once, the ground speed only through drag.
- The gain is real on the same windows (`same_windows.js`, `REF=first`, 10 s, clean pool): 0.5444 → 0.5407 (ASG
  0.5091 → 0.5047, DG 0.7059 → 0.6995, Arcus 0.5424 → 0.5411; `raw/gnss_bridge_2026-11-19_airspeed_later_same_windows.txt`).

Explanation: the fix was compared with the air vector of its own time, about 0.9 s ago on the ASG and the DG. The wind
state was the wind of then, and the wind-work term of the vario used it. A later airspeed gave a fresher along-track
wind. The Arcus's fixes are 0.1–0.3 s old, so it gained least.

## Change

The horizontal inertial acceleration (`acc_fwd` new, `acc_lat`) is integrated in the gyro heading frame and stored in
every snapshot. A fix is carried from its observation time to the newest cycle by that integral's difference, rotated
by the current heading offset. It is then compared with the air vector of that cycle. The heading Jacobian includes the
bridge, its variance is 0.1 m/s per s of span (0.05–0.2 make no difference). If the history cannot bridge, the fix is
fused at its time as before. `gnss_bridge = 0` restores the old fusion.

## Real flights (clean pool; `raw/gnss_bridge_2026-11-19_same_windows.txt`)

| | before | after |
|---|---|---|
| vario 10 s, same windows (21794) | 0.5457 | 0.5418 |
| ASG / Arcus / DG | 0.5090 / 0.5429 / 0.7093 | 0.5032 / 0.5409 / 0.7077 |
| false climbs ≥ 2 m/s per straight hour | 11.1 | 10.3 (ASG 9.0 → 7.6) |
| circling wind error, mean of medians | 1.138 | 1.148 (ASG 1.255 → 1.283, Arcus 1.073 → 1.021, DG 0.992 → 1.056) |

30 s windows, all flights: 0.2481 → 0.2487 (straight 0.2419 → 0.2440).

The airspeed taken 0.4 or 0.8 s later on top of the bridge makes the vario worse (0.5437 → 0.5490, 0.5423 → 0.5501;
`raw/gnss_bridge_2026-11-19_bridge_plus_airspeed_later.txt`): the bridge explains the earlier gain fully.

## Simulator (`raw/gnss_bridge_2026-11-19_sb_*`)

Vario better everywhere: nl cruise 0.330 → 0.322, false lift 3.0 → 2.7 /h, entry 0.513 → 0.498, circling 0.352 → 0.342;
held-out cruise 0.350 → 0.340, false lift 4.7 → 3.7 /h. Wind: nl 0.580 → 0.572, gust 0.733 → 0.693; straight
0.243 → 0.288, thermal 0.200 → 0.217, pull-up 0.305 → 0.330, endurance 0.175 → 0.192.

Why the wind is worse there: the bridge itself is accurate (0.04–0.07 m/s from the true velocity change over 0.8 s). The
air vector of the newest cycle is a single airspeed sample (error rms 0.20 m/s), at the fix's time it was interpolated
between two (0.175). Averaging the air vector over 5 cycles (the fix carried to their middle) brings endurance, pull-up
and thermal back (0.174, 0.306, 0.205; straight 0.281), on real flights it costs a quarter of the vario gain (0.5436 →
0.5445) for the wind (1.148 → 1.130). Carrying the recent air vectors forward by the inertial acceleration is the same
thing: a wind change inside the window is not in the acceleration, the wind becomes the window's average. Not taken,
the vario has priority. Not the cause: the pitot lag (the same loss without it), the bridge's variance, its heading term.
