# Gust detector (Gust Sound), 2026-11-17

`AirMassGust.h`: the head wind change within 5 s, from the airspeed derivative minus the inertial acceleration along
the air velocity, with its slow part (attitude error) taken out over 30 s from the estimator's own 5 s low pass. An
event needs 5 m/s, the airspeed itself changing the same way by half of that, straight flight (the kinetic blend's
bank weight ≥ 0.9), a valid AirMass vario, and 6 s since the last event.

## Synthetic flights

| scenario | detector peak (no threshold) | events |
|---|---|---|
| gust, 4 m/s head wind in 2 s, back in 4 s | +3.8 / −4.2 m/s | none (below 5 m/s) |
| gust6, the same with 6 m/s | | +1 at 252.5 s, −1 at 265.6 s |
| straight, thermal, slip, endurance | ≤ 0.9 m/s | none |
| pullup | 1.1 m/s | none |
| nl (turbulence) | 1.9 m/s | none |

The estimator's outputs are bit-identical to before on all seven scenarios.

Without the detector's own slow part, the estimator's 5 s low pass took a third of the gust: the 4 m/s gust read 2.4 m/s.

## Real flights (`gust_events.js`, ~/Downloads/NMEAlogs)

28 clean logs (the 11 ASG 29 `.nmea.gz`, the plain logs without a `.gz` twin; without the two serial-garbled and
the five gap-heavy Ventus logs), 67.0 h of straight flight (|roll| < 10°, vario valid, IAS > 60 km/h):

| threshold (no airspeed condition, 8 ASG 29 flights, 28.1 h; an earlier recount mode of `gust_events.js`) | per hour |
|---|---|
| 4 m/s | 25.1 |
| 5 m/s | 10.0 |
| 6 m/s | 3.6 |
| 7 m/s | 1.2 |

At 5 m/s, a 3 s window with the estimator's slow part showed events with hardly any IAS change: errors of the
inertial acceleration, mostly after rolling out of a turn. The airspeed condition and the valid vario remove them.

Final detector on the 28 clean logs: **484 events, 7.2 per hour** of straight flight (head wind increase 299, 4.5/h;
drop 185, 2.8/h), per flight of over half an hour 1 to 23 per hour (2025-04-16 DG 800B 23/h, 2026-08-31 ASG 29 17/h).
Before the window kept the remainder of a closing bucket (a 99 ms cycle left a bucket open for a second cycle, so the
logged cycle jitter stretched the 5 s window) it was 534, 8.0 per hour; the robustness figures below are from then.

Robustness:
- 2026-11-13 replays with the estimator status 0 all flight (as before this change, start pitch offset 10.6°); it had 2 events before the
  vario condition, none now.
- The serial-garbled 2025-05-13 and 2025-05-18 give 13 and 31 per hour: corrupt readings that pass the plausibility
  guards look like gusts. The device reads its sensors directly and does not see this logger corruption.
- The gap-heavy Ventus logs: none (the estimator rarely settles there).

## Gusts next to AirMass climbs and sinks

Half the gust events (113 of 269 on the 8 ASG 29 flights) come with an AirMass vario jump of more than 1.5 m/s, the
head wind increase with climbs (85 against 8), the drop with sinks (49 against 6); 21 % of random straight-flight
moments jump that much. They are real gusts, not errors of the inertial acceleration:

| | gust events | airspeed changes of 4 m/s within 4 s without an event |
|---|---|---|
| airspeed change | 5.2 m/s | 4.8 m/s |
| GNSS ground speed change | 0.4 m/s (7 %) | 3.8 m/s (77 %) |
| TE probe energy beyond the static altitude per pitot kinetic energy | 0.81 | 0.91 |
| AirMass jump minus the gust-insensitive GNSS reference's, in the gust's sign | +1.6 m/s | +0.4 m/s |

The glider did not speed up (the ground speed stays), and the TE probe sees the airspeed change the pitot shows. At the
vario-coupled events the GNSS reference climbs too (+1.0 m/s in the gust's sign): straight flight into lift comes with
a head wind increase. AirMass adds about 1.2 m/s of the gust on top, its known gust leak (simulator: 1.9 m/s for a 4 m/s
gust, TE netto 3.9).

## The detector's 30 s slow part in the vario's gust blend: not taken

`kinetic_tau_straight` (the blend's slow part in straight flight, faded to `kinetic_tau` 5 s with the blend's bank
weight) takes the simulator's gust spike from 1.93 to 1.24 / 1.05 / 0.95 m/s at 15 / 30 / 60 s (6 m/s gust 2.63 to
1.18 at 30 s), nl cruise 0.33 to 0.31 m/s and false lift 3 to 1, everything else equal. On the 11 ASG 29 flights it is
worse throughout (`raw/exp_real_kinetic_tau_straight_2026-11-17.txt`, `raw/needle_real_kinetic_tau_straight_2026-11-17.txt`),
against the gust-insensitive reference:

| | now (5 s) | 15 s | 30 s | 60 s | TE netto |
|---|---|---|---|---|---|
| 10 s straight / circling | 0.402 / 0.611 | 0.426 / 0.623 | 0.467 / 0.632 | 0.501 / 0.641 | 0.549 / 0.611 |
| 30 s straight / circling | 0.242 / 0.198 | 0.300 / 0.219 | 0.351 / 0.237 | 0.395 / 0.253 | 0.276 / 0.194 |
| own error 30 s straight (hat) | 0.096 | 0.201 | 0.271 | 0.327 | 0.162 |
| needle 1 s straight | 0.589 | 0.547 | 0.541 | 0.539 | 0.879 |

The real attitude error of the inertial acceleration changes within seconds in straight flight too; the detector gets
away with 30 s because a drift of tenths of m/s² never adds up to 5 m/s within 5 s with the airspeed following.

## Pausing the blend's slow part while the airspeed changes: not taken

`kinetic_hold` (`kinetic_tau`'s low pass pauses as the airspeed derivative reaches it: a gust changes the airspeed, an
attitude error should not) takes the simulator's gust spike from 1.93 to 1.00 / 1.25 / 1.43 m/s at 0.5 / 1 / 2 m/s²
(6 m/s gust 2.63 to 0.93 at 0.5), the pull-up spike 0.68 to 0.61. On the 11 ASG 29 flights it is worse throughout and
reads low in straight flight (`raw/exp_real_kinetic_hold_2026-11-17.txt`, `raw/needle_real_kinetic_hold_2026-11-17.txt`):

| | now | 0.5 m/s² | 1 m/s² | 2 m/s² | TE netto |
|---|---|---|---|---|---|
| 10 s straight / circling | 0.402 / 0.611 | 0.509 / 0.627 | 0.511 / 0.622 | 0.467 / 0.616 | 0.549 / 0.611 |
| 30 s straight (mean) | 0.242 (−0.01) | 0.406 (−0.11) | 0.401 (−0.10) | 0.347 (−0.07) | 0.276 (−0.03) |
| own error 30 s straight (hat) | 0.096 | 0.340 | 0.334 | 0.266 | 0.162 |

What `kinetic_tau` follows on real flights changes with the airspeed, as the error of the AoA model does (the modelled
angle of attack goes with 1/V²): pausing it during airspeed changes stops it when it is needed. Neither this nor a slower
slow part gets the gust leak down without losing the vario's accuracy; a better AoA model might.

## The device's airspeed low pass: AirMass now reads the airspeed without it

On the bench (web serial replay at 1×) the device sounded gusts the web replay did not mark and missed marked ones. The
device fed AirMass the airspeed after AirspeedSensor's low pass (alpha 0.25 at its 10 Hz reads, 0.3 s), in flight as in
replay; the replays (`convert.js`, `sens2rec.py`) and the simulator took the raw reading, as all tuning so far did.

With the low pass modelled in a converter, the gust events of three ASG 29 flights change: 13 / 74 / 36 events become
6 / 58 / 27, 46 / 76 / 58 % of them at the same moment. Modelled in the simulator, the device as it was shows a pull-up
spike of 1.44 m/s (TE netto 1.37; 0.68 without the low pass), nl cruise 0.33 → 0.37 m/s, false lift 3 → 5, endurance
vario 0.15 → 0.23. The estimator's `lag_pitot` at the true total lag (0.5 s) restores the simulator (pull-up 0.76, endurance
0.16). On the 11 ASG 29 flights with the low pass (`raw/exp_real_dynp_filter_lag_pitot_2026-11-17.txt`,
`raw/needle_real_dynp_filter_lag_pitot_2026-11-17.txt`), against the gust-insensitive reference:

| | 10 s straight / circling | 30 s all | needle delay (circling) |
|---|---|---|---|
| raw airspeed, `lag_pitot` 0.2 (all earlier analyses, now the device) | 0.402 / 0.611 | 0.238 | 1.8 s |
| low pass, `lag_pitot` 0.2 (the device before) | 0.419 / 0.622 | 0.239 | 1.8 s (2.0 s) |
| low pass, `lag_pitot` 0.5 | 0.406 / 0.611 | 0.237 | 2.2 s (2.6 s) |
| low pass, `lag_pitot` 0.6 | 0.402 / 0.607 | 0.236 | 2.2 s (2.8 s) |

Taken: AirMassSensor computes its airspeed from the unfiltered dynamic pressure (the airspeed sensor's latest reading,
calibrated); the display, the TE vario and the flap recommendation keep the filtered one. The device now runs what the
replays, the simulator and the tuning ran, and the gust rate above stands.
