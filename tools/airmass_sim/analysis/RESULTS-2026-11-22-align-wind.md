# The wind's uncertainty after a turn alignment (2026-11-22)

The turn alignment (`AirMassEKF::accumulateAlignment`, at least 180° of heading) handed over the wind with a fixed
1 m/s each way, 1.41 m/s in all. The validity limit is 1.5 m/s (or 4.4 % of the airspeed), a first validity 10 % below
it: a fresh alignment alone was valid only above ~128 km/h. On 2026-11-21 (Arcus) the first turn aligned at 628 s,
then six minutes of straight flight kept the wind hidden until the next turn made it valid at 993 s.

Simulator, 56 alignments (thermal, nl with turbulence 0.5/1/2, straight, gust, endurance, 8 seeds each), the wind at
the first alignment against the truth (`aligncal.sh`, scratch): error rms 1.19, median 1.00, 90 % 2.04, max 2.41 m/s.
Claimed 1.41: error/claimed rms 0.84; heading error ~2° against 5.7° claimed (rms 0.36, it does not limit validity).
The fit's own uncertainty (its residual variance through the inverse normal matrix) claims about a third of the
error: the errors of a turn are shared (turbulence, timing), not independent per fix. So a fixed 0.9 m/s each way
(1.27 in all, the measured rms): valid right after the turn.

Standard performance run (`RESULTS-2026-11-22-performance-align-wind-tuned.md` against perf-2026-11-21-tuned): every
vario row ±0 on the 24 shared flights; simulator 52/52 tests, scoreboard losses 8/34 and 4/14 as before, wind metrics
within ±0.008 m/s. Real flights, wind (27 flights, 124.6 h):

| | now | before |
|---|---|---|
| time to the first valid wind, median | 80 s | 99 s |
| wind valid / shown | 68 % / 97 % | 68 % / 97 % |
| instant wind vs circle fits while valid, median / p90 | 1.18 / 2.31 m/s | 1.18 / 2.31 m/s |
| degraded wind vs the wind it becomes valid at, median / p90 | 2.24 / 4.99 m/s | 2.24 / 4.98 m/s |
| degraded samples (every 10 s) | 13747 | 13694 |

2026-11-21: valid at 628 s (before 993 s), then grey after 10 s of straight flight until the next turn instead of
hidden; 2026-11-19: valid at 744 s (856 s); 2026-11-18 was valid at its alignment already.
