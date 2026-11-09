# New NMEA log evaluation, 2026-11-09

One added file: `~/Downloads/NMEAlogs/2026-09-15_10-39..nmea` (26,732,716 bytes).
The other 14 names were covered by the previous batch. Replay uses the current module and default settings,
Mahony off, glider and weight from the log. No estimator tuning was changed.

## Results

Ventus CM 17.6, logged mass 329 kg, approximately 4.36 recorded hours. No temperature fallback was needed.

| Measure | Result |
|---|---:|
| Raw sensor rows | 126,189 |
| Raw sensor gaps longer than 300 ms | 3,631 |
| Backward sensor timestamps | 0 |
| Valid RMC messages / unique UTC epochs | 13,237 / 13,237 |
| Repeated UTC epochs | 0 |
| Replay GNSS observations | 12,521 |
| Counted GNSS rejections | 1,769 |
| Vario valid | 25.46% |
| Wind valid | 7.52% |
| Wind degraded | 55.94% |
| Heading aligned | 87.79% |
| GNSS fresh | 82.89% |
| Median inertial trust | 0.32 |

Validity percentages describe eligible measured free-flight rows using the same heuristic as the November 8
batch; they are not percentages of uniform wall-clock time. Raw sensor-gap counts include the whole file.
The converted dataset has 3,630 gaps over 300 ms; raw-file counting includes one additional interval.

Unlike September 17, this file contains no repeated RMC UTC epochs and no evidence of conflicting RMC solutions
at the same epoch. It therefore demonstrates that the mixed-GNSS pattern is not necessary for the poor replay
availability: frequent missing sensor history is sufficient to keep recovery interrupted.

The common long sensor intervals are 0.8 s (643), 0.9 s (584), 0.7 s (346), and 0.4 s (345). These gaps are
already present in the raw timestamps. They could reflect logger/transport loss; they do not establish an onboard
sensor outage. Current replay reconstructs missing cycles as untrusted, resets vertical settling, and needs roughly
five seconds to restore vario validity and ten seconds for full inertial trust. Repeated gaps interrupt recovery.

The replay track has 369 GNSS epoch gaps longer than three seconds, the longest 16 seconds. This is missing recorded
GNSS history; its cause is not established. Some recorded GNSS data is also skipped by conversion outside flying
conditions or when track is unavailable, so total RMC and replay-observation counts need not match.

## Wind check

Ten full-turn GNSS circle fits pass the independent quality gate (fit residual below 1 m/s) and overlap valid
AirMass wind. Vector disagreement is **0.74 m/s median**, **1.23 m/s at the 90th percentile**. Averaged AirMass wind
has a 0.83 m/s median vector disagreement. These are limited spot checks, assuming fairly steady airspeed and wind
within each circle; they do not validate wind throughout the flight.

The valid instantaneous wind speed has a median of 3.71 m/s (13.3 km/h) and a 95th percentile of 5.85 m/s
(21.1 km/h). End-of-replay calibration: k_tas 1.0126, GNSS latency 251 ms, pitch offset 3.582°, AoA scale 0.9733.

## Vario check

Only about 6.5 minutes of fragmented sample overlap survive the AirMass-valid and gust-insensitive GNSS-reference
checks. After resampling onto a 10 Hz time grid, rejecting sensor gaps, and requiring shared valid windows at each
series' fitted delay, **no complete 10-second scoring windows survive**. The run cannot support an AirMass-versus-TE
accuracy ranking. It does not mean there are no valid vario samples.

The largest raw AirMass output is +33.50 m/s at 10:50:55 UTC, with status 0 (invalid). It should therefore use the
TE fallback in the cockpit display. There are no consecutive converted barometric-altitude jumps over 20 m in
this run, unlike the May 13 corruption case.

## Implication

This file reinforces the need to examine short sensor-gap recovery independently from GNSS source selection.
Wind can agree well with the GNSS-only check when available; replay spends much of this flight recovering from
missing recorded history. Do not infer the onboard estimator's availability directly from this incomplete recording.

## Artifacts

- [Replay metrics](raw/nmealogs-new-2026-11-09.json)
- [Replay diagnostics](raw/nmealogs-new-diagnostics-2026-11-09.jsonl)
- [Raw file quality](raw/nmealogs-new-quality-2026-11-09.json)

Reproduce with `node tools/airmass_sim/analysis/batch.js /private/tmp/sep15.json ~/Downloads/NMEAlogs/2026-09-15_10-39..nmea`.
