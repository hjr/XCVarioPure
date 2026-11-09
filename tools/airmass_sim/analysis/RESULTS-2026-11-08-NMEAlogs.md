# AirMass evaluation of Downloads/NMEAlogs, 2026-11-08

Replayed all **14 logs**, **75.8 recorded hours**, with the current working tree based on `16909f7a`. Estimator tuning was not changed.

## Findings

- The nine flights with enough comparable data and without severe corruption show a repeatable straight-flight gain. Circling remains mixed, with a small positive bias.
- New July 10 and July 11 logs support this pattern. Over 30 seconds TE agrees better with the reference on both flights.
- New September 17 and October 5 logs expose availability/recovery problems: sensor gaps repeatedly reset settling and motion trust. There is insufficient reference overlap to rank accuracy fairly.
- May 13 exposes a robustness defect: grossly corrupt pressure/TAS input can produce huge AirMass output while vario remains marked valid. This flight is excluded from the headline aggregate.

## Method and limits

Default replay settings, Mahony off, cold start, logged glider and mass. Implausible OAT uses the existing 25°C sea-level estimate and 6.5°C/km lapse rate.

Free-flight selection uses the existing heuristic: wait for sustained banking after takeoff or up to 15 minutes, then require IAS above 60 km/h and GS above 10 m/s. Tow, engine and landing phases are not manually labelled.

AirMass and TE use the same 4-second damping for scoring. Resample onto a 10 Hz time grid, interpolate only across measured intervals of at most 300 ms, and require complete valid windows. Fit each series' delay separately over 0–6 seconds.

Main reference: GNSS energy climb with ground velocity relative to a slowly averaged wind, minus polar sink. It shares AirMass's wind and polar assumptions, is noisy while circling, and is **not independent truth**. The pitot reference is also recorded in the JSON; it favours TE because horizontal gusts enter its energy term.

Independent wind cross-check: full-turn GNSS ground-velocity circle fits, requiring fit residual below 1 m/s. Compare the fitted wind with AirMass at the turn midpoint. These fits assume reasonably constant airspeed/wind during each turn.

Validity percentages are fractions of eligible measured free-flight sensor rows, not uniform wall-clock percentages. Missing rows are not scored. Times below are GNSS UTC; window timestamps identify their start.

## Aggregate on nine comparable flights

Exclude May 13 and all four 2026 logs. The remaining flights have about 20 hours of eligible reference overlap; complete-window selection reduces this further. RMS disagreement, m/s:

| Window / phase | Windows | AirMass | TE netto |
|---|---:|---:|---:|
| 10 s / all | 6928 | 0.765 | 0.810 |
| 10 s / straight | 2882 | 0.521 | 0.659 |
| 10 s / circling | 2531 | 1.000 | 0.982 |
| 30 s / all | 2144 | 0.325 | 0.311 |
| 30 s / straight | 750 | 0.286 | 0.306 |
| 30 s / circling | 769 | 0.322 | 0.290 |

At 10 seconds aggregate bias is −0.004 m/s for AirMass and −0.013 for TE. Straight-flight bias: −0.088 versus −0.032; circling bias: +0.082 versus +0.024. The near-zero overall mean hides a phase-dependent bias.

## All logs

RMS disagreement, m/s. † indicates insufficient or corrupted data for a performance ranking. N counts complete 10-second windows.

| Log | Glider | N | 10 s AirMass / TE | 30 s AirMass / TE | Vario valid | Wind valid |
|---|---|---:|---:|---:|---:|---:|
| 2024-09-21_10-38.nmea | DG 800B/18 | 433 | 0.603 / 0.618 | 0.272 / 0.253 | 96.1% | 68.2% |
| 2025-04-16_06-02.nmea | DG 800B/18 | 698 | 0.902 / 0.991 | 0.403 / 0.387 | 99.6% | 52.1% |
| 2025-05-13_08-19.nmea | DG 800B/18 | 789 | 15.406 / 6.580 † | 2.650 / 2.632 | 95.6% | 22.8% |
| 2025-05-16_11-09.nmea | DG 800B/18 | 405 | 0.701 / 0.684 | 0.320 / 0.306 | 99.9% | 38.7% |
| 2025-07-04_10-36.nmea | Arcus | 816 | 0.843 / 0.841 | 0.337 / 0.310 | 99.9% | 33.7% |
| 2025-07-05_10-13.nmea | Arcus | 516 | 0.615 / 0.732 | 0.260 / 0.299 | 100.0% | 33.6% |
| 2025-07-08_06-46.nmea | Arcus | 1003 | 0.497 / 0.572 | 0.289 / 0.288 | 99.9% | 46.2% |
| 2025-07-10_10-04.nmea | Arcus | 1089 | 0.794 / 0.843 | 0.315 / 0.270 | 100.0% | 34.3% |
| 2025-07-11_09-24.nmea | Arcus | 986 | 0.844 / 0.866 | 0.365 / 0.341 | 100.0% | 40.2% |
| 2025-07-12_09-05.nmea | Arcus | 982 | 0.855 / 0.909 | 0.305 / 0.317 | 99.9% | 37.6% |
| 2026-03-03_12-49.nmea | DG 800B/18 | 0 | — / — † | — / — | 100.0% | 0.0% |
| 2026-09-17_09-11.nmea | Ventus CM 17.6 | 0 | — / — † | — / — | 22.1% | 0.0% |
| 2026-10-05_10-33.nmea | Ventus CM 17.6 | 1 | 2.226 / 2.351 † | — / — | 24.7% | 8.4% |
| 2026-10-12_10-21.nmea | Ventus CM 17.6 | 3 | 0.261 / 0.430 † | — / — | 33.1% | 16.3% |

## New flights

### 2025-07-10_10-04.nmea

Arcus, 7.17 recorded hours. AirMass/TE 10-second RMS: **0.794/0.843** m/s; straight **0.552/0.681**; circling **1.021/1.018**. Over 30 seconds: **0.315/0.270** m/s.

Independent wind check: 36 qualifying circles, median vector disagreement **1.30 m/s**, 90th percentile 2.33 m/s.

Useful replay intervals: 11:43 UTC, where both varios strongly disagree with the reference, and 14:10:20 UTC, where the 10-second residual is +3.79 m/s for AirMass versus +2.91 for TE. The latter is a clearer AirMass-versus-TE difference, but still needs investigation before attributing it to gust overcompensation.

### 2025-07-11_09-24.nmea

Arcus, 5.69 recorded hours. AirMass/TE 10-second RMS: **0.844/0.866** m/s; straight **0.420/0.595**; circling **1.205/1.152**. Over 30 seconds: **0.365/0.341** m/s. AirMass circling bias is +0.101 m/s versus TE +0.040.

Independent wind check: 34 qualifying circles, median vector disagreement **1.12 m/s**, 90th percentile 2.56 m/s.

Useful replay interval: **12:06–12:08 UTC**. At 12:06:18 the 10-second residual is +5.33 m/s for AirMass versus +4.24 for TE. At 12:07:38 both reverse strongly (−5.96/−5.31). The shared reversal makes reference contamination a plausible contributor.

### 2026-09-17_09-11.nmea

Ventus CM 17.6, 7.12 recorded hours. Vario valid on **22.1%** of eligible measured rows; wind valid rounds to **0.0%**. No complete valid 10-second comparison windows.

There are **6,378 sensor timestamp gaps >300 ms** across the log. Inertial trust median is 0.309 and its 90th percentile only 0.840; 17,757 counted GNSS rejections. GNSS positions continue arriving: only four UTC epoch gaps exceed three seconds.

Follow-up raw-file inspection: 6,382 timestamp gaps exceed 300 ms before conversion, so the missing recorded history is real. This does not establish that onboard sensor acquisition stopped; the logger or transport could have lost messages while the device continued running.

The file also appears to merge two GNSS sources: 49,182 RMC messages cover 27,727 valid UTC epochs, with different solutions at 21,438 repeated epochs. Among paired airborne solutions, median speed difference is 0.53 km/h, 95th percentile 2.97 km/h; track difference is 0.34° median and 8.34° at the 95th percentile. Paired GGA altitude differences are 9 m median, 27.8 m at the 95th percentile, up to 65.9 m. This explains many duplicate-epoch rejections and can contaminate the replay reference if the sources alternate. Identify/select a consistent GNSS source before interpreting those rejections as receiver faults.

Current Pipeline marks reconstructed missing history untrusted. EKF resets vertical settling and motion trust on gaps; vario requires 50 settling cycles, motion trust needs roughly ten continuous seconds, and cold wind alignment waits for full motion trust. Repeated gaps can keep outputs unavailable. Irregular/repeated GNSS arrivals add further rejections; this is not a total GNSS outage.

### 2026-10-05_10-33.nmea

Ventus CM 17.6, 5.72 recorded hours. Vario valid on **24.7%**, wind on **8.4%**. **4,846 sensor gaps >300 ms**, inertial trust median 0.320, 13,168 counted GNSS rejections.

Only one complete 10-second comparison window survives; its 2.23/2.35 m/s RMS cannot support a performance claim.

Raw output reaches **−91.4 m/s at 14:10:05 UTC**, with vario marked invalid. The UI falls back to TE netto on invalid vario, so this raw extreme is not valid cockpit output.

## Other important failures

### 2025-05-13: corrupt input leaves extreme output marked valid

At **10:53:10 UTC**, converted pressure altitude jumps approximately **−46,243 m** and back in the next sample. TAS briefly becomes **5.88 m/s despite IAS 36.69 m/s**. This is grossly invalid air data.

At **10:55:14 UTC**, raw AirMass reaches **+341.46 m/s with status 11**, including the vario-valid bit. Unlike the invalid October extreme, this can reach the cockpit AirMass path.

GNSS altitude also has 70 jumps faster than 30 m/s, contaminating the energy reference. Full-flight 10-second RMS of 15.41/6.58 m/s is a combined failure/data-quality score, not normal vario accuracy.

Priority: validate pressure/TAS/accelerometer inputs, contain output after invalid input, and require a recovery check before restoring validity. Preserve this interval as a robustness regression case.

### 2026-03-03: no wind alignment

6,780 GNSS observations reach replay, but heading alignment and wind validity never become true, so the gust-insensitive reference has no overlap. Vario validity alone does not establish that the complete wind solution is working.

This run does not establish why alignment fails. Inspect gyro/airspeed/turn consistency and alignment fit rejection separately. All temperature samples use the altitude-based fallback.

Follow-up on November 9 identified an inconsistent axis-conversion header. The log declares
`!XCV,FMT,SENS,7,1,-1,-1,1,-1,-1`; its recorded acceleration instead matches the logged gravity direction,
so all three acceleration axes must be negated for AirMass's specific-force convention. The gyros are more
consistent with logged attitude changes when left unchanged. An in-memory replay using
`!XCV,FMT,SENS,7,-1,-1,-1,1,1,1` changes the shown AirMass at 13:07:27 UTC from +1.66 to +5.05 m/s
(TE netto +5.50), and at 13:07:47 from +6.61 to +3.80 (TE +4.07). These results supersede interpreting the
March alignment failure as an unexplained estimator limitation. The original file was not modified.
The OAT values also appear to be Kelvin (285.59 corresponds to the device's logged 12.4°C), so the fallback
is being applied to a unit mismatch. Axis correction alone produces the improvement above.
See [baseline and corrected replay](raw/march-start-axes-2026-11-09.json).

The converters now apply these repairs automatically when the independent attitude checks support them,
and recognize Kelvin OAT by agreement with PXCV Celsius. Replaying the unchanged March log after this fix gives
371 complete 10-second comparison windows: AirMass RMS disagreement 0.597 m/s versus TE 0.643 m/s, about
1.07 hours of eligible reference overlap, and 11 independent wind circles with median vector disagreement
0.91 m/s. See [metrics after conversion fix](raw/march-conversion-fixed-2026-11-09.json).

### 2026-10-12: intermittent data remains limiting

**3,514 sensor gaps >300 ms**, plus **315 GNSS epoch gaps >3 s**; longest GNSS gap **302 s**. Vario valid on 33.1% and wind on 16.3% of eligible measured rows. Only three complete 10-second comparison windows survive; their numerical ranking is not representative.

## Suggested next work

1. Contain the May 13 pressure/TAS failure and prevent extreme output from retaining the valid flag.
2. Revisit short sensor-gap recovery separately from complete vertical settling and cold wind alignment. Preserve rejection of unobserved history while avoiding repeated long dropouts after modest gaps. Check real logs and deliberate gap cases before changing the recovery policy.
3. Inspect July 10/11 circling intervals with an independent reference before changing gust compensation. The straight-flight gain is repeatable; circling bias and longer-window performance still need work.

## Reproduce

Build the browser replay with `tools/airmass_sim/web/build.sh`, then:

```sh
node --max-old-space-size=6144 tools/airmass_sim/analysis/batch.js /private/tmp/airmass-nmealogs.json ~/Downloads/NMEAlogs/*.nmea
```

Artifacts: [per-flight metrics](raw/nmealogs-2026-11-08.json), [targeted sensor diagnostics](raw/nmealogs-diagnostics-2026-11-08.jsonl), [batch script](batch.js).
