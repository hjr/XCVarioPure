# AirMass reevaluation after conversion fixes, 2026-11-09

**15 logs, 80.2 recorded hours.** Current default estimator, Mahony off, logged glider and mass. The March log now receives validated axis and Kelvin-temperature corrections. No estimator tuning was changed.

This supersedes the earlier batch's interpretation of the March flight. The other 13 flights from the November 8 batch reproduce their previous equal-damping scores exactly; September 15 was added separately. Input hashes and converter/module hashes are recorded in the raw metrics.

## Main result

AirMass shows a repeatable short-window gain in straight flight. At matched damping and after compensating each series' fitted delay, circling is slightly worse than TE. As displayed, its faster response produces a stronger 10-second gain, including a small gain while circling. Robustness to corrupt sensor input and recovery from missing recorded history remain substantial weaknesses.

## Comparable-data aggregate

Ten flights have enough reference overlap for the main aggregate. Exclude May 13's severe air-data corruption and the four September/October flights with fewer than 30 complete 10-second comparison windows. Those five logs remain in the availability and robustness results below. About **21.2 hours** of eligible sample overlap remain; requiring complete windows reduces scored duration further. Selection favours periods when AirMass is working and must not be interpreted as whole-flight performance.

Reference disagreement is RMS in m/s. Main reference: GNSS energy climb using ground velocity relative to a slowly averaged wind, minus polar sink.

### Same damping, fitted delay removed

AirMass uses the TE vario's 4-second damping. Fit each series' delay independently over 0–6 seconds. This removes much of the response-speed benefit and compares their agreement at similar smoothing.

| Window / phase | Windows | AirMass | TE netto | AirMass RMS change |
|---|---:|---:|---:|---:|
| 10 s / all | 7299 | 0.757 | 0.803 | -5.6% |
| 10 s / straight | 3078 | 0.523 | 0.658 | -20.6% |
| 10 s / circling | 2598 | 0.992 | 0.975 | +1.8% |
| 30 s / all | 2260 | 0.325 | 0.311 | +4.4% |
| 30 s / straight | 808 | 0.286 | 0.305 | -6.5% |
| 30 s / circling | 788 | 0.320 | 0.289 | +10.8% |
| 300 s / all | 81 | 0.054 | 0.053 | +1.3% |

AirMass's near-zero overall 10-second bias (−0.005 m/s) hides a phase split: **−0.091 m/s straight**, **+0.083 m/s circling**. TE is −0.032 and +0.024 respectively. AirMass agrees better on 8 of the 10 individual flights over 10 seconds; one of the other two is essentially tied.

### Cockpit display, without shifting time

Use the actual displayed series: AirMass at its default half damping, TE at the logged damping. No fitted time shift. Each pair is compared over the same eligible windows within this mode. These scores also exclude TE-fallback periods. Counts differ from the delay-adjusted table because shifted windows require additional valid history.

| Window / phase | Windows | AirMass | TE netto | AirMass RMS change |
|---|---:|---:|---:|---:|---:|
| 10 s / all | 7374 | 0.841 | 0.950 | -11.5% |
| 10 s / straight | 3141 | 0.627 | 0.828 | -24.2% |
| 10 s / circling | 2603 | 1.068 | 1.102 | -3.1% |
| 30 s / all | 2282 | 0.350 | 0.359 | -2.3% |
| 30 s / straight | 824 | 0.312 | 0.358 | -12.9% |
| 30 s / circling | 790 | 0.342 | 0.328 | +4.4% |

The displayed 10-second series agrees better on all ten selected flights, but this is agreement with a reconstructed reference, not a truth-based instrument certification. Longer averages reduce the speed advantage; 30-second circling remains worse.

## Reference sensitivity

The main reference is not fully independent: it uses AirMass's slowly averaged wind and shares polar assumptions. GNSS altitude noise and wind error can dominate short windows, especially in circles. No real log supplies the true air-mass vertical velocity.

The alternative pitot-TAS energy reference gives these equal-damping, delay-adjusted results:

| Window | AirMass | TE netto |
|---|---:|---:|
| 10 s | 0.603 | 0.450 |
| 30 s | 0.275 | 0.178 |
| 300 s | 0.050 | 0.047 |

That reference favours TE because both interpret pitot speed changes as energy changes. The reversal shows that an absolute claim of accuracy would be unjustified. The main result demonstrates better short-window agreement with the gust-insensitive reference; the true air-mass velocity remains unknown.

## Per-flight scores and availability

Equal-damping, delay-adjusted RMS disagreement with the main reference. N counts complete 10-second comparison windows. Availability percentages describe eligible measured free-flight rows, not wall-clock time. † marks corruption or insufficient windows, excluded from the comparable-data aggregate.

| Log | Glider | N | 10 s AirMass / TE | 30 s AirMass / TE | Vario valid | Wind valid |
|---|---|---:|---:|---:|---:|---:|
| 2024-09-21_10-38.nmea | DG 800B/18 | 433 | 0.603 / 0.618 | 0.272 / 0.253 | 96.1% | 68.2% |
| 2025-04-16_06-02.nmea | DG 800B/18 | 698 | 0.902 / 0.991 | 0.403 / 0.387 | 99.6% | 52.1% |
| 2025-05-13_08-19.nmea † | DG 800B/18 | 789 | 15.406 / 6.580 | 2.650 / 2.632 | 95.6% | 22.8% |
| 2025-05-16_11-09.nmea | DG 800B/18 | 405 | 0.701 / 0.684 | 0.320 / 0.306 | 99.9% | 38.7% |
| 2025-07-04_10-36.nmea | Arcus | 816 | 0.843 / 0.841 | 0.337 / 0.310 | 99.9% | 33.7% |
| 2025-07-05_10-13.nmea | Arcus | 516 | 0.615 / 0.732 | 0.260 / 0.299 | 100.0% | 33.6% |
| 2025-07-08_06-46.nmea | Arcus | 1003 | 0.497 / 0.572 | 0.289 / 0.288 | 99.9% | 46.2% |
| 2025-07-10_10-04.nmea | Arcus | 1089 | 0.794 / 0.843 | 0.315 / 0.270 | 100.0% | 34.3% |
| 2025-07-11_09-24.nmea | Arcus | 986 | 0.844 / 0.866 | 0.365 / 0.341 | 100.0% | 40.2% |
| 2025-07-12_09-05.nmea | Arcus | 982 | 0.855 / 0.909 | 0.305 / 0.317 | 99.9% | 37.6% |
| 2026-03-03_12-49.nmea | DG 800B/18 | 371 | 0.597 / 0.643 | 0.326 / 0.304 | 100.0% | 51.8% |
| 2026-09-15_10-39..nmea † | Ventus CM 17.6 | 0 | — / — | — / — | 25.5% | 7.5% |
| 2026-09-17_09-11.nmea † | Ventus CM 17.6 | 0 | — / — | — / — | 22.1% | 0.0% |
| 2026-10-05_10-33.nmea † | Ventus CM 17.6 | 1 | 2.226 / 2.351 | — / — | 24.7% | 8.4% |
| 2026-10-12_10-21.nmea † | Ventus CM 17.6 | 3 | 0.261 / 0.430 | — / — | 33.1% | 16.3% |

## March flight after conversion repair

`2026-03-03_12-49.nmea` previously never aligned its heading/wind. The inconsistent FMT mapping is now repaired only when independent gravity/attitude and gyro/pitch-rate comparisons support it; Kelvin temperatures are validated against PXCV Celsius.

- 86,965 OAT samples are converted from Kelvin; no OAT fallback is needed.
- Vario is valid on 100% and wind on 51.8% of eligible free-flight rows. Heading is aligned on 99.3% of these rows, after the initial alignment phase.
- There are now 371 complete 10-second windows: **0.597 m/s AirMass versus 0.643 TE**.
- Over 30 seconds: **0.326 versus 0.304 m/s**, so TE still agrees better on that timescale.
- Independent wind check: 11 qualifying circles, **0.91 m/s median** vector disagreement.

The conversion fix removes an incorrect negative assessment of this flight; it does not change the other flight scores.

## Robustness failures remain

### May 13: extreme valid output

At 10:53:10 UTC a corrupt pressure reading produces a roughly −46 km pressure-altitude excursion and invalid TAS. The corrected converter does not yet reject this unrelated type of corruption.

The estimator still reaches **+341.46 m/s raw with vario valid at 10:55:14**. The displayed AirMass series reaches **+341.44 m/s at 10:55:19**, also marked valid. This is not just an unused internal diagnostic value. The drawn dial clips at its range, but the underlying displayed value is invalid.

The energy reference is also corrupted by GNSS altitude jumps. The 15.41/6.58 m/s full-flight 10-second RMS is therefore not a normal accuracy ranking. Excluding it from the headline score does not excuse the valid-output failure. Input sanity checks and validity containment are the highest priority.

### September/October: repeated recovery after recorded gaps

| Log | Converted sensor gaps >300 ms | Vario valid | Wind valid | Complete 10 s windows |
|---|---:|---:|---:|---:|
| 2026-09-15_10-39..nmea | 3630 | 25.5% | 7.5% | 0 |
| 2026-09-17_09-11.nmea | 6378 | 22.1% | 0.0% | 0 |
| 2026-10-05_10-33.nmea | 4846 | 24.7% | 8.4% | 1 |
| 2026-10-12_10-21.nmea | 3514 | 33.1% | 16.3% | 3 |

Pipeline fills missing sensor time with untrusted history. EKF resets vertical settling and motion trust; roughly five seconds of settling and ten seconds of uninterrupted measured history are needed for recovery. These logs frequently lose recorded samples before recovery completes. This could be a logger/transport limitation rather than an onboard acquisition outage, so replay availability must not be presented as proven onboard availability.

September 15 has one RMC solution per epoch, showing that the problem persists without September 17's conflicting same-epoch GNSS solutions. The September 17 file additionally needs consistent GNSS source selection before interpreting its reference or rejection counts.

The October 5 run also has −91.4 m/s raw output at 14:10:05 while invalid, and a later −20.94 m/s raw output at 15:02:08 while valid. This latter excursion deserves inspection against contemporaneous TE before treating it as a real air-mass change.

## Wind cross-check

Independent full-turn GNSS circle fits assume fairly steady airspeed and wind over one turn. Require fit residual below 1 m/s; compare with valid AirMass wind at the turn midpoint. These are spot checks and are unavailable on long straight segments.

| Log | Qualifying circles | Median vector difference | 90th percentile |
|---|---:|---:|---:|
| 2024-09-21_10-38.nmea | 41 | 0.960 m/s | 1.829 m/s |
| 2025-04-16_06-02.nmea | 22 | 1.375 m/s | 1.987 m/s |
| 2025-05-13_08-19.nmea | 0 | — m/s | — m/s |
| 2025-05-16_11-09.nmea | 25 | 1.134 m/s | 2.606 m/s |
| 2025-07-04_10-36.nmea | 33 | 1.344 m/s | 3.233 m/s |
| 2025-07-05_10-13.nmea | 17 | 1.476 m/s | 2.266 m/s |
| 2025-07-08_06-46.nmea | 4 | 1.627 m/s | 2.631 m/s |
| 2025-07-10_10-04.nmea | 36 | 1.298 m/s | 2.333 m/s |
| 2025-07-11_09-24.nmea | 34 | 1.125 m/s | 2.556 m/s |
| 2025-07-12_09-05.nmea | 54 | 1.432 m/s | 4.256 m/s |
| 2026-03-03_12-49.nmea | 11 | 0.907 m/s | 1.187 m/s |
| 2026-09-15_10-39..nmea | 10 | 0.739 m/s | 1.229 m/s |
| 2026-09-17_09-11.nmea | 0 | — m/s | — m/s |
| 2026-10-05_10-33.nmea | 4 | 1.159 m/s | 3.841 m/s |
| 2026-10-12_10-21.nmea | 23 | 1.051 m/s | 1.726 m/s |

On the ten comparable flights there are 277 qualifying circles. Their per-flight median vector disagreement ranges from 0.91 to 1.63 m/s. The newer gap-heavy flights can also have good wind fits when valid, but those sparse intervals do not establish whole-flight performance.

## What to improve next

1. Reject impossible pressure/airspeed/IMU inputs and prevent extreme output from retaining a valid flag. Preserve the May 13 interval as a robustness case.
2. Revisit recovery after brief sensor gaps without allowing inferred motion into GNSS fusion or learning. Test both missing recorded data and genuine acquisition interruptions.
3. Address the straight/circling bias split and 30-second circling behaviour with independent-reference sensitivity checks. Avoid tuning solely to AirMass-derived reference wind.

## Reproduce and artifacts

```sh
node --max-old-space-size=6144 tools/airmass_sim/analysis/batch.js /private/tmp/reevaluation.json ~/Downloads/NMEAlogs/*.nmea
```

- [Per-flight results, input hashes, conversion flags and module hashes](raw/nmealogs-reevaluated-2026-11-09.json)
- [Aggregate results and included-file list](raw/nmealogs-reevaluated-aggregate-2026-11-09.json)
- [Batch evaluator](batch.js)

Scores use a 10 Hz time grid and do not interpolate across sensor gaps longer than 300 ms. The free-flight mask uses the existing tow-exclusion heuristic and IAS/GS thresholds, not manually annotated tow/engine/landing phases. Pitot comparisons are restricted to the same main-reference overlap for consistency with the earlier batch. Absolute error claims and all-flight availability claims are not supported by this dataset alone.

## Measured cockpit response delay

Follow-up: correlate each displayed series with the same main GNSS reference, on eligible AirMass-valid periods. Search nonnegative delays from 0 to 6 seconds in 0.2-second increments, on the regular 10 Hz grid. These are per-flight medians over the ten comparable flights, not direct measurements of thermal-entry detection time.

| Phase | Median AirMass delay | Median TE delay | Median paired AirMass lead |
|---|---:|---:|---:|
| all | 1.6 s | 2.6 s | 1.0 s |
| straight | 1.4 s | 2.2 s | 0.8 s |
| circling | 1.8 s | 2.4 s | 0.6 s |

Across flights the overall paired lead ranges from 0 to 2.4 seconds; July 10 has no measured overall advantage. Medians do not imply identical delay for every change. When AirMass falls back to TE, there is no independent AirMass speed benefit. This compares the replay's default half damping for AirMass with the TE smoothing in the log.

| Flight | AirMass delay | TE delay | Lead |
|---|---:|---:|---:|
| 2024-09-21_10-38.nmea | 1.0 s | 3.4 s | 2.4 s |
| 2025-04-16_06-02.nmea | 1.2 s | 1.6 s | 0.4 s |
| 2025-05-16_11-09.nmea | 1.6 s | 3.0 s | 1.4 s |
| 2025-07-04_10-36.nmea | 1.6 s | 2.6 s | 1.0 s |
| 2025-07-05_10-13.nmea | 1.6 s | 2.6 s | 1.0 s |
| 2025-07-08_06-46.nmea | 1.4 s | 2.8 s | 1.4 s |
| 2025-07-10_10-04.nmea | 2.2 s | 2.2 s | 0.0 s |
| 2025-07-11_09-24.nmea | 1.6 s | 2.6 s | 1.0 s |
| 2025-07-12_09-05.nmea | 1.8 s | 2.0 s | 0.2 s |
| 2026-03-03_12-49.nmea | 1.6 s | 2.2 s | 0.6 s |

[Per-flight response-delay metrics](raw/nmealogs-response-delay-2026-11-09.json).

## March landing: low-speed state remains displayed as valid

After the landing/runout, the replay still displays AirMass while the converter waits for 30 seconds below
10 m/s IAS before emitting PXCVX. At 15:00:01 UTC, IAS is 61.5 km/h and displayed AirMass is +0.89 m/s
versus TE −1.70; at 15:00:06 it is +1.95 versus TE −1.30. The polar sink is already unavailable below the
model's flying-speed range, so energy fusion/anchoring is skipped while the kinematic flight model remains active.

At 15:00:16 IAS/TAS are zero and raw AirMass is −0.908 m/s. Once TAS falls below 10 m/s, EKF no longer calls
fuseKinematic/updateKinetic/fuseEnergy, leaving the old air-mass output essentially frozen. The vario-valid flag
still follows vertical settling alone and stays set. At 15:00:21 the log's TE vario is zero but AirMass remains
−0.908, through the last replayed sample at 15:00:41. This is a low-speed validity/ground-mode defect, not true
post-landing air movement or the TE fallback. The converter stops the replay shortly afterward; the full log
continues to 15:04:48.

Use a flying-speed/ground validity policy and TE fallback through runout and stopping, rather than displaying a
stale valid AirMass estimate. Preserve this interval when checking that policy. See [landing samples](raw/march-landing-2026-11-09.json).

Fixed afterward in the shared Pipeline/EKF: invalidate below the polar's usable IAS range, skip aerodynamic
fusion and unsupported delayed GNSS air vectors there, clear stale correction filters, and require settling
again after speed recovery. The rebuilt web replay now shows TE fallback at 15:00:01 (−1.7 m/s), 15:00:06
(−1.3), and zero from 15:00:21 through the final replay sample. The former −0.908 valid hold is gone. All 28
host tests pass, including landing, ground hold, delayed-history and recovery cases. March and July 11
flight-score spot checks remain close to the preceding results; the full-batch tables above describe the
pre-landing-guard run. See [landing samples after the fix](raw/march-landing-fixed-2026-11-09.json).
