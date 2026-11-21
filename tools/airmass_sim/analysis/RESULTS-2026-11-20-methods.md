# All vario methods on the real flights (2026-11-20)

`te_pressure.js` on the 37 logs of `~/Downloads/NMEAlogs` (raw/methods-2026-11-20.{json,txt}). All series replayed
from the `$SENS` data, none taken from the log:

- AirMass: the replay's AirMass netto.
- eComp: XCVarioPure `VarioFilter::postProcess` with electronic compensation, adjust 92 %: the mean of the ISA
  altitude plus TAS²/2g and the ISA altitude of the static minus the dynamic pressure, through the TEK altitude
  filter (`TeVario`).
- TE pressure: the TE probe pressure through the same filter (`TeVario`).
- IMU TEK: XCVarioProModules' eComp with `ImuPressurePredictor` (`ImuTek`, adjust 92 %).
- TEK0: `ImuTek` without its IMU term, ProModules' plain eComp, as the control for eComp.

The vario logged in `$PXCV` is not scored: it is whatever method the device showed. All undamped series through the
page's `Damp(4)` (not ProModules' single 4 s low pass), each shifted by its fitted delay. Left out: the corrupt
2025-05-13/18, the gap-heavy Ventus logs, 2026-11-13_11-34 (nothing scorable) and 2025-07-12_09-05, where every method
scores 5.0 m/s (the reference is broken on that log; it scored 0.81 on 11-11, cause not investigated).

On 2026-11-18 and 2026-11-19 (Arcus) the TE probe is faulty: TE pressure 1.13 / 1.38 at 10 s, eComp 0.70 / 0.65,
AirMass 0.55 / 0.52. The table pools 26 flights, 87.1 h, without these two (raw/methods-2026-11-20-combined.md, made by
`te_pressure.js` from the saved run); gust-insensitive reference, best of each row in bold, TEK0 not ranked. Own error
by the three-cornered hat with AirMass (AirMass: the range over its hats with each other vario, ≈0 a negative
variance). False climbs as batch.js counts them: straight flight (bank under 10° for 8 s), the needle as shown, not
shifted, above the reference's centred 2 s mean, counted again after it came back within half the threshold; false
sinks the same below it.

| | AirMass | eComp | TE pressure | IMU TEK | TEK0 (control) |
|---|---|---|---|---|---|
| rms 10 s all, m/s | **0.548** | 0.621 | 0.621 | 0.648 | 0.645 |
| rms 10 s straight, m/s | **0.480** | 0.607 | 0.607 | 0.640 | 0.639 |
| rms 10 s circling, m/s | 0.659 | **0.650** | 0.650 | 0.669 | 0.661 |
| rms 30 s all, m/s | **0.252** | 0.271 | 0.264 | 0.284 | 0.284 |
| rms 30 s straight, m/s | **0.248** | 0.288 | 0.278 | 0.305 | 0.305 |
| rms 30 s circling, m/s | 0.216 | 0.206 | **0.203** | 0.213 | 0.212 |
| own error 10 s all (hat), m/s | **0.125–0.178** | 0.323 | 0.318 | 0.390 | 0.381 |
| own error 10 s straight (hat), m/s | **≈0–0.105** | 0.367 | 0.364 | 0.436 | 0.430 |
| own error 10 s circling (hat), m/s | 0.228–0.263 | 0.209 | **0.202** | 0.289 | 0.265 |
| median delay, s | **2.4** | 3.0 | 3.2 | 2.8 | 2.6 |
| median needle movement in 1 s, m/s | **0.485** | 0.677 | 0.565 | 1.338 | 1.328 |
| false climbs 2 m/s 2 s, per h straight | **12.8** | 33.6 | 35.0 | 40.5 | 38.6 |
| false sinks 2 m/s 2 s, per h straight | **16.0** | 32.4 | 34.7 | 44.5 | 42.2 |
| false climbs 1 m/s 1 s, per h straight | **122.5** | 175.8 | 164.1 | 252.0 | 251.2 |
| false sinks 1 m/s 1 s, per h straight | **120.5** | 165.8 | 155.6 | 266.6 | 259.7 |
| flights best at 10 s | **22** | 0 | 3 | 1 | 0 |

With the two Arcus flights (28 flights, 93.7 h, raw/methods-2026-11-20-combined-all28.md): rms 10 s AirMass 0.547,
eComp 0.625, TE pressure 0.683, IMU TEK 0.650, TEK0 0.647; false climbs 2 m/s 13.2 / 35.5 / 36.9 / 42.3 / 40.5 per h;
AirMass best at 10 s on 24 of 28 flights.

AirMass has the fewest false climbs of 2 m/s on 23 of 26 flights; tied on 2026-09-02_09-52 (with TEK0) and 2026-09-05_16-16 (none at
all), and behind TEK0 only on 2026-09-02_10-15 (0.1 h). False climbs and sinks come at about
the same rate for every method: they are noise around the truth (the method's and the reference's), not a lift bias.
AirMass's 13 per hour bound what the reference contributes, so a pressure TE shows about 20 per hour more false climbs
of 2 m/s. The IMU term adds about 2 per hour (TEK 40.5 vs TEK0 38.6). Against the pitot reference, which moves with
the pitot's gusts like the pressure TEs, the order is mixed (AirMass 55, eComp 59, TE pressure 77, IMU TEK 39).

- AirMass is best overall and in straight flight; when circling the three TE methods are level with it (TE pressure
  own 0.20, eComp 0.21, AirMass 0.23).
- eComp and TE pressure score the same (0.621 at 10 s); eComp has a busier needle (0.68 vs 0.57) and does not suffer
  from a faulty TE probe.
- The control does not match: TEK0 (ProModules' eComp) differs from eComp by 0.16 m/s rms at 10 s and scores worse
  on 26 of 28 flights (0.645 vs 0.621, own 0.38 vs 0.32), with twice the needle movement (1.35 vs 0.68). The same
  energy equation through the two pressure Kalman filters passes much more noise than XCVarioPure's altitude filter.
- The IMU term does not help: IMU TEK and TEK0 differ by 0.09 m/s rms, TEK0 scores equal or better everywhere
  (circling own 0.27 vs 0.29). It matches the sim's pull-up, where it doubled the error, because only the dynamic
  pressure filter gets the IMU's lead.
- The pitot reference shares the pitot's gust response with every pressure TE, so it favours them; the
  gust-insensitive reference is the one to rank by, as before.

## Why TEK0 scores worse than eComp

Seven flights (4 ASG 29, 2 Arcus 2025-07, DG 2025-06-09), scratch variants of the two (not kept), each through
`Damp(4)`, gust-insensitive reference, 10 s rms / 1 s needle movement / delay:

| variant | rms 10 s | needle 1 s | delay, s |
|---|---|---|---|
| eComp (XCVarioPure: energy altitude through `TeVario`) | 0.572 | 0.66 | 3.2 |
| TEK0 (ProModules: static and dynamic `PressureKF`) | 0.594 | 1.30 | 2.7 |
| TEK0 scaled by the actual temperature, T/T0 | 0.585 | 1.29 | 2.7 |
| TEK0's energy formula through `TeVario` | 0.573 | 0.69 | 3.2 |
| eComp only method 1 (TAS²/2g) / only method 2 (p − q) | 0.573 / 0.571 | 0.67 / 0.65 | 3.2 / 3.1 |
| TEK0, process noise q / 3, / 10, / 30 | 0.592 / 0.590 / 0.591 | 1.24 / 1.13 / 1.00 | 2.9 / 3.2 / 3.6 |
| TEK0 + 0.5 s low pass | 0.588 | 0.98 | 3.1 |
| TEK0, both KFs tuned alike (q 30, σ 3.5 / q 50, σ 1.5) | 0.602 / 0.604 | 1.08 / 1.28 | 2.6 / 2.3 |
| one `PressureKF` on p − q·adjust | 0.601 | 1.09 | 2.6 |
| `TeVario` with a fixed gain 0.2 / 0.1 / 0.05 instead of the adaptive one | 0.583 / 0.575 / 0.562 | 1.03 / 0.76 / 0.50 | 2.5 / 3.0 / 3.7 |

- The energy formula is not it: TEK0's formula through XCVarioPure's filter scores like eComp (0.573), and either
  half of eComp's two-method mean alone does too.
- The temperature is about 0.009 of the 0.022 gap: ProModules converts with T0 = 15 °C throughout
  (`updateP2MPS()` without a temperature, ρ = p / (R·T0)), so its whole TE reads T0/T too large, 3–8 % in the cold
  air at altitude.
- The rest, about 0.013, is the filter: the constant-velocity `PressureKF` estimates the rate as a state, which is
  about 0.5 s faster but twice as noisy, and the 4 s damping after it does not take that noise out. At the same delay
  (q / 10, 3.2 s) it is still 0.015 worse than `TeVario` with a fixed gain (0.1, 3.0 s), so it is not only the
  tuning; a plausible cause, not measured, is the second-order filter's response peaking near its cut-off. Matched
  tunings or one KF on the energy pressure do not help, so it is not the separate static and dynamic filters either,
  and `TeVario`'s adaptive gain is not needed for the advantage.
