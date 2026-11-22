# The TE compensation setting for eComp and AirMass (2026-11-21)

`te_comp_adjust` (default 92 %) does two different things. eComp scales the whole kinetic energy term with it
(the mean of h + adjust·TAS²/2g and the ISA altitude of p − adjust·q). AirMass takes it as `static_comp` and only
corrects the static altitude by (adjust − 1)·TAS²/2g (`AirMassEKF.cpp`); it learns the airspeed scale (`k_tas`) itself.
A static port reading ε·q off also puts −ε·q into the measured dynamic pressure (total − static), so eComp needs
about 1 + 2ε and AirMass about 1 + ε: half the deviation from 100. The simulator's `--static-adj` puts the error on
the static altitude only, so there the two optima coincide (and both are 100 with no port error, the tuning point).
The real-flight replays always ran AirMass at 100 (the converter's setup), while devices run it at the setting, 92 by
default.

`te_adjust.js` on the 37 logs (raw/te-adjust-2026-11-21.{json,txt}): AirMass replayed with 100, 96 and 90 in its
setup, eComp at 84–104, all through `Damp(4)` against the gust-insensitive reference (from the 100 replay for all;
reference samples beyond ±20 m/s, a GNSS altitude glitch on 2025-07-04, left out). "Speed change": 10 s windows
where TAS²/2g changes by more than 15 m, where the compensation shows. The wind's validity is the same at all three
AirMass values on every flight.

Pooled over 28 flights, 10 s rms m/s:

| | all | straight | circling | speed change | false climbs 2 m/s /h |
|---|---|---|---|---|---|
| AirMass 100 | 0.518 | 0.418 | 0.656 | 0.601 | 13.6 |
| AirMass 96 | **0.514** | **0.413** | 0.652 | **0.589** | 12.6 |
| AirMass 90 | 0.516 | 0.422 | **0.647** | 0.603 | **12.0** |
| eComp 84 | 0.601 | 0.567 | **0.644** | 0.756 | **32.6** |
| eComp 88 | 0.598 | 0.562 | 0.645 | 0.744 | 33.7 |
| eComp 90 | **0.598** | **0.561** | 0.646 | **0.743** | 34.4 |
| eComp 92 | 0.599 | 0.562 | 0.647 | 0.745 | 35.3 |
| eComp 96 | 0.604 | 0.569 | 0.652 | 0.760 | 37.5 |
| eComp 100 | 0.614 | 0.582 | 0.658 | 0.788 | 40.3 |
| eComp 104 | 0.627 | 0.600 | 0.666 | 0.827 | 42.6 |

Per glider, 10 s rms in speed-change windows:

| glider (flights) | eComp 84 / 88 / 90 / 92 / 96 / 100 | best eComp | AirMass 100 / 96 / 90 | best AirMass |
|---|---|---|---|---|
| ASG 29/18 (14) | 0.690 / 0.680 / 0.679 / 0.681 / 0.694 / 0.718 | 88–90 | 0.565 / 0.553 / 0.564 | 96 |
| DG 800B/18 (5) | 0.896 / 0.941 / 0.969 / 1.001 / 1.072 / 1.152 | ≤ 84 | 0.776 / 0.738 / 0.722 | ≤ 90 |
| Arcus (9) | 0.792 / 0.757 / 0.744 / 0.735 / 0.729 / 0.737 | 96 | 0.587 / 0.583 / 0.614 | 96–100 |

- The setting belongs to the installation (the static port): the DG wants less than 84, the ASG 29 about 89, the
  Arcus about 96. 92 is a reasonable average for eComp; 100 is wrong for all three.
- AirMass's optimum is about halfway between eComp's and 100 on every glider, as the static port model says: ASG 29
  eComp ~89 / AirMass 96, Arcus ~96 / 96–100, DG ≤ 84 / ≤ 90.
- 90 for both: eComp 90 is the pooled best (0.743 in speed changes, 92 gives 0.745). AirMass 90 pooled is about the
  same as 100 (speed change 0.603 vs 0.601, circling 0.647 vs 0.656, false climbs 12.0 vs 13.6), but it helps the DG
  (0.776 → 0.722), does nothing for the ASG 29 and costs the Arcus (0.587 → 0.614). AirMass at 92, what a device
  runs by default, lies between 96 and 90.
- One setting can serve both if AirMass takes half its deviation: static_comp = 1 + (adjust − 100) / 200, so the
  default 92 gives AirMass 96, the pooled best in every column but circling. The simulator would then need the port
  error in the dynamic pressure too, or its `--comp` checks no longer match.

## The November 2026 Arcus flights

2026-11-18 (ridge, joined log), 2026-11-19 and 2026-11-21, finer sweep (raw/te-adjust-2026-11-21-arcus-nov.{json,txt},
`AIRMASS=100,104,98,96,94,92,90,88,84 ECOMP=80,84,86,88,90,92,94,96,98,100,104`). The device ran all three at 90.1 %
(`$PXCVC`). 10 s rms in speed-change windows (their number in brackets), m/s:

| | 84 | 88 | 90 | 92 | 94 | 96 | 98 | 100 | 104 | best |
|---|---|---|---|---|---|---|---|---|---|---|
| eComp 11-18 (208) | 0.850 | 0.844 | **0.844** | 0.847 | 0.851 | 0.857 | 0.866 | 0.876 | 0.901 | 88–90 |
| eComp 11-19 (311) | 0.775 | 0.761 | **0.759** | 0.759 | 0.763 | 0.770 | 0.781 | 0.794 | 0.828 | 90–92 |
| eComp 11-21 (466) | 0.852 | 0.837 | **0.834** | 0.834 | 0.838 | 0.844 | 0.854 | 0.867 | 0.901 | 90–92 |
| eComp, 3 flights (985) | 0.828 | 0.815 | **0.813** | 0.814 | 0.818 | 0.824 | 0.834 | 0.846 | 0.879 | 90 |
| AirMass 11-18 | 0.682 | 0.666 | 0.662 | **0.661** | 0.663 | 0.667 | 0.674 | 0.685 | 0.712 | 92 |
| AirMass 11-19 | 0.689 | 0.641 | 0.623 | 0.608 | 0.598 | 0.591 | **0.589** | 0.592 | 0.609 | 98 |
| AirMass 11-21 | 0.760 | 0.719 | 0.704 | 0.692 | 0.684 | **0.680** | 0.680 | 0.685 | 0.711 | 96–98 |
| AirMass, 3 flights | 0.722 | 0.684 | 0.670 | 0.660 | 0.653 | **0.650** | 0.652 | 0.657 | 0.681 | 96 |

All windows, 3 flights: AirMass 0.566 at 96, 0.572 at 90, 0.569 at 100; eComp 0.697 at 90 and 92, 0.711 at 100.
eComp is best at 90 on every flight (92 within 0.001), AirMass at 96–98 on two and 92 on the ridge flight. At the
device's 90 AirMass loses 3 % in speed changes against 96 (0.670 vs 0.650) and is worse than at 100. The half rule
gives 95 for eComp's 90. This Arcus wants less than the 2025 Arcus flights (eComp ~96): another installation or
another static port. False climbs fall steadily with a lower value for both (AirMass 27.9 per h at 100, 23.0 at 90,
21.9 at 84): under-compensation also hides some real climbs, so the speed-change rms, which counts both, is the
measure to set it by.

## The half rule over all flights

Why: a static port reading ε·q off (ε < 0: suction) puts −ε·q/(ρg) into the altitude and the same error into the
measured dynamic pressure, q_m = (1 − ε)·q. eComp corrects both: adjust = (1 + ε)/(1 − ε). AirMass corrects only the
altitude, (static_comp − 1)·q_m/(ρg) = ε·q/(ρg), so static_comp = 1/(1 − ε); the airspeed scale it learns (k_tas)
takes the airspeed part. Together: static_comp = (1 + adjust)/2, exactly, if the pitot is right and k_tas has
converged.

Fine sweep of the 29 clean flights (raw/te-adjust-2026-11-21-fine.json, AirMass 80–100 and eComp 76–104 in steps of
2–4), evaluated per installation by `half_rule.js` (raw/half-rule-2026-11-21.md): the setting is eComp's best, AirMass
gets the half rule's value, its own best, the setting itself (the firmware now) or 100 (the replays' tuning). 10 s rms
in speed-change windows, AirMass against its own best:

| installation (flights) | eComp best | half rule | AirMass at it | AirMass best | at the setting | at 100 |
|---|---|---|---|---|---|---|
| ASG 29/18 (14) | 90 | 95 | 0.552 (+0.0 %) | 96 (0.552) | 0.563 (+2.0 %) | 0.563 (+2.0 %) |
| Arcus 2025-07 (5) | 96 | 98 | 0.585 (+0.4 %) | 96 (0.583) | 0.583 (+0.0 %) | 0.592 (+1.5 %) |
| Arcus 2026-11 (3) | 90 | 95 | 0.652 (+0.2 %) | 96 (0.651) | 0.670 (+3.0 %) | 0.657 (+0.9 %) |
| DG 800B/18 (5) | ≤ 76 | 88 | 0.693 (+1.2 %) | 90–92 (0.685) | 0.776 (+13.4 %) | 0.739 (+7.9 %) |
| Arcus 2026-05 (2) | 100 | 100 | 0.486 (+0.0 %) | 100 (0.486) | 0.486 (+0.0 %) | 0.486 (+0.0 %) |
| all, each at its own setting | | | 0.587 (+0.3 %) | 0.586 | 0.606 (+3.5 %) | 0.601 (+2.6 %) |
| all at the default 92 | | 96 | 0.589 | 96 (0.589) | 0.595 | 0.601 |

All windows: half rule +0.1 %, the setting itself +0.9 %, 100 +1.1 % over the optimum.

- The half rule is within 0.4 % of AirMass's own optimum everywhere but the DG, and a pilot's eComp setting then sets
  AirMass right too. Using the setting directly, as the firmware does, costs 3.5 % overall and 13 % on the DG.
- The DG is the exception: eComp still improves below 76 (the edge of the sweep), while AirMass wants 90–92, which
  the half rule maps to an eComp of about 82. Its learned k_tas (0.91–0.96) fits a suction static port of
  ε ≈ −0.09, as AirMass's optimum says, so eComp there corrects for something beyond the static port; pneumatic lag
  of its tubes (AirMass models it, eComp does not) is a candidate, not verified (see below: it is the cause). The half rule still costs only 1.2 %
  there, against 13 % for the setting itself.
- k_tas is not a clean measure of the pitot alone: the ASG 29 learns 1.01–1.03 where its static port (AirMass 95–96)
  would give 0.97, so a rule from k_tas instead of the setting does not work.

Implemented on 2026-11-21: `Pipeline::configure` sets static_comp = 0.5 + adjust/200, and the simulator puts the port
error into the dynamic pressure too (`--static-adj` stays the eComp setting that cancels it, ε = (adj − 1)/(adj + 1);
the expected k_tas is K.k_tas/√(1 − ε)). All 46 host tests pass; k_tas converges to the new expected values
(pullup --static-adj 108: 1.070, err −0.001), the AirMass vario is unchanged, and `pullup --static-adj 92 --comp 100`,
which failed before, passes (a wrong setting now costs AirMass half). `te_adjust.js` writes 2·value − 100 into the
setup, so its AirMass values stay AirMass's own static_comp.

## The DG's pitot lag (added the same day)

The airspeed's time shift against the static altitude that makes h + adj·V²/2g smoothest (scratch script, every
flying sample of each log): the DG 800's airspeed is late by 0.6–0.9 s on all five flights (0.8, 0.7, 0.6, 0.9, 0.9);
the ASG 29 and both Arcus installations 0.0–0.1 s. Whether its tubing or its airspeed filtering, the logs do not tell.
eComp combines the two directly, so on the DG every speed exchange leaves a derivative-shaped error: its optimum drops
to ≤ 76 (less of the late term), and its fitted circling delay comes out early (2.4 s against TE pressure's 3.4 s; at
92 and 100 1.6 s, while its circling 10 s rms rises 0.840 → 0.851 → 0.864). The AirMass needle's circling lead over
eComp on the DG (0.8 s) is short for that reason; over the TE probe it is 1.8 s. AirMass assumes 0.2 s on both lines;
the DG's spread of learned GNSS latencies (422–790 ms) may absorb part of the unmodelled pitot lag, not verified.

## Learned in flight (2026-11-22)

`AirMassTeLearn.h` in the Pipeline: over 10 s windows the change of eComp's energy height (static altitude +
setting·V²/2g) minus the change of the GNSS energy height (GNSS altitude, `$PXCVE`, and the ground velocity of the same
fix against the averaged wind), regressed on the change of V²/2g. Carried over flights (weight and setting in NVS,
forgotten over 20 h), the eAdjustment set from it at power-up on the ground after 1.5 h of windows. What made it work,
each found on the real flights or the simulator:

- the GNSS velocity lags the altitude of its fix by 0–0.64 s (the page's reference measures it, 0.3 s typical): the
  altitude is paired with the velocity interpolated 0.3 s towards the next fix;
- the GNSS altitude's age at arrival differs per receiver (0.2–1.05 s): learned like the page's `gnssAltDelay` (2 s
  changes against the static altitude). With a wrong age, windows where speed and climb change together (thermal
  entries, dolphin flight) biased the setting 7–13 % in the simulator;
- a horizontal gust moves the measured V²/2g but neither altitude: plain window ends read the setting 2–7 % low on real
  flights, means over ±2.5 s around them fixed it. An instrument (the GNSS kinetic height) did not: its noise is in the
  GNSS energy height too, the setting came out 15–25 % high;
- windows more than 8 m off what the setting learned so far expects are left out after 15 min: in double turbulence
  (simulator) a few of them took 3 of 8 seeds to ~68 % against ~114 %.

Real flights, each replayed at its installation's tuned setting from a fresh learner (`telearn.js`, scratch), carried
over in date order like the device:

| installation | after the 1st flight | carried over all | sweep optimum |
|---|---|---|---|
| DG 800B/18 (5) | 77–80 | 75.0 | 75 |
| Arcus 2025-07 (5) | 93–99 | 95.6 | 95 |
| Arcus 2026-05 (2) | 99 | 99.0 | 99 |
| ASG 29/18 (14) | 92–93 | 90.0 | 88 |
| Arcus 2026-11 (3) | 85–88 | 91.4 | 90 |

Simulator (`nl`, one hour, 8 seeds): learned 108–119 against `static_adj·k_tas²` = 110.2 (the formula leaves out the
lines' lags), 95–98 against 90, ~113 against 110; double turbulence 90–119. Tests `nl_te_learn*` (±8 %). The vario of a
flight is not changed (the setting moves at the next power-up); the GNSS altitude ages learned on the real flights:
ASG 29 0.65 s, DG 0.8–1.0 s, Arcus 0.0–0.1 s (the page's reference adds 0.2 s to its fit).

## Applied in flight (2026-11-22)

The learned eAdjustment was set only at power-up on the ground. It is now also set in flight, every 5 minutes when it
moved by 1 % or more. eComp re-anchors its TE altitude on a change (one 0.1 s sample without a rate). AirMass did not:
its static compensation (half the setting's deviation) steps the compensated altitude by Δ·V²/2g, and the estimator
read the step as climb or sink. Simulator (`--comp-change`, the same seed with and without the change), shown vario
before the fix:

| | per 1 % of eAdjustment, peak | after |
|---|---|---|
| `nl` cruise, 96 km/h | 0.07 m/s | 1.5 s, gone in ~20 s |
| `nl` circling, 79 km/h | 0.04 m/s | 2 s |
| scaled with V², 150 / 200 km/h | ~0.17 / ~0.30 m/s | |

Since then the estimator's altitude moves with the step (`AirMassEKF::setConfig`): mean difference in the first 10 s
after a change of 2–18 % at most 0.006 m/s (0.05–0.33 before); `straight` with 100 → 80 % at 97 km/h: largest shown
error over the next 20 s 0.15 m/s, the same as without a change, against 1.37 m/s before (`straight_comp_change`).

The learner does not depend on the setting in use (its regression subtracts it again): `endurance` (3 h) from 85, 100
or 115 % learns 109.8 against 110.2 and sets the eAdjustment once, to 110.3 (`endurance_te_apply`).

## When to apply: the learner at hours of windows (2026-11-22)

The browser replay's rows carry the learner's state (`te_setting`, `te_weight`, `te_hours`). On the 29 clean real
flights, error of the learned setting against the sweep optimum [%] (scratch `checkpoints.js`; carried over an
installation's flights in date order with the device's forgetting):

| hours of windows | each flight fresh: median / max | carried: median / max |
|---|---|---|
| 0.5 | 5.2 / 17.8 | 8.4 / 17.2 |
| 1 | 4.5 / 11.2 | 9.2 / 11.2 |
| 1.5 | 3.6 / 9.3 | 2.5 / 9.3 |
| 2 | 4.1 / 7.1 | 3.4 / 7.1 |
| 3 | 3.7 / 6.2 | 2.3 / 4.1 |
| 5 | 1.7 / 2.0 (3 flights) | 1.4 / 4.9 |

The default 92: median 4.0, max 17 over the five installations. Carried, per installation, at 1.5 / 3 / 5 h and the
end: DG (75) 77.5 / 77.3 / 72.7 / 75.1; Arcus 2025-07 (95) 96.9 / 96.1 / 93.6 / 96.0; Arcus 2026-05 (99) 97.0 / 98.5 /
– / 99.0; ASG 29 (88) 92.9 / 92.1 / 92.9 / 88.9; Arcus 2026-11 (90) 80.7 / 86.8 / 90.9 / 91.7. Within one flight the
error levels off at about 4 %; it shrinks over flights. At 1.5 h the November Arcus (first flight on the ridge) would
get 80.7 against the default's 92, 9 % off instead of 2; at 3 h no installation is more than 1 % worse off than at the
default. A flight gives 0.4–0.9 h of windows per hour, 2.6 of 6.1 h on the ridge.

Why the November Arcus reads low (the learner's windows printed from a debug build, grouped, scratch `winsplit.py`): on
the ridge flight 2026-11-18 the windows below 1000 m read 69–80, above 1000 m 85–87; on 2026-11-21 every altitude
band reads 91–94. Bank, speed, turns, wind strength and the flight's airspeed scale (k_tas 0.99–1.01, temperature
logged) make no difference. AirMass's own sweep of that flight agrees with the learner (best at static_comp 92, the
half rule's 84); eComp's is flat there (84 at 0.850 m/s, 88–90 at 0.844). A local error near the ridge (the flow's
pressure field over the terrain, or the wind there against the averaged wind) the learner cannot tell from the
installation's: only other days dilute it.

The wind for the GNSS kinetic height, the 29 clean flights each from a fresh learner (|error| against the sweep
optimum per flight: median / mean / max; carried over flights at 1.5 and 3 h: max), with the 11-18 ridge flight:

| wind | per flight | carried max at 1.5 / 3 h | 2026-11-18 |
|---|---|---|---|
| averaged (`WindAverage`, as in use) | 3.6 / 3.2 / 7.8 | 9.3 / 4.1 | 83.6 |
| instant (the estimator's) | 3.3 / 3.8 / 10.0 | 13.2 / 9.4 | 80.0 |
| instant, 30 s low pass | 3.0 / 3.1 / 11.5 | 15.8 / 6.6 | 78.5 |
| instant, 10 s low pass | 2.5 / 3.3 / 14.1 | 18.2 / 9.1 | 75.9 |

The faster the wind follows, the lower the ridge flight reads: not a local wind the average misses. The averaged wind
stays: the smallest worst cases, per flight and carried.

## Two flights before applying (2026-11-22)

The learned setting is applied from 1.5 h of windows and two flights on, a flight counting once it gave FLIGHT_H of
windows in one power-up (`TeLearn::State::flights`, stored as `AM_TEFL`, on the status page). The first value applied
on the real flights (scratch `checkpoints.js`, the device's learner carried over flights), off by, against the sweep
optimum:

| a flight counts after | DG (default off by 17) | Arcus 2025-07 (3) | Arcus 2026-05 (7) | ASG 29 (4) | Arcus 2026-11 (2) |
|---|---|---|---|---|---|
| 0.25 h | 2.9 | 0.1 | 0.2 | 4.0 | 4.2 |
| 0.5 h | 3.2 | 0.5 | 0.3 | 4.0 | 3.0 |
| **1 h** (in use) | 3.0 | 0.6 | 0.9 | 4.6 | 1.1 |
| 1.5 h | 2.5 (from flight 3) | 1.6 | 1.0 | 4.6 | 0.7 |

After a quarter of an hour the second flight barely moves the first one's setting (the November Arcus at 85.8 after
its ridge flight). The ASG's first flights all read 90–94: no rule applies it closer than ~4 before its later flights.
Tests: `endurance_te_apply` (one flight before, applied), `endurance_te_one_flight` (none before, not applied).

## Recommended, not applied (2026-11-22)

The device no longer sets the eAdjustment: a few % off costs eComp little (November Arcus, speed-change rms: 4 % off
+0.6 %, 10 % off +4 %), the learned value is 1–5 off when first available, and setting it automatically overrode the
pilot's setting and learned from Bluetooth replays. The eAdjustment's help in the TE Compensation menu ends with
"Recommended value for this glider: N%." once the flight rule above allows it. The estimator's altitude step on a
change of the setting stays: a pilot changing it in flight.
