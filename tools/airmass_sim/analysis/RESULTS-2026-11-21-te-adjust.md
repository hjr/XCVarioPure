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
  of its tubes (AirMass models it, eComp does not) is a candidate, not verified. The half rule still costs only 1.2 %
  there, against 13 % for the setting itself.
- k_tas is not a clean measure of the pitot alone: the ASG 29 learns 1.01–1.03 where its static port (AirMass 95–96)
  would give 0.97, so a rule from k_tas instead of the setting does not work.

Implemented on 2026-11-21: `Pipeline::configure` sets static_comp = 0.5 + adjust/200, and the simulator puts the port
error into the dynamic pressure too (`--static-adj` stays the eComp setting that cancels it, ε = (adj − 1)/(adj + 1);
the expected k_tas is K.k_tas/√(1 − ε)). All 46 host tests pass; k_tas converges to the new expected values
(pullup --static-adj 108: 1.070, err −0.001), the AirMass vario is unchanged, and `pullup --static-adj 92 --comp 100`,
which failed before, passes (a wrong setting now costs AirMass half). `te_adjust.js` writes 2·value − 100 into the
setup, so its AirMass values stay AirMass's own static_comp.
