"""The instant wind in the simulator, as wind.js on the real flights: how much the estimate and the true wind at the glider
move in straight flight and while circling, the estimate's error, and the share of it locked to the heading.
  python3 sim_wind.py out/dev_s1.csv ...   (airmass_sim nl ... --csv)"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from analyze_sens import load_sim_csv, sim_columns  # noqa: E402

HZ, HALF = 10, 300
T, WN, WE, EWN, EWE, ST, ROLL, GN, GE = sim_columns(
    "t", "true_wind_n", "true_wind_e", "wind_n", "wind_e", "status", "true_roll", "pos_n", "pos_e")


acc = {}


def add(p, key, v):
    a = acc.setdefault(p, {}).setdefault(key, [0, 0.0])
    a[0] += 1
    a[1] += v * v


heading_locked, corr = 0.0, {}
tot_err = [0, 0.0]
for path in sys.argv[1:]:
    R = load_sim_csv(path)
    n = len(R)
    ok = [int(r[ST]) & 2 and r[T] > 600 for r in R]
    series = {"estimate": (EWN, EWE), "truth": (WN, WE)}
    cum = {}
    for name, (cn, ce) in series.items():
        sn, se, sc = [0.0], [0.0], [0]
        for k in range(n):
            f = ok[k]
            sn.append(sn[-1] + (R[k][cn] if f else 0))
            se.append(se[-1] + (R[k][ce] if f else 0))
            sc.append(sc[-1] + (1 if f else 0))
        cum[name] = (sn, se, sc)
    bins = [[0, 0.0, 0.0] for _ in range(12)]
    for k in range(n):
        if not ok[k]:
            continue
        bank = abs(R[k][ROLL])
        ph = "straight" if bank < 10 else "circling" if bank > 25 else None
        if not ph:
            continue
        a, b = max(0, k - HALF), min(n, k + HALF)
        devs = {}
        for name, (cn, ce) in series.items():
            sn, se, sc = cum[name]
            m = sc[b] - sc[a]
            if m < HALF:
                break
            devs[name] = (R[k][cn] - (sn[b] - sn[a]) / m, R[k][ce] - (se[b] - se[a]) / m)
            add(ph, name + " dev", math.hypot(*devs[name]))
            for lag, lab in ((10 * HZ, "10 s"), (2 * HZ, "2 s")):
                if k + lag < n and ok[k + lag]:
                    add(ph, f"{name} moves {lab}", math.hypot(R[k + lag][cn] - R[k][cn], R[k + lag][ce] - R[k][ce]))
        if len(devs) == 2:
            # how much of the true wind's movement the estimate follows: sums for the correlation of the deviations
            c = corr.setdefault(ph, [0.0, 0.0, 0.0])
            (xn, xe), (yn, ye) = devs["estimate"], devs["truth"]
            c[0] += xn * yn + xe * ye
            c[1] += xn * xn + xe * xe
            c[2] += yn * yn + ye * ye
            # the error of the estimate's 60 s mean against the instant truth
            sn, se, sc = cum["estimate"]
            m = sc[b] - sc[a]
            add(ph, "error of the 60 s mean", math.hypot((sn[b] - sn[a]) / m - R[k][WN], (se[b] - se[a]) / m - R[k][WE]))
        en, ee = R[k][EWN] - R[k][WN], R[k][EWE] - R[k][WE]
        add(ph, "error", math.hypot(en, ee))
        if k + 2 * HZ < n and ok[k + 2 * HZ]:
            e2n, e2e = R[k + 2 * HZ][EWN] - R[k + 2 * HZ][WN], R[k + 2 * HZ][EWE] - R[k + 2 * HZ][WE]
            add(ph, "error moves 2 s", math.hypot(e2n - en, e2e - ee))
        if ph == "circling":
            psi = math.atan2(R[k][GE] - R[k][WE], R[k][GN] - R[k][WN])  # the air velocity's direction
            j = int(((math.degrees(psi) % 360) + 360) % 360 // 30)
            bins[j][0] += 1
            bins[j][1] += en
            bins[j][2] += ee
            tot_err[0] += 1
            tot_err[1] += en * en + ee * ee
    for c, sn_, se_ in bins:
        if c:
            heading_locked += (sn_ * sn_ + se_ * se_) / c

rms = lambda p, key: math.sqrt(acc[p][key][1] / acc[p][key][0])
print(f"{len(sys.argv) - 1} flights; instant wind, m/s rms")
print("phase       estimate: dev  moves 10 s  moves 2 s   truth: dev  moves 10 s  moves 2 s   error  error moves 2 s")
for p in ("straight", "circling"):
    print(f"{p:<10}  {rms(p, 'estimate dev'):13.2f} {rms(p, 'estimate moves 10 s'):11.2f} {rms(p, 'estimate moves 2 s'):10.2f}"
          f"  {rms(p, 'truth dev'):11.2f} {rms(p, 'truth moves 10 s'):11.2f} {rms(p, 'truth moves 2 s'):10.2f}"
          f"  {rms(p, 'error'):6.2f} {rms(p, 'error moves 2 s'):10.2f}")
for p in ("straight", "circling"):
    c = corr[p]
    print(f"{p}: correlation of the estimate's and the truth's deviations from their 60 s means {c[0] / math.sqrt(c[1] * c[2]):.2f}; "
          f"error against the instant truth: instant estimate {rms(p, 'error'):.2f}, its 60 s mean {rms(p, 'error of the 60 s mean'):.2f}")
e = heading_locked
print(f"circling: share of the error's variance locked to the heading (30° bins per flight) {100 * e / tot_err[1]:.0f} % "
      f"(rms {math.sqrt(e / tot_err[0]):.2f} m/s)")
