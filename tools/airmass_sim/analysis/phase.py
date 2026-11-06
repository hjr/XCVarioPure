"""Where around the circle does the IMU acceleration disagree with the airspeed change? On the real flights (native
replays, exp/base): d = k_tas * dTAS/dt - acc_long (the inertial acceleration along the air velocity AirMass uses), both
as 1 s means, its slow part (30 s mean) removed so only what changes around the circle is left. Averaged by the
heading relative to the wind, by the heading itself, and for left and right circles apart; the first harmonic says how
large the circle-periodic part is and which angle it follows. Also AirMass's pitch against the device AHRS pitch."""
import csv, glob, math, os, sys
import numpy as np

base = sys.argv[1]
NB = 12
acc = {}  # (binning, side) -> [sum per bin, count per bin] per flight


def load(stem):
    out = list(csv.DictReader(open(stem + ".out.csv")))
    aux = {int(r["t_ms"]) // 100: r for r in csv.DictReader(open(stem + ".aux.csv"))}
    rows = []
    for r in out:
        a = aux.get(int(round(float(r["t_s"]) * 10)))
        if a is None:
            continue
        try:
            rows.append((float(r["t_s"]), int(r["status"]), float(r["heading_deg"]), float(r["k_tas"]), float(r["avg_wind_n"]),
                         float(r["avg_wind_e"]), float(r["roll_deg"]), float(r["pitch_deg"]), float(r["acc_long"]), float(a["tas"]),
                         float(a["pitch_deg"]) if a["pitch_deg"] else math.nan, float(r["wind_n"]), float(r["wind_e"])))
        except ValueError:
            pass
    return np.array(rows)


def movmean(x, n):
    c = np.cumsum(np.insert(np.nan_to_num(x), 0, 0.0))
    m = np.convolve(np.isfinite(x).astype(float), np.ones(n), mode="same")
    s = (c[n:] - c[:-n])
    out = np.full(len(x), np.nan)
    out[n // 2: n // 2 + len(s)] = s / n
    return out


def harmonic(sums, cnts):
    m = sums / np.maximum(cnts, 1)
    ang = (np.arange(NB) + 0.5) * 2 * np.pi / NB
    a, b = 2 * np.mean(m * np.cos(ang)), 2 * np.mean(m * np.sin(ang))
    return math.hypot(a, b), math.degrees(math.atan2(b, a)) % 360, m


per_flight = []
for stem in sorted(f[:-8] for f in glob.glob(os.path.join(base, "*.out.csv"))):
    d = load(stem)
    t, st, hdg, ktas, wn, we, roll, pitch, acc_long, tas, ahrs_pitch, iwn, iwe = d.T
    # 1 s centred means; the airspeed derivative over 1 s
    tas1 = movmean(tas, 10)
    tdot = np.full(len(t), np.nan)
    tdot[5:-5] = (tas1[10:] - tas1[:-10]) / 1.0
    dd = ktas * tdot - movmean(acc_long, 10)
    dd = dd - movmean(dd, 300)  # the circle-periodic part
    dp = (pitch - ahrs_pitch) - movmean(pitch - ahrs_pitch, 300)
    circ = (np.abs(movmean(roll, 300)) > 25) & ((st.astype(int) & 1) == 1)
    wind_from = np.degrees(np.arctan2(-we, -wn))
    flight = {}
    for name, ang in [("heading rel. to wind", hdg - wind_from), ("heading", hdg)]:
        b = (((ang % 360) / 360 * NB).astype(int)) % NB
        for side, m in [("left", circ & (roll < 0)), ("right", circ & (roll > 0))]:
            ok = m & np.isfinite(dd)
            for what, val in [("d", dd), ("pitch", dp)]:
                okv = ok & np.isfinite(val)
                sums = np.bincount(b[okv], weights=val[okv], minlength=NB)
                cnts = np.bincount(b[okv], minlength=NB)
                key = (what, name, side)
                tot = acc.setdefault(key, [np.zeros(NB), np.zeros(NB)])
                tot[0] += sums; tot[1] += cnts
                flight[key] = harmonic(sums, cnts)[:2] if cnts.min() > 20 else (math.nan, math.nan)
    per_flight.append((os.path.basename(stem), flight))
    sys.stderr.write(os.path.basename(stem) + "\n")

for what, unit in [("d", "m/s²"), ("pitch", "°")]:
    print(f"\n{'IMU acceleration minus airspeed change' if what == 'd' else 'AirMass pitch minus device AHRS pitch'}, circle-periodic part [{unit}]:"
          " first harmonic amplitude @ phase (where it peaks), all flights")
    for name in ["heading rel. to wind", "heading"]:
        for side in ["left", "right"]:
            amp, ph, m = harmonic(*acc[(what, name, side)])
            print(f"  by {name:21s} {side:5s} circles: {amp:.3f} @ {ph:5.0f}°   bins: " + " ".join(f"{v:+.2f}" for v in m))
    print("  per flight, amplitude @ phase (by heading rel. to wind | by heading), left / right:")
    for fn, fl in per_flight:
        s = []
        for name in ["heading rel. to wind", "heading"]:
            s.append("  ".join(f"{fl[(what, name, side)][0]:.2f}@{fl[(what, name, side)][1]:3.0f}" for side in ["left", "right"]))
        print(f"    {fn:20s} " + " | ".join(s))
