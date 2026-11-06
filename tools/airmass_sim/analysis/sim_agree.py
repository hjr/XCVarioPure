"""The real-flight comparison, in the simulator where the truth is known: AirMass and TE netto against the true air
movement and against a GNSS-netto-style reference built like on the real flights (energy height from the altitude at
1 Hz fixes plus the measured TAS, 2 s centred difference, plus the polar sink), without and with gyro gating."""
import glob
import os
import sys

import numpy as np

G = 9.80665
T, TRUTH, AM_SHOWN, TE_NETTO, STATUS, ROLL, AM_RAW, H, TAS, POLAR, INS = 0, 5, 6, 7, 12, 14, 17, 22, 34, 35, 33
W = [5, 10, 30, 60]


def reference(d):
    """GNSS netto as the page builds it, here with the true altitude (no GNSS noise, no latency)"""
    t = d[:, T]
    e = d[:, H] + d[:, TAS] ** 2 / (2 * G)
    fix = np.arange(0, len(t), 10)  # 1 Hz fixes
    tf, ef = t[fix], e[fix]
    rate = np.full(len(fix), np.nan)
    rate[1:-1] = (ef[2:] - ef[:-2]) / (tf[2:] - tf[:-2])
    # the mean polar sink over the 2 s, then interpolated to 10 Hz
    pol = np.convolve(d[:, POLAR], np.ones(21) / 21, mode="same")[fix]
    return np.interp(t, tf, rate + pol)


def load(path):
    d = np.loadtxt(path, delimiter=",")
    ref = reference(d)
    free = (d[:, T] > 300) & ((d[:, STATUS].astype(int) & 1) == 1) & np.isfinite(ref)
    series = {"AirMass raw": d[:, AM_RAW], "AirMass cockpit": d[:, AM_SHOWN], "TE netto": d[:, TE_NETTO], "GNSS/INS (Larus)": d[:, INS]}
    return d, ref, free, series


def window_rms(x, y, free, w, phase_mask):
    n = int(w * 10)
    errs = []
    for k in range(0, len(x) - n, n):
        sl = slice(k, k + n)
        if free[sl].all() and phase_mask[sl].mean() > 0.8:
            errs.append(x[sl].mean() - y[sl].mean())
    e = np.array(errs)
    return (np.sqrt((e ** 2).mean()), len(e)) if len(e) else (np.nan, 0)


def lag_corr(x, y, free):
    best = (-9, None)
    at0 = None
    for lag in np.arange(-2, 10.01, 0.25):
        s = int(round(lag * 10))
        if s >= 0:
            a, b, m = x[s:], y[: len(y) - s], free[s:] & free[: len(free) - s]
        else:
            a, b, m = x[: len(x) + s], y[-s:], free[: len(free) + s] & free[-s:]
        c = np.corrcoef(a[m], b[m])[0, 1]
        if abs(lag) < 1e-6:
            at0 = c
        if c > best[0]:
            best = (c, lag)
    return best[1], best[0], at0


def main(folder):
    for gate in sys.argv[2:] or ["0", "0.5"]:
        files = sorted(glob.glob(os.path.join(folder, f"*_g{gate}.csv")))
        runs = [load(f) for f in files]
        print(f"\n===== gyro gating {gate} °/s ({len(files)} runs, development day 6 seeds + 6 held-out days)")
        for target in ["truth", "GNSS-style reference"]:
            print(f"\n  against the {target}: rms m/s over windows")
            for ph in ["all", "straight", "circling"]:
                print(f"    {ph:9s}" + "".join(f"{w:>9d} s" for w in W))
                for name in ["AirMass raw", "AirMass cockpit", "TE netto", "GNSS/INS (Larus)"]:
                    vals = []
                    for w in W:
                        s2, n = 0.0, 0
                        for d, ref, free, ser in runs:
                            roll = np.abs(d[:, ROLL])
                            mask = np.ones(len(d), bool) if ph == "all" else (roll > 20 if ph == "circling" else roll < 20)
                            y = d[:, TRUTH] if target == "truth" else ref
                            r, k = window_rms(ser[name], y, free, w, mask)
                            if k:
                                s2 += r * r * k
                                n += k
                        vals.append(np.sqrt(s2 / n) if n else np.nan)
                    print(f"      {name:17s}" + "".join(f"{v:11.3f}" for v in vals))
            print(f"    delay and correlation against the {target}")
            for name in ["AirMass raw", "AirMass cockpit", "TE netto", "GNSS/INS (Larus)"]:
                xs = np.concatenate([ser[name] for d, ref, free, ser in runs])
                ys = np.concatenate([d[:, TRUTH] if target == "truth" else ref for d, ref, free, ser in runs])
                fr = np.concatenate([free for d, ref, free, ser in runs])
                lag, c, c0 = lag_corr(xs, ys, fr)
                print(f"      {name:17s} delay {lag:5.2f} s  correlation {c:.3f} (at 0 s {c0:.3f})")
        print("\n  needle movement within 1 s, rms m/s")
        for name in ["AirMass cockpit", "TE netto", "GNSS/INS (Larus)", "truth"]:
            s2, n = 0.0, 0
            for d, ref, free, ser in runs:
                x = d[:, TRUTH] if name == "truth" else ser[name]
                dx = x[10:] - x[:-10]
                m = free[10:] & free[:-10]
                s2 += (dx[m] ** 2).sum()
                n += m.sum()
            print(f"      {name:17s} {np.sqrt(s2 / n):.3f}")


if __name__ == "__main__":
    main(sys.argv[1])
