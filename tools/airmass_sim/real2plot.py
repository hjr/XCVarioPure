"""A replayed real flight (replay_sens.py) as a simulator CSV (sim.cpp --csv columns), for plot_wind.py and plot_vario.py.
A real flight has no truth; in its place, marked as such in the plots:
  wind: the GNSS drift of each full circle while circling (empty in straight flight)
  vario: the air movement from the GNSS altitude, the airspeed energy and the polar sink (TAS terms), 20 s centred
The device TE netto is reconstructed with the TAS/IAS polar fix, the GNSS/INS reference netto the same way.

  python3 tools/airmass_sim/real2plot.py replay_dir/<name> out.csv
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_sens as A  # noqa: E402

G = 9.80665
NCOL = 35  # the simulator's 34 columns (0..33) plus the vario stand-in (34)


def main():
    base, out_path = sys.argv[1], sys.argv[2]
    rows = A.load_flight(base, 4.0)
    aux = {int(round(int(r["t_ms"]) / 100.0)): r for r in csv.DictReader(open(base + ".aux.csv"))}
    ext = []
    for r in rows:
        x = aux[int(round(r["t"] * 10.0))]
        ext.append(dict(lat=A.num(x["lat"]), lon=A.num(x["lon"]), h=A.num(x["gnss_alt"]), tas=A.num(x["tas"]),
                        sink=A.num(x["polar_sink"]) * A.num(x["tas"]) / max(A.num(x["ias"]), 10.0)))
    lat0 = next(e["lat"] for e in ext if math.isfinite(e["lat"]))
    lon0 = next(e["lon"] for e in ext if math.isfinite(e["lon"]))
    k_e = 111195.0 * math.cos(math.radians(lat0))

    # wind stand-in: the drift of each full circle, on its samples
    drift_n = [math.nan] * len(rows)
    drift_e = [math.nan] * len(rows)
    for i0, i1, vn, ve in A.circles(rows):
        for k in range(i0, i1 + 1):
            drift_n[k], drift_e[k] = vn, ve

    # vario stand-in: GNSS altitude energy over +-10 s
    half = 100
    w_gnss = [math.nan] * len(rows)
    for k in range(half, len(rows) - half):
        a, b = ext[k - half], ext[k + half]
        T = rows[k + half]["t"] - rows[k - half]["t"]
        if not (15.0 < T < 25.0) or not all(map(math.isfinite, (a["h"], b["h"], a["tas"], b["tas"]))):
            continue
        sinks = [ext[j]["sink"] for j in range(k - half, k + half + 1, 10) if math.isfinite(ext[j]["sink"])]
        if not sinks:
            continue
        w_gnss[k] = (b["h"] - a["h"]) / T + (b["tas"] ** 2 - a["tas"] ** 2) / (2 * G * T) - sum(sinks) / len(sinks)

    with open(out_path, "w") as f:
        for k, r in enumerate(rows):
            c = [0.0] * NCOL
            c[0] = r["t"]
            c[1], c[2] = drift_n[k], drift_e[k]
            c[3], c[4] = r["wn"], r["we"]
            c[5] = math.nan  # no truth
            c[6] = r["am"]
            c[7] = r["netto"]
            c[12] = r["st"]
            c[14] = r["roll"]
            e = ext[k]
            c[29] = (e["lat"] - lat0) * 111195.0 if math.isfinite(e["lat"]) else math.nan
            c[30] = (e["lon"] - lon0) * k_e if math.isfinite(e["lon"]) else math.nan
            c[31], c[32] = r["rn"], r["re"]
            c[33] = r["ref"]
            c[34] = w_gnss[k]
            f.write(",".join("nan" if isinstance(v, float) and math.isnan(v) else f"{v:.3f}" for v in c) + "\n")
    print(f"{out_path}: {len(rows)} rows")


if __name__ == "__main__":
    main()
