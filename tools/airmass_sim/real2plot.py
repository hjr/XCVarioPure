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
COL = A.COL  # the simulator's columns plus the vario stand-in


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
            c = [0.0] * len(COL)
            c[COL["t"]] = r["t"]
            c[COL["true_wind_n"]], c[COL["true_wind_e"]] = drift_n[k], drift_e[k]
            c[COL["wind_n"]], c[COL["wind_e"]] = r["wn"], r["we"]
            c[COL["true_vario"]] = math.nan  # no truth
            c[COL["vario_shown"]] = r["am"]
            c[COL["te_netto"]] = r["netto"]
            c[COL["status"]] = r["st"]
            c[COL["true_roll"]] = r["roll"]
            e = ext[k]
            c[COL["pos_n"]] = (e["lat"] - lat0) * 111195.0 if math.isfinite(e["lat"]) else math.nan
            c[COL["pos_e"]] = (e["lon"] - lon0) * k_e if math.isfinite(e["lon"]) else math.nan
            c[COL["ins_wind_n"]], c[COL["ins_wind_e"]] = r["rn"], r["re"]
            c[COL["ins_netto"]] = r["ref"]
            c[COL["stand_in"]] = w_gnss[k]
            f.write(",".join("nan" if isinstance(v, float) and math.isnan(v) else f"{v:.3f}" for v in c) + "\n")
    print(f"{out_path}: {len(rows)} rows")


if __name__ == "__main__":
    main()
