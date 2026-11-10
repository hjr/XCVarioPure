"""Evaluate AirMass on replayed XCVario $SENS flights (replay_sens.py) against what the flight data itself can tell:

  * pull-ups in calm air (a list of "<local time> | v1 -> v2 km/h | duration s | residual h m" lines, e.g. from the
    XCVario discussion 451): the air movement over the manoeuvre is residual / duration, a vario must show that and
    nothing of the speed change; AirMass and the device's own TE netto against it
  * circling drift: over a full circle the GNSS ground velocity averages to the wind (constant airspeed), AirMass's
    instant and averaged wind against it
  * the AirMass vario against the device's TE netto over the whole flight (no truth, the agreement and the difference)
The device netto is reconstructed from the logged TE vario and the polar, with the polar sink in TAS terms as
VarioFilter computes it since the TAS/IAS fix (the logged flights ran without it).

  python3 tools/airmass_sim/analyze_sens.py replay_dir [--pullups pullups.txt] [--utc-offset h] [--plots dir]
"""
import argparse
import csv
import glob
import math
import os
import re

ST_VARIO, ST_WIND, ST_DEGRADED = 1, 2, 16

# The columns of the simulator CSV (sim.cpp --csv, no header), in order; real2plot.py writes a real flight like it, with
# the vario stand-in after them. true_*: the simulated truth, pos_n/e: the ground track [m], ins_*: the GNSS/INS
# reference, te_polar: the polar sink TE netto adds.
SIM_COLUMNS = ("t", "true_wind_n", "true_wind_e", "wind_n", "wind_e", "true_vario", "vario_shown", "te_netto",
               "heading_error", "k_tas", "pitch_offset", "acc_bias", "status", "wind_sigma", "true_roll", "true_vz", "v_down",
               "vario", "roll_error", "pitch_error", "true_v", "load_factor", "true_h", "true_xa", "true_ya", "pilot_mode",
               "te_vario", "heading_sigma", "yaw_rate_bias", "pos_n", "pos_e", "ins_wind_n", "ins_wind_e", "ins_netto",
               "tas", "te_polar", "stand_in")


COL = {name: i for i, name in enumerate(SIM_COLUMNS)}


def sim_columns(*names):
    """the indices of the named simulator CSV columns"""
    return [COL[n] for n in names]


def load_sim_csv(path):
    """the rows of a simulator CSV, as floats"""
    with open(path) as f:
        return [[float(x) for x in row] for row in csv.reader(f) if row]


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else math.nan


def mean(v):
    return sum(v) / len(v) if v else math.nan


class Damp:
    """the device's vario damping (VarioFilter), 10 Hz"""

    def __init__(self, d):
        self.d, self.l, self.y = d, None, None

    def f(self, x):
        if self.l is None:
            self.l = self.y = x
        self.l += (x - self.l) * 0.1 / (self.d / 3.0 + 0.1)
        self.y += (self.l - self.y) / max(self.d, 1.0)
        return self.y


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return math.nan


def free_flight(base):
    """decisecond keys of the free flight samples: not on the ground (IAS under 60 km/h), not on tow (from the launch to
    the first 10 s banked over 30 deg, at most 15 min), not in the landing approach (the last 30 min before the landing
    below the field elevation + 400 m: airbrakes, landing flaps and wheel add drag the polar does not know)"""
    rows = [r for r in csv.DictReader(open(base + ".aux.csv")) if r["running"] == "1"]
    keys = set()
    k = 0
    while k < len(rows):
        # one launch: consecutive running samples (a gap over 5 s ends it)
        j = k
        while j + 1 < len(rows) and int(rows[j + 1]["t_ms"]) - int(rows[j]["t_ms"]) < 5000:
            j += 1
        seg = rows[k:j + 1]
        k = j + 1
        t = [int(r["t_ms"]) / 1000.0 for r in seg]
        alts = [num(r["gnss_alt"]) for r in seg[-50:] if math.isfinite(num(r["gnss_alt"]))]
        field = min(alts) if alts else math.nan
        tow_end, banked = t[0] + 900.0, 0
        for r, ti in zip(seg, t):
            banked = banked + 1 if abs(num(r["roll_deg"])) > 30.0 else 0
            if banked >= 100:
                tow_end = min(tow_end, ti)
                break
        for r, ti in zip(seg, t):
            if num(r["ias"]) < 60.0 / 3.6 or ti < tow_end:
                continue
            if t[-1] - ti < 1800.0 and math.isfinite(field) and num(r["gnss_alt"]) < field + 400.0:
                continue
            keys.add(int(round(int(r["t_ms"]) / 100.0)))
    return keys


def load_flight(base, vario_delay):
    # the replay CSV has the time to 0.1 s
    aux = {int(round(int(r["t_ms"]) / 100.0)): r for r in csv.DictReader(open(base + ".aux.csv"))}
    ref = {}
    if os.path.exists(base + ".ref.csv"):
        ref = {int(round(float(r["t_s"]) * 10.0)): r for r in csv.DictReader(open(base + ".ref.csv"))}
    am_damp, sink_damp = Damp(max(1.0, vario_delay * 0.5)), Damp(vario_delay)
    free = free_flight(base)
    rows = []
    for r in csv.DictReader(open(base + ".out.csv")):
        t_ms = int(round(float(r["t_s"]) * 1000))
        x = aux.get(int(round(float(r["t_s"]) * 10.0)))
        if x is None:
            continue
        st = int(r["status"])
        am = float(r["vario"])
        # the device netto with the polar sink in TAS terms (VarioFilter: getSink(ias) * tas / ias)
        sink = num(x["polar_sink"]) * num(x["tas"]) / max(num(x["ias"]), 10.0)
        te = num(x["te_vario"])
        y = ref.get(int(round(float(r["t_s"]) * 10.0)))
        sink_now = sink if math.isfinite(sink) else 0.0
        rows.append(dict(t=t_ms / 1000.0, utc=num(x["utc_s"]), st=st, free=int(round(float(r["t_s"]) * 10.0)) in free, am_raw=am, am=am_damp.f(am),
                         netto=te - sink_damp.f(sink_now), te=te, ias=num(x["ias"]),
                         ref=(num(y["vario"]) - sink_now) if y else math.nan,
                         rn=num(y["wind_n"]) if y else math.nan, re=num(y["wind_e"]) if y else math.nan,
                         roll=num(x["roll_deg"]), wn=float(r["wind_n"]), we=float(r["wind_e"]), an=float(r["avg_wind_n"]),
                         ae=float(r["avg_wind_e"]), gs=num(x["gnss_speed"]), gt=num(x["gnss_track"])))
    return rows


def circles(rows):
    """full circles from the GNSS track while banked: (start index, end index, drift wind N, E)"""
    out, start, turned, prev = [], None, 0.0, None
    fixes = []  # indices where a new fix arrived
    last_utc = None
    for i, r in enumerate(rows):
        if r["utc"] != last_utc and math.isfinite(r["gt"]):
            fixes.append(i)
            last_utc = r["utc"]
    for i in fixes:
        r = rows[i]
        if abs(r["roll"]) < 20.0 or r["gs"] < 10.0 or not r["free"]:
            start, turned, prev = None, 0.0, None
            continue
        if start is None:
            start, turned, prev, pts = i, 0.0, r["gt"], [i]
            continue
        turned += math.remainder(r["gt"] - prev, 360.0)
        prev = r["gt"]
        pts.append(i)
        if abs(turned) >= 360.0:
            # the fixes of the circle, the last one closes it (not counted twice)
            vn = mean([rows[k]["gs"] * math.cos(math.radians(rows[k]["gt"])) for k in pts[:-1]])
            ve = mean([rows[k]["gs"] * math.sin(math.radians(rows[k]["gt"])) for k in pts[:-1]])
            out.append((start, i, vn, ve))
            start, turned, prev, pts = i, 0.0, r["gt"], [i]
    return out


def parse_pullups(path, utc_offset):
    out = []
    for line in open(path):
        m = re.match(r"(\d{4}-\d{2}-\d{2})T(\d{2}):(\d{2}):(\d{2})[^|]*\|\s*([0-9.]+) -> ([0-9.]+) km/h \|\s*([0-9.]+) s \| residual ([-0-9.]+) m", line)
        if m:
            date, hh, mm, ss = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
            out.append(dict(date=date, utc=(hh - utc_offset) * 3600 + mm * 60 + ss, v1=float(m.group(5)), v2=float(m.group(6)),
                            dur=float(m.group(7)), residual=float(m.group(8)), label=line.split("|")[0].strip()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("replay_dir")
    ap.add_argument("--pullups")
    ap.add_argument("--utc-offset", type=float, default=2.0, help="hours of the local times in the pull-up list")
    ap.add_argument("--vario-delay", type=float, default=4.0, help="the device's Vario Damping setting [s]")
    ap.add_argument("--plots")
    a = ap.parse_args()

    flights = {}
    for out_csv in sorted(glob.glob(os.path.join(a.replay_dir, "*.out.csv"))):
        base = out_csv[: -len(".out.csv")]
        flights[os.path.basename(base)] = load_flight(base, a.vario_delay)

    print(f"{'flight':18s} {'hours':>5s} {'vario':>6s} {'wind':>5s} {'shown':>6s} {'AM-netto':>9s} {'circles':>7s} "
          f"{'inst-drift':>10s} {'avg-drift':>9s}")
    wind_pairs = []
    for name, rows in flights.items():
        n = len(rows)
        if not n:
            print(f"{name:18s} no data")
            continue
        vario_ok = [r for r in rows if r["st"] & ST_VARIO and r["free"]]
        diff = [r["am"] - r["netto"] for r in vario_ok if math.isfinite(r["netto"])]
        circ = circles(rows)
        inst, avg = [], []
        for i0, i1, vn, ve in circ:
            seg = [rows[k] for k in range(i0, i1 + 1) if rows[k]["st"] & (ST_WIND | ST_DEGRADED)]
            if len(seg) < 0.8 * (i1 - i0 + 1):
                continue
            inst.append(math.hypot(mean([r["wn"] for r in seg]) - vn, mean([r["we"] for r in seg]) - ve))
            end = rows[i1]
            if end["st"] & ST_WIND:
                avg.append(math.hypot(end["an"] - vn, end["ae"] - ve))
            refseg = [r for r in seg if math.isfinite(r["rn"])]
            rn = mean([r["rn"] for r in refseg]) if len(refseg) > 0.8 * len(seg) else math.nan
            ren = mean([r["re"] for r in refseg]) if len(refseg) > 0.8 * len(seg) else math.nan
            wind_pairs.append((name, rows[i0]["utc"], vn, ve, mean([r["wn"] for r in seg]), mean([r["we"] for r in seg]), rn, ren))
        print(f"{name:18s} {n / 36000.0:5.2f} {100 * len(vario_ok) / n:5.0f}% {100 * sum(r['st'] & ST_WIND > 0 for r in rows) / n:4.0f}% "
              f"{100 * sum(r['st'] & (ST_WIND | ST_DEGRADED) > 0 for r in rows) / n:5.0f}% {rms(diff):9.2f} {len(circ):7d} "
              f"{rms(inst):10.2f} {rms(avg):9.2f}")
    print("AM-netto: rms of AirMass (shown damping) minus the device's TE netto; inst-drift: AirMass instant wind over a "
          "circle against the circle's GNSS drift [m/s]; avg-drift: its averaged wind at the circle's end")

    if a.pullups:
        pulls = parse_pullups(a.pullups, a.utc_offset)
        print(f"\npull-ups in calm air: the air movement is residual / duration; mean and max error of the vario over the manoeuvre [m/s]")
        print(f"{'local time':26s} {'IAS':>13s} {'dur':>5s} {'air':>6s} | {'AirMass mean':>12s} {'max':>5s} | {'TE netto mean':>13s} {'max':>5s}"
              f" | {'GNSS/INS mean':>13s} {'max':>5s}")
        found = []
        for p in pulls:
            day = p["date"].replace("-", "")
            for name, rows in flights.items():
                if not name.startswith(p["date"]):
                    continue
                win = [r for r in rows if math.isfinite(r["utc"]) and p["utc"] <= r["utc"] <= p["utc"] + p["dur"]]
                if len(win) < 10:
                    continue
                air = p["residual"] / p["dur"]
                am = [r["am"] - air for r in win if r["st"] & ST_VARIO]
                te = [r["netto"] - air for r in win if math.isfinite(r["netto"])]
                rf = [r["ref"] - air for r in win if math.isfinite(r["ref"])]
                found.append((p, name, win, air, am, te, rf))
                print(f"{p['label']:26s} {win[0]['ias'] * 3.6:5.0f}->{win[-1]['ias'] * 3.6:4.0f} km/h {p['dur']:5.1f} {air:+6.2f} | "
                      f"{mean(am):+12.2f} {max(map(abs, am), default=math.nan):5.2f} | {mean(te):+13.2f} {max(map(abs, te), default=math.nan):5.2f}"
                      f" | {mean(rf):+13.2f} {max(map(abs, rf), default=math.nan):5.2f}")
        if found:
            am_all = [x for f in found for x in f[4]]
            te_all = [x for f in found for x in f[5]]
            rf_all = [x for f in found for x in f[6]]
            print(f"{len(found)} of {len(pulls)} pull-ups in the replayed flights; error rms / mean |max| over all: AirMass "
                  f"{rms(am_all):.2f} / {mean([max(map(abs, f[4])) for f in found if f[4]]):.2f}, TE netto {rms(te_all):.2f} / "
                  f"{mean([max(map(abs, f[5])) for f in found if f[5]]):.2f}, GNSS/INS reference {rms(rf_all):.2f} / "
                  f"{mean([max(map(abs, f[6])) for f in found if f[6]]):.2f} m/s")
        if a.plots and found:
            plot_pullups(found, os.path.join(a.plots, "pullups.png"))
    if a.plots and wind_pairs:
        plot_wind(wind_pairs, os.path.join(a.plots, "circle_wind.png"))


def plot_pullups(found, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    INK, INK_2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    AIRMASS, TE, REFERENCE = "#2a78d6", "#1baf7a", "#eb6834"
    n = len(found)
    cols = 4
    rows_n = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows_n, cols, figsize=(12, 2.3 * rows_n + 0.8), dpi=140, sharey=True, squeeze=False)
    fig.patch.set_facecolor(SURFACE)
    for k, ax in enumerate(axes.flat):
        if k >= n:
            ax.set_visible(False)
            continue
        p, name, win, air = found[k][:4]
        t0 = win[0]["t"]
        ts = [r["t"] - t0 for r in win]
        ax.set_facecolor(SURFACE)
        ax.axhline(air, color=INK, lw=2.0)
        ax.plot(ts, [r["ref"] for r in win], color=REFERENCE, lw=1.4)
        ax.plot(ts, [r["netto"] for r in win], color=TE, lw=1.6)
        ax.plot(ts, [r["am"] if r["st"] & ST_VARIO else math.nan for r in win], color=AIRMASS, lw=1.6)
        ax.set_title(f"{p['label'][:16].replace('T', ' ')}  {p['v1']:.0f}→{p['v2']:.0f} km/h", fontsize=8, color=INK, loc="left")
        ax.set_ylim(-7.0, 11.0)  # the reference's spikes go beyond, clipped
        ax.grid(True, color=GRID, lw=0.5)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(colors=INK_2, labelsize=7, length=0)
    for ax in axes[:, 0]:
        ax.set_ylabel("m/s", color=INK_2, fontsize=8)
    for ax in axes[-1, :]:
        ax.set_xlabel("s from the pull-up start", color=INK_2, fontsize=8)
    fig.legend(handles=[Line2D([], [], color=INK, lw=2, label="air movement over the manoeuvre (height residual / duration)"),
                        Line2D([], [], color=AIRMASS, lw=1.6, label="AirMass (as shown)"),
                        Line2D([], [], color=TE, lw=1.6, label="device TE netto (as shown, with the TAS/IAS polar fix)"),
                        Line2D([], [], color=REFERENCE, lw=1.4, label="GNSS/INS reference after the Larus design, on this installation's 1 Hz GNSS")],
               loc="lower left", ncol=2, frameon=False, fontsize=9, bbox_to_anchor=(0.01, 0.0))
    fig.suptitle("Real flights: pull-ups in calm air (ASG 29E, XCVario discussion 451; y axis clipped at -7..11 m/s)", x=0.01,
                 ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.045, 1, 0.97))
    fig.savefig(path, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.2)
    print("wrote", path)


def plot_wind(pairs, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    INK, INK_2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
    AIRMASS, REFERENCE = "#2a78d6", "#eb6834"
    from matplotlib.lines import Line2D
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4), dpi=140)
    fig.patch.set_facecolor(SURFACE)
    for ax, comp, lab in ((axes[0], (2, 4, 6), "north"), (axes[1], (3, 5, 7), "east")):
        both = [p for p in pairs if math.isfinite(p[comp[2]])]  # circles where both have a wind
        x = [p[comp[0]] for p in both]
        y = [p[comp[1]] for p in both]
        z = [p[comp[2]] for p in both]
        lim = max(max(map(abs, x)), max(map(abs, y)), 1.0) * 1.1
        ax.plot([-lim, lim], [-lim, lim], color=INK_2, lw=1.0)
        ax.plot(x, z, "o", ms=4, color=REFERENCE, mec=SURFACE, mew=0.8, alpha=0.6)
        ax.plot(x, y, "o", ms=4, color=AIRMASS, mec=SURFACE, mew=0.8, alpha=0.7)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, lw=0.5)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(colors=INK_2, labelsize=8, length=0)
        ax.set_xlabel(f"wind {lab} from the circle's GNSS drift [m/s]", color=INK_2, fontsize=9)
        ax.set_ylabel(f"instant wind {lab}, mean over the circle [m/s]", color=INK_2, fontsize=9)
        err = [b - c for b, c in zip(y, x)]
        err_ref = [b - c for b, c in zip(z, x)]
        ax.set_title(f"{lab}: {len(x)} circles, difference rms\nAirMass {rms(err):.2f} m/s, GNSS/INS reference {rms(err_ref):.2f} m/s",
                     fontsize=10, color=INK, loc="left")
    fig.legend(handles=[Line2D([], [], ls="", marker="o", color=AIRMASS, label="AirMass"),
                        Line2D([], [], ls="", marker="o", color=REFERENCE,
                               label="GNSS/INS reference after the Larus design (1 Hz GNSS, AirMass's heading)")],
               loc="lower left", ncol=2, frameon=False, fontsize=9, bbox_to_anchor=(0.01, 0.0))
    fig.suptitle("Real flights: instant wind against the circle drift (ASG 29E, 11 flights)", x=0.01, ha="left", fontsize=12,
                 fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.06, 1, 0.94))
    fig.savefig(path, facecolor=SURFACE)
    print("wrote", path)


if __name__ == "__main__":
    main()
