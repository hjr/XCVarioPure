"""Wind along a simulated flight: every few seconds an arrow of the wind vector (the direction the air moves to), for the
truth, AirMass and the GNSS/INS reference.

  airmass_sim <scenario> --csv flight.csv
  python3 tools/airmass_sim/plot_wind.py flight.csv flight.png [--title T] [--layout map|time] [--every s] [--from s] [--to s]

  --layout map: the arrows start on the ground track, all three at the same point (cross country)
  --layout time: a feather plot, one row per source over time, north up (circling, where a map stacks the arrows);
                 default: time when the window has more than one full circle, else map
  --every: seconds between arrows (default: about 60 arrows over the window)
  --from, --to: time window [s]

The truth is smoothed over 5 s (what an estimator can follow, like the simulator's evaluation). AirMass is drawn while
its wind is shown: solid while valid, dashed while degraded (not valid any more, drawn grey on the device), no arrow
while not shown. Needs matplotlib."""
import argparse
import csv
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import transforms  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# columns of the simulator CSV (sim.cpp)
T, TRUE_N, TRUE_E, AM_N, AM_E, STATUS, ROLL, GN, GE, REF_N, REF_E = 0, 1, 2, 3, 4, 12, 14, 29, 30, 31, 32
ST_WIND_VALID, ST_WIND_DEGRADED = 2, 16
SHOWN = ST_WIND_VALID | ST_WIND_DEGRADED

INK, INK_2, MUTED, GRID, SURFACE, BAND = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb", "#efeeea"
AIRMASS, REFERENCE = "#2a78d6", "#eb6834"  # categorical slots 1, 2
DASHED = (0, (3, 2))


def load(path):
    with open(path) as f:
        return [[float(x) for x in row] for row in csv.reader(f) if row]


def smooth(rows, col, half):
    """centred moving average over +-half rows"""
    pre = [0.0]
    for r in rows:
        pre.append(pre[-1] + r[col])
    n = len(rows)
    return [(pre[min(n, i + half + 1)] - pre[max(0, i - half)]) / (min(n, i + half + 1) - max(0, i - half)) for i in range(n)]


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")


def heading_turned(rows, idx):
    """total heading change along the ground track [rad]"""
    total, prev = 0.0, None
    for i0, i1 in zip(idx[:-10:10], idx[10::10]):
        h = math.atan2(rows[i1][GE] - rows[i0][GE], rows[i1][GN] - rows[i0][GN])
        if prev is not None:
            total += abs(math.remainder(h - prev, 2 * math.pi))
        prev = h
    return total


def arrow(ax, x, y, dx, dy, color, lw, ls="-", z=3):
    ax.annotate("", xy=(x + dx, y + dy), xytext=(x, y), annotation_clip=False,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0, mutation_scale=9), zorder=z)


def style(ax, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(colors=INK_2, labelsize=9, length=0)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("png")
    ap.add_argument("--title", default="")
    ap.add_argument("--layout", choices=["map", "time"], default="")
    ap.add_argument("--every", type=float, default=0.0)
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=1e9)
    a = ap.parse_args()

    rows = load(a.csv)
    dt = rows[1][T] - rows[0][T]
    # a real flight (real2plot.py) has no truth, the circle drift stands in for it while circling
    real = sum(math.isfinite(r[TRUE_N]) for r in rows) < 0.98 * len(rows)
    if real:
        tn, te = [r[TRUE_N] for r in rows], [r[TRUE_E] for r in rows]
        truth_label = "circle drift (GNSS, per full circle; no truth in a real flight)"
    else:
        tn, te = smooth(rows, TRUE_N, int(2.5 / dt)), smooth(rows, TRUE_E, int(2.5 / dt))
        truth_label = "truth (5 s smoothed)"
    fin = math.isfinite
    idx = [i for i, r in enumerate(rows) if a.t0 <= r[T] <= a.t1]
    t_a, t_b = rows[idx[0]][T], rows[idx[-1]][T]
    every = a.every or max(1.0, round((t_b - t_a) / 60.0))
    sel = idx[:: max(1, int(round(every / dt)))]
    layout = a.layout or ("time" if heading_turned(rows, idx) > 2.5 * math.pi else "map")

    # accuracy over the whole window, against the smoothed truth
    err_am, err_ref, shown = [], [], 0
    for i in idx:
        r = rows[i]
        if int(r[STATUS]) & SHOWN:
            shown += 1
        if not fin(tn[i]):
            continue
        if fin(r[REF_N]):
            err_ref.append(math.hypot(r[REF_N] - tn[i], r[REF_E] - te[i]))
        if int(r[STATUS]) & SHOWN:
            err_am.append(math.hypot(r[AM_N] - tn[i], r[AM_E] - te[i]))
    mags = [math.hypot(tn[i], te[i]) for i in sel if fin(tn[i])] or \
           [math.hypot(rows[i][AM_N], rows[i][AM_E]) for i in sel if int(rows[i][STATUS]) & SHOWN] or [1.0]
    wind_max = max(max(mags), 1.0)
    ref_speed = 5.0 if wind_max > 3.0 else 2.0
    sources = [  # label, north, east, color, width, z
        (truth_label, lambda i: tn[i], lambda i: te[i], INK, 2.4, 3),
        ("GNSS/INS reference", lambda i: rows[i][REF_N], lambda i: rows[i][REF_E], REFERENCE, 1.4, 4),
        ("AirMass", lambda i: rows[i][AM_N], lambda i: rows[i][AM_E], AIRMASS, 1.4, 5),
    ]

    if layout == "map":
        idx = [i for i in idx if fin(rows[i][GE]) and fin(rows[i][GN])]
        sel = [i for i in sel if fin(rows[i][GE]) and fin(rows[i][GN])]
        xs = [rows[i][GE] / 1000.0 for i in idx]  # km east
        ys = [rows[i][GN] / 1000.0 for i in idx]  # km north
        extent = max(max(xs) - min(xs), max(ys) - min(ys), 0.5)
        scale = 0.12 * extent / wind_max  # km per m/s
        tips_x, tips_y = list(xs), list(ys)
        for i in sel:
            for _, fn, fe, *_ in sources:
                if fin(fn(i)):
                    tips_x.append(rows[i][GE] / 1000.0 + fe(i) * scale)
                    tips_y.append(rows[i][GN] / 1000.0 + fn(i) * scale)
        w, h = max(tips_x) - min(tips_x), max(tips_y) - min(tips_y)
        pad = 0.04 * max(w, h)
        fig, ax = plt.subplots(figsize=(11, min(max(11.0 * (h + 2 * pad) / (w + 2 * pad), 3.5), 12.0)), dpi=150)
        ax.plot(xs, ys, color=MUTED, lw=1.2, zorder=1)
        ax.plot(xs[0], ys[0], "o", ms=7, color=MUTED, mec=SURFACE, mew=2, zorder=2)
        for i in sel:
            x, y, st = rows[i][GE] / 1000.0, rows[i][GN] / 1000.0, int(rows[i][STATUS])
            for label, fn, fe, color, lw, z in sources:
                if (label == "AirMass" and not st & SHOWN) or not fin(fn(i)):
                    continue
                ls = DASHED if label == "AirMass" and not st & ST_WIND_VALID else "-"
                arrow(ax, x, y, fe(i) * scale, fn(i) * scale, color, lw, ls, z)
        ax.set_xlim(min(tips_x) - pad, max(tips_x) + pad)
        ax.set_ylim(min(tips_y) - pad, max(tips_y) + pad)
        ax.set_aspect("equal")
        style(ax, "east [km]", "north [km]")
        scale_note = f"arrow length: {ref_speed:.0f} m/s = {ref_speed * scale * 1000:.0f} m on the map"
        track = [Line2D([], [], color=MUTED, lw=1.2, label="ground track (dot: start)")]
    else:
        # feather plot: x is time, one baseline per source, arrows north up with true direction (equal aspect)
        span = t_b - t_a
        scale = 0.06 * span / wind_max  # s of x per m/s
        # rows as high as the arrows reach north and south
        up = max(max((fn(i) for i in sel if fin(fn(i))), default=0.0) for _, fn, *_ in sources)
        down = max(max((-fn(i) for i in sel if fin(fn(i))), default=0.0) for _, fn, *_ in sources)
        up, down = max(up, 0.0) * scale, max(down, 0.0) * scale
        gap = 0.35 * wind_max * scale
        row_h = up + down + gap
        base = {label: -k * row_h for k, (label, *_) in enumerate(sources)}
        fig, ax = plt.subplots(figsize=(11, min(max(11.0 * (len(sources) + 0.6) * row_h / (span * 1.04), 4.0), 12.0)), dpi=150)
        # turning (bank over 20°) as bands
        start = None
        for i in idx + [None]:
            circling = i is not None and abs(rows[i][ROLL]) > 20.0
            if circling and start is None:
                start = rows[i][T]
            elif not circling and start is not None:
                ax.axvspan(start, rows[i][T] if i is not None else t_b, color=BAND, zorder=0, lw=0)
                start = None
        for label, fn, fe, color, lw, z in sources:
            y0 = base[label]
            ax.axhline(y0, color=GRID, lw=0.8, zorder=1)
            ax.text(t_a - 0.01 * span, y0, label.split(" (")[0], ha="right", va="center", fontsize=9, color=INK)
            for i in sel:
                st = int(rows[i][STATUS])
                if (label == "AirMass" and not st & SHOWN) or not fin(fn(i)):
                    continue
                ls = DASHED if label == "AirMass" and not st & ST_WIND_VALID else "-"
                arrow(ax, rows[i][T], y0, fe(i) * scale, fn(i) * scale, color, lw, ls, z)
        ax.set_xlim(t_a - 0.02 * span, t_b + 0.06 * span)
        ax.set_ylim(base["AirMass"] - down - gap / 2, up + gap / 2)
        ax.set_aspect("equal")
        style(ax, "time [s]", "")
        ax.set_yticks([])
        scale_note = f"north up; arrow length: {ref_speed:.0f} m/s = {ref_speed * scale:.0f} s on the time axis"
        track = [Patch(color=BAND, label="turning (bank over 20°)")]

    def err_text(e):
        return f"{rms(e):.2f} m/s rms{' vs the circle drift' if real else ''}" if e else "no circle drift here to compare"

    legend = track + [
        Line2D([], [], color=INK, lw=2.4, label=truth_label),
        Line2D([], [], color=AIRMASS, lw=1.4, label=f"AirMass: {err_text(err_am)}, shown {100 * shown / len(idx):.0f}% of the time"),
        Line2D([], [], color=AIRMASS, lw=1.4, ls=DASHED, label="AirMass degraded (drawn grey on the device)"),
        Line2D([], [], color=REFERENCE, lw=1.4, label=f"GNSS/INS reference{' (after Larus, 1 Hz GNSS, AirMass heading)' if real else ''}: "
                                                     f"{err_text(err_ref)}"),
    ]
    # below the axis label, a fixed distance whatever the figure height
    below = transforms.offset_copy(ax.transAxes, fig=fig, y=-38, units="points")
    ax.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.0, 0.0), bbox_transform=below, ncol=2, frameon=False,
              fontsize=9, labelcolor=INK, borderaxespad=0.0)
    fig.patch.set_facecolor(SURFACE)
    ax.set_title(a.title, loc="left", color=INK, fontsize=12, fontweight="bold", pad=22)
    ax.text(0.0, 1.01, f"wind every {every:.0f} s, {t_a:.0f}–{t_b:.0f} s; {scale_note}", transform=ax.transAxes, fontsize=9,
            color=INK_2, va="bottom")
    fig.savefig(a.png, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.25)
    print(f"{a.png} ({layout}): {len(sel)} arrows, AirMass {rms(err_am):.2f} m/s rms (shown {100 * shown / len(idx):.0f}%), "
          f"reference {rms(err_ref):.2f} m/s rms")


if __name__ == "__main__":
    main()
