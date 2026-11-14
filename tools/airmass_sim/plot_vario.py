"""Vario of a simulated flight: the vertical air movement (truth) against what the pilot is shown by AirMass, TE netto
and the GNSS/INS reference, each with its display damping, and their errors below.

  airmass_sim <scenario> --csv flight.csv
  python3 tools/airmass_sim/plot_vario.py flight.csv flight.png [--title T] [--from s] [--to s]

AirMass is drawn while its vario is valid (the device falls back to netto otherwise). Needs matplotlib."""
import argparse
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import transforms  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_sens import ST_VARIO, load_sim_csv, rms, sim_columns  # noqa: E402

T, TRUTH, AIRMASS_SHOWN, TE_NETTO, STATUS, ROLL, REF_NETTO, STAND_IN = sim_columns(
    "t", "true_vario", "vario_shown", "te_netto", "status", "true_roll", "ins_netto", "stand_in")

INK, INK_2, GRID, SURFACE, BAND = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb", "#efeeea"
AIRMASS, REFERENCE, TE = "#2a78d6", "#eb6834", "#1baf7a"  # categorical slots 1, 2, 3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("png")
    ap.add_argument("--title", default="")
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=1e9)
    a = ap.parse_args()

    rows = [r for r in load_sim_csv(a.csv) if a.t0 <= r[T] <= a.t1]
    t = [r[T] for r in rows]
    truth = [r[TRUTH] for r in rows]
    # a real flight (real2plot.py) has no truth: one panel, the GNSS altitude energy (20 s) as a slow stand-in
    real = not any(math.isfinite(x) for x in truth)
    if real:
        return plot_real(a, rows, t)
    am = [r[AIRMASS_SHOWN] if int(r[STATUS]) & ST_VARIO else math.nan for r in rows]
    series = [  # label, values, color, z
        ("GNSS/INS reference", [r[REF_NETTO] for r in rows], REFERENCE, 3),
        ("TE netto", [r[TE_NETTO] for r in rows], TE, 4),
        ("AirMass", am, AIRMASS, 5),
    ]

    fig, (ax, ex) = plt.subplots(2, 1, figsize=(11, 6.6), dpi=150, sharex=True, gridspec_kw=dict(height_ratios=[2.2, 1], hspace=0.12))
    fig.patch.set_facecolor(SURFACE)
    # turning (bank over 20°) as bands
    start = None
    for r in rows + [None]:
        turning = r is not None and abs(r[ROLL]) > 20.0
        if turning and start is None:
            start = r[T]
        elif not turning and start is not None:
            for x in (ax, ex):
                x.axvspan(start, r[T] if r is not None else t[-1], color=BAND, zorder=0, lw=0)
            start = None

    ax.plot(t, truth, color=INK, lw=2.4, zorder=2, solid_capstyle="round")
    stats = {}
    for label, v, color, z in series:
        ax.plot(t, v, color=color, lw=2.0, zorder=z, solid_capstyle="round", solid_joinstyle="round")
        err = [x - y for x, y in zip(v, truth)]
        ex.plot(t, err, color=color, lw=1.6, zorder=z, solid_capstyle="round", solid_joinstyle="round")
        e = [x for x in err if not math.isnan(x)]
        stats[label] = (rms(e), max((abs(x) for x in e), default=math.nan))
    ex.axhline(0.0, color=INK_2, lw=1.0, zorder=1)

    for x, ylabel in ((ax, "vario [m/s], up positive"), (ex, "error [m/s]")):
        x.set_facecolor(SURFACE)
        x.grid(True, color=GRID, lw=0.6)
        x.set_axisbelow(True)
        for s in x.spines.values():
            s.set_visible(False)
        x.tick_params(colors=INK_2, labelsize=9, length=0)
        x.set_ylabel(ylabel, color=INK_2, fontsize=9)
    ex.set_xlabel("time [s]", color=INK_2, fontsize=9)
    ex.set_xlim(t[0], t[-1])

    legend = [Line2D([], [], color=INK, lw=2.4, label="truth: vertical air movement")]
    for label, _, color, _ in reversed(series):
        r, m = stats[label]
        legend.append(Line2D([], [], color=color, lw=2.0, label=f"{label}: error {r:.2f} m/s rms, {m:.2f} max"))
    legend.append(Patch(color=BAND, label="turning (bank over 20°)"))
    below = transforms.offset_copy(ex.transAxes, fig=fig, y=-38, units="points")
    ex.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.0, 0.0), bbox_transform=below, ncol=2, frameon=False,
              fontsize=9, labelcolor=INK, borderaxespad=0.0)
    ax.set_title(a.title, loc="left", color=INK, fontsize=12, fontweight="bold", pad=22)
    ax.text(0.0, 1.01, f"{t[0]:.0f}–{t[-1]:.0f} s, as shown: AirMass with 0.6 of the TE netto low pass, the reference with its own",
            transform=ax.transAxes, fontsize=9, color=INK_2, va="bottom")
    fig.savefig(a.png, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.25)
    print(f"{a.png}: " + ", ".join(f"{k} {r:.2f} rms / {m:.2f} max" for k, (r, m) in stats.items()))


def plot_real(a, rows, t):
    stand_in = [r[STAND_IN] if len(r) > STAND_IN else math.nan for r in rows]
    series = [  # label, values, color, z
        ("GNSS/INS reference after Larus (1 Hz GNSS)", [r[REF_NETTO] for r in rows], REFERENCE, 3),
        ("device TE netto (with the TAS/IAS polar fix)", [r[TE_NETTO] for r in rows], TE, 4),
        ("AirMass", [r[AIRMASS_SHOWN] if int(r[STATUS]) & ST_VARIO else math.nan for r in rows], AIRMASS, 5),
    ]
    fig, ax = plt.subplots(figsize=(11, 5.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    start = None
    for r in rows + [None]:
        turning = r is not None and abs(r[ROLL]) > 20.0
        if turning and start is None:
            start = r[T]
        elif not turning and start is not None:
            ax.axvspan(start, r[T] if r is not None else t[-1], color=BAND, zorder=0, lw=0)
            start = None
    ax.plot(t, stand_in, color=INK, lw=2.4, ls=(0, (4, 2)), zorder=2)
    for label, v, color, z in series:
        ax.plot(t, v, color=color, lw=2.0 if label == "AirMass" else 1.6, zorder=z, solid_capstyle="round",
                solid_joinstyle="round")
    vals = [x for _, v, _, _ in series for x in v if math.isfinite(x)] + [x for x in stand_in if math.isfinite(x)]
    lo, hi = sorted(vals)[int(0.002 * len(vals))], sorted(vals)[int(0.998 * len(vals)) - 1]  # clip the rare spikes
    ax.set_ylim(lo - 0.5, hi + 0.5)
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(colors=INK_2, labelsize=9, length=0)
    ax.set_ylabel("vario [m/s], up positive", color=INK_2, fontsize=9)
    ax.set_xlabel("time [s, UTC of day]", color=INK_2, fontsize=9)
    ax.set_xlim(t[0], t[-1])
    legend = [Line2D([], [], color=INK, lw=2.4, ls=(0, (4, 2)), label="GNSS altitude energy, 20 s average (no truth in a real flight)")]
    for label, _, color, _ in reversed(series):
        legend.append(Line2D([], [], color=color, lw=2.0, label=label))
    legend.append(Patch(color=BAND, label="turning (bank over 20°)"))
    below = transforms.offset_copy(ax.transAxes, fig=fig, y=-38, units="points")
    ax.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.0, 0.0), bbox_transform=below, ncol=2, frameon=False,
              fontsize=9, labelcolor=INK, borderaxespad=0.0)
    ax.set_title(a.title, loc="left", color=INK, fontsize=12, fontweight="bold", pad=22)
    hh = int(t[0] // 3600)
    ax.text(0.0, 1.01, f"{hh:02d}:{int(t[0] % 3600 // 60):02d} UTC, {t[-1] - t[0]:.0f} s, as shown: AirMass with 0.6 of the TE netto low pass",
            transform=ax.transAxes, fontsize=9, color=INK_2, va="bottom")
    fig.savefig(a.png, facecolor=SURFACE, bbox_inches="tight", pad_inches=0.25)
    print(f"{a.png}: real flight, {len(rows)} samples")


if __name__ == "__main__":
    main()
