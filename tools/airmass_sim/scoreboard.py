"""Scoreboard of the AirMass estimator against TE netto and the GNSS/INS reference (ins_reference.h), lower is better.
Every metric of the synthetic flights, NL day and segment scenarios averaged over 6 noise seeds; LOSS marks a metric
where TE netto or the reference does better. Run after tools/airmass_sim/run.sh built build_sim, from the repo root:
  docker run --rm --entrypoint "" -v "$PWD":/project -w /project espressif/idf:release-v5.4 python3 -I tools/airmass_sim/scoreboard.py [sim args]"""
import re
import subprocess
import sys

B = "build_sim/airmass_sim"
extra = sys.argv[1:]


def run(args):
    return subprocess.run([B] + args, capture_output=True, text=True).stdout.replace("\n", " ")


def grab(t, p):
    m = re.search(p, t)
    return [float(x) for x in m.groups()] if m else None


rows = []  # (name, ours, te, ref)


def add_means(prefix, acc):
    """one row per metric: the mean over the seeds of (ours, TE, reference)"""
    for k, xs in acc.items():
        mean = tuple(None if xs[0][i] is None else sum(x[i] for x in xs) / len(xs) for i in range(3))
        rows.append((f"{prefix}: {k}", *mean))


def nl(variant, seeds=6):
    acc = {}
    for s in range(1, seeds + 1):
        t = run(["nl", "--seed", str(s)] + variant + extra)
        v = {}
        c = grab(t, r"cruise rms ([0-9.]+) / ([0-9.]+)")
        f = grab(t, r"false lift events ([0-9]+) / ([0-9]+)")
        e = grab(t, r"thermal entry rms ([0-9.]+) / ([0-9.]+) \(max ([0-9.]+) / ([0-9.]+)")
        ci = grab(t, r"circling rms ([0-9.]+) / ([0-9.]+)")
        cl = grab(t, r"thermals\): rms ([0-9.]+) / ([0-9.]+)")
        lag = grab(t, r"display lag ([0-9.]+) / ([0-9.]+)")
        r = grab(t, r"reference \(GNSS/INS TE netto, 10 Hz GNSS\): cruise rms ([0-9.]+), false lift events ([0-9]+), thermal entry rms ([0-9.]+) "
                    r"\(max ([0-9.]+)\), circling rms ([0-9.]+), climb reading rms ([0-9.]+) \(max [0-9.]+\), display lag ([0-9.]+)")
        av = grab(t, r"30s average ([0-9.]+) m/s off")
        mw = grab(t, r"averaged wind off the mean wind ([0-9.]+) / ([0-9.]+)")
        iv = grab(t, r"instant ([0-9.]+) m/s off the 5s")
        ia = grab(t, r"instant over the whole flight ([0-9.]+)")
        rw = grab(t, r"reference wind \(where ours is valid / whole flight\): instant ([0-9.]+) / ([0-9.]+), 30s average ([0-9.]+) / ([0-9.]+) m/s, "
                     r"averaged off the mean wind ([0-9.]+) / ([0-9.]+)")
        v["cruise rms"] = (c[0], c[1], r[0])
        v["false lift /h"] = (f[0], f[1], r[1])
        v["entry rms"] = (e[0], e[1], r[2])
        v["entry max"] = (e[2], e[3], r[3])
        v["circling rms"] = (ci[0], ci[1], r[4])
        v["climb per thermal"] = (cl[0], cl[1], r[5])
        v["display lag s"] = (lag[0], lag[1], r[6])
        v["wind instant (valid)"] = (iv[0], None, rw[0])
        v["wind instant (all)"] = (ia[0], None, rw[1])
        v["wind 30s avg"] = (av[0], None, rw[2])
        v["avg vs mean, straight"] = (mw[0], None, rw[4])
        v["avg vs mean, circling"] = (mw[1], None, rw[5])
        for k, x in v.items():
            acc.setdefault(k, []).append(x)
    name = "nl " + " ".join(variant)
    add_means(name, acc)


def segment(sc, seeds=6):
    acc = {}
    for seed in range(1, seeds + 1):
        t = run([sc, "--seed", str(seed)] + extra)
        m = grab(t, r"wind rms ([0-9.]+) m/s \(max [0-9.]+\), vario rms ([0-9.na]+) m/s \(TE netto ([0-9.na]+)\), spike max ([0-9.]+) m/s \(TE netto ([0-9.]+)\)")
        r = grab(t, r"reference \(GNSS/INS\): wind rms ([0-9.]+) m/s, averaged wind rms ([0-9.na]+), vario rms ([0-9.na]+), spike max ([0-9.]+)")
        a = grab(t, r"averaged wind rms ([0-9.]+) m/s \(max")
        v = {"wind rms": (m[0], None, r[0])}
        if a:
            v["averaged wind rms"] = (a[0], None, r[1])
        if m[1] == m[1]:
            v["steady vario rms"] = (m[1], m[2], r[2])
        if m[3] > 0 or m[4] > 0:
            v["spike max"] = (m[3], m[4], r[3])
        for k, x in v.items():
            acc.setdefault(k, []).append(x)
    add_means(sc, acc)


nl([])
for sc in ["straight", "thermal", "gust", "pullup", "slip", "endurance"]:
    segment(sc)

losses = 0
print(f"{'metric':52s} {'ours':>7s} {'TE':>7s} {'ref':>7s}")
for name, o, te, rf in rows:
    others = [x for x in (te, rf) if x is not None]
    lose = [x for x in others if o > x + 1e-9]
    tie = [x for x in others if abs(o - x) <= 1e-9]
    mark = "LOSS" if lose else ("tie" if tie else "")
    losses += bool(lose)
    f = lambda x: "-" if x is None else f"{x:.3f}"
    print(f"{name:52s} {f(o):>7s} {f(te):>7s} {f(rf):>7s}  {mark}")
print(f"losses: {losses} of {len(rows)}")
