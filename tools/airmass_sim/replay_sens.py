"""Replay XCVario $SENS logs through AirMass, flight after flight like the device: each flight starts with the
calibration the previous one learned (airspeed scale, pitch offset, AoA scale, GNSS latency).

  python3 tools/airmass_sim/replay_sens.py out_dir log1.nmea.gz log2.nmea.gz ... [--mass kg] [--glider "ASG 29/18"]

Per flight in out_dir: <name>.rec (the recording, sens2rec.py), <name>.aux.csv, <name>.out.csv (airmass_replay --csv),
<name>.ref.csv (the GNSS/INS reference, ref_replay)."""
import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPLAY = os.path.join(HERE, "..", "..", "build_sim", "airmass_replay")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--mass", default="0")
    ap.add_argument("--glider", default="ASG 29/18")
    ap.add_argument("--cold", action="store_true", help="every flight starts without learned calibration")
    ap.add_argument("--no-flaps", action="store_true", help="converter: no flap lever")
    ap.add_argument("--flap-fractional", action="store_true", help="converter: the flap recommendation unrounded")
    ap.add_argument("--lever-arm", default="0", help="converter: IMU ahead of the CG [m]")
    ap.add_argument("--bugs", default="-1", help="converter: polar degradation [%%]")
    ap.add_argument("--set", action="append", default=[], help="replay: estimator setting name=value")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    warm = None
    for log in sorted(a.logs, key=os.path.basename):
        name = os.path.basename(log).split(".")[0]
        base = os.path.join(a.out_dir, name)
        conv = subprocess.run([sys.executable, "-I", os.path.join(HERE, "sens2rec.py"), log, base + ".rec", "--aux", base + ".aux.csv",
                               "--mass", a.mass, "--glider", a.glider, "--bugs", a.bugs, "--lever-arm", a.lever_arm] + (["--no-flaps"] if a.no_flaps else []) + (["--flap-fractional"] if a.flap_fractional else []), capture_output=True, text=True)
        if conv.returncode:
            print(name, "conversion failed:", conv.stderr.strip()[-300:])
            continue
        cmd = [REPLAY, base + ".rec", "--csv", base + ".out.csv"] + [x for s in a.set for x in ("--set", s)]
        if warm and not a.cold:
            cmd += ["--warm", warm]
        rep = subprocess.run(cmd, capture_output=True, text=True)
        # the GNSS/INS reference on the same flight (needs AirMass's heading)
        subprocess.run([os.path.join(os.path.dirname(REPLAY), "ref_replay"), base + ".aux.csv", base + ".out.csv", base + ".ref.csv"],
                       capture_output=True, text=True)
        m = re.search(r"k_tas ([0-9.]+), pitch ofs ([-0-9.]+)°, aoa scale ([0-9.]+), gnss latency ([0-9]+) ms", rep.stdout)
        if m:
            k, thb, kcl, lat = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
            warm = f"{k:.4f},{thb * 3.14159265 / 180.0:.5f},{kcl:.4f},{(lat - 750.0) / 1000.0:.3f}"
        first = rep.stdout.splitlines()[0] if rep.stdout else rep.stderr.strip()
        print(f"{name}: {conv.stdout.strip().split(': ', 1)[-1]} | {first.split(': ', 1)[-1]}")
        print(f"   learned: k_tas {m.group(1)}, pitch ofs {m.group(2)}°, aoa scale {m.group(3)}, latency {m.group(4)} ms" if m else "   (no result)")


if __name__ == "__main__":
    main()
