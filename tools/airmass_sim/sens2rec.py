"""Convert an XCVario "$SENS" sensor log (NMEA log with $SENS raw sensor sentences, $PXCV and $GPRMC) into the AirMass
recording format (main/airmass/AirMassRecord.h), so airmass_replay runs the flight through the device's AirMass code.

  python3 tools/airmass_sim/sens2rec.py flight.nmea[.gz] out.nmea [--aux out_aux.csv] [--glider "ASG 29/18"]

The inputs are derived the way the device derives them (AirMassSensor, SetupNG calc_speeds, S2F, Flap):
  IAS from the dynamic pressure, TAS with the static pressure and OAT, the ISA altitude of the static pressure, the
  polar of the glider type at the logged ballast factor and bugs, the flap lever from the flap recommendation (no flap
  sensor in the log), the GNSS velocity with its arrival time from the $SENS GPS age field.
Differences to the device: the $SENS gyro is the processed one, with the "Gyro Gating" dead zone (rates below the gate
read 0), AirMass on the device takes the gyro without it; the start attitude is the device AHRS of $PXCV; the
estimator starts cold (no learned calibration), the IMU lever arm is taken as 0.

The aux CSV has per $SENS cycle: device time, UTC of the last fix, position, GNSS altitude, the device's TE vario
($PXCV, te_gross), its roll and pitch, IAS, TAS, ISA altitude, the polar sink at IAS and load, the flap lever, the
specific force [g, body]."""
import argparse
import gzip
import math
import re

RHO0, R_AIR, G0, P0, T0DIVL, G0DIVRXL = 1.225, 287.058, 9.80665, 101325.0, 44330.76923, 5.25588
KNOT = 0.514444

# components/glider/PolarTable.txt and FlapTable.txt
GLIDERS = {
    "ASG 29/18": dict(id=1032, wingload=42.9, polar=((82, -0.47), (132, -0.80), (184, -1.80)), area=10.5, vne=280.0,
                      flaps=(180, 142, 110, 94, 57)),
    "ASG 29/15": dict(id=1033, wingload=47.7, polar=((86, -0.58), (127, -0.80), (182, -1.80)), area=9.22, vne=285.0,
                      flaps=(198, 156, 121, 104, 60)),
}


def parse_xcv(line, logged):
    """What the device logs besides the sensors (main/protocol/nmea/XCVarioMsg.cpp): "!XCV,CA1,glider type,empty weight,
    crew weight,ballast,speed cal,QNH", since protocol version 2 "!xcv,empty-weight|crew-weight|bal-water,kg" on
    changes, and in a sensor log "!XCV,FLP,lever x 10" when the flap lever moves (FlapsBox: the flap sensor's position
    to 0.5, or without a sensor the recommendation). Updates logged (kg, lever) from the line; returns "flap", "setup"
    (glider or weights), or None if it is none"""
    f = nmea_fields(line)
    if f[0].lower() != "xcv" or len(f) < 3:
        return None
    key = {"empty-weight": "empty", "crew-weight": "crew", "bal-water": "ballast"}.get(f[1])
    if f[1] not in ("FLP", "CA1") and not key:
        return None
    try:
        if f[1] == "FLP":
            logged["flap"] = float(f[2]) / 10.0
        elif f[1] == "CA1":
            logged["type"], logged["empty"], logged["crew"], logged["ballast"] = (float(x) for x in f[2:6])
        else:
            logged[key] = float(f[2])
    except (ValueError, IndexError):
        pass
    return "flap" if f[1] == "FLP" else "setup"


def checksum(body):
    cs = 0
    for c in body[1:]:
        cs ^= ord(c)
    return f"{body}*{cs:02x}\r\n"


def fmt(v, f):
    return "" if v is None or (isinstance(v, float) and not math.isfinite(v)) else (f % v)


def sentence(name, fields):
    return checksum("$" + name + "".join("," + x for x in fields))


class Polar:
    """S2F::recalculatePolar and getSink"""

    def __init__(self, g, ballast_factor, bugs):
        (v1, w1), (v2, w2), (v3, w3) = [(v / 3.6, w) for v, w in g["polar"]]
        d = v1 * v1 * (v2 - v3) + v2 * v2 * (v3 - v1) + v3 * v3 * (v1 - v2)
        a2 = ((v2 - v3) * (w1 - w3) + (v3 - v1) * (w2 - w3)) / d
        a1 = (w2 - w3 - a2 * (v2 * v2 - v3 * v3)) / (v2 - v3)
        a0 = w3 - a2 * v3 * v3 - a1 * v3
        lf = math.sqrt(ballast_factor)
        b = (bugs + 100.0) / 100.0
        self.a0, self.a1, self.a2 = a0 * lf * b, a1 * b, a2 / lf * b
        stall = math.sqrt(2.0 * g["wingload"] * G0 / (RHO0 * 1.4)) * 1.05  # S2F default estimate
        self.v_stall = stall * math.sqrt(lf)
        self.valid = True

    def sink(self, v, n):
        if v < self.v_stall * 0.9:
            return 0.0
        sqn = math.sqrt(max(n, 0.0))
        return self.a0 * n * sqn + self.a1 * v * n + self.a2 * v * v * sqn


class Flaps:
    """Flap::prepLevels and getOptimum: the recommended lever position, fractional"""

    def __init__(self, g, ballast_factor):
        self.prep = [s / 3.6 * math.sqrt(ballast_factor) for s in g["flaps"]]
        prev = g["vne"] / 3.6
        self.delta = []
        for p in self.prep:
            self.delta.append(min(p - prev, -1.0))
            prev = p

    def optimum(self, ias, load):
        load = max(load, 0.3)
        v = ias / math.sqrt(load)
        i = 0
        while i < len(self.prep) and v <= self.prep[i]:
            i += 1
        i = min(i, len(self.prep) - 1)
        if v < 30.0 / 3.6:
            return 0.0
        return min(max(i + 0.5 + (v - self.prep[i]) / self.delta[i], -0.1), len(self.prep) - 1)


def nmea_fields(line):
    line = line.strip()
    star = line.find("*")
    body = line[1:star] if star > 0 else line[1:]
    return body.split(",")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--aux")
    ap.add_argument("--glider", default="ASG 29/18", choices=sorted(GLIDERS) + ["log"],
                    help="log: the glider type the log names (CA1) if known here, else the ASG 29/18")
    ap.add_argument("--lever-arm", type=float, default=0.0)
    ap.add_argument("--mass", type=float, default=0.0,
                    help="gross weight [kg], instead of the logged weights (CA1) or else the logged ballast factor")
    ap.add_argument("--no-flaps", action="store_true", help="no flap lever (one pitch offset for all flap positions)")
    ap.add_argument("--flap-fractional", action="store_true", help="the recommendation unrounded (the device rounds it)")
    ap.add_argument("--bugs", type=float, default=-1.0, help="polar degradation [%%], instead of the logged bugs setting")
    a = ap.parse_args()
    logged = dict(type=math.nan, empty=math.nan, crew=math.nan, ballast=math.nan, flap=math.nan)
    g = None

    opener = gzip.open if a.log.endswith(".gz") else open
    out = open(a.out, "w")
    aux = open(a.aux, "w") if a.aux else None
    if aux:
        aux.write("t_ms,utc_s,lat,lon,gnss_alt,gnss_speed,gnss_track,te_vario,roll_deg,pitch_deg,ias,tas,alt_isa,polar_sink,flap,running,"
                  "fx,fy,fz\n")

    pxcv = dict(vario=math.nan, bugs=0.0, ballast=1.0, roll=0.0, pitch=0.0)
    fix = None            # the last $GPRMC: utc seconds of day, speed, track, lat, lon
    gnss_alt = math.nan
    pending_fix = None    # a fix not yet passed to the estimator (its arrival time comes with the next $SENS)
    running, above, below = False, 0, 0
    setup_line, setup_count, starts, frames, fixes = None, 0, 0, 0, 0
    polar = flaps = None

    def setup():
        nonlocal polar, flaps, g
        # the glider and the weights as logged so far
        name = next((n for n, x in GLIDERS.items() if x["id"] == logged["type"]), "ASG 29/18") if a.glider == "log" else a.glider
        g = GLIDERS[name]
        logged_mass = logged["empty"] + (logged["crew"] if logged["crew"] > 0 else 0) + (logged["ballast"] if logged["ballast"] > 0 else 0) \
            if logged["empty"] > 0 else math.nan
        ref = g["wingload"] * g["area"]
        bf = a.mass / ref if a.mass > 0 else logged_mass / ref if logged_mass > 0 else (pxcv["ballast"] if pxcv["ballast"] > 0.5 else 1.0)
        polar, flaps = Polar(g, bf, a.bugs if a.bugs >= 0 else pxcv["bugs"]), Flaps(g, bf)
        mass = bf * ref
        return sentence("PXCVC", [fmt(a.lever_arm, "%.4f"), fmt(mass, "%.2f"), fmt(g["area"], "%.4f"), fmt(polar.a0, "%.8g"),
                                  fmt(polar.a1, "%.8g"), fmt(polar.a2, "%.8g"), fmt(polar.v_stall, "%.4f"), "1", "1.000", "1.000",
                                  "100.000"])

    for raw in opener(a.log, "rt", errors="replace"):
        xcv = parse_xcv(raw, logged) if raw.startswith("!") else None
        if xcv:
            # a new glider or weight in flight (water ballast dumped): the setup the device sends from now on
            if xcv == "setup" and running:
                setup_line = setup()
            continue
        if raw.startswith("$PXCV,"):
            f = nmea_fields(raw)
            try:
                pxcv["vario"] = float(f[1])
                pxcv["bugs"] = float(f[3] or 0)
                pxcv["ballast"] = float(f[4]) if f[4] else pxcv["ballast"]
                pxcv["roll"] = float(f[10]) if len(f) > 11 and f[10] else pxcv["roll"]
                pxcv["pitch"] = float(f[11]) if len(f) > 11 and f[11] else pxcv["pitch"]
            except (ValueError, IndexError):
                pass
        elif raw.startswith("$GPRMC,") or raw.startswith("$GNRMC,"):
            f = nmea_fields(raw)
            if len(f) > 8 and f[2] == "A" and f[7]:
                try:
                    hh, mm, ss = int(f[1][0:2]), int(f[1][2:4]), float(f[1][4:])
                    lat = (int(f[3][:2]) + float(f[3][2:]) / 60.0) * (1 if f[4] == "N" else -1)
                    lon = (int(f[5][:3]) + float(f[5][3:]) / 60.0) * (1 if f[6] == "E" else -1)
                    spd = float(f[7]) * KNOT
                    trk = float(f[8]) if f[8] else math.nan
                    fix = dict(utc=hh * 3600 + mm * 60 + ss, speed=spd, track=trk, lat=lat, lon=lon)
                    pending_fix = fix
                except ValueError:
                    pass
        elif raw.startswith("$GPGGA,") or raw.startswith("$GNGGA,"):
            f = nmea_fields(raw)
            try:
                gnss_alt = float(f[9])
            except (ValueError, IndexError):
                pass
        elif raw.startswith("$SENS,"):
            f = nmea_fields(raw)
            if len(f) < 13:
                continue
            try:
                t_ms = int(round(float(f[1]) * 1000.0))
                age = int(f[2])
                p, q, oat = float(f[3]) * 100.0, float(f[5]), float(f[6])
                acc = [float(x) for x in f[7:10]]
                gyro = [math.radians(float(x)) for x in f[10:13]]
            except ValueError:
                continue
            ias = math.sqrt(2.0 * q / RHO0) if q > 0.0 else 0.0
            tas = ias * math.sqrt(RHO0 * R_AIR * (oat + 273.15) / p) if p > 0 else 0.0
            alt = T0DIVL * (1.0 - (p / P0) ** (1.0 / G0DIVRXL)) if p > 0 else math.nan
            gnss_speed = fix["speed"] if fix else 0.0

            # airborne: flying speed and moving over ground for 10 s; landed: 30 s slow
            if not running:
                above = above + 1 if (ias > 20.0 and gnss_speed > 15.0) else 0
                if above >= 100:
                    running = True
                    starts += 1
                    setup_line = setup()
                    out.write(setup_line)
                    rr, pp = math.radians(pxcv["roll"]), math.radians(pxcv["pitch"])
                    down = (-math.sin(pp), math.sin(rr) * math.cos(pp), math.cos(rr) * math.cos(pp))
                    warm = ["1.00000", "0.000000", "0.0000", "1.00000"] + ["1.000000"] * 7  # cold, flap offsets unknown
                    out.write(sentence("PXCVS", [str(t_ms), fmt(alt, "%.2f")] + [fmt(x, "%.6f") for x in down] +
                                       [fmt(x, "%.6f") for x in gyro] + warm))
                    setup_count = 0
            else:
                below = below + 1 if ias < 10.0 else 0
                if below >= 300:
                    running, below, above = False, 0, 0
                    out.write(sentence("PXCVX", [str(t_ms)]))
            flap, sink = math.nan, math.nan
            if running:
                load = 1.0 / max(math.cos(math.radians(pxcv["roll"])), 0.3)
                opt = flaps.optimum(ias, load)
                # the lever as logged (FLP), else the recommendation
                flap = math.nan if a.no_flaps else logged["flap"] if math.isfinite(logged["flap"]) \
                    else (opt if a.flap_fractional else float(round(opt)))
                sink = polar.sink(ias, max(-acc[2], 0.3))
                setup_count += 1
                if setup_count % 100 == 0:
                    out.write(setup_line)  # like the device, every 10 s
                out.write(sentence("PXCVR", [str(t_ms)] + [fmt(x, "%.5f") for x in acc] + [fmt(x, "%.6f") for x in gyro] +
                                   [fmt(ias, "%.3f"), fmt(tas, "%.3f"), fmt(alt, "%.2f"), fmt(flap, "%.3f")]))
                frames += 1
                # the fix arrived age ms before this cycle; the estimator gets it in this cycle (AirMassSensor polls)
                if pending_fix is not None and pending_fix["speed"] > 1.0 and math.isfinite(pending_fix["track"]):
                    trk = math.radians(pending_fix["track"])
                    vn, ve = pending_fix["speed"] * math.cos(trk), pending_fix["speed"] * math.sin(trk)
                    out.write(sentence("PXCVG", [str(t_ms - age), fmt(vn, "%.3f"), fmt(ve, "%.3f"), "0.500", "750", "0",
                                                 "0.000", "0.000"]))
                    fixes += 1
                pending_fix = None
            if aux:
                fx = fix or {}
                aux.write(",".join([str(t_ms), fmt(fx.get("utc"), "%.1f"), fmt(fx.get("lat"), "%.6f"), fmt(fx.get("lon"), "%.6f"),
                                    fmt(gnss_alt, "%.1f"), fmt(fx.get("speed"), "%.2f"), fmt(fx.get("track"), "%.1f"),
                                    fmt(pxcv["vario"], "%.2f"), fmt(pxcv["roll"], "%.1f"), fmt(pxcv["pitch"], "%.1f"),
                                    fmt(ias, "%.2f"), fmt(tas, "%.2f"), fmt(alt, "%.1f"), fmt(sink, "%.3f"), fmt(flap, "%.0f"),
                                    "1" if running else "0"] + [fmt(x, "%.4f") for x in acc]) + "\n")
    if running:
        out.write(sentence("PXCVX", [str(t_ms)]))
    out.close()
    if aux:
        aux.close()
    print(f"{a.out}: {starts} start(s), {frames} cycles ({frames / 36000.0:.2f} h), {fixes} GNSS fixes")


if __name__ == "__main__":
    main()
