"""Convert an XCVario "$SENS" sensor log (NMEA log with $SENS raw sensor sentences, $PXCV and $GPRMC) into the AirMass
recording format (main/airmass/AirMassRecord.h), so airmass_replay runs the flight through the device's AirMass code.

  python3 tools/airmass_sim/sens2rec.py flight.nmea[.gz] out.nmea [--aux out_aux.csv] [--glider "ASG 29/18"]

The inputs are derived the way the device derives them (AirMassSensor, SetupNG calc_speeds, S2F, Flap):
  IAS from the dynamic pressure, TAS with the static pressure and OAT, the ISA altitude of the static pressure, the
  polar of the glider type at the logged ballast factor and bugs, the flap lever from the flap recommendation (no flap
  sensor in the log), the GNSS velocity with its arrival time from the $SENS GPS age field.
Where the device recorded its AirMass start ($PXCVS), setup ($PXCVC) and output ($PXCVA) with the sensor data, the
estimator starts there with the device's attitude and calibration, with its setup while no glider, mass or bugs are
given, and airmass_replay compares itself with the output.
Kelvin OAT is recognized by agreement with PXCV Celsius; inconsistent legacy FMT axes are corrected only when
stationary gravity and pitch-rate checks against PXCV attitude both support the repair.
Missing or implausible OAT (outside -60..50°C) is estimated as 25°C at sea level minus 6.5°C/km, using GNSS altitude
or pressure altitude if GNSS altitude is unavailable, like the browser converter.
Differences to the device: the $SENS gyro is the processed one, with the "Gyro Gating" dead zone (rates below the gate
read 0), AirMass on the device takes the gyro without it; the start attitude is the device AHRS of $PXCV; the
estimator starts cold (no learned calibration), the IMU lever arm is taken as 0.

The aux CSV has per $SENS cycle: device time, UTC of the last fix, position, GNSS altitude, the device's TE vario
($PXCV, te_gross), its roll and pitch, IAS, TAS, ISA altitude, the polar sink at IAS and load, the flap lever, the
specific force [g, body]."""
import argparse
import gzip
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "web"))
from make_gliders import gliders  # noqa: E402

RHO0, R_AIR, G0, P0, T0DIVL, G0DIVRXL = 1.225, 287.058, 9.80665, 101325.0, 44330.76923, 5.25588
KNOT = 0.514444


def plausible_temperature(oat):
    return math.isfinite(oat) and -60 <= oat <= 50


def estimated_temperature(alt):
    """25°C at sea level, standard 6.5°C/km lapse rate (GNSS altitude, else pressure altitude)."""
    return 25 - 0.0065 * (alt if math.isfinite(alt) else 0)


# components/glider/PolarTable.txt and FlapTable.txt, as the browser converter has them (gliders.js)
GLIDERS = gliders()


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


def clean_line(raw):
    """A logger's text before the sentence, and the ';' separators of old $SENS logs"""
    line = raw if raw[:1] in ("$", "!") else raw[max(raw.rfind("$"), raw.rfind("!")):] if max(raw.rfind("$"), raw.rfind("!")) >= 0 else ""
    return line.replace(";", ",") if line.startswith("$SENS;") else line


class SensorConventions:
    """Validate legacy metadata against PXCV attitude/temperature; mirrored in web/convert.js."""
    def __init__(self):
        self.sensor = self.prev = None
        self.kelvin = self.temperature_pairs = self.ground = self.rates = 0
        self.gravity_declared = self.gravity_corrected = self.rate_declared = self.rate_corrected = 0.0
        self.quiet_gravity = self.gravity_matches = self.force_matches = 0

    @staticmethod
    def number(f, i):
        try:
            return float(f[i])
        except (ValueError, IndexError):
            return math.nan

    def sens(self, f):
        t = self.number(f, 1) / (1 if len(f) > 1 and "." in f[1] else 1000)
        a = [self.number(f, i) for i in (7, 8, 9)]
        g = [math.radians(self.number(f, i)) for i in (10, 11, 12)]
        q = self.number(f, 5)
        self.sensor = dict(t=t, q=q, oat=self.number(f, 6), a=a, g=g) if all(math.isfinite(v) for v in [t, q] + a + g) else None

    def pxcv(self, f):
        s = self.sensor
        if s is None:
            return
        temp = self.number(f, 6)
        if plausible_temperature(s["oat"] - 273.15) and plausible_temperature(temp):
            self.temperature_pairs += 1
            self.kelvin += abs(s["oat"] - 273.15 - temp) < 2
        roll, pitch = (math.radians(self.number(f, i)) for i in (10, 11))
        if not all(math.isfinite(v) for v in (roll, pitch)):
            return
        norm = math.sqrt(sum(v*v for v in s["a"]))
        rate = math.sqrt(sum(v*v for v in s["g"]))
        if (s["q"] < 50 and 0.85 < norm < 1.15 and rate < math.radians(3)
                and abs(roll) < math.pi / 6 and abs(pitch) < math.pi / 7 and abs(math.sin(pitch)) > 0.05):
            force = [math.sin(pitch), -math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.cos(pitch)]
            for j in range(3):
                self.gravity_declared += (s["a"][j] / norm * (1 if j == 0 else -1) - force[j])**2
                self.gravity_corrected += (-s["a"][j] / norm - force[j])**2
            self.ground += 1
        dt = s["t"] - self.prev[0] if self.prev else 0
        # Infer missing metadata only from repeated, quiet low-speed attitude agreement plus gyro-rate evidence.
        if (.15 < dt < .3 and s["q"] < 50 and .85 < norm < 1.15 and abs(roll) < math.pi / 6
                and abs(pitch) < math.pi / 7 and abs(math.sin(pitch)) > .05
                and abs(roll - self.prev[2]) / dt < math.pi / 180 and abs(pitch - self.prev[1]) / dt < math.pi / 180):
            force = [math.sin(pitch), -math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.cos(pitch)]
            gravity_error = sum((-s["a"][j] / norm - force[j])**2 for j in range(3))
            force_error = sum((s["a"][j] / norm - force[j])**2 for j in range(3))
            self.quiet_gravity += 1
            self.gravity_matches += gravity_error < .01
            self.force_matches += force_error < .01
        if 0.15 < dt < 0.3 and abs(roll) < math.pi / 3 and abs(pitch) < math.pi / 6 and rate < 1.4:
            measured = (pitch - self.prev[1]) / dt
            predicted = s["g"][1]*math.cos(roll) - s["g"][2]*math.sin(roll)
            if abs(measured) < math.pi / 6:
                self.rate_declared += (-predicted - measured)**2
                self.rate_corrected += (predicted - measured)**2
                self.rates += 1
        self.prev = (s["t"], pitch, roll)

    def apply(self, out):
        out["temperature_kelvin"] = self.kelvin >= 20 and self.kelvin >= self.temperature_pairs * 0.8
        if (out["fmt"] == (7, [1, -1, -1, 1, -1, -1]) and self.ground >= 50 and self.rates >= 100
                and self.gravity_corrected / self.ground < 0.01
                and self.gravity_corrected < self.gravity_declared * 0.5
                and self.rate_corrected < self.rate_declared * 0.5):
            out["fmt"] = (7, [-1, -1, -1, 1, 1, 1])
            out["axes_corrected"] = True
        elif (out["fmt"] is None and self.quiet_gravity >= 50 and self.gravity_matches >= self.quiet_gravity * .8
                and self.force_matches < self.quiet_gravity * .1 and self.rates >= 100
                and self.rate_corrected < self.rate_declared * .5):
            out["fmt"] = (7, [-1, -1, -1, 1, 1, 1])
            out["axes_inferred"] = True


def scan_log(lines):
    """One pass through the log before the conversion, for what applies to all of it wherever it stands:
    "!XCV,FMT,SENS,start,factors...": the $SENS fields from index start on (SENS itself is 0) are multiplied by the
      factors (logs before 2025: east-north-up axes)
    "!XCV,CMN,glider type,empty weight,crew weight,ballast,speed cal": CA1 without the QNH; a CA1 in the log wins
    the QNH [hPa]: of CA1, else of $PXCV, else from the GNSS altitude and static pressure of the first fixes on the ground"""
    out = dict(fmt=None, cmn=None, ca1=None, qnh=math.nan, qnh_from="")
    pairs = []
    conventions = SensorConventions()
    p, dyn, pxcv = math.nan, math.nan, math.nan

    def number(f, i):
        try:
            return float(f[i])
        except (ValueError, IndexError):
            return math.nan

    for raw in lines:
        line = clean_line(raw.strip())
        if line.startswith("!XCV,"):
            f = nmea_fields(line)
            try:
                if f[1] == "FMT" and f[2] == "SENS":
                    out["fmt"] = (int(f[3]), [float(x) for x in f[4:]])
                elif f[1] == "CMN":
                    out["cmn"] = [float(x) for x in f[2:6]]
                elif f[1] == "CA1":
                    if out["ca1"] is None:
                        values = [float(x) for x in f[2:6]]
                        if len(values) == 4 and all(math.isfinite(x) for x in values):
                            out["ca1"] = values
                    q = float(f[7])
                    q = q / 100.0 if q > 8e4 else q  # Pa, or hPa
                    if 800 < q < 1100:
                        out["qnh"], out["qnh_from"] = q, "CA1"
            except (ValueError, IndexError):
                pass
        elif line.startswith("$PXCV,"):
            conventions.pxcv(nmea_fields(line))
            if not math.isfinite(pxcv) and number(nmea_fields(line), 7) > 800:
                pxcv = number(nmea_fields(line), 7)
        elif line.startswith("$SENS,"):
            f = nmea_fields(line)
            conventions.sens(f)
            p, dyn = number(f, 3), number(f, 5)
        elif len(pairs) < 20 and (line.startswith("$GPGGA,") or line.startswith("$GNGGA,")):
            f = nmea_fields(line)
            # on the ground only: high up the air is too far from ISA (a log started at 3500 m came out 24 hPa high)
            if number(f, 6) > 0 and math.isfinite(number(f, 9)) and p > 300 and dyn < 150:
                pairs.append((p, number(f, 9)))
    conventions.apply(out)
    if out["qnh_from"]:
        return out
    if math.isfinite(pxcv):
        out["qnh"], out["qnh_from"] = pxcv, "$PXCV"
    elif pairs:
        median = lambda a: sorted(a)[len(a) >> 1]
        isa = (1.0 - median([x[1] for x in pairs]) / T0DIVL) ** G0DIVRXL
        out["qnh"], out["qnh_from"] = median([x[0] for x in pairs]) / isa, "GNSS altitude and pressure"
    return out


class StampRepair:
    """Keep forward sensor gaps confirmed by elapsed GNSS time, allowing two seconds for fix cadence/arrival.
    Other large jumps (UTC synchronization) and backward jumps (midnight/reboot) continue at the sensor cycle.
    Frozen timestamps use measured increments of an independently advancing, stale GNSS-age counter.
    Mirrored in web/convert.js for offline and serial replay."""

    def __init__(self):
        self.last, self.last_utc, self.correction = None, math.nan, 0
        self.last_raw, self.last_age, self.recovered, self.recovering = None, math.nan, False, False

    def __call__(self, raw, utc=math.nan, age=math.nan):
        self.recovered = False
        if self.last is not None:
            delta = raw - self.correction - self.last
            elapsed_utc = ((utc - self.last_utc) % 86400) * 1000
            sensor_gap = math.isfinite(elapsed_utc) and abs(delta - elapsed_utc) <= 2000
            age_delta = age - self.last_age
            age_ticks = age > 2000 and self.last_age > 2000 and 0 < age_delta <= 2000
            self.recovered = delta == 0 and raw == self.last_raw and age_ticks
            if self.recovered or (self.recovering and raw != self.last_raw and age_ticks):
                self.correction = raw - self.last - age_delta
            elif delta < 0 or (delta > 5000 and not sensor_gap):
                self.correction = raw - self.last - 100
            self.recovering = self.recovered or (self.recovering and raw == self.last_raw)
        self.last_raw, self.last_utc, self.last_age = raw, utc, age
        self.last = raw - self.correction
        return self.last


def checksum(body):
    cs = 0
    for c in body[1:]:
        cs ^= ord(c)
    return f"{body}*{cs:02x}\r\n"


def damaged(line):
    """A logged sentence a logger damaged: its checksum fails, or without one another sentence starts inside it"""
    star = line.find("*")
    if star < 0:
        return "$" in line[1:] or "!" in line[1:]
    return checksum(line[:star])[star + 1:].strip() != line[star + 1:].strip().lower()


def fmt(v, f):
    return "" if v is None or (isinstance(v, float) and not math.isfinite(v)) else (f % v)


def sentence(name, fields):
    return checksum("$" + name + "".join("," + x for x in fields))


class Polar:
    """S2F::recalculatePolar and airmass::Polar::sink (main/airmass/AirMassPolar.h)"""

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
    ap.add_argument("--glider", default="ASG 29/18", choices=sorted(GLIDERS) + ["log"], metavar="NAME",
                    help="a glider type of components/glider/PolarTable.txt, or log: the glider type the log names (CA1) "
                         "if known, else the ASG 29/18")
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
    last_ms = 0
    pending_fix = None    # a fix not yet passed to the estimator (its arrival time comes with the next $SENS)
    running, above, below = False, 0, 0
    setup_line, setup_count, starts, frames, fixes = None, 0, 0, 0, 0
    # the setup the device recorded ($PXCVC), used while no glider, mass or bugs are set here
    device_setup = None
    use_device_setup = a.glider == "log" and not a.mass > 0 and a.bugs < 0
    temperature_estimated_samples = 0
    temperature_kelvin_samples = 0
    timestamp_recovered_samples = 0
    polar = flaps = None
    with opener(a.log, "rt", errors="replace") as log:
        lines = list(log)
    scan = scan_log(lines)
    if scan.get("axes_corrected"):
        print("Corrected inconsistent legacy FMT: gravity-vector acceleration and unchanged body gyros (verified against PXCV attitude)")
    if scan.get("axes_inferred"):
        print("Inferred missing sensor axes: gravity-vector acceleration and unchanged body gyros (verified against PXCV attitude)")
    monotone = StampRepair()
    initial_setup = scan["ca1"] or scan["cmn"]
    if initial_setup is not None and len(initial_setup) == 4:
        logged["type"], logged["empty"], logged["crew"], logged["ballast"] = initial_setup
    if scan["qnh_from"]:
        print(f"QNH {scan['qnh']:.1f} hPa ({scan['qnh_from']})")

    def setup():
        nonlocal polar, flaps, g
        # the glider and the weights as logged so far
        name = next((n for n, x in GLIDERS.items() if x["id"] == logged["type"]), None) if a.glider == "log" else a.glider
        if name is None:
            if math.isfinite(logged["type"]):
                print(f"warning: glider type {logged['type']:.0f} of the log is not known here, using the ASG 29/18")
            name = "ASG 29/18"
        g = GLIDERS[name]
        logged_mass = logged["empty"] + (logged["crew"] if logged["crew"] > 0 else 0) + (logged["ballast"] if logged["ballast"] > 0 else 0) \
            if logged["empty"] > 0 else math.nan
        ref = g["wingload"] * g["area"]
        bf = a.mass / ref if a.mass > 0 else logged_mass / ref if logged_mass > 0 else (pxcv["ballast"] if pxcv["ballast"] > 0.5 else 1.0)
        polar, flaps = Polar(g, bf, a.bugs if a.bugs >= 0 else pxcv["bugs"]), Flaps(g, bf) if g.get("flaps") else None
        mass = bf * ref
        own = sentence("PXCVC", [fmt(a.lever_arm, "%.4f"), fmt(mass, "%.2f"), fmt(g["area"], "%.4f"), fmt(polar.a0, "%.8g"),
                                 fmt(polar.a1, "%.8g"), fmt(polar.a2, "%.8g"), fmt(polar.v_stall, "%.4f"), "1", "1.000", "1.000",
                                 "100.000"])
        return device_setup if use_device_setup and device_setup else own

    for original in lines:
        raw = clean_line(original)
        xcv = parse_xcv(raw, logged) if raw.startswith("!") else None
        if xcv:
            # a new glider or weight in flight (water ballast dumped): the setup the device sends from now on
            if xcv == "setup" and running:
                setup_line = setup()
            continue
        # The device's AirMass records in a sensor log (System > Logging "All Sensor Data"): the estimator starts where
        # and how the device started it (attitude, calibration), with its setup, and the replay checks itself against
        # the device's output. Its GNSS velocities ($PXCVG) are left out, the navigation sentences give them.
        if raw[:6] in ("$PXCVS", "$PXCVC", "$PXCVA", "$PXCVH", "$PXCVX") and raw[6:7] == ",":
            line, kind = raw.strip(), raw[5]
            if damaged(line):
                continue
            if kind == "C":
                device_setup = line + "\r\n"
                if running and use_device_setup:
                    setup_line = device_setup
                    out.write(setup_line)
            elif kind == "S":
                running = True
                starts += 1
                above = below = setup_count = 0
                setup_line = setup()
                out.write(setup_line)
                out.write(line + "\r\n")
            elif running:
                out.write(line + "\r\n")
                if kind == "X":
                    running, above, below = False, 0, 0
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
                if scan["fmt"]:
                    start, factors = scan["fmt"]
                    for i, c in enumerate(factors):
                        if start + i < len(f) and f[start + i]:
                            f[start + i] = float(f[start + i]) * c + 0.0  # + 0.0: no negative zero
                stamp = int(round(float(f[1]) * 1000.0)) if "." in f[1] else int(f[1])  # old logs: integer milliseconds
                raw_age = age = int(f[2])
                age = age if 0 <= age <= 2000 else 0  # old logs also hold other values here: the fix counts as just arrived
                p, q = float(f[3]) * 100.0, float(f[5])
                acc = [float(x) for x in f[7:10]]
                gyro = [math.radians(float(x)) for x in f[10:13]]
            except ValueError:
                continue
            try:
                oat = float(f[6])
            except ValueError:
                oat = math.nan
            if scan["temperature_kelvin"] and plausible_temperature(oat - 273.15):
                oat -= 273.15
                temperature_kelvin_samples += 1
            t_ms = monotone(stamp, fix["utc"] if fix else math.nan, raw_age)
            before_ms, last_ms = last_ms, t_ms
            if monotone.recovered:
                timestamp_recovered_samples += 1
            ias = math.sqrt(2.0 * q / RHO0) if q > 0.0 else 0.0
            alt = T0DIVL * (1.0 - (p / P0) ** (1.0 / G0DIVRXL)) if p > 0 else math.nan
            if not plausible_temperature(oat):
                oat = estimated_temperature(gnss_alt if math.isfinite(gnss_alt) else alt)
                temperature_estimated_samples += 1
            tas = ias * math.sqrt(RHO0 * R_AIR * (oat + 273.15) / p) if p > 0 else 0.0
            # Airspeed detects flight without recorded GNSS, including a headwind/low ground speed.
            # Require 100 flying-speed samples; landed: 300 slow samples.
            if not running:
                above = above + 1 if ias > 20.0 else 0
                if above >= 100:
                    running = True
                    starts += 1
                    setup_line = setup()
                    out.write(setup_line)
                    rr, pp = math.radians(pxcv["roll"]), math.radians(pxcv["pitch"])
                    down = (-math.sin(pp), math.sin(rr) * math.cos(pp), math.cos(rr) * math.cos(pp))
                    warm = ["1.00000", "0.000000", "0.7500", "1.00000"] + ["1.000000"] * 7 + ["2"]  # cold, flap offsets unknown, format 2
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
                opt = flaps.optimum(ias, load) if flaps else math.nan
                # the lever as logged (FLP), else the recommendation
                flap = math.nan if a.no_flaps else logged["flap"] if math.isfinite(logged["flap"]) \
                    else (opt if a.flap_fractional or not flaps else float(round(opt)))
                sink = polar.sink(ias, max(-acc[2], 0.3))
                setup_count += 1
                if setup_count % 100 == 0:
                    out.write(setup_line)  # like the device, every 10 s
                out.write(sentence("PXCVR", [str(t_ms)] + [fmt(x, "%.5f") for x in acc] + [fmt(x, "%.6f") for x in gyro] +
                                   [fmt(ias, "%.3f"), fmt(tas, "%.3f"), fmt(alt, "%.2f"), fmt(flap, "%.3f")]))
                frames += 1
                # the fix arrived age ms before this cycle, but after the cycle before: the log has fixes ahead of a cycle
                # whose age still counts from the fix before; the estimator gets it in this cycle (AirMassSensor polls)
                if pending_fix is not None and pending_fix["speed"] > 1.0 and math.isfinite(pending_fix["track"]):
                    trk = math.radians(pending_fix["track"])
                    vn, ve = pending_fix["speed"] * math.cos(trk), pending_fix["speed"] * math.sin(trk)
                    out.write(sentence("PXCVG", [str(max(t_ms - age, before_ms)), fmt(vn, "%.3f"), fmt(ve, "%.3f"), "0.500", "750", "0",
                                                 "0.000", "0.000", str(round(pending_fix["utc"] * 1000) % 86400000)]))
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
    if temperature_kelvin_samples:
        print(f"OAT converted from Kelvin for {temperature_kelvin_samples} sensor samples (verified against PXCV Celsius)")
    if temperature_estimated_samples:
        print(f"OAT estimated for {temperature_estimated_samples} sensor samples: 25°C at sea level, −6.5°C/km")
    if timestamp_recovered_samples:
        print(f"Timing recovered for {timestamp_recovered_samples} samples with frozen timestamps (advancing GNSS-age counter)")
    print(f"{a.out}: {starts} start(s), {frames} cycles ({frames / 36000.0:.2f} h), {fixes} GNSS fixes")


if __name__ == "__main__":
    main()
