#!/usr/bin/env python3
import os
import sys
import argparse
import serial
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import statistics
import numpy as np
#from bisect import bisect_left
from enum import Enum, auto

# --- Konfiguration ---
BAUDRATE = 115200
FLIGHT_DATA_DIR = os.getenv("FLIGHT_DATA", "${HOME}/flight_data")
WO_GPS = False
IS_ENU = False

# --- Konstanten ---
G = 9.80665
R = 287.05
L = 0.0065
T0 = 288.15
ISA_EXP = G / (R * L)

# Nur diese Sätze weiterleiten (zB PXCV ignorieren)
ALLOWED_PREFIXES = {"$GPGGA", "$GPGSA", "$GPRMC", "$PGRMZ", "$PFLAU", "$PFLAA", "$PFLAL", "$SENS", "!XCV"}
#ALLOWED_PREFIXES = {"$GPGGA", "$GPGSA", "$GPRMC", "$GPRMZ", "$PFLAU", "$PFLAA", "$PFLAL"}

lock = threading.Lock()
seek_generation = 0
gps_offset = 0

# --- Erstes .nmea File finden ---
def find_first_nmea_file(flight):
    directory = os.path.expandvars(os.path.join(FLIGHT_DATA_DIR, flight))
    for file in sorted(os.listdir(directory)):
        if file.lower().endswith(".nmea") and not "weg" in file:
            return os.path.join(directory, file)
    return None

#Original :
#$SENS,3242.043,2908723,955.400,955.395,27.352,17.44,-0.1967,-0.1073,0.8429,-1.4391,0.0000,4.6402,1185.0690,-1188.3779,7553.8076
#
#Converted:
#$SENS,3242.043,2908723,955.400,955.395,27.352,17.44,0.1967,-0.1073,0.8429,-1.4391,-0.0000,-4.6402,1185.0690,1188.3779,-7553.8076q
def flip_25_to_ned(line: str) -> str:
    parts = line.strip().split(',')
    if len(parts) < 8 or parts[0] != '$SENS':
        return line
    #fixed = parts[:7] # Erste 7 Felder bleiben gleich (Index 0 bis 6)
    # Ab Index 7: Vorzeichen nach dieser maske umkehren
    to_flip = [False, True, True, False, True, True , False, True, True]
    flipped = []
    for i in range(7, len(parts)):
        val = parts[i]
        if to_flip[i-7] == True:
            num = -float(val)
            flipped.append(str(num))
        else:
            flipped.append(val)  # falls kein Float → unverändert
    #flipped = [f"{ -float(v):.4f}" if v.lstrip('-+').replace('.','').isdigit() else v
    #           for v in parts[7:]]
    return ','.join(parts[:7] + flipped)

# --- NMEA-Zeilen laden und mit Zeitstempeln versehen ---



def parse_gps_time(value):
    """NMEA HHMMSS.sss -> Sekunden seit UTC-Mitternacht."""
    if not value:
        return None

    try:
        hh = int(value[0:2])
        mm = int(value[2:4])
        ss = float(value[4:])

        if not (0 <= hh < 24 and 0 <= mm < 60 and 0 <= ss < 60):
            return None

        return hh * 3600 + mm * 60 + ss

    except (ValueError, IndexError):
        return None


def parse_nmea_position(value, direction, is_latitude):
    """NMEA ddmm.mmmm / dddmm.mmmm -> Dezimalgrad."""
    if not value or not direction:
        return None

    try:
        digits = 2 if is_latitude else 3

        degrees = int(value[:digits])
        minutes = float(value[digits:])

        if not 0 <= minutes < 60:
            return None

        if is_latitude:
            if degrees > 90 or direction not in ("N", "S"):
                return None
        else:
            if degrees > 180 or direction not in ("E", "W"):
                return None

        result = degrees + minutes / 60.0

        if direction in ("S", "W"):
            result = -result

        return result

    except (ValueError, IndexError):
        return None


from statistics import median


def repair_sens_timestamps(records, cycle_time=0.1, jump_threshold=0.5, verbose=False ):
    """
    Repariert Uhrensprünge in den $SENS-Zeitstempeln.

    Die ursprünglichen Zeitstempel bleiben in
    record["sens_time_raw"] erhalten.

    Der korrigierte Zeitstempel steht in
    record["vario_time"].

    Die Korrektur erfolgt durch einen kumulativen Offset.

    Parameter:
        cycle_time:
            Erwartete Dauer eines Sensorzyklus.

        jump_threshold:
            Ab dieser Abweichung wird ein Uhrensprung
            angenommen. Kleine Lücken bleiben erhalten.

    Rückgabe:
        Liste der erkannten Korrekturen.
    """

    corrections = []

    correction = 0.0
    previous_raw = None
    previous_corrected = None

    # Typischer Abstand zwischen benachbarten SENS-Sätzen.
    # Hier sammeln wir nur offensichtlich plausible Abstände.
    recent_intervals = []

    for index, record in enumerate(records):

        raw = record.get("vario_time")

        if raw is None:
            continue

        record["sens_time_raw"] = raw

        if previous_raw is None:

            record["vario_time"] = raw
            record["time_correction"] = 0.0

            previous_raw = raw
            previous_corrected = raw

            continue

        dt_raw = raw - previous_raw

        # ---------------------------------------------
        # Normalen Zeitabstand abschätzen
        # ---------------------------------------------

        if recent_intervals:
            expected_dt = median(recent_intervals)
        else:
            expected_dt = cycle_time

        # ---------------------------------------------
        # Uhrensprung erkennen
        # ---------------------------------------------

        if dt_raw < 0 or dt_raw > jump_threshold:

            # Wir nehmen an, dass zwischen diesen beiden
            # SENS-Sätzen mindestens ein Sensorzyklus
            # vergangen ist.
            #
            # Bei fehlenden SENS-Zeilen ist die tatsächliche
            # Zeitspanne möglicherweise größer.

            dt_reconstructed = expected_dt

            # Neue Korrektur:
            #
            # raw - correction =
            # previous_corrected + dt_reconstructed

            correction = (raw - previous_corrected - dt_reconstructed)

            event = {
                "index": index,
                "raw_time": raw,
                "raw_delta": dt_raw,
                "reconstructed_delta": dt_reconstructed,
                "correction": correction,
            }

            corrections.append(event)

            if verbose:
                print(f"SENS jump at record {index:7d}: " f"dt={dt_raw:+12.3f} s, "
                    f"reconstructed={dt_reconstructed:.3f} s, " f"correction={correction:+12.3f} s")

        elif 0 < dt_raw < 0.25:
            # Nur reguläre Intervalle für die Abschätzung
            # des nächsten Sensorzyklus verwenden.
            recent_intervals.append(dt_raw)
            if len(recent_intervals) > 100:
                recent_intervals.pop(0)

        # ---------------------------------------------
        # Korrigierte Zeit berechnen
        # ---------------------------------------------

        corrected = raw - correction
        record["vario_time"] = corrected
        record["time_correction"] = correction
        previous_raw = raw
        previous_corrected = corrected

    if verbose:
        print()
        print("SENS timestamp repair:")
        print(f"  Corrections: {len(corrections)}")

        sens_times = [
            r["vario_time"]
            for r in records
            if r.get("sens_time_raw") is not None
        ]

        if sens_times:
            duration = sens_times[-1] - sens_times[0]
            print(f"  Reconstructed duration: {duration:.1f} s")
            print(f"  Final correction:       {correction:+.3f} s")

    return corrections


def assign_vario_times(records):
    """
    Ordnet allen Datensätzen eine Variometer-Zeit zu.

    $SENS besitzt einen eigenen Zeitstempel.
    Andere Datensätze erhalten eine interpolierte Zeit anhand
    ihrer Position zwischen benachbarten $SENS-Datensätzen.

    Vor dem ersten und nach dem letzten $SENS wird die
    jeweilige Randzeit verwendet.
    """

    sens_indices = [
        i for i, r in enumerate(records)
        if r["vario_time"] is not None
    ]

    if not sens_indices:
        raise ValueError("Keine gültigen $SENS-Zeitstempel gefunden")

    # Vor dem ersten SENS
    first = sens_indices[0]
    first_time = records[first]["vario_time"]

    for i in range(first):
        records[i]["vario_time"] = first_time

    # Zwischen zwei SENS-Datensätzen
    for i0, i1 in zip(sens_indices, sens_indices[1:]):
        t0 = records[i0]["vario_time"]
        t1 = records[i1]["vario_time"]

        if t1 <= t0:
            raise ValueError(
                f"Nicht monotone SENS-Zeit bei Logindex {i1}: "
                f"{t0} -> {t1}"
            )

        # Gleichmäßige Verteilung der dazwischenliegenden Sätze.
        # Das ist eine Näherung, keine gemessene Empfangszeit.
        for i in range(i0 + 1, i1):
            fraction = (i - i0) / (i1 - i0)
            records[i]["vario_time"] = t0 + fraction * (t1 - t0)

    # Nach dem letzten SENS
    last = sens_indices[-1]
    last_time = records[last]["vario_time"]

    for i in range(last + 1, len(records)):
        records[i]["vario_time"] = last_time


def synchronize_gps_time(records):
    """
    Bestimmt den konstanten Offset zwischen GPS- und Variometer-Uhr.

    Rückgabe:
        offset, number_of_inliers, median_absolute_residual

    GPS-Zeiten werden nur für gültige Positions-Fixes verwendet.
    """
    candidates = []

    for r in records:
        if r["gps_time"] is None:
            continue

        if r["lat"] is None or r["lon"] is None:
            continue

        candidates.append(r)

    if not candidates:
        print("GPS synchronization: no valid GPS samples")
        return None, 0, None

    # GPS-Zeit entfalten: Tageswechsel berücksichtigen.
    #
    # Wichtig: Bei einem Fehler der GPS-Uhr kann es einen
    # beliebigen Zeitsprung geben. Deshalb erfolgt anschließend
    # noch eine robuste Offset-Auswahl.

    day_offset = 0.0
    previous_tod = None

    for r in candidates:
        tod = r["gps_time"]

        if previous_tod is not None:
            # Nur einen großen Rücksprung als Mitternacht interpretieren.
            if tod < previous_tod - 43200:
                day_offset += 86400.0

        r["gps_unwrapped"] = tod + day_offset
        previous_tod = tod

    # Kandidaten für den Offset.
    offsets = [r["gps_unwrapped"] - r["vario_time"] for r in candidates]

    # Ein einfaches Median-Verfahren funktioniert gut, solange
    # mehr als die Hälfte der GPS-Zeitwerte korrekt ist.
    offset = statistics.median(offsets)

    residuals = [abs(x - offset) for x in offsets]
    mad = statistics.median(residuals)

    # Toleranz gegen Zuordnungsfehler und Zeitquantisierung.
    #
    # Die GPS-Sätze haben keine eigene Variometer-Zeit.
    # Deshalb ist die Synchronisierung nicht beliebig genau.
    tolerance = max(0.5, 5.0 * mad)

    inliers = [
        x for x in offsets
        if abs(x - offset) <= tolerance
    ]

    if not inliers:
        return None, 0, None

    offset = statistics.median(inliers)

    final_residuals = [abs(x - offset) for x in inliers]
    error = statistics.median(final_residuals)

    print(f"GPS/Vario offset:       {offset:+.3f} s")
    print(f"GPS samples:            {len(candidates)}")
    print(f"GPS synchronization:    {len(inliers)} inliers")
    print(f"GPS rejected:           {len(candidates) - len(inliers)}")
    print(f"Median abs. residual:   {error:.3f} s")

    return offset, len(inliers), error



def nmea_checksum(sentence):
    """XOR über alle Zeichen zwischen !/$ und *."""
    body = sentence[1:].split("*", 1)[0]

    checksum = 0
    for c in body:
        checksum ^= ord(c)

    return f"{checksum:02X}"


def create_ca1(cmn, qnh):
    """
    CMN:
        !XCV,CMN,2305,324,100,0,0

    QNH:
        Pascal
    """
    fields = cmn.strip().split("*", 1)[0].split(",")

    if len(fields) != 7 or fields[:2] != ["!XCV", "CMN"]:
        raise ValueError(f"Invalid CMN: {cmn}")

    fields[1] = "CA1"
    body = ",".join(fields) + f",{qnh:.2f}"
    return f"{body}*{nmea_checksum(body)}"


def reconstruct_ca1(records, cmn, reference_window=20.0):
    """
    Rekonstruiert QNH aus GPS-Höhe, statischem Druck
    und Außentemperatur.

    records:
        Zeitlich synchronisierte NMEA-Datensätze.
        Benötigt 'timestamp' und 'line'.

    cmn:
        Manueller Konfigurationssatz.

    reference_window:
        Zeitfenster am Anfang zur Bestimmung der
        Starthöhe in Sekunden.

    Rückgabe:
        (ca1, info)
    """

    sens_t = []
    sens_p = []
    sens_T = []

    gps_t = []
    gps_h = []

    # --------------------------------------------------
    # 1. Messdaten extrahieren
    # --------------------------------------------------

    for record in records:
        line = record["line"]
        t = record["timestamp"]

        if t is None or not math.isfinite(t):
            continue

        fields = line.split(",")

        try:
            if line.startswith("$SENS,"):
                p = float(fields[3]) * 100.0 # hPa
                T = float(fields[6]) + 273.15 # K

                if (math.isfinite(p) and math.isfinite(T) and 30000 < p < 110000 and 200 < T < 330):
                    sens_t.append(t)
                    sens_p.append(p)
                    sens_T.append(T)

            elif line.startswith("$GPGGA,"):
                if int(fields[6]) == 0:
                    continue

                h = float(fields[9])

                if math.isfinite(h):
                    gps_t.append(t)
                    gps_h.append(h)

        except (ValueError, IndexError):
            continue

    if len(sens_t) < 2 or len(gps_t) < 10:
        raise ValueError("Insufficient SENS/GPGGA samples")

    sens_t = np.asarray(sens_t, dtype=np.float64)
    sens_p = np.asarray(sens_p, dtype=np.float64)
    sens_T = np.asarray(sens_T, dtype=np.float64)

    gps_t = np.asarray(gps_t, dtype=np.float64)
    gps_h = np.asarray(gps_h, dtype=np.float64)

    # --------------------------------------------------
    # 2. Druck und Temperatur auf GPS-Zeitpunkte
    #    interpolieren
    # --------------------------------------------------

    valid = ((gps_t >= sens_t[0]) & (gps_t <= sens_t[-1]))
    gps_t = gps_t[valid]
    gps_h = gps_h[valid]

    p = np.interp(gps_t, sens_t, sens_p)
    T = np.interp(gps_t, sens_t, sens_T)

    # --------------------------------------------------
    # 3. Temperaturprofil T(h) bestimmen
    #
    #    Linearer Fit:
    #        T(h) = a + b*h
    # --------------------------------------------------

    b, a = np.polyfit(gps_h, T, 1)

    # Temperaturgradient begrenzen
    b = np.clip(b, -0.012, 0.003)

    # --------------------------------------------------
    # 4. Starthöhe bestimmen
    # --------------------------------------------------

    t_start = gps_t[0]

    start_mask = gps_t <= t_start + reference_window

    h_start = median(gps_h[start_mask])

    # --------------------------------------------------
    # 5. Alle Druckwerte auf Starthöhe reduzieren
    #
    #    Hydrostatische Gleichung:
    #
    #    dp/p = -g/(R*T(h)) dh
    # --------------------------------------------------

    T_start = a + b * h_start

    if abs(b) < 1e-6:
        p_start = p * np.exp(G * (gps_h - h_start) / (R * T_start))
    else:
        p_start = p * (T / T_start) ** (G / (R * b))

    # --------------------------------------------------
    # 6. Robusten Referenzdruck bestimmen
    # --------------------------------------------------

    p_ref = median(p_start)

    # --------------------------------------------------
    # 7. QNH nach ISA
    # --------------------------------------------------

    isa_factor = (1.0 - L * h_start / T0) ** ISA_EXP

    qnh = p_ref / isa_factor

    # --------------------------------------------------
    # 8. Qualitätsinformation
    # --------------------------------------------------

    residuals = p_start - p_ref
    sigma_p = 1.4826 * median(np.abs(residuals))
    ca1 = create_ca1(cmn, qnh)

    info = {
        "qnh": qnh,
        "qnh_hpa": qnh / 100.0,
        "start_altitude": h_start,
        "reference_pressure": p_ref,
        "temperature_gradient": b,
        "pressure_sigma": sigma_p,
        "samples": len(p_start),
    }

    print(
        f"QNH reconstructed: {qnh / 100:.2f} hPa\n"
        f"Start altitude:     {h_start:.1f} m\n"
        f"Temperature lapse:  {b * 1000:.2f} K/km\n"
        f"Pressure scatter:   {sigma_p:.2f} Pa\n"
        f"Samples:            {len(p_start)}\n"
        f"Generated CA1:      {ca1}"
    )

    return ca1, info


def load_flight_data(filepath, verbose=False):
    global gps_offset
    records = []
    cmn = None
    xcv_config = None

    # ---------------------------------------------------------
    # 1. Log vollständig einlesen
    # ---------------------------------------------------------

    with open(filepath, "r", errors="ignore") as f:
        for raw_line in f:

            line = raw_line.rstrip("\r\n")

            if not line:
                continue

            # Eventuell vorangestellte Log-Ausgaben entfernen.
            index = max(line.rfind("$"), line.rfind("!"))

            if index < 0:
                continue

            line = line[index:]
            line = line.replace(";", ",", 1)

            # Manueller XCV Konfigurations Ersatz
            if line.startswith("!XCV,CMN,"):
                cmn = line
                continue

            # XCV-Konfiguration erkennen
            if line.startswith("!XCV,CA1,"):
                xcv_config = line
                continue

            fields = line.split(",")
            prefix = fields[0]

            if prefix not in ALLOWED_PREFIXES:
                continue

            record = {
                "line": line,
                "timestamp": None,
                "vario_time": None,
                "gps_time": None,
                "gps_unwrapped": None,
                "lat": None,
                "lon": None,
            }

            try:
                # ---------------------------------------------
                # Variometer-Zeit
                # ---------------------------------------------

                if prefix == "$SENS":

                    record["vario_time"] = float(fields[1])

                    if IS_ENU:
                        record["line"] = flip_25_to_ned(line)

                # ---------------------------------------------
                # GPS-Zeit und Position
                # ---------------------------------------------

                elif prefix in ("$GPGGA", "$GPRMC"):

                    gps_time = parse_gps_time(fields[1])

                    if prefix == "$GPGGA":
                        valid_fix = int(fields[6]) > 0

                        lat_field = fields[2]
                        lat_dir = fields[3]
                        lon_field = fields[4]
                        lon_dir = fields[5]

                    else:
                        valid_fix = fields[2] == "A"

                        lat_field = fields[3]
                        lat_dir = fields[4]
                        lon_field = fields[5]
                        lon_dir = fields[6]

                    if valid_fix:
                        record["lat"] = parse_nmea_position(lat_field, lat_dir, True)
                        record["lon"] = parse_nmea_position(lon_field, lon_dir, False)
                        if (record["lat"] is not None and record["lon"] is not None):
                            record["gps_time"] = gps_time

            except (ValueError, IndexError) as e:
                if verbose:
                    print(f"Parse error: {line}: {e}")

            records.append(record)

    if not records:
        return []

    # ---------------------------------------------------------
    # 1.5 Uhrensprünge der SENS-Zeitstempel reparieren
    # ---------------------------------------------------------

    corrections = repair_sens_timestamps(records)

    # ---------------------------------------------------------
    # 2. Gemeinsame Variometer-Zeitachse erstellen
    # ---------------------------------------------------------

    assign_vario_times(records)

    # ---------------------------------------------------------
    # 3. GPS-Uhr synchronisieren
    # ---------------------------------------------------------

    if not WO_GPS:
        gps_offset, gps_count, gps_error = synchronize_gps_time(records)
    else:
        gps_offset = 0

    # ---------------------------------------------------------
    # 4. Relative Zeit für den Player
    # ---------------------------------------------------------

    time_ref = records[0]["vario_time"]

    for r in records:
        r["timestamp"] = r["vario_time"] - time_ref

    duration = records[-1]["timestamp"]

    print(f"Log records:            {len(records)}")
    print(f"Log duration:           {duration:.1f} s")

    utc_start = time_ref + gps_offset

    hh = int(utc_start // 3600) % 24
    mm = int(utc_start % 3600 // 60)
    ss = utc_start % 60

    print(f"Log start UTC:          {hh:02d}:{mm:02d}:{ss:02f}")

    # ---------------------------------------------------------
    # 5. Rekonstruiere einen CA1 Satz
    # ---------------------------------------------------------

    if xcv_config is None and cmn is not None:
        xcv_config, flight_info = reconstruct_ca1(records, cmn)

    return records, xcv_config


class FlightState(Enum):
    GROUND = auto()
    TAKEOFF_CANDIDATE = auto()
    FLIGHT = auto()
    LANDING_CANDIDATE = auto()


def detect_flights(records, takeoff_pressure=50.0, landing_pressure=35.0,
    takeoff_duration=40.0, landing_duration=10.0,
    max_sample_gap=2.0, verbose=True):
    """
    Erkennt Start und Landung anhand des dynamischen Drucks.

    Erwartete Datenstruktur:
        records[i]["line"]       NMEA-Datensatz
        records[i]["timestamp"]  korrigierte Zeit in Sekunden

    Dynamischer Druck:
        $SENS,...,p_dyn,...
        p_dyn = fields[5], in Pascal

    Rückgabe:
        Liste von Flügen mit Start-/Landeindex und Zeitstempeln.

    Hinweise:
        - Hysterese durch unterschiedliche Druckschwellen.
        - Bestätigung über tatsächliche Zeitdauer.
        - Fehlende Sensorwerte werden toleriert.
        - Große Datenlücken unterbrechen die Bestätigung.
        - Ein am Logende noch laufender Flug bleibt offen.
    """

    flights = []

    state = FlightState.GROUND

    takeoff_index = None
    takeoff_time = None

    landing_index = None
    landing_time = None

    candidate_index = None
    candidate_time = None

    previous_time = None

    for index, record in enumerate(records):

        line = record["line"]

        if not line.startswith("$SENS,"):
            continue

        try:
            fields = line.split(",")
            p_dyn = float(fields[5])
            timestamp = float(record["timestamp"])

        except (ValueError, IndexError, KeyError, TypeError):
            continue

        if not (math.isfinite(p_dyn) and math.isfinite(timestamp)):
            continue

        # --------------------------------------------------
        # Zeitlücken erkennen
        # --------------------------------------------------

        if previous_time is not None:

            dt = timestamp - previous_time
            if dt < 0:
                raise ValueError(
                    f"Non-monotonic timestamp at record {index}"
                )

            if dt > max_sample_gap:
                # Eine laufende Bestätigung wird ungültig.
                # Einen bereits erkannten Flug behalten wir.
                if state == FlightState.TAKEOFF_CANDIDATE:
                    state = FlightState.GROUND
                elif state == FlightState.LANDING_CANDIDATE:
                    state = FlightState.FLIGHT

                candidate_index = None
                candidate_time = None

        previous_time = timestamp

        # --------------------------------------------------
        # GROUND
        # --------------------------------------------------

        if state == FlightState.GROUND:

            if p_dyn >= takeoff_pressure:
                candidate_index = index
                candidate_time = timestamp
                state = FlightState.TAKEOFF_CANDIDATE

        # --------------------------------------------------
        # TAKEOFF_CANDIDATE
        # --------------------------------------------------

        elif state == FlightState.TAKEOFF_CANDIDATE:

            if p_dyn < takeoff_pressure:
                state = FlightState.GROUND
                candidate_index = None
                candidate_time = None

            elif timestamp - candidate_time >= takeoff_duration:
                takeoff_index = candidate_index
                takeoff_time = candidate_time
                state = FlightState.FLIGHT

                if verbose:
                    print(f"Takeoff: index={takeoff_index}, time={takeoff_time:.1f} s")

        # --------------------------------------------------
        # FLIGHT
        # --------------------------------------------------

        elif state == FlightState.FLIGHT:

            if p_dyn <= landing_pressure:
                candidate_index = index
                candidate_time = timestamp
                state = FlightState.LANDING_CANDIDATE

        # --------------------------------------------------
        # LANDING_CANDIDATE
        # --------------------------------------------------

        elif state == FlightState.LANDING_CANDIDATE:

            if p_dyn > landing_pressure:

                state = FlightState.FLIGHT
                candidate_index = None
                candidate_time = None

            elif timestamp - candidate_time >= landing_duration:

                landing_index = candidate_index
                landing_time = candidate_time

                flight = {
                    "takeoff_index": takeoff_index,
                    "landing_index": landing_index,
                    "takeoff_time": takeoff_time,
                    "landing_time": landing_time,
                    "duration": landing_time - takeoff_time,
                }
                flights.append(flight)

                if verbose:
                    duration = landing_time - takeoff_time
                    print(f"Landing: index={landing_index}, time={landing_time:.1f} s, duration={duration / 60:.1f} min")

                state = FlightState.GROUND
                takeoff_index = None
                takeoff_time = None

                candidate_index = None
                candidate_time = None

    # ------------------------------------------------------
    # Flug am Ende der Aufzeichnung noch aktiv?
    # ------------------------------------------------------

    if state in (FlightState.FLIGHT, FlightState.LANDING_CANDIDATE):
        flight = {
            "takeoff_index": takeoff_index,
            "landing_index": None,
            "takeoff_time": takeoff_time,
            "landing_time": None,
            "duration": None,
        }

        flights.append(flight)

        if verbose:
            print(f"Flight still active at end of log: takeoff={takeoff_time:.1f} s")

    if verbose:
        print(f"Detected flights: {len(flights)}")

    return flights


def playback_worker():
    global playback_time, current_index
    was_plaing = False
    speed = 1.0
    start_time = time.time()
    last_time = start_time
    num_sens = 0
    idx_inc = 0
    max_sens = 11
    smax = [11, 11, 5, 3, 2, 1]
    while True:
        time.sleep(0.01)  # Sehr kurze Sleep → reagiert schnell

        with lock:
            # Wichtig: Index-Inkrement nur hier unter Lock
            generation = seek_generation
            current_index += idx_inc
            idx = current_index
            if speed != playback_speed:
                was_plaing = False
            if not is_playing or current_index >= len(flight_data):
                was_plaing = False
                continue
            speed = playback_speed

        idx_inc = 0
        if is_playing:
            max_sens = smax[int(speed)]
            timestamp = flight_data[idx]['timestamp']
            now = time.time()
            if not was_plaing:
                last_time = now
                ser.write(("$PFLAX,S," + str(speed) + "\r\n").encode("ascii"))
            delta = now - last_time
            with lock:
                playback_time += delta * speed
            last_time = now
            while playback_time >= timestamp:
                with lock:
                    if generation != seek_generation:
                        idx_inc = 0
                        break

                if not flight_data[idx]['line'].startswith("$SENS") or num_sens < max_sens:
                    ser.write((flight_data[idx]['line'] + "\r\n").encode("ascii", errors="ignore"))
                    #print(now, playback_time, ": ", flight_data[idx]['line'])
                if flight_data[idx]['line'].startswith("$SENS"):
                    num_sens += 1
                elif flight_data[idx]['line'].startswith("$GPRMC"):
                    num_sens = 0
                idx_inc += 1
                idx += 1
                if idx >= len(flight_data):
                    break
                timestamp = flight_data[idx]['timestamp']

            was_plaing = True

# --- Matplotlib Visualisierung ---
def setup_plot():
    global current_index

    lats = [e['lat'] for e in flight_data if e['lat'] is not None]
    lons = [e['lon'] for e in flight_data if e['lon'] is not None]
    valid_entries = (len(lats)+len(lons))/2

    if not lats:
        print("Keine Positionsdaten zum Plotten.")
        return

    print(f"min/max Lat {min(lats):.2f}/{max(lats):.2f}")
    print(f"min/max Lon {min(lons):.2f}/{max(lons):.2f}")
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(lons, lats, 'b-', linewidth=1, alpha=0.6, label="Flugspur")

    # --- Roter Punkt viel auffälliger machen ---
    current_point, = ax.plot([], [], marker='o', markersize=14,
                             markerfacecolor='red', markeredgecolor='yellow',
                             markeredgewidth=3, label="Aktuelle Position")

    ax.set_xlabel("Längengrad(°E)")
    ax.set_ylabel("Breitengrad(°N)")
    ax.set_title(f"Flugtrack: {os.path.basename(nmea_file)} ({valid_entries } Punkte)")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.text(0.015, 0.98,
             "Steuerung:\n"
             "Leertaste     → Play / Pause\n"
             "+             → schneller\n"
             "-             → langsamer\n"
             "← →           → 10 Sekunden zurück/vor\n"
             "↑ ↓           → 60 Sekunden\n"
             "Pos1 / Ende   → Anfang / Ende\n"
             "Q             → Beenden",
             fontsize=9.5,
             fontfamily='monospace',
             verticalalignment='top',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8f9fa',
                       edgecolor='#444444', alpha=0.92))
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Platz für Text oben lassen

    # Status-Text (wird später aktualisiert)
    status_text = ax.text(
        0.02, 0.02,  # unten links (in Axes-Koordinaten 0..1)
        "PAUSE   1.0×",
        transform=ax.transAxes,
        fontsize=12,
        fontweight='bold',
        color='white',
        bbox=dict(facecolor='black', alpha=0.7, edgecolor='gray', boxstyle='round,pad=0.4'),
        verticalalignment='bottom'
    )

    def update(frame):
        global current_index, is_playing, playback_speed

        with lock:
            idx = current_index
            playing = is_playing
            speed = playback_speed

            if 0 <= idx < len(flight_data):
                e = flight_data[idx]
                if e['lat'] is not None and e['lon'] is not None:
                    current_point.set_data([e['lon']], [e['lat']])
                    # --- Kurzer Schweif der letzten 5 Positionen ---
 
        # Status aktualisieren
        status = "PLAY" if playing else "PAUSE"
        status_text.set_text(f"{status}   {speed:.1f}×")

        return current_point, status_text

    ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)

    # --- Neue Sprung-Routine ---
    def jump_by_seconds(seconds):
        global current_index, playback_time, seek_generation

        new_idx = 0
        target_time = playback_time + seconds
        if target_time < flight_data[0]['timestamp']:
            target_time = flight_data[0]['timestamp']
        elif target_time > flight_data[-1]['timestamp']:
            target_time = flight_data[-1]['timestamp']

        # Finde nächsten Index >= target_time
        for i in range(0, len(flight_data)):
            if flight_data[i]['timestamp'] >= target_time and flight_data[i]['line'].startswith("$GPRMC"):
                new_idx = i
                break
        else:
            new_idx = len(flight_data) - 1

        #with lock:
        current_index = new_idx
        playback_time = flight_data[current_index]['timestamp']
        seek_generation += 1
        print(f"Sprung zu Index {current_index}, Zeit {playback_time:.3f}s")

    def jump_to_index(idx):
        global current_index, playback_time, seek_generation

        current_index = idx
        playback_time = flight_data[current_index]['timestamp']
        seek_generation += 1
        print(f"Sprung zu Index {current_index}, Zeit {playback_time:.3f}s")

    def on_key(event):
        global is_playing, playback_speed, current_index, start_idx, end_idx
        changed = False
        with lock:
            old_index = current_index
            if event.key == ' ':  # Leertaste
                is_playing = not is_playing
            elif event.key == '+':
                playback_speed = min(5.0, playback_speed + 1)
                print(f"Geschwindigkeit: {playback_speed:.1f}x")
            elif event.key == '-':
                playback_speed = max(1.0, playback_speed - 1)
                print(f"Geschwindigkeit: {playback_speed:.1f}x")
            elif event.key == 'right':
                jump_by_seconds(10)
            elif event.key == 'left':
                jump_by_seconds(-10)
            elif event.key == 'up':
                jump_by_seconds(60)
            elif event.key == 'down':
                jump_by_seconds(-60)
            elif event.key == 'home':
                jump_to_index(start_idx)
            elif event.key == 'end':
                jump_to_index(end_idx)
            elif event.key == 'q':
                plt.close()
                return

            #status = "PLAY" if is_playing else "PAUSE"
            #print(f"Index: {current_index}/{len(flight_data)-1} | {status} | {playback_speed:.1f}x")
            if current_index != old_index:
                changed = True

        # Sofort-Update erzwingen, wenn Index geändert
        if changed:
            # Manuelles Update des Plots
            #fig.canvas.draw_idle()
            #fig.canvas.flush_events()
            # Oder noch direkter:
            update(None)  # Ruft deine update-Funktion einmal manuell auf

    fig.canvas.mpl_connect('key_press_event', on_key)
    print("\n=== STEUERUNG ===")
    print("Leertaste    → Play / Pause")
    print("+            → schneller")
    print("-            → langsamer")
    print("← →          → 10 Sekunden zurück/vor")
    print("↑ ↓          → 60 Sekunden")
    print("Pos1 / Ende  → Anfang / Ende")
    print("Q            → Beenden")
    print("=====================\n")

    plt.show()


def send_initial_configuration(serio, xcv_config):
    serio.write("$PFLAX,S*3C\n".encode("ascii"))
    time.sleep(1.0)
    if xcv_config:
        serio.write((xcv_config + "\r\n").encode("ascii"))
        print(f"Initial configuration: {xcv_config}")


# --- Start ---+++++----
if __name__ == "__main__":
    # Initialize the parser
    parser = argparse.ArgumentParser(description="Read data from a serial device.")
    # Pflicht-Argument (Positional Argument)
    parser.add_argument("directory", type=str, help="Directory name, which contains the NMEA log")
    parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB1", help="Serial port path (e.g., /dev/ttyUSB1)")
    parser.add_argument("-e", "--enforce", action="store_true", default=False, help="Enforce loading flight data w/o GPS data")

    args = parser.parse_args()
    flight_dir = args.directory
    IS_ENU =  flight_dir.startswith("25") or flight_dir.startswith("24")
    SERIAL_PORT = args.port
    WO_GPS = args.enforce

    # --- Serielle Schnittstelle ---
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Serial port {SERIAL_PORT} geöffnet.")
    except Exception as e:
        print(f"Fehler beim Öffnen von {SERIAL_PORT}: {e}")
        sys.exit(1)

    nmea_file = find_first_nmea_file(flight_dir)
    if not nmea_file:
        print(f"Kein .nmea-File in {flight_dir} gefunden.")
        ser.close()
        sys.exit(1)

    print(f"Lade Flugdatei: {nmea_file}")


    flight_data, xcv_config = load_flight_data(nmea_file)
    if not flight_data:
        print("Keine gültigen NMEA-Daten gefunden.")
        ser.close()
        sys.exit(1)

    print(f"{len(flight_data)} Datensätze geladen.")
    print(f"{flight_data[-1]['timestamp']/3600:.2f} Stunden Logdaten.")

    flights = detect_flights(flight_data)
    print(f"{len(flights)} {'Flug' if len(flights) == 1 else 'Flüge'} gefunden")
    i = 1
    for f in flights:
        utc = f["takeoff_time"] + gps_offset
        hh = int(utc // 3600) % 24
        mm = int(utc % 3600 // 60)
        ss = int(utc % 60)
        print(f"Start  UTC:             {hh:02d}:{mm:02d}:{ss:02d}")
        utc = f["landing_time"] + gps_offset
        hh = int(utc // 3600) % 24
        mm = int(utc % 3600 // 60)
        ss = int(utc % 60)
        print(f"Land. UTC:              {hh:02d}:{mm:02d}:{ss:02d}")

    # --- Playback Steuerung ---
    current_index = 0
    is_playing = False
    playback_speed = 1.0
    playback_time = 0.0
    # pic by default the first flight
    start_idx = 0
    end_idx = len(flight_data) - 1
    if (len(flights) > 0):
        start_idx = flights[0]["takeoff_index"]
        end_idx = flights[0]["landing_index"]

    lock = threading.Lock()

    # XCVario config übertragen
    send_initial_configuration(ser, xcv_config)

    # Playback-Thread starten
    thread = threading.Thread(target=playback_worker, daemon=True)
    thread.start()

    # GUI starten (blockiert bis Fenster geschlossen)
    try:
        setup_plot()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\nBeendet.")