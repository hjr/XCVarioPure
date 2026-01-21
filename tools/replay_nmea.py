#!/usr/bin/env python3
import os
import sys
import argparse
import serial
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Konfiguration ---
BAUDRATE = 115200
FLIGHT_DATA_DIR = os.getenv("FLIGHT_DATA", "${HOME}/flight_data")

# Nur diese Sätze weiterleiten (PXCV ignorieren)
ALLOWED_PREFIXES = {"$GPGGA", "$GPGSA", "$GPRMC", "$GPRMZ", "$PFLAU", "$PFLAA", "$PFLAL", "$SENS"}

# --- Erstes .nmea File finden ---
def find_first_nmea_file(flight):
    directory = os.path.expandvars(os.path.join(FLIGHT_DATA_DIR, flight))
    for file in sorted(os.listdir(directory)):
        if file.lower().endswith(".nmea") and not "weg" in file:
            return os.path.join(directory, file)
    return None

# --- NMEA-Zeilen laden und mit Zeitstempeln versehen ---
def load_flight_data(filepath):
    lines = []
    last_gps_timestamp = None  # Letzter valider GPS-Timestamp (Ground Truth)
    last_sens_timestamp = None  # Letzter SENS-Timestamp
    time_ref = 0.0  # Relative-Zeit seit Start (für Deltas)
    has_valid_gps = False  # Flag: Erst nach validem GPS starten
    seen_sens_line = False # Flag. Eine erste SENS Zeile gefunden

    with open(filepath, "r", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.rstrip("\r\n")  # Nur Zeilenende entfernen, keine weiteren strip()
            if not line or not line.startswith('$'):
                continue

            index = line.rfind('$')
            if index > 0:
                line = line[index:]
                print(line)

            line = line.replace(';', ',', 1)
            fields = line.split(',')
            prefix = fields[0]
            if prefix not in ALLOWED_PREFIXES:
                continue  # z. B. $PXCV ignorieren

            timestamp = None
            lat = lon = None

            try:
                # --- GPS-Sätze: Ground Truth ---
                if line.startswith(("$GPGGA", "$GPRMC")):
                    time_str = fields[1]
                    if time_str and len(time_str) >= 6:
                        hhmmss = time_str.split('.')[0]
                        hh = int(hhmmss[0:2])
                        mm = int(hhmmss[2:4])
                        ss = int(hhmmss[4:6])
                        ms = int(time_str.split('.')[1][:3]) / 1000 if '.' in time_str else 0
                        timestamp = (hh * 3600 + mm * 60 + ss) + ms  # Relative Sekunden seit Mitternacht
                        if last_gps_timestamp is not None:
                            if timestamp < last_gps_timestamp:  # Mitternacht-Sprung
                                timestamp += 86400  # +24h
                        last_gps_timestamp = timestamp

                    # Position parsen (wie vorher)
                    if line.startswith("$GPGGA"):
                        lat_field, lat_dir = fields[2], fields[3]
                        lon_field, lon_dir = fields[4], fields[5]
                    elif line.startswith("$GPRMC"):
                        lat_field, lat_dir = fields[3], fields[4]
                        lon_field, lon_dir = fields[5], fields[6]
                    if lat_field and lon_field:
                        lat = float(lat_field[:2]) + float(lat_field[2:]) / 60
                        if lat_dir == "S": lat = -lat
                        lon_deg_part = lon_field[:3]
                        lon_min_part = lon_field[3:]
                        lon = float(lon_deg_part) + float(lon_min_part) / 60
                        if lon_dir == "W": lon = -lon
                    if not has_valid_gps:
                        if timestamp is not None and lat is not None and lon is not None and seen_sens_line:
                            time_ref = timestamp
                            print("First timestamp", timestamp)
                            has_valid_gps = True  # Ab jetzt einlesen

                elif line.startswith("$SENS"):
                    seen_sens_line = True
                    timestamp = last_gps_timestamp + 0.1

                # --- Andere Sätze: sofort ---
                else:
                    timestamp = last_gps_timestamp  # $PFLAU, $PFLAA usw. ohne Wartezeit
                last_gps_timestamp = timestamp

            except Exception as e:
                #print(f"Parse-Fehler bei: {line} → {e}")
                last_gps_timestamp = timestamp

            # --- Nur nach validem GPS einlesen ---
            if not has_valid_gps:
                continue

            #print(timestamp - time_ref, fields[0])
            lines.append({
                'line': line,
                'timestamp': timestamp - time_ref,
                'lat': lat,
                'lon': lon
            })

    return lines


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
            playback_time += delta * speed
            last_time = now
            while playback_time >= timestamp:
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

    print("minmaxlat", min(lats), max(lats))
    print("minmaxlon", min(lons), max(lons))
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
        global current_index, playback_time

        target_time = playback_time + seconds
        if target_time < flight_data[0]['timestamp']:
            target_time = flight_data[0]['timestamp']
        elif target_time > flight_data[-1]['timestamp']:
            target_time = flight_data[-1]['timestamp']

        # Finde nächsten Index >= target_time
        for i in range(0, len(flight_data)):
            if flight_data[i]['timestamp'] >= target_time and flight_data[i]['line'].startswith("$GPRMC"):
                current_index = i
                break
        else:
            current_index = len(flight_data) - 1
        playback_time = flight_data[current_index]['timestamp']
        print(f"Sprung zu Index {current_index}, Zeit {playback_time:.3f}s")

    def on_key(event):
        global is_playing, playback_speed, current_index
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
                current_index = 0
            elif event.key == 'end':
                current_index = len(flight_data) - 1
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
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            # Oder noch direkter:
            #update(None)  # Ruft deine update-Funktion einmal manuell auf

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

# --- Start ---
if __name__ == "__main__":
    # Initialize the parser
    parser = argparse.ArgumentParser(description="Read data from a serial device.")
    # Pflicht-Argument (Positional Argument)
    parser.add_argument("directory", type=str, help="Directory name, which contains the NMEA log")
    parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB1", help="Serial port path (e.g., /dev/ttyUSB1)")

    args = parser.parse_args()
    flight_dir = args.directory
    SERIAL_PORT = args.port

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

    flight_data = load_flight_data(nmea_file)
    if not flight_data:
        print("Keine gültigen NMEA-Daten gefunden.")
        ser.close()
        sys.exit(1)

    print(f"{len(flight_data)} Datensätze geladen.")
    print(f"{flight_data[-1]['timestamp']/3600:.2f} Stunden Logdaten.")

    # --- Playback Steuerung ---
    current_index = 0
    is_playing = False
    playback_speed = 1.0
    playback_time = 0.0

    lock = threading.Lock()

    # XCVario in SIM-Modus versetzen
    ser.write("$PFLAX,S*3C\n".encode("ascii"))

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