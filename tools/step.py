#!/usr/bin/env python3
import serial
import time
import math

# Constants
R = 287.05       # J/(kg·K)
g = 9.80665      # m/s²
L = 0.0065       # K/m
T0 = 288.15      # K
p0 = 101325.0    # Pa


def isa_pressure(h_m):
    """
    Standard ISA static pressure at height h [m]
    valid for 0–11000 m
    returns pressure in Pascal
    """
    return p0 * (1.0 - L * h_m / T0) ** (g / (R * L))

def air_density(height_m, temp_C):
    T = temp_C + 273.15
    p = p0 * (1.0 - L * height_m / T0) ** (g / (R * L))
    rho = p / (R * T)
    return rho

def dynamic_pressure_from_TAS(TAS_kmh, height_m, temp_C):
    rho = air_density(height_m, temp_C)
    v = TAS_kmh / 3.6
    q = 0.5 * rho * v * v
    return q

# ---------------- CONFIG ----------------

PORT = "/dev/ttyUSB1"
BAUD = 115200

RATE_HZ = 10.0           # 1..10 Hz
DT = 1.0 / RATE_HZ

HEIGHT = 500.0
BASE_PRESSURE_MBAR = isa_pressure(HEIGHT)/100.
DPDH_MBAR_PER_M = -0.1145    # mbar per meter (≈ -11.45 Pa/m)

TEMP_C = 13.43
TAS = 100.0
dynamicP_Pa = dynamic_pressure_from_TAS(TAS, HEIGHT, TEMP_C)


# Dummy IMU / MAG
AX, AY, AZ = 0.1728, 0.0805, 0.9861
GX, GY, GZ = -0.2031, -0.0145, 0.0053
MX, MY, MZ = 19.6474, 19.7357, -34.4881

# ----------------------------------------

ser = serial.Serial(PORT, BAUD, timeout=0)

pressure = BASE_PRESSURE_MBAR
t0 = time.time()

def utc_seconds():
    now = time.gmtime()
    return now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec + time.time() % 1

print("Streaming synthetic SENS data...")
ser.write(("$PFLAX,S,1\r\n").encode("ascii")) # switch XCV to sim mode

while True:
    now = time.time()
    deltaT_ms = 0
    sim_time = now - t0

    # 20s on / 20s off climb
    climb = int(sim_time / 20) % 4
    if climb == 0 or climb == 2:
        vario = 0.0
    elif climb == 1:
        vario = 1.0 # m/s
    elif climb == 3:
        vario = -1.0

    # TEK pressure integration
    dpdt = DPDH_MBAR_PER_M * vario
    pressure += dpdt * DT

    gpsT = utc_seconds()

    msg = (
        f"$SENS;"
        f"{gpsT:0.3f},"
        f"{deltaT_ms},"
        f"{pressure:0.3f},"
        f"{pressure:0.3f},"   # teP (identisch für idealen Test)
        f"{dynamicP_Pa:0.3f},"
        f"{TEMP_C:0.2f},"
        f"{AX:0.4f},{AY:0.4f},{AZ:0.4f},"
        f"{GX:0.4f},{GY:0.4f},{GZ:0.4f},"
        f"{MX:0.4f},{MY:0.4f},{MZ:0.4f}\r\n"
    )

    ser.write(msg.encode("ascii"))
    print(msg)
    time.sleep(DT)
