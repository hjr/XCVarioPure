import re
import numpy as np

# ------------------------------------
# v(L/D max) numerisch aus Polare
# ------------------------------------
def v_ld_max(v1, w1, v2, w2, v3, w3):
    v = np.array([v1, v2, v3])
    w = np.array([abs(w1), abs(w2), abs(w3)])

    a, b, c = np.polyfit(v, w, 2)

    v_test = np.linspace(min(v), max(v), 2000)
    w_test = a * v_test**2 + b * v_test + c

    valid = w_test > 0
    v_test = v_test[valid]
    w_test = w_test[valid]

    ld = v_test / w_test
    return v_test[np.argmax(ld)]


# ------------------------------------
# Regex für Polarenzeile
# ------------------------------------
polar_re = re.compile(
    r'''
    ^\s*\{\s*
    (\d+)\s*,\s*
    "([^"]+)"\s*,\s*
    ([\d.]+)\s*,\s*
    ([\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*
    ([\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*
    ([\d.]+)\s*,\s*(-?[\d.]+)\s*
    (.*)
    \}\s*,?\s*$
    ''',
    re.VERBOSE
)

# ------------------------------------
# Datei lesen → erweiterte Tabelle ausgeben
# ------------------------------------
with open("PolarTable.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.rstrip()
        m = polar_re.match(line)

        if not m:
            print(line)
            continue

        groups = m.groups()

        idx  = groups[0]
        name = groups[1]
        ldref = groups[2]

        v1, w1 = float(groups[3]), float(groups[4])
        v2, w2 = float(groups[5]), float(groups[6])
        v3, w3 = float(groups[7]), float(groups[8])

        rest = groups[9].rstrip().rstrip(',')

        try:
            v_ld = round(v_ld_max(v1, w1, v2, w2, v3, w3), 1)
        except Exception:
            print(line)
            continue

        print(
            f'{{ {idx}, "{name}", {ldref}, '
            f'{v1:g},{w1:g},{v2:g},{w2:g},{v3:g},{w3:g}'
            f'{rest}, {v_ld} }},'
        )

