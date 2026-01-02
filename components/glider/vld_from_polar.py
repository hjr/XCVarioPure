#!/usr/bin/env python3

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
    return float(v_test[np.argmax(ld)])


# ------------------------------------
# Regex: extract the three polar points
# (ONLY what we need)
# ------------------------------------
polar_re = re.compile(
    r"""
    ^\s*\{\s*
    \d+\s*,\s*           # index
    "[^"]+"\s*,\s*      # name
    [\d.]+\s*,\s*       # L/D ref

    ([\d.]+)\s*,\s*(-?[\d.]+)\s*,   # v1, w1
    ([\d.]+)\s*,\s*(-?[\d.]+)\s*,   # v2, w2
    ([\d.]+)\s*,\s*(-?[\d.]+)\s*,   # v3, w3
    """,
    re.VERBOSE
)


def has_vld(line: str) -> bool:
    # Require TWO numeric fields at the end: flaps + V(L/D)
    return bool(
        re.search(
            r",\s*\d+\s*,\s*\d+(\.\d+)?\s*\}\s*,\s*$",
            line
        )
    )

def append_vld(line: str, v_ld: float) -> str:
    line = line.rstrip()

    # must end with }, allowing whitespace
    if not re.search(r"\}\s*,\s*$", line):
        return line

    if has_vld(line):
        return line

    # remove closing "},"
    base = re.sub(r"\s*\}\s*,\s*$", "", line)

    # remove ALL trailing whitespace
    base = re.sub(r"\s+$", "", base)

    # normalize flap flag: exactly ",0" or ",1"
    base = re.sub(r",\s*([01])$", r",\1", base)

    # append V(L/D) with ZERO spaces
    return base + "," + format(v_ld, ".1f") + "},"


# ------------------------------------
# Main
# ------------------------------------
with open("PolarTable.txt", "r", encoding="utf-8") as f:
    for line in f:
        m = polar_re.match(line)

        if not m:
            print(line, end="")
            continue

        try:
            v1, w1, v2, w2, v3, w3 = map(float, m.groups())
            v_ld = v_ld_max(v1, w1, v2, w2, v3, w3)
            print(append_vld(line, v_ld))
        except Exception:
            # safety: never destroy data
            print(line, end="")

