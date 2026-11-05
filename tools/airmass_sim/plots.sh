#!/usr/bin/env bash
# Wind and vario plots of the simulated test flights (plot_wind.py, plot_vario.py) into build_sim/plots, inside the ESP-IDF Docker image like
# run.sh; build the simulator with run.sh first. matplotlib is installed once into build_sim/pylib.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${IDF_IMAGE:-espressif/idf:release-v5.4}"

docker run --rm --entrypoint "" -v "$REPO":/project -w /project "$IMAGE" bash -c '
    set -e
    . $IDF_PATH/export.sh > /dev/null 2>&1
    [ -d build_sim/pylib/matplotlib ] || python3 -m pip install -q --target build_sim/pylib matplotlib
    export PYTHONPATH=build_sim/pylib
    O=build_sim/plots
    mkdir -p $O
    for sc in straight thermal gust pullup slip nl; do build_sim/airmass_sim $sc --csv $O/$sc.csv > /dev/null || true; done
    P="python3 tools/airmass_sim/plot_wind.py"
    $P $O/straight.csv $O/straight.png --layout map --title "straight: S-turns in a constant wind"
    $P $O/thermal.csv $O/thermal.png --layout time --title "thermal: glide in, 4 minutes circling, glide out"
    $P $O/gust.csv $O/gust.png --layout time --title "gust: 4 m/s head wind gust at 250 s"
    $P $O/slip.csv $O/slip.png --layout time --title "slip: 8° sideslip from 200 s, straight and turning"
    $P $O/nl.csv $O/nl.png --layout time --every 30 --title "Dutch day: 1 hour cross country"
    $P $O/nl.csv $O/nl_glide.png --layout map --from 1940 --to 2260 --every 5 --title "Dutch day: glide with degraded wind"
    $P $O/nl.csv $O/nl_climb.png --layout time --from 1400 --to 1620 --every 4 --title "Dutch day: thermal entry and climb"
    V="python3 tools/airmass_sim/plot_vario.py"
    $V $O/thermal.csv $O/vario_thermal.png --title "thermal: 2.5 m/s lift while circling, smooth edges"
    $V $O/pullup.csv $O/vario_pullup.png --from 200 --to 380 --title "pullup: stick thermals, 45 to 25 m/s and 40 to 25 m/s in calm air"
    $V $O/gust.csv $O/vario_gust.png --from 220 --to 300 --title "gust: 4 m/s head wind gust at 250 s, no vertical air movement"
    $V $O/slip.csv $O/vario_slip.png --from 180 --to 480 --title "slip: 8° sideslip, straight and turning"
    $V $O/nl.csv $O/vario_nl_climb.png --from 1400 --to 1620 --title "Dutch day: thermal entry and climb"
    $V $O/nl.csv $O/vario_nl_cruise.png --from 1950 --to 2240 --title "Dutch day: cruise between thermals"
'
