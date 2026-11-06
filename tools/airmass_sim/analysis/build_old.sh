#!/bin/sh
# Build the AirMass estimator as it was before any real-flight tuning (commit a833f536) into a browser/node module, for
# comparisons (summary.js). Output: build_sim/old/tools/airmass_sim/web/airmass_old.js
set -e
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
OUT=$REPO/build_sim/old
rm -rf "$OUT" && mkdir -p "$OUT/tools/airmass_sim/web"
git -C "$REPO" archive a833f536 main components | tar -x -C "$OUT"
T=$REPO/tools/airmass_sim
cp -r "$T/stub" "$T/stats.h" "$T/replay_core.h" "$T/replay_settings.h" "$OUT/tools/airmass_sim/"
cp "$T/web/airmass_wasm.cpp" "$OUT/tools/airmass_sim/web/"
# the old estimator has no config() to set settings by name
ROOT=$OUT OUTPUT=airmass_old.js EXTRA=-DAIRMASS_NO_CONFIG_SETTINGS "$T/web/build.sh"
