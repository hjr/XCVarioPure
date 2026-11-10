#!/bin/sh
# Build the AirMass estimator as it was before any real-flight tuning (commit a833f536) into a browser/node module, for
# comparisons (summary.js). Output: build_sim/old/tools/airmass_sim/web/airmass_old.js
# a833f536 has no browser replay yet: its replay tools come from 323f5a9c, the last revision that builds against it.
set -e
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
OUT=$REPO/build_sim/old
TOOLS=tools/airmass_sim
rm -rf "$OUT" && mkdir -p "$OUT"
git -C "$REPO" archive a833f536 main components | tar -x -C "$OUT"
git -C "$REPO" archive 323f5a9c $TOOLS/stub $TOOLS/stats.h $TOOLS/replay_core.h $TOOLS/replay_settings.h \
    $TOOLS/web/airmass_wasm.cpp $TOOLS/web/build.sh | tar -x -C "$OUT"
# the old estimator has no config() to set settings by name
ROOT=$OUT OUTPUT=airmass_old.js EXTRA=-DAIRMASS_NO_CONFIG_SETTINGS sh "$OUT/$TOOLS/web/build.sh"
