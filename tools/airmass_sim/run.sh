#!/usr/bin/env bash
# Build and run the AirMass EKF simulation tests inside the ESP-IDF Docker image (same as CI).
# Extra arguments are passed to ctest, e.g. ./run.sh -R thermal
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="${IDF_IMAGE:-espressif/idf:release-v5.4}"

docker run --rm --entrypoint "" -v "$REPO":/project -w /project "$IMAGE" bash -c "
    . \$IDF_PATH/export.sh > /dev/null 2>&1 &&
    cmake -S tools/airmass_sim -B build_sim -DCMAKE_BUILD_TYPE=Release > /dev/null &&
    cmake --build build_sim -j > /dev/null &&
    ctest --test-dir build_sim --output-on-failure $*
"
