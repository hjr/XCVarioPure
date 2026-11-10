#!/bin/sh
# Build the browser replay module (airmass.js, the WebAssembly embedded) in the Emscripten Docker image.
#   tools/airmass_sim/web/build.sh        then open tools/airmass_sim/web/index.html
# ROOT (a source tree laid out like this repo), OUTPUT (the module's name) and EXTRA (compiler flags) build other
# versions of the estimator (analysis/build_old.sh).
set -e
ROOT=${ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}
OUTPUT=${OUTPUT:-airmass.js}
docker run --rm -v "$ROOT":/project -w /project/tools/airmass_sim/web emscripten/emsdk:3.1.74 \
    em++ -std=c++17 -O2 -Wall -Wextra $EXTRA \
    -include ../stub/host_compat.h -include cstdint -I.. -I../stub -I../../../main/airmass -I../../../components/util -I../../../components/glider/include -I../../../main \
    airmass_wasm.cpp ../../../main/airmass/AirMassEKF.cpp ../../../main/protocol/nmea_util.cpp \
    ../../../components/util/math/Quaternion.cpp ../../../components/util/math/vector_3d.cpp \
    ../../../components/util/math/Trigonometry.cpp ../../../components/util/math/Floats.cpp ../../../components/util/math/Units.cpp \
    -sMODULARIZE=1 -sEXPORT_NAME=AirMassModule -sSINGLE_FILE=1 -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web,node \
    -sEXPORTED_FUNCTIONS=_malloc,_free -sEXPORTED_RUNTIME_METHODS=UTF8ToString,stringToUTF8,lengthBytesUTF8,HEAPF32,HEAPU32 \
    -o "$OUTPUT"
echo "built $ROOT/tools/airmass_sim/web/$OUTPUT"
