#!/usr/bin/env bash
# Build passive USB capture firmware without writing to a device.
set -euo pipefail
CAPTURE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$CAPTURE_ROOT"
export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-$CAPTURE_ROOT/.idf-tools}"
. "${IDF_PATH:-$CAPTURE_ROOT/esp-idf}/export.sh"
idf.py -B build-capture -DIDF_TARGET=esp32s3 \
    -D SDKCONFIG=build-capture/sdkconfig.generated \
    -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults' build
