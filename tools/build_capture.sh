#!/usr/bin/env bash
# Build USB transmission firmware without writing to a device.
set -euo pipefail
CAPTURE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$CAPTURE_ROOT"
export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-$CAPTURE_ROOT/.idf-tools}"
. "${IDF_PATH:-$CAPTURE_ROOT/esp-idf}/export.sh"
if (( $# > 1 )) || [[ -n "${1:-}" && "$1" != --usb-tx ]]; then
    echo "Usage: $0 [--usb-tx]" >&2
    exit 2
fi
idf.py -B build-usb-tx -DIDF_TARGET=esp32s3 \
    -D SDKCONFIG=build-usb-tx/sdkconfig.generated \
    -D SDKCONFIG_DEFAULTS=sdkconfig.defaults build
