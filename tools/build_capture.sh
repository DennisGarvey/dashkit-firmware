#!/usr/bin/env bash
# Build USB capture or transmission firmware without writing to a device.
set -euo pipefail
CAPTURE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$CAPTURE_ROOT"
export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-$CAPTURE_ROOT/.idf-tools}"
. ./esp-idf/export.sh
profile=capture
defaults='sdkconfig.defaults;sdkconfig.capture'
case "${1:-}" in
    "") ;;
    --usb-tx)
        profile=usb-tx
        defaults="$defaults;sdkconfig.usb-tx"
        ;;
    --usb-tx-classic)
        profile=usb-tx-classic
        defaults="$defaults;sdkconfig.usb-tx;sdkconfig.usb-tx-classic"
        ;;
    *) echo "Usage: $0 [--usb-tx | --usb-tx-classic]" >&2; exit 2 ;;
esac
idf.py -B "build-$profile" -DIDF_TARGET=esp32s3 \
    -D "SDKCONFIG=sdkconfig.$profile.generated" \
    -D "SDKCONFIG_DEFAULTS=$defaults" build
