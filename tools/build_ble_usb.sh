#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# Reuse the local toolchain when building beside the USB TX checkout.
if [[ -z "${IDF_TOOLS_PATH:-}" && -d "$PWD/../dashkit-firmware/.idf-tools" ]]; then
    export IDF_TOOLS_PATH="$PWD/../dashkit-firmware/.idf-tools"
fi
idf_export=./esp-idf/export.sh
if [[ ! -f "$idf_export" ]]; then
    idf_export="${IDF_PATH:-$PWD/../dashkit-firmware/esp-idf}/export.sh"
fi
export IDF_COMPONENT_MANAGER=0
. "$idf_export"
idf.py -B build-ble-usb -DIDF_TARGET=esp32s3 \
    -D SDKCONFIG=sdkconfig.ble-usb.generated \
    -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.ble-usb' build
