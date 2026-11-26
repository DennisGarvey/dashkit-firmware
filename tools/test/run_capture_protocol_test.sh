#!/usr/bin/env bash
set -euo pipefail
CAPTURE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$CAPTURE_ROOT/.cache/capture-tests"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -I "$CAPTURE_ROOT/main/capture" \
    "$CAPTURE_ROOT/tools/test/test_capture_protocol.c" \
    -o "$CAPTURE_ROOT/.cache/capture-tests/test_capture_protocol"
"$CAPTURE_ROOT/.cache/capture-tests/test_capture_protocol"
