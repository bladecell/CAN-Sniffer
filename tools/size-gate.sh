#!/usr/bin/env bash
# App size gate: fail if the app image exceeds the OTA slot, warn at 90%.
# Slot size comes from the partition table of the build in use (ota_0 row).
#   tools/size-gate.sh [build_dir]       default sw/firmware/build
#   SIZE_GATE_LIMIT=<bytes>              override the slot size (to test the gate)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
B="${1:-$ROOT/sw/firmware/build}"
BIN="$(ls "$B"/*.bin | grep -v -e ota_data -e www -e partition | head -n1)"
slot="${SIZE_GATE_LIMIT:-}"
if [ -z "$slot" ]; then
  slot="$(python3 "$IDF_PATH/components/partition_table/gen_esp32part.py" "$B/partition_table/partition-table.bin" \
    | awk -F, '$1=="ota_0"{print $5}')"
  case "$slot" in
    *K) slot=$(( ${slot%K} * 1024 )) ;;
    *M) slot=$(( ${slot%M} * 1024 * 1024 )) ;;
  esac
fi
[ -n "$slot" ] || { echo "size-gate: ota_0 not found in the partition table"; exit 1; }
size="$(stat -c %s "$BIN")"
pct=$(( size * 100 / slot ))
echo "size-gate: $(basename "$BIN") $size B of $slot B OTA slot ($pct%)"
[ "$size" -le "$slot" ] || { echo "size-gate: FAIL, app exceeds the OTA slot"; exit 1; }
[ "$pct" -lt 90 ] || echo "size-gate: WARNING, app is at or above 90% of the OTA slot"
