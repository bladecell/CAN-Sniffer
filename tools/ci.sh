#!/usr/bin/env bash
# Quality gate: must be green before every commit. Run from anywhere in the repo.
#   tools/ci.sh          full gate
#   tools/ci.sh --host   host tests only (fast loop)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FW="$ROOT/sw/firmware"                       # ESP-IDF project
FRONTEND="$ROOT/sw/frontend"                 # Svelte app
HOST_TESTS="$FW/test/host"                   # IDF project for the linux target
SRC_DIRS=(main components)                   # ours, relative to $FW (not managed_components); vendored tinyexpr is excluded below

HOST_ONLY=0
[ "${1:-}" = "--host" ] && HOST_ONLY=1

step() { printf '\n==> %s\n' "$*"; }

# --- ESP-IDF environment ---
IDF_EXPORT="${IDF_PATH:-$HOME/.espressif/v6.1/esp-idf}/export.sh"
if ! command -v idf.py >/dev/null 2>&1; then
  [ -f "$IDF_EXPORT" ] || { echo "ESP-IDF not found at $IDF_EXPORT (set IDF_PATH)"; exit 1; }
  # shellcheck disable=SC1090
  . "$IDF_EXPORT" >/dev/null
fi

# --- Host tests (IDF linux target, own build dir) ---
run_host_tests() {
  if [ ! -f "$HOST_TESTS/CMakeLists.txt" ]; then
    echo "No host tests yet (sw/firmware/test/host/), skipping (see P0.3)"; return 0
  fi
  step "Host tests (linux target)"
  [ -f "$HOST_TESTS/build/CMakeCache.txt" ] || idf.py -C "$HOST_TESTS" --preview set-target linux >/dev/null
  idf.py -C "$HOST_TESTS" build
  elf="$(find "$HOST_TESTS/build" -maxdepth 1 -name '*.elf' | head -n1)"
  [ -n "$elf" ] || { echo "host test ELF not found"; exit 1; }
  "$elf"
}

if [ "$HOST_ONLY" = 1 ]; then
  run_host_tests
  step "Host tests green"; exit 0
fi

# --- Frontend first: the firmware may embed its build output ---
if [ -f "$FRONTEND/package.json" ]; then
  step "Frontend (Svelte)"
  ( cd "$FRONTEND"
    if   [ -f bun.lock ] || [ -f bun.lockb ]; then PM=bun;  bun install --frozen-lockfile
    elif [ -f pnpm-lock.yaml ];               then PM=pnpm; pnpm install --frozen-lockfile
    elif [ -f package-lock.json ];            then PM=npm;  npm ci
    else echo "No lockfile in sw/frontend"; exit 1; fi
    has() { grep -q "\"$1\"[[:space:]]*:" package.json; }
    has check && "$ROOT/tools/frontend-check.sh"   # svelte-check, zero errors allowed
    has lint  && "$PM" run lint
    has test  && "$PM" run test
    "$PM" run build )
else
  echo "No frontend at sw/frontend, skipping"
fi

# --- Firmware build (components use -Werror) ---
step "Firmware build (esp32s3)"
idf.py -C "$FW" build

step "Size"
idf.py -C "$FW" size
"$ROOT/tools/size-gate.sh" "$FW/build"

# --- cppcheck on our sources only, from the compile database ---
# Run from $FW: cppcheck matches --file-filter against project-relative paths.
# -D__cplusplus=201703L makes esp_log.h use ##__VA_ARGS__ (cppcheck 2.17 cannot preprocess its __VA_OPT__ branch).
# Suppressed: returnByReference (a copy is thread-safe), duplInheritedMember (OBD2 hides OBD2DataModel members; review).
command -v cppcheck >/dev/null || { echo "cppcheck not installed (pacman -S cppcheck, or build it into ~/.local)"; exit 1; }
step "cppcheck"
filters=()
for d in "${SRC_DIRS[@]}"; do filters+=("--file-filter=$d/*"); done
( cd "$FW" && cppcheck --project=build/compile_commands.json "${filters[@]}" -D__cplusplus=201703L \
    --enable=warning,performance,portability --inline-suppr \
    --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=normalCheckLevelMaxBranches \
    --suppress=returnByReference --suppress=duplInheritedMember --suppress='*:components/tinyexpr/*' --suppress="*:$IDF_PATH/*" --suppress='*:*/managed_components/*' \
    --error-exitcode=1 --quiet -j"$(nproc)" )

# --- clang-tidy (esp-clang via tools/clang-tidy.sh; config in sw/firmware/.clang-tidy; any finding fails) ---
step "clang-tidy"
"$ROOT/tools/clang-tidy.sh"

# --- Host tests ---
run_host_tests

step "CI green"
