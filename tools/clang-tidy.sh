#!/usr/bin/env bash
# clang-tidy over our firmware sources (sw/firmware/main, sw/firmware/components; not vendored tinyexpr).
# Needs a configured build (compile_commands.json) and esp-clang (idf_tools.py install esp-clang).
# Why not `idf.py clang-check`: pyclang leaves -mdisable-hardware-atomics in the command line, which
# clang rejects, so those files are never analysed. Same engine, own flag clean-up.
# IDF 6 keeps the toolchain flags (-specs=picolibc.specs, -mdisable-hardware-atomics, ...) in the response
# files build/toolchain/{c,cxx}flags. Clang cannot use them, so the @file is dropped and the Picolibc
# include dirs are taken from the toolchain gcc run with that same file.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FW="$ROOT/sw/firmware"
OUT="$FW/build/clang-tidy"
DB="$FW/build/compile_commands.json"

[ -f "$DB" ] || { echo "$DB missing, build first"; exit 1; }
command -v run-clang-tidy >/dev/null || { echo "run-clang-tidy not found (source export.sh, install esp-clang)"; exit 1; }
command -v xtensa-esp32s3-elf-g++ >/dev/null || { echo "xtensa toolchain not on PATH"; exit 1; }
mkdir -p "$OUT"

# GCC-only flags clang rejects, plus -Werror so clang's own warnings never gate us.
python3 - "$FW" "$DB" "$OUT/compile_commands.json" <<'PY'
import json, re, sys
fw, src, dst = sys.argv[1:4]
drop = re.compile(r' (-mdisable-hardware-atomics|-Wno-old-style-declaration|-fstrict-volatile-bitfields|'
                  r'-fno-tree-switch-conversion|-fno-test-coverage|-mlongcalls|-mlong-calls|-Werror(=\S+)?|-fmacro-prefix-map=\S+|'
                  r'-fdebug-prefix-map=\S+|-ffile-prefix-map=\S+|"?@\S+/toolchain/c(xx)?flags"?)(?= )')
out = []
for e in json.load(open(src)):
    f = e["file"]
    if not (f.startswith(fw + "/main/") or f.startswith(fw + "/components/")) or "/components/tinyexpr/" in f:
        continue
    e["command"] = drop.sub("", e["command"] + " ").strip()
    out.append(e)
json.dump(out, open(dst, "w"))
print(f"clang-tidy: {len(out)} translation units")
PY

# clang cannot find the toolchain's libc/libstdc++ headers on its own; ask the xtensa gcc, with the build's
# toolchain flags so it reports the Picolibc include dirs rather than the newlib ones.
args=(-quiet -p "$OUT" -j "$(nproc)" -extra-arg=-Wno-unknown-warning-option)
while IFS= read -r d; do args+=("-extra-arg=-isystem$(realpath "$d")"); done < <(
  echo | xtensa-esp32s3-elf-g++ "@$FW/build/toolchain/cxxflags" -E -x c++ - -v 2>&1 | sed -n '/^#include <...>/,/^End of search/p' | grep '^ ' | sed 's/^ //')

rc=0
run-clang-tidy "${args[@]}" > "$OUT/warnings.txt" 2>&1 || rc=$?
grep -E '^/.*(warning|error):' "$OUT/warnings.txt" | sort -u | head -n "${TIDY_SHOW:-40}" || true
echo "clang-tidy exit $rc (full log: $OUT/warnings.txt)"
exit $rc
