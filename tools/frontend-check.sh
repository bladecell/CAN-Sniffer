#!/usr/bin/env bash
# svelte-check gated on a baseline: fails only on errors not in sw/frontend/svelte-check.baseline (P0.9, removed by P0.10).
# Lines are "<count><TAB><file><TAB><message>" without line numbers, so edits that only move code don't churn it.
#   tools/frontend-check.sh            compare against the baseline
#   tools/frontend-check.sh --update   rewrite the baseline
set -euo pipefail
FRONTEND="$(cd "$(dirname "$0")/.." && pwd)/sw/frontend"
BASE="$FRONTEND/svelte-check.baseline"
cd "$FRONTEND"
raw="$(bun x svelte-check --tsconfig ./jsconfig.json --output machine 2>&1 || true)"
cur="$(printf '%s\n' "$raw" | python3 -c '
import re, sys, collections
c = collections.Counter()
for l in sys.stdin:
    m = re.match(r"^\d+ ERROR \"([^\"]*)\" \d+:\d+ \"(.*)\"$", l.rstrip("\n"))
    if m: c[(m.group(1), m.group(2).replace("\\n", " "))] += 1
for (f, msg), n in sorted(c.items()): print(f"{n}\t{f}\t{msg}")
')"
if ! printf '%s\n' "$raw" | grep -q ' COMPLETED '; then echo "svelte-check did not complete:"; printf '%s\n' "$raw" | tail -20; exit 1; fi
if [ "${1:-}" = "--update" ]; then printf '%s\n' "$cur" > "$BASE"; echo "baseline: $(wc -l < "$BASE") entries"; exit 0; fi
[ -f "$BASE" ] || { echo "$BASE missing (run with --update)"; exit 1; }
CUR="$cur" python3 - "$BASE" <<'PY'
import os, sys
def load(lines):
    d = {}
    for l in lines:
        if l.strip():
            n, f, m = l.rstrip("\n").split("\t", 2); d[(f, m)] = int(n)
    return d
base = load(open(sys.argv[1])); cur = load(os.environ["CUR"].splitlines())
new = {k: n - base.get(k, 0) for k, n in cur.items() if n > base.get(k, 0)}
for (f, m), n in sorted(new.items()): print(f"NEW type error x{n}: {f}: {m}")
fixed = sum(1 for k in base if k not in cur or cur[k] < base[k])
print(f"svelte-check: {sum(cur.values())} errors, baseline {sum(base.values())}, new {sum(new.values())}, fixed-since-baseline entries {fixed}")
sys.exit(1 if new else 0)
PY
