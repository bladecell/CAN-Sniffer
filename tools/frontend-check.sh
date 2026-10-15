#!/usr/bin/env bash
# svelte-check on sw/frontend with zero errors allowed (F3). Warnings (a11y etc.) do not fail.
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)/sw/frontend"
raw="$(bun x svelte-check --tsconfig ./tsconfig.json --output machine 2>&1 || true)"
if ! printf '%s\n' "$raw" | grep -q ' COMPLETED '; then echo "svelte-check did not complete:"; printf '%s\n' "$raw" | tail -20; exit 1; fi
errors="$(printf '%s\n' "$raw" | grep ' ERROR ' || true)"
if [ -n "$errors" ]; then printf '%s\n' "$errors" | sed -E 's/^[0-9]+ //'; echo "svelte-check: $(printf '%s\n' "$errors" | wc -l) errors"; exit 1; fi
echo "svelte-check: 0 errors"
