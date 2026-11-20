// Typed durations for slider readouts: "250", "250 ms", "1.5 s", "2 min", "1 h" to milliseconds.

const UNIT_MS: Record<string, number> = { ms: 1, s: 1000, sec: 1000, min: 60_000, m: 60_000, h: 3_600_000 };

/** Milliseconds for a typed duration, or null when it is not one. A bare number is milliseconds. */
export function parseDurationMs(text: string): number | null {
  const m = /^(\d+(?:[.,]\d+)?)\s*(ms|sec|s|min|m|h)?$/i.exec(text.trim());
  if (!m) return null;
  const ms = Number(m[1].replace(",", ".")) * UNIT_MS[(m[2] ?? "ms").toLowerCase()];
  return Number.isFinite(ms) ? Math.round(ms) : null;
}
