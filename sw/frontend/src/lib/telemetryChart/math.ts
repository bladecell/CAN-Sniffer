// Pure chart math: nice ticks, time labels and value text. No DOM, no NaN out.

const MINUS = "−";
const MAX_TICKS = 50;

export function finiteOr(v: unknown, fallback: number): number {
  return typeof v === "number" && Number.isFinite(v) ? v : fallback;
}

export function clamp(x: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, x));
}

/** Ticks at 1, 2 or 5 times a power of ten, inside [lo, hi]. Empty for a range it cannot draw. */
export function niceTicks(lo: number, hi: number, count = 4): number[] {
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [];
  const span = hi - lo || 1;
  const raw = span / Math.max(1, count);
  const mag = 10 ** Math.floor(Math.log10(raw));
  const r = raw / mag;
  const step = (r < 1.5 ? 1 : r < 3 ? 2 : r < 7 ? 5 : 10) * mag;
  if (!(step > 0) || span / step > MAX_TICKS) return [];
  const out: number[] = [];
  for (let k = Math.ceil(lo / step); k * step <= hi + 1e-9 * step && out.length <= MAX_TICKS; k++) {
    out.push(Number((k * step).toPrecision(12)));
  }
  return out;
}

const STEPS = [5, 10, 15, 30, 60, 120, 300];

/** Smallest label step (seconds) that leaves at least 75 px per label. */
export function timeStep(windowSec: number, plotPx: number): number {
  const maxLabels = Math.max(2, Math.floor(plotPx / 75));
  return STEPS.find((s) => windowSec / s <= maxLabels) ?? STEPS[STEPS.length - 1];
}

/** Label for a time this many seconds before the present. */
export function agoLabel(sec: number): string {
  const s = Math.max(0, Math.round(finiteOr(sec, 0)));
  if (s === 0) return "now";
  if (s < 60) return `${MINUS}${s}s`;
  const m = Math.floor(s / 60);
  const rest = s % 60;
  return `${MINUS}${m}m${rest ? ` ${rest}s` : ""}`;
}

/** Value text: one decimal below 100, whole numbers above. Non-finite values show N/A. */
export function formatValue(v: number): string {
  if (!Number.isFinite(v)) return "N/A";
  const digits = Math.abs(v) >= 100 ? 0 : 1;
  return v.toLocaleString("en", { minimumFractionDigits: 0, maximumFractionDigits: digits });
}
