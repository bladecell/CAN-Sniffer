// Text for the CAN Bus page. Pure, so the units and the device labels are tested once.

/** Seconds since the first frame, as `12.345`. */
export function relTime(t: number, startUs: number | null): string {
  const us = startUs === null ? 0 : Math.max(0, t - startUs);
  return (us / 1e6).toFixed(3);
}

export function hzText(hz: number): string {
  return hz >= 10 ? `${Math.round(hz)} Hz` : `${hz.toFixed(1)} Hz`;
}

/** Permille of the bit rate as a percentage, one decimal. Null (unknown) shows a dash. */
export function loadText(permille: number | null): string {
  return permille === null ? "–" : `${(permille / 10).toFixed(1)}%`;
}

const TWAI_LABEL: Record<string, string> = {
  error_active: "Active",
  error_warning: "Warning",
  error_passive: "Passive",
  bus_off: "Bus off",
};

export function twaiLabel(state: string | undefined): string {
  if (!state) return "Unknown";
  return TWAI_LABEL[state] ?? state;
}

/** Seen in the last `windowMs`: "now", else seconds ago. */
export function ageText(seenMs: number, nowMs: number): string {
  const s = Math.max(0, Math.round((nowMs - seenMs) / 1000));
  return s === 0 ? "now" : `${s} s ago`;
}

/** Points for a sparkline in a 100 x 30 box, newest on the right. Empty or flat input gives a flat line. */
export function sparkPoints(values: readonly number[], width = 100, height = 30): string {
  if (values.length === 0) return `0,${height / 2} ${width},${height / 2}`;
  const max = Math.max(...values, 1);
  const step = values.length > 1 ? width / (values.length - 1) : 0;
  return values.map((v, i) => `${(i * step).toFixed(1)},${(height - (v / max) * (height - 2) - 1).toFixed(1)}`).join(" ");
}
