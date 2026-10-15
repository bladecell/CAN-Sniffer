// Bar geometry shared by PID bars and the car battery bar. Pure, no DOM.

/** Position of a value on a bar, 0 at min and 1 at max, clamped. Null is no data. */
export function barFraction(value: number, min: number, max: number): number | null {
  if (!Number.isFinite(value) || !(max > min)) return null;
  return Math.min(1, Math.max(0, (value - min) / (max - min)));
}

export interface BarZone {
  /** Fractions of the bar, 0 to 1. */
  from: number;
  to: number;
  opacity: number;
  /** Zone colour. Falls back to the bar colour. */
  color?: string;
}

/** Zones for a PID, which has only min and max: three equal thirds, each a little stronger than the last. */
export function equalThirds(): BarZone[] {
  return [
    { from: 0, to: 1 / 3, opacity: 0.4 },
    { from: 1 / 3, to: 2 / 3, opacity: 0.7 },
    { from: 2 / 3, to: 1, opacity: 1 },
  ];
}

/** Pixel width of the fill inside a track of the given width. No data is 0. */
export function fillWidth(fraction: number | null, width: number): number {
  if (fraction === null) return 0;
  return Math.min(1, Math.max(0, fraction)) * width;
}
