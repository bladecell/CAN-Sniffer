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

/** A value pinned to [min, max] for a gauge or bar. NaN and infinities count as min. */
export function clampToRange(value: number, min: number, max: number): number {
  if (!(max > min)) return min;
  if (!Number.isFinite(value)) return min;
  return Math.min(max, Math.max(min, value));
}

const SEGMENT_MIN_PX = 6;
const SEGMENT_GAP_PX = 2;
/** Caps the number of blocks, so the DOM stays small on a wide card. */
export const SEGMENT_MAX_COUNT = 40;

export interface SegmentLayout {
  count: number;
  /** Block width in px. */
  width: number;
  gap: number;
}

/** Blocks that fit a bar of this width. Zero for a width that is not a positive number. */
export function segmentLayout(width: number): SegmentLayout {
  if (!Number.isFinite(width) || width <= 0) return { count: 0, width: 0, gap: SEGMENT_GAP_PX };
  const fit = Math.floor((width + SEGMENT_GAP_PX) / (SEGMENT_MIN_PX + SEGMENT_GAP_PX));
  const count = Math.min(SEGMENT_MAX_COUNT, Math.max(1, fit));
  return { count, width: (width - (count - 1) * SEGMENT_GAP_PX) / count, gap: SEGMENT_GAP_PX };
}

/** Lit blocks for a fraction. No data and zero light none; any positive value lights at least one. */
export function litSegments(fraction: number | null, count: number): number {
  if (fraction === null || !Number.isFinite(fraction) || count <= 0) return 0;
  const f = Math.min(1, Math.max(0, fraction));
  if (f === 0) return 0;
  return Math.max(1, Math.round(f * count));
}

export interface GradientStop {
  offset: number;
  color: string;
  opacity: number;
}

/**
 * Stops for a gradient along the bar. With zones, each zone starts at its own colour (battery bands);
 * without them, the bar colour runs end to end (PIDs: the zone opacities are kept).
 */
export function gradientStops(color: string, zones: BarZone[]): GradientStop[] {
  if (zones.length === 0) {
    return [
      { offset: 0, color, opacity: 1 },
      { offset: 1, color, opacity: 1 },
    ];
  }
  const stops = zones.map((z) => ({ offset: z.from, color: z.color ?? color, opacity: z.opacity }));
  const last = zones[zones.length - 1];
  stops.push({ offset: 1, color: last.color ?? color, opacity: last.opacity });
  return stops;
}

/** Left edge of a marker of this size, centred on the value and kept inside the track. Null is no data. */
export function markerOffset(fraction: number | null, width: number, size: number): number | null {
  if (fraction === null || !Number.isFinite(fraction)) return null;
  const x = fraction * width - size / 2;
  return Math.min(Math.max(0, width - size), Math.max(0, x));
}
