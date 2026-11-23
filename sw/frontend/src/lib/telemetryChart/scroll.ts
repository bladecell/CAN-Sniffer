// Scroll for one chart. Geometry is built for an anchor (a device time a little ahead of the present), and every frame
// shifts it by (anchor - present) instead of rebuilding it. A rebuild happens when the present passes the anchor, or
// when a line's hold would end before it: the held segment must not outlive its hold.

/** How far ahead of the present a build reaches. Also the left margin the geometry is built with. */
export const SCROLL_HEADROOM_MS = 250;

/** The device time a build is made for: the present plus headroom, but no later than the first hold that ends. */
export function buildAnchor(now: number, holdEnds: readonly number[]): number {
  let anchor = now + SCROLL_HEADROOM_MS;
  for (const end of holdEnds) if (end > now && end < anchor) anchor = end;
  return anchor;
}

/** True when geometry built for anchor no longer matches the frame at now. */
export function needsRebuild(anchor: number, now: number, holdEnds: readonly number[]): boolean {
  return now >= anchor || holdEnds.some((end) => end > now && end < anchor);
}

/** One frame of scroll: the anchor after the rebuild rule, and the present the frame draws. */
export function scrollFrame(anchor: number, now: number, holdEnds: readonly number[]): { anchor: number; present: number } {
  return { anchor: needsRebuild(anchor, now, holdEnds) ? buildAnchor(now, holdEnds) : anchor, present: now };
}

/** Device time at which each line's hold runs out, for lines with a value at their last sample. */
export function holdEndsOf(
  series: readonly { t: number[]; v: (number | null)[]; holdMs?: number }[],
  endAgoMs: number,
): number[] {
  const ends: number[] = [];
  for (const s of series) {
    const n = s.t.length;
    if (n === 0 || s.v[n - 1] === null) continue;
    ends.push(s.t[n - 1] + (s.holdMs ?? 2000) + endAgoMs);
  }
  return ends;
}
