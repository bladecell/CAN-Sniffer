// Playback timeline of a recorded trip on the telemetry plots. The plots draw a window that ends at the present;
// playing a trip is that present moving through the trip's device-ms span. Pure, so it runs under bun test.
import { clamp } from "./math";
import { liveWin, type Win } from "./window";

/** Keeps t inside [start, end]; a non-number falls back to the start. */
export function clampMs(t: number, start: number, end: number): number {
  return Number.isFinite(t) ? clamp(t, start, end) : start;
}

/** The time at the right edge of the visible window: the present, minus how far the window is scrolled back. */
export function playheadOf(win: Win, present: number | null, start: number, end: number): number {
  return present === null ? start : clampMs(present - win.endAgo * 1000, start, end);
}

/** The window after a seek: paused at t, or live (the present clock runs on from t) when playing. */
export function seekWin(len: number, t: number, playing: boolean): Win {
  const w = liveWin(len);
  return playing ? w : { ...w, pausedAt: t };
}

export const atEnd = (present: number, end: number): boolean => present >= end;

/** A paused trip stays paused when a window move would resume it (reset, Home, scrolling to the edge): only the player plays. */
export function holdPause(prev: Win, next: Win): Win {
  return prev.pausedAt !== null && next.pausedAt === null ? { ...next, pausedAt: prev.pausedAt } : next;
}
