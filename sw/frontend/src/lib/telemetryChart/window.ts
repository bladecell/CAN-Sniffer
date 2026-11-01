// The shared time window and the present clock. Pure state transitions: every function returns a new Win.
// Window times are seconds before the present ("ago"), so a live window needs no clock to be valid.
// While paused, the present is frozen at pausedAt (device ms).

import { clamp, finiteOr } from "./math";

/** Retained overview span. Samples are kept per PID in chartBuffer (10 000 each). */
export const SPAN_SEC = 300;
export const WINDOW_LENS = [30, 60, 300] as const;
export const MIN_LEN_SEC = 10;
const LIVE_EPS = 0.5;
/** A backwards step of more than this many ms is a device reboot, as in chartBuffer. */
const REBOOT_JUMP_MS = 1000;

export interface Win {
  len: number;
  endAgo: number;
  /** Device ms at which the present was frozen, or null while live. */
  pausedAt: number | null;
}

export type WinStatus = "live" | "paused" | "history";

export function liveWin(len = 60): Win {
  return { len: clamp(finiteOr(len, 60), MIN_LEN_SEC, SPAN_SEC), endAgo: 0, pausedAt: null };
}

export function winStatus(w: Win): WinStatus {
  if (w.endAgo > LIVE_EPS) return "history";
  return w.pausedAt === null ? "live" : "paused";
}

/**
 * Moves the window: dLen changes the length (the start stays put only when dEnd compensates), dEnd moves the end
 * (positive = further back). Reaching the present from history resumes live. Scrubbing back pauses at now.
 */
export function shiftWin(w: Win, dLen: number, dEnd: number, now: number): Win {
  const len = clamp(finiteOr(w.len + finiteOr(dLen, 0), 60), MIN_LEN_SEC, SPAN_SEC);
  const end = clamp(finiteOr(w.endAgo + finiteOr(dEnd, 0), 0), 0, SPAN_SEC - len);
  if (end <= LIVE_EPS) {
    const resumed = w.endAgo > LIVE_EPS;
    return { len, endAgo: 0, pausedAt: resumed ? null : w.pausedAt };
  }
  return { len, endAgo: end, pausedAt: w.pausedAt ?? finiteOr(now, 0) };
}

/** Zooms by factor (below 1 zooms in). anchor is 0 at the left edge, 1 at the right edge: that time stays put. */
export function zoomWin(w: Win, factor: number, anchor: number, now: number): Win {
  const f = finiteOr(factor, 1);
  const a = clamp(finiteOr(anchor, 1), 0, 1);
  const newLen = clamp(w.len * f, MIN_LEN_SEC, SPAN_SEC);
  return shiftWin(w, newLen - w.len, (1 - a) * (w.len - newLen), now);
}

/** Pan by a fraction of the window length. Positive moves the window back in time (content follows the pointer). */
export function panWin(w: Win, dxFrac: number, now: number): Win {
  return shiftWin(w, 0, finiteOr(dxFrac, 0) * w.len, now);
}

export function setLenWin(w: Win, len: number, now: number): Win {
  return shiftWin(w, finiteOr(len, w.len) - w.len, 0, now);
}

export function freezeWin(w: Win, now: number): Win {
  return w.pausedAt === null ? { ...w, pausedAt: finiteOr(now, 0) } : w;
}

export function resumeWin(w: Win): Win {
  return liveWin(w.len);
}

export interface PresentClock {
  /** Feeds a device timestamp (ms) seen at browser time perf (ms). */
  observe(deviceMs: number, perf: number): void;
  /** Estimated device time now, or null before the first sample. */
  now(perf: number): number | null;
}

/** Extrapolates the device clock from the newest sample with the browser's monotonic clock. */
export function createPresentClock(): PresentClock {
  let anchor: { dev: number; perf: number } | null = null;
  return {
    observe(deviceMs, perf) {
      if (!Number.isFinite(deviceMs) || deviceMs < 0) return;
      if (!anchor || deviceMs >= anchor.dev || deviceMs < anchor.dev - REBOOT_JUMP_MS) {
        anchor = { dev: deviceMs, perf };
      }
    },
    now(perf) {
      return anchor ? anchor.dev + (perf - anchor.perf) : null;
    },
  };
}

/** Without data there is no present to pause at, so a window that would freeze is returned live. */
export function guardWin(w: Win, now: number | null): Win {
  return now === null && w.pausedAt !== null ? { ...w, endAgo: 0, pausedAt: null } : w;
}

/** Keyboard on a plot: arrows pan a tenth of the window, +/- zoom about its centre, Home or 0 reset. Null for other keys. */
export function keyWin(w: Win, key: string, now: number): Win | null {
  if (key === "ArrowLeft") return panWin(w, 0.1, now);
  if (key === "ArrowRight") return panWin(w, -0.1, now);
  if (key === "+" || key === "=") return zoomWin(w, 1 / 1.5, 0.5, now);
  if (key === "-") return zoomWin(w, 1.5, 0.5, now);
  if (key === "Home" || key === "0") return liveWin(60);
  return null;
}
