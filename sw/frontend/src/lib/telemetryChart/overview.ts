// Overview strip geometry: the last SPAN_SEC seconds, newest on the right, with the shared window as a frame.

import { agoLabel, clamp, finiteOr } from "./math";
import { GUTTER_LEFT, GUTTER_RIGHT } from "./gutter";
import { domainFor, fracOf, type Range } from "./plotModel";
import { decimate, pathOf, type Pt } from "./series";
import { SPAN_SEC, shiftWin, type Win } from "./window";

export const OV_L = GUTTER_LEFT;
export const OV_R = GUTTER_RIGHT;
const OV_T = 6;
const OV_B = 22;
const EDGE_PX = 14;

export type Grab = "left" | "right" | "move" | "jump";

const plotW = (W: number) => Math.max(1, finiteOr(W, 0) - OV_L - OV_R);

/** X of a time `ago` seconds before the present. */
export function ovX(ago: number, W: number): number {
  return OV_L + (1 - clamp(finiteOr(ago, 0), 0, SPAN_SEC) / SPAN_SEC) * plotW(W);
}

/** Inverse of ovX, clamped to the strip. */
export function ovAgo(x: number, W: number): number {
  return clamp((1 - (finiteOr(x, 0) - OV_L) / plotW(W)) * SPAN_SEC, 0, SPAN_SEC);
}

/** Which part of the frame a pointer at x grabs. Outside the frame it jumps the window there. */
export function grabAt(x: number, w: Win, W: number): Grab {
  const x0 = ovX(w.endAgo + w.len, W);
  const x1 = ovX(w.endAgo, W);
  if (Math.abs(x - x0) <= EDGE_PX) return "left";
  if (Math.abs(x - x1) <= EDGE_PX) return "right";
  return x > x0 && x < x1 ? "move" : "jump";
}

/** New window for a drag of dxPx from the window at the start of the drag. */
export function dragWin(start: Win, grab: Exclude<Grab, "jump">, dxPx: number, W: number, now: number): Win {
  const d = (finiteOr(dxPx, 0) / plotW(W)) * SPAN_SEC;
  if (grab === "move") return shiftWin(start, 0, -d, now);
  if (grab === "left") return shiftWin(start, -d, 0, now);
  return shiftWin(start, d, -d, now);
}

/** Centres the window on the time `ago` (a click on the strip). */
export function jumpTo(w: Win, ago: number, now: number): Win {
  return shiftWin(w, 0, finiteOr(ago, 0) - w.len / 2 - w.endAgo, now);
}

export interface OverviewSeries {
  color: string;
  range: Range | null;
  t: number[];
  v: (number | null)[];
}

export interface OverviewModel {
  frame: { x0: number; x1: number };
  lines: { color: string; d: string }[];
  ticks: { x: number; text: string }[];
  top: number;
  bottom: number;
}

/** Strip drawing: each PID normalised to its own range (or the 5 minute data when it has none). */
export function overviewModel(o: { width: number; height: number; now: number; win: Win; series: OverviewSeries[] }): OverviewModel {
  const W = Math.max(0, finiteOr(o.width, 0));
  const H = Math.max(40, finiteOr(o.height, 84));
  const bottom = H - OV_B;
  const fromT = o.now - SPAN_SEC * 1000 - 1000;
  const pw = plotW(W);
  const lines = o.series.map((s) => {
    const vals: number[] = [];
    for (let k = 0; k < s.t.length; k++) {
      const x = s.v[k];
      if (typeof x === "number" && Number.isFinite(x) && s.t[k] >= fromT) vals.push(x);
    }
    const [lo, hi] = domainFor(s.range, vals, false, true);
    const pts: Pt[] = [];
    for (let k = 0; k < s.t.length; k++) {
      if (!Number.isFinite(s.t[k]) || s.t[k] < fromT || s.t[k] > o.now + 1000) continue;
      const x = s.v[k];
      if (typeof x !== "number" || !Number.isFinite(x)) pts.push(null);
      else {
        const frac = Math.min(1, Math.max(0, fracOf(x, lo, hi)));
        pts.push({ x: ovX((o.now - s.t[k]) / 1000, W), y: bottom - frac * (bottom - OV_T) });
      }
    }
    return { color: s.color, d: pathOf(decimate(pts, pw)) };
  });
  return {
    frame: { x0: ovX(o.win.endAgo + o.win.len, W), x1: ovX(o.win.endAgo, W) },
    lines,
    ticks: [0, 60, 120, 180, 240, 300].map((s) => ({ x: ovX(s, W), text: agoLabel(s) })),
    top: OV_T,
    bottom,
  };
}

/**
 * Client x to viewBox units. An svg drawn narrower or wider than its viewBox (width 100%) scales its coordinates, so
 * handles and clicks must be mapped through the drawn width. Without a measured width the offset is used as is.
 */
export function toViewX(clientX: number, rectLeft: number, drawnW: number, viewW: number): number {
  const off = finiteOr(clientX, 0) - finiteOr(rectLeft, 0);
  return drawnW > 0 ? (off * finiteOr(viewW, 0)) / drawnW : off;
}

export interface DrawnRect {
  left: number;
  width: number;
}

/**
 * The window after a pointer drag from clientX0 to clientX, both mapped through the drawn width so the frame follows
 * the finger one display px per px.
 */
export function overviewDrag(
  start: Win,
  grab: Exclude<Grab, "jump">,
  rect: DrawnRect,
  clientX0: number,
  clientX: number,
  viewW: number,
  now: number,
): Win {
  const x0 = toViewX(clientX0, rect.left, rect.width, viewW);
  const x = toViewX(clientX, rect.left, rect.width, viewW);
  return dragWin(start, grab, x - x0, viewW, now);
}
