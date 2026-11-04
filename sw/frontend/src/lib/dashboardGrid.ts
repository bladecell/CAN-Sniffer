// Pure grid arithmetic for the dashboard: pointer and keyboard resize, keyboard move. No DOM.
import { COLUMNS } from "./dashboardModel";

export const ROW_PX = 56;
export const GAP_PX = 12;

export type Clamp = (w: number, h: number) => { w: number; h: number };
/** A section as the card menus see it: its id, its label, and how many cards it holds. */
export interface SectionRef {
  id: string;
  label: string;
  count: number;
}
export type Size = { w: number; h: number };

/** Width of one column, gap included, for a section grid of the given width. Never below one pixel. */
export function columnPitch(gridWidth: number): number {
  const pitch = (gridWidth - (COLUMNS - 1) * GAP_PX) / COLUMNS + GAP_PX;
  return Number.isFinite(pitch) && pitch > 1 ? pitch : 1;
}

/** Pointer resize: whole columns and rows from the drag distance, then the type's range. */
export function resizeTo(clamp: Clamp, start: Size, dx: number, dy: number, pitch: number): Size {
  const dw = Math.round(dx / pitch);
  const dh = Math.round(dy / (ROW_PX + GAP_PX));
  return clamp(start.w + dw, start.h + dh);
}

const KEY_STEP: Record<string, [number, number]> = {
  ArrowRight: [1, 0],
  ArrowLeft: [-1, 0],
  ArrowDown: [0, 1],
  ArrowUp: [0, -1],
};

/** Keyboard resize by one column or row. Null when the key is not a resize key. */
export function keyResize(clamp: Clamp, start: Size, key: string): Size | null {
  const step = KEY_STEP[key];
  if (!step) return null;
  return clamp(start.w + step[0], start.h + step[1]);
}

/** Index after one step inside a list, or null at either end. */
export function stepIndex(index: number, count: number, delta: number): number | null {
  const next = index + delta;
  return next < 0 || next >= count ? null : next;
}
