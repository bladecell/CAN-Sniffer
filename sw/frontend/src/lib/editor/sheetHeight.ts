/** Sheet height maths for the resizable editor sheet. Pure, so it is unit-tested (sheetHeight.test.ts). */

const MIN_RATIO = 0.3;
const MAX_RATIO = 0.85;
const DEFAULT_RATIO = 0.65;
const KEY_RATIO = 0.05;

export function clampSheetHeight(px: number, viewport: number): number {
  return Math.round(Math.min(Math.max(px, viewport * MIN_RATIO), viewport * MAX_RATIO));
}

export function defaultSheetHeight(viewport: number): number {
  return Math.round(viewport * DEFAULT_RATIO);
}

export function stepSheetHeight(px: number, dir: "up" | "down", viewport: number): number {
  return clampSheetHeight(px + (dir === "up" ? 1 : -1) * viewport * KEY_RATIO, viewport);
}

/** Clamps to explicit pixel bounds (the PID sheet: its head plus one row, up to the viewport less the nav). */
export function clampSheetPx(px: number, min: number, max: number): number {
  const lo = Math.min(min, max);
  const hi = Math.max(min, max);
  return Math.round(Math.min(Math.max(px, lo), hi));
}
