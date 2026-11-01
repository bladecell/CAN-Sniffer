// Plot layouts. Only these counts exist; a plot count is always one of PLOT_COUNTS.

export const PLOT_COUNTS = [1, 2, 4, 6] as const;
export type PlotCount = (typeof PLOT_COUNTS)[number];
export const DEFAULT_PLOT_COUNT: PlotCount = 2;

const ROWS: Record<PlotCount, number> = { 1: 1, 2: 2, 4: 2, 6: 3 };

/** Grid rows for a count: 1, 2, 2, 3. */
export function rowsFor(count: PlotCount): number {
  return ROWS[count];
}

/** Columns on wide screens: two from four plots. Phones always show one column (CSS). */
export function gridColumns(count: PlotCount): number {
  return count >= 4 ? 2 : 1;
}

/**
 * Maps a stored rows x cols pair to the nearest allowed plot count. Ties go to the smaller layout.
 * Anything that is not a finite positive count (NaN, 0, negative, Infinity) gives one plot.
 */
export function migratePlotCount(rows: number, cols: number): PlotCount {
  if (!Number.isFinite(rows) || !Number.isFinite(cols) || rows < 1 || cols < 1) return 1;
  const n = Math.round(rows) * Math.round(cols);
  let best: PlotCount = PLOT_COUNTS[0];
  for (const c of PLOT_COUNTS) {
    if (Math.abs(c - n) < Math.abs(best - n)) best = c;
  }
  return best;
}
