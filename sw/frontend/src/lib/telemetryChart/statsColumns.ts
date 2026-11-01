// Width of each value column in the stats table, from the table's own width. The name column keeps a minimum for its
// names, and the four value columns (Now, Min, Avg, Max) share the rest, bounded to fit the widest value plus its unit.

export const STATS_MIN_NAME_PX = 88;
export const STATS_VALUE_COLS = 4;
/** Widest value with its unit on one line at the phone size (telemetry.css), plus its gap. */
export const STATS_MIN_COL_PX = 60;
export const STATS_MAX_COL_PX = 120;

export function statsColumnPx(tableWidthPx: number): number {
  if (!Number.isFinite(tableWidthPx) || tableWidthPx <= 0) return STATS_MIN_COL_PX;
  const room = (tableWidthPx - STATS_MIN_NAME_PX) / STATS_VALUE_COLS;
  return Math.floor(Math.min(STATS_MAX_COL_PX, Math.max(STATS_MIN_COL_PX, room)));
}
