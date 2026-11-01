import { describe, expect, test } from "bun:test";
import { STATS_MAX_COL_PX, STATS_MIN_COL_PX, STATS_MIN_NAME_PX, STATS_VALUE_COLS, statsColumnPx } from "./statsColumns";

describe("stats table value columns", () => {
  test("four value columns (Now, Min, Avg, Max) and the name column always fit the table width", () => {
    expect(STATS_VALUE_COLS).toBe(4);
    for (const w of [260, 292, 308, 360, 500, 768, 1100, 1400]) {
      const col = statsColumnPx(w);
      expect(col * STATS_VALUE_COLS + STATS_MIN_NAME_PX).toBeLessThanOrEqual(Math.max(w, STATS_MIN_COL_PX * STATS_VALUE_COLS + STATS_MIN_NAME_PX));
    }
  });

  test("value columns are never narrower than the widest value with its unit", () => {
    expect(statsColumnPx(10)).toBe(STATS_MIN_COL_PX);
    expect(statsColumnPx(Number.NaN)).toBe(STATS_MIN_COL_PX);
    expect(statsColumnPx(292)).toBeGreaterThanOrEqual(STATS_MIN_COL_PX);
  });

  test("value columns grow with the card and stop at the maximum", () => {
    expect(statsColumnPx(308)).toBeGreaterThanOrEqual(statsColumnPx(292));
    expect(statsColumnPx(1400)).toBe(STATS_MAX_COL_PX);
  });
});
