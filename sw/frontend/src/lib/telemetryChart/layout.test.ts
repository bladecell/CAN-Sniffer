import { describe, expect, test } from "bun:test";
import { PLOT_COUNTS, gridColumns, migratePlotCount, rowsFor } from "./layout";

describe("plot layouts", () => {
  test("only 1, 2, 4 and 6 plots exist", () => {
    expect(PLOT_COUNTS).toEqual([1, 2, 4, 6]);
    expect(rowsFor(1)).toBe(1);
    expect(rowsFor(2)).toBe(2);
    expect(rowsFor(4)).toBe(2);
    expect(rowsFor(6)).toBe(3);
  });

  test("two columns from four plots, one otherwise", () => {
    expect([1, 2, 4, 6].map(gridColumns)).toEqual([1, 1, 2, 2]);
  });

  test("stored rows x cols map to the nearest allowed count", () => {
    expect(migratePlotCount(1, 1)).toBe(1);
    expect(migratePlotCount(2, 1)).toBe(2);
    expect(migratePlotCount(2, 2)).toBe(4);
    expect(migratePlotCount(3, 2)).toBe(6);
    expect(migratePlotCount(4, 4)).toBe(6);
    expect(migratePlotCount(1, 3)).toBe(2);
  });

  test("hostile stored values fall back to one plot", () => {
    expect(migratePlotCount(0, 0)).toBe(1);
    expect(migratePlotCount(Number.NaN, 2)).toBe(1);
    expect(migratePlotCount(-1, 5)).toBe(1);
    expect(migratePlotCount(Number.POSITIVE_INFINITY, 1)).toBe(1);
  });
});
