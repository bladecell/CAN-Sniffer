import { describe, expect, test } from "bun:test";
import { HINT_GAP_MS, canToggleTicks, chartPidsOf, hintDue, selectPlot, deselectOnEscape, deselectOnPointer } from "./selection";

describe("plot selection", () => {
  test("selecting a plot makes it the one selected; an invalid index changes nothing", () => {
    expect(selectPlot(0, 2, 4)).toBe(2);
    expect(selectPlot(0, 4, 4)).toBe(0);
    expect(selectPlot(1, -1, 4)).toBe(1);
    expect(selectPlot(0, Number.NaN, 4)).toBe(0);
  });

  test("switching plots moves the selection; exactly one plot is selected at a time", () => {
    const first = selectPlot(-1, 0, 6);
    expect(selectPlot(first, 3, 6)).toBe(3);
  });

  test("a pointer outside the plots and the related surfaces deselects; a related one keeps the selection", () => {
    expect(deselectOnPointer(1, false)).toBe(-1);
    expect(deselectOnPointer(1, true)).toBe(1);
    expect(deselectOnPointer(-1, false)).toBe(-1);
  });

  test("Esc deselects only when focus is in a plot", () => {
    expect(deselectOnEscape(2, true)).toBe(-1);
    expect(deselectOnEscape(2, false)).toBe(2);
  });

  test("ticks change only with a plot selected", () => {
    expect(canToggleTicks(-1)).toBe(false);
    expect(canToggleTicks(0)).toBe(true);
  });

  test("the list shows the selected plot's PIDs, and none when no plot is selected", () => {
    const perPlot = [[12, 13], [17]];
    expect(chartPidsOf(perPlot, 0)).toEqual([12, 13]);
    expect(chartPidsOf(perPlot, -1)).toEqual([]);
  });

  test("re-selecting a plot restores its own PIDs: nothing is lost while it was deselected", () => {
    const perPlot = [[12, 13], [17]];
    const none = -1;
    expect(chartPidsOf(perPlot, none)).toEqual([]);
    expect(chartPidsOf(perPlot, selectPlot(none, 0, 2))).toEqual([12, 13]);
  });

  test("the no-plot hint is rate limited", () => {
    expect(hintDue(10_000, undefined)).toBe(true);
    expect(hintDue(10_000, 10_000 - HINT_GAP_MS + 1)).toBe(false);
    expect(hintDue(10_000, 10_000 - HINT_GAP_MS)).toBe(true);
  });
});
