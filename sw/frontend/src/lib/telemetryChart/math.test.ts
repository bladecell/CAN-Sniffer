import { describe, expect, test } from "bun:test";
import { agoLabel, formatValue, niceTicks, timeStep } from "./math";

describe("niceTicks", () => {
  test("steps of 1, 2 or 5 times a power of ten, inside the range", () => {
    expect(niceTicks(0, 7000, 4)).toEqual([0, 2000, 4000, 6000]);
    expect(niceTicks(-1.5, 2.5, 4)).toEqual([-1, 0, 1, 2]);
  });

  test("hostile or empty ranges give finite or no ticks", () => {
    expect(niceTicks(Number.NaN, 3)).toEqual([]);
    expect(niceTicks(0, Number.POSITIVE_INFINITY)).toEqual([]);
    expect(niceTicks(0, 1e9, 1e9)).toEqual([]);
    const same = niceTicks(5, 5);
    expect(same.every(Number.isFinite)).toBe(true);
    expect(same.length).toBeLessThanOrEqual(10);
  });
});

describe("timeStep", () => {
  test("picks the smallest step that keeps the labels apart", () => {
    expect(timeStep(60, 600)).toBe(10);
    expect(timeStep(300, 300)).toBe(120);
  });
});

describe("agoLabel", () => {
  test("counts back from the present", () => {
    expect(agoLabel(0)).toBe("now");
    expect(agoLabel(30)).toBe("−30s");
    expect(agoLabel(120)).toBe("−2m");
    expect(agoLabel(130)).toBe("−2m 10s");
  });
});

describe("formatValue", () => {
  test("keeps one decimal below 100 and none above", () => {
    expect(formatValue(12.345)).toBe("12.3");
    expect(formatValue(1234.5)).toBe("1,235");
  });

  test("non-finite values show N/A", () => {
    expect(formatValue(Number.NaN)).toBe("N/A");
    expect(formatValue(Number.POSITIVE_INFINITY)).toBe("N/A");
  });
});
