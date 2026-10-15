import { describe, expect, test } from "bun:test";
import { barFraction, equalThirds, fillWidth } from "./valueBar";

describe("barFraction", () => {
  test("maps min, middle and max to 0, 0.5 and 1", () => {
    expect(barFraction(0, 0, 100)).toBe(0);
    expect(barFraction(50, 0, 100)).toBe(0.5);
    expect(barFraction(100, 0, 100)).toBe(1);
  });

  test("clamps values outside the range", () => {
    expect(barFraction(-5, 0, 100)).toBe(0);
    expect(barFraction(250, 0, 100)).toBe(1);
  });

  test.each([
    ["NaN", Number.NaN],
    ["Infinity", Number.POSITIVE_INFINITY],
  ])("%s is no data", (_label, value) => {
    expect(barFraction(value, 0, 100)).toBeNull();
  });

  test("a degenerate range (max not above min) is no data", () => {
    expect(barFraction(5, 10, 10)).toBeNull();
    expect(barFraction(5, 10, 0)).toBeNull();
  });
});

describe("equalThirds", () => {
  test("three zones split 0 to 1 evenly, with rising opacity", () => {
    const zones = equalThirds();
    expect(zones.map((z) => [z.from, z.to])).toEqual([
      [0, 1 / 3],
      [1 / 3, 2 / 3],
      [2 / 3, 1],
    ]);
    const opacities = zones.map((z) => z.opacity);
    expect(opacities[0]).toBeLessThan(opacities[1]);
    expect(opacities[1]).toBeLessThan(opacities[2]);
  });
});

describe("fillWidth", () => {
  test("is the fraction of the width, clamped to the track", () => {
    expect(fillWidth(0.25, 200)).toBe(50);
    expect(fillWidth(1, 200)).toBe(200);
    expect(fillWidth(1.5, 200)).toBe(200);
    expect(fillWidth(-0.5, 200)).toBe(0);
  });

  test("no data draws no fill", () => {
    expect(fillWidth(null, 200)).toBe(0);
  });

  test("a tiny value keeps a tiny positive width", () => {
    expect(fillWidth(0.001, 200)).toBeGreaterThan(0);
  });
});
