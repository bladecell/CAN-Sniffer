import { describe, expect, test } from "bun:test";
import { barFraction, clampToRange, equalThirds, fillWidth, gradientStops, litSegments, markerOffset, segmentLayout } from "./valueBar";

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

describe("clampToRange (gauge value on its scale)", () => {
  test("a value inside the scale is unchanged", () => {
    expect(clampToRange(12.6, 9.5, 15.5)).toBe(12.6);
  });

  test("below the scale is pinned to min (0.9 V on a 9.5 to 15.5 V gauge)", () => {
    expect(clampToRange(0.9, 9.5, 15.5)).toBe(9.5);
    expect(clampToRange(-3, 9.5, 15.5)).toBe(9.5);
  });

  test("above the scale is pinned to max", () => {
    expect(clampToRange(20, 9.5, 15.5)).toBe(15.5);
  });

  test.each([
    ["NaN", Number.NaN],
    ["Infinity", Number.POSITIVE_INFINITY],
    ["-Infinity", Number.NEGATIVE_INFINITY],
  ])("%s is treated as min, never passed on to the angle", (_label, value) => {
    expect(clampToRange(value, 9.5, 15.5)).toBe(9.5);
  });

  test("a reversed scale (max not above min) returns min", () => {
    expect(clampToRange(5, 10, 10)).toBe(10);
  });
});

describe("segmentLayout", () => {
  test("a 200 px bar fits 25 blocks of 6 px plus 2 px gaps", () => {
    const l = segmentLayout(200);
    expect(l.count).toBe(25);
    expect(l.count * l.width + (l.count - 1) * l.gap).toBeCloseTo(200, 6);
  });

  test("the block count is capped", () => {
    expect(segmentLayout(10_000).count).toBe(40);
  });

  test("a bar narrower than one block still gets one block", () => {
    expect(segmentLayout(3)).toMatchObject({ count: 1, width: 3 });
  });

  test.each([
    ["zero", 0],
    ["NaN", Number.NaN],
    ["negative", -10],
    ["Infinity", Number.POSITIVE_INFINITY],
  ])("%s width has no blocks", (_label, width) => {
    expect(segmentLayout(width).count).toBe(0);
  });
});

describe("litSegments", () => {
  test("rounds the fraction of the block count", () => {
    expect(litSegments(0.5, 10)).toBe(5);
    expect(litSegments(1, 25)).toBe(25);
    expect(litSegments(0, 25)).toBe(0);
  });

  test("no data lights nothing", () => {
    expect(litSegments(null, 25)).toBe(0);
  });

  test("a small positive value lights at least one block", () => {
    expect(litSegments(0.001, 25)).toBe(1);
    expect(litSegments(0.001, 0)).toBe(0);
  });

  test("NaN lights nothing", () => {
    expect(litSegments(Number.NaN, 25)).toBe(0);
  });

  test("a value above 1 lights no more than the count", () => {
    expect(litSegments(3, 25)).toBe(25);
  });
});

describe("gradientStops", () => {
  test("without zones, the bar colour runs at full strength from end to end", () => {
    expect(gradientStops("#ffffff", [])).toEqual([
      { offset: 0, color: "#ffffff", opacity: 1 },
      { offset: 1, color: "#ffffff", opacity: 1 },
    ]);
  });

  test("PID thirds keep the bar colour and step the opacity", () => {
    const stops = gradientStops("#abcdef", equalThirds());
    expect(stops.map((s) => s.offset)).toEqual([0, 1 / 3, 2 / 3, 1]);
    expect(stops.every((s) => s.color === "#abcdef")).toBe(true);
    expect(stops.map((s) => s.opacity)).toEqual([0.4, 0.7, 1, 1]);
  });

  test("battery zones keep their own colours", () => {
    const stops = gradientStops("#000000", [
      { from: 0, to: 0.5, opacity: 1, color: "#ff0000" },
      { from: 0.5, to: 1, opacity: 1, color: "#00ff00" },
    ]);
    expect(stops.map((s) => s.offset)).toEqual([0, 0.5, 1]);
    expect(stops.map((s) => s.color)).toEqual(["#ff0000", "#00ff00", "#00ff00"]);
  });
});

describe("markerOffset", () => {
  test("centres the marker on the value and keeps it inside the track", () => {
    expect(markerOffset(0.5, 100, 10)).toBe(45);
    expect(markerOffset(0, 100, 10)).toBe(0);
    expect(markerOffset(1, 100, 10)).toBe(90);
  });

  test("no data has no marker", () => {
    expect(markerOffset(null, 100, 10)).toBeNull();
  });

  test("a NaN fraction has no marker, never NaN", () => {
    expect(markerOffset(Number.NaN, 100, 10)).toBeNull();
  });
});
