import { describe, expect, test } from "bun:test";
import { ageText, hzText, loadText, relTime, sparkPoints, twaiLabel } from "./canFormat";

describe("canFormat", () => {
  test("relative time from the first frame", () => {
    expect(relTime(1_500_000, 1_000_000)).toBe("0.500");
    expect(relTime(5, null)).toBe("0.000");
  });

  test("rates, load and states", () => {
    expect(hzText(9.6)).toBe("9.6 Hz");
    expect(hzText(10.4)).toBe("10 Hz");
    expect(hzText(0.5)).toBe("0.5 Hz");
    expect(loadText(270)).toBe("27.0%");
    expect(loadText(null)).toBe("–");
    expect(twaiLabel("bus_off")).toBe("Bus off");
    expect(twaiLabel(undefined)).toBe("Unknown");
    expect(twaiLabel("novel")).toBe("novel");
  });

  test("age", () => {
    expect(ageText(10_000, 10_400)).toBe("now");
    expect(ageText(10_000, 24_000)).toBe("14 s ago");
  });

  test("sparkline is flat when empty and stays inside the box", () => {
    expect(sparkPoints([])).toBe("0,15 100,15");
    const pts = sparkPoints([0, 10, 5]).split(" ").map((p) => p.split(",").map(Number));
    expect(pts.length).toBe(3);
    for (const [x, y] of pts) {
      expect(x).toBeGreaterThanOrEqual(0);
      expect(x).toBeLessThanOrEqual(100);
      expect(y).toBeGreaterThanOrEqual(0);
      expect(y).toBeLessThanOrEqual(30);
    }
  });
});
