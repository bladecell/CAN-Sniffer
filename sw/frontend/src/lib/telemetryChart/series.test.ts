import { describe, expect, test } from "bun:test";
import { decimate, statsOf, valueAt, windowPoints, type Pt } from "./series";

describe("decimate", () => {
  test("leaves short series alone", () => {
    const pts: Pt[] = [{ x: 0, y: 1 }, { x: 1, y: 2 }];
    expect(decimate(pts, 100)).toBe(pts);
  });

  test("keeps the extremes of every pixel column", () => {
    const pts: Pt[] = [];
    for (let i = 0; i < 1000; i++) pts.push({ x: i / 10, y: i === 500 ? 999 : i === 700 ? -5 : 1 });
    const out = decimate(pts, 100).filter((p): p is { x: number; y: number } => p !== null);
    expect(out.length).toBeLessThanOrEqual(4 * 100 + 100);
    expect(out.some((p) => p.y === 999)).toBe(true);
    expect(out.some((p) => p.y === -5)).toBe(true);
  });

  test("gaps stay breaks", () => {
    const pts: Pt[] = [];
    for (let i = 0; i < 300; i++) pts.push(i === 150 ? null : { x: i / 3, y: i });
    expect(decimate(pts, 20).includes(null)).toBe(true);
  });
});

describe("valueAt", () => {
  test("is the latest sample at or before the time", () => {
    expect(valueAt([10, 20, 30], [1, 2, 3], 25)).toBe(2);
    expect(valueAt([10, 20, 30], [1, 2, 3], 30)).toBe(3);
  });

  test("is null before the first sample or on a gap", () => {
    expect(valueAt([10], [1], 5)).toBeNull();
    expect(valueAt([10, 20], [1, null], 25)).toBeNull();
  });
});

describe("statsOf", () => {
  test("min, average and max over the window, skipping gaps and NaN", () => {
    const s = statsOf([0, 10, 20, 30, 40], [1, 3, null, Number.NaN, 5], 0, 40);
    expect(s).toEqual({ min: 1, avg: 3, max: 5 });
  });

  test("empty windows have no stats", () => {
    expect(statsOf([0], [1], 5, 10)).toBeNull();
  });
});

describe("windowPoints", () => {
  const pts = (t: number[], v: (number | null)[], fromT: number, toT: number, now: number, hold = 1500) =>
    windowPoints(t, v, fromT, toT, now, hold);

  test("interpolates the line at the left window edge", () => {
    expect(pts([0, 1000], [0, 10], 500, 2000, 1000)).toEqual([
      { t: 500, v: 5 },
      { t: 1000, v: 10 },
    ]);
  });

  test("interpolates at the right window edge when the window ends between samples", () => {
    const out = pts([0, 1000, 2000], [0, 10, 20], -1, 1500, 1500);
    expect(out[out.length - 1]).toEqual({ t: 1500, v: 15 });
  });

  test("data that starts less than one sample interval after the window start begins at the edge", () => {
    expect(pts([1200, 2200], [4, 5], 1000, 2500, 2500)[0]).toEqual({ t: 1000, v: 4 });
  });

  test("data that starts a full interval or more after the window start leaves the edge empty", () => {
    expect(pts([5000, 6000], [1, 2], 1000, 6500, 6500)[0]).toEqual({ t: 5000, v: 1 });
  });

  test("a live series holds its last value to the present while within the hold", () => {
    expect(pts([0, 1000], [0, 10], 0, 2000, 2000, 1500)).toEqual([
      { t: 0, v: 0 },
      { t: 1000, v: 10 },
      { t: 2000, v: 10 },
    ]);
  });

  test("a stale series does not extend: the line ends in a gap", () => {
    expect(pts([0, 1000], [0, 10], 0, 5000, 5000, 1500)).toEqual([
      { t: 0, v: 0 },
      { t: 1000, v: 10 },
    ]);
  });

  test("a single sample in the window draws, and holds when fresh", () => {
    expect(pts([1000], [7], 0, 1200, 1200)).toEqual([
      { t: 1000, v: 7 },
      { t: 1200, v: 7 },
    ]);
  });

  test("gaps stay as null breaks", () => {
    const out = pts([0, 500, 1000], [1, null, 3], 0, 1000, 1000);
    expect(out.map((p) => p.v)).toEqual([1, null, 3]);
  });

  test("no samples means no points", () => {
    expect(pts([], [], 0, 1000, 1000)).toEqual([]);
  });
});
