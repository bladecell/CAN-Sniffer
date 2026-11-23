import { describe, expect, test } from "bun:test";
import { buildAnchor, holdEndsOf, needsRebuild, scrollFrame, SCROLL_HEADROOM_MS } from "./scroll";
import { buildPlotModel, type PlotSeries } from "./plotModel";

describe("scroll anchor", () => {
  test("the anchor is the present plus headroom when no hold ends before it", () => {
    expect(buildAnchor(10_000, [])).toBe(10_000 + SCROLL_HEADROOM_MS);
  });

  test("the anchor stops at the first hold that ends after the present", () => {
    expect(buildAnchor(10_000, [10_100, 50_000])).toBe(10_100);
  });

  test("a hold that already ended does not constrain the anchor", () => {
    expect(buildAnchor(10_000, [9_000])).toBe(10_000 + SCROLL_HEADROOM_MS);
  });

  test("a frame past the anchor needs a rebuild", () => {
    const anchor = buildAnchor(10_000, []);
    expect(needsRebuild(anchor, anchor - 1, [])).toBe(false);
    expect(needsRebuild(anchor, anchor, [])).toBe(true);
  });

  test("a hold that ends before the anchor needs a rebuild, and the rebuilt anchor does not", () => {
    const holds = [10_100];
    const anchor = buildAnchor(10_000, []);
    expect(needsRebuild(anchor, 10_000, holds)).toBe(true);
    const next = buildAnchor(10_000, holds);
    expect(needsRebuild(next, 10_000, holds)).toBe(false);
    expect(needsRebuild(next, 10_100, holds)).toBe(true);
  });
});

describe("hold ends", () => {
  test("a series ends its hold at its last value plus the hold, shifted by the window end", () => {
    const s = { t: [1000, 2000], v: [1, 2] as (number | null)[], holdMs: 1500 };
    expect(holdEndsOf([s], 0)).toEqual([3500]);
    expect(holdEndsOf([s], 4000)).toEqual([7500]);
  });

  test("a series whose last sample is a gap has no hold to end", () => {
    expect(holdEndsOf([{ t: [1000, 2000], v: [1, null], holdMs: 1500 }], 0)).toEqual([]);
  });
});

describe("translated geometry", () => {
  // Geometry built for the anchor and shifted by the offset must land where geometry built at the exact present does.
  const samples = (() => {
    const t: number[] = [];
    const v: (number | null)[] = [];
    for (let k = 0; k <= 100; k++) {
      t.push(k * 500);
      v.push(50 + 20 * Math.sin(k / 6));
    }
    return { t, v };
  })();
  const series = (): PlotSeries[] => [{ id: 1, name: "A", unit: "", color: "#fff", range: [0, 100], holdMs: 1500, ...samples }];
  const build = (now: number, len = 60) =>
    buildPlotModel({ width: 640, height: 200, mode: "overlay", fit: false, laneH: 90, win: { len, endAgo: 0 }, now, series: series(), hoverF: null });
  const verts = (d: string) => [...d.matchAll(/[ML]([-\d.]+),([-\d.]+)/g)].map((m) => ({ x: +m[1], y: +m[2] }));

  test("shifting the anchor geometry by the offset matches the exact present geometry", () => {
    const presentNow = 49_900;
    const anchor = buildAnchor(presentNow, holdEndsOf(series(), 0));
    const built = build(anchor);
    const exact = build(presentNow);
    const offset = (anchor - presentNow) * (built.pw / (60 * 1000));
    const bv = verts(built.lines[0].d);
    // Interior vertices are samples, which both builds share. The edge vertices are interpolated per build.
    const ev = verts(exact.lines[0].d).filter((p) => p.x > exact.L + 0.5 && p.x < exact.W - exact.R - 0.5);
    expect(ev.length).toBeGreaterThan(50);
    for (const p of ev) {
      // Nearest built vertex after the shift: both are rounded to 0.1 px, so 0.1 px is the bound.
      const gap = Math.min(...bv.map((b) => Math.hypot(b.x + offset - p.x, b.y - p.y)));
      expect(gap).toBeLessThan(0.1);
    }
  });
});

describe("scroll frame across a rebuild", () => {
  test("the anchor is kept until the present reaches it, then moves 250 ms ahead", () => {
    const first = buildAnchor(1000, []);
    expect(first).toBe(1250);
    expect(scrollFrame(first, 1000, [])).toEqual({ anchor: 1250, present: 1000 });
    expect(scrollFrame(first, 1249, [])).toEqual({ anchor: 1250, present: 1249 });
    expect(scrollFrame(first, 1250, [])).toEqual({ anchor: 1500, present: 1250 });
  });

  test("a hold that ends inside the headroom pulls the rebuild forward to the hold end", () => {
    const anchor = buildAnchor(1000, [1400]);
    expect(anchor).toBe(1250);
    const rebuilt = scrollFrame(anchor, 1250, [1400]);
    expect(rebuilt).toEqual({ anchor: 1400, present: 1250 });
    expect(scrollFrame(rebuilt.anchor, 1400, [1400])).toEqual({ anchor: 1650, present: 1400 });
  });
});
