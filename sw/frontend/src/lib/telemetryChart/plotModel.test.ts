import { describe, expect, test } from "bun:test";
import { buildPlotModel, domainFor, fracOf, type PlotInput, type PlotSeries } from "./plotModel";
import { DEFAULT_GUTTERS } from "./gutter";

const NOW = 100_000;
const series = (over: Partial<PlotSeries> = {}): PlotSeries => ({
  id: 1,
  name: "Engine RPM",
  unit: "rpm",
  color: "#01aaff",
  range: [0, 100],
  t: [NOW - 30_000, NOW - 15_000, NOW],
  v: [0, 50, 100],
  ...over,
});
const input = (over: Partial<PlotInput> = {}): PlotInput => ({
  width: 600,
  height: 230,
  mode: "overlay",
  fit: false,
  laneH: 90,
  win: { len: 60, endAgo: 0 },
  now: NOW,
  series: [series()],
  hoverF: null,
  ...over,
});
const finiteNumbers = (s: string) => !/NaN|Infinity/.test(s);

describe("domainFor", () => {
  test("uses the PID range unless fitting to the visible data", () => {
    expect(domainFor([0, 7000], [800, 900], false, true)).toEqual([0, 7000]);
    const [lo, hi] = domainFor([0, 7000], [800, 900], true, true);
    expect(lo).toBeLessThan(800);
    expect(hi).toBeGreaterThan(900);
  });

  test("falls back to the visible data when the PID has no range", () => {
    const [lo, hi] = domainFor(null, [4, 9], false, true);
    expect(lo).toBeLessThan(4);
    expect(hi).toBeGreaterThan(9);
  });

  test("a flat series still gets a non-zero domain", () => {
    const [lo, hi] = domainFor(null, [5, 5], true, true);
    expect(hi).toBeGreaterThan(lo);
  });
});

describe("buildPlotModel", () => {
  test("no series means no data and no lines", () => {
    const m = buildPlotModel(input({ series: [] }));
    expect(m.hasData).toBe(false);
    expect(m.lines).toEqual([]);
  });

  test("every coordinate is finite for hostile samples", () => {
    const m = buildPlotModel(
      input({
        series: [
          series({ v: [Number.NaN, null, Number.POSITIVE_INFINITY] }),
          series({ id: 2, range: null, t: [NOW - 1000], v: [1e300] }),
        ],
      }),
    );
    expect(finiteNumbers(JSON.stringify(m))).toBe(true);
  });

  test("a single sample still draws a path", () => {
    const m = buildPlotModel(input({ series: [series({ t: [NOW], v: [50] })] }));
    expect(m.lines[0].d.startsWith("M")).toBe(true);
    expect(m.hasData).toBe(true);
  });

  test("overlay maps the PID range to the plot height", () => {
    const m = buildPlotModel(input({ series: [series({ t: [NOW], v: [50] }), series({ id: 2, t: [NOW], v: [0] })] }));
    const mid = m.bottom - 0.5 * (m.bottom - m.T);
    expect(m.lines[0].d).toBe(`M${(m.L + m.pw).toFixed(1)},${mid.toFixed(1)}l0.01,0`);
  });

  test("lanes get one lane per PID and their own height", () => {
    const m = buildPlotModel(
      input({ mode: "lanes", series: [series(), series({ id: 2, name: "Speed", unit: "km/h" })] }),
    );
    expect(m.lanes.length).toBe(2);
    expect(m.H).toBe(m.T + 2 * 90 + m.B);
    expect(m.lanes[0].top).toBeLessThan(m.lanes[1].top);
  });

  test("the cursor reports values and a time label at its position", () => {
    const m = buildPlotModel(input({ hoverF: 1 }));
    expect(m.cursor?.ago).toBe(0);
    expect(m.cursor?.rows[0].text).toBe("100");
  });

});

describe("fracOf", () => {
  test("maps a value to its position in the domain", () => {
    expect(fracOf(50, 0, 100)).toBe(0.5);
  });

  test("huge finite values never give NaN", () => {
    expect(fracOf(1e308, -1e308, 1e308)).toBeCloseTo(1, 9);
    expect(fracOf(-1e308, -1e308, 1e308)).toBeCloseTo(0, 9);
    expect(Number.isFinite(fracOf(Number.MAX_VALUE, -Number.MAX_VALUE, Number.MAX_VALUE))).toBe(true);
  });

  test("a model over huge samples has only finite path coordinates", () => {
    const m = buildPlotModel(
      input({
        series: [series({ range: null, t: [NOW - 1000, NOW], v: [-1e308, 1e308] }), series({ id: 2, range: [-1e308, 1e308], t: [NOW], v: [1e308] })],
      }),
    );
    expect(/NaN|Infinity/.test(JSON.stringify(m.lines))).toBe(false);
  });
});

describe("time labels under the cursor", () => {
  test("hovering removes no tick, and the time chip sits on its own row below the labels", () => {
    const base = input({ width: 600, win: { len: 60, endAgo: 0 } });
    const idle = buildPlotModel({ ...base, hoverF: null });
    const hovered = buildPlotModel({ ...base, hoverF: 0.5 });
    expect(hovered.timeTicks.map((t) => t.text)).toEqual(idle.timeTicks.map((t) => t.text));
    expect(hovered.bottom).toBe(idle.bottom);
    const chipY = hovered.cursor!.chipY;
    expect(chipY - hovered.bottom).toBeGreaterThanOrEqual(18 + 6); // below the label baseline
    expect(chipY + 20).toBeLessThanOrEqual(hovered.H);
  });
});

describe("plot edges", () => {
  test("the line starts at the left edge of the plot and ends at the right edge in live mode", () => {
    const m = buildPlotModel(
      input({
        win: { len: 60, endAgo: 0 },
        now: NOW,
        series: [series({ t: [NOW - 90_000, NOW - 30_000, NOW - 500], v: [10, 20, 30] })],
      }),
    );
    const xs = [...m.lines[0].d.matchAll(/[ML]([\d.]+),/g)].map((r) => Number(r[1]));
    expect(xs[0]).toBeCloseTo(m.L, 0);
    expect(xs[xs.length - 1]).toBeCloseTo(m.L + m.pw, 0);
  });

  test("every mode uses the shared gutters, so a wide lane readout does not move the plot edge", () => {
    const narrow = buildPlotModel(input({ mode: "lanes", series: [series({ t: [NOW], v: [6] })] }));
    const wide = buildPlotModel(input({ mode: "lanes", series: [series({ t: [NOW], v: [1234567] })] }));
    expect(narrow.L).toBe(DEFAULT_GUTTERS.left);
    expect(narrow.R).toBe(DEFAULT_GUTTERS.right);
    expect(wide.R).toBe(DEFAULT_GUTTERS.right);
  });
});

describe("range covers the data", () => {
  test("a definition range smaller than the data widens to cover it", () => {
    expect(domainFor([0, 100], [150, 20], false, false)).toEqual([0, 150]);
    expect(domainFor([0, 100], [-30, 20], false, false)).toEqual([-30, 100]);
  });

  test("out-of-range values stay inside the plot area", () => {
    const m = buildPlotModel(
      input({
        series: [series({ t: [NOW - 2000, NOW - 1000, NOW], v: [150, -40, 60] })],
      }),
    );
    const ys = [...m.lines[0].d.matchAll(/[ML][\d.]+,([\d.]+)/g)].map((r) => Number(r[1]));
    expect(ys.length).toBeGreaterThan(0);
    for (const y of ys) {
      expect(y).toBeGreaterThanOrEqual(m.T - 0.05);
      expect(y).toBeLessThanOrEqual(m.bottom + 0.05);
    }
  });
});

describe("stats now", () => {
  test("Now is the value under the cursor while the cursor is on the plot", () => {
    const s = [series({ t: [NOW - 30_000, NOW - 15_000, NOW], v: [0, 50, 42] })];
    expect(buildPlotModel(input({ series: s, hoverF: 0.5 })).stats[0].now).toBe(0);
    expect(buildPlotModel(input({ series: s, hoverF: 1 })).stats[0].now).toBe(42);
  });

  test("Now is the value at the window end, and null when the window ends before any sample", () => {
    const m = buildPlotModel(input({ series: [series({ t: [NOW - 30_000, NOW - 15_000, NOW], v: [0, 50, 42] })] }));
    expect(m.stats[0].now).toBe(42);
    const early = buildPlotModel(input({ win: { len: 60, endAgo: 300 }, series: [series()] }));
    expect(early.stats[0]?.now ?? null).toBe(null);
  });
});
