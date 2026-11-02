// UI15: gaps after leaving Telemetry and coming back. Tests marked "BUG" are red on purpose until fixed.
import { describe, expect, test } from "bun:test";
import { createChartBuffer } from "../chartBuffer";
import { createChartHistory } from "../chartHistory";
import type { PidDefinition } from "../types";
import { buildPlotModel, type PlotInput } from "./plotModel";

const NOW = 100_000;
const SPLIT = 2000;

/** Samples every `step` ms over [from, to] through the real buffer, so gaps get their NaN break like live data. */
function fill(buf: ReturnType<typeof createChartBuffer>, from: number, to: number, step = 500) {
  for (let t = from; t <= to; t += step) buf.push(t, 20 + ((t / 500) % 10), SPLIT);
}

function model(buf: ReturnType<typeof createChartBuffer>, over: Partial<PlotInput> = {}) {
  const s = buf.snapshot(NOW - 61_000, 1);
  return buildPlotModel({
    width: 600, height: 230, mode: "overlay", fit: false, laneH: 90,
    win: { len: 60, endAgo: 0 }, now: NOW, hoverF: null,
    series: [{ id: 1, name: "Engine Load", unit: "%", color: "#01aaff", range: [0, 100], holdMs: 1500, t: s.t, v: s.v }],
    ...over,
  });
}

const subpaths = (d: string) => d.split(/(?=M)/).filter(Boolean);
const verts = (sp: string) => [...sp.matchAll(/[ML](-?[\d.]+),(-?[\d.]+)/g)].map((m) => ({ x: +m[1], y: +m[2] }));

// Mount for 0-20 s, unmount 20-50 s (no samples seen), remount 50-100 s; window is 40-100 s.
function gapBuffer() {
  const b = createChartBuffer();
  fill(b, 45_000, 60_000);
  fill(b, 85_000, 100_000);
  return b;
}

describe("(a) line breaks at a long gap", () => {
  test("two sub-paths, and no line segment spans the gap", () => {
    const m = model(gapBuffer());
    const sp = subpaths(m.lines[0].d);
    expect(sp.length).toBe(2);
    const endOfFirst = verts(sp[0]).at(-1)!.x;
    const startOfSecond = verts(sp[1])[0].x;
    expect(startOfSecond - endOfFirst).toBeGreaterThan(100);
  });
});

describe("(b) area per segment", () => {
  // BUG: plotModel.ts:164 appends one baseline close ("L last,bot L first,bot Z") to the whole multi-segment path d.
  // Open sub-paths are implicitly closed by a straight chord when filled, which draws the sloped wedges.
  test("BUG: every segment of the area is closed on its own (one Z per M)", () => {
    const m = model(gapBuffer());
    const area = m.lines[0].area!;
    expect((area.match(/M/g) ?? []).length).toBe((area.match(/Z/g) ?? []).length);
  });

  test("BUG: each area sub-path returns to the baseline directly below its own first point", () => {
    const m = model(gapBuffer());
    const bot = m.bottom;
    for (const sp of subpaths(m.lines[0].area!)) {
      const v = verts(sp);
      const [a, b] = v.slice(-2);
      expect(a.y).toBeCloseTo(bot, 0);
      expect(b.y).toBeCloseTo(bot, 0);
      expect(b.x).toBeCloseTo(v[0].x, 0);
      expect(sp.trim().endsWith("Z")).toBe(true);
    }
  });

  test("a gap-free series closes correctly (current behaviour)", () => {
    const b = createChartBuffer();
    fill(b, 40_000, 100_000);
    const area = model(b).lines[0].area!;
    expect(subpaths(area).length).toBe(1);
    expect(area.endsWith("Z")).toBe(true);
  });
});

describe("(c) data starting mid-window or ending before now", () => {
  test("data that starts mid-window has no line or area left of its first sample", () => {
    const b = createChartBuffer();
    fill(b, 70_000, 100_000);
    const m = model(b);
    expect(m.lines[0].d.startsWith("M")).toBe(true);
    const firstX = verts(m.lines[0].d)[0].x;
    expect(firstX).toBeGreaterThan(m.L + m.pw * 0.4);
    expect(verts(m.lines[0].area!)[0].x).toBeCloseTo(firstX, 0);
  });

  test("data that ended 30 s ago stops there, not held to the present", () => {
    const b = createChartBuffer();
    fill(b, 40_000, 70_000);
    const m = model(b);
    const lastX = verts(m.lines[0].d).at(-1)!.x;
    expect(lastX).toBeLessThan(m.L + m.pw - m.pw * 0.4);
  });
});

describe("(d) the history keeps recording while no chart is mounted", () => {
  const def = { pid: 5, update_interval_ms: 500 } as PidDefinition;
  function harness() {
    let onMessage: ((u: { pid: number; value: number; timestamp: number }) => void) | null = null;
    const defs = [def];
    const history = createChartHistory({ subscribeMessages: (cb) => ((onMessage = cb), () => {}), definitions: () => defs });
    return { history, send: (t: number) => onMessage?.({ pid: 5, value: 1, timestamp: t }) };
  }

  test("samples arriving with no listener are kept, so a remount sees them (no gap if the device kept sending)", () => {
    const h = harness();
    let unsub = h.history.subscribe(5, () => {});
    for (let t = 0; t <= 10_000; t += 500) h.send(t);
    unsub();
    for (let t = 10_500; t <= 40_000; t += 500) h.send(t);
    let size = 0;
    unsub = h.history.subscribe(5, (b) => (size = b.size));
    expect(size).toBe(81);
  });

  test("samples that never arrived (device or socket silent) show up as one NaN break, not as interpolation", () => {
    const h = harness();
    let snap: ReturnType<ReturnType<typeof createChartBuffer>["snapshot"]> | null = null;
    h.history.subscribe(5, (b) => (snap = b.snapshot()));
    h.send(0); h.send(500); h.send(40_000);
    let last: typeof snap = null;
    h.history.subscribe(5, (b) => (last = b.snapshot()));
    expect(last!.v).toEqual([1, 1, null, 1]);
  });
});

describe("(e) single-point segments", () => {
  test("an isolated sample between gaps draws as a stub in the line", () => {
    const b = createChartBuffer();
    fill(b, 45_000, 60_000);
    b.push(75_000, 30, SPLIT);
    fill(b, 90_000, 100_000);
    const d = model(b).lines[0].d;
    expect(subpaths(d).length).toBe(3);
    expect(d).toContain("l0.01,0");
  });

  // BUG: same plotModel.ts:164 cause; a lone point has zero area, but is still joined into the wedge fan.
  test("BUG: a single-point segment gets a closed zero-width area, not a chord to its neighbours", () => {
    const b = createChartBuffer();
    fill(b, 45_000, 60_000);
    b.push(75_000, 30, SPLIT);
    fill(b, 90_000, 100_000);
    const area = model(b).lines[0].area!;
    expect((area.match(/M/g) ?? []).length).toBe((area.match(/Z/g) ?? []).length);
  });
});
