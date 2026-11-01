import { describe, expect, test } from "bun:test";
import { OV_L, OV_R, grabAt, jumpTo, overviewDrag, overviewModel, ovAgo, ovX, dragWin, toViewX } from "./overview";
import { liveWin } from "./window";

describe("overview geometry", () => {
  test("the newest time is on the right, five minutes back on the left", () => {
    expect(ovX(0, 500)).toBe(500 - OV_R);
    expect(ovX(300, 500)).toBe(OV_L);
    expect(ovAgo(ovX(120, 500), 500)).toBeCloseTo(120, 6);
  });

  test("grabs the edges, the frame or the strip", () => {
    const w = { len: 60, endAgo: 0, pausedAt: null };
    expect(grabAt(ovX(60, 500), w, 500)).toBe("left");
    expect(grabAt(ovX(0, 500), w, 500)).toBe("right");
    expect(grabAt(ovX(30, 500), w, 500)).toBe("move");
    expect(grabAt(ovX(200, 500), w, 500)).toBe("jump");
  });

  test("dragging the frame moves it, the edges zoom", () => {
    const start = liveWin(60);
    const pw = 500 - OV_L - OV_R;
    const dx = (30 / 300) * pw;
    expect(dragWin(start, "move", dx, 500, 1000)).toEqual({ len: 60, endAgo: 0, pausedAt: null }); // live: toward the present stays live
    const back = dragWin(start, "move", -dx, 500, 1000);
    expect(back.endAgo).toBeCloseTo(30, 6);
    const zoom = dragWin(start, "left", -dx, 500, 1000);
    expect(zoom.len).toBeCloseTo(90, 6);
  });

  test("a click on the strip centres the window there", () => {
    const w = jumpTo(liveWin(60), 150, 1000);
    expect(w.endAgo).toBeCloseTo(120, 6);
    expect(w.pausedAt).toBe(1000);
  });

  test("the model has a finite frame and lines for hostile input", () => {
    const m = overviewModel({
      width: 500,
      height: 84,
      now: 100_000,
      win: { len: 60, endAgo: 0, pausedAt: null },
      series: [{ color: "#fff", range: null, t: [99_000, 100_000], v: [Number.NaN, 4] }],
    });
    expect(m.frame.x1).toBeGreaterThan(m.frame.x0);
    expect(/NaN|Infinity/.test(JSON.stringify(m))).toBe(false);
  });
});

describe("client to view coordinates", () => {
  test("a strip drawn narrower than its viewBox still grabs the right edge", () => {
    // Viewbox 362 units, drawn 322 px wide (390 px phone). The right handle is at ovX(0, 362).
    const viewW = 362;
    const drawnW = 322;
    const handleView = ovX(0, viewW);
    const clientX = (handleView * drawnW) / viewW;
    const x = toViewX(clientX, 0, drawnW, viewW);
    expect(x).toBeCloseTo(handleView, 6);
    expect(grabAt(x, liveWin(60), viewW)).toBe("right");
  });

  test("without a measured rect it falls back to plain offset", () => {
    expect(toViewX(30, 10, 0, 362)).toBe(20);
  });
});

describe("overview drag", () => {
  test("the frame moves one display px per finger px when the strip is drawn narrower than its viewBox", () => {
    const viewW = 362;
    const drawn = 322;
    const rect = { left: 100, width: drawn };
    const start = { len: 60, endAgo: 100, pausedAt: 1000 };
    const frameDisplayX = (ago: number) => rect.left + (ovX(ago, viewW) * drawn) / viewW;
    const grabX = frameDisplayX(start.endAgo + start.len / 2);
    const d = 10;
    const w = overviewDrag(start, "move", rect, grabX, grabX + d, viewW, 1000);
    expect(frameDisplayX(w.endAgo) - frameDisplayX(start.endAgo)).toBeCloseTo(d, 6);
  });
});

describe("overview line edges", () => {
  const NOW = 1_000_000;
  const firstX = (t: number[], width = 500) => {
    const v = t.map((_, i) => i);
    const m = overviewModel({ width, height: 84, now: NOW, win: liveWin(60), series: [{ color: "#000", range: [0, 100], t, v }] });
    const x = m.lines[0].d.match(/^M([\d.]+),/);
    return x ? Number(x[1]) : NaN;
  };
  const every = (fromAgoMs: number, stepMs = 1000) => {
    const out: number[] = [];
    for (let t = NOW - fromAgoMs; t <= NOW; t += stepMs) out.push(t);
    return out;
  };

  test("data older than five minutes starts the line at the left edge, the five-minute mark", () => {
    expect(firstX(every(400_000))).toBeCloseTo(ovX(300, 500), 1);
  });

  test("data that starts less than one sample interval after five minutes begins at the edge too", () => {
    expect(firstX(every(299_500))).toBeCloseTo(ovX(300, 500), 1);
  });

  test("a slow PID (one sample every 5 s) still reaches the left edge", () => {
    expect(firstX(every(302_000, 5000))).toBeCloseTo(ovX(300, 500), 1);
  });

  test("a buffer shorter than five minutes starts at its first sample, not at the edge", () => {
    expect(firstX(every(180_000))).toBeCloseTo(ovX(180, 500), 1);
  });
});
