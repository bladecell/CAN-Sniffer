import { describe, expect, test } from "bun:test";
import {
  SPAN_SEC,
  createPresentClock,
  freezeWin,
  guardWin,
  keyWin,
  liveWin,
  panWin,
  resumeWin,
  setLenWin,
  shiftWin,
  winStatus,
  zoomWin,
} from "./window";

describe("window state", () => {
  test("scrubbing back from live pauses at the present", () => {
    const w = shiftWin(liveWin(60), 0, 20, 1000);
    expect(w).toEqual({ len: 60, endAgo: 20, pausedAt: 1000 });
    expect(winStatus(w)).toBe("history");
  });

  test("moving back to the present resumes live", () => {
    const w = shiftWin({ len: 60, endAgo: 20, pausedAt: 1000 }, 0, -20, 2000);
    expect(w).toEqual({ len: 60, endAgo: 0, pausedAt: null });
    expect(winStatus(w)).toBe("live");
  });

  test("a manual pause stays paused when the window does not move", () => {
    const paused = freezeWin(liveWin(60), 1000);
    expect(winStatus(paused)).toBe("paused");
    expect(winStatus(shiftWin(paused, 0, 0, 1500))).toBe("paused");
  });

  test("the end is clamped so the window stays inside the retained span", () => {
    expect(shiftWin(liveWin(60), 0, 1000, 5).endAgo).toBe(SPAN_SEC - 60);
  });

  test("the length is clamped between 10 s and the span", () => {
    expect(shiftWin(liveWin(60), 1000, 0, 0).len).toBe(SPAN_SEC);
    expect(shiftWin(liveWin(60), -1000, 0, 0).len).toBe(10);
  });

  test("zooming around the right edge of a live window keeps it live", () => {
    expect(zoomWin(liveWin(60), 1 / 1.5, 1, 1000)).toEqual({ len: 40, endAgo: 0, pausedAt: null });
  });

  test("zooming around the left edge keeps the oldest visible time", () => {
    const w = zoomWin({ len: 60, endAgo: 30, pausedAt: 1000 }, 2 / 3, 0, 1000);
    expect(w.len).toBe(40);
    expect(w.endAgo + w.len).toBe(90);
  });

  test("panning moves the window back by the dragged fraction", () => {
    expect(panWin(liveWin(60), 0.1, 1000)).toEqual({ len: 60, endAgo: 6, pausedAt: 1000 });
  });

  test("changing the length keeps the end", () => {
    expect(setLenWin({ len: 60, endAgo: 30, pausedAt: 1000 }, 30, 1000)).toEqual({
      len: 30,
      endAgo: 30,
      pausedAt: 1000,
    });
  });

  test("resume returns to a live window of the same length", () => {
    expect(resumeWin({ len: 30, endAgo: 50, pausedAt: 9 })).toEqual({ len: 30, endAgo: 0, pausedAt: null });
  });

  test("non-finite inputs fall back to safe values", () => {
    expect(liveWin(Number.NaN).len).toBe(60);
    expect(shiftWin(liveWin(60), Number.NaN, Number.NaN, 1000)).toEqual(liveWin(60));
  });
});

describe("present clock", () => {
  test("is unknown until a sample arrives", () => {
    expect(createPresentClock().now(0)).toBeNull();
  });

  test("extrapolates from the newest sample with the browser clock", () => {
    const c = createPresentClock();
    c.observe(1000, 0);
    expect(c.now(500)).toBe(1500);
  });

  test("ignores samples older than the newest one", () => {
    const c = createPresentClock();
    c.observe(1000, 0);
    c.observe(900, 600);
    expect(c.now(600)).toBe(1600);
  });

  test("a large backwards step is a device reboot and re-anchors", () => {
    const c = createPresentClock();
    c.observe(1650, 0);
    c.observe(50, 800);
    expect(c.now(800)).toBe(50);
  });

  test("rejects non-finite timestamps", () => {
    const c = createPresentClock();
    c.observe(Number.NaN, 0);
    expect(c.now(0)).toBeNull();
  });
});

describe("guardWin", () => {
  test("without data there is no present, so the window cannot leave live", () => {
    const g = guardWin({ len: 60, endAgo: 20, pausedAt: 0 }, null);
    expect(g).toEqual({ len: 60, endAgo: 0, pausedAt: null });
  });

  test("with data the window is unchanged", () => {
    const w = { len: 60, endAgo: 20, pausedAt: 1000 };
    expect(guardWin(w, 2000)).toEqual(w);
  });
});

describe("keyWin", () => {
  test("arrows pan, +/- zoom, Home and 0 reset to live 1 min", () => {
    const live = liveWin(60);
    expect(keyWin(live, "ArrowLeft", 1000)?.endAgo).toBe(6);
    expect(keyWin(live, "ArrowRight", 1000)?.endAgo).toBe(0);
    expect(keyWin(live, "+", 1000)?.len).toBeCloseTo(40, 6);
    expect(keyWin(live, "-", 1000)?.len).toBeCloseTo(90, 6);
    expect(keyWin({ len: 300, endAgo: 0, pausedAt: null }, "Home", 1000)).toEqual(liveWin(60));
    expect(keyWin(live, "0", 1000)).toEqual(liveWin(60));
  });

  test("other keys are not handled", () => {
    expect(keyWin(liveWin(60), "a", 1000)).toBeNull();
  });
});
