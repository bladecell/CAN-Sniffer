import { describe, expect, test } from "bun:test";
import { atEnd, clampMs, holdPause, playheadOf, seekWin } from "./playback";
import { liveWin, type Win } from "./window";

describe("playback timeline", () => {
  test("clampMs keeps the playhead inside the trip", () => {
    expect(clampMs(-5, 100, 200)).toBe(100);
    expect(clampMs(250, 100, 200)).toBe(200);
    expect(clampMs(150, 100, 200)).toBe(150);
    expect(clampMs(Number.NaN, 100, 200)).toBe(100);
  });

  test("playheadOf is the right edge of the visible window", () => {
    expect(playheadOf({ len: 60, endAgo: 0, pausedAt: 5000 }, 5000, 0, 9000)).toBe(5000);
    expect(playheadOf({ len: 60, endAgo: 10, pausedAt: 50000 }, 50000, 0, 9e9)).toBe(40000);
    expect(playheadOf(liveWin(60), null, 100, 200)).toBe(100);
    expect(playheadOf(liveWin(60), 900, 100, 200)).toBe(200);
  });

  test("seekWin pauses at the target, or plays live from it", () => {
    expect(seekWin(30, 4000, false)).toEqual({ len: 30, endAgo: 0, pausedAt: 4000 });
    expect(seekWin(30, 4000, true)).toEqual({ len: 30, endAgo: 0, pausedAt: null });
  });

  test("atEnd is true from the last sample on", () => {
    expect(atEnd(199, 200)).toBe(false);
    expect(atEnd(200, 200)).toBe(true);
    expect(atEnd(300, 200)).toBe(true);
  });
});

describe("holdPause", () => {
  const paused: Win = { len: 60, endAgo: 20, pausedAt: 50000 };

  test("a window move never starts playback: it stays paused at the same present", () => {
    expect(holdPause(paused, liveWin(60))).toEqual({ len: 60, endAgo: 0, pausedAt: 50000 });
    expect(holdPause(paused, { len: 30, endAgo: 0, pausedAt: null })).toEqual({ len: 30, endAgo: 0, pausedAt: 50000 });
  });

  test("pausing, panning and playing windows pass through", () => {
    const moved: Win = { len: 60, endAgo: 5, pausedAt: 50000 };
    expect(holdPause(paused, moved)).toBe(moved);
    const playing = liveWin(60);
    expect(holdPause(playing, playing)).toBe(playing);
    expect(holdPause(playing, { ...playing, pausedAt: 7000 })).toEqual({ len: 60, endAgo: 0, pausedAt: 7000 });
  });
});
