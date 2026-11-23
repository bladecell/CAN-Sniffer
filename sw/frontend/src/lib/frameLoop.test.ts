import { describe, expect, test } from "bun:test";
import { createFrameLoop, type FrameSource } from "./frameLoop";

// Manual frame source: a frame runs only when frame(t) is called with the pending callback.
function fakeFrames() {
  let pending: ((t: number) => void) | null = null;
  let requests = 0;
  const source: FrameSource = {
    request(cb) {
      pending = cb;
      requests++;
      return cb;
    },
  };
  return {
    source,
    frame(t: number) {
      const cb = pending;
      pending = null;
      cb?.(t);
    },
    hasPending: () => pending !== null,
    requests: () => requests,
  };
}

function loop(over: { fps?: number; active?: () => boolean } = {}) {
  const f = fakeFrames();
  const fps = over.fps ?? 60;
  const frames = createFrameLoop(f.source, { maxFps: () => fps, active: over.active ?? (() => true) });
  return { f, frames };
}

describe("frame loop", () => {
  test("a request runs once, at the next frame", () => {
    const { f, frames } = loop();
    const seen: number[] = [];
    frames.request((t) => seen.push(t));
    expect(seen).toEqual([]);
    f.frame(1000);
    f.frame(1016.7);
    expect(seen).toEqual([1000]);
  });

  test("repeated requests before a frame run the callback once", () => {
    const { f, frames } = loop();
    let calls = 0;
    const flush = () => calls++;
    frames.request(flush);
    frames.request(flush);
    frames.request(flush);
    f.frame(0);
    expect(calls).toBe(1);
  });

  test("a subscriber runs on every frame until it is unsubscribed", () => {
    const { f, frames } = loop();
    const seen: number[] = [];
    const off = frames.subscribe((t) => seen.push(t));
    f.frame(0);
    f.frame(16.7);
    off();
    f.frame(33.4);
    expect(seen).toEqual([0, 16.7]);
    expect(f.hasPending()).toBe(false);
  });

  test("at 60 fps every 60 Hz frame runs, despite rAF jitter", () => {
    const { f, frames } = loop({ fps: 60 });
    const seen: number[] = [];
    frames.subscribe((t) => seen.push(t));
    for (let k = 0; k < 60; k++) f.frame(k * 16.67 + (k % 2 ? 0.8 : -0.8));
    expect(seen.length).toBe(60);
  });

  test("at 30 fps a 60 Hz display runs every second frame", () => {
    const { f, frames } = loop({ fps: 30 });
    const seen: number[] = [];
    frames.subscribe((t) => seen.push(t));
    for (let k = 0; k < 60; k++) f.frame(k * 16.67);
    expect(seen.length).toBe(30);
    for (let i = 1; i < seen.length; i++) expect(seen[i] - seen[i - 1]).toBeGreaterThanOrEqual(32);
  });

  test("while inactive nothing runs and no frame is requested", () => {
    let active = false;
    const { f, frames } = loop({ active: () => active });
    let calls = 0;
    frames.subscribe(() => calls++);
    f.frame(0);
    expect(calls).toBe(0);
    expect(f.hasPending()).toBe(false);
  });

  test("kick after wake resumes subscribers with no new request or subscription", () => {
    let active = false;
    const { f, frames } = loop({ active: () => active });
    const seen: number[] = [];
    frames.subscribe((t) => seen.push(t));
    f.frame(0);
    active = true;
    frames.kick();
    f.frame(100);
    expect(seen).toEqual([100]);
  });

  test("a throwing callback does not stop the loop: the next frame still runs", () => {
    const { f, frames } = loop();
    const seen: number[] = [];
    frames.subscribe((t) => {
      if (t === 0) throw new Error("boom");
      seen.push(t);
    });
    expect(() => f.frame(0)).toThrow("boom");
    f.frame(16.7);
    expect(seen).toEqual([16.7]);
  });

  test("the loop is idle with no subscribers and no requests", () => {
    const { f, frames } = loop();
    const off = frames.subscribe(() => {});
    off();
    f.frame(0);
    expect(f.hasPending()).toBe(false);
  });
});
