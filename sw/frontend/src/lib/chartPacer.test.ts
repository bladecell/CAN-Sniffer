import { describe, expect, test } from "bun:test";
import { createPacer, type PacerClock } from "./chartPacer";

// Manual clock: timers fire only when advance() moves time past their deadline.
function fakeClock() {
  let now = 0;
  let id = 0;
  let timers: { at: number; fn: () => void; id: number }[] = [];
  const clock: PacerClock = {
    now: () => now,
    setTimer(fn, ms) {
      const handle = { at: now + ms, fn, id: ++id };
      timers.push(handle);
      return handle;
    },
    clearTimer(handle) {
      timers = timers.filter((t) => t.id !== (handle as { id: number }).id);
    },
  };
  return {
    clock,
    advance(ms: number) {
      const target = now + ms;
      for (;;) {
        const due = timers.filter((t) => t.at <= target).sort((a, b) => a.at - b.at)[0];
        if (!due) break;
        timers = timers.filter((t) => t.id !== due.id);
        now = due.at;
        due.fn();
      }
      now = target;
    },
  };
}

describe("chart pacer", () => {
  test("a first mark flushes right away with the marked key", () => {
    const { clock, advance } = fakeClock();
    const calls: number[][] = [];
    const pacer = createPacer<number>(200, (keys) => calls.push(keys), clock);
    pacer.mark(7);
    advance(0);
    expect(calls).toEqual([[7]]);
  });

  test("repeated marks of one key flush once with that key", () => {
    const { clock, advance } = fakeClock();
    const calls: number[][] = [];
    const pacer = createPacer<number>(200, (keys) => calls.push(keys), clock);
    pacer.mark(1);
    pacer.mark(1);
    pacer.mark(1);
    advance(0);
    expect(calls).toEqual([[1]]);
  });

  test("a burst of messages flushes at most 5 times per second", () => {
    const { clock, advance } = fakeClock();
    const times: number[] = [];
    const pacer = createPacer<number>(200, () => times.push(clock.now()), clock);
    for (let t = 0; t <= 1000; t += 10) {
      pacer.mark(t % 3);
      advance(10);
    }
    advance(1000);
    expect(times.length).toBeGreaterThanOrEqual(5);
    expect(times.length).toBeLessThanOrEqual(7);
    for (let i = 1; i < times.length; i++) {
      expect(times[i] - times[i - 1]).toBeGreaterThanOrEqual(200);
    }
  });

  test("marks inside the interval are delivered after it, not dropped", () => {
    const { clock, advance } = fakeClock();
    const calls: number[][] = [];
    const pacer = createPacer<number>(200, (keys) => calls.push(keys), clock);
    pacer.mark(1); // flushes at 0
    advance(0);
    advance(10);
    pacer.mark(2); // deferred to the 200 ms boundary
    advance(189);
    expect(calls).toEqual([[1]]);
    advance(1);
    expect(calls).toEqual([[1], [2]]);
  });

  test("keys marked during the interval are merged into one trailing flush", () => {
    const { clock, advance } = fakeClock();
    const calls: number[][] = [];
    const pacer = createPacer<number>(200, (keys) => calls.push(keys), clock);
    pacer.mark(1);
    advance(0);
    pacer.mark(2);
    pacer.mark(3);
    pacer.mark(2);
    advance(200);
    expect(calls).toEqual([[1], [2, 3]]);
  });

  test("a mark after a quiet period flushes at once, not at the old deadline", () => {
    const { clock, advance } = fakeClock();
    const times: [number, number[]][] = [];
    const pacer = createPacer<number>(200, (keys) => times.push([clock.now(), keys]), clock);
    pacer.mark(1);
    advance(0);
    pacer.mark(2);
    advance(200);
    advance(1000);
    pacer.mark(3);
    advance(0);
    expect(times).toEqual([
      [0, [1]],
      [200, [2]],
      [1200, [3]],
    ]);
  });
});
