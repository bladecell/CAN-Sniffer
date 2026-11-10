import { describe, expect, test } from "bun:test";
import { createPidRetry } from "./pidRetry";

function rig(results: boolean[], loaded = false) {
  const timers: Array<() => void> = [];
  let calls = 0;
  const retry = createPidRetry({
    load: async () => results[Math.min(calls++, results.length - 1)],
    isLoaded: () => loaded,
    setTimeout: (fn) => (timers.push(fn), timers.length),
    clearTimeout: () => timers.splice(0),
  });
  const tick = async () => {
    timers.shift()?.();
    await Promise.resolve();
    await Promise.resolve();
  };
  return { retry, tick, calls: () => calls, pending: () => timers.length };
}

describe("pidRetry", () => {
  test("8 cards mounted run one retry loop, not 8", async () => {
    const r = rig([false, false, true]);
    const releases = Array.from({ length: 8 }, () => r.retry.acquire());
    await Promise.resolve();
    expect(r.calls()).toBe(1);
    expect(r.pending()).toBe(1);
    await r.tick();
    expect(r.calls()).toBe(2);
    await r.tick();
    expect(r.calls()).toBe(3);
    expect(r.pending()).toBe(0);
    releases.forEach((f) => f());
  });

  test("the loop stops when the last card unmounts", async () => {
    const r = rig([false]);
    const a = r.retry.acquire();
    const b = r.retry.acquire();
    await Promise.resolve();
    a();
    expect(r.pending()).toBe(1);
    b();
    expect(r.pending()).toBe(0);
  });

  test("nothing starts when the definitions are already loaded", async () => {
    const r = rig([false], true);
    r.retry.acquire();
    await Promise.resolve();
    expect(r.calls()).toBe(0);
  });
});
