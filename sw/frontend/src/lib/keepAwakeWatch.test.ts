import { describe, expect, test } from "bun:test";
import { KEEP_AWAKE_MS, createKeepAwakeWatch } from "./keepAwakeWatch";

function setup(read: () => Promise<boolean> = async () => false) {
  const timers: { fn: () => void; ms: number; live: boolean }[] = [];
  let notified = 0;
  const watch = createKeepAwakeWatch({
    read,
    notify: () => notified++,
    setTimer: (fn, ms) => timers.push({ fn, ms, live: true }) - 1,
    clearTimer: (h) => (timers[h as number].live = false),
  });
  const fire = async () => {
    const t = timers.filter((x) => x.live).at(-1);
    t!.live = false;
    t!.fn();
    await Promise.resolve();
    await Promise.resolve();
  };
  return { watch, timers, fire, notified: () => notified };
}

describe("keep awake watch", () => {
  test("alerts when the 30 minute timer runs out and the device dropped the hold", async () => {
    const s = setup();
    s.watch.arm();
    expect(s.timers[0].ms).toBeGreaterThan(KEEP_AWAKE_MS);
    await s.fire();
    expect(s.notified()).toBe(1);
    expect(s.watch.armed).toBe(false);
  });

  test("a hold renewed elsewhere keeps watching", async () => {
    const s = setup(async () => true);
    s.watch.arm();
    await s.fire();
    expect(s.notified()).toBe(0);
    expect(s.watch.armed).toBe(true);
  });

  test("an unreachable device counts as ended", async () => {
    const s = setup(async () => Promise.reject(new Error("offline")));
    s.watch.arm();
    await s.fire();
    expect(s.notified()).toBe(1);
  });

  test("turning it off yourself gives no alert", () => {
    const s = setup();
    s.watch.arm();
    s.watch.disarm();
    s.watch.seen(false);
    expect(s.notified()).toBe(0);
    expect(s.timers.every((t) => !t.live)).toBe(true);
  });

  test("a status read that drops the hold alerts once, and one that finds it starts watching", () => {
    const s = setup();
    s.watch.seen(true);
    expect(s.watch.armed).toBe(true);
    s.watch.seen(true);
    expect(s.timers).toHaveLength(1);
    s.watch.seen(false);
    s.watch.seen(false);
    expect(s.notified()).toBe(1);
  });
});
