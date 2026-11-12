import { describe, expect, test } from "bun:test";
import { createLimiter, rebooted } from "./openReload";

describe("createLimiter", () => {
  test("never more than the limit in flight across callers", async () => {
    const limit = createLimiter(2);
    let active = 0;
    let peak = 0;
    const job = () => limit(async () => {
      peak = Math.max(peak, ++active);
      await new Promise((r) => setTimeout(r, 1));
      active--;
    });
    await Promise.all(Array.from({ length: 9 }, job));
    expect(peak).toBe(2);
  });

  test("a rejection frees the slot", async () => {
    const limit = createLimiter(1);
    await expect(limit(async () => { throw new Error("x"); })).rejects.toThrow("x");
    expect(await limit(async () => 5)).toBe(5);
  });
});

describe("rebooted", () => {
  test("no sample means load", () => expect(rebooted(null, 100, 0)).toBe(true));
  test("uptime that kept counting is not a reboot", () => {
    expect(rebooted({ uptimeS: 100, at: 0 }, 160, 60_000)).toBe(false);
    expect(rebooted({ uptimeS: 100, at: 0 }, 155, 60_000)).toBe(false);
  });
  test("uptime below the prediction is a reboot", () => {
    expect(rebooted({ uptimeS: 5, at: 0 }, 3, 60_000)).toBe(true);
    expect(rebooted({ uptimeS: 3000, at: 0 }, 20, 60_000)).toBe(true);
  });
});
