import { describe, expect, test } from "bun:test";
import { retryBusy, retryWaitMs, RETRY_CAP_MS } from "./retryBusy";

const busy = (retryAfter?: string) => new Response("{}", { status: 503, headers: retryAfter ? { "Retry-After": retryAfter } : {} });

describe("retryBusy", () => {
  test("waits for Retry-After, then returns the first good answer", async () => {
    const waits: number[] = [];
    const answers = [busy("2"), new Response("ok")];
    const res = await retryBusy(async () => answers.shift()!, { sleep: async (ms) => void waits.push(ms) });
    expect(await res.text()).toBe("ok");
    expect(waits).toEqual([2000]);
  });

  test("doubles the wait on each retry, capped, and stops after the attempts", async () => {
    const waits: number[] = [];
    let calls = 0;
    const res = await retryBusy(async () => (calls++, busy("1")), { attempts: 5, sleep: async (ms) => void waits.push(ms) });
    expect(res.status).toBe(503);
    expect(calls).toBe(5);
    expect(waits).toEqual([1000, 2000, 4000, 8000]);
  });

  test("clamps the wait to the cap", async () => {
    const waits: number[] = [];
    await retryBusy(async () => busy("120"), { attempts: 3, sleep: async (ms) => void waits.push(ms) });
    expect(waits).toEqual([RETRY_CAP_MS, RETRY_CAP_MS]);
  });

  test("other statuses are not retried", async () => {
    let calls = 0;
    await retryBusy(async () => (calls++, new Response("{}", { status: 500 })), { sleep: async () => {} });
    expect(calls).toBe(1);
  });

  test("a missing or bad header waits one second, then doubles", () => {
    expect(retryWaitMs(null, 1)).toBe(1000);
    expect(retryWaitMs("abc", 1)).toBe(1000);
    expect(retryWaitMs("0", 1)).toBe(1000);
    expect(retryWaitMs(null, 3)).toBe(4000);
  });

  test("Retry-After is the base of the exponential wait", () => {
    expect(retryWaitMs("5", 1)).toBe(5000);
    expect(retryWaitMs("5", 2)).toBe(RETRY_CAP_MS);
  });
});
