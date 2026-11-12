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

  test("clamps the wait to the cap and stops after the attempts", async () => {
    const waits: number[] = [];
    let calls = 0;
    const res = await retryBusy(async () => (calls++, busy("120")), { attempts: 3, sleep: async (ms) => void waits.push(ms) });
    expect(res.status).toBe(503);
    expect(calls).toBe(3);
    expect(waits).toEqual([RETRY_CAP_MS, RETRY_CAP_MS]);
  });

  test("other statuses are not retried", async () => {
    let calls = 0;
    await retryBusy(async () => (calls++, new Response("{}", { status: 500 })), { sleep: async () => {} });
    expect(calls).toBe(1);
  });

  test("a missing or bad header waits one second", () => {
    expect(retryWaitMs(null)).toBe(1000);
    expect(retryWaitMs("abc")).toBe(1000);
    expect(retryWaitMs("0")).toBe(1000);
  });
});
