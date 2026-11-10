import { describe, expect, test } from "bun:test";
import { singleFlight } from "./singleFlight";

function deferred<T>() {
  let resolve!: (v: T) => void;
  let reject!: (e: unknown) => void;
  const promise = new Promise<T>((res, rej) => ((resolve = res), (reject = rej)));
  return { promise, resolve, reject };
}

describe("singleFlight", () => {
  test("20 callers while one is in flight share one request", async () => {
    let calls = 0;
    const d = deferred<number>();
    const load = singleFlight(() => (calls++, d.promise));
    const all = Array.from({ length: 20 }, () => load());
    d.resolve(7);
    expect(await Promise.all(all)).toEqual(Array(20).fill(7));
    expect(calls).toBe(1);
  });

  test("a call after the flight settled starts a new request", async () => {
    let calls = 0;
    const load = singleFlight(async () => ++calls);
    await load();
    await load();
    expect(calls).toBe(2);
  });

  test("a rejected flight is dropped so the next call retries", async () => {
    let calls = 0;
    const load = singleFlight(async () => {
      if (++calls === 1) throw new Error("busy");
      return calls;
    });
    await expect(load()).rejects.toThrow("busy");
    expect(await load()).toBe(2);
  });
});
