import { describe, expect, test } from "bun:test";
import { API_TIMEOUT_MS, SD_TIMEOUT_MS, timedFetch, timeoutFor } from "./fetchTimeout";
import { singleFlight } from "./singleFlight";

describe("timeoutFor", () => {
  test("longer for SD, ECU and trip routes, none for transfers", () => {
    expect(timeoutFor("/api/v1/system")).toBe(API_TIMEOUT_MS);
    expect(timeoutFor("/api/v1/pid_def")).toBe(API_TIMEOUT_MS);
    expect(timeoutFor("/api/v1/sd_card/info")).toBe(SD_TIMEOUT_MS);
    expect(timeoutFor("/api/v1/recorder/trips")).toBe(SD_TIMEOUT_MS);
    expect(timeoutFor("/api/v1/dtc/descriptions?codes=P0100")).toBe(SD_TIMEOUT_MS);
    expect(timeoutFor("/api/v1/req/dtc")).toBe(SD_TIMEOUT_MS);
    expect(timeoutFor("/api/v1/sd_card/file/logs/a.csv?download=true")).toBeNull();
    expect(timeoutFor("/api/v1/sd_card/format")).toBeNull();
  });
});

describe("timedFetch", () => {
  const hung = (_url: string, init?: RequestInit) =>
    new Promise<Response>((_, reject) => init?.signal?.addEventListener("abort", () => reject(new Error("aborted"))));

  test("a timed-out fetch releases the single-flight slot and the next call retries", async () => {
    let calls = 0;
    const timer = new AbortController();
    const load = singleFlight(() => (calls++, timedFetch(calls === 1 ? hung : async () => new Response("ok"), "/api/v1/system", undefined, () => timer.signal)));
    const first = load();
    timer.abort();
    await expect(first).rejects.toThrow("aborted");
    expect(await (await load()).text()).toBe("ok");
    expect(calls).toBe(2);
  });

  test("passes the timeout in ms and keeps a caller signal", async () => {
    const seen: number[] = [];
    const sig = new AbortController().signal;
    let got: AbortSignal | null | undefined;
    await timedFetch(async (_u, i) => ((got = i?.signal), new Response()), "/api/v1/sd_card/info", undefined, (ms) => (seen.push(ms), sig));
    expect(seen).toEqual([SD_TIMEOUT_MS]);
    const own = new AbortController().signal;
    await timedFetch(async (_u, i) => ((got = i?.signal), new Response()), "/api/v1/system", { signal: own });
    expect(got).toBe(own);
  });
});
