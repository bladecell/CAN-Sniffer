import { describe, expect, test } from "bun:test";
import { dispatchToListeners } from "./dispatch";
import { reconnectDelay } from "./backoff";
import { createStreamRefCount } from "./streamRefCount";
import { createThrottle } from "./throttle";
import { createLatest } from "./latest";
import { getModeLabel } from "./modeLabel";
import { normalizePidDefinition } from "./pidDefinition";

test("a throwing listener does not stop the others", () => {
  const seen: number[] = [];
  const set = new Set<(n: number) => void>([() => { throw new Error("boom"); }, (n) => seen.push(n)]);
  dispatchToListeners(set, 7);
  expect(seen).toEqual([7]);
});

test("reconnect backoff grows and is capped", () => {
  expect(reconnectDelay(0)).toBe(2000);
  expect(reconnectDelay(1)).toBe(4000);
  expect(reconnectDelay(50)).toBe(30000);
  expect(reconnectDelay(-1)).toBe(2000);
  expect(reconnectDelay(NaN)).toBe(2000);
});

describe("stream ref count", () => {
  test("starts on 0->1 and stops on 1->0 only", () => {
    const calls: string[] = [];
    const rc = createStreamRefCount(() => calls.push("start"), () => calls.push("stop"));
    const a = rc.acquire();
    const b = rc.acquire();
    expect(calls).toEqual(["start"]);
    a();
    expect(calls).toEqual(["start"]);
    b();
    expect(calls).toEqual(["start", "stop"]);
  });
  test("release is idempotent and cannot go negative", () => {
    const calls: string[] = [];
    const rc = createStreamRefCount(() => calls.push("start"), () => calls.push("stop"));
    const a = rc.acquire();
    a(); a();
    expect(rc.count()).toBe(0);
    rc.acquire();
    expect(calls).toEqual(["start", "stop", "start"]);
  });
});

test("throttle lets one through per interval", () => {
  let t = 1000;
  const ok = createThrottle(60000, () => t);
  expect([ok(), ok()]).toEqual([true, false]);
  t += 59999;
  expect(ok()).toBe(false);
  t += 1;
  expect(ok()).toBe(true);
});

test("latest: only the newest call is current", () => {
  const latest = createLatest();
  const a = latest.begin();
  const b = latest.begin();
  expect(a()).toBe(false);
  expect(b()).toBe(true);
});

test("getModeLabel tolerates undefined/NaN", () => {
  expect(getModeLabel(undefined as unknown as number)).toBe("Unknown");
  expect(getModeLabel(NaN)).toBe("Unknown");
  expect(getModeLabel(1)).toBe("Current Data");
  expect(getModeLabel(0x99)).toBe("0x99");
});

test("normalize replaces an invalid colour with the default", () => {
  const base = { id: 1, mode: 1, pid: 2, length: 2, name: "n", formula: "A", update_interval_ms: 256 };
  expect(normalizePidDefinition({ ...base, color: "red; x" }).color).toBe(0x01aaff);
  expect(normalizePidDefinition({ ...base, color: -5 }).color).toBe(0x01aaff);
  expect(normalizePidDefinition({ ...base, color: 255 }).color).toBe(255);
});
