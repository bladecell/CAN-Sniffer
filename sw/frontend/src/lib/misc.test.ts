import { describe, expect, test } from "bun:test";
import { dispatchToListeners } from "./dispatch";
import { reconnectDelay } from "./backoff";
import { createStreamRefCount } from "./streamRefCount";
import { createThrottle } from "./throttle";
import { createLatest } from "./latest";
import { parseStoredLayout } from "./layoutStorage";
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

describe("parseStoredLayout", () => {
  const good = { id: "a", cardType: "pid", pid: 12, displayMode: "card", w: 10, h: 7 };
  test("keeps valid items", () => {
    expect(parseStoredLayout(JSON.stringify([good]))).toEqual([good]);
  });
  test("null, junk and wrong shapes give an empty layout", () => {
    expect(parseStoredLayout(null)).toEqual([]);
    expect(parseStoredLayout("{")).toEqual([]);
    expect(parseStoredLayout("{}")).toEqual([]);
    expect(parseStoredLayout("123")).toEqual([]);
  });
  test("drops invalid items but keeps the rest", () => {
    const bad = [null, 5, { id: "b" }, { ...good, id: "c", w: "x" }, { ...good, id: "d", cardType: "evil" }, { ...good, id: "e", w: -1 }, { id: "f", cardType: "overview", displayMode: "default", w: 4, h: 4, pids: [1], color: "#fff" }];
    expect(parseStoredLayout(JSON.stringify([...bad, good]))).toEqual([good]);
  });
  test("drops duplicate ids", () => {
    expect(parseStoredLayout(JSON.stringify([good, good]))).toEqual([good]);
  });
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

test("layout colours are sanitised", () => {
  const ov = { id: "o", cardType: "overview", displayMode: "default", w: 4, h: 4, pids: [1, 2, 3], color: "red;background:url(x)" };
  expect((parseStoredLayout(JSON.stringify([ov]))[0] as any).color).toBe("#3b82f6");
  expect((parseStoredLayout(JSON.stringify([{ ...ov, color: "#112233" }]))[0] as any).color).toBe("#112233");
});
