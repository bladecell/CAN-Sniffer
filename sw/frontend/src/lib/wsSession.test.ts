import { describe, expect, test } from "bun:test";
import { closeOutcome, newSessionId, sessionId, wsUrl, WS_CLOSE_IN_USE, WS_CLOSE_REPLACED } from "./wsSession";

describe("closeOutcome", () => {
  test("only the two session codes stop the reconnect", () => {
    expect(closeOutcome(WS_CLOSE_IN_USE)).toBe("inUse");
    expect(closeOutcome(WS_CLOSE_REPLACED)).toBe("replaced");
    for (const code of [1000, 1001, 1005, 1006, 3999, 4002, 0, Number.NaN]) expect(closeOutcome(code)).toBe("drop");
  });
});

describe("session id", () => {
  test("8 hex digits, never zero", () => {
    expect(newSessionId(() => 0x1a2b3c4d)).toBe("1a2b3c4d");
    expect(newSessionId(() => 0xf)).toBe("0000000f");
    expect(newSessionId(() => 0)).toBe("00000001");
    expect(newSessionId(() => -1)).toBe("ffffffff");
    expect(newSessionId()).toMatch(/^[0-9a-f]{8}$/);
  });
  test("a reload keeps the id, bad or missing storage makes a new one", () => {
    const store = new Map<string, string>();
    const storage = { getItem: (k: string) => store.get(k) ?? null, setItem: (k: string, v: string) => void store.set(k, v) };
    const first = sessionId(storage, () => "0000abcd");
    expect(first).toBe("0000abcd");
    expect(sessionId(storage, () => "11111111")).toBe("0000abcd");
    for (const bad of ["", "0", "00000000", "xyz", "123456789", "ABCDEF12"]) {
      store.set("ws_sid", bad);
      expect(sessionId(storage, () => "22222222")).toBe("22222222");
    }
    expect(sessionId(null, () => "33333333")).toBe("33333333");
    const blocked = { getItem: () => { throw new Error("blocked"); }, setItem: () => { throw new Error("blocked"); } };
    expect(sessionId(blocked, () => "44444444")).toBe("44444444");
  });
});

describe("wsUrl", () => {
  test("carries the session id, and takeover only when asked", () => {
    expect(wsUrl({ protocol: "http:", host: "192.168.4.1" }, "0000abcd", false)).toBe("ws://192.168.4.1/ws?sid=0000abcd");
    expect(wsUrl({ protocol: "https:", host: "can-sniffer.local" }, "0000abcd", true)).toBe("wss://can-sniffer.local/ws?sid=0000abcd&takeover=1");
  });
});
