import { describe, expect, test } from "bun:test";
import { idle, press, settle, tick, cancel, type ClearState } from "./clearFlow";

const confirm = { kind: "confirm", token: "0123456789abcdef", expiresMs: 10000 } as const;

function armed(now = 1000): ClearState {
  const p = press(idle, now);
  return settle(p.state, confirm, now).state;
}

describe("clear flow", () => {
  test("press 1 sends a token-less request", () => {
    const p = press(idle, 0);
    expect(p.send).toEqual({});
    expect(p.state.kind).toBe("sending");
  });
  test("a 409 arms with the token and expiry", () => {
    expect(armed(1000)).toEqual({ kind: "armed", token: "0123456789abcdef", expiresAt: 11000, armedAt: 1000 });
  });
  test("press 2 sends exactly that token", () => {
    expect(press(armed(), 2000).send).toEqual({ token: "0123456789abcdef" });
  });
  test("a press within 500 ms of arming sends nothing and stays armed", () => {
    const s = armed(1000);
    const p = press(s, 1050);
    expect(p.send).toBeNull();
    expect(p.state).toEqual(s);
    expect(press(s, 1499).send).toBeNull();
    expect(press(s, 1600).send).toEqual({ token: "0123456789abcdef" });
  });
  test("the expiry window starts when the request was sent", () => {
    const sent = settle(press(idle, 0).state, confirm, 3000, 0).state;
    expect(sent).toEqual({ kind: "armed", token: "0123456789abcdef", expiresAt: 10000, armedAt: 3000 });
  });
  test("press while sending is ignored", () => {
    const p = press(idle, 0);
    expect(press(p.state, 1).send).toBeNull();
  });
  test("an expired armed state never sends the token; it reverts and a press starts over", () => {
    const s = armed(1000);
    expect(tick(s, 11000)).toEqual(idle);
    expect(tick(s, 10999)).toEqual(s);
    expect(press(s, 11000).send).toEqual({});
  });
  test("a second 409 reverts to idle with an expired notice and no new arming", () => {
    const p2 = press(armed(), 2000);
    const r = settle(p2.state, confirm, 2100);
    expect(r.state).toEqual(idle);
    expect(r.notice).toBe("expired");
  });
  test("success and error revert to idle", () => {
    const p2 = press(armed(), 2000);
    expect(settle(p2.state, { kind: "done" }, 2100)).toEqual({ state: idle, notice: null });
    expect(settle(p2.state, { kind: "error" }, 2100)).toEqual({ state: idle, notice: null });
    expect(settle(press(idle, 0).state, { kind: "error" }, 1).state).toEqual(idle);
    expect(settle(press(idle, 0).state, { kind: "done" }, 1).state).toEqual(idle);
  });
  test("cancel disarms and sends nothing; it cannot interrupt a send", () => {
    expect(cancel(armed())).toEqual(idle);
    const sending = press(armed(), 2000).state;
    expect(cancel(sending)).toEqual(sending);
  });
});
