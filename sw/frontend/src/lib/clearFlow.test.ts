import { describe, expect, test } from "bun:test";
import { ARM_WINDOW_MS, idle, press, settle, tick, cancel, type ClearState } from "./clearFlow";

function armed(now = 1000): ClearState {
  return press(idle, now).state;
}

describe("clear flow", () => {
  test("press 1 arms and sends nothing", () => {
    const p = press(idle, 1000);
    expect(p.send).toBe(false);
    expect(p.state).toEqual({ kind: "armed", armedAt: 1000, expiresAt: 1000 + ARM_WINDOW_MS });
  });
  test("press 2 sends once and moves to sending", () => {
    const p = press(armed(), 2000);
    expect(p.send).toBe(true);
    expect(p.state).toEqual({ kind: "sending" });
  });
  test("a press within 500 ms of arming does nothing and stays armed", () => {
    const s = armed(1000);
    expect(press(s, 1050)).toEqual({ state: s, send: false });
    expect(press(s, 1499).send).toBe(false);
    expect(press(s, 1600).send).toBe(true);
  });
  test("press while sending is ignored", () => {
    const sending = press(armed(), 2000).state;
    expect(press(sending, 2100)).toEqual({ state: sending, send: false });
  });
  test("an expired arm reverts and the next press arms again without sending", () => {
    const s = armed(1000);
    expect(tick(s, 1000 + ARM_WINDOW_MS)).toEqual(idle);
    expect(tick(s, 1000 + ARM_WINDOW_MS - 1)).toEqual(s);
    expect(press(s, 1000 + ARM_WINDOW_MS)).toEqual({
      state: { kind: "armed", armedAt: 11000, expiresAt: 11000 + ARM_WINDOW_MS },
      send: false,
    });
  });
  test("settle always returns to idle", () => {
    expect(settle()).toEqual(idle);
  });
  test("cancel disarms and cannot interrupt a send", () => {
    expect(cancel(armed())).toEqual(idle);
    const sending = press(armed(), 2000).state;
    expect(cancel(sending)).toEqual(sending);
  });
});
