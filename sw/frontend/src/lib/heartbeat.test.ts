import { describe, expect, test } from "bun:test";
import { createHeartbeat, PING_INTERVAL_MS, PONG_TIMEOUT_MS, type HeartbeatTimers } from "./heartbeat";

/** Minimal fake clock: runs due timers in time order when advanced. */
function fakeTimers() {
  let now = 0;
  let nextId = 1;
  const timers = new Map<number, { at: number; fn: () => void; every: number | null }>();
  const timerApi: HeartbeatTimers = {
    setInterval: (fn, ms) => {
      const id = nextId++;
      timers.set(id, { at: now + ms, fn, every: ms });
      return id;
    },
    clearInterval: (h) => void timers.delete(h as number),
    setTimeout: (fn, ms) => {
      const id = nextId++;
      timers.set(id, { at: now + ms, fn, every: null });
      return id;
    },
    clearTimeout: (h) => void timers.delete(h as number),
  };
  return {
    timerApi,
    advance(ms: number) {
      const end = now + ms;
      for (;;) {
        const due = [...timers.entries()].filter(([, t]) => t.at <= end).sort((a, b) => a[1].at - b[1].at)[0];
        if (!due) break;
        const [id, t] = due;
        now = t.at;
        if (t.every === null) timers.delete(id);
        else t.at += t.every;
        t.fn();
      }
      now = end;
    },
  };
}

function rig(pingOpen = true) {
  const clock = fakeTimers();
  const log = { pings: 0, timeouts: 0, pongs: 0 };
  const hb = createHeartbeat({
    ping: () => {
      log.pings++;
      return pingOpen;
    },
    onTimeout: () => log.timeouts++,
    onPong: () => log.pongs++,
    timers: clock.timerApi,
  });
  return { hb, clock, log };
}

describe("heartbeat", () => {
  test("pings on the interval", () => {
    const { hb, clock, log } = rig();
    hb.start();
    clock.advance(PING_INTERVAL_MS);
    expect(log.pings).toBe(1);
  });

  test("a pong 5 s after the ping does not drop the link", () => {
    const { hb, clock, log } = rig();
    hb.start();
    clock.advance(PING_INTERVAL_MS);
    clock.advance(5000);
    hb.pong();
    clock.advance(PONG_TIMEOUT_MS);
    expect(log.timeouts).toBe(0);
    expect(log.pongs).toBe(1);
  });

  test("no pong times out exactly PONG_TIMEOUT_MS after the ping", () => {
    const { hb, clock, log } = rig();
    hb.start();
    clock.advance(PING_INTERVAL_MS);
    clock.advance(PONG_TIMEOUT_MS - 1);
    expect(log.timeouts).toBe(0);
    clock.advance(1);
    expect(log.timeouts).toBe(1);
  });

  test("no second ping is sent while a pong is outstanding", () => {
    const { hb, clock, log } = rig();
    hb.start();
    clock.advance(PING_INTERVAL_MS);
    clock.advance(PING_INTERVAL_MS);
    expect(log.pings).toBe(1);
  });

  test("a closed socket does not arm a timeout", () => {
    const { hb, clock, log } = rig(false);
    hb.start();
    clock.advance(PING_INTERVAL_MS + PONG_TIMEOUT_MS);
    expect(log.timeouts).toBe(0);
  });

  test("stop cancels pending pings and the timeout", () => {
    const { hb, clock, log } = rig();
    hb.start();
    clock.advance(PING_INTERVAL_MS);
    hb.stop();
    clock.advance(PING_INTERVAL_MS + PONG_TIMEOUT_MS);
    expect(log.pings).toBe(1);
    expect(log.timeouts).toBe(0);
  });
});
