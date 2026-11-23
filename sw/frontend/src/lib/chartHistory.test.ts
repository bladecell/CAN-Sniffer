import { describe, expect, test } from "bun:test";
import { createChartHistory } from "./chartHistory";
import type { PidDefinition } from "./types";

function def(pid: number, interval = 500): PidDefinition {
  return { pid, update_interval_ms: interval } as PidDefinition;
}

// Frames run only when runFrame() is called, so each test decides when a redraw happens.
function harness(initial: PidDefinition[]) {
  let defs = initial;
  let onMessage: ((u: { pid: number; value: number; timestamp: number }) => void) | null = null;
  let pending: (() => void) | null = null;
  const history = createChartHistory(
    {
      subscribeMessages: (cb) => {
        onMessage = cb;
        return () => {
          onMessage = null;
        };
      },
      definitions: () => defs,
    },
    { request: (fn) => (pending = fn) },
  );
  return {
    history,
    send: (pid: number, value: number, timestamp: number) => onMessage?.({ pid, value, timestamp }),
    runFrame: () => {
      const fn = pending;
      pending = null;
      fn?.();
    },
    setDefinitions: (next: PidDefinition[]) => {
      defs = next;
    },
  };
}

describe("chart history", () => {
  test("samples for a PID without a definition are dropped", () => {
    const h = harness([]);
    const seen: number[] = [];
    h.history.subscribe(9, (b) => seen.push(b.size));
    h.send(9, 1, 100);
    h.runFrame();
    expect(seen).toEqual([0]);
  });

  test("a PID with a definition reaches its listener on the next frame, and a later definition load enables it", () => {
    const h = harness([]);
    const sizes: number[] = [];
    h.history.subscribe(5, (b) => sizes.push(b.size));
    h.send(5, 1, 100);
    h.runFrame();
    expect(sizes).toEqual([0]);

    h.setDefinitions([def(5)]);
    h.send(5, 2, 200);
    h.runFrame();
    expect(sizes).toEqual([0, 1]);
  });

  test("a burst of samples within one frame notifies the listener once", () => {
    const h = harness([def(3)]);
    const sizes: number[] = [];
    h.history.subscribe(3, (b) => sizes.push(b.size));
    sizes.length = 0;
    for (let t = 0; t < 50; t++) h.send(3, t, 100 + t * 10);
    h.runFrame();
    expect(sizes).toEqual([50]);
  });

  test("subscribe calls the listener at once, and unsubscribe stops further calls", () => {
    const h = harness([def(4)]);
    let calls = 0;
    const off = h.history.subscribe(4, () => calls++);
    expect(calls).toBe(1);
    h.send(4, 1, 100);
    h.runFrame();
    expect(calls).toBe(2);
    off();
    h.send(4, 2, 300);
    h.runFrame();
    expect(calls).toBe(2);
  });

  test("replacing a PID definition starts a fresh in-memory history", () => {
    const h = harness([def(6)]);
    const sizes: number[] = [];
    h.history.subscribe(6, (buffer) => sizes.push(buffer.size));
    h.send(6, 1, 100);
    h.runFrame();
    h.setDefinitions([{ ...def(6), name: "replacement" } as PidDefinition]);
    h.send(6, 2, 200);
    h.runFrame();
    expect(sizes).toEqual([0, 1, 1]);
  });
});
