import { describe, expect, test } from "bun:test";
import { createChartHistory } from "./chartHistory";
import type { PidDefinition } from "./types";

function def(pid: number, interval = 500): PidDefinition {
  return { pid, update_interval_ms: interval } as PidDefinition;
}

function harness(initial: PidDefinition[]) {
  let defs = initial;
  let onMessage: ((u: { pid: number; value: number; timestamp: number }) => void) | null = null;
  const history = createChartHistory({
    subscribeMessages: (cb) => {
      onMessage = cb;
      return () => {
        onMessage = null;
      };
    },
    definitions: () => defs,
  });
  return {
    history,
    send: (pid: number, value: number, timestamp: number) => onMessage?.({ pid, value, timestamp }),
    setDefinitions: (next: PidDefinition[]) => {
      defs = next;
    },
  };
}

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));

describe("chart history", () => {
  test("samples for a PID without a definition are dropped", async () => {
    const h = harness([]);
    const seen: number[] = [];
    h.history.subscribe(9, (b) => seen.push(b.size));
    h.send(9, 1, 100);
    await wait(250);
    expect(seen).toEqual([0]);
  });

  test("a PID with a definition reaches its listener, and a later definition load enables it", async () => {
    const h = harness([]);
    const sizes: number[] = [];
    h.history.subscribe(5, (b) => sizes.push(b.size));
    h.send(5, 1, 100);
    await wait(250);
    expect(sizes).toEqual([0]);

    h.setDefinitions([def(5)]);
    h.send(5, 2, 200);
    await wait(250);
    expect(sizes).toEqual([0, 1]);
  });

  test("subscribe calls the listener at once, and unsubscribe stops further calls", async () => {
    const h = harness([def(4)]);
    let calls = 0;
    const off = h.history.subscribe(4, () => calls++);
    expect(calls).toBe(1);
    h.send(4, 1, 100);
    await wait(250);
    expect(calls).toBe(2);
    off();
    h.send(4, 2, 300);
    await wait(250);
    expect(calls).toBe(2);
  });
});
