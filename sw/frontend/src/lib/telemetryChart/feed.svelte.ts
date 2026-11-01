// Reads the stored samples of a set of PIDs. Listeners arrive at 5 Hz from chartHistory; `version` bumps with them.
import { untrack } from "svelte";
import type { ChartBuffer } from "$lib/chartBuffer";
import { chartHistoryStore } from "$lib/chartHistoryStore";

export interface Samples {
  t: number[];
  v: (number | null)[];
}

const NONE: Samples = { t: [], v: [] };

/**
 * Subscribes to the PIDs from getPids while the owning component lives. Call from component setup.
 * onSample receives the newest sample time of each update, for the present clock.
 */
export function createSeriesFeed(getPids: () => number[], onSample: (deviceMs: number) => void) {
  let buffers = new Map<number, ChartBuffer>();
  let version = $state(0);

  $effect(() => {
    const pids = [...new Set(getPids())];
    const next = new Map<number, ChartBuffer>();
    buffers = next;
    const stops = pids.map((pid) =>
      chartHistoryStore.subscribe(pid, (buffer) => {
        next.set(pid, buffer);
        const last = buffer.lastTime();
        if (last !== undefined) onSample(last);
        untrack(() => version++);
      }),
    );
    return () => stops.forEach((stop) => stop());
  });

  return {
    /** Read it inside a $derived to recompute on new samples. */
    get version() {
      return version;
    },
    /** Samples at or after fromT (device ms), oldest first, plus the one before it when there is one. */
    read(pid: number, fromT: number): Samples {
      const buffer = buffers.get(pid);
      return buffer ? buffer.snapshot(fromT, 1) : NONE;
    },
  };
}
