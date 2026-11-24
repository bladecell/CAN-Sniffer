// Reads the stored samples of a set of PIDs. Listeners arrive once per frame that has new samples (chartHistory); `version` bumps with them.
import { getContext, untrack } from "svelte";
import type { ChartBuffer } from "$lib/chartBuffer";
import { chartHistoryStore } from "$lib/chartHistoryStore";

export interface Samples {
  t: number[];
  v: (number | null)[];
}

/** Where a plot reads its samples. The page default is the live history; the trip viewer provides recorded buffers in context. */
export interface SeriesSource {
  subscribe(pid: number, listener: (buffer: ChartBuffer) => void): () => void;
  /** Live sources move the present clock with each sample; a recorded one does not (the player owns the clock). */
  live: boolean;
}

export const TELEMETRY_SOURCE = "telemetry-source";
const liveSource: SeriesSource = { subscribe: (pid, listener) => chartHistoryStore.subscribe(pid, listener), live: true };

const NONE: Samples = { t: [], v: [] };

/**
 * Subscribes to the PIDs from getPids while the owning component lives. Call from component setup.
 * onSample receives the newest sample time of each update, for the present clock.
 */
export function createSeriesFeed(getPids: () => number[], onSample: (deviceMs: number) => void) {
  const source = getContext<SeriesSource | undefined>(TELEMETRY_SOURCE) ?? liveSource;
  let buffers = new Map<number, ChartBuffer>();
  let version = $state(0);

  $effect(() => {
    const pids = [...new Set(getPids())];
    const next = new Map<number, ChartBuffer>();
    buffers = next;
    const stops = pids.map((pid) =>
      source.subscribe(pid, (buffer) => {
        next.set(pid, buffer);
        const last = buffer.lastTime();
        if (source.live && last !== undefined) onSample(last);
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
    /** Buffered sample count of a PID, without copying the samples. */
    size(pid: number): number {
      return buffers.get(pid)?.size ?? 0;
    },
    /** Samples at or after fromT (device ms), oldest first, plus the one before it when there is one. */
    read(pid: number, fromT: number): Samples {
      const buffer = buffers.get(pid);
      return buffer ? buffer.snapshot(fromT, 1) : NONE;
    },
  };
}
