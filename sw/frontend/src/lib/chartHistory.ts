import type { PidDefinition } from "./types";
import { createChartBuffer, type ChartBuffer, type ChartBufferWriter } from "./chartBuffer";
import { createPacer, realClock, CHART_REDRAW_MS, type PacerClock } from "./chartPacer";
import { dispatchToListeners } from "./dispatch";

export type ChartListener = (buffer: ChartBuffer) => void;

export interface ChartHistorySource {
  /** Subscribes to live samples; returns the unsubscribe function. */
  subscribeMessages(cb: (u: { pid: number; value: number; timestamp: number }) => void): () => void;
  definitions(): readonly PidDefinition[];
}

/**
 * Shared chart history: one bounded ring buffer per PID with a definition.
 * Ingest is O(1) per message and does not notify listeners directly. Listeners are called
 * at most 5 Hz (CHART_REDRAW_MS) and read the buffer themselves.
 * Samples for PIDs without a definition are dropped, which bounds the number of buffers.
 */
export function createChartHistory(source: ChartHistorySource, clock: PacerClock = realClock) {
  const buffers = new Map<number, ChartBufferWriter>();
  const listeners = new Map<number, Set<ChartListener>>();
  let defsRef: readonly PidDefinition[] | null = null;
  let defByPid = new Map<number, PidDefinition>();

  // The definitions array is replaced as a whole on load, so a reference check keeps the Map current.
  function definition(pid: number): PidDefinition | undefined {
    const defs = source.definitions();
    if (defs !== defsRef) {
      defsRef = defs;
      const next = new Map(defs.map((d) => [d.pid, d]));
      for (const [oldPid, oldDef] of defByPid) {
        if (next.get(oldPid) !== oldDef) buffers.delete(oldPid);
      }
      for (const oldPid of buffers.keys()) {
        if (!next.has(oldPid)) buffers.delete(oldPid);
      }
      defByPid = next;
    }
    return defByPid.get(pid);
  }

  function bufferFor(pid: number): ChartBufferWriter {
    const buffer = buffers.get(pid) ?? createChartBuffer();
    buffers.set(pid, buffer);
    return buffer;
  }

  const pacer = createPacer<number>(
    CHART_REDRAW_MS,
    (pids) => {
      for (const pid of pids) {
        const buffer = buffers.get(pid);
        if (buffer) dispatchToListeners(listeners.get(pid) ?? [], buffer);
      }
    },
    clock,
  );

  source.subscribeMessages((update) => {
    const def = definition(update.pid);
    if (!def) return;
    const splitAfterMs = def.update_interval_ms ? Math.max(2000, def.update_interval_ms * 1.5) : 2000;
    if (bufferFor(update.pid).push(update.timestamp, update.value, splitAfterMs)) pacer.mark(update.pid);
  });

  return {
    /** Calls the listener now with the current buffer, then at most 5 Hz while samples arrive. */
    subscribe(pid: number, listener: ChartListener): () => void {
      const set = listeners.get(pid) ?? new Set<ChartListener>();
      listeners.set(pid, set);
      set.add(listener);
      listener(bufferFor(pid));
      return () => {
        set.delete(listener);
      };
    },
  };
}
