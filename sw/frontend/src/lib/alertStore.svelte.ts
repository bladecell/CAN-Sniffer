import type { Alert } from "./types";
import { MAX_TOASTS, TOAST_MS, evictOldest, remainingMs } from "./toast";

interface Timer {
  handle: ReturnType<typeof setTimeout> | null;
  leftMs: number;
  startedAt: number;
}

let alerts = $state<Alert[]>([]);
const timers = new Map<number, Timer>();

function start(id: number) {
  const t = timers.get(id);
  if (!t || t.handle !== null) return;
  t.startedAt = Date.now();
  t.handle = setTimeout(() => alertStore.remove(id), t.leftMs);
}

function stop(id: number) {
  const t = timers.get(id);
  if (!t || t.handle === null) return;
  clearTimeout(t.handle);
  t.handle = null;
  t.leftMs = remainingMs(t.leftMs, t.startedAt, Date.now());
}

export const alertStore = {
  get alerts() {
    return alerts;
  },

  add(message: string, type: Alert["type"] = "info", duration = TOAST_MS) {
    // Date.now() instead of crypto.randomUUID(): the UI is served over plain HTTP, where randomUUID is unavailable.
    const id = Date.now() + Math.random();
    for (const old of evictOldest(alerts.map((a) => a.id), MAX_TOASTS)) alertStore.remove(old);
    alerts.push({ id, message, type });
    timers.set(id, { handle: null, leftMs: duration, startedAt: 0 });
    start(id);
  },

  remove(id: number) {
    stop(id);
    timers.delete(id);
    alerts = alerts.filter((a) => a.id !== id);
  },

  pause(id: number) {
    stop(id);
  },

  resume(id: number) {
    start(id);
  },
};
