// Pure rate limiter for chart redraws. Marks are collected per key; flush runs at most once per
// intervalMs, with the keys marked since the last flush. The first mark after a quiet period flushes
// right away, and marks inside the interval wait for its end (trailing edge), so nothing is lost.

/** Redraw rate for chart consumers: 5 Hz. */
export const CHART_REDRAW_MS = 200;

export interface PacerClock {
  now(): number;
  setTimer(fn: () => void, ms: number): unknown;
  clearTimer(handle: unknown): void;
}

export const realClock: PacerClock = {
  now: () => Date.now(),
  setTimer: (fn, ms) => setTimeout(fn, ms),
  clearTimer: (handle) => clearTimeout(handle as ReturnType<typeof setTimeout>),
};

export function createPacer<K>(
  intervalMs: number,
  flush: (keys: K[]) => void,
  clock: PacerClock = realClock,
) {
  const dirty = new Set<K>();
  let handle: unknown = null;
  let lastFlushAt = -Infinity;

  function run() {
    handle = null;
    lastFlushAt = clock.now();
    if (dirty.size === 0) return;
    const keys = [...dirty];
    dirty.clear();
    flush(keys);
  }

  return {
    mark(key: K) {
      dirty.add(key);
      if (handle !== null) return;
      const wait = Math.max(0, lastFlushAt + intervalMs - clock.now());
      handle = clock.setTimer(run, wait);
    },
  };
}
